#!/usr/bin/env python3

# Copyright 2026 Yibing Lu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""
MolmoPoint target localization with RealSense depth and optional SAM2 masks.

The filename is retained as a compatibility entry point for older launch files.
New code should run ``molmopoint_node.py``.

Parameters (declared):
- device: 'auto', 'cpu', or 'cuda:0'
- model_name_or_path: Hugging Face id or local MolmoPoint checkpoint
- quantization: '4bit' (recommended for 12 GB VRAM), '8bit', or 'none'
- publish_poses: bool
- image_topic: D405 color images (default: /cameras/d405/color/image_raw)
- depth_topic: D405 aligned depth (default: /cameras/d405/aligned_depth_to_color/image_raw)
- camera_info_topic: D405 color camera info (default: /cameras/d405/color/camera_info)
- camera_frame: TF frame used for back-projected camera points. If empty, use CameraInfo header.
- inference_rate: maximum Hz for running inference (default 0.5 in config)
- task_id: 1-6 to select targets for different parts of the pipeline
- world_frame: TF parent frame for published poses (default world); use odom,
  world, base_link, etc. as in your TF tree
- tf_lookup_timeout_sec: max wait for each TF lookup (default 0.5)
- remote_inference_url: if non-empty (e.g. http://192.168.64.1:8765), skip local
  model loading and call the remote /point and /segment endpoints instead
- remote_timeout_sec: HTTP timeout for each remote call (default 30)
- debug_dump_dir: if non-empty, save each inferred cycle's RGB-D/K/TF inputs as
  .npz for offline replay with tests/detector_replay.py

Outputs:
- /molmopoint/detections (std_msgs/String) JSON list of pointed targets
- /molmopoint/debug_image (sensor_msgs/Image) annotated image
- /molmopoint_pose_world (geometry_msgs/PoseStamped) 3D target in world_frame
- /molmopoint/point (std_msgs/String) point plus optional camera/world 3D data
- /molmopoint/diagnostics (std_msgs/String) per-cycle JSON: stage reached and
  the reason the cycle ended, with timing/mask/depth/TF counters
"""

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge
import numpy as np
import cv2
import base64
import json
import os
import math
import sys
import threading
import time
import urllib.error
import urllib.request
import yaml
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple
from PIL import Image as PILImage


from collections import deque
from copy import deepcopy
from homeplus_vision.target_geometry import offset_surface_goal, fit_button_plane

class MolmoPointNode(Node):
    # Defaults for state used by diagnostics, so partially constructed nodes
    # (offline tests use __new__) keep working.
    remote_inference_url = ''
    remote_timeout_sec = 30.0
    debug_dump_dir = ''
    diagnostics_pub = None
    _future_frames_dropped = 0
    _motion_status_msgs = 0
    _motion_still_seen = False
    _diag_last_key = None
    _diag_last_log = None
    _last_inference_info = None
    _last_segment_info = None

    def __init__(self):
        os.environ["HYDRA_FULL_ERROR"] = "1"
        super().__init__('molmopoint')

        workspace_root = Path(__file__).resolve().parents[2]
        default_grounded_sam_root = workspace_root / "Grounded-SAM-2"
        default_task_registry = (
            Path(get_package_share_directory('homeplus_vision'))
            / 'config'
            / 'task_registry.yaml'
        )

        # Parameters
        self.declare_parameter('device', 'auto')
        self.declare_parameter('model_name_or_path', 'allenai/MolmoPoint-8B')
        self.declare_parameter(
            'model_revision', '188130f961c8e0888a34e11121a1423c461a01ba'
        )
        self.declare_parameter('quantization', '4bit')
        self.declare_parameter(
            'model_cache_dir', str(workspace_root / '.models' / 'huggingface')
        )
        self.declare_parameter('local_files_only', False)
        self.declare_parameter('max_new_tokens', 64)
        self.declare_parameter('max_points', 1)
        self.declare_parameter('point_selection', 'closest_to_center')
        self.declare_parameter('publish_poses', True)
        # An empty caption selects the task preset. A non-empty caption is a
        # custom pointing prompt and can be changed while the node runs.
        self.declare_parameter('caption', '')
        self.declare_parameter('task_registry_path', str(default_task_registry))
        self.declare_parameter('grounded_sam_root', str(default_grounded_sam_root))
        self.declare_parameter('sam2_config_path', '')
        self.declare_parameter('sam2_checkpoint_path', '')
        self.declare_parameter('image_topic', '/cameras/d405/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/cameras/d405/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic', '/cameras/d405/color/camera_info'
        )
        self.declare_parameter('camera_frame', 'd405_color_optical_frame')
        self.declare_parameter('inference_rate', 0.5)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('tf_lookup_timeout_sec', 0.5)
        self.declare_parameter('depth_sample_radius_px', 5)
        self.declare_parameter('rgbd_max_skew_sec', 0.035)
        self.declare_parameter('max_observation_age_sec', 15.0)
        self.declare_parameter('motion_settle_sec', 0.7)
        self.declare_parameter('mask_depth_near_quantile', 0.25)
        self.declare_parameter('mask_depth_cluster_margin_m', 0.04)
        self.declare_parameter('mask_depth_min_points', 40)
        self.declare_parameter('cup_surface_to_center_m', 0.0)
        self.declare_parameter('cup_collision_radius_m', 0.065)
        self.declare_parameter('cup_collision_height_m', 0.15)
        self.declare_parameter('cup_min_radius_m', 0.025)
        self.declare_parameter('cup_max_radius_m', 0.065)
        self.declare_parameter('cup_height_from_mask_extent', False)
        self.declare_parameter('cup_min_palm_clearance_m', 0.0)
        # Placement targets (horizontal_surface tasks) are never on the cup the
        # gripper holds: reject a surface centred inside this cylinder around
        # held_object_frame (radius; extent below/above it). A 16 oz Solo cup held
        # 8.8 cm above its base reaches ~9 cm below and ~4 cm above grasp_tcp.
        # The cup stands on the table: its bottom IS the fitted table surface
        # (/vision/table_plane, ~1000 points), which is far steadier than the
        # mask's lowest depths at range (far and close-up grasp heights differed
        # by 4.5 cm, 2026-10-30). Used when the measured bottom is within this.
        self.declare_parameter('cup_bottom_from_table_max_diff_m', 0.06)
        self.declare_parameter('table_plane_max_age_sec', 10.0)
        self.declare_parameter('held_object_frame', 'grasp_tcp')
        # A horizontal_surface target (something to set an object on) must be
        # level: reject a masked surface whose fitted plane tilts more than this.
        # The drip-tray prompt alternated with the machine's vertical dispensing
        # recess 15 cm higher (2026-10-29).
        self.declare_parameter('surface_max_tilt_deg', 30.0)
        # Half-size (px) of the window around the model's point whose masked depth
        # defines a horizontal_surface target; a 9 cm tray spans ~28 px at 1.4 m.
        self.declare_parameter('surface_window_px', 12)
        self.declare_parameter('surface_window_min_points', 30)
        self.declare_parameter('held_object_exclusion_radius_m', 0.08)
        self.declare_parameter('held_object_exclusion_below_m', 0.12)
        self.declare_parameter('held_object_exclusion_above_m', 0.08)
        self.declare_parameter('gripper_palm_below_tcp_m', 0.047)
        self.declare_parameter('approach_reference_frame', '')
        self.declare_parameter(
            'target_audit_dir', str(workspace_root / 'detection_logs')
        )
        self.declare_parameter('debug_image_topic', '/molmopoint/debug_image')
        self.declare_parameter('detections_topic', '/molmopoint/detections')
        self.declare_parameter('pose_topic', '/molmopoint_pose_world')
        self.declare_parameter('mask_topic', '/object_mask')
        self.declare_parameter('mask_center_topic', '/molmopoint/point')
        self.declare_parameter('pause_during_motion', True)
        self.declare_parameter('motion_status_topic', '/arduino/status')
        # Empty runs MolmoPoint/SAM2 in-process. Otherwise POST /point and
        # /segment to this server (parallel_prompts/CONTRACT.md).
        self.declare_parameter('remote_inference_url', '')
        self.declare_parameter('remote_timeout_sec', 30.0)
        self.declare_parameter('diagnostics_topic', '/molmopoint/diagnostics')
        self.declare_parameter('debug_dump_dir', '')

        self.device = str(self.get_parameter('device').value).strip()
        self.model_name_or_path = str(
            self.get_parameter('model_name_or_path').value
        ).strip()
        self.model_revision = str(
            self.get_parameter('model_revision').value
        ).strip()
        self.quantization = str(
            self.get_parameter('quantization').value
        ).strip().lower()
        self.model_cache_dir = Path(
            str(self.get_parameter('model_cache_dir').value)
        ).expanduser()
        if not self.model_cache_dir.is_absolute():
            self.model_cache_dir = workspace_root / self.model_cache_dir
        self.model_cache_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('HF_HOME', str(self.model_cache_dir))
        os.environ.setdefault(
            'HF_MODULES_CACHE', str(self.model_cache_dir / 'modules')
        )
        self.local_files_only = bool(
            self.get_parameter('local_files_only').value
        )
        self.max_new_tokens = max(
            8, int(self.get_parameter('max_new_tokens').value)
        )
        self.max_points = max(1, int(self.get_parameter('max_points').value))
        self.point_selection = str(
            self.get_parameter('point_selection').value
        ).strip().lower()
        if self.quantization not in ('4bit', '8bit', 'none'):
            raise ValueError("quantization must be '4bit', '8bit', or 'none'")
        if self.point_selection not in ('model_order', 'closest_to_center'):
            raise ValueError(
                "point_selection must be 'model_order' or 'closest_to_center'"
            )
        self.publish_poses = bool(self.get_parameter('publish_poses').value)
        self.caption = str(self.get_parameter('caption').value)
        self.task_registry_path = Path(
            str(self.get_parameter('task_registry_path').value)
        ).expanduser()
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = str(self.get_parameter('camera_frame').value).strip()
        self.inference_rate = float(self.get_parameter('inference_rate').value)
        self.grounded_sam_root = Path(
            str(self.get_parameter('grounded_sam_root').value)).expanduser()
        self.enable_segmentation = False
        self.world_frame = str(self.get_parameter('world_frame').value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_lookup_timeout_sec').value))
        self.depth_sample_radius_px = max(
            0, int(self.get_parameter('depth_sample_radius_px').value)
        )
        self.mask_depth_near_quantile = min(
            0.9,
            max(0.0, float(self.get_parameter('mask_depth_near_quantile').value)),
        )
        self.mask_depth_cluster_margin_m = max(
            0.0,
            float(self.get_parameter('mask_depth_cluster_margin_m').value),
        )
        self.mask_depth_min_points = max(
            3, int(self.get_parameter('mask_depth_min_points').value)
        )
        self.pause_during_motion = bool(
            self.get_parameter('pause_during_motion').value
        )
        self.motion_status_topic = str(
            self.get_parameter('motion_status_topic').value
        )
        self.remote_inference_url = str(
            self.get_parameter('remote_inference_url').value
        ).strip()
        self.remote_timeout_sec = max(
            1.0, float(self.get_parameter('remote_timeout_sec').value)
        )
        self.debug_dump_dir = str(
            self.get_parameter('debug_dump_dir').value
        ).strip()
        target_audit_dir = Path(
            str(self.get_parameter('target_audit_dir').value)
        ).expanduser()
        target_audit_dir.mkdir(parents=True, exist_ok=True)
        audit_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.target_audit_path = target_audit_dir / f'molmopoint_targets_{audit_stamp}.jsonl'
        self.get_logger().info(f'Target detection audit: {self.target_audit_path}')

        # Target resolution for inference
        self.target_w = 640
        self.target_h = 480

        self.declare_parameter('task_id', 1)
        self.task_id = int(self.get_parameter('task_id').value)

        # Publishers
        self.debug_img_pub = self.create_publisher(
            Image, str(self.get_parameter('debug_image_topic').value), 10
        )
        self.detections_pub = self.create_publisher(
            String, str(self.get_parameter('detections_topic').value), 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('pose_topic').value), 10
        )
        self.geometry_pub = self.create_publisher(String, '/molmopoint/geometry', 10)
        self.mask_pub = self.create_publisher(
            Image, str(self.get_parameter('mask_topic').value), 10
        )
        self.mask_center_pub = self.create_publisher(
            String, str(self.get_parameter('mask_center_topic').value), 10
        )
        self.diagnostics_pub = self.create_publisher(
            String, str(self.get_parameter('diagnostics_topic').value), 10
        )

        # VLM inference may take longer than the nominal timer period. Keep TF
        # and sensor callbacks in callback groups that can run concurrently so
        # a long model call cannot leave the world transform several seconds
        # behind the robot's completed motion.
        self._sensor_callback_group = ReentrantCallbackGroup()
        self._inference_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_lock = threading.Lock()
        self._config_lock = threading.Lock()
        self._robot_moving = self.pause_during_motion
        self._motion_finished_stamp_ns = 0
        self._last_processed_color_stamp_ns = -1

        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self._depth_frames = deque(maxlen=60)
        self.camera_info = None
        self._have_logged_color = False
        self._have_logged_depth = False
        self._have_logged_camera_info = False

        # RealSense image/depth/camera-info topics are sensor streams and usually
        # publish with best-effort QoS. Match that profile so subscriptions
        # connect reliably instead of silently receiving nothing.
        self.create_subscription(String, '/vision/table_plane', self._on_table_plane, 10)
        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data, callback_group=self._sensor_callback_group,
        )
        if self.pause_during_motion:
            self.create_subscription(
                Bool, self.motion_status_topic, self.motion_status_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                callback_group=self._sensor_callback_group,
            )

        self.tasks = self._load_task_registry(self.task_registry_path)
        if self.task_id not in self.tasks:
            self.get_logger().warn(f"Invalid task_id {self.task_id}, defaulting to 1")
            self.task_id = 1
        self.task = self.tasks[self.task_id]
        # One pose topic per task that drives a motion command.
        self.task_pose_pubs = {task: self.create_publisher(PoseStamped,
            f'/molmopoint/task_{task}/pose', 10)
            for task, cfg in self.tasks.items() if cfg.get('command_name')}
        if not self.caption.strip():
            self.caption = self.task["caption"]

        # Keep the model resident while a test client changes the prompt. This
        # is the same callback/inference path used during
        # normal operation, so interactive tests cannot drift from the ROS node.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Model setup. Transformers is imported lazily so ROS tools can inspect
        # this package even when the dedicated MolmoPoint environment is absent.
        self._configure_import_paths()
        if self.remote_inference_url:
            self.enable_segmentation = True
            self._check_remote_health()
        else:
            self._init_molmopoint()

            if self.publish_poses:
                self._init_sam2()
        if self.publish_poses and not self.enable_segmentation:
            self.get_logger().error(
                'SAM2 segmentation is unavailable, but every 3-D target path '
                'requires a mask: no pose will ever be published. Fix the SAM2 '
                'install/checkpoint or set remote_inference_url.'
            )
        if self.pause_during_motion:
            self.get_logger().info(
                f'Detection starts paused until {self.motion_status_topic} '
                'reports the arm is still (Bool true)'
            )

        self.timer = self.create_timer(
            1.0 / max(self.inference_rate, 0.1),
            self.detect_loop,
            callback_group=self._inference_callback_group,
        )
        self.get_logger().info(
            f'MolmoPointNode initialized: model={self.model_name_or_path}, '
            f"inference={self.remote_inference_url or 'local'}, "
            f'quantization={self.quantization}, caption="{self.caption}", '
            f'world_frame="{self.world_frame}", '
            f"segmentation={'on' if self.enable_segmentation else 'off'}"
        )

    @staticmethod
    def _load_task_registry(path: Path):
        if not path.is_file():
            raise FileNotFoundError(f'Task registry not found: {path}')
        with path.open('r', encoding='utf-8') as handle:
            payload = yaml.safe_load(handle) or {}
        raw_tasks = payload.get('tasks', {})
        if not isinstance(raw_tasks, dict) or not raw_tasks:
            raise ValueError(f'No tasks configured in {path}')

        tasks = {}
        for raw_task_id, raw_task in raw_tasks.items():
            if not isinstance(raw_task, dict):
                raise ValueError(f'Task {raw_task_id} in {path} must be a map')
            task_id = int(raw_task_id)
            caption = str(raw_task.get('caption', '')).strip()
            if not caption:
                raise ValueError(f'Task {task_id} in {path} has no caption')
            tasks[task_id] = dict(raw_task)
            tasks[task_id]['caption'] = caption
        return tasks

    def _on_set_parameters(self, parameters):
        with self._config_lock:
            caption, task_id = self.caption, self.task_id
            for parameter in parameters:
                if parameter.name == 'task_id':
                    if parameter.value not in self.tasks:
                        return SetParametersResult(successful=False, reason='Unknown task_id')
                    task_id = int(parameter.value)
                    caption = self.tasks[task_id]['caption']
                elif parameter.name == 'caption':
                    if not isinstance(parameter.value, str) or not parameter.value.strip():
                        return SetParametersResult(successful=False, reason='caption must be nonempty text')
                    caption = parameter.value.strip()
            self.caption, self.task_id = caption, task_id
            self.task = self.tasks[task_id]
        return SetParametersResult(successful=True)

    def _on_table_plane(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        height = data.get('height_m')
        if height is not None and (data.get('found') or data.get('held') or data.get('kept')):
            self._table_height = (float(height), time.monotonic())

    def _fresh_table_height(self) -> Optional[float]:
        table = getattr(self, '_table_height', None)
        if table is None or time.monotonic() - table[1] > float(self.get_parameter('table_plane_max_age_sec').value):
            return None
        return table[0]

    def _inside_held_object(self, point_world, stamp) -> Optional[str]:
        """Reason to reject a placement target inside the held-object volume around
        held_object_frame, else None (also None when that frame has no TF)."""
        frame = str(self.get_parameter('held_object_frame').value).strip()
        if not frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, frame, Time.from_msg(stamp),
                                                 timeout=self._tf_timeout)
        except Exception:
            return None
        t = tf.transform.translation
        horizontal = math.hypot(point_world[0] - t.x, point_world[1] - t.y)
        dz = point_world[2] - t.z
        if (horizontal < float(self.get_parameter('held_object_exclusion_radius_m').value)
                and -float(self.get_parameter('held_object_exclusion_below_m').value) < dz
                < float(self.get_parameter('held_object_exclusion_above_m').value)):
            return (f'target_on_held_object: surface centre {horizontal:.3f} m from the {frame} axis, '
                    f'{dz:+.3f} m vertical; the held object hides the target')
        return None

    def _configure_import_paths(self) -> None:
        candidates = [
            self.grounded_sam_root,
        ]
        for candidate in candidates:
            if candidate.is_dir():
                candidate_str = str(candidate)
                if candidate_str not in sys.path:
                    sys.path.insert(0, candidate_str)

        sam2_config_root = self.grounded_sam_root / "sam2"
        if sam2_config_root.is_dir():
            os.environ["HYDRA_CONFIG_PATH"] = str(sam2_config_root / "configs")

    def _resolve_path(self,
                      parameter_name: str,
                      candidates,
                      required: bool = False) -> Optional[Path]:
        override = str(self.get_parameter(parameter_name).value).strip()
        if override:
            path = Path(override).expanduser()
            if path.is_file():
                return path
            raise FileNotFoundError(f"{parameter_name} not found: {path}")

        for candidate in candidates:
            if candidate.is_file():
                return candidate

        if required:
            tried = "\n".join(str(candidate) for candidate in candidates)
            raise FileNotFoundError(
                f"Required file for {parameter_name} not found. Tried:\n{tried}"
            )
        return None

    def _init_molmopoint(self) -> None:
        try:
            import torch
            from transformers import (
                AutoModelForImageTextToText,
                AutoProcessor,
                BitsAndBytesConfig,
            )
            from huggingface_hub import snapshot_download
        except ImportError as exc:
            raise RuntimeError(
                'MolmoPoint dependencies are missing. Run '
                '`bash scripts/setup_molmopoint.sh` before launching.'
            ) from exc

        if self.device == 'auto':
            if not torch.cuda.is_available():
                raise RuntimeError(
                    'MolmoPoint-8B requires CUDA for this deployment, but PyTorch '
                    'cannot access a GPU. Check the NVIDIA driver/container runtime.'
                )
            requested_device = 'cuda:0'
        else:
            requested_device = self.device

        if self.quantization in ('4bit', '8bit') and not requested_device.startswith('cuda'):
            raise RuntimeError(
                f'{self.quantization} MolmoPoint deployment requires a CUDA device'
            )

        checkpoint_path = Path(self.model_name_or_path).expanduser()
        if checkpoint_path.is_dir():
            checkpoint_source = str(checkpoint_path)
        else:
            try:
                checkpoint_source = snapshot_download(
                    repo_id=self.model_name_or_path,
                    revision=self.model_revision or None,
                    cache_dir=str(self.model_cache_dir),
                    local_files_only=self.local_files_only,
                )
            except Exception as exc:
                if self.local_files_only:
                    raise RuntimeError(
                        'The pinned MolmoPoint checkpoint is not complete in '
                        f'{self.model_cache_dir}. Run '
                        '`bash scripts/setup_molmopoint.sh --download-model`.'
                    ) from exc
                raise

        model_kwargs = {
            'trust_remote_code': True,
            'dtype': torch.bfloat16,
            'local_files_only': self.local_files_only,
        }
        if self.quantization == '4bit':
            model_kwargs['quantization_config'] = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type='nf4',
                bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_use_double_quant=True,
            )
            model_kwargs['device_map'] = {'': requested_device}
        elif self.quantization == '8bit':
            model_kwargs['quantization_config'] = BitsAndBytesConfig(load_in_8bit=True)
            model_kwargs['device_map'] = {'': requested_device}
        else:
            model_kwargs['device_map'] = {'': requested_device}

        self.get_logger().info(
            f'Loading {self.model_name_or_path} ({self.quantization}) on '
            f'{requested_device} from {checkpoint_source}'
        )
        self.processor = AutoProcessor.from_pretrained(
            checkpoint_source,
            trust_remote_code=True,
            padding_side='left',
            use_fast=False,
            local_files_only=self.local_files_only,
        )
        self.model = AutoModelForImageTextToText.from_pretrained(
            checkpoint_source,
            **model_kwargs,
        )
        self.model.eval()
        self.torch = torch
        self.model_device = next(self.model.parameters()).device
        self.inference_dtype = torch.bfloat16

    def _init_sam2(self) -> None:
        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as exc:
            self.get_logger().warn(
                f"SAM2 import failed; disabling segmentation/pose output: {exc}"
            )
            return

        sam2_config = self._resolve_path(
            "sam2_config_path",
            [
                self.grounded_sam_root / "sam2/configs/sam2.1/sam2.1_hiera_t.yaml",
                self.grounded_sam_root / "sam2/configs/sam2/sam2_hiera_t.yaml",
                self.grounded_sam_root / "sam2/sam2_hiera_t.yaml",
            ],
            required=False,
        )
        sam2_checkpoint = self._resolve_path(
            "sam2_checkpoint_path",
            [
                self.grounded_sam_root / "checkpoints/sam2_hiera_tiny.pt",
                self.grounded_sam_root / "checkpoints/sam2.1_hiera_tiny.pt",
            ],
            required=False,
        )

        if sam2_config is None or sam2_checkpoint is None:
            self.get_logger().warn(
                "SAM2 config/checkpoint not found; running detections without mask segmentation. "
                "3D pose output can still use detection bbox center plus depth."
            )
            return

        sam2_config_name = str(sam2_config)
        sam2_package_root = self.grounded_sam_root / "sam2"
        try:
            sam2_config_name = sam2_config.relative_to(sam2_package_root).as_posix()
        except ValueError:
            pass

        sam2_device = str(self.model_device)
        sam2_model = build_sam2(
            sam2_config_name,
            str(sam2_checkpoint),
            device=sam2_device,
        )
        self.sam2_model = SAM2ImagePredictor(sam2_model)
        self.enable_segmentation = True

    def image_callback(self, msg: Image):
        ahead_sec = (self._header_stamp_ns(msg.header) - self.get_clock().now().nanoseconds) / 1e9
        if ahead_sec > 0.1:
            self._future_frames_dropped += 1
            self.get_logger().error(
                f'Camera timestamp is {ahead_sec:.3f} s ahead of ROS at receipt; '
                'dropping frame. Check camera/host clock synchronization; '
                'a latest-TF fallback would hide this timing error.',
                throttle_duration_sec=5.0)
            return
        color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._state_lock:
            self.latest_color = color
            self.latest_color_header = msg.header
        if not self._have_logged_color:
            self.get_logger().info(f"Received first color frame on {self.image_topic}")
            self._have_logged_color = True

    def depth_callback(self, msg: Image):
        with self._state_lock:
            self.latest_depth = msg
            self._depth_frames.append(msg)
        if not self._have_logged_depth:
            self.get_logger().info(f"Received first depth frame on {self.depth_topic}")
            self._have_logged_depth = True

    def camera_info_callback(self, msg: CameraInfo):
        with self._state_lock:
            self.camera_info = msg
        if not self._have_logged_camera_info:
            self.get_logger().info(f"Received first camera info on {self.camera_info_topic}")
            self._have_logged_camera_info = True

    def motion_status_callback(self, msg: Bool):
        moving = not bool(msg.data)
        self._motion_status_msgs += 1
        if not moving:
            self._motion_still_seen = True
        with self._state_lock:
            changed = moving != self._robot_moving
            self._robot_moving = moving
            if not moving:
                # Only observations captured after DONE may be published. This
                # also invalidates inference that started on an in-motion image
                # but completed after the executor cleared its detection cache.
                self._motion_finished_stamp_ns = self.get_clock().now().nanoseconds
        if changed:
            state = 'paused during robot motion' if moving else 'resumed on fresh frames'
            self.get_logger().info(f'Detection {state}')

    @staticmethod
    def _header_stamp_ns(header) -> int:
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def _frame_is_publishable(self, frame_stamp_ns: int) -> bool:
        if not self.pause_during_motion:
            return True
        with self._state_lock:
            return (
                not self._robot_moving
                and frame_stamp_ns > self._motion_finished_stamp_ns + int(float(self.get_parameter('motion_settle_sec').value) * 1e9)
                and 0 <= self.get_clock().now().nanoseconds - frame_stamp_ns < int(float(self.get_parameter('max_observation_age_sec').value) * 1e9)
            )

    def detect_loop(self):
        diag = self._new_cycle_diagnostics()
        with self._state_lock:
            color = self.latest_color
            header = self.latest_color_header
            depth = min(self._depth_frames, key=lambda d: abs(self._header_stamp_ns(d.header) -
                        self._header_stamp_ns(header)), default=None) if header else None
            camera_info = self.camera_info
            robot_moving = self._robot_moving
            motion_finished_stamp_ns = self._motion_finished_stamp_ns
        diag['robot_moving'] = bool(robot_moving)

        if color is None or header is None or camera_info is None:
            if color is None or header is None:
                self.get_logger().warn(
                    f"Waiting for color image on {self.image_topic}",
                    throttle_duration_sec=5.0,
                )
            if camera_info is None:
                self.get_logger().warn(
                    f"Waiting for camera info on {self.camera_info_topic}",
                    throttle_duration_sec=5.0,
                )
            # Already logged above with its own throttle.
            self._report_cycle(diag, 'inputs', 'waiting_for_color_or_camera_info')
            return

        frame_stamp_ns = self._header_stamp_ns(header)
        diag['stamp_ns'] = frame_stamp_ns
        diag['frame_age_sec'] = round(
            (self.get_clock().now().nanoseconds - frame_stamp_ns) / 1e9, 3
        )
        if depth is None:
            self._report_cycle(
                diag, 'inputs', 'no_depth_frame',
                f"No depth frame received on {getattr(self, 'depth_topic', '?')}; "
                'skipping cycle', 'warn')
            return
        rgbd_skew_sec = abs(self._header_stamp_ns(depth.header) - frame_stamp_ns) / 1e9
        diag['rgbd_skew_sec'] = round(rgbd_skew_sec, 4)
        max_skew_sec = float(self.get_parameter('rgbd_max_skew_sec').value)
        if rgbd_skew_sec > max_skew_sec:
            self._report_cycle(
                diag, 'inputs', 'rgbd_skew',
                f'Nearest depth frame is {rgbd_skew_sec * 1e3:.1f} ms from the '
                f'color frame (rgbd_max_skew_sec={max_skew_sec}); skipping cycle',
                'warn')
            return
        if self.pause_during_motion and (
            robot_moving or frame_stamp_ns <= motion_finished_stamp_ns
        ):
            topic = getattr(self, 'motion_status_topic', '/arduino/status')
            if robot_moving and not self._motion_status_msgs:
                reason, level, message = 'motion_status_never_received', 'warn', (
                    f'Detection paused: no message ever received on {topic} '
                    '(pause_during_motion=true starts paused). Is arduino_reader '
                    'running and publishing?')
            elif robot_moving and not self._motion_still_seen:
                reason, level, message = 'motion_status_never_still', 'warn', (
                    f'Detection paused: {topic} has only reported false (moving/'
                    'not ready); waiting for the first true (arm still). Check '
                    'Arduino serial connection and settled feedback.')
            elif robot_moving:
                reason, level, message = 'robot_moving', 'info', (
                    f'Detection paused: {topic} reports the arm is moving')
            else:
                reason, level, message = 'frame_before_motion_done', 'info', (
                    'Skipping color frame stamped before the last motion '
                    f'DONE ({(motion_finished_stamp_ns - frame_stamp_ns) / 1e9:.2f} s '
                    'earlier); waiting for a fresh frame')
            self._report_cycle(diag, 'motion_gate', reason, message, level)
            return
        with self._state_lock:
            already_processed = frame_stamp_ns <= self._last_processed_color_stamp_ns
            if not already_processed:
                self._last_processed_color_stamp_ns = frame_stamp_ns
        if already_processed:
            message = (
                'No new color frame since the last cycle '
                f"(latest frame age {diag['frame_age_sec']:.2f} s)"
            )
            if self._future_frames_dropped:
                message += (
                    f'; {self._future_frames_dropped} frame(s) dropped so far for '
                    'camera timestamps ahead of the ROS clock'
                )
            diag['future_frames_dropped'] = self._future_frames_dropped
            self._report_cycle(diag, 'frame_gate', 'frame_already_processed', message)
            return

        self.get_logger().info("Running detection loop")
        # 1. Resize Color for Inference
        img_resized = cv2.resize(color, (self.target_w, self.target_h))

        with self._config_lock:
            caption = self.caption
            task_id = self.task_id
        diag['caption'] = caption
        diag['task_id'] = task_id
        try:
            frame_tf = self.tf_buffer.lookup_transform(self.world_frame,
                self.camera_frame or camera_info.header.frame_id, Time.from_msg(header.stamp),
                timeout=self._tf_timeout)
        except Exception as exc:
            diag['tf_ok'] = False
            self._report_cycle(
                diag, 'tf', 'tf_lookup_failed',
                f'No acquisition-time camera transform {self.world_frame} <- '
                f'{self.camera_frame or camera_info.header.frame_id} '
                f"(frame age {diag['frame_age_sec']:.2f} s): {exc}", 'warn')
            return
        diag['tf_ok'] = True
        self._dump_cycle_inputs(color, depth, camera_info, frame_tf,
                                frame_stamp_ns, caption, task_id)

        debug_img = img_resized.copy()
        inference_start = time.monotonic()
        try:
            detections = self.run_model_inference(img_resized, caption)
        except Exception as exc:
            diag['inference_sec'] = round(time.monotonic() - inference_start, 3)
            self._report_cycle(
                diag, 'inference', 'inference_failed',
                f'MolmoPoint inference failed: {exc}', 'error', debug_img=debug_img)
            return
        diag['inference_sec'] = round(time.monotonic() - inference_start, 3)
        diag['n_model_points'] = len(detections)
        diag.update(self._last_inference_info or {})
        if not self._frame_is_publishable(frame_stamp_ns):
            reason, detail = self._unpublishable_reason(frame_stamp_ns)
            self._draw_model_points(debug_img, detections)
            self._report_cycle(
                diag, 'publish_gate', reason,
                f'Discarding detection captured before robot motion completed: {detail}',
                'info', debug_img=debug_img)
            return
        self.detections_pub.publish(String(data=json.dumps(detections)))

        targets = diag['targets'] = []
        for d in detections:
            target_diag = {'model_point': list(d['point'])}
            targets.append(target_diag)
            xmin, ymin, xmax, ymax = map(int, d["bbox"])
            xmin = max(0, min(xmin, self.target_w - 1))
            xmax = max(0, min(xmax, self.target_w))
            ymin = max(0, min(ymin, self.target_h - 1))
            ymax = max(0, min(ymax, self.target_h))
            if xmax <= xmin or ymax <= ymin:
                target_diag['reason'] = 'degenerate_bbox'
                continue

            center_u = int(d['point'][0])
            center_v = int(d['point'][1])
            full_mask = None

            if self.enable_segmentation:
                segment_start = time.monotonic()
                try:
                    masks, _ = self.run_segmentation(
                        img_resized,
                        (center_u, center_v),
                    )
                except Exception as exc:
                    target_diag['segment_sec'] = round(time.monotonic() - segment_start, 3)
                    self._draw_model_points(debug_img, detections)
                    self._report_cycle(
                        diag, 'segmentation', 'segmentation_failed',
                        f'SAM2 segmentation failed: {exc}', 'error',
                        debug_img=debug_img)
                    if self.remote_inference_url:
                        return
                    raise
                target_diag['segment_sec'] = round(time.monotonic() - segment_start, 3)
                target_diag.update(self._last_segment_info or {})
                target_diag['mask_pixels'] = 0
                if masks is not None and len(masks) > 0:
                    full_mask = masks[0].astype(np.uint8) * 255
                    mask_ys, mask_xs = np.where(full_mask > 0)
                    target_diag['mask_pixels'] = int(mask_xs.size)
                    if mask_xs.size > 0:
                        center_u = int(np.mean(mask_xs))
                        center_v = int(np.mean(mask_ys))
                        mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding='mono8')
                        mask_msg.header = header
                        # Published only after geometry and final motion/config checks.
                    self._draw_mask_overlay(debug_img, full_mask)
                else:
                    target_diag['reason'] = 'sam2_returned_no_mask'
            else:
                target_diag['reason'] = 'segmentation_disabled'

            mask_center_payload = {
                "label": d["label"],
                "score": float(d["score"]),
                "backend": "molmopoint",
                "model_point": {"u": int(d['point'][0]), "v": int(d['point'][1])},
                "pixel": {"u": center_u, "v": center_v},
            }

            point_3d = None
            geometry = None
            approach = None
            depth_source = None
            depth_point_count = 0
            geometry_diagnostics = {}
            if self.publish_poses and depth is not None and full_mask is not None:
                arr = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
                z = arr.astype(np.float64) * (0.001 if arr.dtype == np.uint16 else 1.0)
                mask = cv2.resize(full_mask, (depth.width, depth.height), interpolation=cv2.INTER_NEAREST) > 0
                mask = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
                rows, cols = np.nonzero(mask & np.isfinite(z) & (z > 0.12) & (z < 2.0))
                fx = camera_info.k[0] * depth.width / camera_info.width
                fy = camera_info.k[4] * depth.height / camera_info.height
                cx = camera_info.k[2] * depth.width / camera_info.width
                cy = camera_info.k[5] * depth.height / camera_info.height
                ranges = z[rows, cols]
                points = np.column_stack(((cols-cx)*ranges/fx, (rows-cy)*ranges/fy, ranges))
                rotation, translation = self._transform_matrix(frame_tf)
                cup_bottom_z = None
                if task_id == 1 and caption == self.tasks[1]['caption']:
                    estimate = self._estimate_point_from_mask_depth(
                        depth, full_mask, camera_info, geometry_diagnostics)
                    if estimate is not None:
                        surface_world = rotation @ np.asarray(estimate[:3]) + translation
                        if (self.get_parameter('cup_height_from_mask_extent').value
                                and len(points) >= self.mask_depth_min_points):
                            # The near-depth cluster is the rim when the hand
                            # camera looks down, so take the grasp height from
                            # the middle of the cup's measured vertical extent.
                            heights = (points @ rotation.T + translation)[:, 2]
                            bottom, top = np.quantile(heights, [0.05, 0.95])
                            target_diag['cup_extent_z'] = [float(bottom), float(top)]
                            table_z = self._fresh_table_height()
                            if (table_z is not None and abs(bottom - table_z)
                                    <= float(self.get_parameter('cup_bottom_from_table_max_diff_m').value)):
                                target_diag['cup_bottom_from_table'] = [float(bottom), float(table_z)]
                                bottom = table_z
                            cup_bottom_z = float(bottom)
                            target_diag['near_cluster_z'] = float(surface_world[2])
                            surface_world[2] = 0.5 * (bottom + top)
                            clearance = float(self.get_parameter('cup_min_palm_clearance_m').value)
                            if clearance > 0.0:
                                # The palm reaches below the TCP; on a short cup a
                                # mid-height grasp leaves it closer to the table
                                # (the cup's bottom) than the OctoMap can resolve.
                                # Raise the grasp until the palm keeps `clearance`.
                                palm = float(self.get_parameter('gripper_palm_below_tcp_m').value)
                                floor_z = bottom + palm + clearance
                                if floor_z > surface_world[2]:
                                    target_diag['grasp_z_raised_for_palm_clearance'] = [
                                        float(surface_world[2]), float(floor_z)]
                                    surface_world[2] = floor_z
                        target_diag['surface_world'] = surface_world.tolist()
                        try:
                            viewpoint = translation
                            reference = str(self.get_parameter('approach_reference_frame').value).strip()
                            if reference:
                                # The hand camera sits beside the gripper; at close
                                # range a camera-to-cup ray is yawed far off the
                                # arm's line, so measure the approach from the TCP.
                                try:
                                    tcp_tf = self.tf_buffer.lookup_transform(
                                        self.world_frame, reference,
                                        Time.from_msg(header.stamp), timeout=self._tf_timeout)
                                    t = tcp_tf.transform.translation
                                    viewpoint = np.array([t.x, t.y, t.z])
                                    target_diag['approach_reference'] = reference
                                except Exception as exc:
                                    target_diag['approach_reference'] = f'camera ({exc})'
                            world_center, approach = offset_surface_goal(surface_world, viewpoint,
                                float(self.get_parameter('cup_surface_to_center_m').value))
                            point_3d = tuple(rotation.T @ (world_center - translation))
                            center_u, center_v = estimate[3:5]
                            geometry = {
                                'radius': float(self.get_parameter('cup_collision_radius_m').value),
                                'height': float(self.get_parameter('cup_collision_height_m').value),
                                'source': 'configured_collision_envelope',
                            }
                            # The detection is a front surface, not the object
                            # center. Anchor the envelope behind that surface;
                            # never extend a guessed radius toward the palm.
                            collision_center = surface_world + geometry['radius'] * approach
                            collision_center[2] -= 0.15 * geometry['height']
                            geometry['collision_center'] = collision_center.tolist()
                            if cup_bottom_z is not None:
                                # Measured, unlike the envelope: the executor
                                # records it at closure to place the cup later.
                                geometry['bottom_z'] = cup_bottom_z
                            depth_source = 'segmented_depth_with_center_offset'
                        except ValueError as exc:
                            geometry_diagnostics['reason'] = f'offset_surface_goal: {exc}'
                elif task_id == 2 and caption == self.tasks[2]['caption']:
                    u = d['point'][0] * depth.width / self.target_w
                    v = d['point'][1] * depth.height / self.target_h
                    local = (abs(cols-u) <= 15) & (abs(rows-v) <= 15)
                    estimate = fit_button_plane(points[local], [(u-cx)/fx, (v-cy)/fy, 1.0])
                    if estimate is not None:
                        point_3d, normal = estimate
                        approach = rotation @ normal
                        geometry = {'normal': approach.tolist()}
                        depth_source = 'button_plane_ray_intersection'
                    else:
                        geometry_diagnostics['reason'] = (
                            'button plane fit rejected '
                            f'({int(np.count_nonzero(local))} depth points within 15 px)'
                        )
                elif (self.tasks[task_id].get('target_geometry') == 'horizontal_surface'
                        and caption == self.tasks[task_id]['caption']):
                    # A flat support to put something on (e.g. a drip tray): the
                    # model's point is on it, but SAM can grow the mask over the
                    # whole appliance (2026-10-29: a drip-tray point gave a
                    # whole-machine mask whose centroid sat on its vertical front).
                    # So use the masked depth in a window around the point: its
                    # centroid at the median measured height, not a body centre.
                    u = d['point'][0] * depth.width / self.target_w
                    v = d['point'][1] * depth.height / self.target_h
                    r = float(self.get_parameter('surface_window_px').value)
                    local = (abs(cols-u) <= r) & (abs(rows-v) <= r)
                    need = int(self.get_parameter('surface_window_min_points').value)
                    target_diag['surface_window_points'] = int(np.count_nonzero(local))
                    if np.count_nonzero(local) < need:
                        geometry_diagnostics['reason'] = (
                            f'too few masked depth points around the model point: '
                            f'{int(np.count_nonzero(local))} < {need} within {r:.0f} px')
                    else:
                        world = points[local] @ rotation.T + translation
                        low, height, high = np.quantile(world[:, 2], [0.25, 0.5, 0.75])
                        # Spread of heights (grate slots, rim, depth noise), for
                        # judging the surface estimate; not a rejection.
                        target_diag['surface_z_quartiles'] = [float(low), float(high)]
                        centroid = np.array([np.median(world[:, 0]), np.median(world[:, 1]), height])
                        # Plane through the masked points (inner 80% by height, so
                        # a rim or stray background pixels don't tip the fit).
                        core = world[(world[:, 2] >= np.quantile(world[:, 2], 0.1))
                                     & (world[:, 2] <= np.quantile(world[:, 2], 0.9))]
                        normal = np.linalg.svd(core - core.mean(axis=0), full_matrices=False)[2][-1]
                        tilt = float(np.degrees(np.arccos(min(1.0, abs(normal[2])))))
                        target_diag['surface_tilt_deg'] = round(tilt, 1)
                        held = self._inside_held_object(centroid, header.stamp)
                        max_tilt = float(self.get_parameter('surface_max_tilt_deg').value)
                        if tilt > max_tilt:
                            geometry_diagnostics['reason'] = (
                                f'surface_not_level: masked surface tilts {tilt:.0f} deg from horizontal '
                                f'(limit {max_tilt:.0f}); not something to set an object on')
                        elif held is not None:
                            # e.g. the inside of the held cup when it hides the tray
                            geometry_diagnostics['reason'] = held
                        else:
                            point_3d = tuple(rotation.T @ (centroid - translation))
                        # Horizontal camera-to-target direction: a level pose
                        # facing the surface (pitch 0), like the cup's.
                        approach = centroid - translation
                        approach[2] = 0.0
                        depth_source = 'segmented_surface_centroid'
                else:
                    estimate = self._estimate_point_from_mask_depth(
                        depth, full_mask, camera_info, geometry_diagnostics)
                    if estimate is not None:
                        point_3d = estimate[:3]
                        depth_source = 'segmented_depth_cloud'
                depth_point_count = len(points)
                target_diag['valid_depth_points'] = depth_point_count
                self._draw_depth_points(debug_img, geometry_diagnostics)
            target_diag.update({
                key: value for key, value in geometry_diagnostics.items()
                if not key.startswith('_') and key != 'reason'
            })
            if 'reason' in geometry_diagnostics:
                target_diag['reason'] = geometry_diagnostics['reason']
            # Manipulation targets must never silently fall back to background
            # depth or a view-dependent visible-surface centroid.
            with self._config_lock:
                unchanged = caption == self.caption and task_id == self.task_id
            if not unchanged or not self._frame_is_publishable(frame_stamp_ns):
                if not unchanged:
                    reason = 'caption_or_task_changed'
                    detail = (f'caption/task changed to "{self.caption}"/{self.task_id} '
                              'during inference')
                else:
                    reason, detail = self._unpublishable_reason(frame_stamp_ns)
                self._draw_model_points(debug_img, detections)
                self._report_cycle(
                    diag, 'publish_gate', reason,
                    f'Discarding target before publishing: {detail}', 'info',
                    debug_img=debug_img)
                return
            if full_mask is not None:
                mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding='mono8')
                mask_msg.header = deepcopy(header)
                self.mask_pub.publish(mask_msg)

            mask_center_payload["pixel"] = {"u": center_u, "v": center_v}
            target_diag['chosen_pixel'] = [int(center_u), int(center_v)]

            if self.publish_poses and point_3d is not None:
                X, Y, Z = point_3d
                camera_frame = self.camera_frame or camera_info.header.frame_id
                target_diag['point_camera'] = [float(X), float(Y), float(Z)]
                target_diag['depth_source'] = depth_source

                pose_cam = PoseStamped()
                pose_cam.header = deepcopy(header)
                pose_cam.header.frame_id = camera_frame
                pose_cam.pose.position.x = X
                pose_cam.pose.position.y = Y
                pose_cam.pose.position.z = Z
                pose_cam.pose.orientation.w = 1.0

                mask_center_payload["point_3d_camera"] = {"x": X, "y": Y, "z": Z}
                mask_center_payload["depth_source"] = depth_source
                mask_center_payload["depth_point_count"] = depth_point_count
                self.get_logger().info(
                    f"Target {d['label']} in {camera_frame}: "
                    f"x={X:.2f}m, y={Y:.2f}m, z={Z:.2f}m "
                    f"(pixel=({center_u}, {center_v}), "
                    f"source={depth_source}, points={depth_point_count})"
                )

                pose_world = self._transform_pose_to_world(pose_cam, frame_tf)
                if pose_world is not None:
                    if approach is not None:
                        length = np.linalg.norm(approach)
                        if length < 1e-6:
                            target_diag['reason'] = 'degenerate_approach_vector'
                            self.get_logger().warning(
                                f"Target {d['label']}: approach vector has zero "
                                'length; not publishing pose',
                                throttle_duration_sec=5.0)
                            continue
                        approach /= length
                        yaw = math.atan2(approach[1], approach[0])
                        pitch = math.atan2(-approach[2], math.hypot(approach[0], approach[1]))
                        pose_world.pose.orientation.x = -math.sin(pitch/2) * math.sin(yaw/2)
                        pose_world.pose.orientation.y = math.sin(pitch/2) * math.cos(yaw/2)
                        pose_world.pose.orientation.z = math.cos(pitch/2) * math.sin(yaw/2)
                        pose_world.pose.orientation.w = math.cos(pitch/2) * math.cos(yaw/2)
                    self.pose_pub.publish(pose_world)
                    if task_id in self.task_pose_pubs and caption == self.tasks[task_id]['caption']:
                        self.task_pose_pubs[task_id].publish(pose_world)
                    if geometry is not None:
                        geometry.update(task_id=task_id, stamp_ns=frame_stamp_ns,
                            center=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z])
                        self.geometry_pub.publish(String(data=json.dumps(geometry)))
                    pw = pose_world.pose.position
                    mask_center_payload["point_3d_world"] = {"x": pw.x, "y": pw.y, "z": pw.z}
                    target_diag['point_world'] = [float(pw.x), float(pw.y), float(pw.z)]
                    target_diag['pose_published'] = True
                    self.get_logger().info(
                        f"Target {d['label']} in {self.world_frame}: "
                        f"x={pw.x:.2f}m, y={pw.y:.2f}m, z={pw.z:.2f}m"
                    )
                else:
                    target_diag['reason'] = 'world_transform_failed'
            elif self.publish_poses:
                target_diag.setdefault('reason', 'missing mask/depth or rejected surface fit')
                self.get_logger().info(
                    f"Target {d['label']} detected at pixel=({center_u}, {center_v}), "
                    f"but depth/geometry was insufficient for a reliable 3-D target: "
                    f"{geometry_diagnostics.get('reason', target_diag['reason'])}; "
                    f"model_point={d['point']}, valid_depth_points={depth_point_count}, "
                    f"mask_pixels={int(np.count_nonzero(full_mask)) if full_mask is not None else 0}"
                )
            else:
                self.get_logger().info(
                    f"Target {d['label']} detected at pixel=({center_u}, {center_v})"
                )

            self.mask_center_pub.publish(String(data=json.dumps(mask_center_payload)))
            self._append_target_audit(mask_center_payload, header)

            # Draw Debug
            cv2.circle(
                debug_img,
                (int(d['point'][0]), int(d['point'][1])),
                9,
                (0, 0, 255),
                2,
            )
            cv2.circle(debug_img, (center_u, center_v), 5, (255, 0, 0), -1)
            cv2.putText(
                debug_img,
                d['label'],
                (max(0, center_u - 80), max(20, center_v - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        if not detections:
            info = self._last_inference_info or {}
            self._report_cycle(
                diag, 'inference', 'no_model_points',
                f'MolmoPoint returned no image-0 points for "{caption}" '
                f"(raw points={info.get('raw_points', '?')}, "
                f"text={info.get('generated_text', '')[:120]!r})",
                'warn', debug_img=debug_img)
        elif any(target.get('pose_published') for target in targets):
            self._report_cycle(diag, 'done', 'pose_published', debug_img=debug_img)
        elif not self.publish_poses:
            self._report_cycle(diag, 'done', 'detected_poses_disabled', debug_img=debug_img)
        else:
            reason = targets[0].get('reason', 'no_pose')
            self._report_cycle(
                diag, 'geometry', reason,
                f'No 3-D target published: {reason}; '
                + ', '.join(f'{k}={targets[0][k]}' for k in (
                    'model_point', 'mask_pixels', 'valid_depth_points',
                    'mask_depth_valid', 'mask_depth_kept', 'near_depth_m')
                    if k in targets[0]),
                'warn', debug_img=debug_img)

    # ------------------------------------------------------------------
    # Diagnostics

    def _new_cycle_diagnostics(self) -> dict:
        return {
            'backend': 'remote' if self.remote_inference_url else 'local',
            'pause_during_motion': bool(getattr(self, 'pause_during_motion', False)),
            'motion_status_msgs': self._motion_status_msgs,
            'motion_still_seen': self._motion_still_seen,
            'segmentation': bool(getattr(self, 'enable_segmentation', False)),
            '_t0': time.monotonic(),
        }

    def _report_cycle(self, diag: dict, stage: str, reason: str,
                      message: Optional[str] = None, level: str = 'info',
                      debug_img=None) -> None:
        """End a detection cycle: publish diagnostics and log why (throttled)."""
        diag['stage'] = stage
        diag['reason'] = reason
        if message:
            diag['detail'] = message
        diag['cycle_sec'] = round(time.monotonic() - diag.pop('_t0', time.monotonic()), 3)
        targets = diag.get('targets')
        if targets:
            for key, value in targets[0].items():
                diag.setdefault(key, value)
            if len(targets) == 1:
                diag.pop('targets')
        if debug_img is not None:
            text = f'{stage}: {reason}'[:90]
            cv2.putText(debug_img, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(debug_img, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
        if self.diagnostics_pub is not None:
            self.diagnostics_pub.publish(
                String(data=json.dumps(diag, default=self._json_default)))
        if not message:
            return
        # Log every change of reason immediately; repeat a reason every 5 s.
        key = (stage, reason)
        now = time.monotonic()
        if self._diag_last_log is None:
            self._diag_last_log = {}
        last = self._diag_last_log.get(key)
        if key != self._diag_last_key or last is None or now - last >= 5.0:
            self._diag_last_log[key] = now
            # rclpy binds one severity to each logging call site, so every
            # level needs its own call.
            text = f'[{stage}/{reason}] {message}'
            if level == 'error':
                self.get_logger().error(text)
            elif level in ('warn', 'warning'):
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
        self._diag_last_key = key

    @staticmethod
    def _json_default(value):
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
        return str(value)

    def _unpublishable_reason(self, frame_stamp_ns: int) -> Tuple[str, str]:
        """Name the _frame_is_publishable condition that failed."""
        with self._state_lock:
            moving = self._robot_moving
            finished_ns = self._motion_finished_stamp_ns
        settle_sec = float(self.get_parameter('motion_settle_sec').value)
        max_age_sec = float(self.get_parameter('max_observation_age_sec').value)
        age_sec = (self.get_clock().now().nanoseconds - frame_stamp_ns) / 1e9
        if moving:
            return 'robot_moving', 'arm reported moving during inference'
        if frame_stamp_ns <= finished_ns + int(settle_sec * 1e9):
            return 'frame_within_motion_settle', (
                f'frame stamped {(frame_stamp_ns - finished_ns) / 1e9:.2f} s after '
                f'motion DONE, motion_settle_sec={settle_sec}')
        if age_sec < 0:
            return 'frame_from_future', f'frame stamp is {-age_sec:.2f} s ahead of ROS clock'
        if age_sec >= max_age_sec:
            return 'frame_stale', (
                f'frame age {age_sec:.2f} s >= max_observation_age_sec={max_age_sec}')
        return 'not_publishable', 'publish gate failed'

    def _draw_model_points(self, debug_img, detections) -> None:
        for d in detections:
            cv2.circle(debug_img, (int(d['point'][0]), int(d['point'][1])),
                       9, (0, 0, 255), 2)

    @staticmethod
    def _draw_mask_overlay(debug_img, full_mask) -> None:
        mask = full_mask > 0
        if mask.shape != debug_img.shape[:2] or not mask.any():
            return
        debug_img[mask] = (
            0.6 * debug_img[mask] + 0.4 * np.array([0, 255, 0])
        ).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug_img, contours, -1, (0, 255, 0), 1)

    def _draw_depth_points(self, debug_img, geometry_diagnostics: dict) -> None:
        """Mark the near-depth mask pixels the 3-D estimate actually used."""
        rows = geometry_diagnostics.get('_kept_rows')
        cols = geometry_diagnostics.get('_kept_cols')
        shape = geometry_diagnostics.get('_depth_shape')
        if rows is None or cols is None or shape is None or len(rows) == 0:
            return
        step = max(4, len(rows) // 600)
        us = (cols[::step] * self.target_w / shape[1]).astype(int)
        vs = (rows[::step] * self.target_h / shape[0]).astype(int)
        keep = (us >= 0) & (us < debug_img.shape[1]) & (vs >= 0) & (vs < debug_img.shape[0])
        debug_img[vs[keep], us[keep]] = (0, 255, 255)

    def _dump_cycle_inputs(self, color, depth, camera_info, frame_tf,
                           frame_stamp_ns, caption, task_id) -> None:
        """Save this cycle's inputs for tests/detector_replay.py."""
        if not self.debug_dump_dir:
            return
        try:
            out_dir = Path(self.debug_dump_dir).expanduser()
            out_dir.mkdir(parents=True, exist_ok=True)
            rotation, translation = self._transform_matrix(frame_tf)
            world_from_camera = np.eye(4)
            world_from_camera[:3, :3] = rotation
            world_from_camera[:3, 3] = translation
            np.savez_compressed(
                out_dir / f'cycle_{frame_stamp_ns}.npz',
                color=color,
                depth=self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough'),
                depth_encoding=depth.encoding,
                depth_stamp_ns=self._header_stamp_ns(depth.header),
                K=np.asarray(camera_info.k, dtype=np.float64),
                camera_width=int(camera_info.width),
                camera_height=int(camera_info.height),
                T_world_camera=world_from_camera,
                stamp_ns=frame_stamp_ns,
                caption=caption,
                task_id=task_id,
                camera_frame=self.camera_frame or camera_info.header.frame_id,
                world_frame=self.world_frame,
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Could not dump cycle inputs: {exc}', throttle_duration_sec=5.0)

    def _append_target_audit(self, payload: dict, header) -> None:
        """Persist camera/world target coordinates for later plan diagnosis."""
        record = dict(payload)
        record['recorded_at'] = datetime.now().astimezone().isoformat()
        record['image_frame'] = header.frame_id
        record['image_stamp'] = {
            'sec': int(header.stamp.sec),
            'nanosec': int(header.stamp.nanosec),
        }
        try:
            with self.target_audit_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(record, sort_keys=True) + '\n')
        except Exception as exc:
            self.get_logger().warning(
                f'Could not write target detection audit: {exc}',
                throttle_duration_sec=5.0,
            )

    @staticmethod
    def _transform_matrix(tf_msg):
        q = tf_msg.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        t = tf_msg.transform.translation
        return rotation, np.array([t.x, t.y, t.z])

    def _transform_pose_to_world(self, pose_cam, tf_msg=None):
        try:
            if tf_msg is None:
                tf_msg = self.tf_buffer.lookup_transform(self.world_frame, pose_cam.header.frame_id,
                    Time.from_msg(pose_cam.header.stamp), timeout=self._tf_timeout)
            world = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, tf_msg)
            world.header.frame_id = self.world_frame
            world.header.stamp = pose_cam.header.stamp
            return world
        except Exception as exc:
            self.get_logger().warning(f'Acquisition-time transform unavailable: {exc}')
            return None

    # Scale XYZ to the resized image.
    def _backproject_pixel_to_3d(
            self, u: int, v: int, z: float,
            cam_info: CameraInfo) -> Tuple[float, float, float]:
        scale_x = self.target_w / float(cam_info.width)
        scale_y = self.target_h / float(cam_info.height)

        fx = cam_info.k[0] * scale_x
        fy = cam_info.k[4] * scale_y
        cx = cam_info.k[2] * scale_x
        cy = cam_info.k[5] * scale_y

        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z
        return X, Y, Z

    def _estimate_point_from_mask_depth(
        self,
        depth_msg: Image,
        mask: np.ndarray,
        cam_info: CameraInfo,
        diagnostics: Optional[dict] = None,
    ) -> Optional[Tuple[float, float, float, int, int, int]]:
        """
        Estimate a robust visible-object centroid from the segmented depth.

        Cup openings frequently expose the table or inner wall at the mask
        center. Using a single center patch therefore biases the target toward
        the support surface. This method uses the complete segmented point set
        and keeps the coherent near-depth portion before taking its median.

        If ``diagnostics`` is given it receives the point counts and, on
        rejection, a ``reason``.
        """
        if diagnostics is None:
            diagnostics = {}
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough'
            )
            depth_is_uint16 = depth_raw.dtype == np.uint16
            depth = depth_raw.astype(np.float32, copy=False)
            if depth_msg.encoding in ('16UC1', 'mono16') or depth_is_uint16:
                depth = depth * 0.001

            height, width = depth.shape[:2]
            depth_mask = cv2.resize(
                mask,
                (width, height),
                interpolation=cv2.INTER_NEAREST,
            ) > 0
            valid = depth_mask & np.isfinite(depth) & (depth > 0.1)
            rows, cols = np.nonzero(valid)
            diagnostics['mask_pixels_depth_res'] = int(np.count_nonzero(depth_mask))
            diagnostics['mask_depth_valid'] = int(rows.size)
            if rows.size < self.mask_depth_min_points:
                diagnostics['reason'] = (
                    f'too few valid depth pixels inside mask: {rows.size} < '
                    f'mask_depth_min_points={self.mask_depth_min_points} '
                    f'(mask has {diagnostics["mask_pixels_depth_res"]} px at depth resolution)'
                )
                return None

            ranges = depth[rows, cols]
            near_depth = float(
                np.quantile(ranges, self.mask_depth_near_quantile)
            )
            keep = ranges <= near_depth + self.mask_depth_cluster_margin_m
            rows = rows[keep]
            cols = cols[keep]
            ranges = ranges[keep]
            diagnostics['near_depth_m'] = round(near_depth, 4)
            diagnostics['mask_depth_kept'] = int(rows.size)
            if rows.size < self.mask_depth_min_points:
                diagnostics['reason'] = (
                    f'too few near-depth cluster points: {rows.size} < '
                    f'mask_depth_min_points={self.mask_depth_min_points} '
                    f'(near_depth={near_depth:.3f} m)'
                )
                return None

            info_width = max(1, int(cam_info.width))
            info_height = max(1, int(cam_info.height))
            scale_x = width / float(info_width)
            scale_y = height / float(info_height)
            fx = float(cam_info.k[0]) * scale_x
            fy = float(cam_info.k[4]) * scale_y
            cx = float(cam_info.k[2]) * scale_x
            cy = float(cam_info.k[5]) * scale_y
            if fx <= 0.0 or fy <= 0.0:
                diagnostics['reason'] = f'invalid camera intrinsics fx={fx}, fy={fy}'
                return None
            diagnostics['_kept_rows'] = rows
            diagnostics['_kept_cols'] = cols
            diagnostics['_depth_shape'] = (height, width)

            x_points = (cols.astype(np.float32) - cx) * ranges / fx
            y_points = (rows.astype(np.float32) - cy) * ranges / fy
            x = float(np.median(x_points))
            y = float(np.median(y_points))
            z = float(np.median(ranges))

            center_u = int(np.median(cols) * self.target_w / width)
            center_v = int(np.median(rows) * self.target_h / height)
            return x, y, z, center_u, center_v, int(rows.size)
        except Exception as exc:
            diagnostics['reason'] = f'segmented depth estimate raised: {exc}'
            self.get_logger().warning(
                f'Could not estimate target from segmented depth: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

    def _read_depth_at_pixel(self, depth_msg: Image, u: int, v: int) -> Optional[float]:
        try:
            arr = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if arr.ndim < 2 or not (0 <= u < arr.shape[1] and 0 <= v < arr.shape[0]):
                return None

            radius = self.depth_sample_radius_px
            x0, x1 = max(0, u - radius), min(arr.shape[1], u + radius + 1)
            y0, y1 = max(0, v - radius), min(arr.shape[0], v + radius + 1)
            samples = arr[y0:y1, x0:x1].astype(np.float32, copy=False)
            if arr.dtype == np.uint16:
                samples = samples * 0.001
            valid = samples[np.isfinite(samples) & (samples > 0.1)]
            if valid.size == 0:
                return None
            return float(np.median(valid))
        except Exception as exc:
            self.get_logger().warning(
                f'Could not read aligned depth near pixel ({u}, {v}): {exc}',
                throttle_duration_sec=5.0,
            )
            return None

    def run_model_inference(
        self,
        image_np,
        caption,
    ):
        if self.remote_inference_url:
            prompt, generated_text, points = self._remote_point(image_np, caption)
        else:
            prompt, generated_text, points = self._local_point(image_np, caption)
        h, w = image_np.shape[:2]
        candidates = [
            (int(object_id), int(image_index), float(x), float(y))
            for object_id, image_index, x, y in points
            if int(image_index) == 0
        ]
        self._last_inference_info = {
            'raw_points': len(points),
            'image0_points': len(candidates),
            'generated_text': generated_text[:200],
        }
        if self.point_selection == 'closest_to_center':
            candidates.sort(
                key=lambda point: (point[2] - w / 2.0) ** 2
                + (point[3] - h / 2.0) ** 2
            )
        candidates = candidates[:self.max_points]
        radius = 12
        detections = [
            {
                "label": caption,
                # MolmoPoint returns constrained point tokens, not calibrated
                # detector logits. Keep 1.0 solely for the legacy JSON schema.
                "score": 1.0,
                "backend": "molmopoint",
                "object_id": object_id,
                "point": [
                    int(np.clip(round(x), 0, w - 1)),
                    int(np.clip(round(y), 0, h - 1)),
                ],
                "bbox": [
                    max(0, int(round(x)) - radius),
                    max(0, int(round(y)) - radius),
                    min(w, int(round(x)) + radius),
                    min(h, int(round(y)) + radius),
                ],
            }
            for object_id, _image_index, x, y in candidates
        ]
        self.get_logger().info(
            f'MolmoPoint prompt="{prompt}" returned {len(detections)} point(s): '
            f'{generated_text[:200]}'
        )
        return detections

    def _local_point(self, image_np, caption):
        """Run MolmoPoint in-process; return (prompt, generated_text, points)."""
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_pil = PILImage.fromarray(image_rgb)
        prompt = caption.strip()
        if not prompt.lower().startswith(('point ', 'locate ', 'show ')):
            prompt = f'Point to {prompt}'
        messages = [{
            'role': 'user',
            'content': [
                {'type': 'text', 'text': prompt},
                {'type': 'image', 'image': image_pil},
            ],
        }]
        model_inputs = self.processor.apply_chat_template(
            messages,
            tokenize=True,
            add_generation_prompt=True,
            return_tensors='pt',
            return_dict=True,
            padding=True,
            return_pointing_metadata=True,
        )
        metadata = model_inputs.pop('metadata')
        model_inputs = {
            key: (
                value.to(self.model_device, dtype=self.inference_dtype)
                if self.torch.is_floating_point(value)
                else value.to(self.model_device)
            )
            for key, value in model_inputs.items()
        }

        autocast_enabled = self.model_device.type == 'cuda'
        with self.torch.inference_mode(), self.torch.autocast(
            device_type=self.model_device.type,
            dtype=self.inference_dtype,
            enabled=autocast_enabled,
        ):
            output = self.model.generate(
                **model_inputs,
                logits_processor=self.model.build_logit_processor_from_inputs(
                    model_inputs
                ),
                max_new_tokens=self.max_new_tokens,
                do_sample=False,
            )
        generated_tokens = output[:, model_inputs['input_ids'].shape[1]:]
        if hasattr(self.processor, 'post_process_image_text_to_text'):
            generated_text = self.processor.post_process_image_text_to_text(
                generated_tokens,
                skip_special_tokens=False,
                clean_up_tokenization_spaces=False,
            )[0]
        else:
            generated_text = self.processor.decode(
                generated_tokens[0],
                skip_special_tokens=False,
                clean_up_tokenization_spaces=False,
            )
        points = self.model.extract_image_points(
            generated_text,
            metadata['token_pooling'],
            metadata['subpatch_mapping'],
            metadata['image_sizes'],
        )
        return prompt, generated_text, points

    def run_segmentation(self, image_np, point):
        if not self.enable_segmentation:
            return [], []
        if self.remote_inference_url:
            return self._remote_segmentation(image_np, point)
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        self.sam2_model.set_image(image_rgb)
        masks, scores, _ = self.sam2_model.predict(
            point_coords=np.asarray([point], dtype=np.float32),
            point_labels=np.asarray([1], dtype=np.int32),
            multimask_output=True,
        )
        if masks is None or len(masks) == 0:
            return [], []
        best = int(np.argmax(scores))
        return masks[best:best + 1], scores[best:best + 1]

    # ------------------------------------------------------------------
    # Remote inference (parallel_prompts/CONTRACT.md). Stdlib only.

    def _remote_call(self, path: str, payload: Optional[dict] = None,
                     timeout: Optional[float] = None) -> dict:
        url = self.remote_inference_url.rstrip('/') + path
        data = None if payload is None else json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(
            url, data=data, method='GET' if data is None else 'POST',
            headers={} if data is None else {'Content-Type': 'application/json'},
        )
        # The server is on a private address; never route it through an
        # http_proxy from the environment.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(request, timeout=timeout or self.remote_timeout_sec) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            body = exc.read()[:300].decode('utf-8', 'replace')
            raise RuntimeError(f'{url} returned HTTP {exc.code}: {body}') from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'{url} unreachable: {exc.reason}') from exc

    def _check_remote_health(self) -> None:
        url = self.remote_inference_url
        try:
            health = self._remote_call('/health', timeout=5.0)
        except Exception as exc:
            self.get_logger().error(
                f'Remote inference server not reachable at startup: {exc}. '
                'Cycles will report inference_failed until it responds.'
            )
            return
        self.get_logger().info(f'Remote inference server {url} health: {health}')
        if not health.get('ok') or not health.get('molmopoint'):
            self.get_logger().error(
                f'Remote server {url} reports MolmoPoint not ready: {health}')
        if not health.get('sam2'):
            self.get_logger().error(
                f'Remote server {url} reports SAM2 not loaded; /segment will '
                'return no mask and no pose will be published.')

    @staticmethod
    def _encode_jpeg_b64(image_np) -> str:
        ok, encoded = cv2.imencode(
            '.jpg', image_np, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        return base64.b64encode(encoded.tobytes()).decode('ascii')

    def _remote_point(self, image_np, caption):
        """POST /point; return (prompt, generated_text, points) like _local_point."""
        response = self._remote_call('/point', {
            'image_jpeg_b64': self._encode_jpeg_b64(image_np),
            'caption': caption,
            'max_new_tokens': self.max_new_tokens,
        })
        h, w = image_np.shape[:2]
        image_size = response.get('image_size')
        if image_size and list(image_size) != [w, h]:
            raise RuntimeError(
                f'/point image_size {image_size} does not match sent frame {[w, h]}')
        return (
            response.get('prompt', caption),
            response.get('generated_text', ''),
            response.get('points') or [],
        )

    def _remote_segmentation(self, image_np, point):
        """POST /segment; return (masks, scores) shaped like the local SAM2 path."""
        response = self._remote_call('/segment', {
            'image_jpeg_b64': self._encode_jpeg_b64(image_np),
            'point': [int(point[0]), int(point[1])],
        })
        self._last_segment_info = {
            'sam2_score': response.get('score'),
            'segment_server_sec': response.get('elapsed_sec'),
        }
        if not response.get('mask_png_b64'):
            return [], []
        mask = cv2.imdecode(
            np.frombuffer(base64.b64decode(response['mask_png_b64']), np.uint8),
            cv2.IMREAD_GRAYSCALE,
        )
        if mask is None:
            raise RuntimeError('/segment returned an undecodable mask PNG')
        h, w = image_np.shape[:2]
        if mask.shape != (h, w):
            raise RuntimeError(
                f'/segment mask shape {mask.shape} does not match frame {(h, w)}')
        score = response.get('score')
        return (mask > 127)[None], np.asarray(
            [0.0 if score is None else score], dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = MolmoPointNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
