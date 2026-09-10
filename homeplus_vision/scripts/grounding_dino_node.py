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

Outputs:
- /molmopoint/detections (std_msgs/String) JSON list of pointed targets
- /molmopoint/debug_image (sensor_msgs/Image) annotated image
- /molmopoint_pose_world (geometry_msgs/PoseStamped) 3D target in world_frame
- /molmopoint/point (std_msgs/String) point plus optional camera/world 3D data
"""

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import os
import sys
import threading
import yaml
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple
from PIL import Image as PILImage


class MolmoPointNode(Node):
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
        self.declare_parameter('mask_depth_near_quantile', 0.25)
        self.declare_parameter('mask_depth_cluster_margin_m', 0.04)
        self.declare_parameter('mask_depth_min_points', 40)
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
        self.mask_pub = self.create_publisher(
            Image, str(self.get_parameter('mask_topic').value), 10
        )
        self.mask_center_pub = self.create_publisher(
            String, str(self.get_parameter('mask_center_topic').value), 10
        )

        # VLM inference may take longer than the nominal timer period. Keep TF
        # and sensor callbacks in callback groups that can run concurrently so
        # a long model call cannot leave the world transform several seconds
        # behind the robot's completed motion.
        self._sensor_callback_group = ReentrantCallbackGroup()
        self._inference_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_lock = threading.Lock()
        self._config_lock = threading.Lock()
        self._robot_moving = False
        self._motion_finished_stamp_ns = 0
        self._last_processed_color_stamp_ns = -1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self.camera_info = None
        self._have_logged_color = False
        self._have_logged_depth = False
        self._have_logged_camera_info = False

        # RealSense image/depth/camera-info topics are sensor streams and usually
        # publish with best-effort QoS. Match that profile so subscriptions
        # connect reliably instead of silently receiving nothing.
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
                Bool, self.motion_status_topic, self.motion_status_callback, 10,
                callback_group=self._sensor_callback_group,
            )

        self.tasks = self._load_task_registry(self.task_registry_path)
        if self.task_id not in self.tasks:
            self.get_logger().warn(f"Invalid task_id {self.task_id}, defaulting to 1")
            self.task_id = 1
        self.task = self.tasks[self.task_id]
        if not self.caption.strip():
            self.caption = self.task["caption"]

        # Keep the model resident while a test client changes the prompt. This
        # is the same callback/inference path used during
        # normal operation, so interactive tests cannot drift from the ROS node.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Model setup. Transformers is imported lazily so ROS tools can inspect
        # this package even when the dedicated MolmoPoint environment is absent.
        self._configure_import_paths()
        self._init_molmopoint()

        if self.publish_poses:
            self._init_sam2()

        self.timer = self.create_timer(
            1.0 / max(self.inference_rate, 0.1),
            self.detect_loop,
            callback_group=self._inference_callback_group,
        )
        self.get_logger().info(
            f'MolmoPointNode initialized: model={self.model_name_or_path}, '
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
        """Apply the pointing prompt safely while the model remains resident."""
        caption = self.caption

        for parameter in parameters:
            if parameter.name == 'caption':
                if not isinstance(parameter.value, str):
                    return SetParametersResult(
                        successful=False,
                        reason='caption must be a string',
                    )
                caption = parameter.value.strip()
                if not caption:
                    return SetParametersResult(
                        successful=False,
                        reason='caption must contain at least one keyword',
                    )
        changed = caption != self.caption
        if changed:
            with self._config_lock:
                self.caption = caption
            self.get_logger().info(
                f'Updated MolmoPoint prompt: "{caption}"'
            )
        return SetParametersResult(successful=True)

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
                and frame_stamp_ns > self._motion_finished_stamp_ns
            )

    def detect_loop(self):
        with self._state_lock:
            color = self.latest_color
            header = self.latest_color_header
            depth = self.latest_depth
            camera_info = self.camera_info
            robot_moving = self._robot_moving
            motion_finished_stamp_ns = self._motion_finished_stamp_ns

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
            return

        frame_stamp_ns = self._header_stamp_ns(header)
        if self.pause_during_motion and (
            robot_moving or frame_stamp_ns <= motion_finished_stamp_ns
        ):
            return
        with self._state_lock:
            if frame_stamp_ns <= self._last_processed_color_stamp_ns:
                return
            self._last_processed_color_stamp_ns = frame_stamp_ns

        self.get_logger().info("Running detection loop")
        # 1. Resize Color for Inference
        img_resized = cv2.resize(color, (self.target_w, self.target_h))

        with self._config_lock:
            caption = self.caption

        try:
            detections = self.run_model_inference(img_resized, caption)
        except Exception as exc:
            self.get_logger().error(f'MolmoPoint inference failed: {exc}')
            return
        if not self._frame_is_publishable(frame_stamp_ns):
            self.get_logger().info(
                'Discarding detection captured before robot motion completed'
            )
            return
        debug_img = img_resized.copy()
        self.detections_pub.publish(String(data=json.dumps(detections)))

        for d in detections:
            xmin, ymin, xmax, ymax = map(int, d["bbox"])
            xmin = max(0, min(xmin, self.target_w - 1))
            xmax = max(0, min(xmax, self.target_w))
            ymin = max(0, min(ymin, self.target_h - 1))
            ymax = max(0, min(ymax, self.target_h))
            if xmax <= xmin or ymax <= ymin:
                continue

            center_u = int(d['point'][0])
            center_v = int(d['point'][1])
            full_mask = None

            if self.enable_segmentation:
                masks, _ = self.run_segmentation(
                    img_resized,
                    (center_u, center_v),
                )
                if masks is not None and len(masks) > 0:
                    full_mask = masks[0].astype(np.uint8) * 255
                    mask_ys, mask_xs = np.where(full_mask > 0)
                    if mask_xs.size > 0:
                        center_u = int(np.mean(mask_xs))
                        center_v = int(np.mean(mask_ys))
                        mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding='mono8')
                        mask_msg.header = header
                        self.mask_pub.publish(mask_msg)

            mask_center_payload = {
                "label": d["label"],
                "score": float(d["score"]),
                "backend": "molmopoint",
                "model_point": {"u": int(d['point'][0]), "v": int(d['point'][1])},
                "pixel": {"u": center_u, "v": center_v},
            }

            point_3d = None
            depth_source = None
            depth_point_count = 0
            if self.publish_poses and depth is not None:
                if full_mask is not None:
                    mask_estimate = self._estimate_point_from_mask_depth(
                        depth, full_mask, camera_info
                    )
                    if mask_estimate is not None:
                        X, Y, Z, center_u, center_v, depth_point_count = mask_estimate
                        point_3d = (X, Y, Z)
                        depth_source = "segmented_depth_cloud"

                if point_3d is None:
                    depth_h, depth_w = depth.height, depth.width
                    u_depth = int(center_u * (depth_w / self.target_w))
                    v_depth = int(center_v * (depth_h / self.target_h))
                    z_center = self._read_depth_at_pixel(depth, u_depth, v_depth)
                    if z_center is not None:
                        point_3d = self._backproject_pixel_to_3d(
                            center_u, center_v, z_center, camera_info
                        )
                        depth_source = "center_patch_fallback"

            mask_center_payload["pixel"] = {"u": center_u, "v": center_v}

            if self.publish_poses and point_3d is not None:
                X, Y, Z = point_3d
                camera_frame = self.camera_frame or camera_info.header.frame_id

                pose_cam = PoseStamped()
                pose_cam.header = header
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

                pose_world = self._transform_pose_to_world(pose_cam)
                if pose_world is not None:
                    self.pose_pub.publish(pose_world)
                    pw = pose_world.pose.position
                    mask_center_payload["point_3d_world"] = {"x": pw.x, "y": pw.y, "z": pw.z}
                    self.get_logger().info(
                        f"Target {d['label']} in {self.world_frame}: "
                        f"x={pw.x:.2f}m, y={pw.y:.2f}m, z={pw.z:.2f}m"
                    )
            elif self.publish_poses:
                self.get_logger().info(
                    f"Target {d['label']} detected at pixel=({center_u}, {center_v}), "
                    f"but no valid depth was available"
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

        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

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

    def _transform_pose_to_world(self, pose_cam: PoseStamped) -> Optional[PoseStamped]:
        src_frame = pose_cam.header.frame_id
        try:
            # VLM inference can lag camera stamps by several seconds, while this
            # robot's TF tree is effectively static unless joint states change.
            # Use the latest available transform to avoid extrapolation errors.
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                src_frame,
                Time(),
                timeout=self._tf_timeout,
            )
        except Exception as e:
            self.get_logger().warn(
                f'Could not transform pose from {src_frame} to {self.world_frame}: {e}. '
                '2D detections still work. For hand_camera_* frames, make sure '
                'the robot TF chain '
                f'is connected by publishing /joint_states (for example via the Arduino bridge or '
                f'PUBLISH_JOINT_STATES=true bash scripts/run_robot_tf.sh).',
                throttle_duration_sec=2.0,
            )
            return None
        pose_world = tf2_geometry_msgs.do_transform_pose_stamped(
            pose_cam, tf_msg)
        pose_world.header.frame_id = self.world_frame
        pose_world.header.stamp = tf_msg.header.stamp
        return pose_world

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
    ) -> Optional[Tuple[float, float, float, int, int, int]]:
        """
        Estimate a robust visible-object centroid from the segmented depth.

        Cup openings frequently expose the table or inner wall at the mask
        center. Using a single center patch therefore biases the target toward
        the support surface. This method uses the complete segmented point set
        and keeps the coherent near-depth portion before taking its median.
        """
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
            if rows.size < self.mask_depth_min_points:
                return None

            ranges = depth[rows, cols]
            near_depth = float(
                np.quantile(ranges, self.mask_depth_near_quantile)
            )
            keep = ranges <= near_depth + self.mask_depth_cluster_margin_m
            rows = rows[keep]
            cols = cols[keep]
            ranges = ranges[keep]
            if rows.size < self.mask_depth_min_points:
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
                return None

            x_points = (cols.astype(np.float32) - cx) * ranges / fx
            y_points = (rows.astype(np.float32) - cy) * ranges / fy
            x = float(np.median(x_points))
            y = float(np.median(y_points))
            z = float(np.median(ranges))

            center_u = int(np.median(cols) * self.target_w / width)
            center_v = int(np.median(rows) * self.target_h / height)
            return x, y, z, center_u, center_v, int(rows.size)
        except Exception as exc:
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
        h, w = image_np.shape[:2]
        candidates = [
            (int(object_id), int(image_index), float(x), float(y))
            for object_id, image_index, x, y in points
            if int(image_index) == 0
        ]
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

    def run_segmentation(self, image_np, point):
        if not self.enable_segmentation:
            return [], []
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
