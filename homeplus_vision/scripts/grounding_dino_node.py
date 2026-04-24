#!/usr/bin/env python3

"""
ros2 run homeplus_vision grounding_dino_node.py

Parameters (declared):
- device: 'cpu' or 'cuda:0'
- confidence_threshold: float
- publish_poses: bool
- image_topic: topic for color images (default: /camera/camera/color/image_raw)
- depth_topic: topic for aligned depth images (default: /camera/camera/depth/image_rect_raw)
- camera_info_topic: topic for camera info (default: /camera/camera/color/camera_info)
- inference_rate: Hz for running inference (default 2.0)

Outputs:
- /gdino/detections (std_msgs/String) JSON list of detections: [{label, score, bbox: [xmin,ymin,xmax,ymax]}]
- /gdino_debug_image (sensor_msgs/Image) annotated image
- /gdino_pose_camera (geometry_msgs/PoseStamped) when publish_poses is true (one per detection)
- /gdino/mask_center (std_msgs/String) JSON payload with mask center pixel and optional 3D point
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import os
import sys
from pathlib import Path
from typing import Optional, Tuple
from PIL import Image as PILImage

class GroundingDinoNode(Node):
    def __init__(self):
        os.environ["HYDRA_FULL_ERROR"] = "1"
        super().__init__('grounding_dino')

        workspace_root = Path(__file__).resolve().parents[2]
        default_grounded_sam_root = workspace_root / "Grounded-SAM-2"

        # Parameters
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('publish_poses', True)
        self.declare_parameter('caption', 'coffee cup. coffee machine. mug.')
        self.declare_parameter('grounded_sam_root', str(default_grounded_sam_root))
        self.declare_parameter('grounding_dino_config_path', '')
        self.declare_parameter('grounding_dino_weights_path', '')
        self.declare_parameter('sam2_config_path', '')
        self.declare_parameter('sam2_checkpoint_path', '')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('inference_rate', 2.0)

        self.device = self.get_parameter('device').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.publish_poses = bool(self.get_parameter('publish_poses').value)
        self.caption = str(self.get_parameter('caption').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.inference_rate = float(self.get_parameter('inference_rate').value)
        self.grounded_sam_root = Path(str(self.get_parameter('grounded_sam_root').value)).expanduser()
        self.enable_segmentation = False

        # Target resolution for inference
        self.target_w = 640
        self.target_h = 480

        # Publishers
        self.debug_img_pub = self.create_publisher(Image, '/gdino_debug_image', 10)
        self.detections_pub = self.create_publisher(String, '/gdino/detections', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/gdino_pose_camera', 10)
        self.mask_pub = self.create_publisher(Image, '/object_mask', 10)
        self.mask_center_pub = self.create_publisher(String, '/gdino/mask_center', 10)

        # Subscriptions
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self.camera_info = None

        self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 5)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 5)

        # Models Setup
        self._configure_import_paths()

        from groundingdino.util.inference import load_model
        from groundingdino.datasets.transforms import Compose, RandomResize, ToTensor, Normalize

        config_path = self._resolve_path(
            "grounding_dino_config_path",
            [
                self.grounded_sam_root / "grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py",
                self.grounded_sam_root / "groundingdino/config/GroundingDINO_SwinT_OGC.py",
            ],
            required=True,
        )
        weights_path = self._resolve_path(
            "grounding_dino_weights_path",
            [
                self.grounded_sam_root / "gdino_checkpoints/groundingdino_swint_ogc.pth",
                self.grounded_sam_root / "weights/groundingdino_swint_ogc.pth",
            ],
            required=True,
        )

        self.model = load_model(str(config_path), str(weights_path))
        self.model.to(self.device)
        self.predict_fn = self._load_predict_fn()

        if self.publish_poses:
            self._init_sam2()

        self.transform = Compose([
            RandomResize([800], max_size=1333),
            ToTensor(),
            Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

        self.timer = self.create_timer(1.0 / self.inference_rate, self.detect_loop)
        self.get_logger().info(
            f"GroundingDinoNode initialized: root={self.grounded_sam_root}, "
            f"segmentation={'on' if self.enable_segmentation else 'off'}"
        )

    def _configure_import_paths(self) -> None:
        candidates = [
            self.grounded_sam_root,
            self.grounded_sam_root / "grounding_dino",
        ]
        for candidate in candidates:
            if candidate.is_dir():
                candidate_str = str(candidate)
                if candidate_str not in sys.path:
                    sys.path.insert(0, candidate_str)

        sam2_config_root = self.grounded_sam_root / "sam2"
        if sam2_config_root.is_dir():
            os.environ["HYDRA_CONFIG_PATH"] = str(sam2_config_root / "configs")

    def _resolve_path(self, parameter_name: str, candidates, required: bool = False) -> Optional[Path]:
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
            raise FileNotFoundError(f"Required file for {parameter_name} not found. Tried:\n{tried}")
        return None

    def _load_predict_fn(self):
        from groundingdino.util.inference import predict
        return predict

    def _init_sam2(self) -> None:
        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as exc:
            self.get_logger().warn(f"SAM2 import failed; disabling segmentation/pose output: {exc}")
            return

        sam2_config = self._resolve_path(
            "sam2_config_path",
            [
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
                "SAM2 config/checkpoint not found; running detections without segmentation or 3D pose output."
            )
            return

        sam2_model = build_sam2(str(sam2_config), str(sam2_checkpoint), device=self.device)
        self.sam2_model = SAM2ImagePredictor(sam2_model)
        self.enable_segmentation = True

    def image_callback(self, msg: Image):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_color_header = msg.header

    def depth_callback(self, msg: Image):
        self.latest_depth = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def detect_loop(self):
        if self.latest_color is None or self.camera_info is None:
            return

        # 1. Resize Color for Inference
        img_resized = cv2.resize(self.latest_color, (self.target_w, self.target_h))
        header = self.latest_color_header

        detections = self.run_model_inference(img_resized, self.caption)
        filtered = [d for d in detections if d.get('score', 0.0) >= self.confidence_threshold]
        debug_img = img_resized.copy()
        self.detections_pub.publish(String(data=json.dumps(filtered)))

        for d in filtered:
            xmin, ymin, xmax, ymax = map(int, d["bbox"])
            xmin = max(0, min(xmin, self.target_w - 1))
            xmax = max(0, min(xmax, self.target_w))
            ymin = max(0, min(ymin, self.target_h - 1))
            ymax = max(0, min(ymax, self.target_h))
            if xmax <= xmin or ymax <= ymin:
                continue

            center_u = int((xmin + xmax) / 2)
            center_v = int((ymin + ymax) / 2)
            full_mask = None

            if self.enable_segmentation:
                crop = img_resized[ymin:ymax, xmin:xmax]
                refined = self.run_model_inference(crop, "handle")
                masks, _ = self.run_segmentation(crop, refined if refined else [d])
                if masks is not None and len(masks) > 0:
                    full_mask = np.zeros((self.target_h, self.target_w), dtype=np.uint8)
                    full_mask[ymin:ymax, xmin:xmax] = masks[0].astype(np.uint8) * 255
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
                "pixel": {"u": center_u, "v": center_v},
            }

            z_center = None
            if self.publish_poses and self.latest_depth is not None:
                depth_h, depth_w = self.latest_depth.height, self.latest_depth.width
                u_depth = int(center_u * (depth_w / self.target_w))
                v_depth = int(center_v * (depth_h / self.target_h))
                z_center = self._read_depth_at_pixel(self.latest_depth, u_depth, v_depth)

            if self.publish_poses and z_center is not None:
                X, Y, Z = self._backproject_pixel_to_3d(center_u, center_v, z_center, self.camera_info)

                pose = PoseStamped()
                pose.header = header
                pose.header.frame_id = self.camera_info.header.frame_id
                pose.pose.position.x = X
                pose.pose.position.y = Y
                pose.pose.position.z = Z
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)

                mask_center_payload["point_3d"] = {"x": X, "y": Y, "z": Z}
                self.get_logger().info(f"Target {d['label']}: X={X:.2f}m, Y={Y:.2f}m, Z={Z:.2f}m")

            self.mask_center_pub.publish(String(data=json.dumps(mask_center_payload)))

            # Draw Debug
            cv2.rectangle(debug_img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.circle(debug_img, (center_u, center_v), 5, (255, 0, 0), -1)
            cv2.putText(
                debug_img,
                f"{d['label']} {d['score']:.2f}",
                (xmin, max(20, ymin - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

    #scales xyz to resized image
    def _backproject_pixel_to_3d(self, u: int, v: int, z: float, cam_info: CameraInfo) -> Tuple[float, float, float]:
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

    def _read_depth_at_pixel(self, depth_msg: Image, u: int, v: int) -> Optional[float]:
        try:
            arr = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            z_raw = arr[v, u]
            # Convert 16-bit mm to meters
            z = float(z_raw) / 1000.0 if arr.dtype == np.uint16 else float(z_raw)
            return z if z > 0.1 else None
        except:
            return None

    def run_model_inference(self, image_np, caption):
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_pil = PILImage.fromarray(image_rgb)
        image_tensor, _ = self.transform(image_pil, None)
        boxes, logits, phrases = self.predict_fn(
            model=self.model,
            image=image_tensor.to(self.device),
            caption=caption,
            box_threshold=self.confidence_threshold,
            text_threshold=0.25,
        )
        h, w = image_np.shape[:2]
        return [
            {
                "label": p,
                "score": float(l),
                "bbox": [
                    int((b[0] - b[2] / 2) * w),
                    int((b[1] - b[3] / 2) * h),
                    int((b[0] + b[2] / 2) * w),
                    int((b[1] + b[3] / 2) * h),
                ],
            }
            for b, l, p in zip(boxes, logits, phrases)
        ]

    def run_segmentation(self, image_np, detections):
        if not self.enable_segmentation:
            return [], []
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        self.sam2_model.set_image(image_rgb)
        boxes = np.array([d["bbox"] for d in detections])
        if len(boxes) == 0: return [], []
        masks, scores, _ = self.sam2_model.predict(box=boxes, multimask_output=False)
        return masks, scores

def main(args=None):
    rclpy.init(args=args)
    node = GroundingDinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
