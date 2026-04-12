#!/usr/bin/env python3

"""
ros2 run homeplus_vision grounding_dino_node.py

Parameters (declared):
- device: 'cpu' or 'cuda:0'
- confidence_threshold: float
- publish_poses: bool
- image_topic: topic for color images (default: /camera/camera/color/image_raw)
- depth_topic: topic for aligned depth images (default: /camera/aligned_depth_to_color/image_raw)
- camera_info_topic: topic for camera info (default: /camera/camera/color/camera_info)
- inference_rate: Hz for running inference (default 2.0)

Outputs:
- /gdino/detections (std_msgs/String) JSON list of detections: [{label, score, bbox: [xmin,ymin,xmax,ymax]}]
- /gdino_debug_image (sensor_msgs/Image) annotated image
- /gdino_pose_camera (geometry_msgs/PoseStamped) when publish_poses is true (one per detection)

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import time
from typing import Optional, Tuple
from PIL import Image as PILImage



class GroundingDinoNode(Node):
    def __init__(self):
        import os

        os.environ["HYDRA_FULL_ERROR"] = "1"
        os.environ["HYDRA_CONFIG_PATH"] = "/home/homeplus/sam2/sam2/configs"
        super().__init__('grounding_dino')

        # Parameters
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('publish_poses', False)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('inference_rate', 2.0)
        self.declare_parameter('debug_image_topic', '/gdino_debug_image')
        self.declare_parameter('detections_topic', '/gdino/detections')
        self.declare_parameter('pose_topic', '/gdino_pose_camera')

        self.device = self.get_parameter('device').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.publish_poses = bool(self.get_parameter('publish_poses').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.inference_rate = 1.0#float(self.get_parameter('inference_rate').value)
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=5)

        
        self.debug_img_pub = self.create_publisher(Image, self.debug_image_topic, qos)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.mask_pub = self.create_publisher(Image, '/object_mask', 10)

        # Subscriptions
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self.camera_info = None

        self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        # Try depth subscription
        try:
            self.create_subscription(Image, self.depth_topic, self.depth_callback, 5)
        except Exception:
            self.get_logger().warning(f'Could not subscribe to depth topic {self.depth_topic}; continuing without depth')

        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 5)

        # Timer for inference loop
        period = 1.0 / max(0.0001, self.inference_rate)
        self.timer = self.create_timer(period, self.detect_loop)

        self.get_logger().info(f'GroundingDinoNode initialized (device={self.device}, threshold={self.confidence_threshold}, publish_poses={self.publish_poses})')

        # Initialize dino
        from groundingdino.util.inference import load_model, load_image, predict, annotate
        from groundingdino.datasets.transforms import Compose, RandomResize, ToTensor, Normalize
       
        import os

        base = os.path.expanduser("~/GroundingDINO")

        config_path = os.path.join(base, "groundingdino/config/GroundingDINO_SwinT_OGC.py")
        weights_path = os.path.join(base, "weights/groundingdino_swint_ogc.pth")
        self.model = load_model(
            config_path,
            weights_path
        )
        self.model.to(self.device)
        self.caption = "coffee cup. aluminum coffee machine. aluminum shelf."

        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        sam2_model = build_sam2(
            "configs/sam2/sam2_hiera_t.yaml",
    "/home/homeplus/sam2/checkpoints/sam2_hiera_tiny.pt",
    device="cpu"
        )
        # working: 
        # sam2/sam2_hiera_l.yaml
        # sam2/checkpoints/sam2_hiera_large.pt
        
        self.sam2_model = SAM2ImagePredictor(sam2_model)
        self.transform = Compose([
            RandomResize([800], max_size=1333),
            ToTensor(),
            Normalize([0.485, 0.456, 0.406],
                    [0.229, 0.224, 0.225])
        ])
        self.get_logger().info('GroundingDINO and SAM2 models loaded successfully')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_color = cv_image
            self.latest_color_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = msg
        except Exception as e:
            self.get_logger().error(f'Failed to store depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

 
    def detect_loop(self):
        if self.latest_color is None:
            # No image yet
            return
        self.get_logger().info('Running detection loop')
        img = self.latest_color
        img = cv2.resize(img, (640, 480)) #resize to smaller for ram
        header = self.latest_color_header

        detections = self.run_model_inference(img, self.caption)

        # Filter by confidence 
        filtered = [d for d in detections if d.get('score', 0.0) >= self.confidence_threshold]

        debug_img = img.copy()

        for d in filtered:
            xmin, ymin, xmax, ymax = map(int, d["bbox"])

            # -----------------------------
            # clamp bbox
            # -----------------------------
            h_img, w_img, _ = img.shape
            xmin = max(0, xmin)
            ymin = max(0, ymin)
            xmax = min(w_img, xmax)
            ymax = min(h_img, ymax)

            if xmax <= xmin or ymax <= ymin:
                continue

            # -----------------------------
            # crop
            # -----------------------------
            crop = img[ymin:ymax, xmin:xmax]

            # -----------------------------
            # refine with DINO (optional)
            # -----------------------------
            refined = self.refine_detection(crop, d["label"])
            if refined is None or len(refined) == 0:
                continue

            # -----------------------------
            # SAM2 on crop
            # -----------------------------
            masks, _ = self.run_segmentation(crop, refined)

            if masks is None or len(masks) == 0:
                continue

            crop_mask = masks[0].astype(np.uint8)

            # -----------------------------
            # convert crop mask → full image mask
            # -----------------------------
            full_mask = np.zeros(img.shape[:2], dtype=np.uint8)
            full_mask[ymin:ymax, xmin:xmax] = crop_mask * 255

            # -----------------------------
            # publish mask
            # -----------------------------
            mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding='mono8')
            mask_msg.header = header
            self.mask_pub.publish(mask_msg)

            # -----------------------------
            # debug overlay
            # -----------------------------
            cv2.rectangle(debug_img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(
                debug_img,
                f"{d['label']}:{d['score']:.2f}",
                (xmin, max(ymin - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

            # -----------------------------
            # 3D pose (optional)
            # -----------------------------
            if self.publish_poses and self.latest_depth is not None and self.camera_info is not None:

                cx = int((xmin + xmax) / 2)
                cy = int((ymin + ymax) / 2)

                z = self._read_depth_at_pixel(self.latest_depth, cx, cy)

                if z is not None and z > 0.0:
                    X, Y, Z = self._backproject_pixel_to_3d(cx, cy, z, self.camera_info)

                    pose = PoseStamped()
                    pose.header = header
                    pose.header.frame_id = (
                        self.camera_info.header.frame_id
                        if self.camera_info and self.camera_info.header.frame_id
                        else "camera_color_optical_frame"
                    )

                    pose.pose.position.x = float(X)
                    pose.pose.position.y = float(Y)
                    pose.pose.position.z = float(Z)
                    pose.pose.orientation.w = 1.0

                    self.pose_pub.publish(pose)

        # -----------------------------
        # publish detections JSON
        # -----------------------------
        try:
            payload = json.dumps(filtered)
            self.detections_pub.publish(String(data=payload))
        except Exception as e:
            self.get_logger().error(f"Failed to publish detections JSON: {e}")

        # -----------------------------
        # publish debug image
        # -----------------------------
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = header
            self.debug_img_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")


    def run_model_inference(self, image_np, caption):
        """
        Run grounding dino inference on image

        Example return value:
        [
          {"label": "cup", "score": 0.95, "bbox": [100, 120, 200, 240]},
          {"label": "bottle", "score": 0.80, "bbox": [300, 100, 380, 260]},
        ]
        """
        import cv2
        from groundingdino.util.inference import predict

        # Convert BGR → RGB
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        # Convert to PIL
        image_pil = PILImage.fromarray(image_rgb)

        # Apply transform
        image_tensor, _ = self.transform(image_pil, None)

        # Move to device
        image_tensor = image_tensor.to(self.device)

        # Run Grounding DINO
        boxes, logits, phrases = predict(
            model=self.model,
            image=image_tensor,
            caption=caption,
            box_threshold=self.confidence_threshold,
            text_threshold=0.25
        )

        h, w, _ = image_np.shape

        detections = []

        for box, logit, phrase in zip(boxes, logits, phrases):
            cx, cy, bw, bh = box.cpu().numpy()

            xmin = int((cx - bw/2) * w)
            ymin = int((cy - bh/2) * h)
            xmax = int((cx + bw/2) * w)
            ymax = int((cy + bh/2) * h)

            detections.append({
                "label": phrase,
                "score": float(logit.item()),
                "bbox": [xmin, ymin, xmax, ymax]
            })

        return detections

    def refine_detection(self, crop, label):
        """
        Given detected object, run second stage to detect specific details
        """
        if "machine" in label:
            caption = "aluminum U-shaped handle."
        elif "cup" in label:
            caption = "curved cup handle"
        else:
            return None

        return self.run_model_inference(crop, caption)

    def run_segmentation(self, image_np, detections):
        """
        Run SAM2 segmentation on given image and list of detections (with xyxy boxes).
        Returns masks and scores.
        """
        # If SAM failed to load or is disabled, return empty results
        if self.sam2_model is None:
            return [], []
        import cv2

        # Convert BGR → RGB (SAM expects RGB)
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        self.sam2_model.set_image(image_rgb)

        boxes_xyxy = []
        for d in detections:
            xmin, ymin, xmax, ymax = d["bbox"]
            boxes_xyxy.append([xmin, ymin, xmax, ymax])

        if len(boxes_xyxy) == 0:
            return [], []

        boxes_xyxy = np.array(boxes_xyxy)

        masks, scores, logits = self.sam2_model.predict(
            box=boxes_xyxy,
            multimask_output=False
        )

        return masks, scores
    
    def _read_depth_at_pixel(self, depth_msg: Image, u: int, v: int) -> Optional[float]:
        """Read depth (in meters) from a ROS Image message at integer pixel (u, v).
        Supports common encodings: 32FC1 (meters) and 16UC1 (millimeters).
        Returns None on failure.
        """
        try:
            encoding = depth_msg.encoding if hasattr(depth_msg, 'encoding') else ''
            arr = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            # If 16UC1, convert mm->m
            if arr.dtype == np.uint16:
                z = float(arr[v, u]) / 1000.0
            else:
                z = float(arr[v, u])
            if np.isnan(z) or np.isinf(z) or z <= 0.0:
                # Attempt to search a small neighborhood for valid depth
                h, w = arr.shape
                radius = 3
                vals = []
                for yy in range(max(0, v-radius), min(h, v+radius+1)):
                    for xx in range(max(0, u-radius), min(w, u+radius+1)):
                        val = arr[yy, xx]
                        if arr.dtype == np.uint16:
                            val = float(val) / 1000.0
                        else:
                            val = float(val)
                        if val > 0 and not np.isnan(val) and not np.isinf(val):
                            vals.append(val)
                if vals:
                    return float(np.median(vals))
                return None
            return z
        except Exception as e:
            self.get_logger().debug(f'Failed to read depth at pixel: {e}')
            return None

    def _backproject_pixel_to_3d(self, u: int, v: int, z: float, cam_info: CameraInfo) -> Tuple[float, float, float]:
        """Backproject pixel (u,v) with depth z (meters) using CameraInfo to camera coordinates.
        Returns X,Y,Z in camera frame (meters).
        """
        # CameraInfo.k is row-major 3x3
        k = cam_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z
        return X, Y, Z


def main(args=None):
    rclpy.init(args=args)
    node = GroundingDinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
