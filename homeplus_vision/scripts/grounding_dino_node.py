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
- task_id: 1-6 to select targets for different parts of the pipeline
- world_frame: TF parent frame for published poses (default map); use odom, world, base_link, etc. as in your TF tree
- tf_lookup_timeout_sec: max wait for each TF lookup (default 0.5)

Outputs:
- /gdino/detections (std_msgs/String) JSON list of detections: [{label, score, bbox: [xmin,ymin,xmax,ymax]}]
- /gdino_debug_image (sensor_msgs/Image) annotated image
- /gdino_pose_world (geometry_msgs/PoseStamped) when publish_poses is true — 3D target in world_frame (TF2)
- /gdino/mask_center (std_msgs/String) JSON payload with mask center pixel and optional 3D point

"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import os
from typing import Optional, Tuple
from PIL import Image as PILImage
from groundingdino.util.inference import predict 
# from tf2_geometry_msgs import do_transform_pose

class GroundingDinoNode(Node):
    def __init__(self):
        os.environ["HYDRA_FULL_ERROR"] = "1"
        os.environ["HYDRA_CONFIG_PATH"] = "/home/homeplus/sam2/sam2/configs"
        super().__init__('grounding_dino')

        # Parameters
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('publish_poses', True)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('inference_rate', 2.0)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('tf_lookup_timeout_sec', 0.5)

        self.device = self.get_parameter('device').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.publish_poses = bool(self.get_parameter('publish_poses').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.inference_rate = float(self.get_parameter('inference_rate').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self._tf_timeout = Duration(seconds=float(self.get_parameter('tf_lookup_timeout_sec').value))

        # Target resolution for inference
        self.target_w = 640
        self.target_h = 480

        self.declare_parameter('task_id', 1)
        self.task_id = int(self.get_parameter('task_id').value)

        # Publishers
        self.debug_img_pub = self.create_publisher(Image, '/gdino_debug_image', 10)
        self.detections_pub = self.create_publisher(String, '/gdino/detections', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/gdino_pose_world', 10)
        self.mask_pub = self.create_publisher(Image, '/object_mask', 10)
        self.mask_center_pub = self.create_publisher(String, '/gdino/mask_center', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self.camera_info = None

        self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 5)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 5)

        # targets for different tasks (needed before model setup)
        self.tasks = {
            1: {
                "caption": "cup.",
                "refinement": "handle"
            },
            2: {
                "caption": "aluminum coffee machine.",
                "refinement": "button"
            },
            3: {
                "caption": "aluminum shelf.",
                "refinement": None
            },
            4: {
                "caption": "coffee cup. aluminum coffee machine.",
                "refinement": "handle"
            },
            5: {
                "caption": "coffee cup. aluminum shelf.",
                "refinement": None
            },
            6: {
                "caption": "coffee cup. aluminum coffee machine. aluminum shelf.",
                "refinement": "handle"
            }
        }
        if self.task_id not in self.tasks:
            self.get_logger().warn(f"Invalid task_id {self.task_id}, defaulting to 1")
            self.task_id = 1
        self.task = self.tasks[self.task_id]
        self.caption = self.task["caption"]

        # Models Setup
        from groundingdino.util.inference import load_model
        from groundingdino.datasets.transforms import Compose, RandomResize, ToTensor, Normalize
        
        base = os.path.expanduser("~/GroundingDINO")
        config_path = os.path.join(base, "groundingdino/config/GroundingDINO_SwinT_OGC.py")
        weights_path = os.path.join(base, "weights/groundingdino_swint_ogc.pth")
        
        self.model = load_model(config_path, weights_path)
        self.model.to(self.device)

        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        sam2_model = build_sam2(
            "configs/sam2/sam2_hiera_t.yaml",
            "/home/homeplus/sam2/checkpoints/sam2_hiera_tiny.pt",
            device=self.device
        )
        self.sam2_model = SAM2ImagePredictor(sam2_model)
        
        self.transform = Compose([
            RandomResize([800], max_size=1333),
            ToTensor(),
            Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

        self.timer = self.create_timer(1.0 / max(self.inference_rate, 0.1), self.detect_loop)
        self.get_logger().info(
            f'GroundingDinoNode initialized; target 3D poses published in frame "{self.world_frame}"'
        )

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
        self.get_logger().info("Running detection loop")
        # 1. Resize Color for Inference
        img_resized = cv2.resize(self.latest_color, (self.target_w, self.target_h))
        header = self.latest_color_header
        
        detections = self.run_model_inference(img_resized, self.caption)
        filtered = [d for d in detections if d.get('score', 0.0) >= self.confidence_threshold]
        debug_img = img_resized.copy()

        for d in filtered:
            xmin, ymin, xmax, ymax = map(int, d["bbox"])
            
            # Crop and Segment
            crop = img_resized[ymin:ymax, xmin:xmax]
            refinement_prompt = self.task.get("refinement")

            if refinement_prompt:
                refined = self.run_model_inference(crop, refinement_prompt)
            else:
                refined = [d]
            
            masks, _ = self.run_segmentation(crop, refined if refined else [d])
            if masks is None or len(masks) == 0: continue

            # Create Full Mask (at 640x480)
            full_mask = np.zeros((self.target_h, self.target_w), dtype=np.uint8)
            full_mask[ymin:ymax, xmin:xmax] = masks[0].astype(np.uint8) * 255

            # Calculate Center Pixel (u,v)
            mask_ys, mask_xs = np.where(full_mask > 0)
            if mask_xs.size == 0: continue
            center_u = int(np.mean(mask_xs))
            center_v = int(np.mean(mask_ys))

            # 2. Correct Depth Sampling (Map 640x480 pixel back to raw depth resolution)
            z_center = None
            if self.latest_depth is not None:
                # Calculate scale between 640x480 and the actual depth buffer
                depth_h, depth_w = self.latest_depth.height, self.latest_depth.width
                u_depth = int(center_u * (depth_w / self.target_w))
                v_depth = int(center_v * (depth_h / self.target_h))
                z_center = self._read_depth_at_pixel(self.latest_depth, u_depth, v_depth)
            if z_center is None:
                # currently hard coded depth when it doesn't work
                self.get_logger().info("Depth read failed, defaulting to 1.0m")
                z_center = 1.0
            # self.get_logger().info(f"z_center raw: {z_center}")
            # self.get_logger().info(f"depth msg exists: {self.latest_depth is not None}")
            # 3. Backproject with SCALED Camera Intrinsics, then TF to world_frame
            if z_center is not None:
                X, Y, Z = self._backproject_pixel_to_3d(center_u, center_v, z_center, self.camera_info)
                camera_frame = self.camera_info.header.frame_id

                pose_cam = PoseStamped()
                pose_cam.header = header
                pose_cam.header.frame_id = camera_frame
                pose_cam.pose.position.x = X
                pose_cam.pose.position.y = Y
                pose_cam.pose.position.z = Z
                pose_cam.pose.orientation.w = 1.0

                pose_world = self._transform_pose_to_world(pose_cam)
                self.get_logger().info("Hi")
                self.get_logger().info(str(type(pose_cam)))
                self.get_logger().info(str(pose_world))
                if pose_world is not None:
                    self.pose_pub.publish(pose_world)
                    pw = pose_world.pose.position
                    self.get_logger().info(
                        f"Target {d['label']} in {self.world_frame}: "
                        f"x={pw.x:.2f}m, y={pw.y:.2f}m, z={pw.z:.2f}m"
                    )

            # Publish Mask
            mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding='mono8')
            mask_msg.header = header
            self.mask_pub.publish(mask_msg)

            # Draw Debug
            cv2.rectangle(debug_img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.circle(debug_img, (center_u, center_v), 5, (255, 0, 0), -1)

        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

    def _transform_pose_to_world(self, pose_cam: PoseStamped) -> Optional[PoseStamped]:
        src_frame = pose_cam.header.frame_id
        try:
            stamp = pose_cam.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                t = Time()
            else:
                t = Time.from_msg(stamp)
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                src_frame,
                t,
                timeout=self._tf_timeout,
            )
        except Exception as e:
            self.get_logger().warn(
                f'Could not transform pose from {src_frame} to {self.world_frame}: {e}',
                throttle_duration_sec=2.0,
            )
            return None
        pose_world = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, tf_msg)
        pose_world.header.frame_id = self.world_frame
        pose_world.header.stamp = tf_msg.header.stamp
        return pose_world

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
        boxes, logits, phrases = predict(model=self.model, image=image_tensor.to(self.device), 
                                         caption=caption, box_threshold=self.confidence_threshold, text_threshold=0.25)
        h, w = image_np.shape[:2]
        return [{"label": p, "score": float(l), "bbox": [int((b[0]-b[2]/2)*w), int((b[1]-b[3]/2)*h), 
                int((b[0]+b[2]/2)*w), int((b[1]+b[3]/2)*h)]} for b, l, p in zip(boxes, logits, phrases)]

    def run_segmentation(self, image_np, detections):
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