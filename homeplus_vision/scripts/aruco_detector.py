#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # Parameters - No default values, must be provided via config file or launch args
        self.declare_parameter('aruco_dict_type')
        self.declare_parameter('marker_size')
        self.declare_parameter('camera_frame')
        self.declare_parameter('base_frame')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')

        self.aruco_dict_type = self.get_parameter('aruco_dict_type').value
        self.marker_size = self.get_parameter('marker_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.image_topic = self.get_parameter('image_topic').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.debug_img_pub = self.create_publisher(Image, '/aruco_debug_image', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.get_logger().info('ArUco detector initialized')
    
    def camera_info_callback(self, msg):
        """Extract camera calibration parameters from camera_info"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('Camera calibration parameters received')
    
    def image_callback(self, msg):
        """Process incoming images for ArUco detection"""
        if not self.camera_info_received:
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect ArUco markers
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params
            )
            
            # Create debug image
            debug_image = cv_image.copy()
            
            if ids is not None and len(ids) > 0:
                # Draw detected markers
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
                
                # Estimate pose for each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                for i, marker_id in enumerate(ids.flatten()):
                    # Draw axis for each marker
                    cv2.aruco.drawAxis(
                        debug_image, self.camera_matrix, self.dist_coeffs,
                        rvecs[i], tvecs[i], self.marker_size
                    )
                    
                    # Publish pose and transform for each detected marker
                    self.publish_marker_pose(marker_id, rvecs[i], tvecs[i], msg.header.stamp)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header = msg.header
            self.debug_img_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def publish_marker_pose(self, marker_id, rvec, tvec, timestamp):
        """Publish marker pose as PoseStamped message and TF transform"""
        
        # Convert rotation vector to quaternion
        rotation_matrix = cv2.Rodrigues(rvec)[0]
        r = R.from_matrix(rotation_matrix)
        quaternion = r.as_quat()  # Returns [x, y, z, w]
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.camera_frame
        
        # Position (translation vector)
        pose_msg.pose.position.x = float(tvec[0][0])
        pose_msg.pose.position.y = float(tvec[0][1])
        pose_msg.pose.position.z = float(tvec[0][2])
        
        # Orientation (quaternion)
        pose_msg.pose.orientation.x = float(quaternion[0])
        pose_msg.pose.orientation.y = float(quaternion[1])
        pose_msg.pose.orientation.z = float(quaternion[2])
        pose_msg.pose.orientation.w = float(quaternion[3])
        
        # Publish pose
        self.pose_pub.publish(pose_msg)
        
        # Broadcast TF transform
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = f'aruco_{marker_id}'
        
        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z
        
        transform.transform.rotation.x = pose_msg.pose.orientation.x
        transform.transform.rotation.y = pose_msg.pose.orientation.y
        transform.transform.rotation.z = pose_msg.pose.orientation.z
        transform.transform.rotation.w = pose_msg.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(transform)
        
        # Log pose information
        euler = r.as_euler('xyz', degrees=True)
        self.get_logger().info(
            f'Marker {marker_id}: Position [x={tvec[0][0]:.3f}, y={tvec[0][1]:.3f}, z={tvec[0][2]:.3f}], '
            f'Rotation [r={euler[0]:.1f}°, p={euler[1]:.1f}°, y={euler[2]:.1f}°]'
        )

def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
