#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')
        
        # Parameters
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'aruco_0')  # Default to marker ID 0
        
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for transformed pose
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            f'/aruco_pose_{self.target_frame}', 
            10
        )
        
        # Timer to periodically publish transformed pose
        self.timer = self.create_timer(0.1, self.publish_transformed_pose)  # 10 Hz
        
        self.get_logger().info(f'Publishing ArUco pose from {self.source_frame} to {self.target_frame}')
    
    def publish_transformed_pose(self):
        """Transform ArUco pose to target frame and publish"""
        try:
            # Get transform from source frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            
            # Create a pose in the source frame (origin)
            source_pose = PoseStamped()
            source_pose.header.stamp = self.get_clock().now().to_msg()
            source_pose.header.frame_id = self.source_frame
            source_pose.pose.position.x = 0.0
            source_pose.pose.position.y = 0.0
            source_pose.pose.position.z = 0.0
            source_pose.pose.orientation.x = 0.0
            source_pose.pose.orientation.y = 0.0
            source_pose.pose.orientation.z = 0.0
            source_pose.pose.orientation.w = 1.0
            
            # Transform pose to target frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose(source_pose, transform)
            transformed_pose.header.frame_id = self.target_frame
            
            # Publish transformed pose
            self.pose_pub.publish(transformed_pose)
            
            # Log the pose information
            pos = transformed_pose.pose.position
            orient = transformed_pose.pose.orientation
            self.get_logger().info(
                f'ArUco pose in {self.target_frame}: '
                f'Position [x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}], '
                f'Orientation [x={orient.x:.3f}, y={orient.y:.3f}, z={orient.z:.3f}, w={orient.w:.3f}]',
                throttle_duration_sec=1.0  # Log once per second
            )
            
        except Exception as e:
            self.get_logger().warn(
                f'Could not transform pose from {self.source_frame} to {self.target_frame}: {str(e)}',
                throttle_duration_sec=5.0  # Warn every 5 seconds
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
