"""GPU + MoveIt integration check in an isolated ROS domain; no hardware nodes.

Checks TSDF accumulation, acknowledged readiness, motion gating, reset generations,
and that MoveIt's collision checker actually consumes the exported mesh.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '216'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/homeplus-nvblox-smoke-ros'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import CameraInfo, JointState, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from tf2_ros import StaticTransformBroadcaster


def main():
    log_path = Path('/tmp/homeplus-nvblox-smoke.log')
    log = log_path.open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'homeplus_moveit_config', 'moveit.launch.py',
        'use_mapping:=true', 'use_rviz:=false', 'use_joint_gui:=false',
        'run_arduino_reader:=false', 'run_rebuild_map:=false',
    ], stdout=log, stderr=log, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = Node('nvblox_smoke')
        joints_pub = node.create_publisher(JointState, '/joint_states', 10)
        motion_pub = node.create_publisher(Bool, '/arduino/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        info_pub = node.create_publisher(CameraInfo, '/cameras/d405/color/camera_info', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/obstacle_cloud', 10)
        ready = []
        subscription = node.create_subscription(String, '/vision/planning_map_ready',
            lambda msg: ready.append(json.loads(msg.data)), 10)
        state = JointState(name=['base_x', 'base_y', 'base_theta', 'joint_shoulder',
            'joint_elbow', 'joint_wrist', 'joint_hand', 'joint_grip_L',
            'joint_wheel_FL', 'joint_wheel_FR', 'joint_wheel_RL', 'joint_wheel_RR'], position=[0.0]*12)
        def publish_joints():
            state.header.stamp = node.get_clock().now().to_msg()
            joints_pub.publish(state)
        timer = node.create_timer(0.05, publish_joints)
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'world'; transform.child_frame_id = 'synthetic_camera'
        transform.transform.translation.z = 0.7
        # Optical x -> -world y; optical y -> -world z; optical z -> world x.
        transform.transform.rotation.x = -0.5; transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5; transform.transform.rotation.w = 0.5
        broadcaster.sendTransform(transform)
        info = CameraInfo(width=640, height=480)
        info.header.frame_id = 'synthetic_camera'
        info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]

        def spin(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                if process.poll() is not None:
                    raise RuntimeError(f'Launch exited: inspect {log_path}')
                rclpy.spin_once(node, timeout_sec=0.02)

        def call(cls, name, request):
            client = node.create_client(cls, name)
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError(f'{name} unavailable: inspect {log_path}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            if not future.done():
                raise RuntimeError(f'{name} timed out: inspect {log_path}')
            return future.result()

        def get_map():
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            result = call(GetPlanningScene, '/get_planning_scene', request)
            return next((obj for obj in result.scene.world.collision_objects
                         if obj.id == 'nvblox_environment'), None)

        # Sample the same optical pixel lattice as the production cloud node.
        points = [((u - 320)*1.2/600, (v - 240)*1.2/600, 1.2)
                  for v in range(100, 380, 4) for u in range(160, 480, 4)]
        def publish_cloud():
            info.header.stamp = node.get_clock().now().to_msg(); info_pub.publish(info)
            header = Header(frame_id='synthetic_camera', stamp=node.get_clock().now().to_msg())
            cloud_pub.publish(create_cloud_xyz32(header, points))
            spin(0.7)

        # Wait for services and the complete robot TF tree, then clear even
        # when no map object exists yet (a former MoveIt failure case).
        call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request())
        motion_pub.publish(Bool(data=True)); info_pub.publish(info); spin(2)
        reset = call(Trigger, '/nvblox/clear', Trigger.Request())
        assert reset.success, reset.message
        generation = int(reset.message)
        assert get_map() is None
        ready.clear()
        for _ in range(2): publish_cloud()
        assert not ready, 'Map declared ready before three observations'
        for _ in range(8):
            publish_cloud()
            if ready: break
        assert ready, f'No reconstructed mesh: inspect {log_path}'
        assert ready[-1]['generation'] == generation and ready[-1]['frames'] >= 3
        mesh_object = get_map()
        assert mesh_object and mesh_object.meshes[0].triangles
        mesh = mesh_object.meshes[0]
        assert max(abs(p.x - 1.2) for p in mesh.vertices) < 0.025

        # Attach a virtual sphere at the reconstructed wall. This proves the
        # scene is collision-active, beyond merely publishing an RViz marker.
        attached = AttachedCollisionObject(link_name='base_link')
        attached.object.header.frame_id = 'world'; attached.object.id = 'nvblox_test_probe'
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.06])]
        pose = Pose(); pose.orientation.w = 1.; pose.position.x = 1.2; pose.position.z = 0.7
        attached.object.primitive_poses = [pose]
        request = ApplyPlanningScene.Request(); request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [attached]
        assert call(ApplyPlanningScene, '/apply_planning_scene', request).success
        check = GetStateValidity.Request(group_name='base_arm')
        check.robot_state.joint_state = state; check.robot_state.is_diff = True
        collision = call(GetStateValidity, '/check_state_validity', check)
        assert not collision.valid
        assert any('nvblox_environment' in (c.contact_body_1, c.contact_body_2)
                   for c in collision.contacts), 'MoveIt did not collide with the nvblox mesh'

        motion_pub.publish(Bool(data=False)); spin(0.3); ready.clear()
        publish_cloud(); publish_cloud()
        assert not ready, 'Mapping continued while hardware was moving'
        motion_pub.publish(Bool(data=True)); spin(0.3)
        reset = call(Trigger, '/nvblox/clear', Trigger.Request())
        assert reset.success and int(reset.message) == generation + 1
        assert get_map() is None, 'Old collision mesh survived reset'
        ready.clear()
        for _ in range(2): publish_cloud()
        assert not ready, 'Reset did not restart accumulation'
        print(json.dumps({'result': 'PASS', 'frames': 3, 'generation': generation,
                          'triangles': len(mesh.triangles), 'collision_verified': True,
                          'log': str(log_path)}))
    finally:
        if node:
            node.destroy_node()
            rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close()


if __name__ == '__main__':
    main()
