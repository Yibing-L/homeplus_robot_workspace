#!/usr/bin/env python3
"""Replay a captured Cartesian request on isolated MoveIt; no hardware nodes."""
import os
os.environ['ROS_DOMAIN_ID'] = '213'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/homeplus_replay_ros_logs'
import argparse
from pathlib import Path
import signal
import subprocess
import time
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetStateValidity
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    args = parser.parse_args()
    scene = deserialize_message((args.snapshot/'scene.cdr').read_bytes(), PlanningScene)
    request = deserialize_message((args.snapshot/'request.cdr').read_bytes(), GetCartesianPath.Request)
    log = open(args.snapshot/'replay_moveit.log', 'w')
    process = subprocess.Popen(['ros2','launch','homeplus_moveit_config','moveit.launch.py',
        'use_rviz:=false','use_joint_gui:=false','run_arduino_reader:=false',
        'use_octomap:=false','run_rebuild_map:=false'], stdout=log, stderr=log, start_new_session=True)
    node = None
    try:
        rclpy.init(); node = Node('planning_failure_replay')
        pub = node.create_publisher(JointState, '/joint_states', 10)
        joints = deepcopy(scene.robot_state.joint_state)
        def publish():
            joints.header.stamp=node.get_clock().now().to_msg(); pub.publish(joints)
        timer = node.create_timer(.05, publish)
        def call(cls, name, req):
            client=node.create_client(cls,name)
            if not client.wait_for_service(timeout_sec=25): raise RuntimeError(name+' unavailable')
            future=client.call_async(req)
            rclpy.spin_until_future_complete(node,future,timeout_sec=30)
            if not future.done(): raise RuntimeError(name+' timed out')
            return future.result()
        apply=ApplyPlanningScene.Request();apply.scene=scene;apply.scene.is_diff=False
        assert call(ApplyPlanningScene,'/apply_planning_scene',apply).success
        for collision_check in (True, False):
            probe=deepcopy(request);probe.avoid_collisions=collision_check
            result=call(GetCartesianPath,'/compute_cartesian_path',probe)
            print(f'collision_check={collision_check}: code={result.error_code.val}, fraction={result.fraction:.6f}',flush=True)
            if not collision_check:
                state=deepcopy(request.start_state)
                for i,point in enumerate(result.solution.joint_trajectory.points):
                    values=dict(zip(result.solution.joint_trajectory.joint_names,point.positions))
                    state.joint_state.position=[values.get(n,q) for n,q in zip(state.joint_state.name,state.joint_state.position)]
                    req=GetStateValidity.Request();req.robot_state=state;req.group_name=request.group_name;req.constraints=request.path_constraints
                    valid=call(GetStateValidity,'/check_state_validity',req)
                    if not valid.valid:
                        print('First invalid waypoint',i,'contacts=',sorted({(c.contact_body_1,c.contact_body_2) for c in valid.contacts}), 'constraints=',valid.constraint_result,flush=True)
                        for contact in valid.contacts:
                            print('Contact location:', contact.header.frame_id, contact.position,
                                  'penetration=', contact.depth, flush=True)
                        break
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()


if __name__ == '__main__': main()
