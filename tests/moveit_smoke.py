#!/usr/bin/env python3
"""Isolated MoveIt integration smoke test. Starts no hardware bridge or controller."""
import os
from pathlib import Path
import signal
import sys
import subprocess
import time

# Isolate before importing DDS. The test must never join the robot's ROS domain.
os.environ['ROS_DOMAIN_ID'] = '213'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/homeplus_moveit_smoke_logs'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import JointConstraint, Constraints
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity, GetMotionPlan, GetCartesianPath


def main():
    log = open('/tmp/homeplus_moveit_smoke.log','w')
    process = subprocess.Popen(['ros2','launch','homeplus_moveit_config','moveit.launch.py',
        'use_rviz:=false','use_joint_gui:=false','run_arduino_reader:=false',
        'use_mapping:=false','run_rebuild_map:=false'], stdout=log, stderr=log, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = Node('pipeline_smoke')
        pub = node.create_publisher(JointState, '/joint_states', 10)
        state = JointState(name=['base_x','base_y','base_theta','joint_shoulder','joint_elbow','joint_wrist','joint_hand','joint_grip_L'],
                           position=[0.0]*8)
        def publish():
            state.header.stamp = node.get_clock().now().to_msg()
            pub.publish(state)
        timer = node.create_timer(.05, publish)
        def call(cls, name, request):
            client=node.create_client(cls,name)
            if not client.wait_for_service(timeout_sec=25):
                raise RuntimeError(f'Service not available: {name}; inspect /tmp/homeplus_moveit_smoke.log')
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=20)
            if not future.done():raise RuntimeError(f'Service timeout: {name}')
            return future.result()
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        validity=GetStateValidity.Request(group_name='base_arm')
        validity.robot_state.joint_state=state;validity.robot_state.is_diff=True
        valid=call(GetStateValidity,'/check_state_validity',validity)
        if not valid.valid:
            raise RuntimeError(f'Home state collision: {[(c.contact_body_1,c.contact_body_2) for c in valid.contacts]}')
        for group,link in [('arm','grasp_tcp'),('press_arm','press_tcp'),('base_arm','grasp_tcp'),('base_press_arm','press_tcp')]:
            fk=GetPositionFK.Request();fk.header.frame_id='world';fk.fk_link_names=[link]
            fk.robot_state.joint_state=state;fk.robot_state.is_diff=True
            pose=call(GetPositionFK,'/compute_fk',fk)
            assert pose.error_code.val==1,(group,'FK',pose.error_code.val)
            ik=GetPositionIK.Request();ik.ik_request.group_name=group;ik.ik_request.ik_link_name=link
            ik.ik_request.pose_stamped=pose.pose_stamped[0]
            ik.ik_request.robot_state.joint_state=state;ik.ik_request.robot_state.is_diff=True
            ik.ik_request.avoid_collisions=True;ik.ik_request.timeout.sec=1
            result=call(GetPositionIK,'/compute_ik',ik)
            assert result.error_code.val==1,(group,'IK',result.error_code.val)
            print(f'{group}: FK, IK tip, collision checking OK',flush=True)
        import math
        from geometry_msgs.msg import PoseStamped
        from copy import deepcopy
        target=PoseStamped();target.header.frame_id='world'
        target.pose.position.x=.662;target.pose.position.y=.096;target.pose.position.z=.777
        yaw=math.atan2(.123,.860)
        target.pose.orientation.z=math.sin(yaw/2);target.pose.orientation.w=math.cos(yaw/2)
        for distance in [.35,.45,.5,.6,.7]:
            seed=deepcopy(state);seed.position[0]=target.pose.position.x-distance*math.cos(yaw)
            seed.position[1]=target.pose.position.y-distance*math.sin(yaw)-.056
            seed.position[2]=yaw;seed.position=list(seed.position[:3])+[.8,1.,0.,.2,0.]
            ik=GetPositionIK.Request();ik.ik_request.group_name='base_arm';ik.ik_request.ik_link_name='grasp_tcp'
            ik.ik_request.pose_stamped=target;ik.ik_request.robot_state.joint_state=seed;ik.ik_request.robot_state.is_diff=True
            ik.ik_request.avoid_collisions=True;ik.ik_request.timeout.sec=1
            got=call(GetPositionIK,'/compute_ik',ik)
            assert got.error_code.val==1, ('pickup pregrasp IK',distance,got.error_code.val)
            cart=GetCartesianPath.Request();cart.header.frame_id='world';cart.start_state.joint_state=got.solution.joint_state;cart.start_state.is_diff=True
            cart.group_name='arm';cart.link_name='grasp_tcp';cart.max_step=.005;cart.jump_threshold=2.;cart.revolute_jump_threshold=.15;cart.prismatic_jump_threshold=.01;cart.avoid_collisions=True
            for x,z in [(.19,0),(.19,.1),(.04,.1)]:
                pt=deepcopy(target.pose);pt.position.x+=x*math.cos(yaw);pt.position.y+=x*math.sin(yaw);pt.position.z+=z;cart.waypoints.append(pt)
            got=call(GetCartesianPath,'/compute_cartesian_path',cart)
            assert got.error_code.val==1 and got.fraction>.999999, ('pickup continuation',distance,got.error_code.val,got.fraction)
            print('Pickup approach/lift/retreat continuation OK for seed reach',distance,flush=True)
        plan=GetMotionPlan.Request();request=plan.motion_plan_request
        request.group_name='arm';request.planner_id='RRTConnectkConfigDefault'
        request.start_state.joint_state=state;request.start_state.is_diff=True
        request.allowed_planning_time=5.;request.num_planning_attempts=1
        request.max_velocity_scaling_factor=.15;request.max_acceleration_scaling_factor=.15
        goal=Constraints()
        for joint,value in [('joint_shoulder',.05),('joint_elbow',.1),('joint_wrist',0.),('joint_hand',0.)]:
            goal.joint_constraints.append(JointConstraint(joint_name=joint,position=value,
                tolerance_above=.001,tolerance_below=.001,weight=1.))
        request.goal_constraints=[goal]
        result=call(GetMotionPlan,'/plan_kinematic_path',plan).motion_plan_response
        assert result.error_code.val==1,('RRTConnect',result.error_code.val)
        assert len(result.trajectory.joint_trajectory.points)>1
        print('RRTConnect joint-goal plan and time parameterization OK',flush=True)
        target_state=JointState(name=state.name,position=list(state.position))
        target_state.position[3]=.05;target_state.position[4]=.1
        fk=GetPositionFK.Request();fk.header.frame_id='world';fk.fk_link_names=['grasp_tcp']
        fk.robot_state.joint_state=target_state;fk.robot_state.is_diff=True
        target=call(GetPositionFK,'/compute_fk',fk)
        cart=GetCartesianPath.Request();cart.header.frame_id='world'
        cart.start_state.joint_state=state;cart.start_state.is_diff=True
        cart.group_name='arm';cart.link_name='grasp_tcp';cart.max_step=.005
        cart.jump_threshold=2.;cart.revolute_jump_threshold=.15;cart.prismatic_jump_threshold=.01
        cart.avoid_collisions=True;cart.waypoints=[target.pose_stamped[0].pose]
        result=call(GetCartesianPath,'/compute_cartesian_path',cart)
        assert result.error_code.val==1 and result.fraction>.999999,('Cartesian',result.error_code.val,result.fraction)
        print('Fixed-base Cartesian path, jump checks and collision checking OK',flush=True)
        root=Path(__file__).resolve().parents[1]
        sys.path.insert(0,str(root/'homeplus_moveit_config/scripts'))
        from gesture_motion_executor import GestureMotionExecutor
        from rclpy.parameter import Parameter
        executor=GestureMotionExecutor(parameter_overrides=[Parameter('execution_map_path',
            value=str(root/'homeplus_moveit_config/config/gesture_execution_map.yaml'))])
        assert executor.latest_joint_state is None and not executor._hardware_ready
        assert executor.execution_order=='staged'
        executor.destroy_node()
        print('Executor startup/configuration OK; starts with no assumed hardware state',flush=True)
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()


if __name__=='__main__':main()
