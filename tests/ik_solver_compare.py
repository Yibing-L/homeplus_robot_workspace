#!/usr/bin/env python3
"""Compare IK solvers on align_refined_grasp-style goals on an isolated MoveIt.

Starts its own move_group (separate ROS domain, no hardware nodes), then asks
/compute_ik for base_arm goals built like the grasp step: the gripper level at
cup height, heading offset from the base heading, base limited to a small box
around its current pose. Reports success rate, time and base motion.

Usage (on the ROS machine, workspace sourced):
    python3 tests/ik_solver_compare.py --kinematics homeplus_moveit_config/config/kinematics.yaml
    python3 tests/ik_solver_compare.py --kinematics /tmp/kinematics_pick_ik.yaml
"""
import os
os.environ['ROS_DOMAIN_ID'] = '214'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/homeplus_ik_compare_logs'

import argparse
import itertools
import math
import shutil
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

BASE = ['base_x', 'base_y', 'base_theta']
ARM = ['joint_shoulder', 'joint_elbow', 'joint_wrist', 'joint_hand']

# Situations taken from the step-5 failures on 2026-10-18:
# (base_x, base_y, base_yaw, target_x, target_y, target_z)
LOGGED = [
    (0.302, -0.040, 0.313, 0.889, 0.230, 0.807),
    (0.377, -0.040, 0.288, 0.857, 0.177, 0.745),
    (0.432, 0.001, 0.269, 0.872, 0.192, 0.774),
]


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def cases():
    """Logged situations plus a sweep of heading/lateral/height perturbations."""
    out = []
    for bx, by, byaw, tx, ty, tz in LOGGED:
        for dyaw, dlat, dz in itertools.product((-0.25, -0.12, 0.0, 0.12, 0.25),
                                                (-0.05, 0.0, 0.05), (0.0,)):
            nx, ny = -math.sin(byaw), math.cos(byaw)
            out.append(dict(base=(bx, by, byaw), target=(tx + dlat * nx, ty + dlat * ny, tz + dz),
                            yaw=byaw + dyaw))
    return out


def constraints(case, args):
    c = Constraints()
    bx, by, byaw = case['base']
    for name, center, tol in (('base_x', bx, args.base_xy), ('base_y', by, args.base_xy),
                              ('base_theta', byaw, args.base_yaw)):
        c.joint_constraints.append(JointConstraint(joint_name=name, position=center,
                                                   tolerance_above=tol, tolerance_below=tol, weight=1.0))
    o = OrientationConstraint()
    o.header.frame_id = 'world'
    o.link_name = 'grasp_tcp'
    q = quat_from_yaw(case['yaw'])
    o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w = q
    o.absolute_x_axis_tolerance = args.tilt_tol
    o.absolute_y_axis_tolerance = args.tilt_tol
    o.absolute_z_axis_tolerance = args.yaw_tol
    o.weight = 1.0
    c.orientation_constraints.append(o)
    return c


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--kinematics', type=Path, required=True)
    ap.add_argument('--attempts', type=int, default=24)
    ap.add_argument('--timeout', type=float, default=0.15)
    ap.add_argument('--base-xy', type=float, default=0.10)
    ap.add_argument('--base-yaw', type=float, default=0.20)
    ap.add_argument('--tilt-tol', type=float, default=0.04)
    ap.add_argument('--yaw-tol', type=float, default=0.25)
    args = ap.parse_args()

    # moveit.launch.py reads config/kinematics.yaml from the package share dir;
    # swap it in for this run and always restore it.
    share = Path(subprocess.check_output(['ros2', 'pkg', 'prefix', 'homeplus_moveit_config'],
                                         text=True).strip()) / 'share/homeplus_moveit_config/config'
    installed = (share / 'kinematics.yaml').resolve()
    backup = installed.with_suffix('.yaml.ikcompare_bak')
    swapped = args.kinematics.resolve() != installed
    if swapped:
        shutil.copy2(installed, backup)
        shutil.copy2(args.kinematics, installed)

    log = open('/tmp/homeplus_ik_compare_moveit.log', 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'homeplus_moveit_config', 'moveit.launch.py',
                             'use_rviz:=false', 'use_joint_gui:=false', 'run_arduino_reader:=false',
                             'use_octomap:=false', 'run_rebuild_map:=false'],
                            stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    node = Node('ik_solver_compare')
    try:
        js_pub = node.create_publisher(JointState, '/joint_states', 10)
        client = node.create_client(GetPositionIK, '/compute_ik')
        deadline = time.time() + 90
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() > deadline:
                raise RuntimeError('move_group did not start; see /tmp/homeplus_ik_compare_moveit.log')
        time.sleep(3)

        ok = 0
        total = 0
        times, base_moves = [], []
        for case in cases():
            state = JointState()
            state.name = BASE + ARM
            state.position = list(case['base']) + [0.0] * len(ARM)
            for _ in range(3):
                state.header.stamp = node.get_clock().now().to_msg()
                js_pub.publish(state)
                rclpy.spin_once(node, timeout_sec=0.02)
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = case['target']
            q = quat_from_yaw(case['yaw'])
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            total += 1
            t0 = time.time()
            solved = None
            for _ in range(args.attempts):
                req = GetPositionIK.Request()
                req.ik_request.group_name = 'base_arm'
                req.ik_request.ik_link_name = 'grasp_tcp'
                req.ik_request.pose_stamped = pose
                req.ik_request.robot_state.joint_state = state
                req.ik_request.constraints = constraints(case, args)
                req.ik_request.avoid_collisions = True
                req.ik_request.timeout.nanosec = int(args.timeout * 1e9)
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                res = future.result()
                if res is not None and res.error_code.val == 1:
                    solved = res.solution.joint_state
                    break
            times.append(time.time() - t0)
            if solved is not None:
                ok += 1
                q = dict(zip(solved.name, solved.position))
                bx, by, byaw = case['base']
                base_moves.append((math.hypot(q['base_x'] - bx, q['base_y'] - by),
                                   abs(q['base_theta'] - byaw)))
        print(f'kinematics={args.kinematics.name}  solved {ok}/{total} '
              f'({100.0 * ok / max(total, 1):.0f}%)  mean time {sum(times) / len(times):.2f}s')
        if base_moves:
            moves = sorted(m for m, _ in base_moves)
            turns = sorted(t for _, t in base_moves)
            print(f'  base translation median {moves[len(moves) // 2]:.3f} m, '
                  f'max {moves[-1]:.3f} m; rotation median {turns[len(turns) // 2]:.3f} rad')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        if swapped:
            shutil.copy2(backup, installed)
            backup.unlink()


if __name__ == '__main__':
    main()
