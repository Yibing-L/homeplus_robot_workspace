#!/usr/bin/env python3
"""Check deliberate base placement against MoveIt on an isolated move_group.

Starts its own move_group (separate ROS domain, localhost only, no hardware
nodes, no OctoMap), adds a table and the executor's cup collision envelope,
and for each logged situation plus a sweep of cup positions/heights checks
what the executor will do:

  1. placement: planar_reach chooses the base pose; MoveIt FK must agree
     with the reach model and the pre-grasp state must be collision-free and
     satisfy the step-3 goal constraints (base_arm);
  2. approach: the fixed-base Cartesian pre-grasp -> grasp path (step 3's
     lookahead and step 6) must complete from that state;
  3. arm-only refinement (steps 5-6, base locked): after simulated base
     settling error and refined-detection shifts within the configured
     limits, the in-plane target's closed-form arm solution must pass
     MoveIt's validity check (what the executor tries first) and the
     Cartesian grasp approach must complete. KDL /compute_ik on the same goal
     is reported as the fallback's success rate.

Usage (on the ROS machine, workspace sourced, do not export a FastDDS
profile that peers with other machines):
    python3 tests/placement_moveit_check.py
    python3 tests/placement_moveit_check.py --kinematics /tmp/kinematics_pick_ik.yaml
Exit status is nonzero if any reachable case fails.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '214'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/homeplus_placement_check_logs'
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)

import argparse
import itertools
import math
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (CollisionObject, Constraints, OrientationConstraint,
                             PositionConstraint, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetPositionFK,
                             GetPositionIK, GetStateValidity)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'homeplus_moveit_config/scripts'))
from planar_reach import PlanarArm, arm_plane_error, fixed_link_height, plan_base_placements  # noqa: E402

BASE = ['base_x', 'base_y', 'base_theta']
ARM = ['joint_shoulder', 'joint_elbow', 'joint_wrist', 'joint_hand']
ALL = BASE + ['joint_grip_L'] + ARM

# 2026-10-18 step-5 failures: (base x, y, yaw, far-view cup x, y, z).
LOGGED = [
    (0.302, -0.040, 0.313, 1.137, 0.289, 0.807),
    (0.377, -0.040, 0.288, 1.109, 0.236, 0.745),
    (0.432, 0.001, 0.269, 1.096, 0.247, 0.773),
]


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def load_sequence():
    commands = yaml.safe_load((ROOT / 'homeplus_moveit_config/config/gesture_execution_map.yaml').read_text())['commands']
    steps = {s['name']: s for s in commands['pick_up_coffee']['steps']}
    vision = yaml.safe_load((ROOT / 'homeplus_vision/config/target_detection_params.yaml').read_text())
    return steps, vision['molmopoint']['ros__parameters']


class Checker(Node):
    def __init__(self):
        super().__init__('placement_moveit_check')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f'{client.srv_name} timed out')
        return future.result()


def joint_state(values):
    js = JointState()
    js.name = list(ALL)
    js.position = [float(values.get(n, 0.0)) for n in ALL]
    return js


def goal_constraints(link, position, yaw, tol_xy, tol_z, tol_rot):
    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id = 'world'
    pc.link_name = link
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2 * tol_xy, 2 * tol_xy, 2 * tol_z])
    region = Pose()
    region.position.x, region.position.y, region.position.z = position
    region.orientation.w = 1.0
    pc.constraint_region.primitives = [box]
    pc.constraint_region.primitive_poses = [region]
    pc.weight = 1.0
    c.position_constraints.append(pc)
    oc = OrientationConstraint()
    oc.header.frame_id = 'world'
    oc.link_name = link
    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = quat_from_yaw(yaw)
    oc.parameterization = OrientationConstraint.ROTATION_VECTOR
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = tol_rot
    oc.weight = 1.0
    c.orientation_constraints.append(oc)
    return c


def set_scene(node, cup, heading, envelope_r, envelope_h):
    """Table under the cup plus the executor's cup envelope (the detection is
    the front surface; the envelope sits one radius behind it)."""
    cx, cy, cz = cup
    cup_center = (cx + envelope_r * math.cos(heading), cy + envelope_r * math.sin(heading),
                  cz - 0.15 * envelope_h)
    table_top = cup_center[2] - envelope_h / 2
    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True
    cup_obj = CollisionObject(id='detected_cup', operation=CollisionObject.ADD)
    cup_obj.header.frame_id = 'world'
    cup_obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[envelope_h, envelope_r])]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = cup_center
    pose.orientation.w = 1.0
    cup_obj.primitive_poses = [pose]
    # Table: front edge 0.15 m in front of the cup's front surface, 0.8 m deep.
    table = CollisionObject(id='table', operation=CollisionObject.ADD)
    table.header.frame_id = 'world'
    depth, width = 0.8, 1.2
    table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[depth, width, table_top])]
    tpose = Pose()
    front = 0.15
    centre_along = -front + depth / 2
    tpose.position.x = cx + centre_along * math.cos(heading)
    tpose.position.y = cy + centre_along * math.sin(heading)
    tpose.position.z = table_top / 2
    tpose.orientation.z, tpose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
    table.primitive_poses = [tpose]
    req.scene.world.collision_objects = [cup_obj, table]
    # The fingers may touch the cup (as in the executor's ACM entry).
    if not node.call(node.scene, req).success:
        raise RuntimeError('could not set the planning scene')


def cartesian_ok(node, start_values, link, waypoints, yaw, tol_rot):
    req = GetCartesianPath.Request()
    req.header.frame_id = 'world'
    req.start_state.joint_state = joint_state(start_values)
    req.group_name = 'arm'
    req.link_name = link
    for p in waypoints:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = p
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat_from_yaw(yaw)
        req.waypoints.append(pose)
    req.max_step = 0.005
    req.jump_threshold = 2.0
    req.revolute_jump_threshold = 0.15
    req.prismatic_jump_threshold = 0.01
    req.avoid_collisions = True
    oc = goal_constraints(link, waypoints[-1], yaw, 1, 1, tol_rot).orientation_constraints[0]
    req.path_constraints.orientation_constraints = [oc]
    res = node.call(node.cartesian, req)
    return res.error_code.val == 1 and res.fraction >= 0.999999


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--kinematics', type=Path, help='swap in this kinematics.yaml for the run')
    ap.add_argument('--quick', action='store_true', help='logged cases only')
    args = ap.parse_args()
    steps, vision = load_sequence()
    settings = steps['move_to_pregrasp']['base_placement']
    lateral_limit = steps['refine_target']['max_arm_plane_error_m']
    link = 'grasp_tcp'
    pre = steps['move_to_pregrasp']
    align = steps['align_refined_grasp']
    grasp = steps['move_to_grasp']
    lift = steps['lift']
    retreat = steps['retreat_with_object']
    waypoints = [(pre['offset_x'], pre.get('offset_z', 0.0)), (align['offset_x'], align.get('offset_z', 0.0)),
                 (grasp['offset_x'], grasp.get('offset_z', 0.0))]
    waypoints.append((waypoints[-1][0] + lift.get('offset_x', 0.0), waypoints[-1][1] + lift['offset_z']))
    waypoints.append((waypoints[-1][0] + retreat['offset_x'], waypoints[-1][1] + retreat.get('offset_z', 0.0)))

    urdf = subprocess.check_output(['xacro', str(ROOT / 'homeplus_urdf_description/urdf/homeplus_urdf.xacro')], text=True)
    arm = PlanarArm.from_urdf(urdf, 'base_link', link)
    base_height = fixed_link_height(urdf)

    swapped = False
    if args.kinematics:
        share = Path(subprocess.check_output(['ros2', 'pkg', 'prefix', 'homeplus_moveit_config'],
                                             text=True).strip()) / 'share/homeplus_moveit_config/config'
        installed = (share / 'kinematics.yaml').resolve()
        backup = installed.with_suffix('.yaml.placement_bak')
        if args.kinematics.resolve() != installed:
            shutil.copy2(installed, backup)
            shutil.copy2(args.kinematics, installed)
            swapped = True
    log = open('/tmp/homeplus_placement_check_moveit.log', 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'homeplus_moveit_config', 'moveit.launch.py',
                             'use_rviz:=false', 'use_joint_gui:=false', 'run_arduino_reader:=false',
                             'use_octomap:=false', 'run_rebuild_map:=false'],
                            stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    node = Checker()
    failures = []
    stats = dict(cases=0, placed=0, replaced_lateral=0, replaced_reach=0, unreachable=0, approach_ok=0, arm_checks=0, arm_ok=0, kdl_ok=0,
                 grasp_ok=0, margins=[], forwards=[])
    try:
        deadline = time.time() + 120
        for client in (node.validity, node.fk, node.ik, node.cartesian, node.scene):
            while not client.wait_for_service(timeout_sec=1.0):
                if time.time() > deadline:
                    raise RuntimeError('move_group did not start; see /tmp/homeplus_placement_check_moveit.log')
        time.sleep(2)

        cases = [dict(base=(bx, by, byaw), cup=(cx, cy, cz), label=f'logged {i}')
                 for i, (bx, by, byaw, cx, cy, cz) in enumerate(LOGGED)]
        if not args.quick:
            for cz, cx, cy, byaw in itertools.product((0.70, 0.75, 0.80, 0.85, 0.90), (0.9, 1.2),
                                                      (-0.3, 0.25), (-0.3, 0.3)):
                cases.append(dict(base=(0.2, 0.0, byaw), cup=(cx, cy, cz), label='sweep'))
        # Perturbations for steps 5-6: base settling error (transit profile
        # tolerance) and refined detection shifts inside the configured gates.
        perturbations = list(itertools.product((-lateral_limit, 0.0, lateral_limit), (-0.04, 0.0, 0.04),
                                               (-0.03, 0.0, 0.03)))
        base_errors = [(0.005, -0.005, math.radians(1.1)), (-0.005, 0.005, -math.radians(1.1))]
        for case in cases:
            stats['cases'] += 1
            cup = case['cup']
            placements = plan_base_placements(
                arm, cup, case['base'], waypoints,
                heading_offsets=[math.radians(v) for v in settings['heading_offsets_deg']],
                min_margin=settings['min_reach_margin_m'], limit_margin=settings['joint_limit_margin_rad'],
                base_height=base_height, preference=settings.get('reach_preference', 'centre'))
            if not placements:
                stats['unreachable'] += 1
                print(f"{case['label']}: cup {cup} not reachable with margin (skipped)")
                continue
            set_scene(node, cup, placements[0].heading, vision['cup_collision_radius_m'],
                      vision['cup_collision_height_m'])
            chosen = None
            for placement in placements:
                values = {'base_x': placement.base_x, 'base_y': placement.base_y,
                          'base_theta': placement.heading, **placement.arm}
                c, s = math.cos(placement.heading), math.sin(placement.heading)
                target = (cup[0] + c * pre['offset_x'], cup[1] + s * pre['offset_x'], cup[2] + pre.get('offset_z', 0.0))
                fk_req = GetPositionFK.Request()
                fk_req.header.frame_id = 'world'
                fk_req.fk_link_names = [link]
                fk_req.robot_state.joint_state = joint_state(values)
                fk = node.call(node.fk, fk_req).pose_stamped[0].pose.position
                mismatch = math.dist((fk.x, fk.y, fk.z), target)
                if mismatch > 0.002:
                    failures.append(f"{case['label']} {cup}: reach model vs MoveIt FK {mismatch * 1000:.1f} mm")
                    continue
                req = GetStateValidity.Request()
                req.robot_state.joint_state = joint_state(values)
                req.group_name = 'base_arm'
                req.constraints = goal_constraints(link, target, placement.heading, pre['position_tolerance_xy_m'],
                                                   pre['position_tolerance_z_m'], pre['orientation_tolerance_rad'])
                res = node.call(node.validity, req)
                if not res.valid:
                    continue
                grasp_point = (cup[0] + c * grasp['offset_x'], cup[1] + s * grasp['offset_x'], cup[2])
                if not cartesian_ok(node, values, link, [grasp_point], placement.heading,
                                    pre['orientation_tolerance_rad']):
                    continue
                chosen = (placement, values)
                break
            if chosen is None:
                failures.append(f"{case['label']} {cup}: no placement passed MoveIt (collision/constraints/approach)")
                continue
            placement, values = chosen
            stats['placed'] += 1
            stats['approach_ok'] += 1
            stats['margins'].append(placement.margin)
            stats['forwards'].append(placement.forward)

            # Steps 5-6 from the placed base (with settling error), base locked.
            for (ebx, eby, eth), (dlat, dfwd, dz) in itertools.product(base_errors, perturbations):
                bx, by, th = placement.base_x + ebx, placement.base_y + eby, placement.heading + eth
                c, s = math.cos(th), math.sin(th)
                refined = (cup[0] + c * dfwd - s * dlat, cup[1] + s * dfwd + c * dlat, cup[2] + dz)
                error, forward = arm_plane_error(bx, by, th, refined[:2], arm.lateral_offset)
                if abs(error) > lateral_limit + 1e-9:
                    stats['replaced_lateral'] += 1
                    continue  # refine_target would re-place the base instead
                arm_points = [(forward + dx, refined[2] - base_height + dz) for dx, dz in waypoints[1:]]
                if not any(arm.solve_path(arm_points, b, 0.0) is not None for b in (1, -1)):
                    stats['replaced_reach'] += 1
                    continue  # out of arm-only reach: refine_target re-places too
                stats['arm_checks'] += 1
                in_plane = (refined[0] + s * error, refined[1] - c * error, refined[2])
                # refine_target moves the cup envelope to the refined pose.
                set_scene(node, refined, th, vision['cup_collision_radius_m'], vision['cup_collision_height_m'])
                targets = []
                for step in (align, grasp):
                    ox = step['offset_x']
                    targets.append((in_plane[0] + c * ox, in_plane[1] + s * ox, in_plane[2] + step.get('offset_z', 0.0)))
                solutions = arm.solve_level(forward + align['offset_x'], refined[2] + align.get('offset_z', 0.0) - base_height)
                state = {**values, 'base_x': bx, 'base_y': by, 'base_theta': th}
                ok = False
                for q in solutions:
                    trial = {**state, **{n: q[n] for n in ARM}}
                    req = GetStateValidity.Request()
                    req.robot_state.joint_state = joint_state(trial)
                    req.group_name = 'arm'
                    req.constraints = goal_constraints(link, targets[0], th, align['position_tolerance_m'],
                                                       align['position_tolerance_m'], align['orientation_tolerance_rad'])
                    if node.call(node.validity, req).valid:
                        ok = True
                        if cartesian_ok(node, trial, link, [targets[1]], th, grasp['orientation_tolerance_rad']):
                            stats['grasp_ok'] += 1
                        else:
                            failures.append(f"{case['label']} {cup} perturb {(dlat, dfwd, dz)}: grasp approach failed")
                        break
                if ok:
                    stats['arm_ok'] += 1
                else:
                    failures.append(f"{case['label']} {cup} perturb {(dlat, dfwd, dz)} base err {(ebx, eby, eth)}: "
                                    f'no valid in-plane arm solution ({len(solutions)} geometric)')
                # KDL fallback (what _ik_candidates would do) from the pre-grasp arm state.
                ik = GetPositionIK.Request()
                ik.ik_request.group_name = 'arm'
                ik.ik_request.ik_link_name = link
                ik.ik_request.pose_stamped.header.frame_id = 'world'
                p = ik.ik_request.pose_stamped.pose
                p.position.x, p.position.y, p.position.z = targets[0]
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat_from_yaw(th)
                ik.ik_request.robot_state.joint_state = joint_state(state)
                ik.ik_request.avoid_collisions = True
                ik.ik_request.timeout.nanosec = int(0.15e9)
                if node.call(node.ik, ik).error_code.val == 1:
                    stats['kdl_ok'] += 1
            print(f"{case['label']}: cup ({cup[0]:.3f}, {cup[1]:.3f}, {cup[2]:.3f}) -> base "
                  f"({placement.base_x:.3f}, {placement.base_y:.3f}, {placement.heading:.3f}), "
                  f"{placement.forward:.3f} m ahead, margin {placement.margin:.3f}, heading offset "
                  f"{math.degrees(placement.heading_offset):+.0f} deg")
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
    reachable = stats['cases'] - stats['unreachable']
    print(f"\nkinematics={args.kinematics.name if args.kinematics else 'installed'}")
    print(f"placement: {stats['placed']}/{reachable} reachable cases valid in MoveIt "
          f"({stats['unreachable']} unreachable skipped); approach {stats['approach_ok']}/{reachable}")
    if stats['margins']:
        print(f"  cup forward {min(stats['forwards']):.3f}-{max(stats['forwards']):.3f} m, "
              f"reach margin min {min(stats['margins']):.3f} m")
    checked = stats['arm_checks']
    print(f"arm-only steps 5-6: closed-form valid {stats['arm_ok']}, grasp approach {stats['grasp_ok']}, "
          f"configured-solver /compute_ik {stats['kdl_ok']} (of {checked} perturbed goals; "
          f"refine_target would re-place {stats['replaced_lateral']} for lateral error, "
          f"{stats['replaced_reach']} for reach)")
    for failure in failures:
        print('FAIL', failure)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
