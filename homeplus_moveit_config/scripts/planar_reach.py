#!/usr/bin/env python3
"""Closed-form reach model for the HomePlus arm and deliberate base placement.

The arm's pitch joints (shoulder, elbow, hand) all rotate about axes parallel
to base_link's y axis, so with the roll joint(s) at zero the TCP moves in one
vertical plane that is offset sideways from base_link by a constant distance.
It can neither move the gripper sideways nor yaw it. A level grasp therefore
fixes the base: heading = gripper heading, and the arm plane must pass through
the target. This module derives that geometry from the URDF (no hand-entered
link lengths), solves the planar arm in closed form, and chooses a base pose
from which the arm alone can execute a list of in-plane waypoints.

Pure Python + numpy so it can be tested without ROS; MoveIt remains the
authority on collisions and is used by the executor to validate every result.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle(axis, angle):
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class ChainJoint:
    name: str
    type: str
    xyz: np.ndarray
    rotation: np.ndarray
    axis: np.ndarray
    lower: float = -math.inf
    upper: float = math.inf


@dataclass
class Placement:
    heading: float
    base_x: float
    base_y: float
    forward: float             # target distance ahead of base_link along heading
    margin: float              # half-width of the feasible forward interval
    arm: Dict[str, float]      # arm joints for the first waypoint (pre-grasp)
    waypoints: List[Dict[str, float]] = field(default_factory=list)
    heading_offset: float = 0.0


class PlanarArm:
    """Fixed-base arm chain base_link -> tip with in-plane closed-form IK."""

    def __init__(self, joints: List[ChainJoint], base_link: str, tip_link: str):
        self.joints = joints
        self.base_link = base_link
        self.tip_link = tip_link
        moving = [j for j in joints if j.type in ('revolute', 'continuous')]
        self.pitch_joints = []
        self.roll_joints = []
        for j in moving:
            if abs(abs(j.axis[1]) - 1.0) < 1e-6:
                self.pitch_joints.append(j)
            else:
                self.roll_joints.append(j)
        if len(self.pitch_joints) != 3:
            raise ValueError(f'expected three pitch joints about y, got {[j.name for j in self.pitch_joints]}')
        if [j.type for j in joints if j.type not in ('revolute', 'continuous', 'fixed')]:
            raise ValueError('chain contains unsupported joint types')
        for j in joints:
            # Pitch-only fixed rotations keep the chain planar; anything else
            # would tilt the arm plane and invalidate the closed form.
            r = j.rotation
            if abs(r[1, 1] - 1.0) > 1e-9:
                raise ValueError(f'{j.name}: origin rotation leaves the x-z plane')
        zero = {j.name: 0.0 for j in moving}
        tcp = self.fk(zero)
        # Pitch rotations about y never change y, so this is the plane offset.
        self.lateral_offset = float(tcp[1, 3])
        # Planar link vectors between consecutive pitch joints, expressed in
        # the frame of the earlier joint at zero configuration.
        self._frames = self._pitch_frames(zero)
        # Tip pitch about +y at zero; each pitch joint adds s_k * theta_k.
        self._tip_pitch = math.atan2(tcp[0, 2], tcp[0, 0])

    @classmethod
    def from_urdf(cls, urdf_xml: str, base_link: str = 'base_link', tip_link: str = 'grasp_tcp'):
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for element in root.findall('joint'):
            child = element.find('child').get('link')
            origin = element.find('origin')
            xyz = np.array([float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()])
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis_el = element.find('axis')
            axis = np.array([float(v) for v in (axis_el.get('xyz') if axis_el is not None else '1 0 0').split()])
            if np.linalg.norm(axis) > 0:
                axis = axis / np.linalg.norm(axis)
            limit = element.find('limit')
            lower = float(limit.get('lower', '-inf')) if limit is not None and element.get('type') == 'revolute' else -math.inf
            upper = float(limit.get('upper', 'inf')) if limit is not None and element.get('type') == 'revolute' else math.inf
            by_child[child] = (element.find('parent').get('link'),
                               ChainJoint(element.get('name'), element.get('type'), xyz,
                                          _rpy_matrix(*rpy), axis, lower, upper))
        chain = []
        link = tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f'{tip_link} is not below {base_link} in the URDF')
            parent, joint = by_child[link]
            chain.append(joint)
            link = parent
        return cls(list(reversed(chain)), base_link, tip_link)

    @property
    def joint_names(self):
        return [j.name for j in self.joints if j.type != 'fixed']

    @property
    def limits(self):
        return {j.name: (j.lower, j.upper) for j in self.joints if j.type != 'fixed'}

    def fk(self, q: Dict[str, float], upto: Optional[str] = None) -> np.ndarray:
        """Homogeneous transform base_link -> tip (or -> the child of `upto`)."""
        t = np.eye(4)
        for j in self.joints:
            step = np.eye(4)
            step[:3, :3] = j.rotation
            step[:3, 3] = j.xyz
            t = t @ step
            if j.type != 'fixed':
                rot = np.eye(4)
                rot[:3, :3] = _axis_angle(j.axis, float(q.get(j.name, 0.0)))
                t = t @ rot
            if upto == j.name:
                break
        return t

    def _pitch_frames(self, zero):
        """Planar (x, z) positions of the three pitch joints and the tip at zero."""
        points = []
        for j in self.pitch_joints:
            frame = self.fk(zero, upto=j.name)
            points.append(frame[[0, 2], 3])
        points.append(self.fk(zero)[[0, 2], 3])
        return points

    def solve_level(self, forward: float, height: float, pitch: float = 0.0,
                    limit_margin: float = 0.0) -> List[Dict[str, float]]:
        """Arm joint solutions placing the tip at (forward, lateral_offset,
        height) in base_link with tip pitch `pitch` (0 = level), roll joints 0.

        Rotation about +y by a maps planar (x + i z) by exp(-i a). Joint k
        rotates by s_k * theta_k about +y where s_k is its axis' y sign.
        Returns up to two solutions (elbow branches) inside the joint limits
        shrunk by `limit_margin`, each tagged with its branch in '_branch'.
        """
        s0, s1, s2, tip = (complex(p[0], p[1]) for p in self._frames)
        # Links in the parent pitch frame at zero config (planar).
        l1, l2, l3 = s1 - s0, s2 - s1, tip - s2
        a_total = pitch - self._tip_pitch          # sum of pitch rotations about +y
        target = complex(forward, height)
        h = target - s0 - np.exp(-1j * a_total) * l3
        r1, r2, rh = abs(l1), abs(l2), abs(h)
        c = (rh * rh - r1 * r1 - r2 * r2) / (2 * r1 * r2)
        if not -1.0 <= c <= 1.0:
            return []
        solutions = []
        for branch, delta in ((1, math.acos(c)), (-1, -math.acos(c))):
            # h = r1 e^{i(b1 - A1)} + r2 e^{i(b2 - A2)}, with (b2 - A2) - (b1 - A1) = delta
            u = math.atan2(h.imag, h.real) - math.atan2(r2 * math.sin(delta), r1 + r2 * math.cos(delta))
            v = u + delta
            a1 = math.atan2(l1.imag, l1.real) - u
            a2 = math.atan2(l2.imag, l2.real) - v
            alphas = (a1, a2 - a1, a_total - a2)
            q = {j.name: 0.0 for j in self.roll_joints}
            ok = True
            for joint, alpha in zip(self.pitch_joints, alphas):
                theta = wrap(alpha / joint.axis[1])
                if not joint.lower + limit_margin <= theta <= joint.upper - limit_margin:
                    ok = False
                    break
                q[joint.name] = theta
            if ok:
                q['_branch'] = branch
                solutions.append(q)
        return solutions

    def solve_path(self, points: Sequence[Tuple[float, float]], branch: int,
                   limit_margin: float, step: float = 0.01, pitch: float = 0.0):
        """Solutions on one elbow branch along straight segments between
        consecutive (forward, height) points, or None if any sample fails or
        the joints jump (branch flip). Returns the solutions at the points."""
        at_points = []
        previous = None
        for index, point in enumerate(points):
            segment = [point] if index == 0 else None
            if index:
                start = np.array(points[index - 1])
                end = np.array(point)
                n = max(1, int(math.ceil(np.linalg.norm(end - start) / step)))
                segment = [tuple(start + (end - start) * k / n) for k in range(1, n + 1)]
            for sample in segment:
                match = [s for s in self.solve_level(sample[0], sample[1], pitch, limit_margin)
                         if s['_branch'] == branch]
                if not match:
                    return None
                sol = match[0]
                if previous is not None and max(abs(sol[n] - previous[n]) for n in previous
                                                 if not n.startswith('_')) > 0.2:
                    return None
                previous = sol
            at_points.append({k: v for k, v in previous.items() if not k.startswith('_')})
        return at_points


def placement_heading(base_xy: Sequence[float], target_xy: Sequence[float], lateral_offset: float) -> float:
    """Heading for which the arm plane, carried straight ahead from the
    current base position, passes through the target: the base then reaches
    its goal by turning and driving forward, without strafing."""
    dx, dy = target_xy[0] - base_xy[0], target_xy[1] - base_xy[1]
    distance = math.hypot(dx, dy)
    if distance <= abs(lateral_offset):
        return math.atan2(dy, dx)
    return math.atan2(dy, dx) - math.asin(lateral_offset / distance)


def arm_plane_error(base_x, base_y, heading, target_xy, lateral_offset):
    """Signed sideways distance of the target from the arm plane (left +)
    and its forward distance along the heading, both from base_link."""
    dx, dy = target_xy[0] - base_x, target_xy[1] - base_y
    forward = math.cos(heading) * dx + math.sin(heading) * dy
    lateral = -math.sin(heading) * dx + math.cos(heading) * dy
    return lateral - lateral_offset, forward


def plan_base_placements(arm: PlanarArm, target: Sequence[float], base: Sequence[float],
                         reach_offsets: Sequence[Tuple[float, float]],
                         heading_offsets: Sequence[float] = (0.0,),
                         min_margin: float = 0.02, limit_margin: float = 0.05,
                         base_height: float = 0.0, grid: float = 0.005,
                         forward_alternatives: int = 3, alternative_spacing: float = 0.02,
                         max_forward: float = 1.5) -> List[Placement]:
    """Base poses from which the arm alone (roll joints at 0, tip level)
    reaches every waypoint target + (dx along heading, dz up) in
    `reach_offsets`, moving in straight lines between consecutive waypoints.

    For each heading the feasible set of target-forward distances is computed
    on a grid; the centre of its widest interval maximises tolerance to base
    and detection error in both directions. Alternatives (nearer the interval
    centre first) are appended for collision fallback. Placements whose
    interval half-width is below `min_margin` are not returned.
    """
    tx, ty, tz = target
    psi0 = placement_heading(base[:2], (tx, ty), arm.lateral_offset)
    height = tz - base_height
    results = []
    for offset in heading_offsets:
        psi = wrap(psi0 + offset)
        for branch in (1, -1):
            feasible = []
            for k in range(int(max_forward / grid) + 1):
                f = k * grid
                points = [(f + dx, height + dz) for dx, dz in reach_offsets]
                feasible.append(arm.solve_path(points, branch, limit_margin) is not None)
            intervals = []
            start = None
            for k, ok in enumerate(feasible + [False]):
                if ok and start is None:
                    start = k
                elif not ok and start is not None:
                    intervals.append((start * grid, (k - 1) * grid))
                    start = None
            if not intervals:
                continue
            lo, hi = max(intervals, key=lambda iv: iv[1] - iv[0])
            margin = (hi - lo) / 2
            if margin < min_margin:
                continue
            centre = (lo + hi) / 2
            forwards = [centre]
            for k in range(1, forward_alternatives + 1):
                for sign in (1, -1):
                    f = centre + sign * k * alternative_spacing
                    if lo + min_margin / 2 <= f <= hi - min_margin / 2:
                        forwards.append(f)
            for f in forwards:
                points = [(f + dx, height + dz) for dx, dz in reach_offsets]
                solutions = arm.solve_path(points, branch, limit_margin)
                if solutions is None:
                    continue
                c, s = math.cos(psi), math.sin(psi)
                d = arm.lateral_offset
                results.append(Placement(
                    heading=psi,
                    base_x=tx - c * f + s * d,
                    base_y=ty - s * f - c * d,
                    forward=f, margin=min(f - lo, hi - f),
                    arm=solutions[0], waypoints=solutions, heading_offset=offset))
    # Prefer the direct heading, then the most central reach.
    results.sort(key=lambda p: (abs(p.heading_offset), -p.margin))
    return results


def fixed_link_height(urdf_xml: str, link: str = 'base_link') -> float:
    """Height of `link` above the URDF root with all joints at zero. The
    planar base joints carry no z, so this is base_link's height in world."""
    root = ET.fromstring(urdf_xml)
    parents = {}
    for element in root.findall('joint'):
        origin = element.find('origin')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        parents[element.find('child').get('link')] = (element.find('parent').get('link'), xyz, rpy)
    chain = []
    while link in parents:
        parent, xyz, rpy = parents[link]
        chain.append((xyz, rpy))
        link = parent
    t = np.eye(4)
    for xyz, rpy in reversed(chain):
        step = np.eye(4)
        step[:3, :3] = _rpy_matrix(*rpy)
        step[:3, 3] = xyz
        t = t @ step
    return float(t[2, 3])
