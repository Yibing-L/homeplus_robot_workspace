"""Arm kinematics for teleop, from the robot's own URDF (pure numpy, no ROS).

The arm is planar: shoulder, elbow and hand pitch rotate about axes parallel to base_link Y, so
with wrist roll at 0 the grasp TCP moves in base_link's X-Z plane. Wrist roll (about the forearm)
sits between elbow and hand: with roll != 0 the hand pitch tilts out of that plane.

Teleop controls three task coordinates with the three in-plane joints (wrist roll is set directly):
    reach  = TCP x in base_link (forward)
    height = TCP z in base_link (up)
    pitch  = elevation of the TCP x axis (the approach direction) above the horizontal
Lateral TCP position (base_link y) is not controlled by the arm; the base strafes for that.
With roll != 0 the TCP's y is left free and drifts by up to a few cm as the hand pitches.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ARM_IK_JOINTS = ("joint_shoulder", "joint_elbow", "joint_hand")


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis: np.ndarray, a: float) -> np.ndarray:
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


@dataclass
class _Joint:
    name: str
    type: str
    origin: np.ndarray        # 4x4
    axis: np.ndarray
    lower: float
    upper: float


class ArmChain:
    """The serial chain base_link → tip from a URDF string."""

    def __init__(self, urdf_xml: str, base: str = "base_link", tip: str = "grasp_tcp"):
        root = ET.fromstring(urdf_xml)
        self.links = {l.get("name") for l in root.findall("link")}
        by_child: Dict[str, Tuple[str, _Joint]] = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            T = np.eye(4)
            T[:3, :3] = _rpy(*rpy)
            T[:3, 3] = xyz
            a = j.find("axis")
            axis = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()])
            if np.linalg.norm(axis) > 0:
                axis = axis / np.linalg.norm(axis)
            lim = j.find("limit")
            lo = float(lim.get("lower", "-inf")) if lim is not None else -math.inf
            hi = float(lim.get("upper", "inf")) if lim is not None else math.inf
            by_child[j.find("child").get("link")] = (j.find("parent").get("link"),
                                                      _Joint(j.get("name"), j.get("type"), T, axis, lo, hi))
        chain: List[_Joint] = []
        link = tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"no chain from {base} to {tip} (stuck at {link})")
            parent, joint = by_child[link]
            chain.append(joint)
            link = parent
        self.joints = list(reversed(chain))
        self.names = [j.name for j in self.joints if j.type != "fixed"]

    def fk(self, q: Dict[str, float]) -> np.ndarray:
        """4x4 pose of the tip in the base frame."""
        T = np.eye(4)
        for j in self.joints:
            T = T @ j.origin
            if j.type in ("revolute", "continuous"):
                R = np.eye(4)
                R[:3, :3] = _axis_angle(j.axis, q.get(j.name, 0.0))
                T = T @ R
            elif j.type == "prismatic":
                P = np.eye(4)
                P[:3, 3] = j.axis * q.get(j.name, 0.0)
                T = T @ P
        return T

    def limits(self, name: str) -> Tuple[float, float]:
        j = next(j for j in self.joints if j.name == name)
        return j.lower, j.upper


def task(T: np.ndarray) -> np.ndarray:
    """(reach, height, pitch) of a TCP pose in base_link."""
    ax = T[:3, 0]
    return np.array([T[0, 3], T[2, 3], math.atan2(ax[2], math.hypot(ax[0], ax[1]))])


PITCH_WEIGHT = 0.10   # metres per radian when mixing position and pitch errors


@dataclass
class IKResult:
    q: Dict[str, float]
    residual_m: float      # remaining position error
    residual_rad: float    # remaining pitch error
    ok: bool


def solve(chain: ArmChain, q0: Dict[str, float], target: Sequence[float],
          bounds: Optional[Dict[str, Tuple[float, float]]] = None,
          iters: int = 12, damping: float = 0.02, tol_m: float = 2e-5, tol_rad: float = math.radians(0.02)) -> IKResult:
    """Damped least squares on (shoulder, elbow, hand) toward (reach, height, pitch). Other joints
    (wrist roll) stay as given. Bounds clamp every iterate, so the result is always inside them;
    a target out of reach returns the closest pose found and ok=False."""
    q = dict(q0)
    tgt = np.asarray(target, dtype=float)
    W = np.diag([1.0, 1.0, PITCH_WEIGHT])
    bounds = bounds or {n: chain.limits(n) for n in ARM_IK_JOINTS}

    def err(qq):
        e = tgt - task(chain.fk(qq))
        e[2] = (e[2] + math.pi) % (2 * math.pi) - math.pi
        return e

    e = err(q)
    for _ in range(iters):
        if math.hypot(e[0], e[1]) < tol_m and abs(e[2]) < tol_rad:
            break
        J = np.zeros((3, 3))
        h = 1e-6
        base = task(chain.fk(q))
        for i, n in enumerate(ARM_IK_JOINTS):
            qq = dict(q)
            qq[n] += h
            d = task(chain.fk(qq)) - base
            d[2] = (d[2] + math.pi) % (2 * math.pi) - math.pi
            J[:, i] = d / h
        Jw, ew = W @ J, W @ e
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + damping ** 2 * np.eye(3), ew)
        step = float(np.max(np.abs(dq)))
        if step > 0.2:                      # keep each iterate local (0.2 rad)
            dq *= 0.2 / step
        for i, n in enumerate(ARM_IK_JOINTS):
            lo, hi = bounds[n]
            q[n] = min(hi, max(lo, q[n] + float(dq[i])))
        e_new = err(q)
        if np.linalg.norm(W @ e_new) > np.linalg.norm(W @ e) + 1e-9 and step < 1e-6:
            break
        e = e_new
    ok = math.hypot(e[0], e[1]) < max(2 * tol_m, 5e-4) and abs(e[2]) < max(2 * tol_rad, math.radians(0.3))
    return IKResult(q, math.hypot(e[0], e[1]), abs(e[2]), ok)
