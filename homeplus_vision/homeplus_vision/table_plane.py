"""Find the table (support) plane in an obstacle cloud.

The OctoMap stores the table as 2 cm voxels, which inflates its top by up to a
voxel; with the cup's middle ~5 cm above the table, the gripper's fingertips then
"collide" with a table they physically clear. Fitting the plane lets the table be
removed from the voxel cloud and modelled as one exact collision box instead.

Pure NumPy so it can be tested without ROS.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class TablePlane:
    normal: np.ndarray      # unit normal, pointing up (+z)
    offset: float           # plane: normal . p + offset = 0
    height: float           # z of the plane at the inliers' centroid
    tilt_deg: float         # angle between normal and +z
    inliers: np.ndarray     # boolean mask over the input points

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        return points @ self.normal + self.offset


def fit_table_plane(points: np.ndarray, *, z_min: float, z_max: float,
                    max_tilt_deg: float = 10.0, inlier_tol: float = 0.01,
                    min_inliers: int = 200, iterations: int = 150,
                    min_width: float = 0.20,
                    support: Optional[tuple] = None,
                    level: bool = False,
                    rng: Optional[np.random.Generator] = None) -> Optional[TablePlane]:
    """RANSAC for the best-supported near-horizontal plane with height in [z_min, z_max].

    `points` are (N, 3) in a frame whose +z is up (base_link or world). Returns None
    when no plane with at least `min_inliers` points qualifies (e.g. the table is
    out of view), so callers leave the cloud untouched. The inliers must also span at
    least `min_width` across their narrowest direction: a horizontal slice through a
    tilted surface (e.g. the table seen by a bumped camera) is only a thin strip.

    `support=(x, y, z_bottom)` restricts the search to planes passing just below
    the target's bottom (within 8 cm: the mask rarely reaches the table) at its (x, y): the surface the object stands on, not a
    shelf or appliance base at another height elsewhere in view.

    `level=True` assumes the table is level in this frame (the robot stands on the
    same floor) and estimates only its height. At D405 range, depth noise makes
    small patches of a level table read several degrees of slope, so a full 3D fit
    lands centimetres off; the height alone is stable.
    """
    pts = np.asarray(points, dtype=np.float64)
    band = (pts[:, 2] >= z_min - 0.05) & (pts[:, 2] <= z_max + 0.05)
    candidates = pts[band]
    if len(candidates) < max(3, min_inliers):
        return None
    if level:
        return _fit_level(pts, candidates, z_min, z_max, inlier_tol, min_inliers,
                          min_width, support)
    rng = rng or np.random.default_rng(0)
    cos_tilt = np.cos(np.radians(max_tilt_deg))
    best_count, best = 0, None
    for _ in range(iterations):
        a, b, c = candidates[rng.choice(len(candidates), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[2] < 0:
            n = -n
        if n[2] < cos_tilt:
            continue
        d = -float(n @ a)
        if support is not None and not _supports(n, d, support):
            continue
        count = int(np.count_nonzero(np.abs(candidates @ n + d) <= inlier_tol))
        if count > best_count:
            best_count, best = count, (n, d)
    if best is None or best_count < min_inliers:
        return None
    # Refine with a least-squares fit on the inliers.
    n, d = best
    inl = np.abs(candidates @ n + d) <= inlier_tol
    centroid = candidates[inl].mean(axis=0)
    _, _, vt = np.linalg.svd(candidates[inl] - centroid, full_matrices=False)
    n = vt[-1]
    if n[2] < 0:
        n = -n
    d = -float(n @ centroid)
    height = float(centroid[2])
    if not (z_min <= height <= z_max) or n[2] < cos_tilt:
        return None
    if support is not None and not _supports(n, d, support):
        return None
    xy = candidates[inl][:, :2] - centroid[:2]
    minor_std = float(np.linalg.svd(xy, compute_uv=False)[-1]) / np.sqrt(len(xy))
    # A uniform strip of width w has std w / sqrt(12).
    if minor_std * np.sqrt(12.0) < min_width:
        return None
    all_inliers = np.abs(pts @ n + d) <= inlier_tol
    tilt = float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0))))
    return TablePlane(normal=n, offset=d, height=height, tilt_deg=tilt, inliers=all_inliers)


def _fit_level(pts, candidates, z_min, z_max, inlier_tol, min_inliers, min_width, support):
    up = np.array([0.0, 0.0, 1.0])
    z = candidates[:, 2]
    lo, hi = z_min, z_max
    if support is not None:
        lo, hi = max(lo, support[2] - 0.08), min(hi, support[2] + 0.015)
    if hi < lo:
        return None
    heights = np.arange(lo, hi + 1e-9, 0.002)
    counts = [int(np.count_nonzero(np.abs(z - h) <= inlier_tol)) for h in heights]
    best = int(np.argmax(counts))
    if counts[best] < min_inliers:
        return None
    if support is not None:
        # The surface an object stands on is the first level surface below it, not
        # the largest one in the band: a robot deck/shelf ~8 cm lower out-voted the
        # strip of table beside the cup (0.62 vs 0.70 m, 2026-10-30). Highest local
        # peak with enough support.
        peaks = [i for i in range(len(counts)) if counts[i] >= min_inliers
                 and (i == 0 or counts[i] >= counts[i - 1])
                 and (i == len(counts) - 1 or counts[i] >= counts[i + 1])]
        if peaks:
            best = max(peaks, key=lambda i: heights[i])
    height = float(np.median(z[np.abs(z - heights[best]) <= inlier_tol]))
    inl = np.abs(z - height) <= inlier_tol
    xy = candidates[inl][:, :2] - candidates[inl][:, :2].mean(axis=0)
    minor_std = float(np.linalg.svd(xy, compute_uv=False)[-1]) / np.sqrt(len(xy))
    if minor_std * np.sqrt(12.0) < min_width:
        return None
    return TablePlane(normal=up, offset=-height, height=height, tilt_deg=0.0,
                      inliers=np.abs(pts[:, 2] - height) <= inlier_tol)


def _supports(n, d, support, below=0.08, above=0.015):
    """Whether plane (n, d) lies just below the target bottom at the target's x, y."""
    x, y, bottom = support
    z = -(d + n[0] * x + n[1] * y) / n[2]
    return bottom - below <= z <= bottom + above


def table_box(points_world: np.ndarray, plane: TablePlane, *, thickness: float = 0.04,
              margin: float = 0.02):
    """Axis-aligned box (center, size) whose top face is the fitted table surface."""
    inl = np.asarray(points_world)[plane.inliers]
    lo = inl[:, :2].min(axis=0) - margin
    hi = inl[:, :2].max(axis=0) + margin
    top = float(np.median(inl[:, 2]))
    center = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, top - thickness / 2)
    size = (float(hi[0] - lo[0]), float(hi[1] - lo[1]), thickness)
    return center, size
