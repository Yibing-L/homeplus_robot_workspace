"""Table plane fit used to model the table as an exact box (no ROS needed)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "homeplus_vision"))
from homeplus_vision.table_plane import fit_table_plane, table_box  # noqa: E402


def scene(rng, table_z=0.70, tilt_deg=0.0, n_table=2000, with_floor=True):
    t = math.radians(tilt_deg)
    xy = rng.uniform([0.6, -0.4], [1.4, 0.6], size=(n_table, 2))
    z = table_z + (xy[:, 0] - 1.0) * math.tan(t) + rng.normal(0, 0.003, n_table)
    pts = [np.column_stack((xy, z))]
    cup = np.column_stack((rng.normal(1.05, 0.02, 300), rng.normal(0.30, 0.02, 300),
                           rng.uniform(table_z, table_z + 0.11, 300)))
    machine = np.column_stack((rng.uniform(1.2, 1.35, 500), rng.uniform(0.0, 0.2, 500),
                               rng.uniform(table_z, table_z + 0.35, 500)))
    pts += [cup, machine]
    if with_floor:
        f = rng.uniform([0.3, -0.6], [1.5, 0.8], size=(3000, 2))
        pts.append(np.column_stack((f, rng.normal(0.0, 0.003, 3000))))
    return np.vstack(pts)


class TablePlaneTests(unittest.TestCase):
    def test_finds_table_not_floor(self):
        rng = np.random.default_rng(1)
        plane = fit_table_plane(scene(rng), z_min=0.4, z_max=1.2)
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane.height, 0.70, delta=0.005)
        self.assertLess(plane.tilt_deg, 1.0)

    def test_box_top_is_table_surface(self):
        rng = np.random.default_rng(2)
        pts = scene(rng)
        plane = fit_table_plane(pts, z_min=0.4, z_max=1.2)
        center, size = table_box(pts, plane, thickness=0.04)
        self.assertAlmostEqual(center[2] + size[2] / 2, 0.70, delta=0.005)
        self.assertGreater(size[0], 0.7)

    def test_cup_and_machine_are_not_removed(self):
        rng = np.random.default_rng(3)
        pts = scene(rng, with_floor=False)
        plane = fit_table_plane(pts, z_min=0.4, z_max=1.2)
        dist = plane.signed_distance(pts)
        band = (dist >= -0.04) & (dist <= 0.015)
        above = pts[~band]
        self.assertGreater(np.count_nonzero(above[:, 2] > 0.75), 200)  # cup + machine stay

    def test_tilted_view_is_rejected(self):
        # A bumped camera makes the table look tilted in world: refuse, keep voxels.
        rng = np.random.default_rng(4)
        self.assertIsNone(fit_table_plane(scene(rng, tilt_deg=15.0, with_floor=False),
                                          z_min=0.4, z_max=1.2, max_tilt_deg=4.0))

    def test_support_picks_surface_under_target(self):
        # Table at 0.70 up to x=1.15, a larger raised surface at 0.77 behind it (as on
        # the robot): unanchored RANSAC may bridge or pick the back level; anchored on
        # the cup, the fit is the table the cup stands on.
        rng = np.random.default_rng(6)
        front = rng.uniform([0.85, -0.5], [1.15, 0.6], size=(1200, 2))
        back = rng.uniform([1.15, -0.7], [1.6, 0.9], size=(2500, 2))
        pts = np.vstack((np.column_stack((front, 0.70 + rng.normal(0, 0.003, 1200))),
                         np.column_stack((back, 0.77 + rng.normal(0, 0.003, 2500)))))
        plane = fit_table_plane(pts, z_min=0.4, z_max=1.2, max_tilt_deg=4.0,
                                support=(1.0, 0.35, 0.705))
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane.height, 0.70, delta=0.005)
        self.assertLess(plane.tilt_deg, 1.0)

    def test_level_fit_ignores_noise_slope(self):
        # Depth noise that grows with range biases a 3D fit; the level fit gets the height.
        rng = np.random.default_rng(7)
        xy = rng.uniform([0.85, -0.5], [1.15, 0.6], size=(1500, 2))
        z = 0.70 + rng.normal(0, 0.004, 1500) * (xy[:, 0] / 0.85) ** 2
        back = np.column_stack((rng.uniform([1.15, -0.7], [1.6, 0.9], size=(2500, 2)),
                                0.77 + rng.normal(0, 0.003, 2500)))
        pts = np.vstack((np.column_stack((xy, z)), back))
        plane = fit_table_plane(pts, z_min=0.4, z_max=1.2, level=True,
                                support=(1.05, 0.35, 0.74))
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane.height, 0.70, delta=0.004)

    def test_no_table_in_view(self):
        rng = np.random.default_rng(5)
        floor_only = scene(rng)[-3000:]
        self.assertIsNone(fit_table_plane(floor_only, z_min=0.4, z_max=1.2))


if __name__ == "__main__":
    unittest.main()
