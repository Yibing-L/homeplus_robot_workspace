"""Depth geometry independent of ROS/model inference, with explicit rejection."""
import numpy as np


def fit_cup_center(points, min_radius=0.025, max_radius=0.065, diagnostics=None):
    """Fit an upright tapered cup body; reject ambiguous arcs.

    Returns center, radius, height. The support surface/rim are excluded from
    the fit. This supports upright circular tapered cups (including cylinders), not arbitrary
    containers; unsupported geometry must not become a confident grasp target.
    """
    def reject(reason):
        if diagnostics is not None:
            diagnostics['reason'] = reason
        return None

    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 80:
        return reject(f"too few valid depth points: {len(points)} < 80")
    low, high = np.quantile(points[:, 2], [0.05, 0.95])
    height = high - low
    if not 0.04 <= height <= 0.25:
        return reject(f"vertical extent {height:.4f} m outside [0.04, 0.25]")
    # Fit the outer body, excluding the rim/top opening and bottom edge.
    body = points[(points[:, 2] > low + height * 0.2) &
                  (points[:, 2] < low + height * 0.8)]
    if len(body) < 80:
        return reject(f"too few body points: {len(body)} < 80")
    from scipy.optimize import least_squares
    grasp_z = low + height * 0.65
    # Work near the origin to avoid poor conditioning at large world offsets.
    origin = np.median(body[:, :2], axis=0)
    xy = body[:, :2] - origin
    dz = body[:, 2] - grasp_z
    # Deterministic subsampling bounds inference cost without dropping spatial
    # coverage. Multiple seeds avoid the near-flat, enormous-radius solution.
    sample = np.random.default_rng(0).choice(len(body), min(len(body), 1500), replace=False)
    def residual(v, indices):
        return np.linalg.norm(xy[indices] - v[:2], axis=1) - v[2] - v[3] * dz[indices]
    best = None
    for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
        seed = [0.04*np.cos(angle), 0.04*np.sin(angle),
                (min_radius+max_radius)/2, 0.15]
        fit = least_squares(lambda v: residual(v, sample), seed,
            bounds=([-0.15, -0.15, min_radius, -0.05],
                    [0.15, 0.15, max_radius, 0.6]),
            loss='soft_l1', f_scale=0.002, max_nfev=150)
        if fit.success and (best is None or fit.cost < best.cost):
            best = fit
    if best is None:
        return reject("tapered body optimization did not converge")
    v = best.x
    errors = np.abs(residual(v, np.arange(len(body))))
    inliers = errors < 0.004
    fraction = float(inliers.mean())
    if fraction < 0.65:
        return reject(f"tapered body inlier fraction {fraction:.3f} < 0.65; "
                      f"radius={v[2]:.4f} m, taper={v[3]:.3f} m/m")
    # A single visible stripe or height band cannot establish a cup center.
    for zlo, zhi in zip(np.linspace(low+height*.2, low+height*.8, 4)[:-1],
                       np.linspace(low+height*.2, low+height*.8, 4)[1:]):
        band = inliers & (body[:, 2] >= zlo) & (body[:, 2] <= zhi)
        if band.sum() < 15:
            return reject("insufficient support across body height")
        angles = np.sort(np.mod(np.arctan2(xy[band, 1]-v[1], xy[band, 0]-v[0]), 2*np.pi))
        coverage = 2*np.pi - np.max(np.diff(np.r_[angles, angles[0]+2*np.pi]))
        if coverage < np.deg2rad(60):
            return reject("visible body arc below 60 degrees in a height band")
    if np.median(errors[inliers]) > 0.003:
        return reject("median tapered body residual exceeds 0.003 m")
    bottom_radius = v[2] + v[3] * (low-grasp_z)
    top_radius = v[2] + v[3] * (high-grasp_z)
    if min(bottom_radius, top_radius) < 0.01 or max(bottom_radius, top_radius) > 0.09:
        return reject("extrapolated cup dimensions outside supported bounds")
    if diagnostics is not None:
        diagnostics.update(grasp_radius=float(v[2]), bottom_radius=float(bottom_radius),
                           top_radius=float(top_radius), taper=float(v[3]),
                           inlier_fraction=fraction)
    # Return the enclosing radius for conservative collision geometry; the
    # local grasp radius is carried separately in diagnostics.
    return np.array([*(origin+v[:2]), grasp_z]), float(max(bottom_radius, top_radius)), float(height)


def fit_button_plane(points, ray):
    """Intersect the model's pointing ray with a robust local surface plane."""
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 20:
        return None
    median = np.median(points[:, 2])
    points = points[abs(points[:, 2] - median) < 0.02]
    if len(points) < 20:
        return None
    center = np.median(points, axis=0)
    _, singular, vh = np.linalg.svd(points - center, full_matrices=False)
    if singular[1] < 0.002:
        return None
    normal = vh[-1]
    if np.median(abs((points-center) @ normal)) > 0.003:
        return None
    denominator = np.dot(normal, ray)
    if abs(denominator) < 0.25:
        return None
    hit = np.asarray(ray) * np.dot(normal, center) / denominator
    if hit[2] <= 0 or np.linalg.norm(hit-center) > 0.04:
        return None
    if np.dot(normal, hit) < 0:
        normal = -normal  # +X approach points from camera into the face.
    return hit, normal


def offset_surface_goal(surface_world, camera_world, offset_m):
    """Shift a visible surface point horizontally away from the camera.

    The offset is a calibrated surface-to-grasp-center distance, not an
    inferred radius. Preserve observed height and reject degenerate views.
    """
    surface = np.asarray(surface_world, dtype=float)
    direction = surface - np.asarray(camera_world, dtype=float)
    direction[2] = 0.0
    length = np.linalg.norm(direction)
    if not np.isfinite(surface).all() or not np.isfinite(length) or length < 1e-6:
        raise ValueError('invalid surface point or vertical-only approach')
    if not np.isfinite(offset_m) or not 0 <= offset_m <= 0.10:
        raise ValueError('cup_surface_to_center_m must be between 0 and 0.10 m')
    direction /= length
    return surface + offset_m * direction, direction
