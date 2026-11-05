"""Pure geometry for the touch-off: poses, the table plane, contact spots and the gripper's reach.

Poses are 4x4 homogeneous matrices in millimeters mapping points from a child frame into
its parent (usually the world). Viam describes orientation with an orientation vector
(o_x, o_y, o_z, theta in degrees): the frame's z-axis points along (o_x, o_y, o_z) and theta
spins the frame about that axis, i.e. R = Rz(lon) @ Ry(lat) @ Rz(theta) as in rdk's
spatialmath. A gripper points along its own +z axis.

Nothing in this file talks to a robot, so all of it is unit tested.
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from itertools import product
from typing import Iterable, Sequence

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

# rdk spatialmath orientationVectorPoleRadius: 1 - |o_z| within this is "at the pole", where
# rdk measures theta as if the longitude were zero (theta is then the total turn about z).
_POLE_RADIUS = 1e-4

# Gripper +z pointing straight down (world -z), its x-axis along world +x.
POINTING_DOWN = np.diag([1.0, -1.0, -1.0])


# --- Poses -----------------------------------------------------------------------------------

def _pole_sign(o_z: float) -> int:
    return int(np.sign(o_z)) if 1 - abs(o_z) <= _POLE_RADIUS else 0


def ov_to_matrix(o_x: float, o_y: float, o_z: float, theta_deg: float) -> np.ndarray:
    axis = np.array([o_x, o_y, o_z], dtype=float)
    norm = np.linalg.norm(axis)
    axis = axis / norm if norm > 0 else np.array([0.0, 0.0, 1.0])
    lat = math.acos(np.clip(axis[2], -1.0, 1.0))
    lon = math.atan2(axis[1], axis[0])
    theta = math.radians(theta_deg) - _pole_sign(axis[2]) * lon
    return Rotation.from_euler("ZYZ", [lon, lat, theta]).as_matrix()


def matrix_to_ov(rotation: np.ndarray) -> tuple[float, float, float, float]:
    axis = rotation[:, 2] / np.linalg.norm(rotation[:, 2])
    lat = math.acos(np.clip(axis[2], -1.0, 1.0))
    lon = math.atan2(axis[1], axis[0])
    spin = Rotation.from_euler("ZY", [lon, lat]).as_matrix().T @ rotation
    theta = math.atan2(spin[1, 0], spin[0, 0]) + _pole_sign(axis[2]) * lon
    theta = math.degrees(math.atan2(math.sin(theta), math.cos(theta)))
    return float(axis[0]), float(axis[1]), float(axis[2]), theta


def make_transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = list(translation)
    return transform


def pose_to_matrix(pose) -> np.ndarray:
    """A viam.proto.common.Pose (or anything with its fields) as a 4x4 matrix."""
    return make_transform(ov_to_matrix(pose.o_x, pose.o_y, pose.o_z, pose.theta), [pose.x, pose.y, pose.z])


def matrix_to_pose(transform: np.ndarray):
    from viam.proto.common import Pose

    o_x, o_y, o_z, theta = matrix_to_ov(transform[:3, :3])
    x, y, z = (float(v) for v in transform[:3, 3])
    return Pose(x=x, y=y, z=z, o_x=o_x, o_y=o_y, o_z=o_z, theta=theta)


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def rotation_z(angle_deg: float) -> np.ndarray:
    return Rotation.from_euler("z", angle_deg, degrees=True).as_matrix()


def pointing_down(x: float, y: float, z: float, yaw_deg: float) -> np.ndarray:
    """Gripper frame at (x, y, z) with its pointing axis (+z) straight down, turned `yaw_deg` about it."""
    return make_transform(POINTING_DOWN @ rotation_z(yaw_deg), [x, y, z])


def yaw_of(T: np.ndarray) -> float:
    """The yaw that `pointing_down` would need to match T's x-axis (in degrees)."""
    x_axis = T[:3, 0]
    return math.degrees(math.atan2(-x_axis[1], x_axis[0]))


def tilt_from_down_deg(T: np.ndarray) -> float:
    """Angle between the gripper's pointing axis (+z) and straight down."""
    return math.degrees(math.acos(float(np.clip(-T[2, 2], -1.0, 1.0))))


# --- The table -------------------------------------------------------------------------------

@dataclass
class TablePlane:
    normal: np.ndarray  # unit, pointing up
    offset: float  # normal @ p + offset == 0 on the table

    def height(self, points: np.ndarray) -> np.ndarray:
        """Signed height of each point above the table (mm)."""
        return points @ self.normal + self.offset

    def z_at(self, x: float, y: float) -> float:
        a, b, c = self.normal
        return float(-(a * x + b * y + self.offset) / c)

    @property
    def tilt_deg(self) -> float:
        return math.degrees(math.acos(float(np.clip(self.normal[2], -1.0, 1.0))))

    @classmethod
    def horizontal(cls, z: float) -> "TablePlane":
        return cls(np.array([0.0, 0.0, 1.0]), -float(z))

    def to_dict(self) -> dict:
        return {"normal": [float(v) for v in self.normal], "offset": float(self.offset)}


def _plane_through(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    """Least-squares plane (unit normal pointing up, offset) through points."""
    center = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points - center, full_matrices=False)
    if singular[1] < 1e-9:  # collinear
        return None
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    return normal, float(-normal @ center)


def fit_table_plane(points: np.ndarray, distance: float = 4.0, max_tilt_deg: float = 10.0,
                    iterations: int = 400, min_points: int = 500, seed: int = 0) -> tuple[TablePlane, np.ndarray]:
    """RANSAC the near-horizontal plane with the most points, refined by least squares on its inliers.

    Returns (plane, inlier mask). Raises ValueError when no plane within `max_tilt_deg`
    of horizontal holds at least `min_points` points.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < min_points:
        raise ValueError(f"Only {len(points)} depth points: too few to find the table")
    rng = np.random.default_rng(seed)
    sample = points if len(points) <= 20000 else points[rng.choice(len(points), 20000, replace=False)]
    min_up = math.cos(math.radians(max_tilt_deg))
    best_count, best = 0, None
    for _ in range(iterations):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < min_up:
            continue
        count = int((np.abs(sample @ normal - normal @ a) < distance).sum())
        if count > best_count:
            best_count, best = count, (normal, float(-normal @ a))
    if best is None:
        raise ValueError(f"No plane within {max_tilt_deg:g} deg of horizontal in the depth image")
    normal, offset = best
    for _ in range(2):  # refine: refit to all inliers, then re-select them
        inliers = np.abs(points @ normal + offset) < distance
        refit = _plane_through(points[inliers])
        if refit is None:
            break
        normal, offset = refit
    inliers = np.abs(points @ normal + offset) < distance
    plane = TablePlane(normal, offset)
    if inliers.sum() < min_points or plane.tilt_deg > max_tilt_deg:
        raise ValueError(f"No plane within {max_tilt_deg:g} deg of horizontal with {min_points}+ points")
    return plane, inliers


# --- Depth -----------------------------------------------------------------------------------

_DEPTH_HEADER = b"DEPTHMAP"


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_viam(cls, params) -> "Intrinsics | None":
        if params is None or not params.focal_x_px or not params.focal_y_px:
            return None
        return cls(params.width_px, params.height_px, params.focal_x_px, params.focal_y_px,
                   params.center_x_px, params.center_y_px)


def decode_viam_depth(data: bytes) -> np.ndarray:
    """Viam's image/vnd.viam.dep: 'DEPTHMAP', int64 width, int64 height, uint16 mm pixels (big-endian)."""
    if not data.startswith(_DEPTH_HEADER):
        raise ValueError("Not a Viam depth map")
    width = int.from_bytes(data[8:16], "big")
    height = int.from_bytes(data[16:24], "big")
    return np.frombuffer(data, ">u2", offset=24, count=width * height).reshape(height, width).astype(np.uint16)


def decode_png16(data: bytes) -> np.ndarray:
    """A depth image sent as a single-channel 16-bit PNG in millimeters (some camera modules do this)."""
    import io

    from PIL import Image

    with Image.open(io.BytesIO(data)) as image:
        if image.mode not in ("I;16", "I;16B", "I;16L", "I"):
            raise ValueError(f"Depth PNG is mode {image.mode}, expected 16-bit grayscale")
        depth = np.array(image)
    return depth.astype(np.uint16)


def median_depth(frames: Sequence[np.ndarray]) -> np.ndarray:
    """Per-pixel median over the frames' valid (non-zero) readings; 0 where fewer than half are valid."""
    stack = np.stack(frames).astype(np.float32)
    valid = stack > 0
    stack[~valid] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # pixels with no reading in any frame
        result = np.nanmedian(stack, axis=0)
    result[valid.mean(axis=0) < 0.5] = 0
    return np.nan_to_num(result).round().astype(np.uint16)


def backproject(depth_mm: np.ndarray, intrinsics: Intrinsics, min_depth: float, max_depth: float,
                stride: int = 2) -> np.ndarray:
    """Camera-frame points (mm, OpenCV convention: +z forward) from a depth image."""
    depth = depth_mm[::stride, ::stride].astype(float)
    v, u = np.mgrid[0:depth_mm.shape[0]:stride, 0:depth_mm.shape[1]:stride]
    # Intrinsics may describe a different resolution than the depth image (e.g. color vs depth).
    sx = depth_mm.shape[1] / intrinsics.width if intrinsics.width else 1.0
    sy = depth_mm.shape[0] / intrinsics.height if intrinsics.height else 1.0
    fx, fy, cx, cy = intrinsics.fx * sx, intrinsics.fy * sy, intrinsics.cx * sx, intrinsics.cy * sy
    keep = (depth >= min_depth) & (depth <= max_depth)
    z = depth[keep]
    return np.column_stack([(u[keep] - cx) * z / fx, (v[keep] - cy) * z / fy, z])


def parse_pcd(data: bytes) -> np.ndarray:
    """XYZ points (mm) from a PCD file (ascii or binary), as Viam's GetPointCloud returns (in meters)."""
    header_end, fields, sizes, types, counts, n_points, encoding = 0, [], [], [], [], 0, ""
    offset = 0
    for raw in data.split(b"\n"):
        offset += len(raw) + 1
        line = raw.decode("ascii", "replace").strip()
        if not line or line.startswith("#"):
            continue
        key, *values = line.split()
        key = key.upper()
        if key == "FIELDS":
            fields = values
        elif key == "SIZE":
            sizes = [int(v) for v in values]
        elif key == "TYPE":
            types = values
        elif key == "COUNT":
            counts = [int(v) for v in values]
        elif key == "POINTS":
            n_points = int(values[0])
        elif key == "DATA":
            encoding = values[0].lower()
            header_end = offset
            break
    if not encoding or not all(f in fields for f in "xyz"):
        raise ValueError("Not a PCD point cloud with x, y and z")
    counts = counts or [1] * len(fields)
    if encoding == "ascii":
        rows = np.loadtxt(data[header_end:].decode("ascii").splitlines(), ndmin=2)
        xyz = rows[:, [fields.index(f) for f in "xyz"]]
    elif encoding == "binary":
        kinds = {"F": "f", "I": "i", "U": "u"}
        dtype = np.dtype([(name, f"<{kinds[t]}{s}", (c,) if c > 1 else ())
                          for name, s, t, c in zip(fields, sizes, types, counts)])
        cloud = np.frombuffer(data, dtype, count=n_points, offset=header_end)
        xyz = np.column_stack([cloud[f].astype(float) for f in "xyz"])
    else:
        raise ValueError(f"PCD encoding {encoding!r} is not supported (ascii or binary only)")
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz * 1000.0  # rdk writes PCD in meters


# --- Contact spots ---------------------------------------------------------------------------

@dataclass
class Obstacle:
    """Something standing on the table, as an axis-aligned box in the world (mm)."""
    center: np.ndarray
    size: np.ndarray

    @property
    def radius(self) -> float:
        return float(np.linalg.norm(self.size[:2]) / 2)


def find_obstacles(points: np.ndarray, plane: TablePlane, min_height: float = 10.0,
                   cell: float = 20.0, min_points: int = 30) -> list[Obstacle]:
    """Boxes around everything standing more than `min_height` above the table, from a
    coarse occupancy grid: neighboring occupied cells form one obstacle."""
    above = points[plane.height(points) > min_height]
    if len(above) == 0:
        return []
    ij = np.floor(above[:, :2] / cell).astype(int)
    lo = ij.min(axis=0)
    shape = tuple(ij.max(axis=0) - lo + 1)
    counts = np.zeros(shape, int)
    np.add.at(counts, tuple((ij - lo).T), 1)
    labels, n = ndimage.label(counts >= 3, structure=np.ones((3, 3)))
    point_labels = labels[tuple((ij - lo).T)]
    obstacles = []
    for k in range(1, n + 1):
        group = above[point_labels == k]
        if len(group) < min_points:
            continue
        top = group[:, 2].max()
        low = min(plane.z_at(*group[:, :2].mean(axis=0)), group[:, 2].min())
        xy_lo, xy_hi = group[:, :2].min(axis=0), group[:, :2].max(axis=0)
        center = np.array([*(xy_lo + xy_hi) / 2, (top + low) / 2])
        size = np.array([*(xy_hi - xy_lo + cell), top - low])
        obstacles.append(Obstacle(center, size))
    return obstacles


def bare_table(points: np.ndarray, plane: TablePlane, radius: float = 40.0, spacing: float = 5.0,
               tolerance: float = 3.0):
    """A test for "the camera saw bare table all over this disc" (so a spot isn't off the table's
    edge, or somewhere the camera never looked). Table means within `tolerance` of the plane."""
    table_xy = points[np.abs(plane.height(points)) < tolerance][:, :2]
    table_xy = np.unique(np.round(table_xy / spacing), axis=0) * spacing
    tree = cKDTree(table_xy) if len(table_xy) else None
    u, v = np.meshgrid(np.arange(-radius, radius + spacing, spacing), np.arange(-radius, radius + spacing, spacing))
    disc = np.column_stack([u.ravel(), v.ravel()])
    disc = disc[np.linalg.norm(disc, axis=1) <= radius]

    def covered(xy: np.ndarray) -> bool:
        if tree is None:
            return False
        gaps, _ = tree.query(disc + xy, distance_upper_bound=spacing)
        return bool(np.isfinite(gaps).mean() >= 0.95)  # a few missing readings are normal
    return covered


def candidate_spots(points: np.ndarray, plane: TablePlane, obstacles: Sequence[Obstacle],
                    clearance: float = 80.0, spot_radius: float = 40.0, spacing: float = 20.0,
                    min_base_distance: float = 150.0) -> list[np.ndarray]:
    """Bare-table spots (x, y, table z) the camera saw, at least `clearance` from everything
    standing on the table and `min_base_distance` from the arm's base (the world origin),
    in spread order (see `spread_order`)."""
    table = points[np.abs(plane.height(points)) < 3.0]
    if len(table) == 0:
        return []
    (x0, y0), (x1, y1) = table[:, :2].min(axis=0), table[:, :2].max(axis=0)
    xs, ys = np.meshgrid(np.arange(x0 + spot_radius, x1 - spot_radius + 1e-9, spacing),
                         np.arange(y0 + spot_radius, y1 - spot_radius + 1e-9, spacing))
    on_table = bare_table(points, plane, spot_radius)
    spots = []
    for xy in np.column_stack([xs.ravel(), ys.ravel()]):
        if np.linalg.norm(xy) < min_base_distance:
            continue
        if any(np.linalg.norm(xy - o.center[:2]) < o.radius + clearance for o in obstacles):
            continue
        if not on_table(xy):
            continue
        spots.append(np.array([xy[0], xy[1], plane.z_at(*xy)]))
    return spread_order(spots)


def spread_order(spots: list[np.ndarray]) -> list[np.ndarray]:
    """Spots reordered so that each is as far as possible from those before it, starting with
    the one nearest the middle: the first few are well spread."""
    if not spots:
        return []
    xy = np.array([s[:2] for s in spots])
    first = int(np.argmin(np.linalg.norm(xy - xy.mean(axis=0), axis=1)))
    order = [first]
    nearest = np.linalg.norm(xy - xy[first], axis=1)
    nearest[first] = -1.0
    for _ in range(len(spots) - 1):
        k = int(np.argmax(nearest))
        order.append(k)
        nearest = np.minimum(nearest, np.linalg.norm(xy - xy[k], axis=1))
        nearest[order] = -1.0
    return [spots[k] for k in order]


# --- The gripper's reach ---------------------------------------------------------------------

def modeled_reach(geometries) -> float | None:
    """How far past the gripper frame's origin, along its pointing axis (+z), the gripper's
    collision model reaches; None when it has no geometry this can measure.

    `geometries` are viam.proto.common.Geometry, posed in the gripper's frame.
    """
    reach = None
    for geometry in geometries:
        center = pose_to_matrix(geometry.center)
        kind = geometry.WhichOneof("geometry_type")
        if kind == "box":
            dims = geometry.box.dims_mm
            half = np.array([dims.x, dims.y, dims.z]) / 2
            tips = [float((center[:3, :3] @ (half * signs) + center[:3, 3])[2]) for signs in product((-1, 1), repeat=3)]
        elif kind == "sphere":
            tips = [float(center[2, 3] + geometry.sphere.radius_mm)]
        elif kind == "capsule":
            capsule = geometry.capsule
            half_axis = center[:3, 2] * max(capsule.length_mm / 2 - capsule.radius_mm, 0.0)
            tips = [float(center[2, 3] + s * half_axis[2] + capsule.radius_mm) for s in (-1, 1)]
        else:  # meshes and point clouds: not measured here
            continue
        reach = max(tips) if reach is None else max(reach, *tips)
    return reach


def reach_from_contact(T_world_gripper: np.ndarray, plane: TablePlane, gap_mm: float = 0.0) -> float:
    """Fingertip reach along the gripper's pointing axis, from a pose where the fingertips
    touch the table (or rest `gap_mm` above it, e.g. on a sheet of paper).

    The fingertip point is p + r*a (a: the pointing axis); it lies `gap_mm` above the plane:
    n.(p + r a) + d = gap  =>  r = (gap - n.p - d) / (n.a)
    """
    p, a = T_world_gripper[:3, 3], T_world_gripper[:3, 2]
    along = float(plane.normal @ a)
    if along > -0.5:
        raise ValueError("The gripper isn't pointing down at the table")
    return float((gap_mm - plane.height(p[None])[0]) / along)


def fingertip_gap(T_world_gripper: np.ndarray, plane: TablePlane, reach: float) -> float:
    """Height of the fingertips (reach mm along the pointing axis) above the table."""
    tip = T_world_gripper[:3, 3] + reach * T_world_gripper[:3, 2]
    return float(plane.height(tip[None])[0])


def shifted_translation(translation: dict, R_parent_gripper: np.ndarray, delta_mm: float) -> dict:
    """The gripper frame's translation (in its parent frame) moved `delta_mm` along the
    gripper's own pointing axis, which moves its collision model out by that much."""
    t = np.array([float(translation.get(k, 0.0)) for k in "xyz"])
    t = t + R_parent_gripper[:3, 2] * delta_mm
    return {k: round(float(v), 3) for k, v in zip("xyz", t)}


def summarize(touches: Sequence[dict], modeled: float | None) -> dict:
    """The touch-off's result from its contacts: the median reach, its spread, how far it is
    beyond the collision model, and (with 3+ spots) the tilt between the touched table and the
    table plane used (camera or configured)."""
    reaches = np.array([t["reach_mm"] for t in touches])
    reach = float(np.median(reaches))
    result = {
        "reach_mm": round(reach, 2),
        "reach_spread_mm": round(float(np.ptp(reaches)), 2),
        "touches": len(touches),
        "modeled_reach_mm": None if modeled is None else round(float(modeled), 2),
        "extra_reach_mm": None if modeled is None else round(reach - modeled, 2),
    }
    spots = np.array([t["tip_mm"][:2] for t in touches])
    if len(touches) >= 3 and np.linalg.matrix_rank(spots - spots.mean(axis=0)) == 2:
        A = np.column_stack([spots, np.ones(len(spots))])
        (a, b, _), *_ = np.linalg.lstsq(A, reaches, rcond=None)
        slope = float(np.hypot(a, b))
        result["table_tilt_vs_reference"] = {"deg": round(math.degrees(math.atan(slope)), 3),
                                             "mm_per_100mm": round(100 * slope, 3)}
    return result
