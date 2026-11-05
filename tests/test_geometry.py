import math
import struct

import numpy as np
import pytest
from viam.proto.common import Capsule, Geometry, Pose, RectangularPrism, Sphere, Vector3

from touchoff import geometry as geo
from sim import xarm_gripper_geometries


@pytest.mark.parametrize("ov", [(0, 0, 1, 180), (0.3, -0.2, 0.9, 37), (0, 0, -1, 25), (0.0068, -0.0057, 0.99996, 10), (1, 0, 0, -90)])
def test_orientation_vector_round_trip(ov):
    R = geo.ov_to_matrix(*ov)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(geo.ov_to_matrix(*geo.matrix_to_ov(R)), R, atol=1e-9)


def test_orientation_vector_matches_rdk_convention():
    # theta 180 about +z: the frame's x-axis points along world -x.
    np.testing.assert_allclose(geo.ov_to_matrix(0, 0, 1, 180), np.diag([-1.0, -1.0, 1.0]), atol=1e-12)
    # Pointing along +x, theta 0: z-axis along world +x.
    np.testing.assert_allclose(geo.ov_to_matrix(1, 0, 0, 0)[:, 2], [1, 0, 0], atol=1e-12)


def test_pointing_down_and_its_yaw():
    for yaw in (0.0, 37.0, -120.0, 179.0):
        T = geo.pointing_down(400, -300, 80, yaw)
        np.testing.assert_allclose(T[:3, 2], [0, 0, -1], atol=1e-12)
        assert np.linalg.det(T[:3, :3]) == pytest.approx(1.0)
        assert geo.yaw_of(T) == pytest.approx(yaw)
        assert geo.tilt_from_down_deg(T) == pytest.approx(0.0, abs=1e-6)


def test_tilt_from_down():
    T = geo.pointing_down(0, 0, 0, 0)
    T[:3, :3] = T[:3, :3] @ geo.rotation_z(0)  # still down
    tilted = geo.make_transform(geo.ov_to_matrix(math.sin(math.radians(5)), 0, -math.cos(math.radians(5)), 0), [0, 0, 0])
    assert geo.tilt_from_down_deg(tilted) == pytest.approx(5.0)


def test_modeled_reach_of_the_xarm_gripper_is_50mm():
    assert geo.modeled_reach(xarm_gripper_geometries()) == pytest.approx(50.0)


def test_modeled_reach_of_spheres_capsules_and_nothing():
    assert geo.modeled_reach([]) is None
    assert geo.modeled_reach([Geometry(center=Pose(z=30, o_z=1), sphere=Sphere(radius_mm=10))]) == pytest.approx(40)
    capsule = Geometry(center=Pose(z=50, o_z=1), capsule=Capsule(radius_mm=10, length_mm=100))
    assert geo.modeled_reach([capsule]) == pytest.approx(100)


def test_modeled_reach_of_a_turned_box():
    # A 100mm-long box lying along x, turned 90 deg so it points along z.
    box = Geometry(center=Pose(z=0, o_x=1, theta=0), box=RectangularPrism(dims_mm=Vector3(x=10, y=10, z=100)))
    assert geo.modeled_reach([box]) == pytest.approx(5)


def test_reach_from_contact_on_a_level_and_a_tilted_table():
    T = geo.pointing_down(300, 0, 64.0, 15)
    table = geo.TablePlane.horizontal(-10.0)
    assert geo.reach_from_contact(T, table) == pytest.approx(74.0)
    assert geo.reach_from_contact(T, table, gap_mm=0.1) == pytest.approx(73.9)
    # Tilted 1 deg about y: the table under the gripper is at -10 + 300 * tan(1 deg).
    normal = np.array([-math.sin(math.radians(1)), 0, math.cos(math.radians(1))])
    plane = geo.TablePlane(normal, -normal @ np.array([0, 0, -10.0]))
    expected = 64.0 - (-10 + 300 * math.tan(math.radians(1)))
    assert geo.reach_from_contact(T, plane) == pytest.approx(expected, abs=1e-6)
    assert geo.fingertip_gap(T, plane, expected) == pytest.approx(0.0, abs=1e-9)


def test_reach_from_contact_refuses_a_gripper_not_pointing_down():
    T = geo.make_transform(np.eye(3), [0, 0, 100])  # pointing up
    with pytest.raises(ValueError):
        geo.reach_from_contact(T, geo.TablePlane.horizontal(0))


def test_shifted_translation_follows_the_gripper_axis():
    R = geo.ov_to_matrix(0, 0, 1, 180)  # +z along the arm's +z
    assert geo.shifted_translation({"x": 0, "y": 0, "z": 105}, R, 24.0) == {"x": 0.0, "y": 0.0, "z": 129.0}
    R = geo.ov_to_matrix(1, 0, 0, 0)  # a gripper mounted sideways: +z along the arm's +x
    assert geo.shifted_translation({"x": 10, "z": 50}, R, -5.0) == {"x": 5.0, "y": 0.0, "z": 50.0}


def table_points(z=-10.0, noise=0.5, n=40000, seed=1):
    rng = np.random.default_rng(seed)
    xy = rng.uniform([0, -300], [600, 300], (n, 2))
    return np.column_stack([xy, z + rng.normal(0, noise, n)])


def test_fit_table_plane_finds_the_table_under_an_object():
    points = table_points()
    box = np.random.default_rng(2).uniform([280, -40, 50], [360, 40, 70], (8000, 3))  # an object's top
    plane, inliers = geo.fit_table_plane(np.vstack([points, box]))
    assert plane.z_at(300, 0) == pytest.approx(-10.0, abs=0.05)
    assert plane.tilt_deg < 0.05
    assert inliers[:len(points)].mean() > 0.99 and not inliers[len(points):].any()


def test_fit_table_plane_rejects_a_wall():
    wall = np.random.default_rng(3).uniform([0, 0, 0], [0.5, 500, 500], (5000, 3))
    with pytest.raises(ValueError):
        geo.fit_table_plane(wall)


def test_tilted_table_is_measured():
    points = table_points(noise=0.3)
    points[:, 2] += 0.02 * points[:, 0]  # rises 2mm per 100mm in x
    plane, _ = geo.fit_table_plane(points, max_tilt_deg=5)
    assert plane.tilt_deg == pytest.approx(math.degrees(math.atan(0.02)), abs=0.02)


def test_depth_decoding_and_backprojection():
    depth = np.array([[500, 0], [510, 1600]], np.uint16)
    data = b"DEPTHMAP" + (2).to_bytes(8, "big") + (2).to_bytes(8, "big") + depth.astype(">u2").tobytes()
    np.testing.assert_array_equal(geo.decode_viam_depth(data), depth)
    intrinsics = geo.Intrinsics(2, 2, 100.0, 100.0, 0.0, 0.0)
    points = geo.backproject(depth, intrinsics, 150, 1500, stride=1)
    np.testing.assert_allclose(points, [[0, 0, 500], [0, 5.1, 510]])  # 0 and 1600 are dropped


def test_png16_depth():
    import io

    from PIL import Image

    depth = np.array([[500, 1000], [65535, 0]], np.uint16)
    buffer = io.BytesIO()
    Image.fromarray(depth).save(buffer, format="PNG")
    np.testing.assert_array_equal(geo.decode_png16(buffer.getvalue()), depth)


def test_median_depth_ignores_missing_readings():
    frames = [np.array([[100, 0]], np.uint16), np.array([[102, 0]], np.uint16), np.array([[0, 7]], np.uint16)]
    np.testing.assert_array_equal(geo.median_depth(frames), [[101, 0]])


def test_parse_pcd_ascii_and_binary():
    header = "VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F I\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\n"
    ascii = (header + "DATA ascii\n0.1 0.2 0.5 0\n-0.1 0 0.6 0\n").encode()
    np.testing.assert_allclose(geo.parse_pcd(ascii), [[100, 200, 500], [-100, 0, 600]])
    binary = (header + "DATA binary\n").encode() + struct.pack("<fffi", 0.1, 0.2, 0.5, 0) + struct.pack("<fffi", -0.1, 0, 0.6, 0)
    np.testing.assert_allclose(geo.parse_pcd(binary), [[100, 200, 500], [-100, 0, 600]], atol=1e-4)
    with pytest.raises(ValueError):
        geo.parse_pcd((header + "DATA binary_compressed\n").encode())


def test_spots_avoid_objects_the_base_and_unseen_table():
    points = table_points(z=-10, noise=0.3)
    block = np.random.default_rng(4).uniform([280, -40, -10], [340, 20, 70], (6000, 3))
    points = np.vstack([points, block])
    plane, _ = geo.fit_table_plane(points)
    obstacles = geo.find_obstacles(points, plane)
    assert len(obstacles) == 1
    assert obstacles[0].center[:2] == pytest.approx([310, -10], abs=15)
    spots = geo.candidate_spots(points, plane, obstacles, clearance=80, min_base_distance=150)
    assert spots
    for x, y, z in spots:
        assert np.hypot(x - 310, y + 10) >= obstacles[0].radius + 80
        assert np.hypot(x, y) >= 150
        assert 0 + 40 <= x <= 600 - 40 and -300 + 40 <= y <= 300 - 40  # the whole disc was seen
        assert z == pytest.approx(-10, abs=0.1)
    first = spots[:3]
    assert min(np.linalg.norm(a[:2] - b[:2]) for i, a in enumerate(first) for b in first[i + 1:]) > 200


def test_summarize_finds_a_tilt_but_not_an_offset():
    flat = [{"reach_mm": 74.0, "tip_mm": xy + [-10]} for xy in ([400, -200], [100, -600], [500, -600])]
    result = geo.summarize(flat, 50.0)
    assert result["reach_mm"] == 74.0 and result["extra_reach_mm"] == 24.0
    assert result["table_tilt_vs_reference"]["deg"] == pytest.approx(0, abs=1e-9)
    tilted = [dict(t, reach_mm=74.0 + 0.01 * t["tip_mm"][0]) for t in flat]
    assert geo.summarize(tilted, 50.0)["table_tilt_vs_reference"]["mm_per_100mm"] == pytest.approx(1.0)
    assert "table_tilt_vs_reference" not in geo.summarize(flat[:2], None)
