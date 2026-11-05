"""A simulated cell for tests: an arm holding a gripper over a table, a downward-looking RGB-D
camera, the motion service and the machine config in the cloud.

The gripper's real fingertips reach TRUE_REACH past its frame; its collision model (the xArm
gripper's boxes) only MODEL_REACH. Moving the real fingertips into the table raises Crash.
"""

from __future__ import annotations

import copy
from types import SimpleNamespace

import numpy as np
from viam.proto.common import Geometry, Pose, PoseInFrame, RectangularPrism, Vector3

from touchoff import geometry as geo

TABLE_Z = -10.0
TRUE_REACH = 74.0
MODEL_REACH = 50.0
# The gripper's frame in the arm's frame, as on armfarm12: 105mm out, turned 180 deg.
ARM_GRIPPER = geo.make_transform(geo.ov_to_matrix(0, 0, 1, 180), [0, 0, 105])


class Crash(RuntimeError):
    pass


def xarm_gripper_geometries():
    return [Geometry(center=Pose(z=-50, o_z=1), box=RectangularPrism(dims_mm=Vector3(x=50, y=100, z=100)), label="case"),
            Geometry(center=Pose(z=-2.5, o_z=1), box=RectangularPrism(dims_mm=Vector3(x=40, y=170, z=105)), label="claws")]


class Cell:
    def __init__(self, table_z=TABLE_Z, true_reach=TRUE_REACH, block=(420.0, 0.0, 60.0, 60.0, 80.0), in_fragment=False):
        self.table_z, self.true_reach = table_z, true_reach
        self.block = block  # x, y, size x, size y, height
        self.T_gripper = geo.pointing_down(350, -100, 400, 30)  # world <- gripper
        self.arm_gripper = ARM_GRIPPER.copy()  # the gripper frame in the arm's, as the frame system has it
        self.camera_offset = geo.make_transform(np.eye(3), [-80.0, 0.0, -150.0])  # a camera above and behind the gripper
        self.moves: list[tuple[np.ndarray, bool]] = []
        self.manual = False
        self.arm = FakeArm(self)
        self.gripper = FakeGripper()
        self.motion = FakeMotion(self)
        self.camera = FakeCamera(self)
        self.config = {"components": [
            {"name": "arm", "api": "rdk:component:arm", "model": "viam:ufactory:xArm6", "attributes": {"host": "10.0.0.2"}},
            {"name": "gripper", "api": "rdk:component:gripper", "model": "viam:ufactory:gripper",
             "attributes": {"arm": "arm"},
             "frame": {"parent": "arm", "translation": {"x": 0, "y": 0, "z": 105},
                       "orientation": {"type": "ov_degrees", "value": {"x": 0, "y": 0, "z": 1, "th": 180}}}},
        ]}
        self.fragment = None
        if in_fragment:  # as on armfarm12: the arm and gripper come from a fragment
            gripper = self.config["components"].pop(1)
            gripper["frame"]["parent"] = {"$variable": {"name": "arm-name"}}
            self.fragment = {"components": [self.config["components"].pop(0), gripper]}
            self.config["components"] = [{"name": "cam", "api": "rdk:component:camera", "model": "viam:camera:realsense"}]
            self.config["fragments"] = [{"id": "frag-1", "variables": {"arm-name": "arm"}}]
            self.config["fragment_mods"] = [{"fragment_id": "frag-1", "mods": [
                {"$set": {"components.arm.attributes.speed_degs_per_sec": 20}}]}]
        self.part_config = FakePartConfig(self)

    def config_translation(self) -> dict:
        """The gripper's translation as viam-server would resolve it from the config."""
        if self.fragment is None:
            return self.config["components"][1]["frame"]["translation"]
        translation = self.fragment["components"][1]["frame"]["translation"]
        for entry in self.config.get("fragment_mods", []):
            for mod in entry["mods"]:
                translation = mod.get("$set", {}).get("components.gripper.frame.translation", translation)
        return translation

    def tip_z(self, T=None) -> float:
        T = self.T_gripper if T is None else T
        return float((T[:3, 3] + self.true_reach * T[:3, 2])[2])

    def camera_pose(self) -> np.ndarray:
        # Looking straight down from above the gripper (its frame's +z down, like the gripper's).
        T = self.T_gripper @ self.camera_offset
        return geo.make_transform(geo.POINTING_DOWN, T[:3, 3])

    def apply_config_frame(self) -> None:
        """What viam-server does when the gripper's frame changes in the config: the frame system moves the gripper frame."""
        t = self.config_translation()
        new = geo.make_transform(ARM_GRIPPER[:3, :3], [t["x"], t["y"], t["z"]])
        T_world_arm = self.T_gripper @ np.linalg.inv(self.arm_gripper)
        # The real fingertips stay put on the arm; measured from the new origin they reach this much further:
        self.true_reach += (np.linalg.inv(new) @ self.arm_gripper)[2, 3]
        self.arm_gripper = new
        self.T_gripper = T_world_arm @ new


class FakeArm:
    name = "arm"

    def __init__(self, cell: Cell, manual_mode=True):
        self.cell, self.support_manual_mode, self.stops = cell, manual_mode, 0

    async def is_moving(self, **_):
        return False

    async def stop(self, **_):
        self.stops += 1

    async def get_properties(self, **_):
        return SimpleNamespace(support_manual_mode=self.support_manual_mode, support_cartesian_commands=True)

    async def set_manual_mode(self, manual_mode, enabled_for=0, **_):
        self.cell.manual = manual_mode


class FakeGripper:
    name = "gripper"

    def __init__(self, geometries=None):
        self.geometries = xarm_gripper_geometries() if geometries is None else geometries
        self.grabs = 0

    async def grab(self, **_):
        self.grabs += 1
        return False

    async def get_geometries(self, **_):
        return self.geometries


class FakeMotion:
    name = "builtin"

    def __init__(self, cell: Cell):
        self.cell = cell
        self.fail_moves = 0  # this many moves fail to plan

    async def get_pose(self, component_name, destination_frame, **_):
        if component_name == "gripper" and destination_frame == "world":
            T = self.cell.T_gripper
        elif component_name == "gripper" and destination_frame == "arm":
            t = self.cell.config_translation()
            T = geo.make_transform(ARM_GRIPPER[:3, :3], [t["x"], t["y"], t["z"]])
        elif component_name == "cam" and destination_frame == "world":
            T = self.cell.camera_pose()
        else:
            raise KeyError(f"no frame {component_name} in {destination_frame}")
        return PoseInFrame(reference_frame=destination_frame, pose=geo.matrix_to_pose(T))

    async def move(self, component_name, destination, world_state=None, constraints=None, **_):
        assert component_name == "gripper" and destination.reference_frame == "world"
        if self.fail_moves:
            self.fail_moves -= 1
            raise RuntimeError("no path: collision with touch-off-obstacle-0")
        T = geo.pose_to_matrix(destination.pose)
        linear = constraints is not None and len(constraints.linear_constraint) > 0
        if self.cell.tip_z(T) < self.cell.table_z - 1e-6:
            raise Crash(f"the fingertips hit the table: tip at z={self.cell.tip_z(T):.2f}")
        self.cell.moves.append((T, linear))
        self.cell.T_gripper = T
        return True


class FakeCamera:
    name = "cam"
    width, height, f = 320, 240, 260.0

    def __init__(self, cell: Cell):
        self.cell = cell

    async def get_properties(self, **_):
        return SimpleNamespace(intrinsic_parameters=SimpleNamespace(
            width_px=self.width, height_px=self.height, focal_x_px=self.f, focal_y_px=self.f,
            center_x_px=self.width / 2, center_y_px=self.height / 2))

    def depth(self, noise=0.6, seed=0) -> np.ndarray:
        T = self.cell.camera_pose()
        v, u = np.mgrid[0:self.height, 0:self.width]
        rays = np.stack([(u - self.width / 2) / self.f, (v - self.height / 2) / self.f, np.ones_like(u, float)], -1)
        world_dirs = rays @ T[:3, :3].T  # z-component of each is -1 (camera looks straight down)
        cam = T[:3, 3]
        depth = cam[2] - self.cell.table_z  # every ray's distance along the optical axis to the table
        depth = np.full(u.shape, depth, float)
        bx, by, sx, sy, h = self.cell.block
        top = self.cell.table_z + h
        hit = cam[:2] + (cam[2] - top) * world_dirs[..., :2]
        on_block = (np.abs(hit[..., 0] - bx) < sx / 2) & (np.abs(hit[..., 1] - by) < sy / 2)
        depth[on_block] = cam[2] - top
        depth += np.random.default_rng(seed).normal(0, noise, depth.shape)
        return np.clip(depth, 0, 65535).round().astype(np.uint16)

    async def get_images(self, filter_source_names=None, **_):
        d = self.depth(seed=len(self.cell.moves))
        data = b"DEPTHMAP" + d.shape[1].to_bytes(8, "big") + d.shape[0].to_bytes(8, "big") + d.astype(">u2").tobytes()
        color = SimpleNamespace(name="color", mime_type="image/jpeg", data=b"\xff\xd8")
        depth = SimpleNamespace(name="depth", mime_type="image/vnd.viam.dep", data=data)
        images = [color, depth]
        if filter_source_names:
            images = [i for i in images if i.name in filter_source_names]
        return images, None

    async def get_point_cloud(self, **_):
        raise NotImplementedError


class FakePartConfig:
    def __init__(self, cell: Cell):
        self.cell, self.writes, self.version = cell, [], 1

    async def read(self):
        return copy.deepcopy(self.cell.config), self.version

    async def fragment(self, fragment_id):
        assert fragment_id == "frag-1" and self.cell.fragment is not None
        return "xarm-gripper", copy.deepcopy(self.cell.fragment)

    async def write(self, config, last_updated=None):
        if last_updated != self.version:
            raise RuntimeError("config changed since it was read")
        self.cell.config = copy.deepcopy(config)
        self.writes.append(copy.deepcopy(config))
        self.version += 1
        self.cell.apply_config_frame()
