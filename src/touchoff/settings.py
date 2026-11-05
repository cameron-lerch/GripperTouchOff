"""The service's attributes (its JSON config in the Viam app), with defaults and validation."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Mapping


@dataclass
class Settings:
    # Resources (names in the machine config).
    arm: str = ""
    gripper: str = ""
    motion: str = "builtin"
    camera: str = ""  # an RGB-D camera; optional, used by measure_table
    depth_source: str = ""  # the camera's depth image source name ("" = pick the depth image by its type)

    # The table, when no camera measures it: its height in the world frame (mm).
    table_z_mm: float | None = None

    # Camera table measurement.
    depth_frames: int = 5  # per-pixel median of this many depth frames
    min_depth_mm: float = 150.0
    max_depth_mm: float = 1500.0
    table_fit_tolerance_mm: float = 4.0
    max_table_tilt_deg: float = 10.0
    obstacle_height_mm: float = 10.0  # points this far above the table are things standing on it
    spot_clearance_mm: float = 80.0  # a contact spot stays this far from anything on the table
    min_base_distance_mm: float = 150.0  # ...and this far (horizontally) from the arm's base

    # Jog touch-off.
    travel_height_mm: float = 150.0  # fingertips this high above the table while moving to a spot
    start_gap_mm: float = 30.0  # come straight down until the fingertips should be this high...
    measured_start_gap_mm: float = 10.0  # ...or this high once a touch in this session measured the reach
    max_overtravel_mm: float = 40.0  # never jog more than this below the expected contact (10 once measured)
    max_step_mm: float = 5.0
    paper_thickness_mm: float = 0.1
    close_gripper: bool = True  # close the gripper before touching: the closed fingertips touch
    expected_reach_mm: float | None = None  # fingertip reach to assume when the gripper has no geometry
    contact_obstacles: list[str] = field(default_factory=list)  # frames the gripper may touch on the way down

    # Manual-mode touch-off.
    manual_mode_timeout_s: float = 300.0  # the arm leaves manual mode by itself after this long
    max_tilt_deg: float = 3.0  # a touch counts only with the gripper this close to pointing straight down

    # Moves.
    move_timeout_s: float = 60.0

    # Results.
    auto_apply: bool = True  # write the gripper's new frame to the machine config after `finish`
    min_touches: int = 1  # `finish` needs at least this many touches

    @classmethod
    def from_attributes(cls, attributes: Mapping[str, Any]) -> "Settings":
        known = {f.name: f for f in fields(cls)}
        unknown = sorted(set(attributes) - set(known))
        if unknown:
            raise ValueError(f"Unknown attribute(s) {', '.join(unknown)}; known: {', '.join(sorted(known))}")
        settings = cls()
        for name, value in attributes.items():
            default = getattr(settings, name)
            if isinstance(default, bool):
                if not isinstance(value, bool):
                    raise ValueError(f"{name} must be true or false")
            elif isinstance(default, int):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value):
                    raise ValueError(f"{name} must be a whole number")
                value = int(value)
            elif isinstance(default, float) or name in ("table_z_mm", "expected_reach_mm"):
                if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))):
                    raise ValueError(f"{name} must be a number")
                value = None if value is None else float(value)
            elif isinstance(default, str):
                if not isinstance(value, str):
                    raise ValueError(f"{name} must be a string")
            elif isinstance(default, list):
                if not isinstance(value, list) or not all(isinstance(v, str) for v in value):
                    raise ValueError(f"{name} must be a list of names")
            setattr(settings, name, value)
        settings.check()
        return settings

    def check(self) -> None:
        if not self.arm:
            raise ValueError('"arm" is required: the name of the arm the gripper is mounted on')
        if not self.gripper:
            raise ValueError('"gripper" is required: the name of the gripper component to touch off')
        if not self.motion:
            raise ValueError('"motion" must name a motion service (usually "builtin")')
        positive = ["depth_frames", "travel_height_mm", "start_gap_mm", "measured_start_gap_mm", "max_overtravel_mm",
                    "max_step_mm", "manual_mode_timeout_s", "max_tilt_deg", "move_timeout_s", "min_touches",
                    "table_fit_tolerance_mm", "max_table_tilt_deg", "max_depth_mm"]
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be greater than 0")
        for name in ("paper_thickness_mm", "min_depth_mm", "obstacle_height_mm", "spot_clearance_mm", "min_base_distance_mm"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} can't be negative")
        if self.min_depth_mm >= self.max_depth_mm:
            raise ValueError("min_depth_mm must be less than max_depth_mm")
        if self.expected_reach_mm is not None and self.expected_reach_mm < 0:
            raise ValueError("expected_reach_mm can't be negative")
        if self.travel_height_mm < self.start_gap_mm:
            raise ValueError("travel_height_mm must be at least start_gap_mm")
        if self.max_tilt_deg > 15:
            raise ValueError("max_tilt_deg can be at most 15: a tilted gripper touches with the edge of a fingertip")
