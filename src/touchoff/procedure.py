"""The touch-off procedure: find where the gripper's fingertips really are by touching the table.

    reach = how far the fingertips reach past the gripper frame's origin, along its pointing axis
          = (height of the gripper frame at contact - height of the table) for a gripper pointing down

The table's height comes from an RGB-D camera (`measure_table`) or the `table_z_mm` attribute.
Two ways to touch:

  * Jog (machinist style): the closed gripper, pointing straight down, comes down to a few
    millimeters above bare table; the operator slides a sheet of paper under the fingertips
    and jogs down in small steps until the paper drags, then calls `touch`.
  * Manual: the arm goes into manual mode (moved by hand); the operator brings the fingertips
    down onto the table with the gripper pointing straight down and calls `touch`.

`finish` combines the touches and moves the gripper's frame (in the machine config) along its
pointing axis so that its collision model reaches exactly as far as the real fingertips.
With no collision model, that puts the frame's origin at the fingertips.

Moves that take a while (to a spot, back up after a touch) run in the background so that
DoCommand returns at once; `status` reports progress. Jog steps finish before returning.
"""

from __future__ import annotations

import asyncio
import inspect
import json
import logging
import math
import os
import time
from pathlib import Path
from typing import Any, Awaitable, Callable

import numpy as np
from grpclib import Status
from viam.errors import ViamGRPCError

from . import geometry as geo
from .machine_config import ConfigUnavailable, PartConfig, locate_gripper_frame, with_translation
from .settings import Settings

DEPTH_MIME = "image/vnd.viam.dep"
SPOT_RADIUS_MM = 40.0  # bare table all around a spot, wider than closed fingertips
SLAB_MARGIN_MM = 10.0  # the camera's table, as an obstacle for moves between spots, this far up
SLAB_BASE_CUTOUT_MM = 150.0  # ...with a square this half-size cut out where the arm's base stands on it
OBSTACLE_MARGIN_MM = 20.0
MAX_FRAME_SHIFT_MM = 150.0  # `finish` refuses larger frame changes unless forced: something is wrong
STILL_TOLERANCE_MM = 0.5  # manual touches: the arm must hold this still...
STILL_WINDOW_S = 0.3  # ...over this long
CALL_TIMEOUT_S = 15.0


class TouchOffError(ViamGRPCError):
    """A request the procedure can't carry out now; the message says why and what to do.
    Raised through gRPC as FAILED_PRECONDITION with just the message, so the Viam app and SDK
    clients show the sentence itself (other exceptions get the class name and a file location)."""

    def __init__(self, message: str):
        super().__init__(message, Status.FAILED_PRECONDITION)
        self.args = (message,)


class Procedure:
    def __init__(self, settings: Settings, arm, gripper, motion, camera=None, *,
                 part_config: PartConfig | None = None, data_dir: str | Path | None = None,
                 logger: logging.Logger | None = None):
        self.settings = settings
        self.arm, self.gripper, self.motion, self.camera = arm, gripper, motion, camera
        self.part_config = part_config or PartConfig()
        data_dir = data_dir if data_dir is not None else os.environ.get("VIAM_MODULE_DATA", "")
        self.history_path = Path(data_dir) / "touch_off_history.json" if data_dir else None
        self.log = logger or logging.getLogger(__name__)

        self.state = "idle"  # idle | moving | jogging | manual
        self.message = "Ready. Measure the table (or set table_z_mm), then start a jog or manual touch-off."
        self.last_error = ""
        self.table: geo.TablePlane | None = None
        self.table_info: dict = {}
        if settings.table_z_mm is not None:
            self._set_table(geo.TablePlane.horizontal(settings.table_z_mm), {"source": "config", "z_mm": settings.table_z_mm})
        self.obstacles: list[geo.Obstacle] = []
        self.spots: list[np.ndarray] = []
        self.spot_index = 0
        self.table_region: tuple[float, float, float, float] | None = None
        self.jog: dict | None = None
        self.touches: list[dict] = []
        self.result: dict | None = None
        self.jog_step_mm = min(1.0, settings.max_step_mm)  # the step `jog` uses when it isn't given one
        self._modeled: tuple[float | None] | None = None  # cached (reach,) of the collision model
        self._task: asyncio.Task | None = None
        self._lock = asyncio.Lock()

    # --- DoCommand entry point -----------------------------------------------------------

    async def handle(self, command: str, params: dict[str, Any]) -> dict:
        handlers: dict[str, Callable[..., Awaitable[dict]]] = {
            "status": self.status,
            "capabilities": self.capabilities,
            "measure_table": self.measure_table,
            "set_table": self.set_table,
            "start_jog": self.start_jog,
            "jog": self.jog_by,
            "set_jog_step": self.set_jog_step,
            "touch": self.touch,
            "start_manual": self.start_manual,
            "stop_manual": self.stop_manual,
            "finish": self.finish,
            "undo": self.undo,
            "remove_touch": self.remove_touch,
            "reset": self.reset,
            "stop": self.stop,
        }
        if command not in handlers:
            raise TouchOffError(f"Unknown command {command!r}; commands: {', '.join(handlers)}")
        accepted = inspect.signature(handlers[command]).parameters
        unknown = sorted(set(params) - set(accepted))
        if unknown:
            raise TouchOffError(f"{command} doesn't take {', '.join(unknown)}; it takes: {', '.join(accepted) or 'nothing'}")
        if command in ("status", "stop", "capabilities"):  # never wait behind another command
            return await handlers[command](**params)
        async with self._lock:
            return await handlers[command](**params)

    # --- Status ------------------------------------------------------------------------

    async def status(self) -> dict:
        status: dict[str, Any] = {
            "state": self.state,
            "message": self.message,
            "error": self.last_error,
            "table": self.table_info or None,
            "spots_available": max(len(self.spots) - self.spot_index, 0),
            "touches": self.touches,
            "result": self.result,
            "max_tilt_deg": self.settings.max_tilt_deg,
            "jog_step_mm": self.jog_step_mm,
            "max_step_mm": self.settings.max_step_mm,
            "has_camera": self.camera is not None,
        }
        history = self._history_or_empty()
        if history:  # survives the service being rebuilt when the machine reconfigures after an update
            last = history[-1]
            status["last_applied"] = {k: last.get(k) for k in ("time", "applied_to", "old_translation", "new_translation")}
        if self.jog:
            status["jog"] = {k: round(float(v), 2) for k, v in self.jog.items() if k in ("x", "y", "z", "floor", "top")}
        try:
            T = await self._gripper_pose()
            gripper: dict[str, Any] = {"position_mm": [round(float(v), 2) for v in T[:3, 3]],
                                       "tilt_deg": round(geo.tilt_from_down_deg(T), 2)}
            reach, source = await self._known_reach()
            if self.table is not None and reach is not None and T[2, 2] < -0.5:
                gripper["fingertip_gap_mm"] = round(geo.fingertip_gap(T, self.table, reach), 2)
                gripper["gap_from"] = source
            status["gripper"] = gripper
        except Exception as error:  # status must answer even when the robot doesn't
            status["gripper_error"] = str(error)
        return status

    async def capabilities(self) -> dict:
        """What this setup can do: measure the table (a camera is configured), manual mode (the arm
        supports it), and the largest jog step."""
        try:
            properties = await self._call(self.arm.get_properties())
            manual = bool(getattr(properties, "support_manual_mode", False))
        except Exception as error:
            self.log.warning("Couldn't read the arm's properties: %s", error)
            manual = False
        return {"camera": self.camera is not None, "manual_mode": manual, "max_step_mm": self.settings.max_step_mm}

    # --- The table ---------------------------------------------------------------------

    def _set_table(self, plane: geo.TablePlane, info: dict) -> None:
        self.table = plane
        self.table_info = {**info, "tilt_deg": round(plane.tilt_deg, 3)}

    async def set_table(self, z_mm: float) -> dict:
        """Use a flat, level table at this height in the world frame (mm)."""
        self._check_idle()
        self._set_table(geo.TablePlane.horizontal(float(z_mm)), {"source": "command", "z_mm": float(z_mm)})
        self.obstacles, self.spots, self.spot_index, self.table_region = [], [], 0, None
        self.message = f"Table set to z = {float(z_mm):.2f}mm."
        return {"table": self.table_info}

    async def measure_table(self) -> dict:
        """Fit the table plane in the camera's depth image, find what stands on it and bare spots to touch."""
        if self.camera is None:
            raise TouchOffError('No camera configured: add "camera" to the attributes, or set the table height with '
                                'set_table / table_z_mm')
        self._check_idle()
        await self._wait_until_still()
        try:
            pose = await self._call(self.motion.get_pose(component_name=self.camera.name, destination_frame="world"))
        except Exception as error:
            raise TouchOffError(f"Can't place camera {self.camera.name!r} in the world: {error}. Give the camera a frame "
                                "(parented to the arm for an arm-mounted camera; the camera-calibration module finds it)") from error
        T_world_cam = geo.pose_to_matrix(pose.pose)
        points_cam = await self._capture_points()
        points = geo.transform_points(T_world_cam, points_cam)
        s = self.settings
        try:
            plane, inliers = await asyncio.to_thread(geo.fit_table_plane, points, s.table_fit_tolerance_mm, s.max_table_tilt_deg)
        except ValueError as error:
            raise TouchOffError(f"{error}. Point the camera at bare table ({s.min_depth_mm:.0f}-{s.max_depth_mm:.0f}mm away)") from error
        obstacles = await asyncio.to_thread(geo.find_obstacles, points, plane, s.obstacle_height_mm)
        spots = await asyncio.to_thread(geo.candidate_spots, points, plane, obstacles, s.spot_clearance_mm, SPOT_RADIUS_MM,
                                        20.0, s.min_base_distance_mm)
        table_xy = points[inliers][:, :2]
        self.table_region = (*table_xy.min(axis=0), *table_xy.max(axis=0))
        below = T_world_cam[:3, 3]
        rms = float(np.sqrt(np.mean(plane.height(points[inliers]) ** 2)))
        self._set_table(plane, {"source": "camera", "z_mm": round(plane.z_at(below[0], below[1]), 2),
                                "z_at_mm": [round(float(below[0]), 1), round(float(below[1]), 1)],
                                "points": int(inliers.sum()), "share": round(float(inliers.mean()), 3),
                                "fit_rms_mm": round(rms, 2), "plane": plane.to_dict()})
        self.obstacles, self.spots, self.spot_index = obstacles, spots, 0
        self.message = (f"Table found: z = {self.table_info['z_mm']:.1f}mm under the camera, tilted {plane.tilt_deg:.2f} deg, "
                        f"{len(obstacles)} thing(s) on it, {len(spots)} bare spot(s) to touch.")
        if not spots:
            self.message += (" No bare spot is clear enough: clear some table and measure again, or move the gripper over bare table "
                             "and go to the spot from there (with no bare spot known, it comes down where the gripper is).")
        self.log.info(self.message)
        return {"table": self.table_info, "obstacles": len(obstacles), "spots": [[round(float(v), 1) for v in p] for p in spots[:10]]}

    async def _capture_points(self) -> np.ndarray:
        """Camera-frame points (mm) from the median of several depth frames, or from the camera's point cloud."""
        s = self.settings
        frames = []
        for _ in range(s.depth_frames):
            depth = await self._depth_frame()
            if depth is None:
                break
            frames.append(depth)
        intrinsics = None
        if frames:
            properties = await self._call(self.camera.get_properties())
            intrinsics = geo.Intrinsics.from_viam(getattr(properties, "intrinsic_parameters", None))
        if frames and intrinsics is not None:
            depth = await asyncio.to_thread(geo.median_depth, frames)
            points = geo.backproject(depth, intrinsics, s.min_depth_mm, s.max_depth_mm)
        else:
            data, _ = await self._call(self.camera.get_point_cloud())
            points = geo.parse_pcd(data)
            points = points[(points[:, 2] >= s.min_depth_mm) & (points[:, 2] <= s.max_depth_mm)]
        if len(points) == 0:
            raise TouchOffError(f"The camera saw nothing {s.min_depth_mm:.0f}-{s.max_depth_mm:.0f}mm away")
        return points

    async def _depth_frame(self) -> np.ndarray | None:
        source = self.settings.depth_source
        images, _ = await self._call(self.camera.get_images(filter_source_names=[source] if source else None))
        for image in images:
            if image.mime_type == DEPTH_MIME:
                return geo.decode_viam_depth(image.data)
        for image in images:
            if "png" in image.mime_type and (not source or image.name == source):
                try:
                    return geo.decode_png16(image.data)
                except Exception:
                    continue
        if source:
            raise TouchOffError(f"Camera image {source!r} isn't a depth image")
        return None

    # --- Jog touch-off -----------------------------------------------------------------

    async def start_jog(self, x: float | None = None, y: float | None = None, wait: bool = False) -> dict:
        """Go to a bare spot with the closed gripper pointing down, fingertips `start_gap_mm` above the table."""
        self._check_idle()
        if self.table is None:
            raise TouchOffError("The table's height isn't known: measure it with the camera (measure_table) or set table_z_mm first")
        reach, source = await self._known_reach()
        if reach is None:
            raise TouchOffError("The gripper has no collision geometry, so how far its fingertips reach is unknown. "
                                "Set expected_reach_mm (a generous estimate, mm past the gripper frame) and try again")
        measured = source == "touches"
        gap = self.settings.measured_start_gap_mm if measured else self.settings.start_gap_mm
        overtravel = min(10.0, self.settings.max_overtravel_mm) if measured else self.settings.max_overtravel_mm
        current = await self._gripper_pose()
        if x is not None and y is not None:
            spots = [np.array([float(x), float(y), self.table.z_at(float(x), float(y))])]
        elif x is not None or y is not None:
            raise TouchOffError("Give both x and y, or neither")
        elif self.spot_index < len(self.spots):
            spots = self.spots[self.spot_index:self.spot_index + 5]
        else:
            if self.spots:
                raise TouchOffError("Every bare spot found has been tried: measure the table again for fresh spots (or give start_jog an x and y)")
            spots = [np.array([current[0, 3], current[1, 3], self.table.z_at(current[0, 3], current[1, 3])])]
        yaw = geo.yaw_of(current) if geo.tilt_from_down_deg(current) < 45 else 0.0
        from_list = x is None and bool(self.spots)

        async def go() -> None:
            if self.settings.close_gripper:
                self.message = "Closing the gripper: the closed fingertips are what touches."
                await self._call(self.gripper.grab())
            failures = []
            for k, (sx, sy, table_z) in enumerate(spots):
                top = table_z + reach + self.settings.travel_height_mm
                await self._lift_to(top)
                self.message = f"Moving over ({sx:.0f}, {sy:.0f}), fingertips {self.settings.travel_height_mm:.0f}mm up."
                turn = await self._move_over(sx, sy, top, yaw)
                if from_list:
                    self.spot_index += 1
                if turn is None:
                    failures.append(f"({sx:.0f}, {sy:.0f})")
                    continue
                start = table_z + reach + gap
                self.message = f"Coming down to {gap:.0f}mm above the table."
                await self._move(geo.pointing_down(sx, sy, start, turn), linear=True)
                actual = await self._gripper_pose()
                self.jog = {"x": sx, "y": sy, "yaw": turn, "z": float(actual[2, 3]), "top": top,
                            "floor": table_z + reach - overtravel, "overtravel": overtravel, "reach": reach}
                self.state = "jogging"
                self.message = (f"At ({sx:.0f}, {sy:.0f}): the fingertips should be about {gap:.0f}mm above the table "
                                f"({'measured' if measured else 'by the gripper model'}). Slide a sheet of paper under them and "
                                f"jog down (up to {self.settings.max_step_mm:g}mm a step). Record the touch the moment the paper drags.")
                return
            raise TouchOffError(f"No reachable spot: tried {', '.join(failures)}")

        return await self._run("Moving to the touch-off spot", go, wait)

    async def set_jog_step(self, mm: float) -> dict:
        """Set the step that `jog` uses when it's called without `mm`."""
        mm = float(mm)
        if not math.isfinite(mm) or mm <= 0:
            raise TouchOffError("The jog step must be a positive number of millimeters")
        if mm > self.settings.max_step_mm:
            raise TouchOffError(f"Steps are limited to {self.settings.max_step_mm:g}mm (the max_step_mm attribute)")
        self.jog_step_mm = mm
        return {"jog_step_mm": mm}

    async def jog_by(self, mm: float | None = None, up: bool = False, wait: bool = True) -> dict:
        """Move the fingertips `mm` down (negative: up) in a straight line. Without `mm`, move one
        jog step (set_jog_step) down, or up with `up`."""
        if self.state != "jogging" or self.jog is None:
            raise TouchOffError("Not jogging: go to a spot first (start_jog)")
        if mm is None:
            mm = -self.jog_step_mm if up else self.jog_step_mm
        mm = float(mm)
        if not math.isfinite(mm) or mm == 0:
            raise TouchOffError("mm must be a non-zero number (positive = down)")
        if abs(mm) > self.settings.max_step_mm:
            raise TouchOffError(f"Steps are limited to {self.settings.max_step_mm:g}mm")
        jog = self.jog
        target = jog["z"] - mm
        if mm > 0 and target < jog["floor"]:
            raise TouchOffError(
                f"Refused: that would put the fingertips more than {jog['overtravel']:.0f}mm past where contact was expected "
                f"({jog['z'] - jog['floor']:.1f}mm of travel left). If the paper still slides freely, the table height or the "
                "reach estimate is off: stop, check the table (measure_table) and start again")
        target = min(target, jog["top"])
        await self._move(geo.pointing_down(jog["x"], jog["y"], target, jog["yaw"]), linear=True)
        actual = await self._gripper_pose()
        jog["z"] = float(actual[2, 3])
        gap = geo.fingertip_gap(actual, self.table, jog["reach"])
        self.message = f"Fingertips about {gap:+.1f}mm above the table (estimate). Jog again, or record the touch when the paper drags."
        return {"z_mm": round(jog["z"], 3), "estimated_gap_mm": round(gap, 2)}

    # --- Manual touch-off --------------------------------------------------------------

    async def start_manual(self) -> dict:
        """Put the arm in manual mode so it can be moved by hand."""
        self._check_idle()
        if self.table is None:
            raise TouchOffError("The table's height isn't known: measure it with the camera (measure_table) or set table_z_mm first")
        properties = await self._call(self.arm.get_properties())
        if not getattr(properties, "support_manual_mode", False):
            raise TouchOffError(f"Arm {self.arm.name!r} doesn't support manual mode; use the jog touch-off instead")
        if self.settings.close_gripper:
            await self._call(self.gripper.grab())
        await self._call(self.arm.set_manual_mode(True, int(self.settings.manual_mode_timeout_s)))
        self.state = "manual"
        self.message = (f"Manual mode for up to {self.settings.manual_mode_timeout_s:.0f}s. Hold the gripper pointing straight down "
                        f"(within {self.settings.max_tilt_deg:g} deg) with its fingertips resting on the table, keep still, "
                        "and record the touch. Repeat at other spots, then leave manual mode.")
        return {"state": self.state}

    async def stop_manual(self) -> dict:
        await self._call(self.arm.set_manual_mode(False))
        if self.state == "manual":
            self.state = "idle"
        self.message = f"Manual mode off. {len(self.touches)} touch(es) recorded; finish to apply them."
        return {"state": self.state}

    # --- Touching ----------------------------------------------------------------------

    async def touch(self, paper_mm: float | None = None, wait: bool = False) -> dict:
        """Record that the fingertips are touching now (on the paper, when jogging)."""
        if self.state not in ("jogging", "manual"):
            raise TouchOffError("Nothing to record: go to a spot (start_jog) or start manual mode (start_manual) first")
        manual = self.state == "manual"
        if manual:
            first = await self._gripper_pose()
            await asyncio.sleep(STILL_WINDOW_S)
            T = await self._gripper_pose()
            if np.linalg.norm(T[:3, 3] - first[:3, 3]) > STILL_TOLERANCE_MM:
                raise TouchOffError("The arm moved while recording: hold it still on the table and touch again")
        else:
            await self._wait_until_still()
            T = await self._gripper_pose()
        tilt = geo.tilt_from_down_deg(T)
        if tilt > self.settings.max_tilt_deg:
            raise TouchOffError(f"The gripper is tilted {tilt:.1f} deg from pointing straight down "
                                f"(at most {self.settings.max_tilt_deg:g}): straighten it and touch again")
        paper = float(paper_mm) if paper_mm is not None else (0.0 if manual else self.settings.paper_thickness_mm)
        reach = geo.reach_from_contact(T, self.table, paper)
        tip = T[:3, 3] + reach * T[:3, 2]
        touch = {"reach_mm": round(reach, 3), "tip_mm": [round(float(v), 2) for v in tip],
                 "gripper_mm": [round(float(v), 2) for v in T[:3, 3]], "tilt_deg": round(tilt, 3),
                 "table_z_mm": round(self.table.z_at(tip[0], tip[1]), 3), "paper_mm": paper,
                 "method": "manual" if manual else "jog", "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        self.touches.append(touch)
        modeled = await self._modeled_reach()
        note = "" if modeled is None else f" (the collision model reaches {modeled:.1f}mm)"
        self.log.info("Touch %d: fingertips reach %.2fmm past the gripper frame%s", len(self.touches), reach, note)
        response = {"touch": touch, "touches": len(self.touches), "summary": geo.summarize(self.touches, modeled)}
        if manual:
            self.message = f"Touch {len(self.touches)}: reach {reach:.1f}mm{note}. Touch again elsewhere, or leave manual mode and finish."
            return response
        jog = self.jog

        async def back_up() -> None:
            self.message = "Recorded; going back up."
            await self._move(geo.pointing_down(jog["x"], jog["y"], jog["top"], jog["yaw"]), linear=True)
            self.jog = None
            self.message = (f"Touch {len(self.touches)}: reach {reach:.1f}mm{note}. Go to another spot "
                            "(3 spread-out touches also show the table's tilt), or finish.")

        self.state = "idle"
        await self._run("Recorded; going back up", back_up, wait)
        return response

    async def remove_touch(self, index: int) -> dict:
        index = int(index)
        if not 0 <= index < len(self.touches):
            raise TouchOffError(f"No touch {index}; there are {len(self.touches)} (numbered from 0)")
        removed = self.touches.pop(index)
        return {"removed": removed, "touches": len(self.touches)}

    # --- Results -----------------------------------------------------------------------

    async def finish(self, apply: bool | None = None, force: bool = False) -> dict:
        """Combine the touches; with `apply` (default: the auto_apply attribute), move the gripper's
        frame so that its collision model reaches as far as the real fingertips."""
        self._check_idle()
        if len(self.touches) < self.settings.min_touches:
            raise TouchOffError(f"{len(self.touches)} touch(es) recorded; at least {self.settings.min_touches} needed")
        modeled = await self._modeled_reach()
        summary = geo.summarize(self.touches, modeled)
        shift = summary["reach_mm"] - (modeled or 0.0)
        summary["frame_shift_mm"] = round(shift, 2)
        summary["points"] = list(self.touches)
        if abs(shift) > MAX_FRAME_SHIFT_MM and not force:
            raise TouchOffError(f"The frame would move {shift:.1f}mm, more than {MAX_FRAME_SHIFT_MM:.0f}mm: check the table height "
                                "and the touches (finish with force: true to apply anyway)")
        apply = self.settings.auto_apply if apply is None else bool(apply)
        frame = await self._new_frame(shift)
        summary["frame"] = frame
        if apply and abs(shift) < 0.05:
            summary["applied"] = False
            summary["apply_note"] = "The gripper's frame already matches the fingertips (within 0.05mm); nothing to change."
        elif apply:
            await self._apply(frame, summary)
        else:
            summary["applied"] = False
        self.result = summary
        if summary.get("applied"):
            self.touches = []  # they were measured against the old frame
            self._modeled = None
            self.message = (f"Fingertips reach {summary['reach_mm']:.1f}mm; the gripper's frame moved {shift:+.1f}mm along its "
                            "pointing axis in the machine config. Touch off again to check: the shift should be about 0.")
        else:
            self.message = f"Fingertips reach {summary['reach_mm']:.1f}mm; the gripper's frame would move {shift:+.1f}mm."
        self.log.info(self.message)
        return summary

    async def _locate(self, gripper: str | None = None):
        """(config, last-updated time, where the gripper's frame is defined)."""
        config, updated = await self.part_config.read()
        site = await locate_gripper_frame(config, gripper or self.settings.gripper, self.part_config.fragment)
        return config, updated, site

    async def _new_frame(self, shift: float) -> dict:
        """The gripper's frame translation after the shift, from the machine config when it can be
        read, else from the frame system (parent assumed to be the arm)."""
        translation, warning = None, ""
        try:
            _, _, site = await self._locate()
            parent, translation, source = site.parent, site.translation(), site.describe()
            if translation is None:
                warning = "the configured translation isn't plain numbers; using the frame system's"
        except Exception as error:  # no cloud credentials, an unsupported fragment setup, or the App unreachable
            parent, source = self.settings.arm, f"frame system ({error})"
        pose = await self._call(self.motion.get_pose(component_name=self.settings.gripper, destination_frame=parent))
        T_parent_gripper = geo.pose_to_matrix(pose.pose)
        in_frame_system = dict(zip("xyz", (round(float(v), 3) + 0.0 for v in T_parent_gripper[:3, 3])))
        if translation is None:
            translation = in_frame_system
        elif max(abs(translation[k] - in_frame_system[k]) for k in "xyz") > 0.5:
            warning = (f"the config's translation {translation} differs from the frame system's {in_frame_system}: "
                       "check that the config is the one the machine runs")
        frame = {"parent": parent, "old_translation": translation,
                 "translation": geo.shifted_translation(translation, T_parent_gripper, shift), "from": source}
        if warning:
            frame["warning"] = warning
        return frame

    async def _apply(self, frame: dict, summary: dict) -> None:
        try:
            config, updated, site = await self._locate()
            if site.parent != frame["parent"] or site.translation() != frame["old_translation"]:
                raise ConfigUnavailable("the gripper's frame changed in the config while finishing; finish again")
            await self.part_config.write(with_translation(config, site, self.settings.gripper, frame["translation"]), updated)
        except ConfigUnavailable as error:
            summary["applied"], summary["apply_error"] = False, str(error)
            return
        except Exception as error:  # the App refused (e.g. the config changed meanwhile, or the key can't write)
            summary["applied"], summary["apply_error"] = False, f"The machine config wasn't changed: {error}"
            return
        summary["applied"], summary["applied_to"] = True, site.describe()
        self._record_history({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "gripper": self.settings.gripper,
                              "parent": frame["parent"], "old_translation": frame["old_translation"],
                              "new_translation": frame["translation"], "applied_to": site.describe(),
                              "result": {k: v for k, v in summary.items() if k != "frame"}})

    async def undo(self) -> dict:
        """Put back the gripper frame translation from before the last applied touch-off."""
        self._check_idle()
        history = self._history()
        if not history:
            raise TouchOffError("Nothing to undo: no touch-off has been applied from this module")
        last = history[-1]
        try:
            config, updated, site = await self._locate(last["gripper"])
        except ConfigUnavailable as error:
            raise TouchOffError(str(error)) from error
        now = site.translation() or {}
        if any(abs(float(now.get(k, math.nan)) - float(last["new_translation"][k])) > 1e-3 for k in "xyz"):
            raise TouchOffError(f"The gripper's frame changed since the last touch-off (now {now}); not undoing")
        await self.part_config.write(with_translation(config, site, last["gripper"], last["old_translation"]), updated)
        self._write_history(history[:-1])
        self._modeled, self.touches = None, []
        self.message = "Restored the gripper's previous frame."
        return {"restored_translation": last["old_translation"]}

    def _history_or_empty(self) -> list:
        try:
            return self._history()
        except (OSError, ValueError):
            return []

    def _history(self) -> list:
        if self.history_path is None or not self.history_path.exists():
            return []
        return json.loads(self.history_path.read_text())

    def _write_history(self, history: list) -> None:
        if self.history_path is None:
            return
        self.history_path.parent.mkdir(parents=True, exist_ok=True)
        self.history_path.write_text(json.dumps(history, indent=2) + "\n")

    def _record_history(self, entry: dict) -> None:
        if self.history_path is None:
            self.log.warning("VIAM_MODULE_DATA isn't set: this touch-off can't be undone from the module")
            return
        self._write_history(self._history() + [entry])

    async def reset(self, keep_table: bool = True) -> dict:
        self._check_idle()
        self.touches, self.result, self.jog, self.last_error = [], None, None, ""
        if not keep_table:
            self.table, self.table_info, self.obstacles, self.spots, self.table_region = None, {}, [], [], None
            if self.settings.table_z_mm is not None:
                self._set_table(geo.TablePlane.horizontal(self.settings.table_z_mm), {"source": "config", "z_mm": self.settings.table_z_mm})
        self.spot_index = 0
        self.message = "Reset."
        return {"state": self.state}

    async def stop(self) -> dict:
        """Stop any motion now, leave manual mode, and drop back to idle."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
        errors = []
        try:
            await self._call(self.arm.stop())
        except Exception as error:
            errors.append(f"arm stop: {error}")
        if self.state == "manual":
            try:
                await self._call(self.arm.set_manual_mode(False))
            except Exception as error:
                errors.append(f"leaving manual mode: {error}")
        self.state, self.jog = "idle", None
        self.message = "Stopped." + (f" ({'; '.join(errors)})" if errors else "")
        return {"state": self.state, "errors": errors}

    async def close(self) -> None:
        busy = self._task is not None and not self._task.done()
        if busy or self.state == "manual":
            await self.stop()

    # --- Helpers -----------------------------------------------------------------------

    def _check_idle(self) -> None:
        if self._task is not None and not self._task.done():
            raise TouchOffError(f"Busy: {self.message} (stop to cancel)")
        if self.state == "manual":
            raise TouchOffError("The arm is in manual mode: leave it first (stop_manual)")
        if self.state == "jogging":
            raise TouchOffError("A jog touch-off is in progress: touch, or stop")

    async def _run(self, label: str, work: Callable[[], Awaitable[None]], wait: bool) -> dict:
        """Run `work` in the background (or now, with `wait`); errors end up in `status`."""
        before = self.state
        self.state, self.message, self.last_error = "moving", label, ""

        async def runner() -> None:
            try:
                await work()
                if self.state == "moving":  # work that ends somewhere else (e.g. jogging) sets the state itself
                    self.state = "idle"
            except asyncio.CancelledError:
                self.state, self.message = "idle", f"{label}: stopped"
                raise
            except Exception as error:
                self.log.error("%s failed: %s", label, error)
                self.state, self.jog = "idle", None
                self.last_error = self.message = f"{label} failed: {error}"
                if wait:
                    raise

        self._task = asyncio.create_task(runner())
        if wait:
            await self._task
        return {"state": self.state, "message": self.message, "from_state": before}

    async def _call(self, awaitable, timeout: float = CALL_TIMEOUT_S):
        return await asyncio.wait_for(awaitable, timeout)

    async def _gripper_pose(self) -> np.ndarray:
        pose = await self._call(self.motion.get_pose(component_name=self.settings.gripper, destination_frame="world"))
        return geo.pose_to_matrix(pose.pose)

    async def _wait_until_still(self, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while await self._call(self.arm.is_moving()):
            if time.monotonic() > deadline:
                raise TouchOffError("The arm still reports motion")
            await asyncio.sleep(0.05)

    async def _modeled_reach(self) -> float | None:
        if self._modeled is None:
            self._modeled = (geo.modeled_reach(await self._call(self.gripper.get_geometries())),)
        return self._modeled[0]

    async def _known_reach(self) -> tuple[float | None, str]:
        """The reach to plan with: this session's touches, else expected_reach_mm when set, else the collision model."""
        if self.touches:
            return float(np.median([t["reach_mm"] for t in self.touches])), "touches"
        if self.settings.expected_reach_mm is not None:
            return self.settings.expected_reach_mm, "expected_reach_mm"
        modeled = await self._modeled_reach()
        return (modeled, "model") if modeled is not None else (None, "")

    def _world_state(self):
        """Things the camera saw on the table, and the table itself, as obstacles for moves between spots."""
        from viam.proto.common import Geometry, GeometriesInFrame, Pose, RectangularPrism, Vector3, WorldState

        geometries = []
        for k, obstacle in enumerate(self.obstacles):
            dims = obstacle.size + 2 * OBSTACLE_MARGIN_MM
            c = obstacle.center
            geometries.append(Geometry(center=Pose(x=c[0], y=c[1], z=c[2], o_z=1),
                                       box=RectangularPrism(dims_mm=Vector3(x=dims[0], y=dims[1], z=dims[2])),
                                       label=f"touch-off-obstacle-{k}"))
        if self.table_region is not None and self.table is not None:
            x0, y0, x1, y1 = self.table_region
            x0, y0, x1, y1 = x0 - 150, y0 - 150, x1 + 150, y1 + 150
            c = SLAB_BASE_CUTOUT_MM
            thickness = 100.0
            for k, (bx0, bx1, by0, by1) in enumerate([(x0, x1, c, y1), (x0, x1, y0, -c), (c, x1, -c, c), (x0, -c, -c, c)]):
                if bx1 <= bx0 or by1 <= by0:
                    continue
                top = max(self.table.z_at(x, y) for x in (bx0, bx1) for y in (by0, by1)) + SLAB_MARGIN_MM
                geometries.append(Geometry(center=Pose(x=(bx0 + bx1) / 2, y=(by0 + by1) / 2, z=top - thickness / 2, o_z=1),
                                           box=RectangularPrism(dims_mm=Vector3(x=bx1 - bx0, y=by1 - by0, z=thickness)),
                                           label=f"touch-off-table-{k}"))
        if not geometries:
            return None
        return WorldState(obstacles=[GeometriesInFrame(reference_frame="world", geometries=geometries)])

    def _constraints(self, linear: bool):
        from viam.proto.service.motion import CollisionSpecification, Constraints, LinearConstraint

        if not linear:
            return None
        # Within 1mm of the straight line and 1 deg of the orientation: tight enough for fine jog steps.
        constraints = Constraints(linear_constraint=[LinearConstraint(line_tolerance_mm=1.0, orientation_tolerance_degs=1.0)])
        if self.settings.contact_obstacles:
            allows = [CollisionSpecification.AllowedFrameCollisions(frame1=self.settings.gripper, frame2=name)
                      for name in self.settings.contact_obstacles]
            constraints.collision_specification.append(CollisionSpecification(allows=allows))
        return constraints

    async def _move(self, T_world: np.ndarray, linear: bool = False, world_state=None) -> None:
        from viam.proto.common import PoseInFrame

        try:
            await self.motion.move(component_name=self.settings.gripper,
                                   destination=PoseInFrame(reference_frame="world", pose=geo.matrix_to_pose(T_world)),
                                   world_state=world_state, constraints=self._constraints(linear),
                                   timeout=self.settings.move_timeout_s)
        except asyncio.CancelledError:
            raise
        except Exception as error:
            raise TouchOffError(f"The motion service couldn't make the {'straight ' if linear else ''}move: "
                                f"{getattr(error, 'message', None) or error}") from error
        await self._wait_until_still()

    async def _lift_to(self, z: float) -> None:
        """Straight up from wherever the gripper is, keeping its orientation (skipped if it's that high already)."""
        T = await self._gripper_pose()
        if T[2, 3] < z - 1.0:
            T[2, 3] = z
            await self._move(T, linear=True)

    async def _move_over(self, x: float, y: float, z: float, yaw: float) -> float | None:
        """Point the gripper down over (x, y) at height z, trying a few turns about its axis;
        the turn that worked, or None if none could be planned."""
        world_state = self._world_state()
        for turn in (yaw, yaw + 90, yaw - 90, yaw + 180):
            try:
                await self._move(geo.pointing_down(x, y, z, turn), world_state=world_state)
                return turn
            except TouchOffError as error:
                self.log.info("No path over (%.0f, %.0f) turned %.0f deg: %s", x, y, turn, error)
        return None
