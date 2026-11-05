"""Controls for the touch-off that appear on the machine's CONTROL tab in the Viam app.

The Viam app draws its own card for each resource type it knows: a button component gets a
Push button, a switch gets its positions, a sensor shows its readings, and a discovery service
lists configs to add with one click. A generic service only gets a raw DoCommand box, so
these small components put the touch-off's steps on the CONTROL tab:

  * `button` - one step of the touch-off per button (measure the table, go to a spot, jog
    down, record touch, finish...). Pushing it sends that command to the touch-off service.
  * `jog-step` - a switch whose positions are the jog step sizes (0.1, 0.5, 1, 5mm).
  * `status` - a sensor whose readings are the live state: fingertip gap, tilt, touches,
    the result, and what to do next.
  * `setup` - a discovery service that offers all of the above, already wired to a touch-off
    service, so each can be added with "Add component" (the way find-webcams offers cameras).

They all talk to the touch-off service through its DoCommand, so scripts and the CONTROL tab
drive the same procedure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, ClassVar, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.button import Button
from viam.components.sensor import Sensor
from viam.components.switch import Switch
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.discovery import Discovery
from viam.services.generic import Generic
from viam.utils import SensorReading, ValueTypes, dict_to_struct, struct_to_dict

from .procedure import TouchOffError
from .service import MODULE_NAME, MODULE_NAMESPACE

FAMILY = ModelFamily(MODULE_NAMESPACE, MODULE_NAME)
DEFAULT_STEPS_MM = (0.1, 0.5, 1.0, 5.0)


@dataclass(frozen=True)
class Action:
    key: str  # the button's `action` attribute
    slug: str  # the button's name ends with this (when added by the setup service)
    command: dict  # what pushing it sends to the touch-off service
    needs: str = ""  # "camera" or "manual_mode": only offered when the setup has it


# In the order they're used. The number in each name groups the buttons on the CONTROL tab:
# 1 table, 2 start (jog or manual), 3 jog, 4 record, 5 leave manual mode, 6 finish, 7 afterwards.
ACTIONS: dict[str, Action] = {a.key: a for a in (
    Action("stop", "stop", {"command": "stop"}),
    Action("measure_table", "1-measure-table", {"command": "measure_table"}, needs="camera"),
    Action("go_to_spot", "2-go-to-spot", {"command": "start_jog"}),
    Action("start_manual", "2-start-manual-mode", {"command": "start_manual"}, needs="manual_mode"),
    Action("jog_down", "3-jog-down", {"command": "jog"}),
    Action("jog_up", "3-jog-up", {"command": "jog", "up": True}),
    Action("record_touch", "4-record-touch", {"command": "touch"}),
    Action("stop_manual", "5-leave-manual-mode", {"command": "stop_manual"}, needs="manual_mode"),
    Action("preview", "6-preview-result", {"command": "finish", "apply": False}),
    Action("finish", "6-finish-and-update", {"command": "finish", "apply": True}),
    Action("undo", "7-undo-update", {"command": "undo"}),
    Action("clear_touches", "7-clear-touches", {"command": "reset", "keep_table": True}),
)}


def _attributes(config: ComponentConfig, allowed: Sequence[str]) -> dict:
    attributes = struct_to_dict(config.attributes)
    unknown = sorted(set(attributes) - set(allowed))
    if unknown:
        raise ValueError(f"{config.name}: unknown attribute(s) {', '.join(unknown)}; allowed: {', '.join(allowed)}")
    touch_off = attributes.get("touch_off")
    if not isinstance(touch_off, str) or not touch_off:
        raise ValueError(f"{config.name}: set touch_off to the name of the touch-off service")
    return attributes


def _dependency(touch_off: str) -> str:
    return f"rdk:service:generic/{touch_off}"


def _service(dependencies: Mapping[ResourceName, ResourceBase], touch_off: str) -> Generic:
    return dependencies[Generic.get_resource_name(touch_off)]  # type: ignore[return-value]


def _steps(value: Any, name: str) -> tuple[float, ...]:
    if value is None:
        return DEFAULT_STEPS_MM
    if (not isinstance(value, (list, tuple)) or not value
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0 for v in value)):
        raise ValueError(f"{name}: steps_mm must be a list of positive numbers of millimeters")
    return tuple(float(v) for v in value)


# --- Button ------------------------------------------------------------------------------

class TouchOffButton(Button, EasyResource):
    """One step of the touch-off as a button: pushing it sends `action` to the touch-off service."""

    MODEL: ClassVar[Model] = Model(FAMILY, "button")
    touch_off: Generic
    action: Action

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attributes = _attributes(config, ("touch_off", "action"))
        if attributes.get("action") not in ACTIONS:
            raise ValueError(f"{config.name}: action must be one of {', '.join(ACTIONS)}")
        return [_dependency(attributes["touch_off"])], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        attributes = struct_to_dict(config.attributes)
        button = cls(config.name)
        button.touch_off = _service(dependencies, attributes["touch_off"])
        button.action = ACTIONS[attributes["action"]]
        return button

    async def push(self, *, extra: Optional[Mapping[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> None:
        # Refusals come back as a gRPC error carrying just the reason, which the app shows as is.
        await self.touch_off.do_command(self.action.command, timeout=timeout)

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout: Optional[float] = None,
                         **kwargs) -> Mapping[str, ValueTypes]:
        """{"push": {}} also pushes it, and returns what the touch-off service answered."""
        if "push" in command:
            return await self.touch_off.do_command(self.action.command, timeout=timeout)
        raise TouchOffError(f'{self.name} only knows "push" (it sends {self.action.command})')


# --- Jog step switch --------------------------------------------------------------------------

class JogStepSwitch(Switch, EasyResource):
    """The jog step as a switch: each position is a step size, used by the jog up/down buttons."""

    MODEL: ClassVar[Model] = Model(FAMILY, "jog-step")
    touch_off: Generic
    steps: tuple[float, ...]

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attributes = _attributes(config, ("touch_off", "steps_mm"))
        _steps(attributes.get("steps_mm"), config.name)
        return [_dependency(attributes["touch_off"])], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        attributes = struct_to_dict(config.attributes)
        switch = cls(config.name)
        switch.touch_off = _service(dependencies, attributes["touch_off"])
        switch.steps = _steps(attributes.get("steps_mm"), config.name)
        return switch

    async def get_position(self, *, extra: Optional[Mapping[str, Any]] = None, timeout: Optional[float] = None,
                           **kwargs) -> int:
        status = await self.touch_off.do_command({"command": "status"}, timeout=timeout)
        step = float(status.get("jog_step_mm", self.steps[0]))
        return min(range(len(self.steps)), key=lambda i: abs(self.steps[i] - step))

    async def set_position(self, position: int, *, extra: Optional[Mapping[str, Any]] = None,
                           timeout: Optional[float] = None, **kwargs) -> None:
        if not 0 <= int(position) < len(self.steps):
            raise TouchOffError(f"Position {position} doesn't exist: positions 0-{len(self.steps) - 1} are "
                                f"{', '.join(_label(s) for s in self.steps)}")
        await self.touch_off.do_command({"command": "set_jog_step", "mm": self.steps[int(position)]}, timeout=timeout)

    async def get_number_of_positions(self, *, extra: Optional[Mapping[str, Any]] = None,
                                      timeout: Optional[float] = None, **kwargs) -> Tuple[int, Sequence[str]]:
        return len(self.steps), [_label(s) for s in self.steps]


def _label(step: float) -> str:
    return f"{step:g} mm"


# --- Status sensor --------------------------------------------------------------------------

class TouchOffStatus(Sensor, EasyResource):
    """The touch-off's live state as sensor readings."""

    MODEL: ClassVar[Model] = Model(FAMILY, "status")
    touch_off: Generic
    capabilities: Mapping[str, Any] | None = None

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attributes = _attributes(config, ("touch_off",))
        return [_dependency(attributes["touch_off"])], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        sensor = cls(config.name)
        sensor.touch_off = _service(dependencies, struct_to_dict(config.attributes)["touch_off"])
        return sensor

    async def get_readings(self, *, extra: Optional[Mapping[str, Any]] = None, timeout: Optional[float] = None,
                           **kwargs) -> Mapping[str, SensorReading]:
        if self.capabilities is None:  # fixed for the life of the service (a config change rebuilds this too)
            self.capabilities = await self.touch_off.do_command({"command": "capabilities"}, timeout=timeout)
        status = await self.touch_off.do_command({"command": "status"}, timeout=timeout)
        return readings(status, manual_mode=bool(self.capabilities.get("manual_mode")))


def _xyz(values) -> str:
    if isinstance(values, Mapping):
        values = [values.get(k) for k in "xyz"]
    return ", ".join(f"{float(v):.2f}" for v in values)


def next_step(status: Mapping[str, Any], manual_mode: bool = True) -> str:
    """What to push next, in the buttons' words."""
    state = status.get("state")
    touches = len(status.get("touches") or [])
    result = status.get("result") or {}
    if state == "moving":
        return "Wait for the arm to finish moving (push stop to cancel)."
    if state == "jogging":
        return ("Slide a sheet of paper under the fingertips. Push 3-jog-down (choose the step with 3-jog-step, "
                "smaller as the gap closes) until the paper drags, then push 4-record-touch.")
    if state == "manual":
        return ("Move the arm by hand until the fingertips rest on the table, gripper pointing straight down. "
                "Hold still and push 4-record-touch. Repeat elsewhere, then push 5-leave-manual-mode.")
    if not status.get("table"):
        if status.get("has_camera"):
            return ("Push 1-measure-table to find the table with the camera "
                    "(or set table_z_mm on the touch-off service).")
        return "Set table_z_mm on the touch-off service (or give it a camera to measure the table)."
    if result.get("applied"):
        return ("The gripper's frame was updated. Touch off again to check (the frame shift should be about 0), "
                "or push 7-undo-update to put it back.")
    if touches:
        return (f"{touches} touch(es) recorded. Push 2-go-to-spot for another, 6-preview-result to see the "
                "change, or 6-finish-and-update to write it to the gripper's frame.")
    if manual_mode:
        return "Push 2-go-to-spot to jog down onto paper, or 2-start-manual-mode to move the arm by hand."
    return "Push 2-go-to-spot to jog down onto a sheet of paper."


def readings(status: Mapping[str, Any], manual_mode: bool = True) -> dict[str, SensorReading]:
    """The status, flattened into simple named values for the CONTROL tab's readings table."""
    out: dict[str, SensorReading] = {
        "state": status.get("state", ""),
        "next_step": next_step(status, manual_mode),
        "message": status.get("message", ""),
    }
    if status.get("error"):
        out["error"] = status["error"]
    gripper = status.get("gripper") or {}
    if "fingertip_gap_mm" in gripper:
        out["fingertip_gap_mm"] = gripper["fingertip_gap_mm"]
    if "tilt_deg" in gripper:
        out["tilt_from_down_deg"] = gripper["tilt_deg"]
        if status.get("state") == "manual" and "max_tilt_deg" in status:
            out["max_tilt_deg"] = status["max_tilt_deg"]  # a touch is only recorded within this
    if "position_mm" in gripper:
        out["gripper_position_mm"] = _xyz(gripper["position_mm"])
    if status.get("gripper_error"):
        out["gripper_error"] = status["gripper_error"]
    if "jog_step_mm" in status:
        out["jog_step_mm"] = status["jog_step_mm"]
    table = status.get("table") or {}
    if "z_mm" in table:
        out["table_z_mm"] = table["z_mm"]
        out["table_from"] = table.get("source", "")
    touches = status.get("touches") or []
    out["touches"] = len(touches)

    if touches:
        out["last_touch_reach_mm"] = touches[-1].get("reach_mm")
    result = status.get("result") or {}
    if result:
        out["reach_mm"] = result.get("reach_mm")
        if result.get("modeled_reach_mm") is not None:
            out["collision_model_reach_mm"] = result["modeled_reach_mm"]
        out["frame_shift_mm"] = result.get("frame_shift_mm")
        frame = result.get("frame") or {}
        if frame.get("translation"):
            out["old_frame_translation"] = _xyz(frame["old_translation"])
            out["new_frame_translation"] = _xyz(frame["translation"])
            out["frame_parent"] = frame.get("parent", "")
        if frame.get("warning"):
            out["warning"] = frame["warning"]
        out["applied"] = bool(result.get("applied"))
        for key in ("applied_to", "apply_error", "apply_note"):
            if result.get(key):
                out[key] = result[key]
    last = status.get("last_applied") or {}
    if last.get("time"):
        out["last_update"] = f"{last['time']}: {_xyz(last['old_translation'])} -> {_xyz(last['new_translation'])}"
    return {k: v for k, v in out.items() if v is not None}


# --- Setup (discovery) -------------------------------------------------------------------------

class TouchOffSetup(Discovery, EasyResource):
    """Offers the buttons, jog-step switch and status sensor for a touch-off service, ready to add."""

    MODEL: ClassVar[Model] = Model(FAMILY, "setup")
    touch_off: Generic
    touch_off_name: str
    prefix: str

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attributes = _attributes(config, ("touch_off", "name_prefix"))
        prefix = attributes.get("name_prefix")
        if prefix is not None and (not isinstance(prefix, str) or not prefix):
            raise ValueError(f"{config.name}: name_prefix must be a non-empty string")
        return [_dependency(attributes["touch_off"])], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        attributes = struct_to_dict(config.attributes)
        setup = cls(config.name)
        setup.touch_off_name = attributes["touch_off"]
        setup.touch_off = _service(dependencies, setup.touch_off_name)
        setup.prefix = attributes.get("name_prefix") or setup.touch_off_name
        return setup

    async def discover_resources(self, *, extra: Optional[Mapping[str, ValueTypes]] = None,
                                 timeout: Optional[float] = None, **kwargs) -> List[ComponentConfig]:
        can = await self.touch_off.do_command({"command": "capabilities"}, timeout=timeout)
        return component_configs(self.touch_off_name, self.prefix, camera=bool(can.get("camera")),
                                 manual_mode=bool(can.get("manual_mode")),
                                 max_step_mm=float(can.get("max_step_mm", max(DEFAULT_STEPS_MM))))


def component_configs(touch_off: str, prefix: str, *, camera: bool, manual_mode: bool,
                      max_step_mm: float) -> List[ComponentConfig]:
    """The components that put a touch-off service's controls on the CONTROL tab."""

    def config(name: str, api: str, model: str, attributes: dict) -> ComponentConfig:
        return ComponentConfig(name=f"{prefix}-{name}", api=api, model=f"{MODULE_NAMESPACE}:{MODULE_NAME}:{model}",
                               attributes=dict_to_struct({"touch_off": touch_off, **attributes}))

    has = {"": True, "camera": camera, "manual_mode": manual_mode}
    steps = [s for s in DEFAULT_STEPS_MM if s <= max_step_mm] or [max_step_mm]
    configs = [config("0-status", "rdk:component:sensor", "status", {})]
    for key, action in ACTIONS.items():
        if not has[action.needs]:
            continue
        if key == "jog_down":
            configs.append(config("3-jog-step", "rdk:component:switch", "jog-step", {"steps_mm": steps}))
        configs.append(config(action.slug, "rdk:component:button", "button", {"action": key}))
    return configs
