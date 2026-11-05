"""The touch-off as a Viam generic service: configuration, dependencies and DoCommand."""

from __future__ import annotations

from typing import Any, ClassVar, Mapping, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.generic import Generic
from viam.services.motion import Motion
from viam.utils import ValueTypes, struct_to_dict

from .procedure import Procedure, TouchOffError
from .settings import Settings

MODULE_NAMESPACE = "cameronlerch"
MODULE_NAME = "gripper-touch-off"


def parse_command(command: Mapping[str, Any]) -> tuple[str, dict]:
    """{"command": "jog", "mm": 1} or {"jog": {"mm": 1}} -> ("jog", {"mm": 1})."""
    if "command" in command:
        name = command["command"]
        params = {k: v for k, v in command.items() if k != "command"}
    elif len(command) == 1:
        (name, value), = command.items()
        params = dict(value) if isinstance(value, Mapping) else {}
    else:
        raise TouchOffError('Send {"command": "<name>", ...arguments}; {"command": "status"} shows where things stand')
    if not isinstance(name, str):
        raise TouchOffError("command must be a string")
    return name, params


def without_nulls(value):
    """Drop None values from dicts: they reach clients as 0 through a protobuf Struct, which would mislead."""
    if isinstance(value, Mapping):
        return {k: without_nulls(v) for k, v in value.items() if v is not None}
    if isinstance(value, (list, tuple)):
        return [without_nulls(v) for v in value]
    return value


class TouchOff(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(MODULE_NAMESPACE, MODULE_NAME), "touch-off")

    procedure: Procedure

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        settings = Settings.from_attributes(struct_to_dict(config.attributes))
        required = [settings.arm, settings.gripper, f"rdk:service:motion/{settings.motion}"]
        if settings.camera:
            required.append(settings.camera)
        return required, []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        service = cls(config.name)
        settings = Settings.from_attributes(struct_to_dict(config.attributes))
        arm = dependencies[Arm.get_resource_name(settings.arm)]
        gripper = dependencies[Gripper.get_resource_name(settings.gripper)]
        motion = dependencies[Motion.get_resource_name(settings.motion)]
        camera = dependencies[Camera.get_resource_name(settings.camera)] if settings.camera else None
        service.procedure = Procedure(settings, arm, gripper, motion, camera, logger=getLogger(f"touch-off.{config.name}"))
        return service

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout: float | None = None,
                         **kwargs) -> Mapping[str, ValueTypes]:
        name, params = parse_command(command)
        try:
            return without_nulls(await self.procedure.handle(name, params))
        except TouchOffError:
            raise
        except Exception as error:
            self.procedure.log.exception("%s failed", name)
            raise TouchOffError(f"{name} failed: {error}") from error

    async def close(self) -> None:
        if hasattr(self, "procedure"):
            await self.procedure.close()
