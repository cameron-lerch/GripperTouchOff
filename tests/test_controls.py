"""The CONTROL-tab components (buttons, jog-step switch, status sensor, setup discovery) driving
the touch-off service in the simulated cell, the way an operator clicking in the Viam app would."""

import asyncio

import pytest
from grpclib import Status
from viam.components.button import Button
from viam.components.sensor import Sensor
from viam.components.switch import Switch
from viam.proto.app.robot import ComponentConfig
from viam.services.generic import Generic
from viam.utils import dict_to_struct, struct_to_dict

from sim import MODEL_REACH, TABLE_Z, TRUE_REACH, Cell, FakeArm
from touchoff.controls import (ACTIONS, JogStepSwitch, TouchOffButton, TouchOffSetup, TouchOffStatus,
                               component_configs, readings)
from touchoff.procedure import Procedure, TouchOffError
from touchoff.service import TouchOff
from touchoff.settings import Settings


def service(tmp_path, cell=None, **attributes):
    cell = cell or Cell()
    attributes = {"arm": "arm", "gripper": "gripper", "camera": "cam", **attributes}
    settings = Settings.from_attributes({k: v for k, v in attributes.items() if v is not None})
    touch_off = TouchOff("touch-off")
    touch_off.procedure = Procedure(settings, cell.arm, cell.gripper, cell.motion,
                                    cell.camera if settings.camera else None,
                                    part_config=cell.part_config, data_dir=tmp_path)
    return cell, touch_off


def build(cls, touch_off, **attributes):
    """Validate and construct a control the way viam-server does, from its config."""
    config = ComponentConfig(name=f"c-{cls.__name__}", attributes=dict_to_struct({"touch_off": "touch-off", **attributes}))
    required, optional = cls.validate_config(config)
    assert required == ["rdk:service:generic/touch-off"] and optional == []
    return cls.new(config, {Generic.get_resource_name("touch-off"): touch_off})


def buttons(touch_off):
    return {key: build(TouchOffButton, touch_off, action=key) for key in ACTIONS}


async def settle(touch_off):
    """Wait for a background move, like an operator watching the status card."""
    for _ in range(500):
        if (await touch_off.do_command({"command": "status"}))["state"] != "moving":
            return
        await asyncio.sleep(0.01)
    raise AssertionError("still moving")


def test_a_whole_jog_touch_off_from_the_control_tab(tmp_path):
    async def scenario():
        cell, touch_off = service(tmp_path)
        push = buttons(touch_off)
        step = build(JogStepSwitch, touch_off)
        status = build(TouchOffStatus, touch_off)

        assert await step.get_number_of_positions() == (4, ["0.1 mm", "0.5 mm", "1 mm", "5 mm"])
        assert await step.get_position() == 2  # 1mm to start
        first = await status.get_readings()
        assert first["state"] == "idle" and "1-measure-table" in first["next_step"]

        await push["measure_table"].push()
        assert (await status.get_readings())["table_z_mm"] == pytest.approx(TABLE_Z, abs=0.3)
        await push["go_to_spot"].push()  # returns at once; the move runs in the background
        await settle(touch_off)
        jogging = await status.get_readings()
        assert jogging["state"] == "jogging" and "3-jog-down" in jogging["next_step"]
        assert jogging["fingertip_gap_mm"] == pytest.approx(30.0, abs=0.5)  # by the model; really ~6mm

        # The operator jogs down with the largest step that doesn't press past the paper.
        sizes = [0.1, 0.5, 1.0, 5.0]
        for _ in range(100):
            gap = cell.tip_z() - TABLE_Z - 0.1
            if gap < 0.1:  # the paper drags (to within the smallest step)
                break
            position = max(i for i, s in enumerate(sizes) if s <= gap + 1e-6)
            await step.set_position(position)
            assert await step.get_position() == position
            await push["jog_down"].push()
        else:
            raise AssertionError("never touched")
        await push["record_touch"].push()
        await settle(touch_off)
        recorded = await status.get_readings()
        assert recorded["touches"] == 1 and recorded["last_touch_reach_mm"] == pytest.approx(TRUE_REACH, abs=0.4)

        await push["preview"].push()
        preview = await status.get_readings()
        assert preview["applied"] is False
        assert preview["frame_shift_mm"] == pytest.approx(TRUE_REACH - MODEL_REACH, abs=0.4)
        assert preview["old_frame_translation"] == "0.00, 0.00, 105.00"
        assert cell.config_translation() == {"x": 0, "y": 0, "z": 105}  # preview changes nothing

        await push["finish"].push()
        done = await status.get_readings()
        assert done["applied"] is True and "7-undo-update" in done["next_step"]
        assert cell.config_translation()["z"] == pytest.approx(105 + TRUE_REACH - MODEL_REACH, abs=0.4)
        assert "-> " in done["last_update"]

        await push["undo"].push()
        assert cell.config_translation() == {"x": 0, "y": 0, "z": 105}
    asyncio.run(scenario())


def test_buttons_show_why_they_cant_run_now(tmp_path):
    async def scenario():
        _, touch_off = service(tmp_path)
        push = buttons(touch_off)
        with pytest.raises(TouchOffError, match="Not jogging") as refused:
            await push["jog_down"].push()
        # Through gRPC this is the plain sentence, which is what the Viam app shows.
        status, message, _ = refused.value.grpc_error.args
        assert status == Status.FAILED_PRECONDITION
        assert message.startswith("Not jogging") and "TouchOffError" not in message
        with pytest.raises(TouchOffError, match="at least 1 needed"):
            await push["finish"].push()
        await push["stop"].push()  # always allowed
    asyncio.run(scenario())


def test_jog_step_switch_limits(tmp_path):
    async def scenario():
        _, touch_off = service(tmp_path, max_step_mm=2)
        step = build(JogStepSwitch, touch_off, steps_mm=[0.2, 2, 4])
        assert await step.get_number_of_positions() == (3, ["0.2 mm", "2 mm", "4 mm"])
        await step.set_position(1)
        assert touch_off.procedure.jog_step_mm == 2
        with pytest.raises(TouchOffError, match="limited to 2mm"):
            await step.set_position(2)
        with pytest.raises(TouchOffError, match="doesn't exist"):
            await step.set_position(3)
        assert touch_off.procedure.jog_step_mm == 2
    asyncio.run(scenario())


def test_jog_without_mm_uses_the_step(tmp_path):
    async def scenario():
        cell, touch_off = service(tmp_path)
        await touch_off.do_command({"command": "measure_table"})
        await touch_off.do_command({"command": "start_jog", "wait": True})
        await touch_off.do_command({"command": "set_jog_step", "mm": 0.5})
        z = cell.T_gripper[2, 3]
        await touch_off.do_command({"command": "jog"})
        assert cell.T_gripper[2, 3] == pytest.approx(z - 0.5, abs=1e-6)
        await touch_off.do_command({"command": "jog", "up": True})
        assert cell.T_gripper[2, 3] == pytest.approx(z, abs=1e-6)
    asyncio.run(scenario())


def test_manual_buttons(tmp_path):
    async def scenario():
        cell, touch_off = service(tmp_path, camera=None, table_z_mm=TABLE_Z)
        push = buttons(touch_off)
        status = build(TouchOffStatus, touch_off)
        await push["start_manual"].push()
        assert cell.manual and "4-record-touch" in (await status.get_readings())["next_step"]
        cell.T_gripper[2, 3] = TABLE_Z + TRUE_REACH  # the operator rests the fingertips on the table
        await push["record_touch"].push()
        await push["stop_manual"].push()
        assert not cell.manual
        assert (await status.get_readings())["last_touch_reach_mm"] == pytest.approx(TRUE_REACH, abs=1e-3)
    asyncio.run(scenario())


def test_setup_offers_every_control_wired_to_the_service(tmp_path):
    async def scenario():
        _, touch_off = service(tmp_path)
        setup = build(TouchOffSetup, touch_off)
        configs = await setup.discover_resources()
        names = [c.name for c in configs]
        assert names == ["touch-off-0-status", "touch-off-stop", "touch-off-1-measure-table", "touch-off-2-go-to-spot",
                         "touch-off-2-start-manual-mode", "touch-off-3-jog-step", "touch-off-3-jog-down",
                         "touch-off-3-jog-up", "touch-off-4-record-touch", "touch-off-5-leave-manual-mode",
                         "touch-off-6-preview-result", "touch-off-6-finish-and-update", "touch-off-7-undo-update",
                         "touch-off-7-clear-touches"]
        classes = {"button": TouchOffButton, "jog-step": JogStepSwitch, "status": TouchOffStatus}
        apis = {"button": Button.API, "jog-step": Switch.API, "status": Sensor.API}
        for config in configs:
            family, model = config.model.rsplit(":", 1)
            assert family == "cameronlerch:gripper-touch-off"
            assert config.api == str(apis[model])
            assert classes[model].validate_config(config) == (["rdk:service:generic/touch-off"], [])  # addable as is
        step = next(c for c in configs if c.name.endswith("jog-step"))
        assert struct_to_dict(step.attributes)["steps_mm"] == [0.1, 0.5, 1.0, 5.0]
    asyncio.run(scenario())


def test_setup_leaves_out_what_the_cell_cant_do():
    configs = component_configs("t", "cell", camera=False, manual_mode=False, max_step_mm=2)
    names = [c.name for c in configs]
    assert not any("measure-table" in n or "manual" in n for n in names)
    step = next(c for c in configs if c.name == "cell-3-jog-step")
    assert struct_to_dict(step.attributes)["steps_mm"] == [0.1, 0.5, 1.0]


def test_capabilities(tmp_path):
    async def scenario():
        cell = Cell()
        cell.arm = FakeArm(cell, manual_mode=False)
        _, touch_off = service(tmp_path, cell=cell, camera=None, table_z_mm=0)
        assert await touch_off.do_command({"command": "capabilities"}) == {
            "camera": False, "manual_mode": False, "max_step_mm": 5}
    asyncio.run(scenario())


def test_controls_reject_bad_config():
    def config(**attributes):
        return ComponentConfig(name="x", attributes=dict_to_struct(attributes))
    with pytest.raises(ValueError, match="touch_off"):
        TouchOffButton.validate_config(config(action="stop"))
    with pytest.raises(ValueError, match="action must be one of"):
        TouchOffButton.validate_config(config(touch_off="t", action="dance"))
    with pytest.raises(ValueError, match="unknown attribute"):
        TouchOffStatus.validate_config(config(touch_off="t", colour="red"))
    with pytest.raises(ValueError, match="steps_mm"):
        JogStepSwitch.validate_config(config(touch_off="t", steps_mm=[1, -2]))


def test_readings_are_flat_values():
    values = readings({"state": "idle", "touches": [{"reach_mm": 74.0}], "table": {"z_mm": -10.0, "source": "camera"},
                       "gripper": {"fingertip_gap_mm": 3.2, "tilt_deg": 0.1, "position_mm": [1, 2, 3]},
                       "result": {"reach_mm": 74.0, "frame_shift_mm": 24.0, "applied": False,
                                  "frame": {"parent": "arm", "old_translation": {"x": 0, "y": 0, "z": 105},
                                            "translation": {"x": 0, "y": 0, "z": 129}}}})
    assert all(isinstance(v, (str, int, float, bool)) for v in values.values())
    assert values["new_frame_translation"] == "0.00, 0.00, 129.00" and values["gripper_position_mm"] == "1.00, 2.00, 3.00"


def test_next_step_follows_what_the_cell_can_do():
    from touchoff.controls import next_step
    assert "1-measure-table" in next_step({"state": "idle", "has_camera": True})
    assert "table_z_mm" in next_step({"state": "idle", "has_camera": False})
    idle = {"state": "idle", "table": {"z_mm": 0}}
    assert "start-manual-mode" in next_step(idle, manual_mode=True)
    assert "manual" not in next_step(idle, manual_mode=False)
