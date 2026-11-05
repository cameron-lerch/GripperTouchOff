"""The whole touch-off against the simulated cell (tests/sim.py)."""

import asyncio
import math

import pytest

from sim import MODEL_REACH, TABLE_Z, TRUE_REACH, Cell, Crash, FakeArm, FakeGripper
from touchoff import geometry as geo
from touchoff.procedure import Procedure, TouchOffError
from touchoff.service import parse_command, without_nulls
from touchoff.settings import Settings


def make(tmp_path, cell=None, **attributes):
    cell = cell or Cell()
    attributes = {"arm": "arm", "gripper": "gripper", "camera": "cam", **attributes}
    settings = Settings.from_attributes(attributes)
    procedure = Procedure(settings, cell.arm, cell.gripper, cell.motion, cell.camera if settings.camera else None,
                          part_config=cell.part_config, data_dir=tmp_path)
    return cell, procedure


def run(coroutine):
    return asyncio.run(coroutine)


async def jog_until_paper_drags(cell, procedure, paper=0.1):
    """What the operator does: big steps while far, then small ones, until the fingertips press the paper."""
    for _ in range(200):
        gap = cell.tip_z() - cell.table_z
        if gap <= paper + 1e-9:
            return
        step = min(5.0, gap - paper) if gap - paper > 1 else gap - paper
        await procedure.handle("jog", {"mm": step})
    raise AssertionError("never touched")


def test_camera_jog_touch_off_measures_the_real_reach_and_fixes_the_frame(tmp_path):
    async def scenario():
        cell, p = make(tmp_path)
        table = await p.handle("measure_table", {})
        assert table["table"]["z_mm"] == pytest.approx(TABLE_Z, abs=0.3)
        assert table["obstacles"] == 1 and table["spots"]
        bx, by = cell.block[:2]
        for k in range(3):  # three spots: also gives the tilt between the two tables
            await p.handle("start_jog", {"wait": True})
            assert p.state == "jogging"
            x, y = cell.T_gripper[:2, 3]
            assert math.hypot(x - bx, y - by) > 80
            assert geo.tilt_from_down_deg(cell.T_gripper) < 1e-6
            if k == 0:  # by the model 30mm up, but the real fingertips reach 24mm further
                assert cell.tip_z() - TABLE_Z == pytest.approx(30.0 - (TRUE_REACH - MODEL_REACH), abs=0.5)
            else:  # measured now: 10mm up
                assert cell.tip_z() - TABLE_Z == pytest.approx(10.0, abs=0.5)
            await jog_until_paper_drags(cell, p)
            response = await p.handle("touch", {"wait": True})
            assert response["touch"]["reach_mm"] == pytest.approx(TRUE_REACH, abs=0.4)  # the camera's table is ~0.3mm off
            assert p.state == "idle" and cell.tip_z() > TABLE_Z + 100
        assert cell.gripper.grabs == 3
        result = await p.handle("finish", {})
        assert result["applied"] is True
        assert result["reach_mm"] == pytest.approx(TRUE_REACH, abs=0.4)
        assert result["modeled_reach_mm"] == MODEL_REACH
        assert result["table_tilt_vs_reference"]["deg"] < 0.2
        frame = cell.config["components"][1]["frame"]
        # The gripper's +z is the arm's +z, so the frame moves 24mm further out along it.
        assert frame["translation"]["z"] == pytest.approx(105 + TRUE_REACH - MODEL_REACH, abs=0.4)
        assert frame["translation"]["x"] == pytest.approx(0, abs=1e-6)
        assert frame["orientation"] == {"type": "ov_degrees", "value": {"x": 0, "y": 0, "z": 1, "th": 180}}
        assert cell.config["components"][0]["attributes"] == {"host": "10.0.0.2"}  # nothing else changed
        assert cell.true_reach == pytest.approx(MODEL_REACH, abs=0.4)  # the model now ends at the real fingertips
        assert p.touches == []

        # Touching off again: the frame is right now, so it should hardly move.
        await p.handle("start_jog", {"wait": True})
        assert cell.tip_z() - TABLE_Z == pytest.approx(30.0, abs=0.5)  # by the corrected model: start_gap_mm
        await jog_until_paper_drags(cell, p)
        await p.handle("touch", {"wait": True})
        check = await p.handle("finish", {"apply": False})
        assert abs(check["frame_shift_mm"]) < 0.5 and check["applied"] is False

        # Undo puts the original frame back.
        await p.handle("reset", {})
        await p.handle("undo", {})
        assert cell.config["components"][1]["frame"]["translation"] == {"x": 0, "y": 0, "z": 105}
    run(scenario())


def test_manual_touch_off(tmp_path):
    async def scenario():
        cell, p = make(tmp_path, camera="", table_z_mm=TABLE_Z)
        await p.handle("start_manual", {})
        assert cell.manual and p.state == "manual"
        with pytest.raises(TouchOffError, match="manual mode"):
            await p.handle("start_jog", {})
        # The operator rests the fingertips on the table, 1.5 deg off vertical.
        lean = geo.ov_to_matrix(math.sin(math.radians(1.5)), 0, -math.cos(math.radians(1.5)), 0)
        T = geo.make_transform(lean, [300, -200, 0])
        T[2, 3] = TABLE_Z - TRUE_REACH * T[2, 2]  # tip exactly on the table
        cell.T_gripper = T
        touch = (await p.handle("touch", {}))["touch"]
        assert touch["reach_mm"] == pytest.approx(TRUE_REACH, abs=1e-6)
        assert touch["paper_mm"] == 0.0 and touch["method"] == "manual"
        # Too tilted: refused.
        cell.T_gripper = geo.make_transform(geo.ov_to_matrix(math.sin(math.radians(6)), 0, -math.cos(math.radians(6)), 0), T[:3, 3])
        with pytest.raises(TouchOffError, match="tilted"):
            await p.handle("touch", {})
        await p.handle("stop_manual", {})
        assert not cell.manual and p.state == "idle"
        result = await p.handle("finish", {})
        assert result["applied"] and result["reach_mm"] == pytest.approx(TRUE_REACH, abs=1e-3)
    run(scenario())


def test_manual_touch_needs_the_arm_held_still(tmp_path):
    async def scenario():
        cell, p = make(tmp_path, camera="", table_z_mm=TABLE_Z)
        await p.handle("start_manual", {})
        moving = cell.motion.get_pose

        async def drifting(component_name, destination_frame, **kw):
            cell.T_gripper = cell.T_gripper.copy()
            cell.T_gripper[0, 3] += 2.0
            return await moving(component_name, destination_frame, **kw)
        cell.motion.get_pose = drifting
        with pytest.raises(TouchOffError, match="moved"):
            await p.handle("touch", {})
    run(scenario())


def test_manual_mode_needs_an_arm_that_supports_it(tmp_path):
    cell = Cell()
    cell.arm = FakeArm(cell, manual_mode=False)
    _, p = make(tmp_path, cell, table_z_mm=TABLE_Z)
    with pytest.raises(TouchOffError, match="doesn't support manual mode"):
        run(p.handle("start_manual", {}))


def test_jogging_is_limited(tmp_path):
    async def scenario():
        cell, p = make(tmp_path)
        with pytest.raises(TouchOffError, match="Not jogging"):
            await p.handle("jog", {"mm": 1})
        await p.handle("measure_table", {})
        await p.handle("start_jog", {"wait": True})
        with pytest.raises(TouchOffError, match="limited to 5mm"):
            await p.handle("jog", {"mm": 6})
        with pytest.raises(TouchOffError, match="non-zero"):
            await p.handle("jog", {"mm": 0})
        # Fingertips far shorter than the model (or a table measured too high): the floor stops
        # the jogging max_overtravel_mm (40) below the expected contact, before anything crashes.
        cell.true_reach = 5.0
        with pytest.raises(TouchOffError, match="Refused"):
            for _ in range(100):
                await p.handle("jog", {"mm": 5})
        floor_gap = cell.tip_z() - TABLE_Z
        assert floor_gap > 0
        up = await p.handle("jog", {"mm": -5})
        assert up["z_mm"] == pytest.approx(cell.T_gripper[2, 3], abs=1e-3)
        await p.handle("stop", {})
        assert p.state == "idle" and cell.arm.stops == 1
    run(scenario())


def test_the_floor_stops_before_a_crash_when_the_model_is_short(tmp_path):
    """The real fingertips reach 24mm past the model: jogging can't drive them into the table
    by more than the paper, because the operator touches; and the floor is 40mm below the model's contact."""
    async def scenario():
        cell, p = make(tmp_path)
        await p.handle("measure_table", {})
        await p.handle("start_jog", {"wait": True})
        with pytest.raises((Crash, TouchOffError)):
            for _ in range(100):  # an operator who never calls touch
                await p.handle("jog", {"mm": 1})
    run(scenario())


def test_commands_wait_for_a_move_in_progress(tmp_path):
    async def scenario():
        cell, p = make(tmp_path)
        await p.handle("measure_table", {})
        await p.handle("start_jog", {})  # in the background
        with pytest.raises(TouchOffError, match="Busy|Not jogging"):
            await p.handle("measure_table", {})
        status = await p.handle("status", {})
        assert status["state"] in ("moving", "jogging")
        await p._task
        assert p.state == "jogging"
    run(scenario())


def test_unreachable_spots_are_skipped(tmp_path):
    async def scenario():
        cell, p = make(tmp_path)
        await p.handle("measure_table", {})
        cell.motion.fail_moves = 4  # every turn over the first spot fails
        await p.handle("start_jog", {"wait": True})
        assert p.state == "jogging" and p.spot_index == 2
    run(scenario())


def test_a_failed_move_is_reported_in_status(tmp_path):
    async def scenario():
        cell, p = make(tmp_path, camera="", table_z_mm=TABLE_Z)
        cell.motion.fail_moves = 100
        await p.handle("start_jog", {"x": 300, "y": -200})
        await asyncio.sleep(0.05)
        status = await p.handle("status", {})
        assert status["state"] == "idle" and "No reachable spot" in status["error"]
    run(scenario())


def test_without_geometry_the_reach_must_be_given(tmp_path):
    cell = Cell()
    cell.gripper = FakeGripper(geometries=[])
    _, p = make(tmp_path, cell, camera="", table_z_mm=TABLE_Z)
    with pytest.raises(TouchOffError, match="expected_reach_mm"):
        run(p.handle("start_jog", {"x": 300, "y": -200}))

    async def with_estimate():
        _, p = make(tmp_path, cell, camera="", table_z_mm=TABLE_Z, expected_reach_mm=90)
        await p.handle("start_jog", {"x": 300, "y": -200, "wait": True})
        await jog_until_paper_drags(cell, p)
        await p.handle("touch", {"wait": True})
        result = await p.handle("finish", {})
        # No model: the frame's origin goes to the fingertips.
        assert result["modeled_reach_mm"] is None
        assert result["frame_shift_mm"] == pytest.approx(result["reach_mm"])
        assert cell.true_reach == pytest.approx(0.0, abs=0.01)
    run(with_estimate())


def test_finish_without_cloud_credentials_returns_the_frame(tmp_path, monkeypatch):
    from touchoff.machine_config import PartConfig

    for name in ("VIAM_API_KEY", "VIAM_API_KEY_ID", "VIAM_MACHINE_PART_ID"):
        monkeypatch.delenv(name, raising=False)

    async def scenario():
        cell, p = make(tmp_path, camera="", table_z_mm=TABLE_Z)
        p.part_config = PartConfig()
        await p.handle("start_manual", {})
        cell.T_gripper = geo.pointing_down(300, -200, TABLE_Z + TRUE_REACH, 0)
        await p.handle("touch", {})
        await p.handle("stop_manual", {})
        result = await p.handle("finish", {})
        assert result["applied"] is False and "VIAM_API_KEY" in result["apply_error"]
        assert result["frame"]["parent"] == "arm"
        assert result["frame"]["translation"]["z"] == pytest.approx(105 + TRUE_REACH - MODEL_REACH)
    run(scenario())


def test_finish_refuses_huge_changes_and_too_few_touches(tmp_path):
    async def scenario():
        cell, p = make(tmp_path, camera="", table_z_mm=TABLE_Z, min_touches=2)
        await p.handle("start_manual", {})
        cell.T_gripper = geo.pointing_down(300, -200, TABLE_Z + 300, 0)  # "touching" 300mm up
        await p.handle("touch", {})
        await p.handle("stop_manual", {})
        with pytest.raises(TouchOffError, match="at least 2"):
            await p.handle("finish", {})
        p.settings.min_touches = 1
        with pytest.raises(TouchOffError, match="more than 150mm"):
            await p.handle("finish", {})
        assert cell.part_config.writes == []
    run(scenario())


def test_a_concurrent_config_edit_is_not_overwritten(tmp_path):
    async def scenario():
        cell, p = make(tmp_path, camera="", table_z_mm=TABLE_Z)
        await p.handle("start_manual", {})
        cell.T_gripper = geo.pointing_down(300, -200, TABLE_Z + TRUE_REACH, 0)
        await p.handle("touch", {})
        await p.handle("stop_manual", {})
        original_read = cell.part_config.read

        async def read_then_someone_edits():
            config, version = await original_read()
            cell.part_config.version += 1  # an edit in the Viam app between our read and write
            return config, version
        cell.part_config.read = read_then_someone_edits
        result = await p.handle("finish", {})
        assert result["applied"] is False and "changed since" in result["apply_error"]
        assert cell.part_config.writes == []
    run(scenario())


def test_status_reports_the_live_fingertip_gap(tmp_path):
    async def scenario():
        cell, p = make(tmp_path, camera="", table_z_mm=TABLE_Z)
        cell.T_gripper = geo.pointing_down(300, -200, TABLE_Z + MODEL_REACH + 12.0, 0)
        status = await p.handle("status", {})
        assert status["gripper"]["fingertip_gap_mm"] == pytest.approx(12.0)
        assert status["gripper"]["gap_from"] == "model"
        assert status["table"]["source"] == "config"
    run(scenario())


def test_bad_arguments_are_explained(tmp_path):
    _, p = make(tmp_path)
    with pytest.raises(TouchOffError, match="takes: mm"):
        run(p.handle("jog", {"millimeters": 1}))
    with pytest.raises(TouchOffError, match="Unknown command"):
        run(p.handle("dance", {}))


def test_parse_command_forms():
    assert parse_command({"command": "jog", "mm": 1}) == ("jog", {"mm": 1})
    assert parse_command({"jog": {"mm": 1}}) == ("jog", {"mm": 1})
    assert parse_command({"status": True}) == ("status", {})
    with pytest.raises(TouchOffError):
        parse_command({"a": 1, "b": 2})


def test_settings_validation():
    assert Settings.from_attributes({"arm": "a", "gripper": "g", "depth_frames": 3.0}).depth_frames == 3
    with pytest.raises(ValueError, match="gripper"):
        Settings.from_attributes({"arm": "a"})
    with pytest.raises(ValueError, match="Unknown attribute"):
        Settings.from_attributes({"arm": "a", "gripper": "g", "speed": 3})
    with pytest.raises(ValueError, match="number"):
        Settings.from_attributes({"arm": "a", "gripper": "g", "table_z_mm": "low"})
    with pytest.raises(ValueError, match="greater than 0"):
        Settings.from_attributes({"arm": "a", "gripper": "g", "max_step_mm": 0})
    with pytest.raises(ValueError, match="at most 15"):
        Settings.from_attributes({"arm": "a", "gripper": "g", "max_tilt_deg": 30})
    with pytest.raises(ValueError, match="list of names"):
        Settings.from_attributes({"arm": "a", "gripper": "g", "contact_obstacles": "table"})


def test_responses_drop_nulls():
    assert without_nulls({"a": None, "b": {"c": None, "d": 1}, "e": [{"f": None}]}) == {"b": {"d": 1}, "e": [{}]}


def test_a_gripper_from_a_fragment_is_overridden_for_this_machine_only(tmp_path):
    async def scenario():
        cell, p = make(tmp_path, Cell(in_fragment=True), camera="", table_z_mm=TABLE_Z)
        fragment_before = cell.fragment
        await p.handle("start_manual", {})
        cell.T_gripper = geo.pointing_down(300, -200, TABLE_Z + TRUE_REACH, 0)
        await p.handle("touch", {})
        await p.handle("stop_manual", {})
        result = await p.handle("finish", {})
        assert result["applied"] is True and "fragment xarm-gripper" in result["applied_to"]
        assert cell.fragment == fragment_before  # the shared fragment isn't touched
        mods = cell.config["fragment_mods"][0]["mods"]
        assert mods[0] == {"$set": {"components.arm.attributes.speed_degs_per_sec": 20}}  # kept
        assert mods[-1]["$set"]["components.gripper.frame.translation"]["z"] == pytest.approx(105 + TRUE_REACH - MODEL_REACH)
        assert cell.true_reach == pytest.approx(MODEL_REACH)
        # A second touch-off replaces the override instead of stacking another.
        await p.handle("start_manual", {})
        cell.T_gripper = geo.pointing_down(300, -200, TABLE_Z + MODEL_REACH + 1.0, 0)  # 1mm still off
        await p.handle("touch", {"paper_mm": 1.0})
        await p.handle("stop_manual", {})
        result = await p.handle("finish", {})
        assert result["applied"] and abs(result["frame_shift_mm"]) < 1e-6 or "already matches" in result.get("apply_note", "")
        await p.handle("undo", {})
        overrides = [m for m in cell.config["fragment_mods"][0]["mods"] if "components.gripper.frame.translation" in m.get("$set", {})]
        assert len(overrides) == 1 and overrides[0]["$set"]["components.gripper.frame.translation"]["z"] == 105
    run(scenario())
