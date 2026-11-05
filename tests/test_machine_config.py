import asyncio

import pytest

from touchoff.machine_config import (ConfigUnavailable, apply_mods, locate_gripper_frame, substitute_variables,
                                     with_translation)

# The shape of armfarm12's real config: the gripper comes from a fragment whose frame parent is a variable.
FRAGMENT = {"components": [
    {"name": "arm", "model": "viam:ufactory:xArm6", "frame": {"parent": "world"}},
    {"name": "gripper", "model": "viam:ufactory:gripper",
     "frame": {"parent": {"$variable": {"name": "arm-name"}},
               "translation": {"x": 0, "y": 0, "z": 105},
               "orientation": {"type": "ov_degrees", "value": {"x": 0, "y": 0, "z": 1, "th": 180}}}}]}
PART = {"components": [{"name": "cam", "model": "viam:camera:realsense"}],
        "fragments": [{"id": "f-other"}, {"id": "f-arm", "variables": {"arm-name": "arm"}}],
        "fragment_mods": [{"fragment_id": "f-arm", "mods": [{"$set": {"components.arm.attributes.speed": 20}}]}]}


async def fetch(fragment_id):
    return ("other", {"components": []}) if fragment_id == "f-other" else ("xarm", FRAGMENT)


def test_variables_are_substituted():
    assert substitute_variables({"a": {"$variable": {"name": "v"}}}, {"v": 3}) == {"a": 3}
    assert substitute_variables({"$variable": {"name": "v", "default_value": 1}}, {}) == 1
    with pytest.raises(ConfigUnavailable):
        substitute_variables({"$variable": {"name": "v"}}, {})


def test_mods_set_and_unset():
    component = {"frame": {"translation": {"z": 105}}, "attributes": {"a": 1}}
    mods = [{"$set": {"components.g.frame.translation": {"x": 0, "y": 0, "z": 120}}},
            {"$unset": {"components.g.attributes.a": ""}}, {"$set": {"components.other.frame.parent": "x"}}]
    assert apply_mods(component, "g", mods) == {"frame": {"translation": {"x": 0, "y": 0, "z": 120}}, "attributes": {}}


def test_a_gripper_in_the_part_is_edited_in_place():
    part = {"components": [{"name": "gripper", "frame": {"parent": "arm", "translation": {"x": 1, "y": 2, "z": 3}}}]}
    site = asyncio.run(locate_gripper_frame(part, "gripper", None))
    assert site.kind == "part" and site.translation() == {"x": 1, "y": 2, "z": 3}
    updated = with_translation(part, site, "gripper", {"x": 1, "y": 2, "z": 9})
    assert updated["components"][0]["frame"] == {"parent": "arm", "translation": {"x": 1.0, "y": 2.0, "z": 9.0}}
    assert part["components"][0]["frame"]["translation"]["z"] == 3  # the original isn't modified


def test_a_gripper_in_a_fragment_gets_a_fragment_mod():
    site = asyncio.run(locate_gripper_frame(PART, "gripper", fetch))
    assert site.kind == "fragment" and site.fragment_id == "f-arm" and site.parent == "arm"
    assert site.translation() == {"x": 0, "y": 0, "z": 105}
    updated = with_translation(PART, site, "gripper", {"x": 0, "y": 0, "z": 129})
    assert updated["fragment_mods"] == [{"fragment_id": "f-arm", "mods": [
        {"$set": {"components.arm.attributes.speed": 20}},
        {"$set": {"components.gripper.frame.translation": {"x": 0.0, "y": 0.0, "z": 129.0}}}]}]
    # Read back, the override is what the machine now sees; writing again replaces it.
    site = asyncio.run(locate_gripper_frame(updated, "gripper", fetch))
    assert site.translation()["z"] == 129
    again = with_translation(updated, site, "gripper", {"x": 0, "y": 0, "z": 130})
    assert again["fragment_mods"][0]["mods"][1:] == [{"$set": {"components.gripper.frame.translation": {"x": 0.0, "y": 0.0, "z": 130.0}}}]


def test_an_earlier_per_axis_override_is_replaced():
    part = dict(PART, fragment_mods=[{"fragment_id": "f-arm", "mods": [
        {"$set": {"components.gripper.frame.translation.z": 110, "components.arm.attributes.speed": 20}}]}])
    site = asyncio.run(locate_gripper_frame(part, "gripper", fetch))
    assert site.translation()["z"] == 110
    updated = with_translation(part, site, "gripper", {"x": 0, "y": 0, "z": 120})
    assert updated["fragment_mods"][0]["mods"] == [
        {"$set": {"components.arm.attributes.speed": 20}},
        {"$set": {"components.gripper.frame.translation": {"x": 0.0, "y": 0.0, "z": 120.0}}}]


def test_fragment_without_existing_mods_gets_an_entry():
    part = {"components": [], "fragments": [{"id": "f-arm", "variables": {"arm-name": "arm"}}]}
    site = asyncio.run(locate_gripper_frame(part, "gripper", fetch))
    updated = with_translation(part, site, "gripper", {"x": 0, "y": 0, "z": 100})
    assert updated["fragment_mods"] == [{"fragment_id": "f-arm", "mods": [
        {"$set": {"components.gripper.frame.translation": {"x": 0.0, "y": 0.0, "z": 100.0}}}]}]


def test_unsupported_places_are_explained():
    with pytest.raises(ConfigUnavailable, match="isn't in this machine part's config or a fragment"):
        asyncio.run(locate_gripper_frame({"components": [], "fragments": ["f-other"]}, "gripper", fetch))
    prefixed = {"components": [], "fragments": [{"id": "f-arm", "prefix": "left-", "variables": {"arm-name": "arm"}}]}
    with pytest.raises(ConfigUnavailable, match="prefix"):
        asyncio.run(locate_gripper_frame(prefixed, "left-gripper", fetch))
    no_parent = {"components": [{"name": "gripper", "frame": {"translation": {"z": 1}}}]}
    with pytest.raises(ConfigUnavailable, match="no frame parent"):
        asyncio.run(locate_gripper_frame(no_parent, "gripper", None))
