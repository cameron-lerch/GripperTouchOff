"""Read and write the gripper's frame in the machine part's config through Viam's App API.

A module gets credentials for this from viam-server: VIAM_API_KEY, VIAM_API_KEY_ID and
VIAM_MACHINE_PART_ID are set when the machine is connected to Viam's cloud.

The gripper is either defined in the part's own config, or comes from a fragment the part
imports (common on fleets). In the first case its `frame.translation` is changed in place; in
the second, the shared fragment is left alone and a `fragment_mods` override is written for
this machine only:

    "fragment_mods": [{"fragment_id": "<id>", "mods": [
        {"$set": {"components.<gripper>.frame.translation": {"x": 0, "y": 0, "z": 129.1}}}]}]

Everything else in the config, including the order of its keys, is written back as it was read.
"""

from __future__ import annotations

import copy
import json
import os
import tempfile
from dataclasses import dataclass
from typing import Any, Awaitable, Callable


class ConfigUnavailable(RuntimeError):
    """The machine config can't be read or written from here (no cloud credentials, or the
    gripper is defined somewhere this module doesn't edit)."""


@dataclass
class FrameSite:
    """Where the gripper's frame is defined, and the frame as the machine sees it."""
    kind: str  # "part" or "fragment"
    frame: dict  # with fragment variables substituted and this part's fragment_mods applied
    fragment_id: str = ""
    fragment_name: str = ""

    @property
    def parent(self) -> str:
        return self.frame["parent"]

    def translation(self) -> dict | None:
        """{x, y, z} as numbers, or None when it isn't plain numbers."""
        t = self.frame.get("translation") or {}
        if not isinstance(t, dict) or not all(isinstance(t.get(k, 0), (int, float)) for k in "xyz"):
            return None
        return {k: float(t.get(k, 0.0)) for k in "xyz"}

    def describe(self) -> str:
        return "this machine part's config" if self.kind == "part" else \
            f"fragment {self.fragment_name or self.fragment_id} (overridden in this part's fragment_mods)"


def find_component(config: dict, name: str) -> dict | None:
    for component in config.get("components", []) or []:
        if isinstance(component, dict) and component.get("name") == name:
            return component
    return None


def substitute_variables(value: Any, variables: dict) -> Any:
    """Fragment `{"$variable": {"name": ..., "default_value": ...}}` placeholders, filled in."""
    if isinstance(value, dict):
        if set(value) == {"$variable"} and isinstance(value["$variable"], dict):
            spec = value["$variable"]
            if spec.get("name") in variables:
                return variables[spec["name"]]
            if "default_value" in spec:
                return spec["default_value"]
            raise ConfigUnavailable(f"Fragment variable {spec.get('name')!r} has no value")
        return {k: substitute_variables(v, variables) for k, v in value.items()}
    if isinstance(value, list):
        return [substitute_variables(v, variables) for v in value]
    return value


def _apply_path(target: dict, path: list[str], value: Any, unset: bool) -> None:
    for key in path[:-1]:
        if not isinstance(target.get(key), dict):
            if unset:
                return
            target[key] = {}
        target = target[key]
    if unset:
        target.pop(path[-1], None)
    else:
        target[path[-1]] = copy.deepcopy(value)


def apply_mods(component: dict, name: str, mods: list) -> dict:
    """The component after this part's fragment_mods ($set and $unset on components.<name>...)."""
    component = copy.deepcopy(component)
    prefix = f"components.{name}."
    for mod in mods or []:
        for operator, unset in (("$set", False), ("$unset", True)):
            for key, value in (mod.get(operator) or {}).items():
                if key.startswith(prefix):
                    _apply_path(component, key[len(prefix):].split("."), value, unset)
    return component


def _mods_for(config: dict, fragment_id: str) -> list:
    for entry in config.get("fragment_mods", []) or []:
        if entry.get("fragment_id") == fragment_id:
            return entry.get("mods", []) or []
    return []


FetchFragment = Callable[[str], Awaitable[tuple[str, dict]]]  # fragment id -> (name, fragment config)


async def locate_gripper_frame(config: dict, gripper: str, fetch_fragment: FetchFragment | None) -> FrameSite:
    """Find the gripper's frame in the part's config or one of the fragments it imports."""
    component = find_component(config, gripper)
    if component is not None:
        site = FrameSite("part", copy.deepcopy(component.get("frame") or {}))
    else:
        site = None
        for entry in config.get("fragments", []) or []:
            fragment_id = entry if isinstance(entry, str) else entry.get("id", "")
            if not fragment_id or (isinstance(entry, dict) and entry.get("disabled")):
                continue
            if fetch_fragment is None:
                break
            name, fragment = await fetch_fragment(fragment_id)
            if isinstance(entry, dict) and entry.get("prefix"):
                if find_component(fragment, gripper[len(entry["prefix"]):]) is not None:
                    raise ConfigUnavailable(f"{gripper!r} comes from fragment {name!r} imported with a prefix, which this "
                                            "module doesn't edit; copy the returned frame into the config by hand")
                continue
            found = find_component(fragment, gripper)
            if found is None:
                continue
            variables = entry.get("variables", {}) if isinstance(entry, dict) else {}
            found = apply_mods(substitute_variables(found, variables), gripper, _mods_for(config, fragment_id))
            site = FrameSite("fragment", found.get("frame") or {}, fragment_id, name)
            break
        if site is None:
            raise ConfigUnavailable(
                f"Component {gripper!r} isn't in this machine part's config or a fragment it imports directly "
                "(nested fragments aren't edited); copy the returned frame into the config by hand")
    if not isinstance(site.frame.get("parent"), str) or not site.frame.get("parent"):
        raise ConfigUnavailable(f"Component {gripper!r} has no frame parent; give it one (usually the arm) first")
    return site


def with_translation(config: dict, site: FrameSite, gripper: str, translation: dict) -> dict:
    """A copy of `config` that gives the gripper's frame `translation`; nothing else changes."""
    updated = copy.deepcopy(config)
    translation = {k: float(translation[k]) for k in "xyz"}
    if site.kind == "part":
        component = find_component(updated, gripper)
        component.setdefault("frame", {})["translation"] = translation
        return updated
    key = f"components.{gripper}.frame.translation"
    entries = updated.setdefault("fragment_mods", [])
    entry = next((e for e in entries if e.get("fragment_id") == site.fragment_id), None)
    if entry is None:
        entry = {"fragment_id": site.fragment_id, "mods": []}
        entries.append(entry)
    mods = []
    for mod in entry.get("mods", []) or []:  # drop earlier overrides of the same translation
        mod = {op: {k: v for k, v in (fields or {}).items() if not (k == key or k.startswith(key + "."))}
               for op, fields in mod.items()}
        mod = {op: fields for op, fields in mod.items() if fields}
        if mod:
            mods.append(mod)
    mods.append({"$set": {key: translation}})
    entry["mods"] = mods
    return updated


class PartConfig:
    """This machine part's config, as stored in Viam's cloud."""

    def __init__(self, part_id: str | None = None):
        self.part_id = part_id or os.environ.get("VIAM_MACHINE_PART_ID", "")

    def _check_env(self) -> None:
        missing = [v for v in ("VIAM_API_KEY", "VIAM_API_KEY_ID") if not os.environ.get(v)]
        if not self.part_id:
            missing.append("VIAM_MACHINE_PART_ID")
        if missing:
            raise ConfigUnavailable(
                f"Can't update the machine config: {', '.join(missing)} not set (viam-server sets these for "
                "modules on machines connected to Viam's cloud). Copy the returned frame into the config by hand.")

    async def _client(self):
        from viam.app.viam_client import ViamClient

        self._check_env()
        return await ViamClient.create_from_env_vars()

    async def read(self) -> tuple[dict, Any]:
        """(config dict with its key order kept, the part's last-updated time)."""
        client = await self._client()
        try:
            # get_robot_part only exposes the config with its keys' order through `dest`.
            with tempfile.TemporaryDirectory() as folder:
                path = os.path.join(folder, "config.json")
                part = await client.app_client.get_robot_part(self.part_id, dest=path)
                with open(path, encoding="utf-8") as file:
                    text = file.read()
            config = json.loads(text) if text.strip() else dict(part.robot_config or {})
            return config, part.last_updated
        finally:
            client.close()

    async def fragment(self, fragment_id: str) -> tuple[str, dict]:
        client = await self._client()
        try:
            fragment = await client.app_client.get_fragment(fragment_id)
            return fragment.name, dict(fragment.fragment or {})
        finally:
            client.close()

    async def write(self, config: dict, last_updated=None) -> None:
        """Write `config` back. With `last_updated` (from `read`), the App refuses the write if the
        config changed since it was read, so a concurrent edit in the Viam app isn't overwritten."""
        client = await self._client()
        try:
            part = await client.app_client.get_robot_part(self.part_id)
            await client.app_client.update_robot_part(
                self.part_id, part.name, robot_config=config, last_known_update=last_updated,
                robot_config_json=json.dumps(config, indent=2))
        finally:
            client.close()
