"""Scenario file parser and validator."""
from __future__ import annotations

from pathlib import Path
from typing import List

import yaml

from .schema import RobotInstance, Scenario, SpawnPose, PluginMetadata


def parse_scenario(path: Path) -> Scenario:
    """Parse a scenario YAML file into a Scenario dataclass."""
    if not path.exists():
        raise FileNotFoundError(f"Scenario file not found: {path}")

    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"Scenario file must be a YAML mapping: {path}")
    if "name" not in raw:
        raise ValueError(f"Scenario file missing required field 'name': {path}")

    robots: List[RobotInstance] = []
    for i, r in enumerate(raw.get("robots") or []):
        if not isinstance(r, dict):
            raise ValueError(f"robots[{i}] must be a mapping")
        if "plugin" not in r:
            raise ValueError(f"robots[{i}] missing required field 'plugin'")
        if "instance" not in r:
            raise ValueError(f"robots[{i}] missing required field 'instance'")

        spawn_raw = r.get("spawn") or {}
        spawn = SpawnPose(
            x=float(spawn_raw.get("x", 0.0)),
            y=float(spawn_raw.get("y", 0.0)),
            z=float(spawn_raw.get("z", 0.0)),
            roll=float(spawn_raw.get("roll", 0.0)),
            pitch=float(spawn_raw.get("pitch", 0.0)),
            yaw=float(spawn_raw.get("yaw", 0.0)),
        )
        robots.append(RobotInstance(
            plugin=r["plugin"],
            instance=r["instance"],
            namespace=r.get("namespace", ""),
            spawn=spawn,
        ))

    return Scenario(
        name=raw["name"],
        description=raw.get("description", ""),
        world=raw.get("world", ""),
        robots=robots,
    )


def validate_scenario(scenario: Scenario, plugins: List[PluginMetadata]) -> List[str]:
    """
    Check that all plugins referenced in *scenario* exist and have the right type.
    Returns a list of error strings (empty = OK).
    """
    errors: List[str] = []
    by_name = {p.name: p for p in plugins}

    # World must exist and be an environment plugin
    if scenario.world:
        if scenario.world not in by_name:
            errors.append(f"World plugin '{scenario.world}' not found in plugin/")
        elif by_name[scenario.world].plugin_type != "environment":
            errors.append(
                f"'{scenario.world}' is type '{by_name[scenario.world].plugin_type}', expected 'environment'"
            )

    # Each robot plugin must exist and be a robot plugin
    seen_instances: set[str] = set()
    for ri in scenario.robots:
        if ri.instance in seen_instances:
            errors.append(f"Duplicate robot instance name '{ri.instance}'")
        seen_instances.add(ri.instance)

        if ri.plugin not in by_name:
            errors.append(f"Robot plugin '{ri.plugin}' (instance '{ri.instance}') not found in plugin/")
        elif by_name[ri.plugin].plugin_type != "robot":
            errors.append(
                f"'{ri.plugin}' is type '{by_name[ri.plugin].plugin_type}', expected 'robot'"
            )

    # Plugin dep_plugins must be satisfied by what's in the scenario
    scenario_plugins = set()
    if scenario.world:
        scenario_plugins.add(scenario.world)
    for ri in scenario.robots:
        scenario_plugins.add(ri.plugin)

    for pname in scenario_plugins:
        if pname not in by_name:
            continue
        for dep in by_name[pname].dep_plugins:
            if dep not in scenario_plugins:
                errors.append(
                    f"'{pname}' depends on '{dep}' which is not in this scenario"
                )

    return errors
