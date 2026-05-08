"""METADATA.yaml parser and compatibility checker — v2.0."""
from __future__ import annotations

from pathlib import Path
from typing import List

import yaml

from .schema import (
    ActionSpec,
    IsaacEntry,
    PluginMetadata,
    Ros2Entry,
    Ros2Interface,
    TopicSpec,
    VALID_PLUGIN_TYPES,
)

# Accept old "world" type from v1.0 files and map it to "environment"
_TYPE_ALIASES = {"world": "environment", "app": "asset", "logic": "asset"}


def parse_metadata(plugin_path: Path) -> PluginMetadata:
    """Parse and validate METADATA.yaml in *plugin_path*."""
    meta_file = plugin_path / "METADATA.yaml"
    if not meta_file.exists():
        raise FileNotFoundError(f"METADATA.yaml not found in {plugin_path}")

    raw = yaml.safe_load(meta_file.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"METADATA.yaml in {plugin_path} must be a YAML mapping")

    plugin_type = raw.get("plugin_type", "")
    plugin_type = _TYPE_ALIASES.get(plugin_type, plugin_type)
    if plugin_type not in VALID_PLUGIN_TYPES:
        raise ValueError(
            f"plugin_type must be one of {sorted(VALID_PLUGIN_TYPES)}, got '{raw.get('plugin_type')}'"
        )

    compat = raw.get("compatibility") or {}

    return PluginMetadata(
        name=raw["name"],
        plugin_type=plugin_type,
        version=str(raw.get("version", "1.0.0")),
        description=raw.get("description", ""),
        isaac_lab=compat.get("isaac_lab") if isinstance(compat, dict) else None,
        ros_distro=compat.get("ros_distro") if isinstance(compat, dict) else None,
        isaac_entry=_parse_isaac_entry(raw),
        ros2_entry=_parse_ros2_entry(raw),
        ros2_interface=_parse_ros2_interface(raw),
        dep_plugins=_parse_dep_plugins(raw),
        author=raw.get("author", ""),
        license=raw.get("license", ""),
        repository=raw.get("repository", ""),
    )


def _parse_isaac_entry(raw: dict) -> IsaacEntry:
    if "isaac_entry" in raw:
        ie = raw["isaac_entry"] or {}
        return IsaacEntry(usd=ie.get("usd"), app=ie.get("app"))

    # v1.0 fallback: entry_point block
    ep = raw.get("entry_point") or {}
    if isinstance(ep, dict):
        return IsaacEntry(usd=ep.get("usd"), app=ep.get("app"))

    return IsaacEntry()


def _parse_ros2_entry(raw: dict) -> Ros2Entry:
    if "ros2_entry" in raw:
        re = raw["ros2_entry"] or {}
        return Ros2Entry(
            workspace=re.get("workspace"),
            launch=re.get("launch"),
            launch_args=re.get("launch_args") or {},
            config=re.get("config"),
        )

    # v1.0 fallback: entry_point.launch
    ep = raw.get("entry_point") or {}
    if isinstance(ep, dict) and ep.get("launch"):
        return Ros2Entry(launch=ep.get("launch"), config=ep.get("config"))

    return Ros2Entry()


def _parse_ros2_interface(raw: dict) -> Ros2Interface:
    ri = raw.get("ros2_interface") or {}
    if not isinstance(ri, dict):
        return Ros2Interface()

    def _topics(items) -> list:
        out = []
        for item in (items or []):
            if isinstance(item, dict):
                out.append(TopicSpec(topic=item.get("topic", ""), msg_type=item.get("type", "")))
        return out

    def _actions(items) -> list:
        out = []
        for item in (items or []):
            if isinstance(item, dict):
                out.append(ActionSpec(name=item.get("name", ""), action_type=item.get("type", "")))
        return out

    return Ros2Interface(
        namespace=ri.get("namespace", ""),
        publishes=_topics(ri.get("publishes")),
        subscribes=_topics(ri.get("subscribes")),
        action_servers=_actions(ri.get("action_servers")),
    )


def _parse_dep_plugins(raw: dict) -> List[str]:
    # v2.0: dep_plugins list
    deps = raw.get("dep_plugins")
    if isinstance(deps, list):
        return [str(d) for d in deps]

    # v1.0 fallback: dependencies.plugins
    deps_block = raw.get("dependencies") or {}
    if isinstance(deps_block, dict):
        return [str(d) for d in (deps_block.get("plugins") or [])]

    return []


def validate_compatibility(plugins: List[PluginMetadata]) -> List[str]:
    """Return list of compatibility error strings (empty = all OK)."""
    errors: List[str] = []
    if not plugins:
        return errors

    # All plugins that declare ros_distro must agree (and must be jazzy)
    for p in plugins:
        if p.ros_distro and p.ros_distro != "jazzy":
            errors.append(
                f"'{p.name}' declares ros_distro='{p.ros_distro}' but only 'jazzy' is supported"
            )

    # Every declared plugin dependency must be present
    loaded = {p.name for p in plugins}
    for plugin in plugins:
        for dep in plugin.dep_plugins:
            if dep not in loaded:
                errors.append(
                    f"'{plugin.name}' depends on '{dep}' which is not loaded"
                )

    return errors
