"""Plugin metadata and scenario schema — v2.0."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

VALID_PLUGIN_TYPES = {"environment", "robot", "asset"}


@dataclass
class IsaacEntry:
    usd: Optional[str] = None   # USD scene/model file (relative to plugin dir)
    app: Optional[str] = None   # Isaac Sim standalone Python script


@dataclass
class Ros2Entry:
    workspace: Optional[str] = None          # colcon workspace path (relative to plugin dir)
    launch: Optional[str] = None             # ROS2 .launch.py file
    launch_args: Dict[str, str] = field(default_factory=dict)
    config: Optional[str] = None


@dataclass
class TopicSpec:
    topic: str
    msg_type: str


@dataclass
class ActionSpec:
    name: str
    action_type: str


@dataclass
class Ros2Interface:
    namespace: str = ""
    publishes: List[TopicSpec] = field(default_factory=list)
    subscribes: List[TopicSpec] = field(default_factory=list)
    action_servers: List[ActionSpec] = field(default_factory=list)


@dataclass
class PluginMetadata:
    name: str
    plugin_type: str                                               # environment | robot | asset
    version: str = "1.0.0"
    description: str = ""
    isaac_lab: Optional[str] = None                                # e.g. ">=2.0.0"
    ros_distro: Optional[str] = None                               # always "jazzy"
    isaac_entry: IsaacEntry = field(default_factory=IsaacEntry)
    ros2_entry: Ros2Entry = field(default_factory=Ros2Entry)
    ros2_interface: Ros2Interface = field(default_factory=Ros2Interface)
    dep_plugins: List[str] = field(default_factory=list)
    author: str = ""
    license: str = ""
    repository: str = ""


# ---------------------------------------------------------------------------
# Scenario
# ---------------------------------------------------------------------------

@dataclass
class SpawnPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class RobotInstance:
    plugin: str                  # plugin name (must match a robot plugin)
    instance: str                # unique name for this robot in the scenario
    namespace: str = ""          # ROS2 namespace — defaults to "/<instance>" if empty
    spawn: SpawnPose = field(default_factory=SpawnPose)

    def __post_init__(self):
        if not self.namespace:
            self.namespace = f"/{self.instance}"


@dataclass
class Scenario:
    name: str
    description: str = ""
    world: str = ""              # environment plugin name (empty = no world)
    robots: List[RobotInstance] = field(default_factory=list)
