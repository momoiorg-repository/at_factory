"""Tests for plugsim.parser — schema v2.0."""
import pytest
from pathlib import Path

from plugsim.parser import parse_metadata, validate_compatibility


# --- v2.0 fixtures ---

VALID_ENVIRONMENT = """\
schema_version: "2.0"
plugin_type: environment
name: example_factory_world
version: 1.0.0
description: "Test environment"
compatibility:
  isaac_lab: ">=2.0.0"
  ros_distro: jazzy
isaac_entry:
  usd: assets/scene.usd
  app: app.py
"""

VALID_ROBOT = """\
schema_version: "2.0"
plugin_type: robot
name: example_melon_ros2
version: 1.0.0
description: "Test robot"
compatibility:
  isaac_lab: ">=2.0.0"
  ros_distro: jazzy
isaac_entry:
  usd: assets/robot.usd
ros2_entry:
  launch: melon_ws/src/bringup/launch/robot.launch.py
ros2_interface:
  namespace: /melon
  publishes:
    - topic: joint_states
      type: sensor_msgs/JointState
  subscribes:
    - topic: cmd_vel
      type: geometry_msgs/Twist
dep_plugins:
  - example_factory_world
"""

VALID_ROBOT_WITH_WORKSPACE = """\
schema_version: "2.0"
plugin_type: robot
name: fanuc_crx10ia
version: 1.0.0
compatibility:
  ros_distro: jazzy
ros2_entry:
  workspace: .
  launch: fanuc_hardware_interface/launch/fanuc_mock_control.launch.py
  launch_args:
    robot_model: crx10ia_l
    robot_series: crx
"""

VALID_ASSET = """\
schema_version: "2.0"
plugin_type: asset
name: pallet_stack
version: 1.0.0
isaac_entry:
  usd: assets/pallet.usd
"""

# --- v1.0 compatibility fixtures ---

V1_WORLD = """\
schema_version: "1.0"
plugin_type: world
name: old_world
version: 1.0.0
compatibility:
  isaac_sim: ">=5.0.0"
  ros_distro: jazzy
entry_point:
  usd: assets/scene.usd
  app: app.py
"""

V1_ROBOT = """\
schema_version: "1.0"
plugin_type: robot
name: old_robot
version: 1.0.0
compatibility:
  ros_distro: jazzy
entry_point:
  launch: launch/robot.launch.py
dependencies:
  plugins:
    - old_world
"""


def make_dir(tmp_path, content, name="plugin"):
    d = tmp_path / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "METADATA.yaml").write_text(content, encoding="utf-8")
    return d


# --- parse_metadata: v2.0 ---

def test_parse_environment(tmp_path):
    m = parse_metadata(make_dir(tmp_path, VALID_ENVIRONMENT))
    assert m.name == "example_factory_world"
    assert m.plugin_type == "environment"
    assert m.isaac_entry.app == "app.py"
    assert m.isaac_entry.usd == "assets/scene.usd"
    assert m.ros_distro == "jazzy"
    assert m.isaac_lab == ">=2.0.0"


def test_parse_robot(tmp_path):
    m = parse_metadata(make_dir(tmp_path, VALID_ROBOT))
    assert m.plugin_type == "robot"
    assert m.isaac_entry.usd == "assets/robot.usd"
    assert m.ros2_entry.launch.endswith(".launch.py")
    assert m.ros2_interface.namespace == "/melon"
    assert len(m.ros2_interface.publishes) == 1
    assert m.ros2_interface.publishes[0].topic == "joint_states"
    assert len(m.ros2_interface.subscribes) == 1
    assert m.dep_plugins == ["example_factory_world"]


def test_parse_robot_with_workspace(tmp_path):
    m = parse_metadata(make_dir(tmp_path, VALID_ROBOT_WITH_WORKSPACE))
    assert m.ros2_entry.workspace == "."
    assert m.ros2_entry.launch_args == {"robot_model": "crx10ia_l", "robot_series": "crx"}


def test_parse_asset(tmp_path):
    m = parse_metadata(make_dir(tmp_path, VALID_ASSET))
    assert m.plugin_type == "asset"
    assert m.isaac_entry.usd == "assets/pallet.usd"


# --- parse_metadata: v1.0 backward compatibility ---

def test_v1_world_maps_to_environment(tmp_path):
    m = parse_metadata(make_dir(tmp_path, V1_WORLD))
    assert m.plugin_type == "environment"          # world → environment
    assert m.isaac_entry.usd == "assets/scene.usd" # entry_point → isaac_entry
    assert m.isaac_entry.app == "app.py"


def test_v1_robot_launch_maps_to_ros2_entry(tmp_path):
    m = parse_metadata(make_dir(tmp_path, V1_ROBOT))
    assert m.plugin_type == "robot"
    assert m.ros2_entry.launch == "launch/robot.launch.py"
    assert m.dep_plugins == ["old_world"]           # dependencies.plugins → dep_plugins


# --- error cases ---

def test_missing_metadata(tmp_path):
    d = tmp_path / "empty"
    d.mkdir()
    with pytest.raises(FileNotFoundError):
        parse_metadata(d)


def test_invalid_plugin_type(tmp_path):
    bad = VALID_ENVIRONMENT.replace("plugin_type: environment", "plugin_type: spaceship")
    with pytest.raises(ValueError, match="plugin_type"):
        parse_metadata(make_dir(tmp_path, bad))


def test_not_a_mapping(tmp_path):
    with pytest.raises(ValueError):
        parse_metadata(make_dir(tmp_path, "- list item\n"))


# --- validate_compatibility ---

def test_compatible_plugins(tmp_path):
    env = parse_metadata(make_dir(tmp_path, VALID_ENVIRONMENT, "env"))
    robot = parse_metadata(make_dir(tmp_path, VALID_ROBOT, "robot"))
    assert validate_compatibility([env, robot]) == []


def test_non_jazzy_distro_rejected(tmp_path):
    humble = VALID_ROBOT.replace("ros_distro: jazzy", "ros_distro: humble")
    r = parse_metadata(make_dir(tmp_path, humble, "robot"))
    errors = validate_compatibility([r])
    assert any("jazzy" in e for e in errors)


def test_missing_dep_plugin(tmp_path):
    # VALID_ROBOT depends on example_factory_world which is not loaded
    r = parse_metadata(make_dir(tmp_path, VALID_ROBOT, "robot"))
    errors = validate_compatibility([r])
    assert any("example_factory_world" in e for e in errors)


def test_dep_satisfied_when_present(tmp_path):
    env = parse_metadata(make_dir(tmp_path, VALID_ENVIRONMENT, "env"))
    robot = parse_metadata(make_dir(tmp_path, VALID_ROBOT, "robot"))
    assert validate_compatibility([env, robot]) == []


def test_empty_list():
    assert validate_compatibility([]) == []
