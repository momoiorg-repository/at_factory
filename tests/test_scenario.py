"""Tests for plugsim.scenario — parse and validate scenario files."""
import pytest
from pathlib import Path

from plugsim.scenario import parse_scenario, validate_scenario
from plugsim.parser import parse_metadata
from plugsim.schema import Scenario, RobotInstance


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def write_scenario(tmp_path, content, name="test_scenario.yaml"):
    p = tmp_path / name
    p.write_text(content, encoding="utf-8")
    return p


def make_plugin(tmp_path, content, name):
    d = tmp_path / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "METADATA.yaml").write_text(content, encoding="utf-8")
    return parse_metadata(d)


ENV_PLUGIN = """\
schema_version: "2.0"
plugin_type: environment
name: example_factory_world
version: 1.0.0
compatibility:
  ros_distro: jazzy
isaac_entry:
  usd: assets/scene.usd
"""

ROBOT_PLUGIN = """\
schema_version: "2.0"
plugin_type: robot
name: example_melon_ros2
version: 1.0.0
compatibility:
  ros_distro: jazzy
ros2_entry:
  launch: launch/robot.launch.py
dep_plugins:
  - example_factory_world
"""

ROBOT_NO_DEP = """\
schema_version: "2.0"
plugin_type: robot
name: fanuc_crx10ia
version: 1.0.0
compatibility:
  ros_distro: jazzy
ros2_entry:
  launch: fanuc_hardware_interface/launch/fanuc_mock_control.launch.py
"""

# ---------------------------------------------------------------------------
# parse_scenario
# ---------------------------------------------------------------------------

VALID_SCENARIO = """\
name: factory_melon
description: "Factory with Melon robot"
world: example_factory_world
robots:
  - plugin: example_melon_ros2
    instance: melon
    namespace: /melon
    spawn:
      x: 1.0
      y: 0.5
      z: 0.0
      yaw: 1.57
"""

MULTI_ROBOT_SCENARIO = """\
name: factory_dual_fanuc
world: example_factory_world
robots:
  - plugin: fanuc_crx10ia
    instance: fanuc1
    spawn:
      x: 0.5
  - plugin: fanuc_crx10ia
    instance: fanuc2
    namespace: /arm2
    spawn:
      x: -0.5
      yaw: 3.14
"""


def test_parse_basic(tmp_path):
    p = write_scenario(tmp_path, VALID_SCENARIO)
    s = parse_scenario(p)
    assert s.name == "factory_melon"
    assert s.world == "example_factory_world"
    assert len(s.robots) == 1
    r = s.robots[0]
    assert r.plugin == "example_melon_ros2"
    assert r.instance == "melon"
    assert r.namespace == "/melon"
    assert r.spawn.x == 1.0
    assert r.spawn.yaw == 1.57


def test_default_namespace_from_instance(tmp_path):
    p = write_scenario(tmp_path, MULTI_ROBOT_SCENARIO)
    s = parse_scenario(p)
    # fanuc1 has no namespace declared → defaults to /fanuc1
    assert s.robots[0].namespace == "/fanuc1"
    # fanuc2 has explicit namespace
    assert s.robots[1].namespace == "/arm2"


def test_multi_robot_spawn(tmp_path):
    p = write_scenario(tmp_path, MULTI_ROBOT_SCENARIO)
    s = parse_scenario(p)
    assert s.robots[0].spawn.x == 0.5
    assert s.robots[1].spawn.yaw == pytest.approx(3.14)


def test_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        parse_scenario(tmp_path / "nonexistent.yaml")


def test_not_a_mapping(tmp_path):
    p = write_scenario(tmp_path, "- list\n")
    with pytest.raises(ValueError):
        parse_scenario(p)


def test_missing_name(tmp_path):
    p = write_scenario(tmp_path, "world: some_world\n")
    with pytest.raises(ValueError, match="name"):
        parse_scenario(p)


def test_robot_missing_plugin(tmp_path):
    bad = "name: x\nrobots:\n  - instance: r1\n"
    p = write_scenario(tmp_path, bad)
    with pytest.raises(ValueError, match="plugin"):
        parse_scenario(p)


def test_robot_missing_instance(tmp_path):
    bad = "name: x\nrobots:\n  - plugin: some_robot\n"
    p = write_scenario(tmp_path, bad)
    with pytest.raises(ValueError, match="instance"):
        parse_scenario(p)


# ---------------------------------------------------------------------------
# validate_scenario
# ---------------------------------------------------------------------------

def test_valid_scenario(tmp_path):
    env = make_plugin(tmp_path, ENV_PLUGIN, "env")
    robot = make_plugin(tmp_path, ROBOT_PLUGIN, "robot")
    plugins = [env, robot]

    p = write_scenario(tmp_path, VALID_SCENARIO)
    s = parse_scenario(p)
    errors = validate_scenario(s, plugins)
    assert errors == []


def test_world_plugin_not_found(tmp_path):
    robot = make_plugin(tmp_path, ROBOT_NO_DEP, "robot")
    p = write_scenario(tmp_path, VALID_SCENARIO)
    s = parse_scenario(p)
    errors = validate_scenario(s, [robot])
    assert any("example_factory_world" in e for e in errors)


def test_world_wrong_type(tmp_path):
    wrong = ENV_PLUGIN.replace("plugin_type: environment", "plugin_type: robot")
    env = make_plugin(tmp_path, wrong, "env")
    p = write_scenario(tmp_path, VALID_SCENARIO)
    s = parse_scenario(p)
    errors = validate_scenario(s, [env])
    assert any("environment" in e for e in errors)


def test_robot_plugin_not_found(tmp_path):
    env = make_plugin(tmp_path, ENV_PLUGIN, "env")
    p = write_scenario(tmp_path, VALID_SCENARIO)
    s = parse_scenario(p)
    errors = validate_scenario(s, [env])
    assert any("example_melon_ros2" in e for e in errors)


def test_duplicate_instance_name(tmp_path):
    env = make_plugin(tmp_path, ENV_PLUGIN, "env")
    robot = make_plugin(tmp_path, ROBOT_NO_DEP, "robot")
    dup = """\
name: dup_test
world: example_factory_world
robots:
  - plugin: fanuc_crx10ia
    instance: arm
  - plugin: fanuc_crx10ia
    instance: arm
"""
    p = write_scenario(tmp_path, dup)
    s = parse_scenario(p)
    errors = validate_scenario(s, [env, robot])
    assert any("Duplicate" in e for e in errors)


def test_dep_plugin_not_in_scenario(tmp_path):
    env = make_plugin(tmp_path, ENV_PLUGIN, "env")
    robot = make_plugin(tmp_path, ROBOT_PLUGIN, "robot")  # depends on example_factory_world

    # Scenario with robot but no world
    no_world = """\
name: no_world
robots:
  - plugin: example_melon_ros2
    instance: melon
"""
    p = write_scenario(tmp_path, no_world)
    s = parse_scenario(p)
    errors = validate_scenario(s, [env, robot])
    assert any("example_factory_world" in e for e in errors)


def test_empty_scenario(tmp_path):
    p = write_scenario(tmp_path, "name: empty\n")
    s = parse_scenario(p)
    assert validate_scenario(s, []) == []
