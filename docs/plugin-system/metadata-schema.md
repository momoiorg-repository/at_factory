---
title: METADATA.yaml Schema
parent: Plugin System
nav_order: 1
---

# METADATA.yaml Schema
{: .no_toc }

<details open markdown="block">
  <summary>Table of contents</summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

---

## Full example (robot plugin)

```yaml
schema_version: "2.0"
plugin_type: robot
name: my_robot
version: 1.0.0
description: "My robot with ROS 2 control"

compatibility:
  isaac_lab: ">=2.0.0"
  ros_distro: jazzy

isaac_entry:
  usd: assets/my_robot.usd
  app: scripts/spawn.py

ros2_entry:
  workspace: .                        # colcon workspace to build (optional)
  launch: launch/my_robot.launch.py
  launch_args:
    use_sim_time: "true"
    robot_model: crx10ia_l
  config: config/params.yaml

ros2_interface:
  namespace: /robot
  publishes:
    - topic: joint_states
      type: sensor_msgs/JointState
    - topic: camera/rgb/image_raw
      type: sensor_msgs/Image
  subscribes:
    - topic: joint_commands
      type: trajectory_msgs/JointTrajectory
  action_servers:
    - name: follow_joint_trajectory
      type: control_msgs/FollowJointTrajectory

dep_plugins:
  - my_world

author: "Your Name"
license: MIT
repository: "https://github.com/your-org/my_robot"
```

---

## Top-level fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `schema_version` | string | No | Schema version — `"2.0"` |
| `plugin_type` | string | **Yes** | `environment`, `robot`, or `asset` |
| `name` | string | **Yes** | Unique plugin identifier |
| `version` | string | No | Semantic version (default `"1.0.0"`) |
| `description` | string | No | Short human-readable description |
| `author` | string | No | Author name |
| `license` | string | No | License identifier (e.g. `MIT`) |
| `repository` | string | No | Source repository URL |

---

## Plugin types

| Type | Isaac container | ROS 2 container | Typical use |
|------|----------------|----------------|-------------|
| `environment` | USD scene + app | — | Factory floor, warehouse |
| `robot` | USD model + spawn script | launch files, colcon workspace | Arms, mobile bases |
| `asset` | Passive USD only | — | Boxes, pallets, people |

---

## `compatibility`

| Field | Type | Description |
|-------|------|-------------|
| `isaac_lab` | string | Version constraint, e.g. `">=2.0.0"` |
| `ros_distro` | string | Must be `jazzy` if declared |

{: .warning }
> `plugsim validate` rejects any plugin declaring a `ros_distro` other than `jazzy`.

---

## `isaac_entry`

What runs in the **Isaac container**. All fields are optional.

| Field | Type | Description |
|-------|------|-------------|
| `usd` | string | USD scene or model file, relative to `METADATA.yaml` |
| `app` | string | Isaac Sim standalone Python script, relative to `METADATA.yaml` |

```yaml
isaac_entry:
  usd: assets/scene.usd
  app: scripts/spawn.py
```

For `asset` plugins, only `usd` is needed. For `environment` plugins, `app` handles physics setup and lighting. For `robot` plugins, `app` spawns the robot into the running scene.

---

## `ros2_entry`

What runs in the **ROS 2 container**. All fields are optional.

| Field | Type | Description |
|-------|------|-------------|
| `workspace` | string | Path to a colcon workspace to build before launch, relative to `METADATA.yaml` |
| `launch` | string | ROS 2 `.launch.py` file, relative to `METADATA.yaml` |
| `launch_args` | mapping | Key-value arguments passed to the launch file |
| `config` | string | Parameter file, relative to `METADATA.yaml` |

```yaml
ros2_entry:
  workspace: .
  launch: fanuc_hardware_interface/launch/fanuc_mock_control.launch.py
  launch_args:
    robot_model: crx10ia_l
    robot_series: crx
    launch_rviz: "false"
  config: fanuc_hardware_interface/config/ros2_controllers.yaml
```

{: .note }
> `workspace: .` means the plugin directory itself is a colcon workspace. PlugSim uses this to know that `colcon build` must be run inside the ROS 2 container before the launch file will work.

---

## `ros2_interface`

Declares the ROS 2 topics, services, and actions this plugin exposes. Used by `plugsim info` and future tooling to auto-generate connection configs for external controllers.

| Field | Type | Description |
|-------|------|-------------|
| `namespace` | string | Default ROS 2 namespace for this robot |
| `publishes` | list | Topics this plugin publishes |
| `subscribes` | list | Topics this plugin subscribes to |
| `action_servers` | list | Action servers this plugin provides |

Each `publishes` / `subscribes` entry:

| Sub-field | Description |
|-----------|-------------|
| `topic` | Topic name (relative to namespace) |
| `type` | ROS 2 message type, e.g. `sensor_msgs/JointState` |

Each `action_servers` entry:

| Sub-field | Description |
|-----------|-------------|
| `name` | Action name (relative to namespace) |
| `type` | ROS 2 action type, e.g. `control_msgs/FollowJointTrajectory` |

{: .note }
> The `namespace` declared here is the default. Scenarios can override it per-instance using the `namespace` field in the robot entry.

---

## `dep_plugins`

List of other plugin **names** this plugin depends on.

```yaml
dep_plugins:
  - example_factory_world
```

`plugsim validate --scenario` checks that every listed name is present in the scenario.

---

## Backward compatibility (v1.0 → v2.0)

PlugSim's parser accepts v1.0 files transparently:

| v1.0 field | v2.0 equivalent |
|------------|----------------|
| `plugin_type: world` | `plugin_type: environment` |
| `plugin_type: logic` / `app` | `plugin_type: asset` |
| `entry_point.usd` / `.app` | `isaac_entry.usd` / `.app` |
| `entry_point.launch` | `ros2_entry.launch` |
| `dependencies.plugins` | `dep_plugins` |
| `compatibility.isaac_sim` | (ignored — use `isaac_lab`) |

---

## Minimal examples

**Environment (world-only):**

```yaml
plugin_type: environment
name: my_world
isaac_entry:
  usd: assets/scene.usd
```

**Robot (ROS 2 driver, no Isaac Sim USD yet):**

```yaml
plugin_type: robot
name: fanuc_crx10ia
compatibility:
  ros_distro: jazzy
ros2_entry:
  workspace: .
  launch: fanuc_hardware_interface/launch/fanuc_mock_control.launch.py
```

**Asset (passive object):**

```yaml
plugin_type: asset
name: pallet_stack
isaac_entry:
  usd: assets/pallet.usd
```
