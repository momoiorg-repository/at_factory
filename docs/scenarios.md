---
title: Scenarios
nav_order: 5
---

# Scenarios
{: .no_toc }

<details open markdown="block">
  <summary>Table of contents</summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

---

## What is a scenario?

A scenario is a YAML file that composes one world and one or more robots into a runnable simulation. It answers:

- **Which world** to load in Isaac Sim
- **Which robots** to spawn, and where
- **What ROS 2 namespace** each robot uses
- **Which robot plugin** handles each instance

Scenarios live in `scenarios/` and are passed to `plugsim up` and `plugsim validate`.

---

## Schema

```yaml
name: string                  # required — unique name for this scenario
description: string           # optional

world: string                 # environment plugin name (omit for robot-only scenarios)

robots:
  - plugin: string            # required — robot plugin name
    instance: string          # required — unique name for this robot instance
    namespace: string         # optional — ROS 2 namespace (default: /<instance>)
    spawn:
      x: float                # default 0.0
      y: float                # default 0.0
      z: float                # default 0.0
      roll: float             # default 0.0
      pitch: float            # default 0.0
      yaw: float              # default 0.0
```

---

## Examples

### Single robot

```yaml
# scenarios/factory_melon.yaml
name: factory_melon
description: "Factory environment with Melon mobile manipulator"

world: example_factory_world

robots:
  - plugin: example_melon_ros2
    instance: melon
    namespace: /melon
    spawn:
      x: 0.0
      y: 0.0
      z: 0.0
```

### Multiple robots, same plugin

```yaml
# scenarios/factory_fanuc.yaml
name: factory_fanuc
description: "Two FANUC CRX-10iA/L arms in factory"

world: example_factory_world

robots:
  - plugin: fanuc_crx10ia
    instance: fanuc1
    namespace: /fanuc1
    spawn:
      x: 0.5
      y: 0.0
      z: 0.85

  - plugin: fanuc_crx10ia
    instance: fanuc2
    namespace: /fanuc2
    spawn:
      x: -0.5
      y: 0.0
      z: 0.85
      yaw: 3.14159
```

### Multiple robots, different plugins

```yaml
name: factory_mixed
world: example_factory_world

robots:
  - plugin: fanuc_crx10ia
    instance: arm
    namespace: /arm
    spawn: { x: 0.5, z: 0.85 }

  - plugin: example_melon_ros2
    instance: mobile
    namespace: /mobile
    spawn: { x: 2.0, y: 1.0 }
```

### Robot-only (no Isaac world)

Useful when testing a pure ROS 2 driver against real hardware:

```yaml
name: fanuc_driver_test
robots:
  - plugin: fanuc_crx10ia
    instance: fanuc
    namespace: /fanuc
```

---

## Namespace behaviour

If `namespace` is omitted, PlugSim defaults to `/<instance>`:

```yaml
robots:
  - plugin: fanuc_crx10ia
    instance: arm1        # → namespace: /arm1 (automatic)
  - plugin: fanuc_crx10ia
    instance: arm2
    namespace: /right_arm # → namespace: /right_arm (explicit)
```

All robot topics declared in `ros2_interface` are relative to this namespace. An external controller connects to `/arm1/joint_states`, `/right_arm/joint_states`, etc.

---

## Running a scenario

```bash
# Validate before launch
plugsim validate --scenario scenarios/factory_fanuc.yaml

# Start containers
plugsim up --scenario scenarios/factory_fanuc.yaml

# Connect to Isaac container and start the world
plugsim shell
cd /plugin/example_factory_world && python app.py

# Connect to ROS 2 container and start the robot drivers
plugsim shell ros2
ros2 launch /plugin/fanuc_driver/fanuc_hardware_interface/launch/fanuc_mock_control.launch.py \
    robot_model:=crx10ia_l robot_series:=crx launch_rviz:=false

# Stop everything
plugsim down
```

---

## Connecting external controllers

Because all containers share `--network host` and `ROS_DOMAIN_ID=31`, any ROS 2 node on the host sees the topics immediately:

```bash
# From your control workspace (v_dash_ws, MoveIt, custom VLA, etc.)
export ROS_DOMAIN_ID=31
ros2 topic list
ros2 topic echo /fanuc1/joint_states
```

No PlugSim-specific integration is required. The scenario's `ros2_interface` declarations document exactly which topics to expect.

---

## Validation rules

`plugsim validate --scenario <file>` checks:

| Rule | What is checked |
|------|----------------|
| Plugin existence | Every `world` and `plugin` name must match a discovered plugin |
| Plugin type | `world` must be `environment`, robots must be `robot` |
| No duplicate instances | Every `instance` name must be unique within the scenario |
| Dependencies satisfied | `dep_plugins` of each plugin must be present in the scenario |
| ROS distro | Any declared `ros_distro` must be `jazzy` |
