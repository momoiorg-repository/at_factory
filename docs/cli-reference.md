---
title: CLI Reference
nav_order: 3
---

# CLI Reference
{: .no_toc }

<details open markdown="block">
  <summary>Table of contents</summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

---

All commands are issued from the repository root after `pip install -e .`.

```
plugsim <command> [options]
```

---

## setup

Build both Docker images and initialise Isaac Sim storage directories.

```bash
plugsim setup
```

**What it does:**

1. Creates `isaac-sim/` subdirectories so Docker does not create them as root-owned
2. Builds (or offers to rebuild) `plugsim:isaac` from `Dockerfile`
3. Builds (or offers to rebuild) `plugsim:ros2` from `Dockerfile.ros2`

{: .note }
> Run once after cloning. Re-run after modifying either Dockerfile.

---

## scan

List all discovered plugins.

```bash
plugsim scan
```

Scans every direct child of `plugin/` that contains a `METADATA.yaml`. Prints name, type, version, ROS distro, Isaac Lab constraint, and entry points.

**Example output:**

```
Discovered 3 plugin(s):

  [environment]  example_factory_world               v1.0.0    ros:jazzy  isaac-lab:>=2.0.0
               isaac: app:app.py | usd:assets/factory_base.usd
  [robot      ]  example_melon_ros2                  v1.0.0    ros:jazzy  isaac-lab:>=2.0.0
               isaac: usd:../assets/melon/melon.usd
               ros2:  launch:melon_ws/src/melon_bringup/launch/melon_bringup.launch.py
  [robot      ]  fanuc_crx10ia                       v1.0.0    ros:jazzy  isaac-lab:—
               ros2:  launch:fanuc_hardware_interface/launch/fanuc_mock_control.launch.py | ws:.
```

---

## validate

Check plugin and scenario compatibility.

```bash
plugsim validate [--scenario FILE]
```

**Without `--scenario`:** runs compatibility checks on all discovered plugins.

**With `--scenario`:** validates that all plugins referenced in the scenario exist, have the correct type, and have their dependencies satisfied.

| Check | Description |
|-------|-------------|
| Plugin existence | Every plugin named in the scenario must exist in `plugin/` |
| Plugin type | World must be `environment`, robots must be `robot` |
| Duplicate instances | No two robots may share the same `instance` name |
| Dependency satisfaction | `dep_plugins` of each plugin must be present in the scenario |
| ROS distro | Any declared `ros_distro` must be `jazzy` |

Exits with code `1` if any check fails.

---

## up

Start both simulation containers for a scenario.

```bash
plugsim up --scenario <file>
plugsim up --world <plugin> [--robot <plugin> ...]
```

**`--scenario`** — path to a scenario file. Resolved relative to `scenarios/` first, then the current directory.

**`--world` / `--robot`** — quick shorthand for ad-hoc single-robot runs without writing a scenario file. `--robot` is repeatable.

Runs `plugsim validate` implicitly before starting. If validation fails, the containers are not started.

After startup, prints the commands to run inside each container.

**Examples:**

```bash
plugsim up --scenario scenarios/factory_melon.yaml
plugsim up --scenario /absolute/path/to/my_scenario.yaml
plugsim up --world example_factory_world --robot fanuc_crx10ia
```

---

## down

Stop and remove both containers.

```bash
plugsim down
```

Stops `plugsim-isaac` and `plugsim-ros2`. Isaac Sim cache data in `./isaac-sim/` is preserved.

---

## shell

Open an interactive bash shell inside a running container.

```bash
plugsim shell [isaac|ros2]
```

| Target | Container | Default |
|--------|-----------|---------|
| `isaac` | `plugsim-isaac` | ✓ |
| `ros2` | `plugsim-ros2` | |

Type `exit` to leave — the container keeps running.

---

## info

Show full details for a single plugin.

```bash
plugsim info <plugin-name>
```

Displays type, description, compatibility, Isaac entry, ROS 2 entry (including workspace and launch args), ROS 2 interface contract (topics, services, actions), and dependencies.

---

## init

Interactively scaffold a new plugin directory.

```bash
plugsim init
```

Prompts for plugin type (`environment`, `robot`, or `asset`), name, version, and description, then creates:

```
plugin/<name>/
├── METADATA.yaml    ← pre-filled for the chosen type
├── README.md
└── assets/
```

---

## Container and image names

| Item | Value |
|------|-------|
| Isaac container | `plugsim-isaac` |
| Isaac image | `plugsim:isaac` |
| ROS 2 container | `plugsim-ros2` |
| ROS 2 image | `plugsim:ros2` |
| Plugin mount | `/plugin` (in both containers) |
| ROS domain ID | `31` |
