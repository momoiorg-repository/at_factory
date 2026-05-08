---
title: Home
layout: home
nav_order: 1
permalink: /
description: "PlugSim — plugin-based Isaac Sim + ROS 2 environment manager"
---

# PlugSim

**The fastest way to get a custom robot running in Isaac Sim with a full ROS 2 interface.**

Drop a robot or environment into `plugin/`, describe it with a `METADATA.yaml`, pick a scenario, and PlugSim handles the containers — GPU pass-through, cache directories, ROS 2 DDS setup, all of it.

---

## What it does

{: .highlight }
> PlugSim's job stops at the ROS 2 topic boundary. It sets up the simulation and exposes the interface. Your control stack — MoveIt, Nav2, VLA models, anything — connects via standard ROS 2 topics.

| Concern | How PlugSim handles it |
|---------|------------------------|
| Environment setup | `environment` plugins define USD scenes and physics |
| Robot setup | `robot` plugins define USD models, ROS 2 launch files, and interface contracts |
| Multi-robot composition | Scenario files compose world + robots with namespaces and spawn poses |
| Container lifecycle | Two containers (`plugsim-isaac` GPU + `plugsim-ros2` CPU) wired correctly |
| Plugin discovery | Scans `plugin/*/METADATA.yaml` automatically |
| Compatibility checking | Validates scenarios before launch |

---

## Quick look

```bash
plugsim setup
plugsim up --scenario scenarios/factory_melon.yaml
plugsim shell          # Isaac container
plugsim shell ros2     # ROS 2 container
```

---

## Two-container architecture

```
plugsim-isaac (GPU)          plugsim-ros2 (CPU)
  Isaac Lab 2.3          ◄──►   ROS 2 Jazzy + colcon
  World + Robot USD             Robot launch files
  ROS 2 bridge                  MoveIt / Nav2 / drivers
        ▲                              ▲
        └──────── ROS_DOMAIN_ID=31 ───┘
                  External controllers connect here
```

---

## Navigation

- [Getting Started]({% link getting-started.md %}) — requirements, installation, first run
- [CLI Reference]({% link cli-reference.md %}) — all `plugsim` commands
- [Plugin System]({% link plugin-system/index.md %}) — METADATA.yaml schema, adding plugins
- [Scenarios]({% link scenarios.md %}) — composing worlds and robots
- [Containers]({% link container.md %}) — container details, volumes, troubleshooting
