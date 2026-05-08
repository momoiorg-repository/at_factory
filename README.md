# PlugSim

**The fastest way to get a custom robot running in Isaac Sim with a full ROS 2 interface.**

Drop a robot or environment into `plugin/`, write a `METADATA.yaml`, pick a scenario, and PlugSim handles the containers — all the Docker flags, cache directories, GPU pass-through, and ROS 2 DDS setup that would otherwise take days to figure out.

---

## Requirements

### Hardware

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | NVIDIA RTX 3060 | RTX 3080 or higher |
| RAM | 16 GB | 32 GB |
| Storage | 50 GB free | 100 GB free |

### Software

- Ubuntu 24.04 LTS
- Docker (latest)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- NVIDIA driver 535.x or higher
- Python 3.10+

---

## Quick Start

### 1. Clone and install

Using [uv](https://docs.astral.sh/uv/) (recommended):

```bash
git clone https://github.com/momoiorg-repository/plugsim.git
cd plugsim
uv venv
source .venv/bin/activate
uv pip install -e .
```

Or with plain pip:

```bash
git clone https://github.com/momoiorg-repository/plugsim.git
cd plugsim
pip install -e .
```

### 2. Build the images

```bash
plugsim setup
```

Builds two Docker images and creates Isaac Sim cache directories:

- **`plugsim:isaac`** — Isaac Lab 2.3.0 + ROS 2 Jazzy (GPU container)
- **`plugsim:ros2`** — ROS 2 Jazzy + colcon + MoveIt (control container)

First build takes 10–20 minutes.

### 3. Clone plugin assets

```bash
git clone https://github.com/momoiorg-repository/factory_world1.git \
    plugin/example_factory_world/assets

git clone https://github.com/momoiorg-repository/melon_ros2.git \
    plugin/example_melon_ros2
```

### 4. Set your display

```bash
export DISPLAY=<your-local-ip>:0
xhost +local:docker
```

### 5. Start a scenario

```bash
plugsim up --scenario scenarios/factory_melon.yaml
```

### 6. Connect to each container

```bash
plugsim shell          # Isaac Sim container → run world + robot spawn
plugsim shell ros2     # ROS 2 container → run robot launch files
```

---

## How It Works

PlugSim runs two containers that communicate over ROS 2 DDS (`--network host`):

```
┌────────────────────────────┐   FastDDS / ROS 2 topics   ┌────────────────────────────┐
│  plugsim-isaac  (GPU)      │ ◄─────────────────────────► │  plugsim-ros2  (CPU)       │
│                            │                             │                            │
│  Isaac Lab 2.3             │                             │  ROS 2 Jazzy               │
│  World USD + physics       │                             │  colcon + rosdep           │
│  Robot USD + bridge        │                             │  MoveIt / Nav2 / drivers   │
└────────────────────────────┘                             └────────────────────────────┘
                                         ▲
                              same ROS_DOMAIN_ID=31
                              → external controllers connect here
                              (v_dash_ws, MoveIt, custom VLA)
```

External control stacks connect to the same ROS 2 network without any PlugSim-specific integration — just ROS 2 topics, services, and actions.

---

## CLI Reference

| Command | Description |
|---------|-------------|
| `plugsim setup` | Build both Docker images, create Isaac Sim cache dirs |
| `plugsim up --scenario <file>` | Start both containers for a scenario |
| `plugsim up --world <name> --robot <name>` | Quick single-robot shorthand |
| `plugsim down` | Stop and remove both containers |
| `plugsim shell` | Shell into the Isaac container |
| `plugsim shell ros2` | Shell into the ROS 2 container |
| `plugsim scan` | List all discovered plugins |
| `plugsim validate --scenario <file>` | Check scenario + plugin compatibility |
| `plugsim info <name>` | Show plugin details and ROS 2 interface |
| `plugsim init` | Scaffold a new plugin interactively |

---

## Plugin System

A plugin is any folder under `plugin/` that contains a `METADATA.yaml`.

### Plugin types

| Type | Purpose | Runs where |
|------|---------|------------|
| `environment` | Isaac Sim scene — USD + physics setup | Isaac container |
| `robot` | Robot model + ROS 2 control driver | Both containers |
| `asset` | Passive USD objects (furniture, objects) | Isaac container |

### METADATA.yaml (v2.0)

```yaml
schema_version: "2.0"
plugin_type: robot
name: my_robot
version: 1.0.0
description: "My robot with ROS 2 control"

compatibility:
  isaac_lab: ">=2.0.0"
  ros_distro: jazzy

# What runs in the Isaac container
isaac_entry:
  usd: assets/my_robot.usd
  app: scripts/spawn.py

# What runs in the ROS 2 container
ros2_entry:
  workspace: .                        # colcon workspace to build (optional)
  launch: launch/my_robot.launch.py
  launch_args:
    use_sim_time: "true"

# ROS 2 interface contract — topics this robot exposes
ros2_interface:
  namespace: /robot
  publishes:
    - topic: joint_states
      type: sensor_msgs/JointState
  subscribes:
    - topic: joint_commands
      type: trajectory_msgs/JointTrajectory
  action_servers:
    - name: follow_joint_trajectory
      type: control_msgs/FollowJointTrajectory

dep_plugins: []
author: ""
license: MIT
repository: ""
```

### Directory layout

```
plugin/
├── example_factory_world/    # environment plugin
│   ├── METADATA.yaml
│   ├── app.py
│   └── assets/               # USD files (separate git repo)
├── example_melon_ros2/       # robot plugin
│   ├── METADATA.yaml
│   ├── melon_ws/             # ROS 2 workspace
│   └── assets/
└── fanuc_driver/             # robot plugin — FANUC CRX ros2_control driver
    ├── METADATA.yaml
    ├── fanuc_hardware_interface/
    ├── fanuc_moveit_config/
    └── ...
```

---

## Scenarios

A scenario composes one world + one or more robots with spawn poses and ROS 2 namespaces.

```yaml
# scenarios/factory_dual_fanuc.yaml
name: factory_dual_fanuc
description: "Two FANUC CRX arms in factory"

world: example_factory_world

robots:
  - plugin: fanuc_crx10ia
    instance: fanuc1
    namespace: /fanuc1
    spawn: { x: 0.5, y: 0.0, z: 0.85 }

  - plugin: fanuc_crx10ia
    instance: fanuc2
    namespace: /fanuc2
    spawn: { x: -0.5, y: 0.0, z: 0.85, yaw: 3.14159 }
```

```bash
plugsim validate --scenario scenarios/factory_dual_fanuc.yaml
plugsim up --scenario scenarios/factory_dual_fanuc.yaml
```

---

## Connecting External Controllers

Because both containers use `--network host` with `ROS_DOMAIN_ID=31`, any ROS 2 node on the host connects automatically:

```bash
# From v_dash_ws or any other control container
ROS_DOMAIN_ID=31 ros2 topic list
ROS_DOMAIN_ID=31 ros2 topic echo /fanuc1/joint_states
```

---

## Repository Structure

```
plugsim/
├── plugsim/                 # Python package (pip install -e .)
│   ├── schema.py            # Plugin + scenario dataclasses
│   ├── scanner.py           # Plugin discovery
│   ├── parser.py            # METADATA.yaml parser (v1.0 + v2.0)
│   ├── scenario.py          # Scenario file parser + validator
│   ├── launcher.py          # Two-container lifecycle
│   └── cli.py               # CLI entry point
├── plugin/                  # Drop plugins here
│   ├── example_factory_world/
│   ├── example_melon_ros2/
│   └── fanuc_driver/
├── scenarios/               # Scenario composition files
│   ├── factory_melon.yaml
│   └── factory_fanuc.yaml
├── tests/                   # Unit tests (pytest)
├── Dockerfile               # plugsim:isaac image (Isaac Lab 2.3 + ROS 2 Jazzy)
├── Dockerfile.ros2          # plugsim:ros2 image (ROS 2 Jazzy + colcon)
└── pyproject.toml
```

---

## Running Tests

```bash
pytest tests/ -v
```

---

## License

MIT — see [LICENSE](LICENSE) for details.
