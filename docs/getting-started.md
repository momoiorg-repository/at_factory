---
title: Getting Started
nav_order: 2
---

# Getting Started
{: .no_toc }

<details open markdown="block">
  <summary>Table of contents</summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

---

## Requirements

### Hardware

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | NVIDIA RTX 3060 | RTX 3080 or higher |
| RAM | 16 GB | 32 GB |
| Storage | 50 GB free | 100 GB free |

### Software

| Requirement | Notes |
|-------------|-------|
| Ubuntu 24.04 LTS | Other distros untested |
| Docker (latest) | |
| [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) | Required for GPU pass-through |
| NVIDIA driver 535.x or higher | |
| Python 3.10+ | For the host-side CLI only |

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/momoiorg-repository/plugsim.git
cd plugsim
```

### 2. Install the CLI

Using [uv](https://docs.astral.sh/uv/) (recommended):

```bash
uv venv
source .venv/bin/activate
uv pip install -e .
```

Or with plain pip:

```bash
pip install -e .
```

This installs the `plugsim` command. Runtime dependency: `pyyaml` only.

### 3. Build the Docker images

```bash
plugsim setup
```

This builds two images and creates Isaac Sim cache directories:

| Image | Based on | Purpose |
|-------|----------|---------|
| `plugsim:isaac` | Isaac Lab 2.3.0 (Ubuntu 24.04) | Runs Isaac Sim + ROS 2 bridge |
| `plugsim:ros2` | Ubuntu 24.04 | Runs robot launch files, builds colcon workspaces |

{: .note }
> First build takes 10–20 minutes. You will be prompted before rebuilding if an image already exists.

### 4. Clone plugin assets

Plugin `assets/` directories contain large USD files stored in separate repositories:

```bash
git clone https://github.com/momoiorg-repository/factory_world1.git \
    plugin/example_factory_world/assets

git clone https://github.com/momoiorg-repository/melon_ros2.git \
    plugin/example_melon_ros2
```

### 5. Set your display

PlugSim uses X11 forwarding for Isaac Sim GUI output:

```bash
export DISPLAY=<your-local-ip>:0
xhost +local:docker
```

---

## First Run

### Validate your setup

```bash
plugsim scan                                              # list discovered plugins
plugsim validate --scenario scenarios/factory_melon.yaml  # check before launching
```

### Start a scenario

```bash
plugsim up --scenario scenarios/factory_melon.yaml
```

This starts both containers:
- `plugsim-isaac` — Isaac Sim container (GPU)
- `plugsim-ros2` — ROS 2 control container (CPU)

After startup, PlugSim prints the commands to run in each container.

### Connect to the Isaac container

```bash
plugsim shell
```

From inside:

```bash
cd /plugin/example_factory_world && python app.py
```

### Connect to the ROS 2 container

```bash
plugsim shell ros2
```

From inside:

```bash
ros2 launch /plugin/example_melon_ros2/melon_ws/src/melon_bringup/launch/melon_bringup.launch.py
```

### Stop everything

```bash
plugsim down
```

Isaac Sim cache data in `./isaac-sim/` is preserved and reused on the next `plugsim up`.

---

## Adding Your First Plugin

```bash
plugsim init
```

Follow the prompts to scaffold a new `environment`, `robot`, or `asset` plugin. Then edit the generated `METADATA.yaml` and drop in your USD or launch files.

See [Plugin System]({% link plugin-system/index.md %}) for the full METADATA.yaml reference.
