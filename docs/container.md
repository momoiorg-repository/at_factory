---
title: Containers
nav_order: 6
---

# Containers
{: .no_toc }

<details open markdown="block">
  <summary>Table of contents</summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

---

## Two-container architecture

PlugSim runs two containers that communicate over ROS 2 DDS on the host network:

```
┌──────────────────────────┐   FastDDS (network host)   ┌──────────────────────────┐
│   plugsim-isaac  (GPU)   │ ◄────────────────────────► │   plugsim-ros2  (CPU)    │
│                          │                            │                          │
│  Isaac Lab 2.3.0         │                            │  ROS 2 Jazzy             │
│  Isaac Sim + physics     │                            │  colcon + rosdep         │
│  World USD               │                            │  MoveIt / Nav2           │
│  Robot USD spawn         │                            │  Robot drivers           │
│  ROS 2 bridge nodes      │                            │  Robot launch files      │
└──────────────────────────┘                            └──────────────────────────┘
           ▲                                                        ▲
           └────────── ROS_DOMAIN_ID=31 ──── network host ─────────┘
                       External controllers connect here
```

Both containers and any external ROS 2 node on the host share the same DDS discovery domain. No special networking configuration is needed for external tools to see the topics.

---

## Container details

### Isaac container

| Property | Value |
|----------|-------|
| Name | `plugsim-isaac` |
| Image | `plugsim:isaac` |
| Dockerfile | `Dockerfile` |
| Base | `nvcr.io/nvidia/isaac-lab:2.3.0` (Ubuntu 24.04) |
| Runtime | `--runtime=nvidia --gpus all` |
| Plugin mount | `./plugin` → `/plugin` |
| ROS workspaces | `./IsaacSim-ros_workspaces` → `/IsaacSim-ros_workspaces` |

### ROS 2 container

| Property | Value |
|----------|-------|
| Name | `plugsim-ros2` |
| Image | `plugsim:ros2` |
| Dockerfile | `Dockerfile.ros2` |
| Base | `ubuntu:24.04` |
| Runtime | standard (no GPU) |
| Plugin mount | `./plugin` → `/plugin` |

Both containers share:
- `--network host`
- `--ipc=host`
- `ROS_DOMAIN_ID=31`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

---

## Volume mounts

### Isaac container

| Host path | Container path | Purpose |
|-----------|----------------|---------|
| `./plugin` | `/plugin` | All plugins |
| `./IsaacSim-ros_workspaces` | `/IsaacSim-ros_workspaces` | ROS 2 workspaces |
| `./isaac-sim/cache/kit` | `/isaac-sim/kit/cache` | Kit cache |
| `./isaac-sim/cache/ov` | `/root/.cache/ov` | Omniverse cache |
| `./isaac-sim/cache/pip` | `/root/.cache/pip` | pip cache |
| `./isaac-sim/cache/glcache` | `/root/.cache/nvidia/GLCache` | OpenGL cache |
| `./isaac-sim/cache/computecache` | `/root/.nv/ComputeCache` | CUDA compute cache |
| `./isaac-sim/logs` | `/root/.nvidia-omniverse/logs` | Omniverse logs |
| `./isaac-sim/data` | `/root/.local/share/ov/data` | Omniverse data |
| `./isaac-sim/documents` | `/root/isaac-sim/Documents` | Isaac Sim documents |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 display socket |
| `~/.Xauthority` | `/root/.Xauthority` | X11 auth |

### ROS 2 container

| Host path | Container path | Purpose |
|-----------|----------------|---------|
| `./plugin` | `/plugin` | Plugin launch files and workspaces |

---

## Environment variables

### Isaac container

| Variable | Value | Purpose |
|----------|-------|---------|
| `ACCEPT_EULA` | `Y` | NVIDIA EULA |
| `PRIVACY_CONSENT` | `Y` | NVIDIA privacy |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` | FastDDS transport |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS 2 middleware |
| `ROS_DOMAIN_ID` | `31` | DDS domain isolation |
| `DISPLAY` | from host `$DISPLAY` | X11 display |
| `QT_X11_NO_MITSHM` | `1` | Qt X11 compatibility |
| `QT_QPA_PLATFORM` | `xcb` | Qt platform plugin |
| `PLUGSIM_SCENARIO` | scenario name | Set by `plugsim up` |
| `PLUGSIM_WORLD` | world plugin name | Set by `plugsim up` |
| `PLUGSIM_ROBOTS` | comma-separated instances | Set by `plugsim up` |

### ROS 2 container

| Variable | Value | Purpose |
|----------|-------|---------|
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS 2 middleware |
| `ROS_DOMAIN_ID` | `31` | DDS domain isolation |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` | FastDDS transport |
| `PLUGSIM_SCENARIO` | scenario name | Set by `plugsim up` |
| `PLUGSIM_WORLD` | world plugin name | Set by `plugsim up` |
| `PLUGSIM_ROBOTS` | comma-separated instances | Set by `plugsim up` |

---

## Building images manually

```bash
# Isaac image
docker build -t plugsim:isaac .

# ROS 2 image
docker build -t plugsim:ros2 -f Dockerfile.ros2 .
```

---

## Running Isaac Sim inside the container

```bash
plugsim shell    # connects to plugsim-isaac

# GUI mode
cd /isaac-sim && ./isaac-sim.sh

# Headless mode
cd /isaac-sim && ./runheadless.sh
```

---

## Building a colcon workspace inside the ROS 2 container

Robot plugins that declare `ros2_entry.workspace` need to be built before their launch files work:

```bash
plugsim shell ros2

cd /plugin/fanuc_driver
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch fanuc_hardware_interface/launch/fanuc_mock_control.launch.py \
    robot_model:=crx10ia_l robot_series:=crx launch_rviz:=false
```

---

## Troubleshooting

### Container exits immediately

```bash
nvidia-smi
docker run --rm --runtime=nvidia --gpus all nvidia/cuda:12.0-base nvidia-smi
```

### X11 display errors

```bash
export DISPLAY=<your-ip>:0
xhost +local:docker
plugsim up --scenario ...
```

### Permission errors in isaac-sim/ directories

Run `plugsim setup` again. Docker creates missing mount targets as root-owned if they don't exist beforehand — `plugsim setup` pre-creates them with correct ownership.

### ROS 2 topics not visible between containers

Both containers must use the same `ROS_DOMAIN_ID`. PlugSim sets `ROS_DOMAIN_ID=31` in both. Verify with:

```bash
# In each container
echo $ROS_DOMAIN_ID
ros2 topic list
```
