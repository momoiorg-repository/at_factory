# @Factory Project - Isaac Sim 5.0.0 and ROS 2 Humble Integration Environment

This repository contains configuration files and scripts for setting up NVIDIA Isaac Sim 5.0.0 and ROS 2 Humble integration within a Docker container for the @Factory project.

## Environment Overview

### Included Components
- **NVIDIA Isaac Sim 5.0.0**: Latest physics simulation environment
- **ROS 2 Humble**: Robot development framework


## Prerequisites

### Hardware Requirements
- **GPU**: NVIDIA GPU (RTX 3060 or higher recommended)
- **Memory**: Minimum 16GB RAM (32GB recommended)
- **Storage**: Minimum 50GB free space
- **Display**: X11-compatible display

### Software Requirements
- **OS**: Ubuntu 22.04 LTS
- **Docker**: Latest version
- **NVIDIA Container Toolkit**: Installed
- **NVIDIA Driver**: Latest version (470 or higher)

### NVIDIA Container Toolkit Installation

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

## Setup Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/momoiorg-repository/at_factory.git
cd at_factory
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
```

### 2. Run Initialization Script
```bash
# Execute automated setup script
./init.sh
```

This script automatically performs the following:
- Creates Isaac Sim persistent storage directories
- Builds Docker image (approximately 30-60 minutes)
- Sets script execution permissions
- Displays environment variable setup guide

### 3. Configure Environment Variables
```bash
# Example: export DISPLAY=192.168.100.21:0
export DISPLAY=<your-local-pc-ip-address>:0
```

## Usage

### Plugin and Robot Installation

#### finstall.sh Script
A script for automatically installing plugins, robots, and worlds from GitHub repositories.

```bash
# Usage
./finstall.sh <github_repo_url> [robot|world]

# Examples:
# Install robot plugin (default behavior)
./finstall.sh https://github.com/momoiorg-repository/melon_ros2.git
./finstall.sh https://github.com/momoiorg-repository/melon_ros2.git robot

# Install world plugin
./finstall.sh https://github.com/momoiorg-repository/factory_world1.git world
```

**Features:**
1. Automatic creation of `plugin/robot/` or `plugin/world/` directory based on choice
2. Clones the specified GitHub repository
3. Automatically executes the `install.sh` script within the repository
4. Supports both robot and world plugin installations

**Installation Types:**
- **robot** (default): Installs in `plugin/robot/` directory, mounted to `/robot` in container
- **world**: Installs in `plugin/world/` directory, mounted to `/world` in container

**Notes:**
- If no second parameter is provided, defaults to `robot` installation
- Existing directories are automatically removed before installation
- A warning message is displayed if `install.sh` is not found
- Handles permission issues gracefully with fallback to temporary installation

### Container Management

#### 1. Start Container (Background Execution)
```bash
# Start container in background (continues even after closing editor)
./run_isaac_sim_docker.sh
```

#### 2. Connect to Container
```bash
# Connect to running container
./connect_to_container.sh

# The container will not stop when you type 'exit' inside the container
```

#### 3. Container Management
```bash
# Restart container (does not delete)
./restart_container.sh

# Stop and fully initialize container (delete)
./stop_container.sh

# Check container status
docker ps
```

#### 4. Manual Docker Command Execution (Traditional Method)
```bash
# Allow access to X11 server
xhost +

# Start container (stops when editor is closed)
docker run --name isaac-sim-ws -it --rm \
  --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v ./isaac-sim:/root/isaac-sim:rw \
  -v ./factory_v1:/IsaacLab/scripts/factory_v1:rw \
  isaac_factory:5.0.0
```

### Launching Isaac Sim

#### GUI Mode (Local Display)
```bash
# Execute inside container
runapp
```

#### Headless Mode (Remote Connection)
```bash
# Start headless server
runheadless

# Connect client in another terminal
# Use Omniverse Streaming Client
```

## Project Structure

```
isaac_factory/
├── README.md                    # This file
├── Dockerfile                   # Isaac Sim + ROS 2 image definition
├── init.sh                      # Automated initialization script
├── finstall.sh                  # Plugin/robot automated installation script
├── run_isaac_sim_docker.sh     # Container startup script (background)
├── connect_to_container.sh      # Container connection script
├── stop_container.sh           # Container stop/delete script
├── restart_container.sh        # Container restart script
├── LICENSE                      # MIT License
├── .gitignore                   # Git exclusion settings
├── isaac-sim/                   # Isaac Sim persistent data
│   ├── cache/                   # Cache files
│   ├── logs/                    # Log files
│   ├── data/                    # Simulation data
│   ├── documents/               # Documents
│   └── config/                  # Configuration files
├── plugin/                      # Plugin directory
│   ├── robot/                   # Robot plugins (mounted to /robot in container)
│   └── world/                   # World plugins (mounted to /world in container)
```

## License

This project is provided under the MIT License. See the [LICENSE](LICENSE) file for details.
