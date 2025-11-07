
# at\_factory Project - Isaac Lab 2.3.0 and ROS 2 Jazzy Integration

This repository contains configuration files and scripts for setting up NVIDIA Isaac Lab 2.3.0 and ROS 2 Jazzy integration within a Docker container for the at\_factory project.

## Environment Overview

### Included Components

  * **NVIDIA Isaac Lab 2.3.0**: Simulation environment (based on Isaac Sim 2023.1.1)
  * **ROS 2 Jazzy**: Robot development framework
  * **Base OS**: Ubuntu 24.04

## Prerequisites

### Hardware Requirements

  * **GPU**: NVIDIA GPU (RTX 3060 or higher recommended)
  * **Memory**: Minimum 16GB RAM (32GB recommended)
  * **Storage**: Minimum 50GB free space
  * **Display**: X11-compatible display

### Software Requirements

  * **OS**: Ubuntu 24.04 LTS
  * **Docker**: Latest version
  * **NVIDIA Container Toolkit**: Installed
  * **NVIDIA Driver**: 535.x or higher (550.x recommended)

### NVIDIA Container Toolkit Installation

[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Setup Instructions

### 1\. Clone the Repository

```bash
git clone https://github.com/momoiorg-repository/at_factory.git
cd at_factory
```

### 2\. Run Initialization Script

```bash
# Execute automated setup script
./init.sh
```

This script automatically performs the following:

  * Creates Isaac Sim persistent storage directories
  * Builds Docker image (approximately 10-20 minutes)
  * Sets script execution permissions
  * Displays environment variable setup guide

### 3\. Configure Environment Variables

```bash
# Example: export DISPLAY=192.168.100.21:0
export DISPLAY=<your-local-pc-ip-address>:0
```

## Usage

### Container Management

#### 1\. Start Container (Background Execution)

Start container in background (continues even after closing editor)

```bash
./run_isaac_sim_docker.sh
```

#### 2\. Connect to Container
Connect to running container

```bash
./connect_to_container.sh

# The container will not stop when you type 'exit' inside the container
```

#### 3\. Container Management

```bash
# Restart container (does not delete)
./restart_container.sh

# Stop and fully initialize container (delete)
./stop_container.sh

# Check container status
docker ps
```

### Launching Isaac Sim

#### GUI Mode (Local Display)

```bash
# Execute inside container
cd /isaac-sim
./isaac-sim.sh
```

#### Headless Mode (Remote Connection)

```bash
# Start headless server inside container
cd /isaac-sim
./isaac-sim.sh --headless

# Connect client in another terminal
# Use Omniverse Streaming Client
```

## Project Structure

```
at_factory/
├── README.md                    # This file
├── Dockerfile                   # Isaac Lab + ROS 2 Jazzy image definition
├── init.sh                      # Automated initialization script
├── run_isaac_sim_docker.sh     # Container startup script (background)
├── connect_to_container.sh      # Container connection script
├── stop_container.sh           # Container stop/delete script
├── restart_container.sh        # Container restart script
├── LICENSE                      # MIT License
├── .gitignore                   # Git exclusion settings
└── isaac-sim/                   # Isaac Sim persistent data
    ├── cache/                   # Cache files
    ├── logs/                    # Log files
    ├── data/                    # Simulation data
    ├── documents/               # Documents
    └── config/                  # Configuration files
```

## License

This project is provided under the MIT License. See the [LICENSE](https://www.google.com/search?q=LICENSE) file for details.