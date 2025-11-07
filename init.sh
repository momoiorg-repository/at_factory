#!/bin/bash

# Isaac@Factory - Init Script (Simple)
# Isaac Lab 2.3.0 + ROS 2 Jazzy

set -e # Exit immediately if a command fails

echo "=========================================="
echo "Isaac@Factory - Init Script (Jazzy)"
echo "=========================================="

IMAGE_TAG="at_factory:jazzy"

# 1. Create persistent storage directories
create_directories() {
    echo "[INFO] Checking persistent storage..."
    if [ -d "./isaac-sim" ] && [ "$(ls -A ./isaac-sim 2>/dev/null)" ]; then
        echo "[INFO] ./isaac-sim directory already exists. Skipping."
    else
        echo "[INFO] Creating ./isaac-sim directories..."
        mkdir -p ./isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data,documents,config}
        chmod -R 755 ./isaac-sim 2>/dev/null || true
        echo "[INFO] Storage directories created."
    fi
}

# 2. Check for Dockerfile
check_required_files() {
    echo "[INFO] Checking for Dockerfile..."
    if [ ! -f "./Dockerfile" ]; then
        echo "[ERROR] Dockerfile not found!"
        echo "[ERROR] Please add the Dockerfile to this directory."
        exit 1
    fi
}

# 3. Build the Docker image
build_docker_image() {
    echo "[INFO] Building Docker image: $IMAGE_TAG"
    echo "[INFO] This will take 10-20 minutes..."

    if docker images | grep -q "$IMAGE_TAG"; then
        echo "[INFO] Image $IMAGE_TAG already exists."
        read -p "Do you want to rebuild it? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "[INFO] Skipping build. Using existing image."
            return
        fi
    fi

    echo "[INFO] Starting Docker build..."
    # Build without memory limits (apt install is less intensive)
    docker build -t "$IMAGE_TAG" .
    
    if [ -n "$(docker images -q "$IMAGE_TAG")" ]; then
        echo "[INFO] Docker image build complete: $IMAGE_TAG"
    else
        echo "[ERROR] Docker image build failed."
        exit 1
    fi
}

# 4. Set script permissions
set_script_permissions() {
    echo "[INFO] Setting script permissions..."
    local scripts=("run_isaac_sim_docker.sh" "connect_to_container.sh" "stop_container.sh" "restart_container.sh")
    
    for script in "${scripts[@]}"; do
        if [ -f "./$script" ]; then
            chmod +x "./$script"
        fi
    done
}

# 5. Show next steps
show_usage() {
    echo ""
    echo "=========================================="
    echo "Init Complete. What's next?"
    echo "=========================================="
    echo ""
    echo "1. IMPORTANT: Set your DISPLAY variable:"
    echo "   export DISPLAY=YOUR_LOCAL_IP:0"
    echo "   (e.g., export DISPLAY=192.168.1.10:0)"
    echo ""
    echo "2. Run the container:"
    echo "   ./run_isaac_sim_docker.sh"
    echo ""
    echo "3. Connect to the container:"
    echo "   ./connect_to_container.sh"
    echo ""
    echo "4. Inside the container, run Isaac Sim:"
    echo "   cd /isaac-sim"
    echo "   ./isaac-sim.sh"
    echo ""
    echo "5. Or run ROS 2 commands:"
    echo "   ros2 topic list"
    echo "   rviz2"
    echo ""
    echo "=========================================="
}

# --- Main Execution ---
echo ""
echo "[START] Running initialization..."
echo ""

create_directories
check_required_files
build_docker_image
set_script_permissions
show_usage

echo ""
echo "[COMPLETE] Setup finished!"
echo ""