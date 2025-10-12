#!/bin/bash

# finstall.sh - Automated plugin/robot installation script
# Usage: ./finstall.sh <github_repo_url>
# Example: ./finstall.sh https://github.com/SSatoya/melon_ros2.git

set -e  # Exit on any error

# Function to display usage
usage() {
    echo "Usage: $0 <github_repo_url>"
    echo "Example: $0 https://github.com/SSatoya/melon_ros2.git"
    exit 1
}

# Check if URL is provided
if [ $# -eq 0 ]; then
    echo "Error: GitHub repository URL is required"
    usage
fi

GITHUB_URL="$1"

# Validate URL format
if [[ ! "$GITHUB_URL" =~ ^https://github\.com/.*\.git$ ]]; then
    echo "Error: Invalid GitHub URL format. Expected: https://github.com/username/repository.git"
    exit 1
fi

echo "Starting installation process..."
echo "Repository URL: $GITHUB_URL"

# Step 1: Create plugin/robot directory
echo "Step 1: Creating plugin/robot directory..."
mkdir -p plugin/robot

# Check if we have write permissions to plugin/robot
if [ ! -w "plugin/robot" ]; then
    echo "No write permission to plugin/robot directory. Attempting to fix permissions..."
    sudo chown -R $(whoami):$(whoami) plugin/robot 2>/dev/null || {
        echo "Could not change ownership. Creating temporary directory instead..."
        mkdir -p temp_install
        cd temp_install
    }
else
    cd plugin/robot
fi

# If we're not in temp_install, we should be in plugin/robot now
if [ "$(basename $(pwd))" != "temp_install" ]; then
    cd plugin/robot
fi

# Step 2: Clone the repository
echo "Step 2: Cloning repository..."
REPO_NAME=$(basename "$GITHUB_URL" .git)
echo "Repository name: $REPO_NAME"

# Remove existing directory if it exists
if [ -d "$REPO_NAME" ]; then
    echo "Directory $REPO_NAME already exists. Removing it..."
    rm -rf "$REPO_NAME"
fi

git clone "$GITHUB_URL"
cd "$REPO_NAME"

# Step 3: Check if install.sh exists and run it
echo "Step 3: Looking for install.sh script..."
if [ -f "./install.sh" ]; then
    echo "Found install.sh script. Making it executable and running..."
    chmod +x ./install.sh
    ./install.sh
    echo "Installation completed successfully!"
else
    echo "Warning: install.sh script not found in the repository."
    echo "Repository cloned successfully, but no install.sh script to execute."
    echo "You may need to run the installation manually."
fi

echo "Process completed. Repository is available at: $(pwd)"

# If we used temp_install, provide instructions for moving the files
if [ "$(basename $(pwd))" = "temp_install" ]; then
    echo ""
    echo "Note: Repository was installed in temp_install/ due to permission issues."
    echo "To move it to the proper location, run:"
    echo "  sudo mv temp_install/$REPO_NAME plugin/robot/"
    echo "  sudo chown -R root:root plugin/robot/$REPO_NAME"
    echo "  rm -rf temp_install"
    echo ""
    echo "The repository is currently available at: $(pwd)/$REPO_NAME"
fi
