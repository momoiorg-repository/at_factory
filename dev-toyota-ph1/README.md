# VLM Robot Controller for Spot Robot

This system uses a Vision Language Model (VLM) to control a Spot robot based on camera images. The VLM analyzes RGB images from the robot's camera and provides movement instructions for autonomous navigation.

## Overview

The system consists of four main components:

1. **Spot Robot Simulation** (`spot_env.py`) - Provides the robot simulation and publishes RGB camera data
2. **VLM Controller** (`vlm_robot_controller.py`) - Analyzes images, generates captions, and controls the robot
3. **Captioned Image Viewer** (`captioned_image_viewer.py`) - Displays captioned images with action overlays
4. **Requirements** (`requirements.txt`) - Python package dependencies

## Installation

### Prerequisites

- Python 3.8 or higher
- CUDA-capable GPU (for VLM inference)
- At least 8GB GPU memory (for Qwen2.5-VL-7B model)
- Isaac Sim (for robot simulation)

### Install Python Dependencies

From the `/IsaacLab` directory, run:

```bash
pip install -r scripts/dev-toyota-ph1/requirements.txt
```

This will install:
- `torch==2.8.0` - PyTorch for deep learning
- `transformers>=4.40.0` - Hugging Face transformers for VLM
- `opencv-python==4.8.1.78` - Computer vision library
- `Pillow>=9.0.0` - Image processing
- `numpy==1.24.3` - Numerical computing
- And other required dependencies

### Optional ROS2 Dependencies

If you want to use ROS2 functionality, install:

```bash
pip install rclpy sensor-msgs geometry-msgs cv-bridge
```

## File Descriptions

### `spot_env.py`
- **Purpose**: Spot robot simulation environment with Isaac Sim
- **Features**:
  - Spawns Spot robot in warehouse environment
  - Publishes RGB camera data to ROS2 topic `/spot_camera/rgb`
  - Provides interactive keyboard control (WASD keys)
  - Supports automatic movement patterns
  - Includes ROS2 integration for camera data publishing
- **Usage**: Run this first to start the robot simulation

### `vlm_robot_controller.py`
- **Purpose**: Main VLM controller that analyzes images and controls robot movement
- **Features**:
  - Uses Qwen2.5-VL-7B-Instruct model for image analysis
  - Subscribes to `/spot_camera/rgb` for camera images
  - Publishes movement commands to `/cmd_vel`
  - Publishes captioned images to `/vlm_captioned_image`
  - Configurable navigation instructions
  - Safety controls and speed limiting
- **Usage**: Run this after starting the robot simulation

### `captioned_image_viewer.py`
- **Purpose**: Displays captioned images with action overlays
- **Features**:
  - Subscribes to `/vlm_captioned_image` topic
  - Shows images with instruction and action captions
  - Resizes images for optimal display
  - Real-time visualization of robot decisions
- **Usage**: Optional viewer for monitoring robot actions

### `requirements.txt`
- **Purpose**: Lists all Python package dependencies
- **Contains**: Core ML/AI libraries, computer vision tools, and scientific computing packages

## How to Run

### Step 1: Start Spot Robot Simulation

From `/IsaacLab` directory:

```bash
# Method 1: Using Isaac Sim Python shell
/isaac-sim/python.sh scripts/dev-toyota-ph1/spot_env.py

# Method 2: With specific options
/isaac-sim/python.sh scripts/dev-toyota-ph1/spot_env.py --ros2 --interactive

# Method 3: Test mode (no ROS2)
/isaac-sim/python.sh scripts/dev-toyota-ph1/spot_env.py --test
```

**Interactive Controls** (when simulation is running):
- `W/S`: Forward/Backward
- `A/D`: Left/Right strafe  
- `Q/E`: Turn left/right
- `X`: Stop
- `R`: Reset robot position
- `C`: Display RGB image
- `H`: Show help

### Step 2: Start VLM Controller

In a new terminal, from `/IsaacLab` directory:

```bash
# Basic navigation
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --instruction "Navigate safely through the environment"

# Find specific object
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --instruction "Find and approach the Traffic Cone. Stop when close to it." --frequency 5.0

# Higher frequency control
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --frequency 5.0 --instruction "Avoid obstacles and move forward"

# Save results to files
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --save-results --instruction "Explore the warehouse"

# Test mode (no ROS2)
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --test
```

### Step 3: Start Captioned Image Viewer (Optional)

In another terminal, from `/IsaacLab` directory:

```bash
# Basic viewer
python3 scripts/dev-toyota-ph1/captioned_image_viewer.py

# With save option
python3 scripts/dev-toyota-ph1/captioned_image_viewer.py --save-images
```

## Usage Examples

### Example 1: Basic Navigation
```bash
# Terminal 1: Start simulation
/isaac-sim/python.sh scripts/dev-toyota-ph1/spot_env.py

# Terminal 2: Start VLM controller
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --instruction "Navigate safely through the environment"
```

### Example 2: Object Search
```bash
# Terminal 1: Start simulation
/isaac-sim/python.sh scripts/dev-toyota-ph1/spot_env.py

# Terminal 2: Search for specific object
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --instruction "Find and approach the red box. Stop when close to it." --frequency 3.0

# Terminal 3: View captioned images
python3 scripts/dev-toyota-ph1/captioned_image_viewer.py
```

### Example 3: High-Frequency Control
```bash
# Terminal 1: Start simulation with ROS2
/isaac-sim/python.sh scripts/dev-toyota-ph1/spot_env.py --ros2

# Terminal 2: High-frequency navigation
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py --frequency 5.0 --instruction "Explore the warehouse systematically, avoid obstacles"

# Terminal 3: Monitor actions
python3 scripts/dev-toyota-ph1/captioned_image_viewer.py
```

## Command Line Options

### VLM Controller Options
```bash
python3 scripts/dev-toyota-ph1/vlm_robot_controller.py [OPTIONS]

Options:
  --instruction TEXT    Navigation instruction for the robot
                       Default: "Navigate safely through the environment, avoid obstacles"
  
  --frequency FLOAT     Control frequency in Hz
                       Default: 2.0
  
  --save-results        Save results (images and data) to files
  
  --test               Run in test mode (no ROS2)
```

### Spot Environment Options
```bash
/isaac-sim/python.sh scripts/dev-toyota-ph1/spot_env.py [OPTIONS]

Options:
  --ros2               Enable ROS2 RGB topic subscription (default: True)
  --interactive        Enable interactive keyboard control (default: True)
  --auto               Run automatic square pattern
  --test               Run in test mode
```

## Available Actions

The VLM can generate the following movement commands:

- `move forward X cm` - Move forward at specified distance
- `move backward X cm` - Move backward at specified distance  
- `move left X cm` - Strafe left at specified distance
- `move right X cm` - Strafe right at specified distance
- `turn left X degree` - Turn left at specified angle
- `turn right X degree` - Turn right at specified angle
- `stop` - Stop all movement

## Troubleshooting

### Common Issues

1. **CUDA Out of Memory**
   ```bash
   RuntimeError: CUDA out of memory
   ```
   - Close other GPU applications
   - Reduce model precision in code
   - Consider using a smaller model

2. **ROS2 Not Available**
   ```bash
   Warning: ROS2 libraries not available
   ```
   - Install ROS2 packages: `pip install rclpy sensor-msgs geometry-msgs cv-bridge`
   - Or use `--test` mode for testing without ROS2

3. **No Images Received**
   ```bash
   No image received yet
   ```
   - Ensure Spot simulation is running with `--ros2` flag
   - Check that `/spot_camera/rgb` topic is publishing
   - Verify ROS2 network configuration

4. **VLM Model Loading Failed**
   ```bash
   Failed to load VLM model
   ```
   - Check internet connection for model download
   - Verify sufficient disk space
   - Ensure transformers library is up to date

## Performance Tips

### For Better Performance
- Reduce control frequency: Lower frequency reduces computational load
- Use smaller VLM models for faster inference
- Optimize image size: Reduce camera resolution if needed

### For Better Accuracy
- Increase control frequency: Higher frequency provides more responsive control
- Use larger models: Larger models generally provide better reasoning
- Fine-tune prompts: Customize instructions for specific tasks

## Safety Considerations

- The system includes automatic emergency stops
- Speed limits are enforced to prevent dangerous movements
- Always test in simulation before using with real robots
- Monitor the robot's behavior during operation
- Have a manual override mechanism available

## Future Enhancements

- Multi-modal input (depth, lidar, etc.)
- Learning from demonstrations
- Integration with SLAM for mapping
- Support for multiple robots
- Web-based control interface
- Recording and replay capabilities
