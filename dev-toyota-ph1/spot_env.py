# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

# This sample enables a livestream server to connect to when running headless
CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": False,  # Show the GUI
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Set display options to show default grid
}

simulation_app = SimulationApp(launch_config=CONFIG)

from isaacsim.core.utils.extensions import enable_extension

# Default Livestream settings
simulation_app.set_setting("/app/window/drawMouse", True)

# Enable Livestream extension
enable_extension("omni.services.livestream.nvcf")
enable_extension("omni.kit.livestream.webrtc")

# Enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

import argparse
import threading
import sys
import select
import os

import carb
import numpy as np
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

# ROS2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from tf2_msgs.msg import TFMessage
    from cv_bridge import CvBridge
    import cv2
    ROS2_AVAILABLE = True
    print("ROS2 libraries imported successfully")
except ImportError as e:
    print(f"Warning: ROS2 libraries not available: {e}")
    print("ROS2 functionality will be disabled")
    ROS2_AVAILABLE = False
    # Create dummy classes for compatibility
    class Node:
        pass
    class Image:
        pass
    class TFMessage:
        pass
    class CvBridge:
        pass
    cv2 = None

first_step = True
reset_needed = False

# Global variable to store latest RGB image
latest_rgb_image = None
rgb_image_received = False

# Global variable to store latest TF message
latest_tf_msg = None
tf_received = False

class RGBSubscriber(Node):
    """ROS2 node to subscribe to RGB camera topic"""
    
    def __init__(self):
        if not ROS2_AVAILABLE:
            print("ROS2 not available, creating dummy subscriber")
            return
        super().__init__('spot_rgb_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/spot_camera/rgb',
            self.rgb_callback,
            10
        )
        self.bridge = CvBridge()
        self.message_count = 0
        self.get_logger().info('RGB Subscriber initialized, listening to /spot_camera/rgb topic')
    
    def rgb_callback(self, msg):
        """Callback function for RGB image messages"""
        global latest_rgb_image, rgb_image_received
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            latest_rgb_image = cv_image
            rgb_image_received = True
            self.message_count += 1
            if self.message_count % 10 == 0:  # Log every 10th message
                self.get_logger().info(f'Received {self.message_count} RGB images, latest: {cv_image.shape}')
            else:
                self.get_logger().debug(f'Received RGB image: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

class TFSubscriber(Node):
    """ROS2 node to subscribe to TF topic"""
    
    def __init__(self):
        if not ROS2_AVAILABLE:
            print("ROS2 not available, creating dummy TF subscriber")
            return
        super().__init__('spot_tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/spot_tf',
            self.tf_callback,
            10
        )
        self.get_logger().info('TF Subscriber initialized, listening to /spot_tf topic')
    
    def tf_callback(self, msg):
        """Callback function for TF messages"""
        global latest_tf_msg, tf_received
        try:
            latest_tf_msg = msg
            tf_received = True
            self.get_logger().debug(f'Received TF message with {len(msg.transforms)} transforms')
        except Exception as e:
            self.get_logger().error(f'Error processing TF message: {str(e)}')

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--interactive", default=True, action="store_true", help="Enable interactive keyboard control")
parser.add_argument("--auto", default=False, action="store_true", help="Run automatic square pattern")
parser.add_argument("--ros2", default=True, action="store_true", help="Enable ROS2 RGB topic subscription")
args, unknown = parser.parse_known_args()

# Camera configuration
CAMERA_PRIM_PATH = "/World/Spot/body/Camera"

def setup_camera():
    """Setup camera using Isaac Sim Camera class"""
    
    # Create camera using Isaac Sim Camera class
    camera = Camera(
        prim_path=CAMERA_PRIM_PATH,
        position=np.array([3.47, 0.0, 0.8]),  # Position relative to robot body
        frequency=10,  # 10 Hz - a factor of rendering frequency (50 Hz)
        resolution=(1280, 720),
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
    )
    camera.initialize()
    
    # Set field of view by adjusting focal length and apertures
    # FOV = 2 * arctan(aperture / (2 * focal_length))
    # For narrower FOV, increase focal length
    
    # Calculate focal length for narrower FOV (around 70-75° instead of 90°)
    horizontal_aperture = 20.0  # Standard aperture size in stage units
    # Increase focal length to reduce FOV
    focal_length = horizontal_aperture / 1.4  # This gives approximately 75° FOV (narrower than 90°)
    
    camera.set_focal_length(focal_length)
    camera.set_horizontal_aperture(horizontal_aperture)
    
    # Try to increase frequency to 25 Hz after initialization
    try:
        camera.set_frequency(30)  # 25 Hz for higher update rate
        print(f"Successfully set camera frequency to 30 Hz")
    except Exception as e:
        print(f"Could not set frequency to 30 Hz: {e}")
        print(f"Keeping frequency at 10 Hz")
    
    simulation_app.update()
    camera.initialize()  # Initialize again after update as shown in example
    
    print(f"Camera setup complete at {CAMERA_PRIM_PATH}")
    print("Camera class initialized for RGB and depth data")
    print(f"Camera focal length: {focal_length} stage units")
    print(f"Camera horizontal aperture: {horizontal_aperture} stage units")
    print(f"Approximate horizontal FOV: 75° (narrower than previous 90°)")
    
    return camera

def publish_rgb(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    # Calculate step size based on rendering frequency (50 Hz) and desired publishing frequency
    rendering_freq = 30  # From rendering_dt=1/50
    step_size = max(1, int(rendering_freq/freq))
    topic_name = "spot_camera/rgb"
    queue_size = 10  # Increased queue size to handle bursts
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

    print(f"Setting up ROS2 RGB publisher:")
    print(f"  Topic: {topic_name}")
    print(f"  Frequency: {freq} Hz")
    print(f"  Step size: {step_size}")
    print(f"  Queue size: {queue_size}")
    print(f"  Frame ID: {frame_id}")

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    print(f"ROS2 RGB publisher setup complete")
    return

# initialize robot on first step, run robot advance
def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    if first_step:
        spot.initialize()
        first_step = False
    elif reset_needed:
        sim.reset(True)
        reset_needed = False
        first_step = True
    else:
        spot.forward(step_size, base_command)

# Create Isaac Sim simulation context
sim = SimulationContext(stage_units_in_meters=1.0, physics_dt=1/500, rendering_dt=1/50)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
prim.GetReferences().AddReference(asset_path)

# spawn robot
spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    position=np.array([3.0, 0, 0.8]),
)

# Setup camera
camera = setup_camera()

# Publish RGB camera data to ROS2 topic
if args.ros2:
    # Use the actual camera frequency for publishing
    camera_freq = camera.get_frequency()
    publish_rgb(camera, camera_freq)
    print(f"ROS2 publishing at {camera_freq} Hz")

sim.reset()
sim.add_physics_callback("physics_step", callback_fn=on_physics_step)

# robot command
base_command = np.zeros(3)

# Interactive control variables
i = 0

def print_controls():
    print("\n=== Spot Robot Control Commands ===")
    print("W/S: Forward/Backward")
    print("A/D: Left/Right strafe")
    print("Q/E: Turn left/right")
    print("X: Stop")
    print("R: Reset robot position")
    print("C: Display RGB image (if ROS2 enabled)")
    print("T: Display TF info (if ROS2 enabled)")
    print("S: Show ROS2 topic status")
    print("H: Show this help")
    print("Ctrl+C: Exit")
    print("================================")

def get_key_input():
    """Non-blocking key input for interactive control"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

# Initialize ROS2 if requested
ros2_node = None
tf_node = None
if args.ros2:
    if ROS2_AVAILABLE:
        try:
            rclpy.init()
            ros2_node = RGBSubscriber()
            tf_node = TFSubscriber()
            print("ROS2 RGB and TF subscribers initialized successfully")
        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")
            args.ros2 = False
    else:
        print("ROS2 not available, disabling ROS2 functionality")
        args.ros2 = False

if args.interactive:
    print_controls()
    # Set terminal to raw mode for immediate key input
    import termios
    import tty
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
    except:
        print("Warning: Could not set raw terminal mode. Interactive control may not work properly.")

while simulation_app.is_running():
    # Handle ROS2 callbacks if enabled
    if args.ros2:
        try:
            if ros2_node:
                rclpy.spin_once(ros2_node, timeout_sec=0.001)
            if tf_node:
                rclpy.spin_once(tf_node, timeout_sec=0.001)
        except Exception as e:
            print(f"ROS2 spin error: {e}")
    
    sim.step(render=True)
    
    if sim.is_stopped():
        reset_needed = True
    if sim.is_playing():
        if args.interactive:
            # Interactive keyboard control
            key = get_key_input()
            if key:
                if key.lower() == 'w':
                    base_command = np.array([1.0, 0, 0])  # Forward
                    print("Moving forward")
                elif key.lower() == 's':
                    base_command = np.array([-1.0, 0, 0])  # Backward
                    print("Moving backward")
                elif key.lower() == 'a':
                    base_command = np.array([0, 1.0, 0])  # Left strafe
                    print("Strafing left")
                elif key.lower() == 'd':
                    base_command = np.array([0, -1.0, 0])  # Right strafe
                    print("Strafing right")
                elif key.lower() == 'q':
                    base_command = np.array([0, 0, 1.0])  # Turn left
                    print("Turning left")
                elif key.lower() == 'e':
                    base_command = np.array([0, 0, -1.0])  # Turn right
                    print("Turning right")
                elif key.lower() == 'x':
                    base_command = np.zeros(3)  # Stop
                    print("Stopped")
                elif key.lower() == 'r':
                    reset_needed = True
                    print("Resetting robot position")
                elif key.lower() == 'h':
                    print_controls()
                elif key.lower() == 'c':
                    # Display RGB image if available
                    if args.ros2 and latest_rgb_image is not None and cv2 is not None:
                        print(f"RGB Image shape: {latest_rgb_image.shape}")
                        cv2.imshow('RGB Image from /spot_camera/rgb topic', latest_rgb_image)
                        cv2.waitKey(1)
                    else:
                        print("No RGB image available (ROS2 or OpenCV not available)")
                elif key.lower() == 't':
                    # Display TF info if available
                    if args.ros2 and latest_tf_msg is not None:
                        print(f"TF Message received with {len(latest_tf_msg.transforms)} transforms")
                        for i, transform in enumerate(latest_tf_msg.transforms):
                            print(f"  Transform {i}: {transform.header.frame_id} -> {transform.child_frame_id}")
                    else:
                        print("No TF message available (ROS2 not available)")
                elif key.lower() == 's':
                    # Display ROS2 topic status
                    if args.ros2 and ros2_node:
                        print(f"ROS2 RGB subscriber status:")
                        print(f"  Messages received: {ros2_node.message_count}")
                        print(f"  Latest image shape: {latest_rgb_image.shape if latest_rgb_image is not None else 'None'}")
                        print(f"  Image received: {rgb_image_received}")
                    else:
                        print("ROS2 not available")
                elif key == '\x03':  # Ctrl+C
                    break
            else:
                # Gradually reduce speed when no key is pressed (smooth deceleration)
                base_command *= 0.95
                if np.linalg.norm(base_command) < 0.1:
                    base_command = np.zeros(3)
                    
        elif args.auto:
            # Automatic square pattern
            print(f"Step {i}")
            if i >= 0 and i < 200:
                # Side 1: Forward (positive X direction)
                base_command = np.array([2, 0, 0])
            elif i >= 200 and i < 400:
                # Turn 90 degrees left (positive Z rotation)
                base_command = np.array([0, 0, 2])
            elif i == 400:
                i = 0
                if args.test is True:
                    print("Completed square cycle. Final position: ", spot.robot.get_world_pose()[0])
                    break
            i += 1
        else:
            # Default: simple forward movement
            base_command = np.array([1.0, 0, 0])

# Cleanup terminal settings if interactive mode was used
if args.interactive:
    try:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    except:
        pass

# Cleanup ROS2 and OpenCV
if args.ros2:
    try:
        if ros2_node:
            ros2_node.destroy_node()
        if tf_node:
            tf_node.destroy_node()
        if ROS2_AVAILABLE:
            rclpy.shutdown()
        print("ROS2 nodes destroyed")
    except:
        pass

if cv2 is not None:
    cv2.destroyAllWindows()
simulation_app.close()
