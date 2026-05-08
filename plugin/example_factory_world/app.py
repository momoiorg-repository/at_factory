"""
Standalone Isaac Sim script to open the factory_base.usd world in livestream mode.
This script loads the specified USD world and runs the simulation with livestream capabilities.
"""

import argparse
import json
import os
import sys

# 1. SETUP ARGUMENT PARSER
# We do this BEFORE importing SimulationApp so we can configure it based on flags
parser = argparse.ArgumentParser(description="Isaac Sim Factory Livestream")
parser.add_argument("--cloud", action="store_true", help="Enable cloud livestream settings")
parser.add_argument("--ip", type=str, default="127.0.0.1", help="Public endpoint IP address (used if --cloud is set)")

# Use parse_known_args to avoid errors with internal Isaac Sim arguments
args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

# Configuration for the simulation application in livestream mode
CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,   # Required for livestream mode
    "hide_ui": False,   # Show the GUI
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Set display options to show default grid
}

# Add livestream args only if --cloud is specified
if args.cloud:
    # We use the IP provided in the arguments
    ip_address = args.ip
    CONFIG["extra_args"] = [
        "--/app/livestream/publicEndpointAddress=" + ip_address, 
        "--/app/livestream/port=49100"
    ]
    print(f"Cloud livestream enabled on IP: {ip_address}")

# Start the Isaac Sim application
simulation_app = SimulationApp(launch_config=CONFIG)

# Import necessary modules after SimulationApp is initialized
import omni
import omni.usd
import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
import numpy as np


def main():
    """Main function to load and run the factory USD world in livestream mode."""
    
    # Configure livestream settings
    simulation_app.set_setting("/app/window/drawMouse", True)
    
    # Enable Livestream extension
    print("Enabling livestream extension...")
    enable_extension("omni.services.livestream.nvcf")
    enable_extension("omni.kit.livestream.webrtc")
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("omni.graph.window.action")
    
    simulation_app.update()

    # ROS2 imports are only available after isaacsim.ros2.bridge is enabled
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty

    class FactoryROS2Controller(Node):
        """ROS2 controller for the factory Xform_02 object."""

        def __init__(self, world, stage):
            super().__init__("factory_controller")
            self.world = world
            self.stage = stage
            self.target_path = "/World/quicktrun/Xform_02"
            self._close_position = np.array([-142.32, -155.53, 83.83])
            self._open_position  = np.array([-142.32, -193.53, 83.83])
            self._is_open = False
            self._current_position = self._close_position.copy()
            self._target_position  = self._close_position.copy()
            self._animation_speed  = 0.05
            self._is_animating = False
            self.open_sub  = self.create_subscription(Empty, "cnc/open",  self.open_callback,  10)
            self.close_sub = self.create_subscription(Empty, "cnc/close", self.close_callback, 10)
            self.get_logger().info(f"Factory ROS2 controller ready — controlling {self.target_path}")

        def open_callback(self, msg):
            if self.world.is_playing():
                self._target_position = self._open_position.copy()
                self._is_open = True
                self._is_animating = True

        def close_callback(self, msg):
            if self.world.is_playing():
                self._target_position = self._close_position.copy()
                self._is_open = False
                self._is_animating = True

        def apply_pose(self):
            if self._is_animating:
                dist = np.linalg.norm(self._target_position - self._current_position)
                if dist > 0.01:
                    self._current_position += (self._target_position - self._current_position) / dist * min(self._animation_speed, dist)
                else:
                    self._current_position = self._target_position.copy()
                    self._is_animating = False
            try:
                prim = self.stage.GetPrimAtPath(self.target_path)
                if prim and prim.IsValid():
                    from pxr import Gf
                    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*self._current_position))
            except Exception as e:
                self.get_logger().warn(f"Failed to apply pose: {e}")

    # Path to the USD world file
    usd_path = "/plugin/example_factory_world/assets/at-factory-world/factory_base.usd"

    print(f"Loading USD world: {usd_path}")
    ok = omni.usd.get_context().open_stage(usd_path)
    stage = omni.usd.get_context().get_stage()
    # open_stage returns False and leaves an anonymous layer on failure
    if not ok or not stage or stage.GetRootLayer().anonymous:
        print(f"[ERROR] Failed to open USD world: {usd_path}")
        print("Check that plugin/example_factory_world/assets/ is populated.")
        simulation_app.close()
        return
    print(f"USD world loaded: {stage.GetRootLayer().identifier}")

    # Spawn robots defined by the scenario (set by plugsim up --scenario)
    robots_cfg = json.loads(os.environ.get("PLUGSIM_ROBOTS_JSON", "[]"))
    for robot in robots_cfg:
        usd = robot.get("usd")
        prim_path = robot.get("prim_path", f"/World/{robot['instance']}")
        if not usd:
            print(f"[WARN] No USD declared for robot '{robot['instance']}', skipping spawn")
            continue
        print(f"Spawning '{robot['instance']}' → {usd} at {prim_path}")
        add_reference_to_stage(usd_path=usd, prim_path=prim_path)
        # Apply spawn pose
        spawn = robot.get("spawn", {})
        x, y, z = spawn.get("x", 0.0), spawn.get("y", 0.0), spawn.get("z", 0.0)
        yaw = spawn.get("yaw", 0.0)
        if any(v != 0.0 for v in (x, y, z, yaw)):
            import math
            from pxr import UsdGeom, Gf
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                xform = UsdGeom.Xformable(prim)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
                xform.AddRotateZOp().Set(math.degrees(yaw))

    # Create a world instance for simulation
    world = World(stage_units_in_meters=1.0)
    
    # Initialize ROS2
    print("Initializing ROS2...")
    rclpy.init()
    
    # Create ROS2 controller
    ros_controller = FactoryROS2Controller(world, stage)
    
    # Start the simulation
    print("Starting simulation in livestream mode...")
    world.reset()
    
    # Start timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    
    # Main livestream simulation loop with ROS2 integration
    print("Livestream simulation running with ROS2 control. Press Ctrl+C to exit.")
    print("Connect to the livestream server to view the simulation.")
    print("ROS2 topics available:")
    print("  - factory/xform_02/open (std_msgs/Empty) - Open Xform_02")
    print("  - factory/xform_02/close (std_msgs/Empty) - Close Xform_02")
    
    reset_needed = False
    try:
        while simulation_app.is_running() and not simulation_app.is_exiting():
            # Step the world simulation
            world.step(render=True)
            
            # Process ROS2 messages
            rclpy.spin_once(ros_controller, timeout_sec=0.0)
            
            # Apply pose changes to the target object
            ros_controller.apply_pose()
            
            # Handle timeline reset if needed
            if world.is_stopped() and not reset_needed:
                reset_needed = True
            if world.is_playing():
                if reset_needed:
                    world.reset()
                    reset_needed = False
            
    except KeyboardInterrupt:
        print("\nLivestream simulation stopped by user.")
    except Exception as e:
        print(f"Livestream simulation error: {e}")
    finally:
        # Cleanup
        print("Shutting down livestream simulation...")
        timeline.stop()
        ros_controller.destroy_node()
        rclpy.shutdown()
        world.clear()
        simulation_app.close()

if __name__ == "__main__":
    main()
