#!/usr/bin/env python3

import os
import time
import json
import threading
import argparse
from PIL import Image
import numpy as np
import cv2
from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
import torch

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as ROSImage
    from geometry_msgs.msg import Twist
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
    print("ROS2 libraries imported successfully")
except ImportError as e:
    print(f"Warning: ROS2 libraries not available: {e}")
    print("ROS2 functionality will be disabled")
    ROS2_AVAILABLE = False
    # Create dummy classes for compatibility
    class Node:
        pass
    class ROSImage:
        pass
    class Twist:
        pass
    class CvBridge:
        pass

class VLMController(Node):
    def __init__(self, instruction="Navigate safely through the environment", save_results=False):
        if not ROS2_AVAILABLE:
            print("ROS2 not available, creating dummy controller")
            return
            
        super().__init__('vlm_robot_controller')
        
        # Initialize VLM
        self.init_local_vlm()
        
        # Optional result saving
        self.save_results = save_results
        if self.save_results:
            self.setup_save_directory()
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        self.instruction = instruction
        
        # Subscribe to RGB camera topic
        self.image_subscription = self.create_subscription(
            ROSImage,
            '/spot_camera/rgb',
            self.image_callback,
            10
        )
        
        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Publisher for captioned images
        self.captioned_image_publisher = self.create_publisher(
            ROSImage,
            '/vlm_captioned_image',
            10
        )
        
        # Control parameters
        self.control_frequency = 2.0  # Hz
        self.last_control_time = 0
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_callback)
        
        self.get_logger().info('VLM Robot Controller initialized')
        self.get_logger().info(f'Instruction: {instruction}')
        
    def setup_save_directory(self):
        """Setup directory to save results"""
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.save_dir = f"vlm_control_results_{timestamp}"
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f'Results will be saved to: {self.save_dir}')
        
    def save_result(self, image, action, vlm_response, twist_cmd):
        """Save result data to files"""
        if not self.save_results:
            return
            
        try:
            timestamp = time.strftime('%Y%m%d_%H%M%S_%f')[:-3]  # Include milliseconds
            
            # Save image
            image_filename = f"{self.save_dir}/image_{timestamp}.jpg"
            cv2.imwrite(image_filename, image)
            
            # Save result data
            result_data = {
                "timestamp": timestamp,
                "instruction": self.instruction,
                "action": action,
                "vlm_response": vlm_response,
                "twist_command": {
                    "linear": {
                        "x": float(twist_cmd.linear.x),
                        "y": float(twist_cmd.linear.y),
                        "z": float(twist_cmd.linear.z)
                    },
                    "angular": {
                        "x": float(twist_cmd.angular.x),
                        "y": float(twist_cmd.angular.y),
                        "z": float(twist_cmd.angular.z)
                    }
                }
            }
            
            result_filename = f"{self.save_dir}/result_{timestamp}.json"
            with open(result_filename, 'w') as f:
                json.dump(result_data, f, indent=2)
                
            self.get_logger().debug(f'Saved result: {result_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving result: {str(e)}')
        
    def init_local_vlm(self):
        """Initialize the local Vision Language Model"""
        try:
            model_name = "Qwen/Qwen2.5-VL-7B-Instruct"
            
            self.get_logger().info(f'Loading VLM model: {model_name}')
            self.processor = AutoProcessor.from_pretrained(model_name)
            self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
                model_name,
                torch_dtype=torch.float16
            )
            self.model = self.model.to("cuda")
            self.model.eval()
            self.get_logger().info('VLM model loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load VLM model: {e}')
            self.processor = None
            self.model = None
    
    def image_callback(self, msg):
        """Callback function for RGB image messages"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.image_received = True
            self.get_logger().debug(f'Received RGB image: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')
    
    def analyze_image_with_vlm(self, cv_image, instruction: str) -> dict:
        """Analyze image using VLM and return movement instructions"""
        if self.processor is None or self.model is None:
            self.get_logger().error("VLM model not available")
            return {"action": "stop"}
        
        try:
            # Convert OpenCV image to PIL
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(cv_image_rgb)
            
            # VLM prompt for warehouse robot navigation
            vlm_prompt = f"""
You are an intelligent warehouse robot controller for a Spot robot. You are analyzing a camera image to execute warehouse tasks strategically.

Current instruction: "{instruction}"

WAREHOUSE CONTEXT:
- You are in a warehouse environment with shelves, aisles, and various objects
- You need to navigate safely while completing your assigned task
- For search tasks, be systematic and thorough in your approach
- Consider warehouse layout, obstacles, and efficient pathfinding

SEARCH STRATEGY (if searching for something):
- Look systematically: scan left to right, then move to get different angles
- If you see the target object, move directly toward it
- If you don't see the target, move strategically to explore new areas
- Use turning to get better views of different areas
- Avoid getting stuck in corners or dead ends

Available actions:
- move forward X cm (0-100) - Move straight ahead
- move backward X cm (0-100) - Move backward (use sparingly)
- move left X cm (0-100) - Strafe left
- move right X cm (0-100) - Strafe right  
- turn left X degree (0-90) - Turn left to change viewing angle
- turn right X degree (0-90) - Turn right to change viewing angle
- stop - Stop all movement

SMART NAVIGATION RULES:
- For search tasks: Use turns to scan areas, then move to explore new viewpoints
- For approach tasks: Move directly toward the target when visible
- Avoid obstacles: Use strafing or turning to navigate around them
- Be efficient: Choose the most direct path when possible
- Stay safe: Don't move into tight spaces or near edges

IMPORTANT: Output ONLY ONE JSON object with "action" field:
{{"action": "your action here"}}

Examples:
{{"action": "turn left 45 degree"}} - To scan a new area during search
{{"action": "move forward 30cm"}} - To approach a visible target
{{"action": "move right 20cm"}} - To navigate around an obstacle
{{"action": "stop"}} - When task is complete or unsafe to proceed

Analyze the image and provide your response as a single JSON object with only the action:
"""
            
            conversation = [
                {
                    "role": "user",
                    "content": [
                        {"type": "image"},
                        {"type": "text", "text": vlm_prompt},
                    ],
                },
            ]
            
            text = self.processor.apply_chat_template(conversation, tokenize=False, add_generation_prompt=True)
            inputs = self.processor(text=[text], images=[pil_image], return_tensors="pt")
            inputs = {k: v.to("cuda") for k, v in inputs.items()}
            
            with torch.no_grad():
                generated_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=512,
                    num_beams=1,
                    temperature=0.1,
                    do_sample=True,
                    top_p=0.9,
                    repetition_penalty=1.1
                )
            
            # Separate input and output tokens
            generated_ids_trimmed = [
                out_ids[len(in_ids):] for in_ids, out_ids in zip(inputs['input_ids'], generated_ids)
            ]
            
            # Decode only the generated part
            response = self.processor.tokenizer.batch_decode(
                generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=True
            )[0]
            
            # Parse JSON response
            import re
            json_match = re.search(r'\{[^}]*\}', response)
            if json_match:
                json_str = json_match.group()
                try:
                    result = json.loads(json_str)
                    self.get_logger().info(f'VLM Response: {response.strip()}')
                    return result
                except json.JSONDecodeError:
                    self.get_logger().error(f"JSON parsing error: {json_str}")
                    return {"action": "stop"}
            else:
                self.get_logger().error(f"No JSON found in response: {response}")
                return {"action": "stop"}
                
        except Exception as e:
            self.get_logger().error(f'Error in VLM analysis: {str(e)}')
            return {"action": "stop"}
    
    def parse_action_to_twist(self, action_str: str) -> Twist:
        """Convert action string to ROS2 Twist message"""
        twist = Twist()
        
        try:
            action_str = action_str.lower().strip()
            
            if "stop" in action_str:
                return twist  # All zeros
            
            # Parse movement commands
            if "move forward" in action_str:
                # Extract distance
                import re
                match = re.search(r'(\d+)', action_str)
                if match:
                    distance = int(match.group(1))
                    # Convert cm to m/s, cap at reasonable speed
                    speed = min(distance / 100.0, 1.0)  # Max 1 m/s
                    twist.linear.x = speed
                    
            elif "move backward" in action_str:
                match = re.search(r'(\d+)', action_str)
                if match:
                    distance = int(match.group(1))
                    speed = min(distance / 100.0, 1.0)
                    twist.linear.x = -speed
                    
            elif "move left" in action_str:
                match = re.search(r'(\d+)', action_str)
                if match:
                    distance = int(match.group(1))
                    speed = min(distance / 100.0, 1.0)
                    twist.linear.y = speed
                    
            elif "move right" in action_str:
                match = re.search(r'(\d+)', action_str)
                if match:
                    distance = int(match.group(1))
                    speed = min(distance / 100.0, 1.0)
                    twist.linear.y = -speed
                    
            elif "turn left" in action_str:
                match = re.search(r'(\d+)', action_str)
                if match:
                    angle = int(match.group(1))
                    # Convert degrees to rad/s
                    angular_speed = min(angle * 0.0174533, 1.0)  # Max 1 rad/s
                    twist.angular.z = angular_speed
                    
            elif "turn right" in action_str:
                match = re.search(r'(\d+)', action_str)
                if match:
                    angle = int(match.group(1))
                    angular_speed = min(angle * 0.0174533, 1.0)
                    twist.angular.z = -angular_speed
                    
        except Exception as e:
            self.get_logger().error(f'Error parsing action: {e}')
            
        return twist
    
    def publish_captioned_image(self, action: str):
        """Create and publish image with action caption"""
        try:
            if self.latest_image is None:
                return
                
            # Create a copy of the image for captioning
            captioned_image = self.latest_image.copy()
            
            # Add instruction and action captions to the image
            instruction_caption = f"Instruction: {self.instruction}"
            action_caption = f"Action: {action}"
            
            # Get image dimensions
            height, width = captioned_image.shape[:2]
            
            # Set text properties
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            thickness = 2
            instruction_color = (255, 255, 0)  # Yellow for instruction
            action_color = (0, 255, 0)  # Green for action
            
            # Get text sizes
            (instruction_width, instruction_height), baseline = cv2.getTextSize(instruction_caption, font, font_scale, thickness)
            (action_width, action_height), baseline = cv2.getTextSize(action_caption, font, font_scale, thickness)
            
            # Calculate positions (top-left corner with padding)
            text_x = 20
            instruction_y = instruction_height + 20
            action_y = instruction_y + action_height + 10
            
            # Add background rectangle for instruction
            cv2.rectangle(captioned_image, 
                         (text_x - 10, instruction_y - instruction_height - 10),
                         (text_x + instruction_width + 10, instruction_y + 10),
                         (0, 0, 0), -1)  # Black background
            
            # Add background rectangle for action
            cv2.rectangle(captioned_image, 
                         (text_x - 10, action_y - action_height - 10),
                         (text_x + action_width + 10, action_y + 10),
                         (0, 0, 0), -1)  # Black background
            
            # Add instruction text
            cv2.putText(captioned_image, instruction_caption, (text_x, instruction_y), 
                       font, font_scale, instruction_color, thickness)
            
            # Add action text
            cv2.putText(captioned_image, action_caption, (text_x, action_y), 
                       font, font_scale, action_color, thickness)
            
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(captioned_image, "bgr8")
            
            # Publish captioned image
            self.captioned_image_publisher.publish(ros_image)
            
            self.get_logger().debug(f'Published captioned image with instruction: {self.instruction}, action: {action}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing captioned image: {str(e)}')
    
    def control_callback(self):
        """Main control loop callback"""
        if not self.image_received or self.latest_image is None:
            self.get_logger().warn("No image received yet")
            return
        
        current_time = time.time()
        if current_time - self.last_control_time < 1.0/self.control_frequency:
            return
            
        self.last_control_time = current_time
        
        try:
            # Analyze image with VLM
            result = self.analyze_image_with_vlm(self.latest_image, self.instruction)
            
            # Only handle actions (no thoughts)
            if "action" in result:
                action = result["action"]
                self.get_logger().info(f'Executing action: {action}')
                
                # Convert action to Twist command
                twist_cmd = self.parse_action_to_twist(action)
                
                # Publish command
                self.cmd_vel_publisher.publish(twist_cmd)
                
                # Log command details
                self.get_logger().info(f'Published Twist: linear=({twist_cmd.linear.x:.2f}, {twist_cmd.linear.y:.2f}, {twist_cmd.linear.z:.2f}), angular=({twist_cmd.angular.x:.2f}, {twist_cmd.angular.y:.2f}, {twist_cmd.angular.z:.2f})')
                
                # Save result if enabled
                self.save_result(self.latest_image, action, result, twist_cmd)
                
                # Create and publish captioned image
                self.publish_captioned_image(action)
            else:
                # If no action found, stop and publish stop action image
                stop_twist = Twist()
                self.cmd_vel_publisher.publish(stop_twist)
                self.save_result(self.latest_image, "stop", result, stop_twist)
                self.publish_captioned_image("stop")
                
        except Exception as e:
            self.get_logger().error(f'Error in control callback: {str(e)}')
            # Emergency stop
            stop_twist = Twist()
            self.cmd_vel_publisher.publish(stop_twist)

def main():
    parser = argparse.ArgumentParser(description='VLM-based Robot Controller')
    parser.add_argument('--instruction', type=str, 
                       default="Navigate safely through the environment, avoid obstacles",
                       help='Navigation instruction for the robot')
    parser.add_argument('--frequency', type=float, default=2.0,
                       help='Control frequency in Hz')
    parser.add_argument('--test', action='store_true',
                       help='Run in test mode (no ROS2)')
    parser.add_argument('--save-results', action='store_true',
                       help='Save results (images and data) to files')
    
    args = parser.parse_args()
    
    if args.test:
        print("Running in test mode without ROS2")
        print(f"Instruction: {args.instruction}")
        print("This would normally connect to ROS2 and control the robot")
        return
    
    if not ROS2_AVAILABLE:
        print("ROS2 not available. Please install ROS2 and required packages.")
        return
    
    try:
        rclpy.init()
        controller = VLMController(instruction=args.instruction, save_results=args.save_results)
        controller.control_frequency = args.frequency
        
        print(f"VLM Robot Controller started")
        print(f"Instruction: {args.instruction}")
        print(f"Control frequency: {args.frequency} Hz")
        print(f"Result saving: {'Enabled' if args.save_results else 'Disabled'}")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ROS2_AVAILABLE:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
