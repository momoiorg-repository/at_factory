#!/usr/bin/env python3

"""
Captioned Image Viewer
This script subscribes to the captioned images published by the VLM controller
and displays them in a window.
"""

import cv2
import argparse
import sys

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as ROSImage
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
    class CvBridge:
        pass

class CaptionedImageViewer(Node):
    def __init__(self):
        if not ROS2_AVAILABLE:
            print("ROS2 not available, creating dummy viewer")
            return
            
        super().__init__('captioned_image_viewer')
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        
        # Subscribe to captioned image topic
        self.image_subscription = self.create_subscription(
            ROSImage,
            '/vlm_captioned_image',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Captioned Image Viewer initialized')
        self.get_logger().info('Listening to /vlm_captioned_image topic')
        
    def image_callback(self, msg):
        """Callback function for captioned image messages"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.image_received = True
            self.get_logger().debug(f'Received captioned image: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error processing captioned image: {str(e)}')
    
    def display_image(self):
        """Display the latest captioned image"""
        if self.latest_image is not None:
            # Resize image if too large for display
            height, width = self.latest_image.shape[:2]
            max_width = 1200
            max_height = 800
            
            if width > max_width or height > max_height:
                scale = min(max_width / width, max_height / height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                display_image = cv2.resize(self.latest_image, (new_width, new_height))
            else:
                display_image = self.latest_image
            
            # Add window title with action info
            window_title = 'VLM Robot Action Viewer'
            cv2.imshow(window_title, display_image)
            cv2.waitKey(1)
        else:
            # Display a placeholder if no image received
            placeholder = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for VLM action images...", 
                       (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(placeholder, "Topic: /vlm_captioned_image", 
                       (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.putText(placeholder, "Action captions will appear on images", 
                       (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.imshow('VLM Robot Action Viewer', placeholder)
            cv2.waitKey(1)

def main():
    parser = argparse.ArgumentParser(description='Captioned Image Viewer')
    parser.add_argument('--save-images', action='store_true',
                       help='Save received images to disk')
    parser.add_argument('--save-dir', type=str, default='./captioned_images',
                       help='Directory to save images')
    
    args = parser.parse_args()
    
    if args.save_images:
        import os
        os.makedirs(args.save_dir, exist_ok=True)
        print(f"Images will be saved to: {args.save_dir}")
    
    if not ROS2_AVAILABLE:
        print("ROS2 not available. Please install ROS2 and required packages.")
        return
    
    try:
        rclpy.init()
        viewer = CaptionedImageViewer()
        
        print("VLM Robot Action Viewer started")
        print("Press 'q' to quit, 's' to save current image")
        print("Waiting for VLM action images...")
        
        import numpy as np
        import time
        
        while True:
            # Handle ROS2 callbacks
            rclpy.spin_once(viewer, timeout_sec=0.1)
            
            # Display image
            viewer.display_image()
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s') and viewer.latest_image is not None and args.save_images:
                timestamp = time.strftime('%Y%m%d_%H%M%S')
                save_path = os.path.join(args.save_dir, f"captioned_image_{timestamp}.jpg")
                cv2.imwrite(save_path, viewer.latest_image)
                print(f"Saved image to: {save_path}")
            
            time.sleep(0.1)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
        if ROS2_AVAILABLE:
            rclpy.shutdown()

if __name__ == '__main__':
    main()

