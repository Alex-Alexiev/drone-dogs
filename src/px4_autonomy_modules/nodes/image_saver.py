#!/usr/bin/env python3

import os
import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        
        # Get parameters
        self.save_dir = "./images"
        self.max_images = 1000
        
        # Create save directory if it doesn't exist
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Initialize CV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Counter for saved images
        self.image_count = 0
        
        # QoS settings for image subscription
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscription to the video source
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            qos
        )
        
        self.get_logger().info(f"Image saver initialized. Saving images to: {self.save_dir}")

    def image_callback(self, msg):
        # Stop if we've reached max images
        if self.image_count >= self.max_images:
            return
            
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Create filename with timestamp and counter
            filename = f"{self.save_dir}/image_{self.image_count:04d}.jpg"
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            
            # Increment counter
            self.image_count += 1
            
            # Log every 10 images
            if self.image_count % 10 == 0:
                self.get_logger().info(f"Saved {self.image_count} images")
                
        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Image saving stopped. Total images saved: {node.image_count}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()