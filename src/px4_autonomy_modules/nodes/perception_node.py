#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort
from perception_msgs.msg import Detection
from vision_msgs.msg import Detection2DArray


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

        # Publisher to the perception detection topic
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/perception/detection',
            10
        )
        
        self.image_width = 1280
        self.image_height = 720
        self.model_width = 300
        self.model_height = 300
        
        self.get_logger().info('Perception Node Initialized (Detection2DArray → Detection converter)')

    def detection_callback(self, msg):
        try:
            # Early return if no detections
            if not msg.detections:
                return
            
            # Find detection with highest confidence
            best_detection = None
            highest_confidence = 0.0
            
            for detection in msg.detections:
                # Skip if no results
                if not detection.results:
                    continue
                    
                confidence = detection.results[0].score
                
                if confidence > highest_confidence:
                    highest_confidence = confidence
                    best_detection = detection
            
            # If we found a valid detection
            if best_detection is not None:
                # Extract bounding box info
                bbox = best_detection.bbox
                
                # Get center coordinates
                x_center = bbox.center.x
                y_center = bbox.center.y
                
                # Get width and height
                width = bbox.size_x
                height = bbox.size_y
                
                # Scale from 300x300 to 1280x720 if needed
                # x_scale = self.image_width / self.model_width
                # y_scale = self.image_height / self.model_height
                
                # x_center *= x_scale
                # y_center *= y_scale
                # width *= x_scale
                # height *= y_scale
                
                # Create and publish detection message
                detection_msg = Detection()
                detection_msg.header = msg.header
                detection_msg.confidence = float(highest_confidence)
                detection_msg.x_center = float(x_center)
                detection_msg.y_center = float(y_center)
                detection_msg.width = float(width)
                detection_msg.height = float(height)
                
                self.publisher.publish(detection_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in detection_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()