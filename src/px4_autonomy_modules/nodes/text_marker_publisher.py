#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point

class TextMarkerPublisher(Node):
    def __init__(self):
        super().__init__('text_marker_publisher')
        
        drone_num = 7
        # Create a marker publisher
        self.marker_pub = self.create_publisher(
            Marker, 
            f'rob498_drone_{drone_num}/state/marker', 
            10
        )
        
        
        # Subscribe to the text content you want to display
        self.text_sub = self.create_subscription(
            String,
            f'rob498_drone_{drone_num}/state',
            self.text_callback,
            10
        )
        
        # Store the current state text
        self.current_state = "UNSET"
        
        # Create a timer to continuously publish the marker
        # (publishing at 10Hz ensures the marker is always visible)
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        self.get_logger().info('Text marker publisher started')
        
    def text_callback(self, msg):
        # Update the current state text
        self.current_state = msg.data
        self.get_logger().debug(f"Received new state: {self.current_state}")
    
    def publish_marker(self):
        # Create a text marker
        marker = Marker()
        marker.header.frame_id = "base_link"  # Use appropriate frame
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Set marker properties
        marker.ns = "text_markers"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Set position where text should appear
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25  # Adjust height as needed
        
        # Keep orientation as default quaternion (identity)
        marker.pose.orientation.w = 1.0
        
        # Set text content from stored state
        marker.text = self.current_state
        
        # Set the scale of the text (text height in meters)
        marker.scale.z = 0.2
        
        # Set color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # No lifetime - marker will be continuously updated
        # If you want it to disappear if publisher stops, set a short lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5 seconds
        
        # Publish the marker
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TextMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()