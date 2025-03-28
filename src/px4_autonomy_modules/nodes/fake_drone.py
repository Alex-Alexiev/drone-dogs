#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import threading
import time
import keyboard

class FakeDrone(Node):
    def __init__(self):
        super().__init__('fake_drone')
        
        # Publisher to the drone topic
        self.publisher = self.create_publisher(
            Odometry,
            'mavros/local_position/odom',
            rclpy.qos.qos_profile_system_default
        )
        
        # Initialize position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Start a thread to listen for keyboard input
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_control)
        self.keyboard_thread.start()

        # Timer to publish the drone's position at regular intervals
        self.timer = self.create_timer(0.1, self.publish_position)

    def keyboard_control(self):
        while self.running:
            try:
                if keyboard.is_pressed('w'):
                    self.y += 0.1
                if keyboard.is_pressed('s'):
                    self.y -= 0.1
                if keyboard.is_pressed('a'):
                    self.x -= 0.1
                if keyboard.is_pressed('d'):
                    self.x += 0.1
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Keyboard control error: {e}")

    def publish_position(self):
        # Create and publish an Odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        self.publisher.publish(msg)

    def destroy_node(self):
        self.running = False
        self.keyboard_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FakeDrone()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
