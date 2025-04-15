#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class FakeVicon(Node):
    def __init__(self):
        super().__init__('fake_vicon')
        
        # Publisher to the output topic
        self.publisher = self.create_publisher(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            10
        )
        
        self.timer = self.create_timer(1.0 / 50, self.timer_callback)

    def timer_callback(self):
        # Republish the PoseStamped message
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeVicon()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
