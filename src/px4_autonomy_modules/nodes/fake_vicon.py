#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

class FakeVicon(Node):
    def __init__(self):
        super().__init__('fake_vicon')
        
        # Publisher to the output topic
        self.publisher = self.create_publisher(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            10
        )
        
        publish_freq = 20 
        self.timer = self.create_timer(1 / publish_freq, self.publish_callback)

    def publish_callback(self):
        pose_stamped = PoseStamped()
        
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher.publish(pose_stamped)
        

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
