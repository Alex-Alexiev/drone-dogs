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
        
        # Subscriber to the MAVROS odometry topic
        self.mavros_pose_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/odom', 
            self.callback_mavros_pose,
            rclpy.qos.qos_profile_system_default
        )

    def callback_mavros_pose(self, msg: Odometry):      
        # Republish the PoseStamped message
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
