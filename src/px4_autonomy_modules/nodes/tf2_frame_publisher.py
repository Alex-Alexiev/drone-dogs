#!/usr/bin/env python3
import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry

class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.mavros_pose_sub = self.create_subscription(
            Odometry,
            'mavros/local_position/odom', 
            self.callback_mavros_pose,
            rclpy.qos.qos_profile_system_default
        )

    def callback_mavros_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id # world
        t.child_frame_id = msg.child_frame_id # base_link

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()