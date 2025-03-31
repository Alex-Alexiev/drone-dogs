#!/usr/bin/env python3
import threading
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

import sys
import select
import termios
import tty

POSE_REFRESH_RATE = 20.0  # Hz

class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a publisher for the fake car pose
        self.fake_car_pose_pub = self.create_publisher(
            Odometry,
            'fake_car/pose', 
            rclpy.qos.qos_profile_system_default
        )

        # Initialize the current fake car pose
        self.cur_fake_car_pose = Odometry()
        self.cur_fake_car_pose.header.frame_id = 'map'
        self.cur_fake_car_pose.child_frame_id = 'fake_car/base_link'
        self.cur_fake_car_pose.pose.pose.position.x = 0.0
        self.cur_fake_car_pose.pose.pose.position.y = 0.0
        self.cur_fake_car_pose.pose.pose.position.z = 0.0
        self.cur_fake_car_pose.pose.pose.orientation.x = 0.0
        self.cur_fake_car_pose.pose.pose.orientation.y = 0.0
        self.cur_fake_car_pose.pose.pose.orientation.z = 0.0
        self.cur_fake_car_pose.pose.pose.orientation.w = 1.0

        # Car kinematics parameters
        self.wheelbase = 1.0  # Distance between front and rear axles
        self.linear_velocity = 0.0  # Forward/backward velocity
        self.steering_angle = 0.0  # Steering angle in radians
        self.max_steering_angle = np.radians(30)  # Max steering angle

        # Set up key bindings
        self.key_bindings = {
            'w': 0.1,  # Increase forward velocity
            's': -0.1, # Decrease forward velocity
            'a': 0.1,  # Increase steering angle
            'd': -0.1  # Decrease steering angle
        }

        # Create a timer to periodically publish the fake car pose and transform
        self.timer = self.create_timer(1.0 / POSE_REFRESH_RATE, self.update_and_publish)

        # Start teleoperation keyboard thread
        self.teleop_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.teleop_thread.start()

    def update_and_publish(self):
        # Update the car's position and orientation using the kinematic model
        dt = 0.1  # Time step (same as timer period)
        theta = self.get_yaw_from_orientation(self.cur_fake_car_pose.pose.pose.orientation)

        # Update position
        x = self.cur_fake_car_pose.pose.pose.position.x
        y = self.cur_fake_car_pose.pose.pose.position.y
        x += self.linear_velocity * np.cos(theta) * dt
        y += self.linear_velocity * np.sin(theta) * dt

        # Update orientation
        theta += (self.linear_velocity / self.wheelbase) * np.tan(self.steering_angle) * dt

        # Normalize theta to [-pi, pi]
        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        # Update the pose
        self.cur_fake_car_pose.pose.pose.position.x = x
        self.cur_fake_car_pose.pose.pose.position.y = y
        self.cur_fake_car_pose.pose.pose.orientation = self.get_orientation_from_yaw(theta)

        # Update the header
        self.cur_fake_car_pose.header.stamp = self.get_clock().now().to_msg()
        self.cur_fake_car_pose.header.frame_id = 'map'
        self.cur_fake_car_pose.child_frame_id = 'fake_car/base_link'
        
        # Publish the fake car pose
        self.fake_car_pose_pub.publish(self.cur_fake_car_pose)

        # Create and populate the TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.cur_fake_car_pose.header.stamp
        t.header.frame_id = self.cur_fake_car_pose.header.frame_id
        t.child_frame_id = self.cur_fake_car_pose.child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = self.cur_fake_car_pose.pose.pose.position.z
        t.transform.rotation = self.cur_fake_car_pose.pose.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def keyboard_loop(self):
        """Thread loop to capture key presses and update velocity and steering angle."""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key in self.key_bindings:
                        if key in ['w', 's']:
                            self.linear_velocity += self.key_bindings[key]
                        elif key in ['a', 'd']:
                            self.steering_angle += self.key_bindings[key]
                            self.steering_angle = max(-self.max_steering_angle, min(self.steering_angle, self.max_steering_angle))
                        self.get_logger().info(f"Velocity: {self.linear_velocity:.2f}, Steering Angle: {np.degrees(self.steering_angle):.2f}°")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    @staticmethod
    def get_yaw_from_orientation(orientation):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    @staticmethod
    def get_orientation_from_yaw(yaw):
        """Convert yaw angle to quaternion."""
        half_yaw = yaw / 2.0
        return Quaternion(
            x=0.0,
            y=0.0,
            z=np.sin(half_yaw),
            w=np.cos(half_yaw)
        )

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.teleop_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()