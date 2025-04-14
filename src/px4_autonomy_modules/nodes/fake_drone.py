#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.time import Time
import math

class FakeDrone(Node):
    def __init__(self):
        super().__init__('fake_drone')
        
        # Subscriber to the /mavros/setpoint_position/local topic
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.pose_callback,
            rclpy.qos.qos_profile_system_default
        )
        
        # Subscriber to the /mavros/setpoint_velocity/cmd_vel topic
        self.velocity_subscription = self.create_subscription(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            self.velocity_callback,
            rclpy.qos.qos_profile_system_default
        )
         
        # Publisher to the mavros/local_position/odom topic
        self.publisher = self.create_publisher(
            PoseStamped,
            'mavros/local_position/odom',
            rclpy.qos.qos_profile_system_default
        )
        
        self.current_pose = PoseStamped()
        self.target_pose = None
        self.target_velocity = None
        self.last_update_time = None
        self.speed = 1.0  # Speed of the drone in meters per second

        self.get_logger().info("Linearly interpolating position to the target and republishing to mavros/local_position/odom")

    def pose_callback(self, msg: PoseStamped):
        self.target_pose = msg
        
    def velocity_callback(self, msg: TwistStamped):
        self.target_velocity = msg

    def update_position(self):
        if self.target_pose is None:
            # Publish an initial pose if no target is set
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.position.z = 0.0
            initial_pose.pose.orientation.x = 0.0
            initial_pose.pose.orientation.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0  # No rotation
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            
            self.publisher.publish(initial_pose)
            return
        
        
        current_time = self.get_clock().now()
        if self.last_update_time is None:
            self.last_update_time = current_time
            return
        
        # Calculate time delta
        dt = (current_time - self.last_update_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        self.last_update_time = current_time

        if self.target_velocity is not None:
            # Compare timestamps to determine which target is newer
            velocity_time = Time.from_msg(self.target_velocity.header.stamp)
            pose_time = Time.from_msg(self.target_pose.header.stamp)
            
            if velocity_time > pose_time:
                # Update position based on velocity
                self.current_pose.pose.position.x += self.target_velocity.twist.linear.x * dt
                self.current_pose.pose.position.y += self.target_velocity.twist.linear.y * dt
                self.current_pose.pose.position.z += self.target_velocity.twist.linear.z * dt
                self.publish_pose()
                return

        # Calculate the distance to the target
        dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        dz = self.target_pose.pose.position.z - self.current_pose.pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # If the drone is close enough to the target, stop moving
        if distance < 0.01:  # Threshold for reaching the target
            self.current_pose.pose.position.x = self.target_pose.pose.position.x
            self.current_pose.pose.position.y = self.target_pose.pose.position.y
            self.current_pose.pose.position.z = self.target_pose.pose.position.z
            self.publish_pose()
            return

        # Calculate the direction vector
        direction_x = dx / distance
        direction_y = dy / distance
        direction_z = dz / distance

        # Move towards the target at the specified speed
        move_distance = min(self.speed * dt, distance)
        self.current_pose.pose.position.x += direction_x * move_distance
        self.current_pose.pose.position.y += direction_y * move_distance
        self.current_pose.pose.position.z += direction_z * move_distance

        # Publish the updated pose
        self.publish_pose()

    def publish_pose(self):      
        # Publish the pose stamped message
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = 'map'
        self.publisher.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    node = FakeDrone()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_position()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
