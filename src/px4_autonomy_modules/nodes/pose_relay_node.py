#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64

class PoseRelayNode(Node):
    def __init__(self):
        super().__init__('pose_relay_node')
        self.relay = False 
        self.vicon_to_cube_transform = [-0.103, 0.0, 0.0]
        
        # Subscription to the input topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.listener_callback,
            10
        )
        
        # Publisher to the output topic
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

    def listener_callback(self, msg):
        pose = PoseStamped()
        pose.pose = msg.pose
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "map"
        # self.get_logger().info(f"Received Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z}) "
        #                        f"Orientation: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
        
        if self.relay:
            # Apply transform to the pose
            pose.pose.position.x += self.vicon_to_cube_transform[0]
            pose.pose.position.y += self.vicon_to_cube_transform[1]
            pose.pose.position.z += self.vicon_to_cube_transform[2]
            
            self.get_logger().info(f"Vicon pose: x:{pose.pose.position.x}, y:{pose.pose.position.y}, z:{pose.pose.position.z}")
            self.publisher.publish(pose)
        # self.get_logger().info(f"Published Pose to output_pose")

def main(args=None):
    rclpy.init(args=args)
    node = PoseRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
