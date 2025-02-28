#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry

class CameraPoseRelayNode(Node):
    def __init__(self):
        super().__init__('camera_pose_relay_node')
        
        # Subscription to the input topic
        self.subscription = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.listener_callback,
            rclpy.qos.qos_profile_system_default
        )
        
        # Publisher to the output topic
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        

    def listener_callback(self, msg):
        pose = msg.pose.pose
        relayed_msg = PoseStamped()
        relayed_msg.header = msg.header
        relayed_msg.pose = msg.pose.pose
        # self.get_logger().info(f"Received Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z}) "
        #                        f"Orientation: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
        
        self.publisher.publish(relayed_msg)
        # self.get_logger().info(f"Published Pose to output_pose")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
