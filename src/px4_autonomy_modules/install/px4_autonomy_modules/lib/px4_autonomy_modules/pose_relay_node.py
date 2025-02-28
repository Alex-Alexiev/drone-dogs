import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

class PoseRelayNode(Node):
    def __init__(self):
        super().__init__('pose_relay_node')
        
        # Subscription to the input topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.listener_callback,
            10
        )
        self.subscription
        
        # Publisher to the output topic
        self.publisher = self.create_publisher(
            Pose,
            '/mavros/vision_pose/pose',
            10
        )

    def listener_callback(self, msg):
        msg = msg.pose
        self.get_logger().info(f"Received Pose - Position: ({msg.position.x}, {msg.position.y}, {msg.position.z}) "
                               f"Orientation: ({msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w})")
        
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Published Pose to output_pose")

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
