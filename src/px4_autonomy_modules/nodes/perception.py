import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from perception_msgs.msg import Detection


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )

        # Publisher to the perception detection topic
        self.publisher = self.create_publisher(
            Detection,
            '/perception/detection',
            10
        )

        self.get_logger().info('Perception Node started and listening to /video_source/raw')

    def image_callback(self, msg):
        # Model inference here

        # dummy values for now
        confidence = 0.0
        x_center = 0.0
        y_center = 0.0
        width = 0.0
        height = 0.0

        detection_msg = Detection()
        detection_msg.confidence = confidence
        detection_msg.x_center = x_center
        detection_msg.width = width
        detection_msg.height = height

        self.publisher.publish(detection_msg)
        self.get_logger().info(f'Published detection: conf={confidence}, x={x_center}, y={y_center}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
