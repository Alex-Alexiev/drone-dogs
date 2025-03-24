#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_republisher_node')
    publisher = node.create_publisher(Image, '/IMX219/image', 10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        node.get_logger().error('Camera not accessible!')
        rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                node.get_logger().warn('Failed to capture image')
                continue

            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            publisher.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()
