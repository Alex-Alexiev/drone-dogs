#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageRepublisher(Node):
    def __init__(self):
        super().__init__('camera_republisher_node')

        self.publisher = self.create_publisher(Image, '/IMX219/image', 10)

        self.get_logger().info("Initializing Camera Republisher...")
        self.bridge = CvBridge()
        gst_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! appsink"
        )

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('Camera not accessible!')
            rclpy.shutdown()
            return

        # ret, frame = self.cap.read()
        # if ret:
        #     cv2.imwrite("test_img.png", frame)
        #     self.get_logger().info("Saved test frame")
        # else:
        #     self.get_logger().info("Failed to save test frame")
        
        self.get_logger().info("Camera Found, Starting Frame Capture.")

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Loop")
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture image')
            return
    
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error converting or publishing image: {e}')
    
    def destroy_node(self):
        self.cap.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

