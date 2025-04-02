#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
from perception_msgs.msg import Detection


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Declare and get the ONNX model file path parameter
        self.declare_parameter('onnx_file_path', 'your_model.onnx')
        onnx_file_path = self.get_parameter('onnx_file_path').get_parameter_value().string_value

        # Subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/video_source_old/raw',
            self.image_callback,
            10
        )

        # Publisher to the perception detection topic
        self.publisher = self.create_publisher(
            Detection,
            '/perception/detection',
            10
        )
        
        self.img_publisher = self.create_publisher(
            Image,
            '/perception/detection/img',
            10
        )
        
        self.image_width = 1280
        self.image_height = 720
        self.camera_matrix = np.array([[775.2993107832974, 0.0, 572.8916520968021], 
                                       [0.0, 774.3546328379497, 385.7852440677227], 
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        
        # Distortion coefficients
        self.distortion_coeffs = np.array([[-0.08866930295454323], [0.3412881145258634], 
                                          [-0.6120691795041373], [-0.2083127547123781]], 
                                          dtype=np.float32)
        
        self.bridge = CvBridge()
        
        # Load the ONNX model
        self.session = ort.InferenceSession(onnx_file_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        self.get_logger().info(f'Perception Node started and listening to /video_source/raw with model: {onnx_file_path}')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # use the calibration results to undistort an image
        DIM = (self.image_width, self.image_height)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.camera_matrix, self.distortion_coeffs, np.eye(3), self.camera_matrix, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        
        resize_width = 300
        resize_height = 300
        input_image = cv2.resize(undistorted_img, (resize_width, resize_height))
        input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
        input_image = input_image.astype(np.float32) / 255.0  # Normalization
        input_image = np.transpose(input_image, (2, 0, 1))  # (H, W, C) -> (C, H, W)
        input_image = np.expand_dims(input_image, axis=0)
        
        # Run inference
        outputs = self.session.run([self.output_name], {self.input_name: input_image})[0]
        
        # Check if we have any detections
        if len(outputs) == 0:
            self.get_logger().info('No detections found')
            return
        
        # sort the outputs based on confidence
        outputs = sorted(outputs, key=lambda x: x[4], reverse=True)
        # Assuming the first output is the most confident detection
        x1, y1, x2, y2, confidence, class_id = outputs[0]
        
        # Scale coordinates back to original image size
        x_scale = self.image_width / resize_width
        y_scale = self.image_height / resize_height
        
        x1 *= x_scale
        y1 *= y_scale
        x2 *= x_scale
        y2 *= y_scale
        
        x_center = (x1 + x2) / 2.0
        y_center = (y1 + y2) / 2.0
        width = abs(x2 - x1)
        height = abs(y2 - y1)

        detection_msg = Detection()
        detection_msg.header = msg.header
        detection_msg.confidence = float(confidence)
        detection_msg.x_center = float(x_center)
        detection_msg.y_center = float(y_center)
        detection_msg.width = float(width)
        detection_msg.height = float(height)

        self.publisher.publish(detection_msg)
        self.get_logger().info(f'Published detection: conf={confidence}, x={x_center}, y={y_center}')

        # Draw bounding box on the image (for visualization)
        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(cv_image, f'Conf: {confidence:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Convert OpenCV image back to ROS Image message
        detection_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        detection_image_msg.header = msg.header
        detection_image_msg.header.frame_id = 'camera_frame'
        detection_image_msg.header.stamp = msg.header.stamp
        
        # Publish the detection image
        self.img_publisher.publish(detection_image_msg)

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
