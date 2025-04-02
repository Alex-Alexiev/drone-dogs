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
        self.input_name = 'input_0'  # Hardcode this from the export script
        self.scores_output = 'scores'  # First output is scores
        self.boxes_output = 'boxes'   # Second output is boxes
        
        # SSD model normalization parameters (very important!)
        self.input_mean = np.array([127, 127, 127])  # RGB layout
        self.input_std = 128.0

        self.get_logger().info(f'Perception Node started and listening to /video_source/raw with model: {onnx_file_path}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort image
            DIM = (self.image_width, self.image_height)
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                self.camera_matrix, self.distortion_coeffs, np.eye(3), 
                self.camera_matrix, DIM, cv2.CV_16SC2
            )
            undistorted_img = cv2.remap(
                cv_image, map1, map2, 
                interpolation=cv2.INTER_LINEAR, 
                borderMode=cv2.BORDER_CONSTANT
            )
            
            # Resize to model input size
            resize_width = 300  # Match the export script
            resize_height = 300  # Match the export script
            input_image = cv2.resize(undistorted_img, (resize_width, resize_height))
            
            # Convert BGR to RGB
            input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
            
            # Normalize using the SSD model's expected mean and std
            input_image = input_image.astype(np.float32)
            input_image -= self.input_mean  # [127, 127, 127]
            input_image /= self.input_std   # 128.0
            
            # Transpose from HWC to CHW format
            input_image = np.transpose(input_image, (2, 0, 1))
            
            # Add batch dimension
            input_image = np.expand_dims(input_image, axis=0)
            
            # Run inference with correct output names
            outputs = self.session.run(
                [self.scores_output, self.boxes_output], 
                {self.input_name: input_image}
            )
            
            # Process model outputs
            scores = outputs[0]  # [batch, num_boxes, num_classes]
            boxes = outputs[1]   # [batch, num_boxes, 4]
            
            # Extract detections above threshold
            threshold = 0.2
            detections = []
            
            # Process each class
            for class_idx in range(1, scores.shape[2]):  # Skip background class 0
                class_scores = scores[0, :, class_idx]
                mask = class_scores > threshold
                
                if np.any(mask):
                    selected_boxes = boxes[0, mask, :]
                    selected_scores = class_scores[mask]
                    
                    for i in range(selected_boxes.shape[0]):
                        x1, y1, x2, y2 = selected_boxes[i]
                        
                        # Scale coordinates to original image size
                        x1 = x1 * resize_width * (self.image_width / resize_width)
                        y1 = y1 * resize_height * (self.image_height / resize_height)
                        x2 = x2 * resize_width * (self.image_width / resize_width)
                        y2 = y2 * resize_height * (self.image_height / resize_height)
                        
                        detections.append([x1, y1, x2, y2, selected_scores[i], class_idx])
            
            # If we found any detections
            if detections:
                # Sort by confidence and take the best one
                detections.sort(key=lambda x: x[4], reverse=True)
                x1, y1, x2, y2, confidence, class_id = detections[0]
                
                # Calculate center and dimensions
                x_center = (x1 + x2) / 2.0
                y_center = (y1 + y2) / 2.0
                width = abs(x2 - x1)
                height = abs(y2 - y1)
                
                # Create and publish detection message
                detection_msg = Detection()
                detection_msg.header = msg.header
                detection_msg.confidence = float(confidence)
                detection_msg.x_center = float(x_center)
                detection_msg.y_center = float(y_center)
                detection_msg.width = float(width)
                detection_msg.height = float(height)
                
                self.publisher.publish(detection_msg)
                self.get_logger().info(f'Published detection: conf={confidence:.2f}, x={x_center:.2f}, y={y_center:.2f}')
                
                # Draw bounding box on the image
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f'Conf: {confidence:.2f}', (int(x1), int(y1) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                self.get_logger().info('No detections found')
                
            # Publish visualization image
            detection_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            detection_image_msg.header = msg.header
            self.img_publisher.publish(detection_image_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

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
