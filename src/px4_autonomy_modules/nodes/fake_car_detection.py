#!/usr/bin/env python3
import numpy as np
import cv2
import tf2_ros
import rclpy
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from perception_msgs.msg import Detection
import rclpy
from rclpy.node import Node

class FakeCarDetection(Node):
    def __init__(self):
        """Initialize with camera parameters from OpenCV calibration"""
        super().__init__('fake_car_detection')
        
        # Camera intrinsic parameters
        self.image_width = 1280
        self.image_height = 720
        self.camera_matrix = np.array([[775.2993107832974, 0.0, 572.8916520968021], [0.0, 774.3546328379497, 385.7852440677227], [0.0, 0.0, 1.0]], dtype=np.float32)
        
        # Distortion coefficients
        self.distortion_coeffs = np.array([[-0.08866930295454323], [0.3412881145258634], [-0.6120691795041373], [-0.2083127547123781]], dtype=np.float32)
        
        # Set up TF2 for transform lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.perception_pose_pub = self.create_publisher(
            Detection,
            '/perception/detection',
            rclpy.qos.qos_profile_system_default
        )
        
        # Create a subscriber to the fake car pose
        self.fake_car_pose_sub = self.create_subscription(
            Odometry,
            '/fake_car/pose',
            self.publish_car_detection,
            rclpy.qos.qos_profile_system_default
        )
    
    
    def publish_car_detection(self, msg):
        """Publish a fake car detection message"""
        # Create a fake detection message
        detection_msg = Detection()
        
        # Project car position to image coordinates
        detection = self.project_car_to_image()
        if detection is not None:
            # Create a fake detection
            detection_msg.confidence = 1.0  # Fake confidence
            detection_msg.x_center = detection[0]
            detection_msg.y_center = detection[1]
            detection_msg.header.stamp = detection[2].to_msg()  # Timestamp from TF lookup
            detection_msg.width = 50.0  # Fake width
            detection_msg.height = 50.0  # Fake height
            
            # Publish the detection message
            self.perception_pose_pub.publish(detection_msg)
        else:
            return
    
    def project_car_to_image(self):
        """Project car position to image coordinates"""
        try:
            # Get transform from car to camera
            # This handles all the 3D coordinate transformations for you
            tf_timestamp = self.get_clock().now() - rclpy.time.Duration(seconds=0.5)
            transform = self.tf_buffer.lookup_transform(
                'camera_frame',         # Target frame (camera frame)
                'fake_car/base_link',   # Source frame (car frame)
                tf_timestamp,                   # Time to look up transform
                timeout=rclpy.duration.Duration(seconds=0.1)  # Timeout for lookup
            )
            
            # Extract car position in camera coordinates
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # For a downward-facing camera, check if car is in front of camera
            # The Z axis convention depends on your camera frame definition
            # Usually, Z points out of the camera's lens/sensor
            if z <= 0:
                return None  # Car is behind the camera plane
            
            # Create the object point (single point) in camera frame
            object_point = np.array([[x, y, z]], dtype=np.float32)
            
            # Use OpenCV to project the 3D point to 2D image plane
            # Since point is already in camera frame, rotation and translation are zero
            rvec = np.zeros(3, dtype=np.float32)  # No rotation needed
            tvec = np.zeros(3, dtype=np.float32)  # No translation needed
            
            img_points, _ = cv2.projectPoints(
                object_point, 
                rvec, 
                tvec, 
                self.camera_matrix, 
                self.distortion_coeffs
            )
            
            # Extract pixel coordinates
            u, v = img_points[0][0]
            
            # Check if point is within image bounds
            if 0 <= u < self.image_width and 0 <= v < self.image_height:
                return (float(u), float(v), tf_timestamp)
            else:
                return None  # Car is outside the camera's field of view
                
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF2 error: {e}")
            return None
def main():
    rclpy.init()
    node = FakeCarDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()