#!/usr/bin/env python3
import numpy as np
import tf2_ros
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from perception_msgs.msg import Detection
from nav_msgs.msg import Odometry

class CarPosePublisher(Node):
    def __init__(self):
        super().__init__('car_pose_publisher')
        
        # Camera intrinsic parameters - use the same values as in fake_car_detection.py
        self.image_width = 1280
        self.image_height = 720
        self.camera_matrix = np.array([[775.2993107832974, 0.0, 572.8916520968021], 
                                       [0.0, 774.3546328379497, 385.7852440677227], 
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        
        # Distortion coefficients
        self.distortion_coeffs = np.array([[-0.08866930295454323], [0.3412881145258634], 
                                          [-0.6120691795041373], [-0.2083127547123781]], 
                                          dtype=np.float32)
        
        # Set up TF2 for transform lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Assume car is at this height (in meters) from the ground
        self.car_height = 0.0  # Assuming car is on the ground at z=0
        
        # Create a publisher for the car pose
        self.car_pose_pub = self.create_publisher(
            Odometry,
            'car/pose',
            10
        )
        
        # Subscribe to detection messages
        self.detection_sub = self.create_subscription(
            Detection,
            '/perception/detection',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Detection to pose converter initialized')
    
    def detection_callback(self, msg):
        """Convert 2D detection to 3D pose using camera transform"""
        try:
            # Get the pixel coordinates
            u = msg.x_center
            v = msg.y_center
            
            # Create a ray from the camera center through this pixel
            # Use the inverse of the camera matrix to get the ray direction
            pixel = np.array([[u], [v], [1.0]])
            ray_camera = np.linalg.inv(self.camera_matrix) @ pixel
            
            # Normalize the ray
            ray_camera = ray_camera / np.linalg.norm(ray_camera)
            
            # Get the camera pose in world frame
            camera_transform = self.tf_buffer.lookup_transform(
                'map',             # Target frame (world frame)
                'camera_frame',     # Source frame (camera frame)
                rclpy.time.Time(),  # Get latest transform
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Extract camera position and orientation
            cam_pos = np.array([
                camera_transform.transform.translation.x,
                camera_transform.transform.translation.y,
                camera_transform.transform.translation.z
            ])
            
            # Convert quaternion to rotation matrix for the camera
            qx = camera_transform.transform.rotation.x
            qy = camera_transform.transform.rotation.y
            qz = camera_transform.transform.rotation.z
            qw = camera_transform.transform.rotation.w
            
            # Convert quaternion to rotation matrix
            rot_matrix = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
            
            # Transform ray direction from camera frame to world frame
            ray_world = rot_matrix @ ray_camera
            
            # Compute intersection with ground plane (z=self.car_height)
            # Ground plane is defined as z = self.car_height
            if abs(ray_world[2]) < 1e-6:
                self.get_logger().warn('Ray is parallel to ground plane, cannot compute intersection')
                return
                
            # Calculate distance along ray to ground plane
            t = (self.car_height - cam_pos[2]) / ray_world[2]
            
            # If t is negative, the intersection is behind the camera
            if t < 0:
                self.get_logger().warn('Ground plane intersection is behind the camera')
                return
                
            # Calculate intersection point
            intersection = cam_pos + t * ray_world.flatten()
            
            # Create and publish the odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = msg.header.stamp
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'car/base_link'
            
            # Set position
            odom_msg.pose.pose.position.x = float(intersection[0])
            odom_msg.pose.pose.position.y = float(intersection[1])
            odom_msg.pose.pose.position.z = float(intersection[2])
            
            # Set orientation (default to identity quaternion)
            odom_msg.pose.pose.orientation.w = 1.0
            
            # Set twist to zero (assuming static or updating only position)
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            
            # Publish the car pose
            self.car_pose_pub.publish(odom_msg)
            # self.get_logger().info(f'Published car position at x={intersection[0]:.2f}, y={intersection[1]:.2f}, z={intersection[2]:.2f}')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing detection: {e}')
    
    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """Convert quaternion to rotation matrix"""
        # Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        return np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
        ])

def main():
    rclpy.init()
    node = CarPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()