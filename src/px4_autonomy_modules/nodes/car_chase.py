#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from perception_msgs.msg import Detection
import numpy as np
from copy import deepcopy

# Constants for position control
LOST_CAR_TIMEOUT = 5.0          # seconds
CAMERA_CENTER_X = 1280.0 / 2.0    # Camera image center x
CAMERA_CENTER_Y = 720.0 / 2.0     # Camera image center y
POSITION_CONTROL_STEP = 1e-3      # Gain to scale the delta pose step
STATE_UPDATE_FREQ = 10          # Hz

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_07')
        self.get_logger().info("Starting Chase Test Node using position control.")
        
        self.cur_pose = None
        self.cur_detected_car = None

        # Publisher for position setpoints
        self.mavros_position_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        
        # Subscribers for drone pose and detected car detection
        self.pose_sub = self.create_subscription(
            Odometry,
            'mavros/local_position/odom',
            self.pose_callback,
            10)
        
        self.detection_sub = self.create_subscription(
            Detection,
            '/perception/detection',
            self.detection_callback,
            10)
        
        # Timer to run the chase state logic periodically
        self.timer = self.create_timer(1.0 / STATE_UPDATE_FREQ, self.timer_callback)

    def pose_callback(self, msg):
        # Convert the incoming Odometry message into a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.cur_pose = pose_stamped

    def detection_callback(self, msg):
        self.get_logger().info(f"Detection received: x_center={msg.x_center}, y_center={msg.y_center}")
        self.cur_detected_car = msg

    def timer_callback(self):
        self.handle_chase_state()

    def handle_chase_state(self):
        if self.cur_pose is None:
            self.get_logger().error('Current pose is not available.')
            return
        if self.cur_detected_car is None:
            self.get_logger().error('No detection data available.')
            return

        # Check if the detection message is fresh; if not, do not update the setpoint
        current_time = self.get_clock().now()
        detection_time = rclpy.time.Time.from_msg(self.cur_detected_car.header.stamp)
        elapsed = current_time - detection_time
        seconds_since_detection = elapsed.nanoseconds / 1e9
        if seconds_since_detection > LOST_CAR_TIMEOUT:
            self.get_logger().warn('Lost the detected car. Not updating position setpoint.')
            return

        # Compute error vector in pixel coordinates (difference from camera center)
        car_px_dist_x = self.cur_detected_car.x_center - CAMERA_CENTER_X
        car_px_dist_y = self.cur_detected_car.y_center - CAMERA_CENTER_Y

        # Rotate the pixel error vector by 180 degrees to align with the drone's frame
        theta = np.radians(180.0)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rotated_x = car_px_dist_x * cos_theta - car_px_dist_y * sin_theta
        rotated_y = car_px_dist_x * sin_theta + car_px_dist_y * cos_theta

        # Create a new setpoint pose by adding a small delta in the direction of the error vector.
        # Note: The sign here reflects the original control law; adjust if your coordinate frames differ.
        new_pose = deepcopy(self.cur_pose)
        new_pose.pose.position.x += -POSITION_CONTROL_STEP * rotated_x
        new_pose.pose.position.y += -POSITION_CONTROL_STEP * rotated_y
        # Optionally, maintain the current z position or set it to a desired value
        # new_pose.pose.position.z remains unchanged

        # Update the header timestamp for the new setpoint
        new_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Publishing new setpoint: x={new_pose.pose.position.x:.3f}, y={new_pose.pose.position.y:.3f}")
        self.mavros_position_pub.publish(new_pose)

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()