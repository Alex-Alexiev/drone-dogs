#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from perception_msgs.msg import Detection
from std_srvs.srv import Trigger
from copy import deepcopy
import numpy as np

# Constants for position control and state management
LOST_CAR_TIMEOUT = 5.0          # seconds
CAMERA_CENTER_X = 1280.0 / 2.0    # Camera image center x
CAMERA_CENTER_Y = 720.0 / 2.0     # Camera image center y
POSITION_CONTROL_STEP = 1e-3      # Gain to scale the delta pose step
STATE_UPDATE_FREQ = 10          # Hz
TARGET_Z = 2.5                  # Launch altitude

# Minimal state definitions
INIT_STATE   = "Init"
LAUNCH_STATE = "Launch"
CHASE_STATE  = "Chase"

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_07')
        self.get_logger().info("Starting Chase Node using position control with launch command.")

        # Pose, detection and state
        self.cur_pose = None
        self.cur_detected_car = None
        self.desired_pose = None
        self.state = INIT_STATE

        # Publisher to send position setpoints
        self.mavros_position_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.rotated_error_pub = self.create_publisher(PointStamped, 'rotated_error_point', 10)
        # Subscribers for the drone's current pose and detection messages
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

        # Timer to run the state machine at a fixed rate
        self.timer = self.create_timer(1.0 / STATE_UPDATE_FREQ, self.timer_callback)

        # Service to handle the launch command
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_07/comm/launch', self.callback_launch)

    def pose_callback(self, msg):
        # Convert Odometry to PoseStamped for consistency
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.cur_pose = pose_stamped

    def detection_callback(self, msg):
        self.get_logger().info(f"Detection received: x_center={msg.x_center}, y_center={msg.y_center}")
        self.get_logger().info(f"State is now: {self.state}")
        self.cur_detected_car = msg
        # If in launch state, switch to chase mode when a detection is received
        if self.state == LAUNCH_STATE:
            self.get_logger().info("Detection received during LAUNCH. Switching to CHASE mode.")
            self.state = CHASE_STATE

    def callback_launch(self, request, response):
        self.handle_launch()
        response.success = True
        response.message = "Launch command executed: Taking off to (0,0,2.5)."
        return response

    def handle_launch(self):
        self.get_logger().info("Launch command received. Setting desired pose to (0, 0, 2.5).")
        if self.cur_pose is None:
            self.get_logger().error("Current pose is not available. Cannot execute launch.")
            return

        self.state = LAUNCH_STATE
        # Optionally record the initial pose
        self.initial_pose = deepcopy(self.cur_pose)

        # Create a launch pose with x=0, y=0, z=TARGET_Z
        launch_pose = deepcopy(self.cur_pose)
        launch_pose.pose.position.x = 0.0
        launch_pose.pose.position.y = 0.0
        launch_pose.pose.position.z = TARGET_Z
        self.desired_pose = launch_pose

        # Publish the launch setpoint immediately
        launch_pose.header.stamp = self.get_clock().now().to_msg()
        self.mavros_position_pub.publish(launch_pose)

    def timer_callback(self):
        # In launch mode, repeatedly publish the launch setpoint
        self.get_logger().info("Timer")
        if self.state == LAUNCH_STATE:
            if self.desired_pose is not None:
                self.desired_pose.header.stamp = self.get_clock().now().to_msg()
                self.mavros_position_pub.publish(self.desired_pose)
            return
        # In chase mode, update the setpoint toward the detected target
        elif self.state == CHASE_STATE:
            self.handle_chase_state()
        else:
            # In INIT state, if a current pose exists, continue publishing it as the setpoint
            if self.cur_pose is not None:
                current_setpoint = deepcopy(self.cur_pose)
                current_setpoint.header.stamp = self.get_clock().now().to_msg()
                self.mavros_position_pub.publish(current_setpoint)

    def handle_chase_state(self):
        if self.cur_pose is None:
            self.get_logger().error("Current pose is not available.")
            return
        if self.cur_detected_car is None:
            self.get_logger().error("No detection data available for chase.")
            return

        # Check if the detection is too old
        current_time = self.get_clock().now()
        detection_time = rclpy.time.Time.from_msg(self.cur_detected_car.header.stamp)
        elapsed = current_time - detection_time
        seconds_since_detection = elapsed.nanoseconds / 1e9
        if seconds_since_detection > LOST_CAR_TIMEOUT:
            self.get_logger().warn("Lost the detected car. Not updating setpoint.")
            return

        # Compute error vector (in pixels) between detected car and camera center
        car_px_dist_x = self.cur_detected_car.x_center - CAMERA_CENTER_X
        car_px_dist_y = self.cur_detected_car.y_center - CAMERA_CENTER_Y

        # Rotate the error vector by 180 degrees (for frame alignment)
        theta = np.radians(180.0)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rotated_x = car_px_dist_x * cos_theta - car_px_dist_y * sin_theta
        rotated_y = car_px_dist_x * sin_theta + car_px_dist_y * cos_theta

        # Create a new setpoint by adding a small delta
        new_pose = deepcopy(self.cur_pose)
        new_pose.pose.position.x += -POSITION_CONTROL_STEP * rotated_x
        new_pose.pose.position.y += -POSITION_CONTROL_STEP * rotated_y

        new_pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Chase setpoint: x={new_pose.pose.position.x:.3f}, y={new_pose.pose.position.y:.3f}")
        self.mavros_position_pub.publish(new_pose)

        # Publish the rotated vector as a PointStamped for RViz
        error_point = PointStamped()
        error_point.header.stamp = self.get_clock().now().to_msg()
        error_point.header.frame_id = "map"
        error_point.point.x = rotated_x
        error_point.point.y = rotated_y
        error_point.point.z = self.cur_pose.pose.position.z 

        self.rotated_error_pub.publish(error_point)

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()