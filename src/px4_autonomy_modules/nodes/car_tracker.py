#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from perception_msgs.msg import Detection
from nav_msgs.msg import Odometry
from copy import deepcopy
import numpy as np
import math

# States
INIT_STATE = "Init"
LAUNCH_STATE = "Launch"
SEARCHING_STATE = "Searching"
CHASE_STATE = "Chase"
ESCAPED_BOUNDS_STATE = "EscapedBounds"
LAND_STATE = "Land"
ABORT_STATE = "Abort"
LOST_CAR_TIMEOUT = 5  # seconds

VEL_PUB_FREQ = 10  # Hz
STATE_UPDATE_FREQ = 10  # Hz

# Operating area
X_MAX = 4.0  # m
X_MIN = -4.0  # m
Y_MAX = 4.0  # m
Y_MIN = -4.0  # m
TARGET_Z = 2.0  # m
REACHED_WAYPOINT_THRESHOLD = 0.1  # m

# Safe bounding box
SAFE_X_MAX = 4.5  # m
SAFE_X_MIN = -4.5  # m
SAFE_Y_MAX = 4.5  # m
SAFE_Y_MIN = -4.5  # m
SAFE_Z_MAX = 2.5  # m
SAFE_Z_MIN = 0.0  # m

CAMERA_CENTER_X = 1280 / 2.0
CAMERA_CENTER_Y = 720 / 2.0

class CommNode(Node):

    def __init__(self):
        super().__init__('rob498_drone_07')
        drone_num = '7'
        self.get_logger().info(f'Drone Number: {drone_num}')

        self.initial_pose = None
        self.cur_pose = None
        self.desired_pose = None

        self.state = INIT_STATE

        # Subscribers and Publishers
        self.mavros_pose_sub = self.create_subscription(
            Odometry,
            'mavros/local_position/odom',
            self.callback_mavros_pose,
            rclpy.qos.qos_profile_system_default
        )

        self.vicon_pose_sub = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.listener_callback,
            10
        )

        self.state_pub = self.create_publisher(String, f'rob498_drone_{drone_num}/state', 10)

        self.mavros_pose_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.mavros_velocity_pub = self.create_publisher(TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 10)

        # Services
        self.srv_launch = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/launch', self.callback_launch)
        self.srv_start_search = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/start_search', self.callback_start_search)
        self.srv_land = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/abort', self.callback_abort)

        self.get_logger().info("Finished setting up services and subscribers/publishers")

        self.state_machine_timer = self.create_timer(1 / STATE_UPDATE_FREQ, self.state_machine_callback)

    def listener_callback(self, msg):
        """Callback for Vicon pose subscription."""
        pose = msg.pose

        # Check if the drone is outside the larger bounding box
        if (pose.position.x > SAFE_X_MAX or pose.position.x < SAFE_X_MIN or
                pose.position.y > SAFE_Y_MAX or pose.position.y < SAFE_Y_MIN or
                pose.position.z > SAFE_Z_MAX or pose.position.z < SAFE_Z_MIN):
            self.get_logger().warn(f"Vicon Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
            self.get_logger().warn("Drone is outside the safe bounding box. Aborting.")
            self.handle_abort()

    def handle_abort(self):
        self.get_logger().info('Abort Requested. Your drone should land immediately due to safety considerations')
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None')
            return
        land_now = deepcopy(self.cur_pose)
        land_now.pose.position.z = 0.0
        self.desired_pose = land_now
        self.state = ABORT_STATE

    def callback_mavros_pose(self, msg):
        self.cur_pose = PoseStamped()
        self.cur_pose.pose = msg.pose.pose
        self.cur_pose.header = msg.header

        pose = msg.pose.pose
        self.get_logger().debug(f"Received Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")

        # Check if we've escaped the bounds
        if pose.position.x > X_MAX or pose.position.x < X_MIN or pose.position.y > Y_MAX or pose.position.y < Y_MIN:
            self.state = ESCAPED_BOUNDS_STATE
            self.handle_escaped_bounds_state()

    def state_machine_callback(self):
        self.state_pub.publish(String(data=self.state))
        if self.state == INIT_STATE:
            if self.cur_pose is None:
                return
            self.desired_pose = self.cur_pose
            self.desired_pose.header.stamp = self.get_clock().now().to_msg()
            self.mavros_pose_pub.publish(self.desired_pose)
        elif self.state in [LAUNCH_STATE, ESCAPED_BOUNDS_STATE, LAND_STATE, ABORT_STATE]:
            if self.desired_pose is None:
                return
            self.desired_pose.header.stamp = self.get_clock().now().to_msg()
            self.mavros_pose_pub.publish(self.desired_pose)

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
