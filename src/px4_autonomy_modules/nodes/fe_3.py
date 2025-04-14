#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
import sys
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from copy import deepcopy 
import numpy as np
import math

# Constants
INIT_STATE = "Init"
LAUNCH_STATE = "Launch"
NAVIGATING_STATE = "Navigating"
LAND_STATE = "Land"

class CommNode(Node):

    def __init__(self):
        super().__init__(f'rob498_drone_07')
        drone_num = '7'
        self.get_logger().info(f'Drone Number: {drone_num}')

        self.initial_pose = None
        self.cur_pose = None
        self.desired_pose = None

        self.state = INIT_STATE
        self.waypoints = None
        self.waypoints_received = False
        self.current_waypoint_index = 0
        # TODO: Might want to reduce this threshold
        self.waypoint_reached_threshold = 0.3  # meters
        
        # Subscribers and Publishers
        self.mavros_pose_sub = self.create_subscription(
            PoseStamped, # TODO: This might be Odom for second part
            # '/mavros/vision_pose/pose', # TODO: Change this to mavros/local_position/odom for second part of fe_3
            'mavros/local_position/odom', 
            self.callback_mavros_pose,
            rclpy.qos.qos_profile_system_default
        )
        self.sub_waypoints = self.create_subscription(
            PoseArray, 
            f'rob498_drone_{drone_num}/comm/waypoints', 
            self.callback_waypoints, 
            10
        )

        self.mavros_pose_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        
        # Services
        self.srv_launch = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/abort', self.callback_abort)
        
        self.get_logger().info("Finished setting up services and subscribers/publishers")
        
        publish_freq = 20
        self.timer = self.create_timer(1 / publish_freq, self.pose_publisher_callback)

    # Callback handlers
    def handle_launch(self):
        self.get_logger().info('Launch Requested. Your drone should take off.')
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None, make sure vision_pose is being published to')
            return
        self.state = LAUNCH_STATE
        
        self.get_logger().info('Handling Launch')
        self.initial_pose = deepcopy(self.cur_pose)
        starting_pose = deepcopy(self.cur_pose)
        starting_pose.pose.position.z = 1.5
        starting_pose.pose.position.y = 0.0
        starting_pose.pose.position.x = 0.0
        self.desired_pose = deepcopy(starting_pose)
        
    def handle_test(self):
        self.get_logger().info('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        if not self.waypoints_received:
            self.get_logger().error('Waypoints not received yet, cannot start test')
            return
            
        self.state = NAVIGATING_STATE
        self.current_waypoint_index = 0

        self.navigate_to_next_waypoint()


    def handle_land(self):
        self.get_logger().info('Land Requested. Your drone should land.')
        if self.initial_pose is None:
            self.get_logger().error("Initial pose is None, aborting...")
            self.handle_abort()
            return
        self.desired_pose = deepcopy(self.initial_pose)
        self.state = LAND_STATE
        # TODO: need to return to initial pose?

    def handle_abort(self):
        self.get_logger().info('Abort Requested. Your drone should land immediately due to safety considerations')
        land_now = self.cur_pose
        land_now.pose.position.z = 0
        self.desired_pose = land_now
        
    # Service callbacks
    def callback_launch(self, request, response):
        self.handle_launch()
        return response

    def callback_test(self, request, response):
        self.handle_test()
        return response

    def callback_land(self, request, response):
        self.handle_land()
        return response

    def callback_abort(self, request, response):
        self.handle_abort()
        return response
    
    def callback_waypoints(self, msg):
        # Handle waypoints
        if self.waypoints_received:
            self.get_logger().warn('Waypoints already received, ignoring new waypoints')
            return
        self.get_logger().info('Waypoints received')
        self.waypoints_received = True
        self.waypoints = np.empty((0, 3))
        for pose in msg.poses:
            scale_down = 0.7
            scale_up = 1.1
            pos = np.array([pose.position.x*scale_up, pose.position.y*scale_up, pose.position.z - 0.2])
            scaled_pos = np.array([pose.position.x*scale_down, pose.position.y*scale_down, pose.position.z - 0.3])
            self.waypoints = np.vstack((self.waypoints, pos))
            self.waypoints = np.vstack((self.waypoints, scaled_pos))
        
        self.get_logger().info(f"================== Received {len(self.waypoints)} waypoints")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"Waypoint {i}: ({wp[0]}, {wp[1]}, {wp[2]})")
        
        if self.state != NAVIGATING_STATE:
            self.get_logger().info('Waiting for test command to start navigating')
        while self.state != NAVIGATING_STATE:
            rclpy.spin_once(self)
        self.navigate_to_next_waypoint()
    
    def callback_mavros_pose(self, msg):
        self.cur_pose = msg
        # self.cur_pose = PoseStamped()
        # self.cur_pose.pose = msg.pose.pose
        # self.cur_pose.header = msg.header

        pose = msg.pose
        self.get_logger().debug(f"Received Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
        
        # Check if we're in navigation mode and need to update waypoints
        if self.state == NAVIGATING_STATE and self.waypoints is not None and self.current_waypoint_index < len(self.waypoints):
            self.check_waypoint_reached()
        
    def navigate_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            return
            
        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"===================== Navigating to waypoint {self.current_waypoint_index}: ({waypoint[0]}, {waypoint[1]}, {waypoint[2]})")
        
        # Create a new pose message for the waypoint
        waypoint_pose = deepcopy(self.cur_pose)
        waypoint_pose.pose.position.x = float(waypoint[0])
        waypoint_pose.pose.position.y = float(waypoint[1])
        waypoint_pose.pose.position.z = float(waypoint[2])
        # ? Do we need to set orientation?
        
        self.desired_pose = waypoint_pose

    # def switch_waypoint(self):
    #     self.navigate_to_next_waypoint()
    #     # self.get_logger().info(f'2 seconds elapsed, navigating to next waypoint now') 
    #     # self.delay.cancel()
    #     # self.delay = None
    
    def check_waypoint_reached(self):
        if self.cur_pose is None or self.desired_pose is None:
            return
            
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance to current waypoint
        dx = self.cur_pose.pose.position.x - current_waypoint[0]
        dy = self.cur_pose.pose.position.y - current_waypoint[1]
        dz = self.cur_pose.pose.position.z - current_waypoint[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance < self.waypoint_reached_threshold:
            self.get_logger().info(f"==================== Reached waypoint {self.current_waypoint_index}")
            
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_next_waypoint()
                # self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached, delaying for 2 seconds...')
                # self.delay = self.create_timer(2.0, self.switch_waypoint)
            else:
                self.get_logger().info("All waypoints reached! Landing now...")
                self.state = LAND_STATE
                self.desired_pose = self.initial_pose
    
    def pose_publisher_callback(self):
        if self.desired_pose is None:
            self.desired_pose = self.cur_pose
            return
            
        # Attach timestamp to the desired pose
        self.desired_pose.header.stamp = self.get_clock().now().to_msg()
        self.mavros_pose_pub.publish(self.desired_pose)
        
        pose = self.desired_pose.pose
        # self.get_logger().info(f"Desired Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z}) "
                            #    f"Orientation: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
    
def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
