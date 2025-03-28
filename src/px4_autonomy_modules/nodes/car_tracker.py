#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
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
LOST_CAR_TIMEOUT = 5 # seconds

VEL_PUB_FREQ = 10 # Hz
STATE_UPDATE_FREQ = 10 # Hz

# Operating area
X_MAX = 4 # m
X_MIN = -4 # m
Y_MAX = 4 # m
Y_MIN = -4 # m
TARGET_Z = 2 # m
REACHED_WAYPOINT_THRESHOLD = 0.1 # m

CAMERA_CENTER_X = 1280 / 2.0
CAMERA_CENTER_Y = 720 / 2.0

# PID control parameters
# ? Should these be different for x and y?
VELOCITY_CONTROL_P = 1e-3
VELOCITY_CONTROL_I = 1.0
VELOCITY_CONTROL_D = 1.0

# Search grid parameters
SEARCH_GRID_SIZE_X = 1 # m
SEARCH_GRID_SIZE_Y = 1 # m

class CommNode(Node):

    def __init__(self):
        super().__init__(f'rob498_drone_07')
        drone_num = '7'
        self.get_logger().info(f'Drone Number: {drone_num}')

        self.initial_pose = None
        self.cur_pose = None
        self.desired_pose = None
        
        # PID variables
        self.cur_detected_car_position = None
        self.last_car_px_dist_x = None
        self.last_car_px_dist_y = None
        self.integral_term_x = 0.0
        self.integral_term_y = 0.0

        self.state = INIT_STATE
        self.grid_position_x = 0
        self.grid_position_y = 0
        
        # Subscribers and Publishers
        self.mavros_pose_sub = self.create_subscription(
            Odometry,
            'mavros/local_position/odom', 
            self.callback_mavros_pose,
            rclpy.qos.qos_profile_system_default
        )
        
        self.perception_pose_sub = self.create_subscription(
            Detection,
            '/perception/detection',
            self.callback_detected_car_position,
            rclpy.qos.qos_profile_system_default
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
        self.last_acquired_car_time = None

    # Callback handlers
    def handle_launch(self):
        self.get_logger().info('Launch Requested. Taking off.')
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None')
            return
        self.state = LAUNCH_STATE
        
        self.get_logger().info('Handling Launch')
        self.initial_pose = deepcopy(self.cur_pose)
        starting_pose = deepcopy(self.cur_pose)
        starting_pose.pose.position.z = TARGET_Z
        starting_pose.pose.position.y = 0.0
        starting_pose.pose.position.x = 0.0
        self.desired_pose = deepcopy(starting_pose)
        
    def handle_start_search(self):
        self.get_logger().info('Starting search.')
            
        self.state = CHASE_STATE

    def handle_land(self):
        self.get_logger().info('Land Requested. Your drone should land.')
        if self.initial_pose is None:
            self.get_logger().error("Initial pose is None, aborting...")
            self.handle_abort()
            return
        self.desired_pose = deepcopy(self.initial_pose)
        self.state = LAND_STATE

    def handle_abort(self):
        self.get_logger().info('Abort Requested. Your drone should land immediately due to safety considerations')
        land_now = self.cur_pose
        land_now.pose.position.z = 0
        self.desired_pose = land_now
        self.state = ABORT_STATE
        
    # Service callbacks
    def callback_launch(self, request, response):
        self.handle_launch()
        response.success = True
        response.message = "Taking off now..."
        return response

    def callback_start_search(self, request, response):
        self.handle_start_search()
        response.success = True
        response.message = "Starting search now..."
        return response

    def callback_land(self, request, response):
        self.handle_land()
        response.success = True
        response.message = "Landing now..."
        return response

    def callback_abort(self, request, response):
        self.handle_abort()
        response.success = True
        response.message = "Aborting now..."
        return response
    
    def callback_mavros_pose(self, msg):
        # self.cur_pose = msg
        self.cur_pose = PoseStamped()
        self.cur_pose.pose = msg.pose.pose
        self.cur_pose.header = msg.header

        pose = msg.pose.pose
        self.get_logger().debug(f"Received Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
        
        # Check if we've escaped the bounds
        if pose.position.x > X_MAX or pose.position.x < X_MIN or pose.position.y > Y_MAX or pose.position.y < Y_MIN:
            self.state = ESCAPED_BOUNDS_STATE
            self.handle_escaped_bounds_state()
        elif self.state == ESCAPED_BOUNDS_STATE:
            self.get_logger().info('Back in bounds. Resuming search pattern.')
            # Go back to searching starting from the current position
            self.grid_position_x, self.grid_position_y = self.get_closest_search_grid_point()
            self.state = SEARCHING_STATE
            
    def callback_detected_car_position(self, msg):
        self.get_logger().info(f"Detected car at ({msg.x_center}, {msg.y_center})")
        self.last_acquired_car_time = self.get_clock().now()
        self.cur_detected_car_position = msg
        if self.state == SEARCHING_STATE:
            self.get_logger().info('Detected car while searching. Switching to chase mode.')
            self.last_car_px_dist = None
        self.state = CHASE_STATE
    
    def get_search_grid(self):
        x = np.arange(X_MIN, X_MAX, SEARCH_GRID_SIZE_X)
        y = np.arange(Y_MIN, Y_MAX, SEARCH_GRID_SIZE_Y)
        X, Y = np.meshgrid(x, y)
        X = X.flatten()
        Y = Y.flatten()
        return X, Y
    
    def get_closest_search_grid_point(self):
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None')
            return
        X, Y = self.get_search_grid()
        # Use the current pose not the current grid position
        dists = np.sqrt((self.cur_pose.pose.position.x - X)**2 + (self.cur_pose.pose.position.y - Y)**2)
        min_dist_idx = np.argmin(dists)
        return X[min_dist_idx], Y[min_dist_idx]
    
    def handle_searching_state(self):
        self.get_logger().info('Searching for the car')
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None')
            return
        self.state = SEARCHING_STATE
        
        # Search grid
        X, Y = self.get_search_grid()
        
        # If we reached the current grid point, move to the next one
        dist_to_desired_grid_point = math.sqrt((self.cur_pose.pose.position.x - X[self.grid_position_x])**2 + (self.cur_pose.pose.position.y - Y[self.grid_position_y])**2)
        if dist_to_desired_grid_point < REACHED_WAYPOINT_THRESHOLD:
            self.grid_position_x += 1
            if self.grid_position_x >= len(X):
                self.grid_position_x = 0
                self.grid_position_y += 1
                if self.grid_position_y >= len(Y):
                    self.grid_position_y = 0
            self.get_logger().info(f"Moving to next grid point: ({X[self.grid_position_x]}, {Y[self.grid_position_y]})")
        
        # Set the desired pose to the first point in the grid
        self.desired_pose = deepcopy(self.cur_pose)
        self.desired_pose.pose.position.x = X[self.grid_position_x]
        self.desired_pose.pose.position.y = Y[self.grid_position_y]
        self.desired_pose.pose.position.z = TARGET_Z
        
    def handle_escaped_bounds_state(self):
        self.get_logger().info('Escaped bounds. Going back to nearest in-bounds point.')
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None')
            return
        desired_x_position = np.clip(self.cur_pose.pose.position.x, X_MIN, X_MAX)
        desired_x_position -= np.sign(self.cur_pose.pose.position.x) * REACHED_WAYPOINT_THRESHOLD
        desired_y_position = np.clip(self.cur_pose.pose.position.y, Y_MIN, Y_MAX)
        desired_y_position -= np.sign(self.cur_pose.pose.position.y) * REACHED_WAYPOINT_THRESHOLD
        
        self.desired_pose = deepcopy(self.cur_pose)
        self.desired_pose.pose.position.x = desired_x_position
        self.desired_pose.pose.position.y = desired_y_position
        self.desired_pose.pose.position.z = TARGET_Z
        self.get_logger().info(f"Desired Inbounds pose: ({self.desired_pose.pose.position.x}, {self.desired_pose.pose.position.y}, {self.desired_pose.pose.position.z})")
        
        
    def handle_chase_state(self):
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None')
            return
        if self.cur_detected_car_position is None:
            self.get_logger().error('last_detected_car_position is None')
            return
        if self.last_acquired_car_time is None:
            self.get_logger().error('last_acquired_car_time is None')
            return
        if (self.get_clock().now() - self.last_acquired_car_time).seconds > LOST_CAR_TIMEOUT:
            self.get_logger().info('Lost the car. Going back to searching.')
            self.state = SEARCHING_STATE
            return

        # Calculate the pixel distance of the car from the center of the camera
        # TODO: Apply a transformation to align the pixel coordinates with the drone's frame of reference
        car_px_dist_x = CAMERA_CENTER_X - self.cur_detected_car_position.x_center
        car_px_dist_y = CAMERA_CENTER_Y - self.cur_detected_car_position.y_center
        self.get_logger().info(f"Car pixel distance from center: {car_px_dist_x}, {car_px_dist_y}")
        
        velocity_control = TwistStamped()
        velocity_control.header = self.cur_pose.header
        # Calculate the velocity control
        if self.cur_detected_car_position is not None:
            # PID control
            # if self.last_car_px_dist_x is not None and self.last_car_px_dist_y is not None:
            #     Calculate the derivative term
            #     derivative_x = (car_px_dist_x - self.last_car_px_dist_x) * VEL_PUB_FREQ
            #     derivative_y = (car_px_dist_y - self.last_car_px_dist_y) * VEL_PUB_FREQ
            # else:
            #     derivative_x = 0.0
            #     derivative_y = 0.0
                
            # Calculate the integral term
            # self.integral_term_x += car_px_dist_x * VEL_PUB_FREQ
            # self.integral_term_y += car_px_dist_y * VEL_PUB_FREQ
            velocity_control.twist.linear.x = VELOCITY_CONTROL_P * car_px_dist_x # + VELOCITY_CONTROL_I * self.integral_term_x + VELOCITY_CONTROL_D * derivative_x
            velocity_control.twist.linear.y = VELOCITY_CONTROL_P * car_px_dist_y # + VELOCITY_CONTROL_I * self.integral_term_y + VELOCITY_CONTROL_D * derivative_y

            velocity_control.twist.linear.z = 0.0
            velocity_control.twist.angular.x = 0.0
            velocity_control.twist.angular.y = 0.0
            velocity_control.twist.angular.z = 0.0
            self.get_logger().info(f"Desired velocity: x: ({velocity_control.twist.linear.x}, y: {velocity_control.twist.linear.y})")
            self.mavros_velocity_pub.publish(velocity_control)
            
        self.last_car_px_dist_x = car_px_dist_x
        self.last_car_px_dist_y = car_px_dist_y   
    
    def state_machine_callback(self):
        self.get_logger().info(f"Current state: {self.state}")
        self.state_pub.publish(String(data=self.state))
        if self.state == INIT_STATE:
            return
        elif self.state in [LAUNCH_STATE, ESCAPED_BOUNDS_STATE, LAND_STATE, ABORT_STATE]:
            if self.desired_pose is None:
                self.desired_pose = self.cur_pose
                return
            # Attach timestamp to the desired pose
            self.desired_pose.header.stamp = self.get_clock().now().to_msg()
            self.mavros_pose_pub.publish(self.desired_pose)
        elif self.state == SEARCHING_STATE:
            self.handle_searching_state()
        elif self.state == CHASE_STATE:
            self.handle_chase_state()
        else:
            self.get_logger().error(f"Invalid state: {self.state}")
            self.state = INIT_STATE
    
def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
