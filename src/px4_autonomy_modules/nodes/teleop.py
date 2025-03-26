#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
import sys
from geometry_msgs.msg import PoseStamped
from copy import deepcopy 
import threading
import select
import termios
import tty

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_07')
        drone_num = '7'
        self.initial_pose = None
        self.cur_pose = None
        self.desired_pose = None
        self.get_logger().info(f'Drone Number: {drone_num}')
        
        # Set up key bindings and step size (in meters)
        self.step = 0.1
        self.key_bindings = {
            'w': (self.step, 0, 0),    # forward (increase x)
            's': (-self.step, 0, 0),   # backward (decrease x)
            'a': (0, -self.step, 0),   # left (decrease y)
            'd': (0, self.step, 0),    # right (increase y)
            'z': (0, 0, self.step),    # up (increase z)
            'x': (0, 0, -self.step)    # down (decrease z)
        }
        self.get_logger().info("Teleop keys: w/s (x-axis), a/d (y-axis), z/x (z-axis)")
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.listener_callback,
            rclpy.qos.qos_profile_system_default
        )
        self.publisher = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.srv_launch = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, f'rob498_drone_{drone_num}/comm/abort', self.callback_abort)
        self.get_logger().info("Finished setting up services")
        
        publish_freq = 20  # Hz
        self.timer = self.create_timer(1 / publish_freq, self.pose_publisher_callback)
        
        # Start teleoperation keyboard thread
        self.teleop_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.teleop_thread.start()
        
    # Service callbacks and command handlers (launch, test, land, abort) remain unchanged.
    def handle_launch(self):
        self.get_logger().info('Launch Requested. Your drone should take off.')
        if self.cur_pose is None:
            self.get_logger().error('cur_pose is None, make sure vision_pose is being published to')
            return
        self.initial_pose = deepcopy(self.cur_pose)
        starting_pose = deepcopy(self.cur_pose)
        starting_pose.pose.position.z = 2.5
        starting_pose.pose.position.y = 0.0
        starting_pose.pose.position.x = 0.0
        self.desired_pose = deepcopy(starting_pose)
        
    def handle_test(self):
        self.get_logger().info('Test Requested. Your drone should perform the required tasks. Recording starts now.')

    def handle_land(self):
        self.get_logger().info('Land Requested. Your drone should land.')
        if self.initial_pose is None:
            self.get_logger().error("initial pose is None, aborting...")
            self.handle_abort()
        self.desired_pose = deepcopy(self.initial_pose)

    def handle_abort(self):
        self.get_logger().info('Abort Requested. Your drone should land immediately due to safety considerations')
        land_now = self.cur_pose
        land_now.pose.position.z = 0
        self.desired_pose = land_now
        
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
    
    def listener_callback(self, msg):
        self.cur_pose = msg
        pose = msg.pose
        self.get_logger().info(f"Received Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z}) "
                               f"Orientation: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
        
    def pose_publisher_callback(self):
        # Ensure desired_pose is valid
        if self.desired_pose is None:
            self.desired_pose = deepcopy(self.cur_pose) if self.cur_pose is not None else None
            return
        self.desired_pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.desired_pose)
        
        pose = self.desired_pose.pose
        self.get_logger().info(f"Desired Pose - Position: ({pose.position.x}, {pose.position.y}, {pose.position.z}) "
                               f"Orientation: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
    
    def keyboard_loop(self):
        """Thread loop to capture key presses and update the desired_pose accordingly."""
        # Save original terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                # Check if a key has been pressed (non-blocking with timeout)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key in self.key_bindings:
                        dx, dy, dz = self.key_bindings[key]
                        # If desired_pose is not yet initialized, use current pose
                        if self.desired_pose is None:
                            if self.cur_pose is not None:
                                self.desired_pose = deepcopy(self.cur_pose)
                            else:
                                self.get_logger().warn("No current pose available to initialize desired_pose")
                                continue
                        # Update the desired setpoint
                        self.desired_pose.pose.position.x += dx
                        self.desired_pose.pose.position.y += dy
                        self.desired_pose.pose.position.z += dz
                        self.get_logger().info(f"Key '{key}' pressed. New desired position: "
                                               f"({self.desired_pose.pose.position.x:.2f}, {self.desired_pose.pose.position.y:.2f}, {self.desired_pose.pose.position.z:.2f})")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()