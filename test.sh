
rm -rf build 
colcon build --packages-select px4_autonomy_modules
source install/setup.bash
ros2 launch px4_autonomy_modules test_mavros.launch.py



# ros2 service call /rob498_drone_7/comm/launch std_srvs/srv/Trigger
# ros2 service call /rob498_drone_7/comm/start_search std_srvs/srv/Trigger
# ros2 service call /rob498_drone_7/comm/land std_srvs/srv/Trigger

# ros2 launch realsense2_camera rs_launch.py 
# ros2 topic pub -1 /offsets/x std_msgs/msg/Float64 "{data: 0.1}"
# ros2 topic pub -1 /offsets/y std_msgs/msg/Float64 "{data: 0.1}"
# ros2 topic pub -1 /offsets/z std_msgs/msg/Float64 "{data: 0.1}"

# ros2 bag record -a

# Publish this once
# ros2 topic pub /mavros/local_position/odom nav_msgs/msg/Odometry "{
#   header: {
#     stamp: {sec: 0, nanosec: 0},
#     frame_id: 'map'
#   },
#   child_frame_id: 'base_link',
#   pose: {
#     pose: {
#       position: {x: 0.0, y: 0.0, z: 0.0},
#       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
#     },
#     covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#   },
#   twist: {
#     twist: {
#       linear: {x: 0.0, y: 0.0, z: 0.0},
#       angular: {x: 0.0, y: 0.0, z: 0.0}
#     },
#     covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#   }
# }" --once