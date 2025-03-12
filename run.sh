
rm -rf build 
colcon build 
source install/setup.bash
ros2 launch px4_autonomy_modules mavros.launch.py



# ros2 service call /rob498_drone_7/comm/launch std_srvs/srv/Trigger
# ros2 service call /rob498_drone_7/comm/land std_srvs/srv/Trigger

# ros2 launch realsense2_camera rs_launch.py 
# ros2 topic pub -1 /offsets/x std_msgs/msg/Float64 "{data: 0.1}"
# ros2 topic pub -1 /offsets/y std_msgs/msg/Float64 "{data: 0.1}"
# ros2 topic pub -1 /offsets/z std_msgs/msg/Float64 "{data: 0.1}"
