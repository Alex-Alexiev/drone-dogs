
rm -rf build 
colcon build --packages-select px4_autonomy_modules
source install/setup.bash
ros2 launch px4_autonomy_modules mavros.launch.py



# ros2 service call /rob498_drone_7/comm/launch std_srvs/srv/Trigger
# ros2 service call /rob498_drone_7/comm/land std_srvs/srv/Trigger

# ros2 launch realsense2_camera rs_launch.py 
# ros2 topic pub -1 /offsets/x std_msgs/msg/Float64 "{data: 0.1}"
# ros2 topic pub -1 /offsets/y std_msgs/msg/Float64 "{data: 0.1}"
# ros2 topic pub -1 /offsets/z std_msgs/msg/Float64 "{data: 0.1}"

# ros2 bag record -a

# ros2 launch ros_deep_learning video_source.ros2.launch

# ros2 launch ros_deep_learning detectnet.ros2.launch output:=display://0 model_path:=/home/jetson/ros2_ws/src/px4_autonomy_modules/models/ssd-mobilenet-apr7.onnx input_blob:=input_0 output_cvg:=scores output_bbox:=boxes threshold:=0.2 class_labels_path:=/home/jetson/ros2_ws/src/px4_autonomy_modules/models/labels.txt image_height:=180 image_width:=320


