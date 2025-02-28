
rm -rf build 
colcon build 
source install/setup.bash
ros2 launch px4_autonomy_modules mavros.launch.py
