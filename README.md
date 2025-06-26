# 🚁 Police Chase Drone - ROB498 Capstone Project

![Drone Photo](readme_images/drone_photo.jpg)

## Overview

![Project Banner](readme_images/drone_in_action.gif)

This repository contains the final implementation and documentation for our **ROB498 Capstone Design Project** as part of the **Robotics Engineering** undergraduate program at the **University of Toronto**.

Our project, **Police Chase Drone**, showcases the design and development of an autonomous drone system capable of pursuing a dynamic ground target. Leveraging ROS 2 and onboard computation, the drone tracks and follows a "getaway car" through a series of waypoints and behaviors designed to mimic a pursuit scenario.

## Features

![MobileSSD-Net in Action](readme_images/detectnet.gif)

- 🔍 **Real-time tracking** of a ground vehicle using onboard sensors and vision using a custom MobileSSD-Net
- 🧠 **Autonomous navigation** with waypoint following and obstacle avoidance
- 📡 **ROS 2-based modular software stack** for sensor integration and control
- 🛸 **Custom flight behaviors** to simulate realistic police drone pursuits

## System Overview

![Functional Analysis](readme_images/functional_analysis.jpg)

The software system is built in ROS 2 using a set of nodes that handle:

- Perception: Camera input, object detection, and tracking
- Planning: Waypoint generation and trajectory tracking using a state machine
- Control: Velocity and position control for the drone

Here's the state machine diagram that outlines the drone's behavior:

![State Machine](readme_images/state_machine.jpg)

And here is it in action in the simulation:

![State Machine in Action](readme_images/sim.gif)


## Installation

1. Clone the repository:
```bash
git clone https://github.com/Alex-Alexiev/drone-dogs.git
```

2. Make dependancy folder:
```bash
mkdir -p drone_deps/src
```

3. Install jetson-inference:
```bash
cd ~/drone_deps/src
sudo apt-get install git cmake
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference && cd jetson-inference
mkdir build && cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

4. Install vision_msgs:
```bash
cd ~/drone_deps/src
git clone https://github.com/ros-perception/vision_msgs.git -b foxy
cd ~/drone_deps
colcon build --packages-select vision_msgs
export CMAKE_PREFIX_PATH=~/drone_deps/install:/opt/ros/foxy
```

5. Install ros_deep_learning: 
```bash
cd ~/drone_deps/src
git clone https://github.com/dusty-nv/ros_deep_learning

cd ~/drone_deps/src/ros_deep_learning && mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/drone_deps/install:/opt/ros/foxy -DVPI_DIR=/usr/share/vpi1/cmake
make -j$(nproc)

cd ~/drone_deps
colcon build --packages-select ros_deep_learning
```

6. 
```bash
source install/setup.bash
```

## Usage
From the `drone-dogs` repository:

Launch the Rviz2 'simulation' environment:
```bash
./test.sh
```

For hardware-in-the-loop testing:
```bash
./run.sh
```

Launch video source:
```bash
ros2 launch ros_deep_learning video_source.ros2.launch
```

Launch detectnet:
```bash
ros2 launch ros_deep_learning detectnet.ros2.launch output:=display://0 model_path:=/home/jetson/ros2_ws/src/px4_autonomy_modules/models/ssd-mobilenet-apr7.onnx input_blob:=input_0 output_cvg:=scores output_bbox:=boxes threshold:=0.2 class_labels_path:=/home/jetson/ros2_ws/src/px4_autonomy_modules/models/labels.txt image_height:=180 image_width:=320
```

## Hardware Used

* **NVIDIA Jetson Nano** for onboard computation
* **IMX219 Color Camera**
* **Orange Cube+ PX4-compatible flight controller**
* **Battery-powered quadcopter frame**
* **Wi-Fi telemetry and ground station**

## Team Members

![Team Members](readme_images/team.jpg)

This project was completed by:

* Kevin Angers
* Chris Adolphe
* Alexander Alexiev
* John Wolf

Supervised as part of the **ROB498 Capstone Design Project** at the **University of Toronto**.

## Acknowledgments

We thank Professor Waslander and the teaching assistants staff for their guidance and support throughout this project, as well as the sample code [found here](https://github.com/utiasSTARS/ROB498-flight).
Special thanks to [dusty-nv](https://github.com/dusty-nv) for the [Jetson Inference](https://github.com/dusty-nv/jetson-inference) and [ROS Deep Learning](https://github.com/dusty-nv/ros_deep_learning) repositories, which provided invaluable resources for our computer vision tasks.

## License

This project is licensed under the [MIT License](LICENSE).
