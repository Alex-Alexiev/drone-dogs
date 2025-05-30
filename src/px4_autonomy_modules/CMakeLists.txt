cmake_minimum_required(VERSION 3.8)
project(px4_autonomy_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_extras REQUIRED)

install(
  PROGRAMS nodes/pose_relay_node.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/fe_3.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/teleop.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/camera_pose_relay_node.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/pose_relay_node.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/tf2_frame_publisher.py
  DESTINATION lib/${PROJECT_NAME}/
)


install (
  PROGRAMS nodes/image_saver.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/fake_vicon.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/fake_vicon_zero.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/fake_drone.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/text_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/car_tracker.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/car_tracker_pos_ctrl.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/car_chase_state_pub.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/fake_car.py
  DESTINATION lib/${PROJECT_NAME}/
)

install (
  PROGRAMS nodes/fake_car_detection.py
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  PROGRAMS nodes/car_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  PROGRAMS nodes/perception_node.py
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  PROGRAMS nodes/car_chase.py
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
