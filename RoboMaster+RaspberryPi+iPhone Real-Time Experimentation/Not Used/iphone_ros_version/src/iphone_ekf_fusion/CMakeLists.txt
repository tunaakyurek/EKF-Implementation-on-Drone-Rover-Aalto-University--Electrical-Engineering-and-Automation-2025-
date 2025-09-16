cmake_minimum_required(VERSION 3.0.2)
project(iphone_ekf_fusion)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

## Python setup
catkin_python_setup()

## Generate messages
add_message_files(
  FILES
  IPhoneSensorData.msg
  EKFState.msg
)

## Generate services
add_service_files(
  FILES
  # Add service files here if needed
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    message_runtime
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/ekf_15dof_node.py
  scripts/ekf_visualizer.py
  scripts/performance_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
