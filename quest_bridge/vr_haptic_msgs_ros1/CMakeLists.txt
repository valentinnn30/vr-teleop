# ROS 1 (catkin) build for vr_haptic_msgs.
# The msg/ directory is supplied by the Dockerfile, copied from
# third_party/ROS-TCP-Endpoint/vr_haptic_msgs/msg/.
cmake_minimum_required(VERSION 3.0.2)
project(vr_haptic_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Taxel.msg
  ManoLandmarks.msg
  HandGesture.msg
  HapticReadings.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
