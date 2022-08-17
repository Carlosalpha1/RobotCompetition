cmake_minimum_required(VERSION 3.0.2)
project(robot_competition_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Pose.msg
  Poses.msg
)

add_service_files(
  FILES
  registry.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
