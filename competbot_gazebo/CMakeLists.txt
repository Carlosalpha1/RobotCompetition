cmake_minimum_required(VERSION 3.0.2)
project(competbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  competbot_description
  gazebo_ros
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
