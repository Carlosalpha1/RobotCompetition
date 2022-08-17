cmake_minimum_required(VERSION 3.0.2)
project(blue_team_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robot_competition_msgs
  roscpp
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  robot_competition_msgs
  roscpp
  std_msgs
  nav_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(player src/player.cpp)
target_link_libraries(player ${catkin_LIBRARIES})
