cmake_minimum_required(VERSION 3.0.2)
project(robot_competition_manager)

find_package(catkin REQUIRED COMPONENTS
  competbot_gazebo
  geometry_msgs
  robot_competition_scene
  roscpp
  std_msgs
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  robot_competition_msgs
  pcl_conversions
  pcl_ros
  tf
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS
  competbot_gazebo
  geometry_msgs
  robot_competition_scene
  roscpp
  std_msgs
  std_srvs
  cv_bridge
  image_transport
  std_msgs
  robot_competition_msgs
  pcl_conversions
  pcl_ros
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

add_executable(camera_controller src/camera_controller.cpp)
target_link_libraries(camera_controller ${catkin_LIBRARIES})

add_executable(poses_compute src/poses_compute.cpp)
target_link_libraries(poses_compute ${catkin_LIBRARIES})
