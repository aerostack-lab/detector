cmake_minimum_required(VERSION 3.0.2)
project(detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  belief_manager_msgs
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  belief_manager_msgs
)
