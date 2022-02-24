cmake_minimum_required(VERSION 3.0.2)
project(tams_lwr_wsg50_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tams_lwr_description
)

catkin_package(
)
