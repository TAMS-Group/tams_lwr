cmake_minimum_required(VERSION 3.0.2)
project(tams_lwr)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  moveit_msgs
  actionlib_msgs
  actionlib
  tf
  tf_conversions
  tf2_bullet
  wsg_50_common
  joy
  moveit_msgs
  moveit_core
  moveit_ros_planning
  ros_fri_msgs
  apriltag_ros
  tams_apriltags
  tams_lwr_description
  tams_lwr_wsg50_description
  tams_lwr_wsg50_moveit
)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs trajectory_msgs control_msgs actionlib_msgs actionlib tf roscpp
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


link_directories(
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(joint_state_merger src/joint_state_merger.cpp)
target_link_libraries(joint_state_merger ${catkin_LIBRARIES} pthread)

add_executable(gripper_action_server src/gripper_action_server.cpp)
add_dependencies(gripper_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_action_server ${catkin_LIBRARIES} pthread)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(lwr_nk_teleop src/lwr_nk_teleop)
add_dependencies(lwr_nk_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(lwr_nk_teleop ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)

add_executable(kuka_sphere_object_shape_detection src/kuka_sphere_object_shape_detection)
add_dependencies(kuka_sphere_object_shape_detection ${catkin_EXPORTED_TARGETS})
target_link_libraries(kuka_sphere_object_shape_detection ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)

add_executable(kuka_sphere_object_shape_detection_v2 src/kuka_sphere_object_shape_detection_v2)
add_dependencies(kuka_sphere_object_shape_detection_v2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(kuka_sphere_object_shape_detection_v2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)
