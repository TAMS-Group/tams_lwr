cmake_minimum_required(VERSION 3.0.2)
project(ros_fri_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  LWRStates.msg
  MotionCompletion.msg
  RMLPositionInputParameters.msg
  RMLVelocityInputParameters.msg
  fri_status.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES doro_msgs
   CATKIN_DEPENDS std_msgs message_runtime roscpp geometry_msgs
#  DEPENDS system_lib
)
