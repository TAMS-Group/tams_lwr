cmake_minimum_required(VERSION 3.0.2)
project(wsg_50_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Cmd.msg
  Sensor.msg
  Status.msg
)

add_service_files(
  FILES
  Conf.srv
  Incr.srv
  Move.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
 )

catkin_package(
   CATKIN_DEPENDS geometry_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
