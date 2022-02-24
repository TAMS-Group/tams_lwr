cmake_minimum_required(VERSION 3.0.2)
project(wsg_50_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  wsg_50_common
)

find_package(orocos_kdl REQUIRED)

catkin_package(
 CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(wsg_50_sim_keyboard_teleop src/wsg_50_keyboard_teleop.cpp)
add_executable(wsg_50_sim_driver src/wsg_50_sim_driver.cpp)

add_dependencies(wsg_50_sim_driver ${catkin_EXPORTED_TARGETS})
add_dependencies(wsg_50_sim_keyboard_teleop  ${catkin_EXPORTED_TARGETS})

target_link_libraries(wsg_50_sim_keyboard_teleop ${catkin_LIBRARIES} )
target_link_libraries(wsg_50_sim_driver ${catkin_LIBRARIES} )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
