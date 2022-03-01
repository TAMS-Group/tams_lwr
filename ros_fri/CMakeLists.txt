cmake_minimum_required(VERSION 3.0.2)
project(ros_fri)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        trajectory_msgs
        control_msgs
        moveit_msgs
        ros_fri_msgs
        actionlib_msgs
        actionlib
        tf
        tf_conversions
        tf2_bullet
        moveit_ros_planning
        controller_manager
        reflexxes_type2
        frilibrary
        eigen_conversions
        )


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES doro_msgs
        CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs trajectory_msgs control_msgs actionlib_msgs actionlib
         ros_fri_msgs tf roscpp moveit_msgs
        #  DEPENDS system_lib
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)


link_directories(
        lib
        ${catkin_LIBRARIES}
)

add_executable(ros_fri src/arm_controller.cpp)
add_dependencies(ros_fri ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_fri ${catkin_LIBRARIES} ${Boost_LIBRARIES}
        FastResearchInterfaceLibrary rt ReflexxesTypeII)

# standalone tool to "reset" the arm to candle position
#
add_executable(lwr_move_to_candle src/lwr_move_to_candle.cpp)
add_dependencies(lwr_move_to_candle ${catkin_EXPORTED_TARGETS})
target_link_libraries(lwr_move_to_candle FastResearchInterfaceLibrary rt ReflexxesTypeII
        ${catkin_LIBRARIES} pthread)

# endless loop demo for (small) joint-position motions via ros_fri
#
add_executable(test_jnt_pos_goal src/test_jnt_pos_goal.cpp)
add_dependencies(test_jnt_pos_goal ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_jnt_pos_goal ${catkin_LIBRARIES} pthread)

# current position plus sinewave offset via ros_fri
#
add_executable(lwr_sinewave src/lwr_sinewave.cpp)
add_dependencies(lwr_sinewave ${catkin_EXPORTED_TARGETS})
target_link_libraries(lwr_sinewave ${catkin_LIBRARIES} FastResearchInterfaceLibrary rt pthread)

# generic joint-position motion via ros_fri
#
add_executable(lwr_movej src/lwr_movej.cpp)
add_dependencies(lwr_movej ${catkin_EXPORTED_TARGETS})
target_link_libraries(lwr_movej ${catkin_LIBRARIES} pthread)

# joint-space gravity compensation. Warning: this uses
# the  LWR estimated external torques, and requires correct
# calibration/settings of $TOOL in the Kuka software.
#
add_executable(lwr_grav_comp src/lwr_grav_comp.cpp)
add_dependencies(lwr_grav_comp ${catkin_EXPORTED_TARGETS})
target_link_libraries(lwr_grav_comp ReflexxesTypeII ${catkin_LIBRARIES} pthread)

add_executable(lwr_move_to_startPose src/lwr_move_to_startPose.cpp)
add_dependencies(lwr_move_to_startPose ${catkin_EXPORTED_TARGETS})
target_link_libraries(lwr_move_to_startPose ReflexxesTypeII ${catkin_LIBRARIES} pthread)

add_executable(ros_fri_lwr src/ros_fri_lwr.cpp)
add_dependencies(ros_fri_lwr ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_fri_lwr ${catkin_LIBRARIES} FastResearchInterfaceLibrary rt pthread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_executable(ros_fri_roscontrol_new src/ros_fri_roscontrol_new.cpp)
add_dependencies(ros_fri_roscontrol_new ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_fri_roscontrol_new ${catkin_LIBRARIES} FastResearchInterfaceLibrary rt pthread)

# add_executable(find_kuka_base_transform src/find_kuka_base_transform.cpp)
# target_link_libraries(find_kuka_base_transform ${catkin_LIBRARIES})

# the following don't work (yet)
# target_link_libraries(move_to_candle ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#                               FastResearchInterfaceLibrary rt ReflexxesTypeII pthread)
# target_link_libraries(sinewave ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#                               FastResearchInterfaceLibrary rt ReflexxesTypeII pthread)
