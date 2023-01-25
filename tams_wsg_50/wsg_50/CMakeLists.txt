cmake_minimum_required(VERSION 3.0.2)
project(wsg_50)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        # eigen
        roscpp
        urdf
        message_generation
        std_msgs
        sensor_msgs
        geometry_msgs
        actionlib_msgs
        actionlib
        wsg_50_common
        wsg_50_simulation
        std_srvs
        )

catkin_package(
        CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime wsg_50_common actionlib_msgs roscpp wsg_50_simulation
        actionlib
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# WSG_50_TCP version
set(DRIVER_SOURCES
        src/checksum.cpp
        src/cmd.c
        src/common.cpp
        src/interface.cpp
        src/msg.c
        src/serial.c
        src/tcp.c
        src/udp.c)

# WSG_50_CAN version
# add_executable(wsg_50_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)

add_executable(wsg_50_tcp_tactile src/main_tactile.cpp ${DRIVER_SOURCES})
add_dependencies(wsg_50_tcp_tactile ${catkin_EXPORTED_TARGETS})
target_link_libraries(wsg_50_tcp_tactile ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
