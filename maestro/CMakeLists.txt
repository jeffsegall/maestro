cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_find_ros_package(rtt )
find_package(Orocos-RTT HINTS ${rtt_PACKAGE_PATH}/install rtt-scripting)

rosbuild_genmsg()
# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#common commands for building c++ executables and libraries
orocos_component(${PROJECT_NAME} src/RobotControl.cpp src/HuboMotor.cpp src/huboCan.cpp src/MotorBoard.cpp src/HuboState.cpp src/pugixml.cpp)
orocos_component(${PROJECT_NAME} src/CanGateway.cpp src/huboCan.cpp src/CommHandler.cpp)
target_link_libraries(${PROJECT_NAME} OROCOS-RTT_RTT-SCRIPTING_LIBRARY)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
