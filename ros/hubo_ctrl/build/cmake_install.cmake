# Install script for directory: /home/jeff/hubo/hubo2/ros/hubo_ctrl

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl/libhubo_ctrl-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl/libhubo_ctrl-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl/libhubo_ctrl-gnulinux.so"
         RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/hubo_ctrl:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/electric/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/electric/stacks/ros_comm/utilities/rostime/lib:/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/electric/stacks/orocos_toolchain/ocl/lib:/opt/ros/electric/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/electric/ros/core/roslib/lib:/opt/ros/electric/ros/tools/rospack/lib:/opt/ros/electric/stacks/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/electric/stacks/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/electric/stacks/orocos_toolchain/install/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl" TYPE SHARED_LIBRARY FILES "/home/jeff/hubo/hubo2/ros/hubo_ctrl/lib/orocos/gnulinux/libhubo_ctrl-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl/libhubo_ctrl-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl/libhubo_ctrl-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl/libhubo_ctrl-gnulinux.so"
         OLD_RPATH "/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/electric/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/electric/stacks/ros_comm/utilities/rostime/lib:/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/electric/stacks/orocos_toolchain/ocl/lib:/opt/ros/electric/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/electric/ros/core/roslib/lib:/opt/ros/electric/ros/tools/rospack/lib:/opt/ros/electric/stacks/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/electric/stacks/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/electric/stacks/orocos_toolchain/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/hubo_ctrl:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/electric/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/electric/stacks/ros_comm/utilities/rostime/lib:/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/electric/stacks/orocos_toolchain/ocl/lib:/opt/ros/electric/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/electric/ros/core/roslib/lib:/opt/ros/electric/ros/tools/rospack/lib:/opt/ros/electric/stacks/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/electric/stacks/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/electric/stacks/orocos_toolchain/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/hubo_ctrl/libhubo_ctrl-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/jeff/hubo/hubo2/ros/hubo_ctrl/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/jeff/hubo/hubo2/ros/hubo_ctrl/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
