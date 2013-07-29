# Install script for directory: /opt/ros/fuerte/stacks/maestro/maestro

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
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro/libmaestro-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro/libmaestro-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro/libmaestro-gnulinux.so"
         RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/maestro:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/rtt/install/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/rtt/install/lib/orocos/gnulinux/plugins")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro" TYPE SHARED_LIBRARY FILES "/opt/ros/fuerte/stacks/maestro/maestro/lib/orocos/gnulinux/libmaestro-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro/libmaestro-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro/libmaestro-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro/libmaestro-gnulinux.so"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/rtt/install/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/rtt/install/lib/orocos/gnulinux/plugins:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/maestro:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/rtt/install/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/rtt/install/lib/orocos/gnulinux/plugins")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/maestro/libmaestro-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/opt/ros/fuerte/stacks/maestro/maestro/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/opt/ros/fuerte/stacks/maestro/maestro/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
