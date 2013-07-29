# Install script for directory: /opt/ros/fuerte/stacks/maestro/rtt_hubomsg

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
    SET(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-typekit-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-typekit-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-typekit-gnulinux.so"
         RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_hubomsg/types:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types" TYPE SHARED_LIBRARY FILES "/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/lib/orocos/gnulinux/types/librtt-hubomsg-typekit-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-typekit-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-typekit-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-typekit-gnulinux.so"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_hubomsg/types:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-typekit-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-ros-transport-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-ros-transport-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-ros-transport-gnulinux.so"
         RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_hubomsg/types:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types" TYPE SHARED_LIBRARY FILES "/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/lib/orocos/gnulinux/types/librtt-hubomsg-ros-transport-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-ros-transport-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-ros-transport-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-ros-transport-gnulinux.so"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/build:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_hubomsg/types:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/stacks/orocos/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos/orocos_toolchain/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_hubomsg/types/librtt-hubomsg-ros-transport-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/build/rtt_hubomsg-gnulinux.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/opt/ros/fuerte/stacks/maestro/rtt_hubomsg/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
