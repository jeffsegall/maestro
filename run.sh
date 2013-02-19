#!/bin/bash
source /opt/ros/fuerte/setup.sh
rosmake maestro && roslaunch maestro/launch/jaemi_hubo.launch.xml && roslaunch hubo_ros_visualization/launch/hubo_feedback.launch
