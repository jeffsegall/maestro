#!/bin/bash
source /opt/ros/fuerte/setup.sh
rosmake maestro && roslaunch maestro/launch/jaemi_hubo.launch.xml
