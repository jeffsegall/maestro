#!/bin/bash
source /opt/ros/diamondback/setup.sh
rosmake maestro && roslaunch maestro/launch/jaemi_hubo.launch.xml
