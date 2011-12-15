#!/bin/bash
source /opt/ros/diamondback/setup.sh
rosmake && roslaunch launch/jaemi_hubo.launch.xml
