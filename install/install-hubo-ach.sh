#!/bin/bash
set -e
echo "Hubo-ACH - Maestro installation script"
echo "Version 1.0"
echo ""

#Hubo-Ach
add-apt-repository "deb http://code.golems.org/ubuntu precise golems.org"
add-apt-repository "deb http://www.repo.danlofaro.com/release precise main"
apt-get update
apt-get install -y --no-remove libach1 libach-dev ach-utils hubo-ach hubo-ach-dev

source /opt/ros/fuerte/setup.bash

#Hubo-Ach-Ros
cd /opt/ros/fuerte/stacks
git clone http://github.com/isaacgaretmia/hubo-ach-ros.git
cd hubo-ach-ros
git checkout maestro
rosmake hubo_ros

#Hubo-Ach-Ros-Visualization
cd /opt/ros/fuerte/stacks
git clone http://github.com/isaacgaretmia/hubo-ach-ros-visualization.git
cd hubo-ach-ros-visualization
git checkout maestro

cd /opt/ros/fuerte/stacks
rosmake hubo_ros hubo_ros_visualization maestro

