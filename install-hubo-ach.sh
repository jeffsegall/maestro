#!/bin/bash
#Hubo-Ach
sh -c 'echo "deb http://www.drc-hubo.com/software precise main" > /etc/apt/source.list'
apt-get update
apt-get install hubo-ach hubo-ach-dev

#Hubo-Ach-Ros
cd /opt/ros/fuerte/stacks
git clone http://github.com/isaacgaretmia/hubo-ach-ros.git
cd hubo-ach-ros
git checkout maestro

#Hubo-Ach-Ros-Visualization
cd /opt/ros/fuerte/stacks
git clone http://github.com/isaacgaretmia/hubo-ach-ros-visualization.git
cd hubo-ach-ros-visualization
git checkout maestro

cd /opt/ros/fuerte/stacks
rosmake hubo_ros_visualization

