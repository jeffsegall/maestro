#!/bin/bash

source /opt/ros/fuerte/setup.bash

gnome-terminal -x hubo-ach virtual
sleep 1
gnome-terminal -x ./run.sh
sleep 1

roscd hubo_ros_visualization
echo `pwd`
gnome-terminal -x ./run.sh
sleep 1

roscd hubo_ros
echo `pwd`
gnome-terminal -x ./run-feedback.sh
sleep 1
gnome-terminal -x ./run-feedforward.sh
sleep 1
