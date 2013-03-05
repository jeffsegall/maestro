#!/bin/bash

source /opt/ros/fuerte/setup.bash

gnome-terminal -x hubo-ach virtual
gnome-terminal -x ./run.sh

roscd hubo_ros_visualization
echo `pwd`
gnome-terminal -x ./run.sh

roscd hubo_ros
echo `pwd`
gnome-terminal -x ./run-feedback.sh
gnome-terminal -x ./run-feedforward.sh

