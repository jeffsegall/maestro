#!/bin/bash
export PYTHONPATH=/opt/ros/diamondback/stacks/openrave_planning/openrave/lib/python2.7/site-packages/:$PYTHONPATH
source /opt/ros/diamondback/setup.sh
roslaunch openrave_robot_control jaemi_hubo.launch.xml
