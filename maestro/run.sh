#!/bin/bash
export PYTHONPATH=$PYTHONPATH:`openrave-config --python-dir`
source /opt/ros/diamondback/setup.sh
cp src/maestro.py /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/.
roslaunch openrave_robot_control jaemi_hubo.launch.xml
