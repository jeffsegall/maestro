#!/bin/bash
source /opt/ros/diamondback/setup.sh
l
-sf `pwd`/launch/jaemi_hubo.launch.xml /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/jaemi_hubo.launch.xml
ln -sf `pwd`/models /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/models
ln -sf `pwd`/src /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/maestro-src
