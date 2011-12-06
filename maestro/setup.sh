#!/bin/bash
source /opt/ros/diamondback/setup.sh
ln -sf $HOME/hubo2/maestro/launch/jaemi_hubo.launch.xml /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/jaemi_hubo.launch.xml
ln -sf $HOME/hubo2/maestro/models /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/models
ln -sf $HOME/hubo2/maestro/src /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/maestro-src
