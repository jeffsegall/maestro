#!/bin/bash
# This script installs ROS Diamondback with the Orocos Toolchain and
# openRAVE stacks.
sudo apt-get update
sudo apt-get install ros-diamondback-desktop-full ros-diamondback-orocos-toolchain-ros
cd /opt/ros/diamondback/stacks
sudo chown -R `whoami` *
svn co https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/ .
hg clone https://kforge.ros.org/armnavigation/armnavigation
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/branches/pr2_controllers-1.4/pr2_controllers_msgs
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_common/trunk/pr2_msgs
rosdep install openrave_planning
