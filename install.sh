#!/bin/bash
# This script installs ROS Diamondback with the Orocos Toolchain and
# openRAVE stacks.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu lucid main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get install -y mercurial
apt-get install -y ros-diamondback-desktop-full ros-diamondback-orocos-toolchain-ros
cd /opt/ros/diamondback/stacks
svn co https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/ .
hg clone https://kforge.ros.org/armnavigation/armnavigation
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/branches/pr2_controllers-1.4/pr2_controllers_msgs
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_common/trunk/pr2_msgs
source /opt/ros/diamondback/setup.sh
#rosdep install -y openrave_planning
cd /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/
rosmake --rosdep-yes
ln -sf /opt/ros/diamondback/stacks/openrave_planning/openrave/bin/openrave /usr/bin/openrave
ln -sf /opt/ros/diamondback/stacks/openrave_planning/openrave/bin/openrave-config /usr/bin/openrave-config

