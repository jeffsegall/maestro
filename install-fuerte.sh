#!/bin/bash
# This script installs ROS Fuerte with the Orocos Toolchain and
# openRAVE stacks.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get install -y mercurial
apt-get install -y ros-fuerte-desktop-full
cd /opt/ros/fuerte/stacks/
mkdir workspace
export ROS_WORKSPACE="/opt/ros/fuerte/stacks/workspace"
cd $ROS_WORKSPACE
source /opt/ros/fuerte/setup.sh
rosws init . /opt/ros/fuerte
roscd
rosws set -y orocos/rtt_ros_integration --git http://git.mech.kuleuven.be/robotics/rtt_ros_integration.git
rosws set -y orocos/rtt_ros_comm --git http://git.mech.kuleuven.be/robotics/rtt_ros_comm.git
rosws set -y orocos/rtt_common_msgs --git http://git.mech.kuleuven.be/robotics/rtt_common_msgs.git
rosws set -y orocos/rtt_geometry --git http://git.mech.kuleuven.be/robotics/rtt_geometry.git
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get install -y mercurial
apt-get install -y ros-fuerte-desktop-full
#rosdep install -y openrave_planning
#cd /opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/
#rosmake --rosdep-yes
#ln -sf /opt/ros/diamondback/stacks/openrave_planning/openrave/bin/openrave /usr/bin/openrave
#ln -sf /opt/ros/diamondback/stacks/openrave_planning/openrave/bin/openrave-config /usr/bin/openrave-config

