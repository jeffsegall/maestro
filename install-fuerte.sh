#!/bin/bash
# This script installs ROS Fuerte with the Orocos Toolchain and
# openRAVE stacks.
#ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get install -y mercurial
apt-get install -y ros-fuerte-desktop-full
echo "source /opt/ros/fuerte/setup.bash" >> ~/.bashrc
source /opt/ros/fuerte/setup.bash
apt-get install python-rosinstall python-rosdep
cd /opt/ros/fuerte/stacks/
#OROCOS
mkdir orocos
cd /opt/ros/fuerte/stacks/orocos
apt-get install libreadline-dev omniorb omniidl omniorb-nameserver libomniorb4-1 libomniorb4-dev libomnithread3-dev libomnithread3c2 gccxml antlr libantlr-dev libxslt1-dev liblua5.1-0-dev ruby1.8-dev libruby1.8 rubygems1.8 
git clone --recursive git://gitorious.org/orocos-toolchain/orocos_toolchain.git
git clone http://git.mech.kuleuven.be/robotics/rtt_ros_integration.git
git clone http://git.mech.kuleuven.be/robotics/rtt_ros_comm.git
git clone http://git.mech.kuleuven.be/robotics/rtt_common_msgs.git
git clone http://git.mech.kuleuven.be/robotics/rtt_geometry.git
roscd orocos_toolchain
git checkout toolchain-2.5
git submodule init
git submodule update
git submodule foreach git checkout toolchain-2.5
source env.sh
apt-get install libboost-dev
rosmake orocos_toolchain rtt_ros_integration rtt_ros_comm rtt_common_msgs rtt_geometry
#OPENRAVE
cd /opt/ros/fuerte/stacks
svn co https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/ .
hg clone https://kforge.ros.org/armnavigation/armnavigation
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/branches/pr2_controllers-1.4/pr2_controllers_msgs
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_common/trunk/pr2_msgs
cd /opt/ros/fuerte/stacks/openrave_planning/openrave_robot_control/
rosmake
ln -sf /opt/ros/fuerte/stacks/openrave_planning/openrave/bin/openrave /usr/bin/openrave
ln -sf /opt/ros/fuerte/stacks/openrave_planning/openrave/bin/openrave-config /usr/bin/openrave-config

