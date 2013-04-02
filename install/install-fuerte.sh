#!/bin/bash
# This script installs ROS Fuerte with the Orocos Toolchain and
# openRAVE stacks.
set -e
echo "ROS-Fuerte Maestro installation Script"
echo "Version 1.0"
echo ""
installDir=`pwd`
#ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get install -y --no-remove mercurial
apt-get install -y --no-remove ros-fuerte-desktop-full
echo ""
echo ""
echo "Would you like to add a source line to your bashrc file?"
select yn in "Yes" "No"; do
	case $yn in
		Yes ) echo "Adding source command to bashrc..."; echo "source /opt/ros/fuerte/setup.bash" >> ~/.bashrc; break;;
		No ) echo "Skipping modification of bashrc..."; break;;
	esac
done
echo ""
source /opt/ros/fuerte/setup.bash
apt-get install -y --no-remove python-rosinstall python-rosdep
cd /opt/ros/fuerte/stacks/
#OROCOS
mkdir orocos
cd /opt/ros/fuerte/stacks/orocos
apt-get install -y --no-remove libreadline-dev omniorb omniidl omniorb-nameserver libomniorb4-1 libomniorb4-dev libomnithread3-dev libomnithread3c2 gccxml antlr libantlr-dev libxslt1-dev liblua5.1-0-dev ruby1.8-dev libruby1.8 rubygems1.8 
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
apt-get install -y --no-remove libboost-dev
rosmake orocos_toolchain rtt_ros_integration rtt_ros_comm rtt_common_msgs rtt_geometry
#OPENRAVE
add-apt-repository ppa:openrave/testing
apt-get update
apt-get install openrave
cd /opt/ros/fuerte/stacks
svn co https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/ .
hg clone https://kforge.ros.org/armnavigation/armnavigation
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/branches/pr2_controllers-1.4/pr2_controllers_msgs
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_common/trunk/pr2_msgs
cd /opt/ros/fuerte/stacks/openrave_planning/openrave_robot_control/
rosmake
ln -sf /opt/ros/fuerte/stacks/openrave_planning/openrave/bin/openrave /usr/bin/openrave
ln -sf /opt/ros/fuerte/stacks/openrave_planning/openrave/bin/openrave-config /usr/bin/openrave-config
cd /opt/ros/fuerte/stacks
echo ""
echo ""
echo "Would you like a link to your Maestro install in /opt/ros/fuerte/stacks?"
select yn in "Yes" "No"; do
        case $yn in
                Yes ) echo "Creating symbolic link in /opt/ros/fuerte/stacks..."; ln -s $installDir/../ maestro; break;;
                No ) echo "Skipping symbolic link creation..."; break;;
        esac
done
echo ""
rosmake maestro
echo "Installation complete."
