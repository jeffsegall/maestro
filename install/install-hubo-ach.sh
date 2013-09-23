#!/bin/bash
set -e
echo "Hubo-ACH - Maestro installation script"
echo "Version 1.0"
echo ""

#Hubo-Ach
#add-apt-repository "deb http://code.golems.org/ubuntu precise golems.org"
#add-apt-repository "deb http://www.repo.danlofaro.com/release precise main"
#apt-get update
#apt-get install -y --force-yes --no-remove libach1 libach-dev ach-utils hubo-ach hubo-ach-dev

if [[ $# > 0 && $1 == "-y" ]]; then
	QUIET=true
fi

if [[ -z "$QUIET" ]]; then
	read -p "Please enter installation directory (No trailing '/' Please): " installDir
else
	installDir=~/
fi

if [[ ! -d "$installDir" ]]; then
	mkdir $installDir
	if [[ ! -d "$installDir" ]]; then
		echo "Failed to create install directory."
		exit 1
	fi
fi

cd "$installDir"

git clone https://github.com/isaacgaretmia/hubo-ach

cd hubo-ach

git checkout develop

apt-get install autoconf automake libtool autoconf-archive

autoreconf -i

./configure

make

make install

source /opt/ros/fuerte/setup.bash

#Hubo-Ach-Ros
cd /opt/ros/fuerte/stacks
git clone http://github.com/isaacgaretmia/hubo-ach-ros.git
cd hubo-ach-ros
git checkout maestro
rosmake hubo_ros

#Hubo-Ach-Ros-Visualization
cd /opt/ros/fuerte/stacks
git clone http://github.com/isaacgaretmia/hubo-ach-ros-visualization.git
cd hubo-ach-ros-visualization
git checkout maestro

cd /opt/ros/fuerte/stacks
rosmake hubo_ros hubo_ros_visualization maestro

