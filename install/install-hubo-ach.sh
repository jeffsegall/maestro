#!/bin/bash
#
# This script installs Hubo-Ach and the Hubo-Ach-Ros interface for use with
# various ROS stacks. Additionally, the hubo-ach-ros-visualization package
# will be installed and configured.
#
# Options: None
#
# Dependencies:
#	<Maestro Install Dir>/maestro/utils.sh
#
# Blacklist: None
#
# Author: Solis Knight
# Date: July 2013

# Change directory to the script's directory.
if [[ `echo "$0" | grep "/" | wc -l` > 0 ]]; then
    cd ${0%/*}
fi

# Source environment checking functions.
source ../maestro/utils.sh

# Stop execution on any significant error.
#set -e

echo "Hubo-ACH - Maestro installation script"
echo "Version $VERSION"
echo ""

DEPENDENCY_DIRS="/opt /opt/ros /opt/ros/fuerte /opt/ros/fuerte/stacks"
DEPENDENCY_FILES="/opt/ros/fuerte/setup.bash"
BLACKLISTED_DIRS="/opt/ros/fuerte/stacks/hubo-ach-ros
/opt/ros/fuerte/stacks/hubo-ach-ros-visualization"
BLACKLISTED_FILES=""

check dependency dir "$DEPENDENCY_DIRS"
if [[ $? != 0 ]]; then exit $NOT_FOUND; fi
check dependency file "$DEPENDENCY_FILES"
if [[ $? != 0 ]]; then exit $NOT_FOUND; fi
check blacklist dir "$BLACKLISTED_DIRS"
if [[ $? != 0 ]]; then exit $BLACKLIST_VIOLATED; fi
check blacklist file "$BLACKLISTED_FILES"
if [[ $? != 0 ]]; then exit $BLACKLIST_VIOLATED; fi


#Hubo-Ach
add-apt-repository "deb http://code.golems.org/ubuntu precise golems.org"
add-apt-repository "deb http://www.repo.danlofaro.com/release precise main"
apt-get update
apt-get install -y --force-yes --no-remove libach1 libach-dev ach-utils hubo-ach hubo-ach-dev

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

