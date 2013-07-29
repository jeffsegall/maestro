#!/bin/bash
#
# Comprehensive install script for Maestro, and all dependencies.
# Installs Ros Fuerte, Orocos, OpenRAVE, Maestro, Hubo-Ach, OpenHubo.
#
# Options: None
#
# Dependencies:
#	<Maestro Install Dir>/maestro/utils.sh
#	/opt/ros/fuerte/setup.bash
#	/opt/ros/fuerte/stacks/maestro
#
# Blacklist: None
#
# Author: Solis Knight
# Date: July 2013
#

# Change directory to the script's directory
cd ${0%/*}

source ../maestro/utils.sh

echo "Comprehensive Maestro Update Script"
echo "Version $VERSION"

DEPENDENCY_DIRS="/opt /opt/ros /opt/ros/fuerte /opt/ros/fuerte/stacks
/opt/ros/fuerte/stacks/maestro"
DEPENDENCY_FILES="/opt/ros/fuerte/setup.bash"
BLACKLISTED_DIRS=""
BLACKLISTED_FILES=""

check dependency dir "$DEPENDENCY_DIRS"
if [[ $? != $SUCCESS ]]; then exit $?; fi
check dependency file "$DEPENDENCY_FILES"
if [[ $? != $SUCCESS ]]; then exit $?; fi
check blacklist dir "$BLACKLISTED_DIRS"
if [[ $? != $SUCCESS ]]; then exit $?; fi
check blacklist file "$BLACKLISTED_FILES"
if [[ $? != $SUCCESS ]]; then exit $?; fi

hubo-ach update

source /opt/ros/fuerte/setup.bash

roscd maestro
#Create and null-initialize a variable to store our current branch
CURRENT_BRANCH='' 
currentBranch CURRENT_BRANCH
if [[ $? != $SUCCESS ]]; then
	echo "Finding current branch of Maestro failed. Aborting."
	exit
fi

git pull origin $CURRENT_BRANCH
rosmake maestro

roscd hubo_ros
if [[ $? == 0 ]]; then
	currentBranch CURRENT_BRANCH
	if [[ $? != $SUCCESS ]]; then
		retval=$?
		echo "Finding current branch of hubo_ros failed. Aborting."
		return $retval;
	fi
	git pull origin $CURRENT_BRANCH
	rosmake hubo_ros
else
	echo "Unable to find a hubo-ach-ros installation. Skipping update."
fi

