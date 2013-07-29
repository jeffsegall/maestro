#!/bin/bash
#
# Comprehensive install script for Maestro, and all dependencies.
# Installs Ros Fuerte, Orocos, OpenRAVE, Maestro, Hubo-Ach, OpenHubo.
#
# Options: None
# Dependencies:
#	<Maestro Install Dir>/maestro/utils.sh
#	install-fuerte.sh
#	install-hubo-ach.sh
#	install-openHubo.sh
#
# Author: Solis Knight
# Date: July 2013
#

# Change directory to the script's directory
if [[ `echo "$0" | grep "/" | wc -l` > 0 ]]; then
    cd ${0%/*}
fi

source ../maestro/utils.sh

#set -e
echo "Comprehensive Install Script for Maestro"
echo "Version $VERSION"
echo ""

DEPENDENCY_DIRS=""
DEPENDENCY_FILES="install-fuerte.sh install-hubo-ach.sh install-openHubo.sh"
BLACKLISTED_DIRS=""
BLACKLISTED_FILES=""

check dependency dir "$DEPENDENCY_DIRS"
if [[ $? != 0 ]]; then exit $NOT_FOUND; fi
check dependency file "$DEPENDENCY_FILES"
if [[ $? != 0 ]]; then exit $NOT_FOUND; fi
check blacklist dir "$BLACKLISTED_DIRS"
if [[ $? != 0 ]]; then exit $BLACKLIST_VIOLATED; fi
check blacklist file "$BLACKLISTED_FILES"
if [[ $? != 0 ]]; then exit $BLACKLIST_VIOLATED; fi

echo "Dependencies satisfied."
echo "Installing ROS-Orocos-Maestro..."
sudo bash install-fuerte.sh -y
if [[ $? != 0 ]]; then exit $?; fi
echo "Installing Hubo-ACH..."
sudo bash install-hubo-ach.sh
if [[ $? != 0 ]]; then exit $?; fi
echo "Installing OpenHUBO..."
bash install-openHubo.sh
if [[ $? != 0 ]]; then exit $?; fi
echo "Install Complete. Exiting..."
