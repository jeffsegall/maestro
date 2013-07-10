#!/bin/bash
#
# Comprehensive install script for Maestro, and all dependencies.
# Installs Ros Fuerte, Orocos, OpenRAVE, Maestro, Hubo-Ach, OpenHubo.
#
# Options: None
# Dependencies:
#	install-fuerte.sh
#	install-hubo-ach.sh
#	install-openHubo.sh
#
# Author: Solis Knight
# Date: July 2013
#

# Exit Error Codes
WRONG_NUMBER_ARGUMENTS=1
BAD_ARGUMENTS=2
NOT_FOUND=3
BLACKLIST_VIOLATED=4


# Change directory to the script's directory.
cd ${0%/*}

set -e
echo "Comprehensive Install Script for Maestro"
echo "Version 1.0"
echo ""

DEPENDENCY_DIRS=""
DEPENDENCY_FILES="install-fuerte.sh install-hubo-ach.sh install-openHubo.sh"
BLACKLISTED_DIRS=""
BLACKLISTED_FILES=""

function see() {
        if [[ $# != 2 ]]; then
                return $WRONG_NUMBER_ARGUMENTS
        fi

        if [[ "$1" == "file" ]]; then
                if [[ ! -e "$2" || ! -f "$2" ]]; then
                        return $NOT_FOUND
                fi

        elif [[ "$1" == "dir" ]]; then
                if [[ ! -e "$2" || ! -d "$2" ]]; then
                        return $NOT_FOUND
                fi
        else
                return $BAD_ARGUMENTS
        fi

        return 0
}

for dir in $DEPENDENCY_DIRS; do
        see "dir" "$dir"
        retval=$?
        if [[ "$retval" != 0 ]]; then
                echo "Required dependency $dir was found."
                exit $NOT_FOUND
        fi
done

for file in $DEPENDENCY_FILES; do
        see "file" "$file"
        retval=$?
        if [[ "$retval" != 0 ]]; then
                echo "Required dependency $file was not found."
                exit $NOT_FOUND
        fi
done

for dir in $BLACKLISTED_DIRS; do
        see "dir" "$dir"
        retval=$?
        if [[ "$retval" == 0 ]]; then
                echo "Blacklisted directory $dir was found."
                exit $BLACKLIST_VIOLATED
        fi
done

for file in $BLACKLISTED_FILES; do
        see "file" "$file"
        retval=$?
        if [[ "$retval" == 0 ]]; then
                echo "Blacklisted file $file was found."
                exit $BLACKLIST_VIOLATED
        fi
done

echo "Dependencies satisfied."
echo "Installing ROS-Orocos-Maestro..."
sudo bash install-fuerte.sh -y
echo "Installing Hubo-ACH..."
sudo bash install-hubo-ach.sh
echo "Installing OpenHUBO..."
bash install-openHubo.sh

echo "Install Complete. Exiting..."
