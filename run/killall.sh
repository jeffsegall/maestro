#!/bin/bash
#
# Kill script for Maestro and its child processes
#
# Dependencies: 
# 	<Maestro Install Dir>/maestro/utils.sh (not checked)
#
# Blacklist: None
#
#
# Author: Solis Knight
# Date: July 2013
#
#

DEPENDENCY_DIRS=
DEPENDENCY_FILES=
BLACKLISTED_DIRS=
BLACKLISTED_FILES=


PROCESS_NAMES="hubo_ros_feedba hubo_ros_feedfo"


for process in $PROCESS_NAMES; do
	for pid in `pgrep $process`; do
		sudo kill -9 $pid
	done

done




