#!/bin/bash
echo "Maestro OpenHUBO Robot Control Run Script"

source /opt/ros/fuerte/setup.bash 

#Check if hubo-ach daemon is already running
if [ ! "$(pgrep hubo-daemon)" ]
then
        # Not running
	if [ $# -lt 1 ]
	then
		echo "Usage : $0 <Mode>"
		echo "<Mode>:"
		echo "	virtual : Runs Hubo-Ach in Virtual (No Hardware) Mode"
		echo "	sim	: Runs Hubo-Ach in Simulation (Hardware) Mode"
		echo "	real	: Runs Hubo-Ach in Robot Comm (Hardware) Mode"
		exit
	fi

	case $1 in
		virtual ) xterm -e "hubo-ach virtual" &;;
		sim ) xterm -e "hubo-ach sim openhubo nophysics" &;;
		real ) xterm -e "hubo-ach start" &;;
	esac
	sleep 10
else
	# Running
	echo "hubo-daemon is already running. Using currently running instance.";
fi

xterm -e ./run.sh &
sleep 1

#cd $OPENHUBO_DIR/examples
#xterm -e python -i achread.py &
#sleep 1

roscd hubo_ros
echo `pwd`
xterm -e ./run-feedback.sh &
sleep 1
xterm -e ./run-feedforward.sh &
sleep 1
