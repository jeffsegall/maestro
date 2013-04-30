#!/bin/bash


echo "Maestro OpenHUBO Robot Control Run Script"
if [[ $# -lt 1 && ! "$HUBO_ACH_RUNNING" ]]
then
	echo "Usage : $0 <Mode>"
	echo "<Mode>:"
	echo "	virtual : Runs Hubo-Ach in Virtual (No Hardware) Mode"
	echo "	sim	: Runs Hubo-Ach in Simulation (Software) Mode"
	echo "	sim-real: Runs Hubo-Ach in Simulation (Hardware) Mode"
	echo "	real	: Runs Hubo-Ach in Robot Comm (Hardware) Mode"
	exit
fi

source /opt/ros/fuerte/setup.bash 
SKIP_OPEN_HUBO=""
HUBO_ACH_RUNNING="$(pgrep hubo-daemon)"

if [ "$HUBO_ACH_RUNNING" ]; then
	echo "Hubo-Ach daemon is already running. Terminating..."
	hubo-ach killall &> /dev/null
	echo "Hubo-Ach daemon terminated."
fi

case $1 in
	virtual ) 
		xterm -e "hubo-ach virtual" &;;
	sim ) 
		SKIP_OPEN_HUBO=yes;
		export PYTHONPATH="$PYTHONPATH:/usr/lib/python2.7/dist-packages" 
		xterm -e "hubo-ach sim openhubo physics" &;;
	sim-real ) 
		SKIP_OPEN_HUBO=yes; 
		export PYTHONPATH="$PYTHONPATH:/usr/lib/python2.7/dist-packages" 
		xterm -e "hubo-ach sim openhubo nodynamics" &;;
	real ) 
		xterm -e "hubo-ach start" &;;
esac

sleep 3
xterm -e ./run.sh &
sleep 1

if [ -z "$SKIP_OPEN_HUBO" ]; then
	source "$OPENHUBO_DIR/env.sh"
	cd "$OPENHUBO_DIR"/examples
	xterm -e python -i achread.py &
	sleep 1
fi

roscd hubo_ros

xterm -e ./run-feedback.sh &
sleep 1
xterm -e ./run-feedforward.sh &
sleep 1
