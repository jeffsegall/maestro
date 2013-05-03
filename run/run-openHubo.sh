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
USE_OPEN_HUBO=""
NO_X=""
HUBO_ACH_RUNNING="$(pgrep hubo-daemon)"

if [ "$HUBO_ACH_RUNNING" ]; then
	echo "Hubo-Ach daemon is already running. Terminating..."
	hubo-ach killall &> /dev/null
	echo "Hubo-Ach daemon terminated."
fi

case $1 in
	virtual ) 
		USE_OPEN_HUBO=true
		xterm -e "hubo-ach virtual" &
		;;	
	sim ) 
		export PYTHONPATH="$PYTHONPATH:/usr/lib/python2.7/dist-packages" 
		xterm -e "hubo-ach sim openhubo physics" &
		;;
	sim-real ) 
		export PYTHONPATH="$PYTHONPATH:/usr/lib/python2.7/dist-packages" 
		xterm -e "hubo-ach sim openhubo nodynamics" &
		;;
	real ) 
		NO_X=true
		sudo openvt -v  -- "`pwd`/hubo-ach.sh"
		;;
esac

sleep 3

if [ ! -z "$USE_OPEN_HUBO" ]; then
	source "$OPENHUBO_DIR/env.sh"
	cd "$OPENHUBO_DIR"/examples
	xterm -e python -i achread.py &
	sleep 1
fi

roscd hubo_ros

if [[ -z "$NO_X" ]]; then
	xterm -e "./run-feedback.sh" &
	sleep 1
	xterm -e "./run-feedforward.sh" &
	sleep 1

else
	echo "Opening feedback channel..."
	sudo openvt -v  -- "`pwd`/run-feedback.sh"
	sleep 2 
	echo "Opening interface channel..."
	sudo openvt -v  -- "`pwd`/run-feedforward.sh"
	sleep 2 
fi

roscd maestro/../run
./run.sh 
