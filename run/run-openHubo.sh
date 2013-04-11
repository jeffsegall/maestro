
echo "Maestro OpenHUBO Robot Control Run Script"
if [ $# -lt 1 ]
then
	echo "Usage : $0 <Mode>"
	echo "<Mode>:"
	echo "	virtual : Runs Hubo-Ach in Virtual (No Hardware) Mode"
	echo "	sim	: Runs Hubo-Ach in Simulation (Hardware) Mode"
	echo "	real	: Runs Hubo-Ach in Robot Comm (Hardware) Mode"
	exit
fi

source /opt/ros/fuerte/setup.bash 

case $1 in
	virtual ) xterm -e "hubo-ach virtual" &;;
	sim ) xterm -e "hubo-ach sim openhubo nodynamics" &;;
	real ) xterm -e "hubo-ach start" &;;
esac

sleep 3
xterm -e ./run.sh &
sleep 1

cd $OPENHUBO_DIR/examples
xterm -e python -i achread.py &
sleep 1

roscd hubo_ros
echo `pwd`
xterm -e ./run-feedback.sh &
sleep 1
xterm -e ./run-feedforward.sh &
sleep 1
