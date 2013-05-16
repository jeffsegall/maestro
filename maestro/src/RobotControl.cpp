/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "RobotControl.h"
#include <iostream>
using namespace std;

RobotControl::RobotControl(const std::string& name) : TaskContext(name) {
    
    this->canUpPort = new InputPort<hubomsg::CanMessage>("can_up");
    //this->canDownPort = new OutputPort<hubomsg::CanMessage>("can_down");
    this->huboUpPort = new InputPort<hubomsg::HuboState>("Hubo/HuboState");
	this->huboDownPort = new OutputPort<hubomsg::HuboCommand>("Hubo/HuboCommand");
	this->achDownPort = new OutputPort<hubomsg::AchCommand>("Hubo/AchCommand");

    this->orOutPort = new InputPort<hubomsg::HuboCmd>("or_out");
    this->orInPort = new OutputPort<hubomsg::HuboCmd>("or_in");
    this->commHandler = new CommHandler(canUpPort, orOutPort, huboUpPort);

    //CAN QUEUES
    this->inputQueue = new queue<hubomsg::CanMessage>();
    this->huboOutputQueue = new queue<hubomsg::HuboCommand>();
    this->achOutputQueue = new queue<hubomsg::AchCommand>();
    
    //CAN PORTS 
    this->addEventPort(*canUpPort);
    this->addEventPort(*huboUpPort);
    this->addPort(*huboDownPort);
    this->addPort(*achDownPort);

    //OPENRAVE PORTS
    this->addEventPort(*orOutPort);
    this->addPort(*orInPort);

    this->addOperation("initRobot", &RobotControl::initRobot, this, RTT::OwnThread)
            .doc("Initialize a robot")
            .arg("Path", "The path to the XML robot representation");

    this->addOperation("setProperty", &RobotControl::set, this, RTT::OwnThread)
    		.doc("Set a property of a robot subsystem")
    		.arg("Name", "The name of the subsystem to set properties for.")
    		.arg("Property", "The name of the property to change. See README for a list of properties and expected values.")
    		.arg("Value", "The value to set the property to.");

    this->addOperation("setProperties", &RobotControl::setProperties, this, RTT::OwnThread)
    		.doc("Set multiple properties of a robot subsystem.")
    		.arg("Names", "The names of the subsystems to set properties for.")
    		.arg("Properties", "The names of the properties to change. See README for a list of mutable properties.")
    		.arg("Values", "The values to set these properties to.");

    this->addOperation("getProperty", &RobotControl::get, this, RTT::OwnThread)
    		.doc("Get the value of a property of a robot subsystem.")
    		.arg("Name", "The name of the subsystem to read values from.")
    		.arg("Property", "The type of value to read. See README for a list of properties and expected values.");

    this->addOperation("command", &RobotControl::command, this, RTT::OwnThread)
    		.doc("Send a command to the robot.")
    		.arg("Name", "The name of the command to send. See README for command list and arguments.")
    		.arg("Target", "The target of the command. Usually a joint name.");

    this->addOperation("setMode", &RobotControl::setMode, this, RTT::OwnThread)
    		.doc("Modify the mode of operation of RobotControl.")
    		.arg("Mode", "The mode to be modified. Not yet documented.")
    		.arg("Value", "New value for mode to take on. Not yet documented.");

    this->addOperation("setAlias", &RobotControl::setAlias, this, RTT::OwnThread)
			.doc("Create an alternate name for a motor, property, sensor, or command.")
			.arg("Name", "The name of said entity currently recognized")
			.arg("Alias", "An alias for said name. The old name will not be overwritten. You can not have the same name for different entities.");

    this->addOperation("requiresMotion", &RobotControl::requiresMotion, this, RTT::OwnThread)
			.arg("Name", "The name of the motor to check for necessary motion on.");

    this->addOperation("debugControl", &RobotControl::debugControl, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Operation", "Operation to perform. Use a value of 0 for a list of commands.");

    this->addOperation("setDelay", &RobotControl::setDelay, this, RTT::OwnThread)
			.arg("Microseconds", "Delay amount in microseconds.");

    this->addOperation("runGesture", &RobotControl::runGesture, this, RTT::OwnThread)
	    	.arg("Name", "The name of the gesture to load.");

    this->addOperation("testStarted", &RobotControl::testStarted, this, RTT::OwnThread);

    this->addOperation("startTest", &RobotControl::startTest, this, RTT::OwnThread)
    		.arg("Target", "Target for test.");

    this->written = 0;
    this->printNow = false;
    this->enableControl = false;
    this->delay = 0;
    this->state = NULL;
    this->interpolation = true;	//Interpret all commands as a final destination with given velocity.
    this->override = true;		//Force homing before allowing enabling. (currently disabled)

    commands["Enable"] = ENABLE;
    commands["EnableAll"] = ENABLEALL;
    commands["Disable"] = DISABLE;
    commands["DisableAll"] = DISABLEALL;
    commands["ResetJoint"] = RESET;
    commands["ResetAll"] = RESETALL;
    commands["Home"] = HOME;
    commands["HomeAll"] = HOMEALL;
    commands["InitializeSensors"] = INITSENSORS;
    commands["Update"] = UPDATE;

    ostringstream logfile;
	logfile << LOG_PATH << "RobotControl.log";
    tempOutput.open(logfile.str().c_str());
    vector<string> paths = getGestureScripts(CONFIG_PATH);
    for (int i = 0; i < paths.size(); i++){
		std::cout << "Adding gestures from path: " << paths[i] << std::endl;
		this->getProvider<Scripting>("scripting")->loadPrograms(paths[i]);
    }

    RUN_TYPE = getRunType(CONFIG_PATH);
}
  
RobotControl::~RobotControl(){}

vector<float> trajectoryValues(string path){
	vector<float> val;

	float f;

	ifstream is;
	is.open(path.c_str());

	while (!is.eof()){
		is >> f;
		val.push_back(f*5.0);
	}
	is.close();

	return val;
}

void RobotControl::updateHook(){
	hubomsg::HuboCmd huboCmd = hubomsg::HuboCmd();
	hubomsg::CanMessage canMessage = hubomsg::CanMessage();
	//hubomsg::HuboState huboState = hubomsg::HuboState();

	commHandler->update();
	if (state == NULL) return;

	if (commHandler->isNew(1)){
		//Received update from CanGateway
		canMessage = commHandler->getMessage(); //Deprecated - Soon to be phased out
	}
	if (commHandler->isNew(2)){
		//Recieved update from openRAVE
		huboCmd = commHandler->getCmd();
	}
	if (commHandler->isNew(3)){
		//Received update from Hubo-Ach
		if (testing && get("RHY","position") == this->target){
			timespec finish;
			clock_gettime(REALTIME, finish);
			testing = false;
			tempOutput << (finish.tv_nsec - startTime) << std::endl;
		}

		updateState();

	}

	if (huboOutputQueue->empty() && !this->state->getBoards().empty()) {
		hubomsg::HuboCommand message;
		for (int i = 0; i < this->state->getBoards().size(); i++){
			MotorBoard* mb = this->state->getBoards()[i];
			for (int j = 0; j < mb->getNumChannels(); j++){
				HuboMotor* motor = mb->getMotorByChannel(j);
				if (motor->isEnabled()){
					hubomsg::HuboJointCommand state;
					state.name = motor->getName();
					state.position = interpolation ? motor->interpolate() : motor->getGoalPosition();
					buildHuboCommandMessage(state, message);
				}
			}
		}
		huboOutputQueue->push(message);

	}

	//Write out a message if we have one

	if (!huboOutputQueue->empty()){
		hubomsg::HuboCommand output = huboOutputQueue->front();
		if (printNow)
			tempOutput << "Writing message to " << output.num_joints << " motors." << std::endl;

		this->huboDownPort->write(output);
		huboOutputQueue->pop();
	}

	if (!achOutputQueue->empty()){
		hubomsg::AchCommand output = achOutputQueue->front();
		if (printNow)
			tempOutput << "Writing command " << output.commandName << " to ach." << std::endl;

		this->achDownPort->write(output);
		achOutputQueue->pop();
	}
	usleep(delay);
}

hubomsg::CanMessage RobotControl::buildCanMessage(canMsg* msg){
	hubomsg::CanMessage canMessage;

	canMessage.bno = msg->getBNO();
	canMessage.mType = msg->getType();
	canMessage.cmdType = msg->getCmd();
	canMessage.r1 = msg->getR1();
	canMessage.r2 = msg->getR2();
	canMessage.r3 = msg->getR3();
	canMessage.r4 = msg->getR4();
	canMessage.r5 = msg->getR5();
	canMessage.r6 = msg->getR6();
	canMessage.r7 = msg->getR7();
	canMessage.r8 = msg->getR8();

	return canMessage;
}

void RobotControl::buildHuboCommandMessage(hubomsg::HuboJointCommand& state, hubomsg::HuboCommand& message){
	message.joints.push_back(state);
	message.num_joints = message.joints.size();
}

vector<string> RobotControl::getGestureScripts(string path){
	vector<string> files;

	ifstream is;
	is.open(path.c_str());
	string temp;
	if (is.is_open()){
		do {
			getline(is, temp, '\n');
		} while (temp.compare("Scripts:") != 0);
		do {
			getline(is, temp, '\n');
			if (temp.compare("") != 0) files.push_back(temp);
		} while (!is.eof());
		is.close();
	} else
		std::cout << "Error. Config file nonexistent. Aborting." << std::endl;

	return files;
}

bool RobotControl::getRunType(string path){
	ifstream is;
	is.open(path.c_str());
	string temp;
	if (is.is_open()){
	  do {
		  getline(is, temp, '\n');
	  } while (temp.compare("RunType:") != 0);
	  getline(is, temp, '\n');
	  is.close();
	  if (temp.compare("Hardware") == 0){
		  return HARDWARE;
	  } else if (temp.compare("Simulation") == 0){
		  return SIMULATION;
	  } else {
		  cout << "Error! Unknown RunType specified in config file! Assuming Simulation.";
		  return SIMULATION;
	  }
	} else
	  std::cout << "Error. Config file nonexistent. Aborting." << std::endl;

	return SIMULATION;
}

string RobotControl::getDefaultInitPath(string path){
	ifstream is;
	is.open(path.c_str());
	string temp;
	if (is.is_open()){
		do {
			getline(is, temp, '\n');
		} while (temp.compare("Init File:") != 0);

		getline(is, temp, '\n');
		is.close();
	} else
	std::cout << "Error. Config file nonexistent. Aborting." << std::endl;

	return temp;
}

void RobotControl::initRobot(string path){
	this->state = new HuboState();
	if (strcmp(path.c_str(), "") == 0)
		path = getDefaultInitPath(CONFIG_PATH);

	//@TODO: Check for file existence before initializing.
	this->state->initHuboWithDefaults(path, this->huboOutputQueue);
}

void RobotControl::updateState(){

	hubomsg::HuboState huboState = commHandler->getState();
	map<string, HuboMotor*> motors = state->getBoardMap();

	if (printNow) std::cout << "Updating Robot State..." << std::endl;

		for (int i = 0; i < huboState.joints.size(); i++){
			if (motors.count(huboState.joints[i].name) == 0){
				if (printNow)
					cout << "Joint with name " << huboState.joints[i].name <<
						" not initialized in RobotControl. Skipping update of this motor." << std::endl;
				continue;
			}

			HuboMotor* motor = motors[huboState.joints[i].name];
			double position = (RUN_TYPE == HARDWARE) ? huboState.joints[i].position : huboState.joints[i].commanded;
			motor->update(position,
						huboState.joints[i].velocity,
						huboState.joints[i].temperature,
						huboState.joints[i].current,
						huboState.joints[i].homed,
						huboState.joints[i].status);

		}

		this->state->getIMUSensorMap()["IMU"]->update(huboState.imu.x_acceleration,
													huboState.imu.y_acceleration,
													huboState.imu.z_acceleration,
													huboState.imu.x_rotation,
													huboState.imu.y_rotation);

		this->state->getIMUSensorMap()["LAI"]->update(huboState.left_foot.x_acceleration,
													huboState.imu.y_acceleration,
													huboState.imu.z_acceleration,
													huboState.imu.x_rotation,
													huboState.imu.y_rotation);

		this->state->getIMUSensorMap()["RAI"]->update(huboState.right_foot.x_acceleration,
													huboState.imu.y_acceleration,
													huboState.imu.z_acceleration,
													huboState.imu.x_rotation,
													huboState.imu.y_rotation);

		this->state->getFTSensorMap()["LAT"]->update(huboState.left_ankle.Mx,
													huboState.left_ankle.My,
													huboState.left_ankle.Fz);

		this->state->getFTSensorMap()["RAT"]->update(huboState.right_ankle.Mx,
													huboState.left_ankle.My,
													huboState.left_ankle.Fz);

		this->state->getFTSensorMap()["LWT"]->update(huboState.left_wrist.Mx,
													huboState.left_ankle.My,
													huboState.left_ankle.Fz);

		this->state->getFTSensorMap()["RWT"]->update(huboState.left_wrist.Mx,
													huboState.left_ankle.My,
													huboState.left_ankle.Fz);

}

void RobotControl::set(string name, string property, double value){
	map<string, HuboMotor*> motors = state->getBoardMap();

	if (motors.count(name) == 0){
		std::cout << "Error. Motor with name " << name << " is not on record. Aborting." << std::endl;
		return;
	}
	HuboMotor* motor = motors[name];

	map<string, PROPERTY> properties = state->getPropertyMap();
	if (properties.count(property) == 0){
		std::cout << "Error. No property with name " << property << " registered. Aborting." << std::endl;
		return;
	}

	switch (properties[property]){
	case POSITION:
		if (printNow) std::cout << "Setting position of motor " << name << " to " << value << " ." << std::endl;
		motor->setGoalPosition(value);
		break;
	case VELOCITY:
		if (printNow) std::cout << "Setting velocity of motor " << name << " to " << value << " ." << std::endl;
		motor->setInterVelocity(value);
		if (!interpolation)
			std::cout << "Warning. RobotControl is not currently handling interpolation. " <<
					"This velocity will not be used until interpolation is enabled." << std::endl;
		break;
	default:
		std::cout << "Motor with name " << name << " has no mutable property named " << property << " ." << std::endl;
		return;
	}
}

void RobotControl::setProperties(string names, string properties, string values){
	vector<string> namesList = splitFields(names);
	vector<string> propertiesList = splitFields(properties);
	vector<string> valuesList = splitFields(values);
	if (namesList.size() != propertiesList.size()
			|| namesList.size() != valuesList.size()
			|| propertiesList.size() != valuesList.size()){
		cout << "Error! Size of entered fields not consistent. Aborting.";
		return;
	}

	for (int i = 0; i < namesList.size(); i++){
		istringstream data(valuesList[i]);
		double value = 0;
		data >> value;

		set(namesList[i], propertiesList[i], value);
	}
}

double RobotControl::get(string name, string property){
	map<string, HuboMotor*> motors = state->getBoardMap();
	map<string, FTSensorBoard*> ftSensors = state->getFTSensorMap();
	map<string, IMUBoard*> imuSensors = state->getIMUSensorMap();

	if (motors.count(name) == 1){
		HuboMotor* motor = motors[name];
		map<string, PROPERTY> properties = state->getPropertyMap();

		if (properties.count(property) == 0){
			std::cout << "Error. No property with name " << property << " registered. Aborting." << std::endl;
			return 0;
		}

		switch (properties[property]){
		case POSITION:
			if (printNow) std::cout << "Position of motor " << name << " is " << motor->getPosition() << "." << std::endl;
			return motor->getPosition();
		case VELOCITY:
			if (printNow) std::cout << "Velocity of motor " << name << " is " << motor->getVelocity() << "." << std::endl;
			return motor->getVelocity();
		case TEMPERATURE:
			if (printNow) std::cout << "Temperature of motor " << name << " is " << motor->getTemperature() << "." << std::endl;
			return motor->getTemperature();
		case CURRENT:
			if (printNow) std::cout << "Current of motor " << name << " is " << motor->getCurrent() << "." << std::endl;
			return motor->getCurrent();
		case ENABLED:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->isEnabled() ? "enabled." : "disabled.") << std::endl;
			return motor->isEnabled() ? 1 : 0;
		case HOMED:
			if (printNow) std::cout << "Motor " << name << " has " << (motor->isHomed() ? "" : "not ") << "been homed." << std::endl;
			return motor->isHomed() ? 1 : 0;
		case ERRORED:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError() ? "" : "not ") << "in an error condition.";
			return motor->hasError() ? 1 : 0;
		case JAM_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a jam error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case PWM_SATURATED_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a PWM Saturated error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case BIG_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a big error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case ENC_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing an encoder error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case DRIVE_FAULT_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a drive fault.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case POS_MIN_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a minimum position error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case POS_MAX_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a maximum position error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case VELOCITY_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a velocity error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case ACCELERATION_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing an acceleration error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		case TEMP_ERROR:
			if (printNow) std::cout << "Motor " << name << " is currently " << (motor->hasError(properties[property]) ? "" : "not ") << "experiencing a temperature error.";
			return motor->hasError(properties[property]) ? 1 : 0;
		default:
			if (printNow) std::cout << "Motor with name " << name << " has no readable property named " << property << " ." << std::endl;
			return 0;
		}
	} else if (ftSensors.count(name) == 1){
		FTSensorBoard* board = ftSensors[name];
		map<string, PROPERTY> properties = state->getPropertyMap();

		if (properties.count(property) == 0){
			std::cout << "Error. No property with name " << property << " registered. Aborting." << std::endl;
			return 0;
		}

		switch (properties[property]){
		case M_X:
			if (printNow) std::cout << "M_X of Force Torque Sensor " << name << " is " << board->getMX() << "." << std::endl;
			return board->getMX();
		case M_Y:
			if (printNow) std::cout << "M_Y of Force Torque Sensor " << name << " is " << board->getMY() << "." << std::endl;
			return board->getMY();
		case F_Z:
			if (printNow) std::cout << "F_Z of Force Torque Sensor " << name << " is " << board->getFZ() << "." << std::endl;
			return board->getFZ();
		default:
			if (printNow) std::cout << "Force Torque Sensor with name " << name << " has no readable property named " << property << " ." << std::endl;
			return 0;
		}
	} else if (imuSensors.count(name) == 1){
		IMUBoard* board = imuSensors[name];
		map<string, PROPERTY> properties = state->getPropertyMap();

		if (properties.count(property) == 0){
			std::cout << "Error. No property with name " << property << " registered. Aborting." << std::endl;
			return 0;
		}

		switch (properties[property]){
		case X_ACCEL:
			if (printNow) std::cout << "X Acceleration of IMU " << name << " is " << board->getXAcc() << "." << std::endl;
			return board->getXAcc();
		case Y_ACCEL:
			if (printNow) std::cout << "Y Acceleration of IMU " << name << " is " << board->getYAcc() << "." << std::endl;
			return board->getYAcc();
		case Z_ACCEL:
			if (printNow) std::cout << "Z Acceleration of IMU " << name << " is " << board->getZAcc() << "." << std::endl;
			return board->getZAcc();
		case X_ROTAT:
			if (printNow) std::cout << "X Rotation of IMU " << name << " is " << board->getXRot() << "." << std::endl;
			return board->getXRot();
		case Y_ROTAT:
			if (printNow) std::cout << "Y Rotation of IMU " << name << " is " << board->getYRot() << "." << std::endl;
			return board->getYRot();
		default:
			if (printNow) std::cout << "IMU with name " << name << " has no readable property named " << property << " ." << std::endl;
			return 0;
		}
	} else {
		std::cout << "Error. Readable Object with name " << name << " is not on record. Aborting." << std::endl;
		return 0;
	}
}

void RobotControl::command(string name, string target){
	map<string, HuboMotor*> motors = state->getBoardMap();
	hubomsg::AchCommand output;

	HuboMotor* motor;
	MotorBoard* mb;

	if (commands.count(name) == 0){
		std::cout << "Error. No command with name " << name << " is defined for RobotControl. Aborting.";
		return;
	}

	switch (commands[name]){
	case ENABLE:
		if (motors.count(target) == 0){
			std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			return;
		}
		output.commandName = "enableJoint";
		output.jointName = target;
		achOutputQueue->push(output);

		motor = motors[target];
		updateState();
		if (RUN_TYPE == HARDWARE && !motor->isHomed()){
			std::cout << "Warning! Motor " << target << " has not yet been homed. Skipping enabling of this motor." << std::endl;
			return;
		}
		if (RUN_TYPE == HARDWARE)
			motor->setGoalPosition(motor->getPosition());
		motor->setEnabled(true);
		break;
	case ENABLEALL:
		output.commandName = "enableAll";
		achOutputQueue->push(output);
		for (int i = 0; i < this->state->getBoards().size(); i++){
			mb = this->state->getBoards()[i];
			for (int j = 0; j < mb->getNumChannels(); j++){
				motor = mb->getMotorByChannel(j);
				updateState();
				if (RUN_TYPE == HARDWARE && !motor->isHomed()){
					std::cout << "Warning! Motor " << motor->getName() << " has not yet been homed. Skipping enabling of this motor." << std::endl;
					continue;
				}
				if (RUN_TYPE == HARDWARE)
					motor->setGoalPosition(motor->getPosition());
				motor->setEnabled(true);
			}
		}
		break;
	case DISABLE:
		if (motors.count(target) == 0){
			std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			return;
		}
		output.commandName = "disableJoint";
		output.jointName = target;
		achOutputQueue->push(output);

		motor = motors[target];
		motor->setEnabled(false);
		break;
	case DISABLEALL:
		output.commandName = "disableAll";
		achOutputQueue->push(output);
		for (int i = 0; i < this->state->getBoards().size(); i++){
			mb = this->state->getBoards()[i];
			for (int j = 0; j < mb->getNumChannels(); j++){
				motor = mb->getMotorByChannel(j);
				motor->setEnabled(false);
			}
		}
		break;
	case RESET:
		if (motors.count(target) == 0){
			std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			return;
		}

		output.commandName = "resetJoint";
		output.jointName = target;
		achOutputQueue->push(output);
		break;
	case RESETALL:
		for (int i = 0; i < this->state->getBoards().size(); i++){
			mb = this->state->getBoards()[i];
			for (int j = 0; j < mb->getNumChannels(); j++){
				motor = mb->getMotorByChannel(j);
				command("ResetJoint", motor->getName());
			}
		}
		return;
	case HOME:
		if (motors.count(target) == 0){
			std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			return;
		}

		output.commandName = "homeJoint";
		output.jointName = target;
		achOutputQueue->push(output);

		motor = motors[target];
		set(target, "position", 0);
		break;
	case HOMEALL:
		output.commandName = "homeAll";
		achOutputQueue->push(output);
		for (int i = 0; i < this->state->getBoards().size(); i++){
			mb = this->state->getBoards()[i];
			for (int j = 0; j < mb->getNumChannels(); j++){
				motor = mb->getMotorByChannel(j);
				set(motor->getName(), "position", 0);
			}
		}
		break;
		//TODO: Find a way to pause for a length of time here.
	case INITSENSORS:
		output.commandName = "initializeSensors";
		achOutputQueue->push(output);
		break;
	case UPDATE:
		updateState();
		break;
	}
}

void RobotControl::setMode(string mode, bool value){
	if (mode.compare("Interpolation") == 0){
		if (printNow) std::cout << "Setting interpolation " << (value ? "on." : "off.") << std::endl;
		command("DisableAll","");
		//If we are switching to interpolation, the internal step of each motor must be updated.
		if (value){
			hubomsg::HuboState huboState = hubomsg::HuboState();
			huboState = commHandler->getState();
			map<string, HuboMotor*> motors = state->getBoardMap();
			for (int i = 0; i < huboState.joints.size(); i++){
				if (motors.count(huboState.joints[i].name) == 1){
					HuboMotor* motor = motors[huboState.joints[i].name];
					motor->setInterStep(RUN_TYPE == SIMULATION ? huboState.joints[i].commanded : huboState.joints[i].position);
				}
			}
		}
		interpolation = value;
	} else {
		std::cout << "RobotControl does not have a mutable mode with name " << mode << "." << std::endl;
	}
}

bool RobotControl::setAlias(string name, string alias){
	map<string, HuboMotor*> motors = state->getBoardMap();
	map<string, FTSensorBoard*> ftSensors = state->getFTSensorMap();
	map<string, IMUBoard*> imuSensors = state->getIMUSensorMap();
	map<string, PROPERTY> properties = state->getPropertyMap();

	int entries = 0;

	entries += motors.count(alias);
	entries += ftSensors.count(alias);
	entries += imuSensors.count(alias);
	entries += properties.count(alias);
	entries += commands.count(alias);

	if (entries > 0){
		std::cout << "There already exists an entity named " << alias << " in RobotControl." << std::cout;
		return false;
	} else if (motors.count(name) == 1)
		motors[alias] = motors[name];
	else if (ftSensors.count(name) == 1)
		ftSensors[alias] = ftSensors[name];
	else if (imuSensors.count(name) == 1)
		imuSensors[alias] = imuSensors[name];
	else if (properties.count(name) == 1)
		properties[alias] = properties[name];
	else if (commands.count(name) == 1)
		commands[alias] = commands[name];

	return true;
}

vector<string> RobotControl::splitFields(string input){
	vector<string> output;
	int whitespaceType = 0;
	if (input.find(' ') != string::npos) whitespaceType += 1;
	if (input.find('\t') != string::npos) whitespaceType += 2;
	if (input.find('\n') != string::npos) whitespaceType += 4;

	char whitespace;
	switch (whitespaceType){
	case 1:
		whitespace = ' ';
		break;
	case 2:
		whitespace = '\t';
		break;
	case 4:
		whitespace = '\n';
		break;
	default:
		output.push_back(input);
		return output;
	}
	string field;
	int pos = 0;

	do {
		pos = input.find(whitespace);
		field = input.substr(0, pos);
		output.push_back(field);
		input = input.substr(pos + 1, input.length() - pos - 1);
	} while (input.find(whitespace) != string::npos);
	output.push_back(input);
	return output;
}

void RobotControl::debugControl(int board, int operation){
	switch (operation) {
	case 1:
		this->state->getBoardByNumber(board)->setHIP(0);
		break;
	case 2:
		this->state->getBoardByNumber(board)->disableController();
		break;
	case 3:
		this->state->getBoardByNumber(board)->setHIP(1);
		break;
	case 4:
		this->state->getBoardByNumber(board)->enableController();
		break;
	case 5:
		this->printNow = true;
		break;
	case 6:
		this->printNow = false;
		break;
	default:
		std::cout << "Operations: " << std::endl << "1: disable (step 1)    2: disable (step 2)    3: enable (step 1)    4: enable (step 2)    5: enable printing     6: disable printing";
	}
}

void RobotControl::setDelay(int us){
	this->delay = us;
}

bool RobotControl::requiresMotion(string name){
	map<string, HuboMotor*> motors = state->getBoardMap();
	if (motors.count(name)  == 0){
		std::cout << "Error. Motor with name " << name << " is not on record. Aborting." << std::endl;
		return false;
	}
	return motors[name]->requiresMotion();
}

bool RobotControl::testStarted(){
	return testing;
}

void RobotControl::startTest(double target){
	timespec start;
	this->target = target;
	this->testing = true;
	set("RHY", "position", target);
	clock_gettime(REALTIME, start);
	startTime = start.tv_nsec;
}

void RobotControl::runGesture(string name){
	boost::shared_ptr<Scripting> scripting = this->getProvider<Scripting>("scripting");
	scripting->startProgram(name);
	if (!scripting->isProgramRunning(name))
		std::cout << "Error. Program not running." << std::endl;
	if (scripting->inProgramError(name))
		std::cout << "Error. Program has encountered an error. " << std::endl;
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(RobotControl)
