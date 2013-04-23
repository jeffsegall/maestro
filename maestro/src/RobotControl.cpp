#include "RobotControl.h"
#include <iostream>
using namespace std;

RobotControl::RobotControl(const std::string& name):
    TaskContext(name)
  {
    
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

    this->addOperation("requiresMotion", &RobotControl::requiresMotion, this, RTT::OwnThread)
			.arg("Name", "The name of the motor to check for necessary motion on.");

    this->addOperation("setMaxAccVel", &RobotControl::setMaxAccVel, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Motor", "The motor channel to set maximum velocity and acceleration on.")
			.arg("Max Accel", "Maximum Acceleration in unknown units.")
			.arg("Max Vel", "Maximum Velocity in unknown units.");

    this->addOperation("setPositionGain", &RobotControl::setPositionGain, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Motor", "The motor channel to set gains on.")
			.arg("kP", "Proportion gain.")
			.arg("kI", "Integral gain.")
			.arg("kD", "Derivative gain");

    this->addOperation("debugControl", &RobotControl::debugControl, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Operation", "Operation to perform. Use a value of 0 for a list of commands.");

    this->addOperation("setDelay", &RobotControl::setDelay, this, RTT::OwnThread)
			.arg("Microseconds", "Delay amount in microseconds.");

    this->addOperation("runGesture", &RobotControl::runGesture, this, RTT::OwnThread)
	    	.arg("Name", "The name of the gesture to load.")
			.arg("Board", "The board on which to run the gesture.");

    this->written = 0;
    this->printNow = false;
    this->enableControl = false;
    this->delay = 0;
    this->state = NULL;
    this->interpolation = true;	//Interpret all commands as a final destination with given velocity.
    this->override = true;		//Force homing before allowing enabling. (currently disabled)

    tempOutput.open("/opt/ros/fuerte/stacks/maestro/RobotControlLog.txt");
    vector<string> paths = getGestureScripts(CONFIG_PATH);
    for (int i = 0; i < paths.size(); i++){
		std::cout << "Adding gestures from path: " << paths[i] << std::endl;
		this->getProvider<Scripting>("scripting")->loadPrograms(paths[i]);
    }

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
    hubomsg::HuboState huboState = hubomsg::HuboState();

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
    	if (printNow){
    		tempOutput << "Writing message to " << output.num_joints << " motors." << std::endl;
    	}

		this->huboDownPort->write(output);

        huboOutputQueue->pop();
    }

    if (!achOutputQueue->empty()){

    	hubomsg::AchCommand output = achOutputQueue->front();
    	if (printNow){
    		tempOutput << "Writing command " << output.commandName << " to ach." << std::endl;
    	}

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

	  hubomsg::HuboState huboState = hubomsg::HuboState();
	  huboState = commHandler->getState();
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


		  motor->update(huboState.joints[i].position,
					  huboState.joints[i].velocity,
					  huboState.joints[i].temperature,
					  huboState.joints[i].current);

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
		  std::cout << "Setting position of motor " << name << " to " << value << " ." << std::endl;
		  motor->setGoalPosition(value);
		  //TODO: Check flags before doing anything, or check them before enabling...
		  break;
	  case VELOCITY:
		  std::cout << "Setting velocity of motor " << name << " to " << value << " ." << std::endl;
		  motor->setInterVelocity(value);
		  if (!interpolation)
			  std::cout << "Warning. RobotControl is not currently handling interpolation. " <<
					  "This velocity will not be used until interpolation is enabled." << std::endl;
		  break;
	  default:
		  std::cout << "Motor with name " << name << " has no mutable property named " << property << " ." << std::endl;
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
			  std::cout << "Position of motor " << name << " is " << motor->getPosition() << "." << std::endl;
			  return motor->getPosition();
		  case VELOCITY:
			  std::cout << "Velocity of motor " << name << " is " << motor->getVelocity() << "." << std::endl;
			  return motor->getVelocity();
		  case TEMPERATURE:
			  std::cout << "Temperature of motor " << name << " is " << motor->getTemperature() << "." << std::endl;
			  return motor->getTemperature();
		  case CURRENT:
			  std::cout << "Current of motor " << name << " is " << motor->getCurrent() << "." << std::endl;
			  return motor->getCurrent();
		  case ENABLED:
			  std::cout << "Motor " << name << " is currently " << (motor->isEnabled() ? "enabled." : "disabled.") << std::endl;
			  return motor->isEnabled() ? 1 : 0;
		  case HOMED:
			  std::cout << "Motor " << name << " has " << (motor->isHomed() ? "" : "not ") << "been homed." << std::endl;
			  return motor->isHomed() ? 1 : 0;
		  default:
			  std::cout << "Motor with name " << name << " has no readable property named " << property << " ." << std::endl;
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
			  std::cout << "M_X of Force Torque Sensor " << name << " is " << board->getMX() << "." << std::endl;
			  return board->getMX();
		  case M_Y:
			  std::cout << "M_Y of Force Torque Sensor " << name << " is " << board->getMY() << "." << std::endl;
			  return board->getMY();
		  case F_Z:
			  std::cout << "F_Z of Force Torque Sensor " << name << " is " << board->getFZ() << "." << std::endl;
			  return board->getFZ();
		  default:
			  std::cout << "Force Torque Sensor with name " << name << " has no readable property named " << property << " ." << std::endl;
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
			  std::cout << "X Acceleration of IMU " << name << " is " << board->getXAcc() << "." << std::endl;
			  return board->getXAcc();
		  case Y_ACCEL:
			  std::cout << "Y Acceleration of IMU " << name << " is " << board->getYAcc() << "." << std::endl;
			  return board->getYAcc();
		  case Z_ACCEL:
			  std::cout << "Z Acceleration of IMU " << name << " is " << board->getZAcc() << "." << std::endl;
			  return board->getZAcc();
		  case X_ROTAT:
			  std::cout << "X Rotation of IMU " << name << " is " << board->getXRot() << "." << std::endl;
			  return board->getXRot();
		  case Y_ROTAT:
			  std::cout << "Y Rotation of IMU " << name << " is " << board->getYRot() << "." << std::endl;
			  return board->getYRot();
		  default:
			  std::cout << "IMU with name " << name << " has no readable property named " << property << " ." << std::endl;
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

	  if (name.compare("Enable") == 0){
		  if (motors.count(target) == 0){
			  std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			  return;
		  }
		  output.commandName = "enableJoint";
		  output.jointName = target;

		  HuboMotor* motor = motors[target];

		  if (!motor->isHomed()){
			  std::cout << "Error! Motor " << target << " has not yet been homed. Skipping enabling of this motor." << std::endl;
			  return;
		  }
		  updateState();
		  motor->setGoalPosition(motor->getPosition());
		  motor->setEnabled(true);

	  } else if (name.compare("EnableAll") == 0){
		  output.commandName = "enableAll";
		  for (int i = 0; i < this->state->getBoards().size(); i++){
			  MotorBoard* mb = this->state->getBoards()[i];
			  for (int j = 0; j < mb->getNumChannels(); j++){
			  	  HuboMotor* motor = mb->getMotorByChannel(j);
			  	  if (!motor->isHomed())
					  std::cout << "Error! Motor " << motor->getName() << " has not yet been homed. Skipping enabling of this motor." << std::endl;
			  	  else {
			  		  updateState();
					  motor->setGoalPosition(motor->getPosition());
			  		  motor->setEnabled(true);
			  	  }
			  }
		  }
	  } else if (name.compare("Disable") == 0){

		  if (motors.count(target) == 0){
			  std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			  return;
		  }
		  output.commandName = "disableJoint";
		  output.jointName = target;

		  HuboMotor* motor = motors[target];

		  motor->setEnabled(false);

	  } else if (name.compare("DisableAll") == 0){
		  output.commandName = "disableAll";
		  for (int i = 0; i < this->state->getBoards().size(); i++){
			  MotorBoard* mb = this->state->getBoards()[i];
			  for (int j = 0; j < mb->getNumChannels(); j++){
			  	  HuboMotor* motor = mb->getMotorByChannel(j);
			  	  motor->setEnabled(false);
			  }
		  }
	  } else if (name.compare("Home") == 0){
		  if (motors.count(target) == 0){
			  std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			  return;
		  }

		  output.commandName = "homeJoint";
		  output.jointName = target;

		  HuboMotor* motor = motors[target];
		  motor->setHomed(true);
		  set(target, "position", 0);

	  } else if (name.compare("HomeAll") == 0){
		  output.commandName = "homeAll";
		  for (int i = 0; i < this->state->getBoards().size(); i++){
			  MotorBoard* mb = this->state->getBoards()[i];
			  for (int j = 0; j < mb->getNumChannels(); j++){
				  HuboMotor* motor = mb->getMotorByChannel(j);
				  motor->setHomed(true);
				  set(motor->getName(), "position", 0);
			  }
		  }
		  //TODO: Find a way to pause for a length of time here.
	  } else if (name.compare("ResetJoint") == 0){
		  if (motors.count(target) == 0){
			  std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			  return;
		  }

		  output.commandName = "resetJoint";
		  output.jointName = target;

		  command("Disable", target);
		  set(target, "position", 0);
	  } else if (name.compare("ResetAll") == 0){
		  for (int i = 0; i < this->state->getBoards().size(); i++){
			  MotorBoard* mb = this->state->getBoards()[i];
			  for (int j = 0; j < mb->getNumChannels(); j++){
				  HuboMotor* motor = mb->getMotorByChannel(j);
				  command("ResetJoint", motor->getName());
			  }
		  }
		  return;
	  } else if (name.compare("InitializeSensors") == 0){
		  output.commandName = "initializeSensors";
	  } else if (name.compare("Update") == 0){
		  updateState();
	  } else {
		  std::cout << "Error. No command with name " << name << " is defined for RobotControl. Aborting.";
		  return;
	  }

	  achOutputQueue->push(output);
  }

  void RobotControl::setMode(string mode, bool value){
	  if (mode.compare("Interpolation") == 0){
		  std::cout << "Setting interpolation " << (value ? "on." : "off.") << std::endl;
		  command("DisableAll","");
		  //If we are switching to interpolation, the internal step of each motor must be updated.
		  if (value){
			  hubomsg::HuboState huboState = hubomsg::HuboState();
			  huboState = commHandler->getState();
			  map<string, HuboMotor*> motors = state->getBoardMap();
			  for (int i = 0; i < huboState.joints.size(); i++){
				  if (motors.count(huboState.joints[i].name) == 1){
					  HuboMotor* motor = motors[huboState.joints[i].name];
					  motor->setInterStep(huboState.joints[i].commanded);
				  }
			  }
		  }
		  interpolation = value;
	  } else {
		  std::cout << "RobotControl does not have a mutable mode with name " << mode << "." << std::endl;
	  }
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

  void RobotControl::setMaxAccVel(int board, int motor, int acc, int vel){
	  this->state->getBoardByNumber(board)->setMaxAccVel((char)motor, acc, vel);
  }

  void RobotControl::setPositionGain(int board, int motor, int kp, int ki, int kd){
	  this->state->getBoardByNumber(board)->setPositionGain(motor, kp, ki, kd);
  }

  void RobotControl::runGesture(string name, int board){
	  boost::shared_ptr<Scripting> scripting = this->getProvider<Scripting>("scripting");
	  scripting->startProgram(name);
	  if (!scripting->isProgramRunning(name))
		std::cout << "Error. Program not running." << std::endl;
	  if (scripting->inProgramError(name))
		std::cout << "Error. Program has encountered an error. " << std::endl;
  }

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(RobotControl)
