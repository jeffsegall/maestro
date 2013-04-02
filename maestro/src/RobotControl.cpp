#include "RobotControl.h"
#include <iostream>
using namespace std;

RobotControl::RobotControl(const std::string& name):
    TaskContext(name)
  {
    
    this->canUpPort = new InputPort<hubomsg::CanMessage>("can_up");
    //this->canDownPort = new OutputPort<hubomsg::CanMessage>("can_down");
	this->huboDownPort = new OutputPort<hubomsg::HuboCommand>("Hubo/HuboCommand");
    this->orOutPort = new InputPort<hubomsg::HuboCmd>("or_out");
    this->orInPort = new OutputPort<hubomsg::HuboCmd>("or_in");
    this->commHandler = new CommHandler(canUpPort, orOutPort);

    //CAN QUEUES
    this->inputQueue = new queue<hubomsg::CanMessage>();
    this->outputQueue = new queue<hubomsg::HuboCommand>();
    
    //CAN PORTS 
    this->addEventPort(*canUpPort);
    this->addPort(*huboDownPort);

    //OPENRAVE PORTS
    this->addEventPort(*orOutPort);
    this->addPort(*orInPort);

    this->addOperation("setWaist", &RobotControl::setWaist, this, RTT::OwnThread)
            .doc("Set Torso Yaw")
            .arg("Value", "New ticks for torso yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setWaistRad", &RobotControl::setWaistRad, this, RTT::OwnThread)
                .doc("Set Torso Yaw")
                .arg("Rads", "New radians for torso yaw.")
                .arg("Timestamp", "Timestamp delay (in milliseconds)");
  
    this->addOperation("setNeck", &RobotControl::setNeck, this, RTT::OwnThread)
            .doc("Set Neck positions")
            .arg("Neck", "New ticks for neck.")
            .arg("One", "New ticks for one.")
            .arg("Two", "New ticks for two.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
 
    this->addOperation("setLeftShoulderRoll", &RobotControl::setLeftShoulderRoll, this, RTT::OwnThread)
            .doc("Set Left Shoulder Roll")
            .arg("Value", "New ticks for left shoulder roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
    
    this->addOperation("setLeftShoulderRollRad", &RobotControl::setLeftShoulderRollRad, this, RTT::OwnThread)
            .doc("Set Left Shoulder Roll")
            .arg("Rads", "New radians for left shoulder roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftShoulderPitch", &RobotControl::setLeftShoulderPitch, this, RTT::OwnThread)
            .doc("Set Left Shoulder Pitch")
            .arg("Value", "New ticks for left shoulder pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftShoulderPitchRad", &RobotControl::setLeftShoulderPitchRad, this, RTT::OwnThread)
            .doc("Set Left Shoulder Pitch")
            .arg("Rads", "New radians for left shoulder pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
   
    this->addOperation("setLeftShoulderYaw", &RobotControl::setLeftShoulderYaw, this, RTT::OwnThread)
            .doc("Set Left Shoulder Yaw")
            .arg("Value", "New ticks for left shoulder yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftShoulderYawRad", &RobotControl::setLeftShoulderYawRad, this, RTT::OwnThread)
            .doc("Set Left Shoulder Yaw")
            .arg("Rads", "New radians for left shoulder yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
   
    this->addOperation("setLeftElbow", &RobotControl::setLeftElbow, this, RTT::OwnThread)
            .doc("Set Left Elbow")
            .arg("Value", "New ticks for left elbow.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftElbowRad", &RobotControl::setLeftElbowRad, this, RTT::OwnThread)
            .doc("Set Left Elbow")
            .arg("Rads", "New radians for left elbow.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftWristPitch", &RobotControl::setLeftWristPitch, this, RTT::OwnThread)
            .doc("Set Left Wrist Pitch")
            .arg("Value", "New ticks for left wrist pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftWristPitchRad", &RobotControl::setLeftWristPitchRad, this, RTT::OwnThread)
            .doc("Set Left Wrist Pitch")
            .arg("Rads", "New radians for left wrist pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftWristYaw", &RobotControl::setLeftWristYaw, this, RTT::OwnThread)
            .doc("Set Left Wrist Yaw")
            .arg("Value", "New ticks for left Wrist Yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftWristYawRad", &RobotControl::setLeftWristYawRad, this, RTT::OwnThread)
            .doc("Set Left Wrist Yaw")
            .arg("Rads", "New radians for left Wrist Yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderRoll", &RobotControl::setRightShoulderRoll, this, RTT::OwnThread)
            .doc("Set Right Shoulder Roll")
            .arg("Value", "New ticks for right shoulder roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderRollRad", &RobotControl::setRightShoulderRollRad, this, RTT::OwnThread)
            .doc("Set Right Shoulder Roll")
            .arg("Rads", "New radians for right shoulder roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderPitch", &RobotControl::setRightShoulderPitch, this, RTT::OwnThread)
            .doc("Set Right Shoulder Pitch")
            .arg("Value", "New ticks for right shoulder pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderPitchRad", &RobotControl::setRightShoulderPitchRad, this, RTT::OwnThread)
            .doc("Set Right Shoulder Pitch")
            .arg("Rads", "New radians for right shoulder pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderYaw", &RobotControl::setRightShoulderYaw, this, RTT::OwnThread)
            .doc("Set Right Shoulder Yaw")
            .arg("Value", "New ticks for right shoulder yaw.")			
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderYawRad", &RobotControl::setRightShoulderYawRad, this, RTT::OwnThread)
            .doc("Set Right Shoulder Yaw")
            .arg("Rads", "New radians for right shoulder yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
			
    this->addOperation("setRightElbow", &RobotControl::setRightElbow, this, RTT::OwnThread)
            .doc("Set Right Elbow")
            .arg("Value", "New ticks for right elbow.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightElbowRad", &RobotControl::setRightElbowRad, this, RTT::OwnThread)
            .doc("Set Right Elbow")
            .arg("Rads", "New radians for right elbow.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightWristPitch", &RobotControl::setRightWristPitch, this, RTT::OwnThread)
            .doc("Set Right Wrist Pitch")
            .arg("Value", "New ticks for right wrist pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightWristPitchRad", &RobotControl::setRightWristPitchRad, this, RTT::OwnThread)
            .doc("Set Right Wrist Pitch")
            .arg("Rads", "New radians for right wrist pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightWristYaw", &RobotControl::setRightWristYaw, this, RTT::OwnThread)
            .doc("Set Right Wrist Yaw")
            .arg("Value", "New ticks for right wrist yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightWristYawRad", &RobotControl::setRightWristYawRad, this, RTT::OwnThread)
            .doc("Set Right Wrist Yaw")
            .arg("Rads", "New radians for right wrist yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipYaw", &RobotControl::setLeftHipYaw, this, RTT::OwnThread)
            .doc("Set Left Hip Yaw")
            .arg("Value", "New ticks for left hip yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipYawRad", &RobotControl::setLeftHipYawRad, this, RTT::OwnThread)
            .doc("Set Left Hip Yaw")
            .arg("Rads", "New radians for left hip yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipRoll", &RobotControl::setLeftHipRoll, this, RTT::OwnThread)
            .doc("Set Left Hip Roll")
            .arg("Value", "New ticks for left hip roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipRollRad", &RobotControl::setLeftHipRollRad, this, RTT::OwnThread)
            .doc("Set Left Hip Roll")
            .arg("Rads", "New radians for left hip roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipPitch", &RobotControl::setLeftHipPitch, this, RTT::OwnThread)
            .doc("Set Left Hip Pitch")
            .arg("Value", "New ticks for left hip pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipPitchRad", &RobotControl::setLeftHipPitchRad, this, RTT::OwnThread)
            .doc("Set Left Hip Pitch")
            .arg("Rads", "New radians for left hip pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftKnee", &RobotControl::setLeftKnee, this, RTT::OwnThread)
            .doc("Set Left Knee")
            .arg("Value", "New ticks for left knee.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftKneeRad", &RobotControl::setLeftKneeRad, this, RTT::OwnThread)
            .doc("Set Left Knee")
            .arg("Rads", "New radians for left knee.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftAnklePitch", &RobotControl::setLeftAnklePitch, this, RTT::OwnThread)
            .doc("Set Left Ankle Pitch")
            .arg("Value", "New ticks for left ankle pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftAnklePitchRad", &RobotControl::setLeftAnklePitchRad, this, RTT::OwnThread)
            .doc("Set Left Ankle Pitch")
            .arg("Rads", "New radians for left ankle pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftAnkleRoll", &RobotControl::setLeftAnkleRoll, this, RTT::OwnThread)
            .doc("Set Left Ankle Roll")
            .arg("Value", "New ticks for left ankle roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftAnkleRollRad", &RobotControl::setLeftAnkleRollRad, this, RTT::OwnThread)
            .doc("Set Left Ankle Roll")
            .arg("Rads", "New radians for left ankle roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipYaw", &RobotControl::setRightHipYaw, this, RTT::OwnThread)
            .doc("Set Right Hip Yaw")
            .arg("Value", "New ticks for right hip yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipYawRad", &RobotControl::setRightHipYawRad, this, RTT::OwnThread)
			.doc("Set Right Hip Yaw")
			.arg("Rads", "New radians for right hip yaw.")
			.arg("Timestamp", "Timestamp delay (in milliseconds)");

	this->addOperation("setRightHipRoll", &RobotControl::setRightHipRoll, this, RTT::OwnThread)
			.doc("Set Right Hip Roll")
			.arg("Value", "New ticks for right hip roll.")
			.arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipRollRad", &RobotControl::setRightHipRollRad, this, RTT::OwnThread)
			.doc("Set Right Hip Yaw")
			.arg("Rads", "New radians for right hip roll.")
			.arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipPitch", &RobotControl::setRightHipPitch, this, RTT::OwnThread)
            .doc("Set Right Hip Pitch")
            .arg("Value", "New ticks for right hip pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipPitchRad", &RobotControl::setRightHipPitchRad, this, RTT::OwnThread)
            .doc("Set Right Hip Pitch")
            .arg("Rads", "New radians for right hip pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightKnee", &RobotControl::setRightKnee, this, RTT::OwnThread)
            .doc("Set Right Knee")
            .arg("Value", "New ticks for right knee.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightKneeRad", &RobotControl::setRightKneeRad, this, RTT::OwnThread)
            .doc("Set Right Knee")
            .arg("Rads", "New radians for right knee.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightAnklePitch", &RobotControl::setRightAnklePitch, this, RTT::OwnThread)
            .doc("Set Right Ankle Pitch")
            .arg("Value", "New ticks for right ankle pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightAnklePitchRad", &RobotControl::setRightAnklePitchRad, this, RTT::OwnThread)
            .doc("Set Right Ankle Pitch")
            .arg("Rads", "New radians for right ankle pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightAnkleRoll", &RobotControl::setRightAnkleRoll, this, RTT::OwnThread)
            .doc("Set Right Ankle Roll")
            .arg("Value", "New ticks for right ankle roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightAnkleRollRad", &RobotControl::setRightAnkleRollRad, this, RTT::OwnThread)
            .doc("Set Right Ankle Roll")
            .arg("Rads", "New radians for right ankle roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("initRobot", &RobotControl::initRobot, this, RTT::OwnThread)
            .doc("Initialize a robot")
            .arg("Path", "The path to the XML robot representation");

    this->addOperation("enable", &RobotControl::enable, this, RTT::OwnThread)
            .arg("Board", "The board to enable")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("disable", &RobotControl::disable, this, RTT::OwnThread)
            .arg("Board", "The board to disable")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("requestEncoderPosition", &RobotControl::requestEncoderPosition, this, RTT::OwnThread)
			.arg("Board", "The board to request encoder data from")
			.arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("getCurrentTicks", &RobotControl::getCurrentTicks, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Motor", "The motor channel to request current perceived position from.")
			.arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setCurrentTicks", &RobotControl::setCurrentTicks, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Motor", "The motor channel to set current perceived position on.")
			.arg("Ticks", "Desired current perceived position in ticks.");

    this->addOperation("getCurrentGoal", &RobotControl::getCurrentGoal, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Motor", "The motor channel to request current requested position from.")
			.arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("requiresMotion", &RobotControl::requiresMotion, this, RTT::OwnThread)
			.arg("Board", "The board to send commands to")
			.arg("Motor", "The motor channel to check need for motion from.")
			.arg("Timestamp", "Timestamp delay (in milliseconds)");

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
    CONFIG_PATH = "/opt/ros/fuerte/stacks/maestro/test/config.txt";
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

    //commHandler->update();

    MotorBoard* mb;
    if (state != NULL)
    	mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);

    if (false /*commHandler->isNew()*/){
        //Received update from CanGateway

    	canMessage = commHandler->getMessage();

    	if (canMessage.mType == RX_ENC_VAL+BNO_R_HIP_YAW_ROLL){
    		long yaw_ticks = canMessage.r1;
    		long roll_ticks = canMessage.r2;
    		vector<long> ticks(2);
    		ticks[0] = yaw_ticks;
    		ticks[1] = roll_ticks;
    		std::cout << "Encoder Position value received! ticks: " << std::endl << "yaw: " << yaw_ticks << std::endl << "roll: " << roll_ticks << std::endl;
    		std::cout << "In Radians: yaw: " << mb->getMotorByChannel(0)->ticksToRadians(ticks[0]) << std::endl << "roll: " << mb->getMotorByChannel(0)->ticksToRadians(ticks[1]) << std::endl;
    	}
    }
    if (NewData == this->orOutPort->read(huboCmd)){
        //Recieved update from openRAVE
    }

    if (outputQueue->empty() && state != NULL && !this->state->getBoards().empty()) {
		//tempOutput << "Boards not empty. Map size: " << this->state->getBoards().size() << std::endl;
    	hubomsg::HuboCommand message;
		for (int i = 0; i < this->state->getBoards().size(); i++){
			//if (this->state->getBoards()[i]->requiresMotion()){
				//tempOutput << "Attempting to build message for :" << this->state->getBoards()[i]->getBoardNumber() << std::endl;
				vector<hubomsg::HuboJointCommand> states = this->state->getBoards()[i]->sendPositionReference();
				buildHuboCommandMessage(states, message);

			//}
		}
		outputQueue->push(message);

	}
   
    //Write out a message if we have one

    if (!outputQueue->empty()){

    	hubomsg::HuboCommand output = outputQueue->front();
    	if (printNow){
    		tempOutput << "Writing message to " << output.commanded << " motors." << std::endl;
    	}

		this->huboDownPort->write(output);

        outputQueue->pop();
        usleep(delay);
    }
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

  void RobotControl::buildHuboCommandMessage(vector<hubomsg::HuboJointCommand>& states, hubomsg::HuboCommand& message){
      for (int i = 0; i < states.size(); i++)
    	  message.joints.push_back(states[i]);
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
      this->state->initHuboWithDefaults(path, this->outputQueue);
  }

  void RobotControl::setWaist(int ticks, int delay){
      //ros_gateway->transmit(0,ticks);
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_WAIST);
	  mb->getMotorByChannel(0)->setDesiredPosition(ticks);
  }

  void RobotControl::setWaistRad(double rads, int delay){
        //ros_gateway->transmit(0,ticks);
  	  HuboMotor* motor = this->state->getBoardByNumber(BNO_WAIST)->getMotorByChannel(0);
  	  motor->setDesiredPosition(motor->radiansToTicks(rads));
  }

  void RobotControl::setNeck(int ticks, int one, int two, int delay){
	  this->state->getBoardByNumber(BNO_NECK_YAW_1_2)->getMotorByChannel(0)->setDesiredPosition(ticks);
	  this->state->getBoardByNumber(BNO_NECK_YAW_1_2)->getMotorByChannel(1)->setDesiredPosition(one);
	  this->state->getBoardByNumber(BNO_NECK_YAW_1_2)->getMotorByChannel(1)->setDesiredPosition(two);
     //ros_gateway->transmit(1,ticks);
  }

  void RobotControl::setLeftShoulderRoll(int ticks, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_SHOULDER_PITCH_ROLL)->getMotorByChannel(1);
      motor->setDesiredPosition(ticks);
     //ros_gateway->transmit(3,ticks);
  }

  void RobotControl::setLeftShoulderRollRad(double rads, int delay){
	  HuboMotor* motor = this->state->getBoardByNumber(BNO_L_SHOULDER_PITCH_ROLL)->getMotorByChannel(1);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
       //ros_gateway->transmit(3,ticks);
  }

  void RobotControl::setLeftShoulderPitch(int ticks, int delay){
	  HuboMotor* motor = this->state->getBoardByNumber(BNO_L_SHOULDER_PITCH_ROLL)->getMotorByChannel(0);
      motor->setDesiredPosition(ticks);
     //ros_gateway->transmit(4,ticks);
  }

  void RobotControl::setLeftShoulderPitchRad(double rads, int delay){
	  HuboMotor* motor = this->state->getBoardByNumber(BNO_L_SHOULDER_PITCH_ROLL)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
       //ros_gateway->transmit(4,ticks);
  }
  
  void RobotControl::setLeftShoulderYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_SHOULDER_YAW_ELBOW);
	  mb->getMotorByChannel(0)->setDesiredPosition(ticks);
     //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftShoulderYawRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_SHOULDER_YAW_ELBOW)->getMotorByChannel(0);
  	  motor->setDesiredPosition(motor->radiansToTicks(rads));
       //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftElbow(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_SHOULDER_YAW_ELBOW);
	  mb->getMotorByChannel(1)->setDesiredPosition(ticks);
     //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftElbowRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_SHOULDER_YAW_ELBOW)->getMotorByChannel(1);
  	  motor->setDesiredPosition(motor->radiansToTicks(rads));
       //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftWristPitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_WRIST_YAW_PITCH);
	  mb->getMotorByChannel(1)->setDesiredPosition(ticks);
     //ros_gateway->transmit(8,ticks);
  }

  void RobotControl::setLeftWristPitchRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_WRIST_YAW_PITCH)->getMotorByChannel(1);
	  motor->setDesiredPosition(motor->radiansToTicks(rads));
     //ros_gateway->transmit(8,ticks);
  }

  void RobotControl::setLeftWristYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_WRIST_YAW_PITCH);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(9,ticks);
  }

  void RobotControl::setLeftWristYawRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_WRIST_YAW_PITCH)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(9,ticks);
  }

  void RobotControl::setRightShoulderRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_PITCH_ROLL);
	  mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(11,ticks);
  }

  void RobotControl::setRightShoulderRollRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_SHOULDER_PITCH_ROLL)->getMotorByChannel(1);
	  motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(11,ticks);
  }

  void RobotControl::setRightShoulderPitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_PITCH_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(12,ticks);
  }

  void RobotControl::setRightShoulderPitchRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_SHOULDER_PITCH_ROLL)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(12,ticks);
  }

  void RobotControl::setRightShoulderYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_YAW_ELBOW);
	  mb->getMotorByChannel(0)->setDesiredPosition(ticks);
  }
  
  void RobotControl::setRightShoulderYawRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_SHOULDER_YAW_ELBOW)->getMotorByChannel(0);
	  motor->setDesiredPosition(motor->radiansToTicks(rads));
  }

  void RobotControl::setRightElbow(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_YAW_ELBOW);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(14,ticks);
  }

  void RobotControl::setRightElbowRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_SHOULDER_YAW_ELBOW)->getMotorByChannel(1);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(14,ticks);
  }

  void RobotControl::setRightWristPitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_WRIST_YAW_PITCH);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(16,ticks);
  }

  void RobotControl::setRightWristPitchRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_WRIST_YAW_PITCH)->getMotorByChannel(1);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(16,ticks);
  }

  void RobotControl::setRightWristYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_WRIST_YAW_PITCH);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(17,ticks);
  }

  void RobotControl::setRightWristYawRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_WRIST_YAW_PITCH)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(17,ticks);
  }

  void RobotControl::setLeftHipYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_HIP_YAW_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(19,ticks);
  }

  void RobotControl::setLeftHipYawRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_HIP_YAW_ROLL)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(19,ticks);
  }

  void RobotControl::setLeftHipRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_HIP_YAW_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(20,ticks);
  }

  void RobotControl::setLeftHipRollRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_HIP_YAW_ROLL)->getMotorByChannel(1);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(20,ticks);
  }

  void RobotControl::setLeftHipPitch(int ticks, int delay){
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_L_HIP_PITCH);
	  mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(21,ticks);
  }

  void RobotControl::setLeftHipPitchRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_HIP_PITCH)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
  }

  void RobotControl::setLeftKnee(int ticks, int delay){
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_L_KNEE);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(22,ticks);
  }

  void RobotControl::setLeftKneeRad(double rads, int delay){
	  HuboMotor* motor = this->state->getBoardByNumber(BNO_L_KNEE)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(22,ticks);
  }

  void RobotControl::setLeftAnklePitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(23,ticks);
  }

  void RobotControl::setLeftAnklePitchRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_ANKLE_PITCH_ROLL)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(23,ticks);
  }

  void RobotControl::setLeftAnkleRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(24,ticks);
  }

  void RobotControl::setLeftAnkleRollRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_L_ANKLE_PITCH_ROLL)->getMotorByChannel(1);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(24,ticks);
  }

  void RobotControl::setRightHipYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(26,ticks);
      //this->canDownPort->write(buildCanMessage(out));
  }

  void RobotControl::setRightHipYawRad(double rads, int delay){
        HuboMotor* motor = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL)->getMotorByChannel(0);
        motor->setDesiredPosition(motor->radiansToTicks(rads));
  }

  void RobotControl::setRightHipRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(27,ticks);
  }

  void RobotControl::setRightHipRollRad(double rads, int delay){
	  HuboMotor* motor = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL)->getMotorByChannel(1);
	  motor->setDesiredPosition(motor->radiansToTicks(rads));
  }

  void RobotControl::setRightHipPitch(int ticks, int delay){
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_PITCH);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(28,ticks);
  }

  void RobotControl::setRightHipPitchRad(double rads, int delay){
	  HuboMotor* motor = this->state->getBoardByNumber(BNO_R_HIP_PITCH)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(28,ticks);
  }

  void RobotControl::setRightKnee(int ticks, int delay){
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_R_KNEE);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(29,ticks);
  }

  void RobotControl::setRightKneeRad(double rads, int delay){
	  HuboMotor* motor = this->state->getBoardByNumber(BNO_R_KNEE)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(29,ticks);
  }

  void RobotControl::setRightAnklePitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(30,ticks);
  }

  void RobotControl::setRightAnklePitchRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_ANKLE_PITCH_ROLL)->getMotorByChannel(0);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(30,ticks);
  }

  void RobotControl::setRightAnkleRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(31,ticks);
  }

  void RobotControl::setRightAnkleRollRad(double rads, int delay){
      HuboMotor* motor = this->state->getBoardByNumber(BNO_R_ANKLE_PITCH_ROLL)->getMotorByChannel(1);
      motor->setDesiredPosition(motor->radiansToTicks(rads));
      //ros_gateway->transmit(31,ticks);
  }

  void RobotControl::setRightHand(int f0, int f1, int f2, int f3, int f4, int delay){
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HAND);
	  mb->getMotorByChannel(0)->setDesiredPosition(f0);
	  mb->getMotorByChannel(1)->setDesiredPosition(f1);
	  mb->getMotorByChannel(2)->setDesiredPosition(f2);
	  mb->getMotorByChannel(3)->setDesiredPosition(f3);
	  mb->getMotorByChannel(4)->setDesiredPosition(f4);
  }

  void RobotControl::setLeftHand(int f0, int f1, int f2, int f3, int f4, int delay){
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_L_HAND);
	  mb->getMotorByChannel(0)->setDesiredPosition(f0);
	  mb->getMotorByChannel(1)->setDesiredPosition(f1);
	  mb->getMotorByChannel(2)->setDesiredPosition(f2);
	  mb->getMotorByChannel(3)->setDesiredPosition(f3);
	  mb->getMotorByChannel(4)->setDesiredPosition(f4);
  }

  void RobotControl::enable(int board, int delay){
      this->state->getBoardByNumber(board)->setHIP(1);
      this->state->getBoardByNumber(board)->enableController();
      enableControl = true;
  }

  void RobotControl::disable(int board, int delay){
      this->state->getBoardByNumber(board)->setHIP(0);
      this->state->getBoardByNumber(board)->disableController();
      enableControl = false;
  }

  void RobotControl::requestEncoderPosition(int board, int delay){
      this->state->getBoardByNumber(board)->requestEncoderPosition(0);
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

  void RobotControl::getCurrentTicks(int board, int motor, int delay){
	  std::cout << "Motor[" << motor << "] ticks: " << this->state->getBoardByNumber(board)->getMotorByChannel(motor)->getTicksPosition() << std::endl;
  }

  void RobotControl::setCurrentTicks(int board, int motor, int ticks){
	  this->state->getBoardByNumber(board)->getMotorByChannel(motor)->setTicksPosition((long)ticks);
  }

  void RobotControl::getCurrentGoal(int board, int motor, int delay){
  	  std::cout << "Motor[" << motor << "] goal: " << this->state->getBoardByNumber(board)->getMotorByChannel(motor)->getDesiredPosition() << std::endl;
   }

  bool RobotControl::requiresMotion(int board, int motor, int delay){
	  return state->getBoardByNumber(board)->requiresMotion(motor);
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
