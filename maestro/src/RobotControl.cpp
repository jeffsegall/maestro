#include "RobotControl.h"
#include <iostream>
using namespace std;

RobotControl::RobotControl(const std::string& name):
    TaskContext(name)
  {
    
    this->canUpPort = new InputPort<hubomsg::CanMessage>("can_up");
    this->canDownPort = new OutputPort<hubomsg::CanMessage>("can_down");
    this->orOutPort = new InputPort<hubomsg::HuboCmd>("or_out");
    this->orInPort = new OutputPort<hubomsg::HuboCmd>("or_in");
    this->commHandler = new CommHandler(canUpPort, orOutPort);

    //CAN QUEUES
    this->inputQueue = new queue<hubomsg::CanMessage>();
    this->outputQueue = new queue<hubomsg::CanMessage>();
    
    //CAN PORTS 
    this->addEventPort(*canUpPort);
    this->addPort(*canDownPort);

    //OPENRAVE PORTS
    this->addEventPort(*orOutPort);
    this->addPort(*orInPort);

    this->addOperation("setWaist", &RobotControl::setWaist, this, RTT::OwnThread)
            .doc("Set Torso Yaw")
            .arg("Value", "New ticks for torso yaw.")
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
    
    this->addOperation("setLeftShoulderPitch", &RobotControl::setLeftShoulderPitch, this, RTT::OwnThread)
            .doc("Set Left Shoulder Pitch")
            .arg("Value", "New ticks for left shoulder pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
   
    this->addOperation("setLeftShoulderYaw", &RobotControl::setLeftShoulderYaw, this, RTT::OwnThread)
            .doc("Set Left Shoulder Yaw")
            .arg("Value", "New ticks for left shoulder yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
   
    this->addOperation("setLeftElbow", &RobotControl::setLeftElbow, this, RTT::OwnThread)
            .doc("Set Left Elbow")
            .arg("Value", "New ticks for left elbow.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftWristPitch", &RobotControl::setLeftWristPitch, this, RTT::OwnThread)
            .doc("Set Left Wrist Pitch")
            .arg("Value", "New ticks for left wrist pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftWristYaw", &RobotControl::setLeftWristYaw, this, RTT::OwnThread)
            .doc("Set Left Wrist Yaw")
            .arg("Value", "New ticks for left Wrist Yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderRoll", &RobotControl::setRightShoulderRoll, this, RTT::OwnThread)
            .doc("Set Right Shoulder Roll")
            .arg("Value", "New ticks for right shoulder roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderPitch", &RobotControl::setRightShoulderPitch, this, RTT::OwnThread)
            .doc("Set Right Shoulder Pitch")
            .arg("Value", "New ticks for right shoulder pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightShoulderYaw", &RobotControl::setRightShoulderYaw, this, RTT::OwnThread)
            .doc("Set Right Shoulder Yaw")
            .arg("Value", "New ticks for right shoulder yaw.")			
            .arg("Timestamp", "Timestamp delay (in milliseconds)");
			
    this->addOperation("setRightElbow", &RobotControl::setRightElbow, this, RTT::OwnThread)
            .doc("Set Right Elbow")
            .arg("Value", "New ticks for right elbow.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightWristPitch", &RobotControl::setRightWristPitch, this, RTT::OwnThread)
            .doc("Set Right Wrist Pitch")
            .arg("Value", "New ticks for right wrist pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightWristYaw", &RobotControl::setRightWristYaw, this, RTT::OwnThread)
            .doc("Set Right Wrist Yaw")
            .arg("Value", "New ticks for right wrist yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipYaw", &RobotControl::setLeftHipYaw, this, RTT::OwnThread)
            .doc("Set Left Hip Yaw")
            .arg("Value", "New ticks for left hip yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipRoll", &RobotControl::setLeftHipRoll, this, RTT::OwnThread)
            .doc("Set Left Hip Roll")
            .arg("Value", "New ticks for left hip roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftHipPitch", &RobotControl::setLeftHipPitch, this, RTT::OwnThread)
            .doc("Set Left Hip Pitch")
            .arg("Value", "New ticks for left hip pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftKnee", &RobotControl::setLeftKnee, this, RTT::OwnThread)
            .doc("Set Left Knee")
            .arg("Value", "New ticks for left knee.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftAnklePitch", &RobotControl::setLeftAnklePitch, this, RTT::OwnThread)
            .doc("Set Left Ankle Pitch")
            .arg("Value", "New ticks for left ankle pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setLeftAnkleRoll", &RobotControl::setLeftAnkleRoll, this, RTT::OwnThread)
            .doc("Set Left Ankle Roll")
            .arg("Value", "New ticks for left ankle roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipYaw", &RobotControl::setRightHipYaw, this, RTT::OwnThread)
            .doc("Set Right Hip Yaw")
            .arg("Value", "New ticks for right hip yaw.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipYawRads", &RobotControl::setRightHipYawRad, this, RTT::OwnThread)
                .doc("Set Right Hip Yaw")
                .arg("Rads", "New radians for right hip yaw.")
                .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipRoll", &RobotControl::setRightHipRoll, this, RTT::OwnThread)
            .doc("Set Right Hip Roll")
            .arg("Value", "New ticks for right hip roll.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipRollRads", &RobotControl::setRightHipRollRad, this, RTT::OwnThread)
                    .doc("Set Right Hip Yaw")
                    .arg("Rads", "New radians for right hip roll.")
                    .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightHipPitch", &RobotControl::setRightHipPitch, this, RTT::OwnThread)
            .doc("Set Right Hip Pitch")
            .arg("Value", "New ticks for right hip pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightKnee", &RobotControl::setRightKnee, this, RTT::OwnThread)
            .doc("Set Right Knee")
            .arg("Value", "New ticks for right knee.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightAnklePitch", &RobotControl::setRightAnklePitch, this, RTT::OwnThread)
            .doc("Set Right Ankle Pitch")
            .arg("Value", "New ticks for right ankle pitch.")
            .arg("Timestamp", "Timestamp delay (in milliseconds)");

    this->addOperation("setRightAnkleRoll", &RobotControl::setRightAnkleRoll, this, RTT::OwnThread)
            .doc("Set Right Ankle Roll")
            .arg("Value", "New ticks for right ankle roll.")
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
            .arg("Path", "The path to the file that contains the gesture.")
            .arg("Board", "The board on which to run the gesture");

    this->written = 0;
    this->printNow = false;
    this->enableControl = false;
    this->delay = 0;
    tempOutput.open("/home/hubo/maestro/RobotControlLog.txt");
    //initRobot("/home/hubo/maestro/maestro/models/hubo_testrig.xml");

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

    return val;
}

  void RobotControl::updateHook(){

	tempOutput << "In Update Hook." << std::endl;
    
    hubomsg::HuboCmd huboCmd = hubomsg::HuboCmd();
    hubomsg::CanMessage canMessage = hubomsg::CanMessage();

    commHandler->update();

    MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);

    if (commHandler->isNew()){
    	tempOutput << "New Message. " << std::endl;
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

    if (outputQueue->empty() && !this->state->getBoards().empty()) {
		tempOutput << "Boards not empty. Map size: " << this->state->getBoards().size() << std::endl;
		for (int i = 0; i < this->state->getBoards().size(); i++){
			if (this->state->getBoards()[i]->requiresMotion()){
				//tempOutput << "Attempting to build message for :" << this->state->getBoards()[i]->getBoardNumber() << std::endl;
				this->outputQueue->push(buildCanMessage(this->state->getBoards()[i]->sendPositionReference()));
			}
		}
	}
   
    //Write out a message if we have one


    if (!outputQueue->empty()){

    	hubomsg::CanMessage output = outputQueue->front();
    	if (printNow){
    		tempOutput << "Writing message to Board " << output.bno << ": R1 = " << outputQueue->front().r1 << std::endl;
    	}

		this->canDownPort->write(output);

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

  void RobotControl::initRobot(string path){
      this->state = new HuboState();
      if (strcmp(path.c_str(), "") == 0)
          path = "/home/hubo/maestro/maestro/models/hubo_testrig.xml";
      
      //@TODO: Check for file existence before initializing.
      this->state->initHuboWithDefaults(path, this->outputQueue);
  }

  void RobotControl::setWaist(int ticks, int delay){
      //ros_gateway->transmit(0,ticks);
	  this->state->getBoardByNumber(BNO_WAIST)->getMotorByChannel(0)->setDesiredPosition(ticks);
  }

  void RobotControl::setNeck(int ticks, int one, int two, int delay){
	  this->state->getBoardByNumber(BNO_NECK_YAW_1_2)->getMotorByChannel(0)->setDesiredPosition(ticks);
	  this->state->getBoardByNumber(BNO_NECK_YAW_1_2)->getMotorByChannel(1)->setDesiredPosition(one);
	  this->state->getBoardByNumber(BNO_NECK_YAW_1_2)->getMotorByChannel(1)->setDesiredPosition(two);
     //ros_gateway->transmit(1,ticks);
  }

  void RobotControl::setLeftShoulderRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_SHOULDER_PITCH_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
     //ros_gateway->transmit(3,ticks);
  }

  void RobotControl::setLeftShoulderPitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_SHOULDER_PITCH_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
     //ros_gateway->transmit(4,ticks);
  }
  
  void RobotControl::setLeftShoulderYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_SHOULDER_YAW_ELBOW);
	  mb->getMotorByChannel(0)->setDesiredPosition(ticks);
     //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftElbow(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_SHOULDER_YAW_ELBOW);
	  mb->getMotorByChannel(1)->setDesiredPosition(ticks);
     //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftWristPitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_WRIST_YAW_PITCH);
	  mb->getMotorByChannel(1)->setDesiredPosition(ticks);
     //ros_gateway->transmit(8,ticks);
  }

  void RobotControl::setLeftWristYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_WRIST_YAW_PITCH);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(9,ticks);
  }

  void RobotControl::setRightShoulderRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_PITCH_ROLL);
	  mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(11,ticks);
  }

  void RobotControl::setRightShoulderPitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_PITCH_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(12,ticks);
  }

  void RobotControl::setRightShoulderYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_YAW_ELBOW);
	  mb->getMotorByChannel(0)->setDesiredPosition(ticks);
  }
  
  void RobotControl::setRightElbow(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_SHOULDER_YAW_ELBOW);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(14,ticks);
  }

  void RobotControl::setRightWristPitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_WRIST_YAW_PITCH);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(16,ticks);
  }

  void RobotControl::setRightWristYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_WRIST_YAW_PITCH);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(17,ticks);
  }

  void RobotControl::setLeftHipYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_HIP_YAW_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(19,ticks);
  }

  void RobotControl::setLeftHipRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_HIP_YAW_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(20,ticks);
  }

  void RobotControl::setLeftHipPitch(int ticks, int delay){
      this->state->getBoardByNumber(BNO_L_HIP_PITCH)->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(21,ticks);
  }

  void RobotControl::setLeftKnee(int ticks, int delay){
      this->state->getBoardByNumber(BNO_L_KNEE)->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(22,ticks);
  }

  void RobotControl::setLeftAnklePitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(23,ticks);
  }

  void RobotControl::setLeftAnkleRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_L_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(24,ticks);
  }

  void RobotControl::setRightHipYaw(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(26,ticks);
      //this->canDownPort->write(buildCanMessage(out));
  }

  void RobotControl::setRightHipYawRad(double rads, int delay){
        MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);
        mb->getMotorByChannel(0)->setDesiredPosition(mb->getMotorByChannel(0)->radiansToTicks(rads));
  }

  void RobotControl::setRightHipRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
      //ros_gateway->transmit(27,ticks);
  }

  void RobotControl::setRightHipRollRad(double rads, int delay){
	  MotorBoard* mb = this->state->getBoardByNumber(BNO_R_HIP_YAW_ROLL);
	  mb->getMotorByChannel(1)->setDesiredPosition(mb->getMotorByChannel(1)->radiansToTicks(rads));
  }

  void RobotControl::setRightHipPitch(int ticks, int delay){
      this->state->getBoardByNumber(BNO_R_HIP_PITCH)->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(28,ticks);
  }

  void RobotControl::setRightKnee(int ticks, int delay){
      this->state->getBoardByNumber(BNO_R_KNEE)->getMotorByChannel(0)->setDesiredPosition(ticks);;
      //ros_gateway->transmit(29,ticks);
  }

  void RobotControl::setRightAnklePitch(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(0)->setDesiredPosition(ticks);
      //ros_gateway->transmit(30,ticks);
  }

  void RobotControl::setRightAnkleRoll(int ticks, int delay){
      MotorBoard* mb = this->state->getBoardByNumber(BNO_R_ANKLE_PITCH_ROLL);
      mb->getMotorByChannel(1)->setDesiredPosition(ticks);
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
      //this->state->getBoardByNumber(board)->requestEncoderPosition(0);
  }

  void RobotControl::disable(int board, int delay){
      this->state->getBoardByNumber(board)->setHIP(0);
      this->state->getBoardByNumber(board)->disableController();
      enableControl = false;
      //this->state->getBoardByNumber(board)->requestEncoderPosition(0);
  }

  void RobotControl::requestEncoderPosition(int board, int delay){ 
      this->state->getBoardByNumber(board)->requestEncoderPosition(0);
  }

  void RobotControl::debugControl(int board, int operation){
	  switch (operation)
	  {
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

  void RobotControl::setMaxAccVel(int board, int motor, int acc, int vel){
	  this->state->getBoardByNumber(board)->setMaxAccVel((char)motor, acc, vel);
  }

  void RobotControl::setPositionGain(int board, int motor, int kp, int ki, int kd){
	  this->state->getBoardByNumber(board)->setPositionGain(motor, kp, ki, kd);
  }

  void RobotControl::runGesture(string path, int board){
      int val = 0;
      vector<float> trajVal;
      if (this->gestures.find(path) != this->gestures.end())
          trajVal = this->gestures[path];
      else{
          trajVal = trajectoryValues(path);
          this->gestures[path] = trajVal;
      }
      for (int i = 0; i < (int)trajVal.size(); i++){
          val = (int)trajVal.at(i);
          vector<int> ticks(2); //TODO: Request number of channels, or make runGesture more robust.
          ticks[0] = val;
          ticks[1] = val;
          this->state->getBoardByNumber(board)->sendPositionReference(ticks, 125, 50);
      }
  }

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(RobotControl)
