#include "RobotControl.h"

RobotControl::RobotControl(const std::string& name):
    TaskContext(name)
  {
    this->canUpPort = new InputPort<hubomsg::CanMessage>("can_up");
    this->canDownPort = new OutputPort<hubomsg::CanMessage>("can_down");
    this->orOutPort = new InputPort<hubomsg::HuboCmd>("or_out");
    this->orInPort = new OutputPort<hubomsg::HuboCmd>("or_in"); 

    //CAN PORTS 
    this->addEventPort(*canUpPort);
    this->addPort(*canDownPort);

    //OPENRAVE PORTS
    this->addEventPort(*orOutPort);
    this->addPort(*orInPort);

    this->addOperation("setWaist", &RobotControl::setWaist, this, RTT::OwnThread)
            .doc("Set Torso Yaw")
            .arg("Value", "New ticks for torso yaw.");
    
    this->addOperation("setLeftShoulderRoll", &RobotControl::setLeftShoulderRoll, this, RTT::OwnThread)
            .doc("Set Left Shoulder Roll")
            .arg("Value", "New ticks for left shoulder roll.");
    
    this->addOperation("setLeftShoulderPitch", &RobotControl::setLeftShoulderPitch, this, RTT::OwnThread)
            .doc("Set Left Shoulder Pitch")
            .arg("Value", "New ticks for left shoulder pitch.");
    
    this->addOperation("setLeftElbow", &RobotControl::setLeftElbow, this, RTT::OwnThread)
            .doc("Set Left Elbow")
            .arg("Value", "New ticks for left elbow.");

    this->addOperation("setLeftWristPitch", &RobotControl::setLeftWristPitch, this, RTT::OwnThread)
            .doc("Set Left Wrist Pitch")
            .arg("Value", "New ticks for left wrist pitch.");

    this->addOperation("setLeftWristYaw", &RobotControl::setLeftWristYaw, this, RTT::OwnThread)
            .doc("Set Left Wrist Yaw")
            .arg("Value", "New ticks for left Wrist Yaw.");

    this->addOperation("setRightShoulderRoll", &RobotControl::setRightShoulderRoll, this, RTT::OwnThread)
            .doc("Set Right Shoulder Roll")
            .arg("Value", "New ticks for right shoulder roll.");

    this->addOperation("setRightShoulderPitch", &RobotControl::setRightShoulderPitch, this, RTT::OwnThread)
            .doc("Set Right Shoulder Pitch")
            .arg("Value", "New ticks for right shoulder pitch.");

    this->addOperation("setRightElbow", &RobotControl::setRightElbow, this, RTT::OwnThread)
            .doc("Set Right Elbow")
            .arg("Value", "New ticks for right elbow.");

    this->addOperation("setRightWristPitch", &RobotControl::setRightWristPitch, this, RTT::OwnThread)
            .doc("Set Right Wrist Pitch")
            .arg("Value", "New ticks for right wrist pitch.");

    this->addOperation("setRightWristYaw", &RobotControl::setRightWristYaw, this, RTT::OwnThread)
            .doc("Set Right Wrist Yaw")
            .arg("Value", "New ticks for right wrist yaw.");

    this->addOperation("setLeftHipYaw", &RobotControl::setLeftHipYaw, this, RTT::OwnThread)
            .doc("Set Left Hip Yaw")
            .arg("Value", "New ticks for left hip yaw.");

    this->addOperation("setLeftHipRoll", &RobotControl::setLeftHipRoll, this, RTT::OwnThread)
            .doc("Set Left Hip Roll")
            .arg("Value", "New ticks for left hip roll.");

    this->addOperation("setLeftHipPitch", &RobotControl::setLeftHipPitch, this, RTT::OwnThread)
            .doc("Set Left Hip Pitch")
            .arg("Value", "New ticks for left hip pitch.");

    this->addOperation("setLeftKnee", &RobotControl::setLeftKnee, this, RTT::OwnThread)
            .doc("Set Left Knee")
            .arg("Value", "New ticks for left knee.");

    this->addOperation("setLeftAnklePitch", &RobotControl::setLeftAnklePitch, this, RTT::OwnThread)
            .doc("Set Left Ankle Pitch")
            .arg("Value", "New ticks for left ankle pitch.");

    this->addOperation("setLeftAnkleRoll", &RobotControl::setLeftAnkleRoll, this, RTT::OwnThread)
            .doc("Set Left Ankle Roll")
            .arg("Value", "New ticks for left ankle roll.");

    this->addOperation("setRightHipYaw", &RobotControl::setRightHipYaw, this, RTT::OwnThread)
            .doc("Set Right Hip Yaw")
            .arg("Value", "New ticks for right hip yaw.");

    this->addOperation("setRightHipRoll", &RobotControl::setRightHipRoll, this, RTT::OwnThread)
            .doc("Set Right Hip Roll")
            .arg("Value", "New ticks for right hip roll.");

    this->addOperation("setRightHipPitch", &RobotControl::setRightHipPitch, this, RTT::OwnThread)
            .doc("Set Right Hip Pitch")
            .arg("Value", "New ticks for right hip pitch.");

    this->addOperation("setRightKnee", &RobotControl::setRightKnee, this, RTT::OwnThread)
            .doc("Set Right Knee")
            .arg("Value", "New ticks for right knee.");

    this->addOperation("setRightAnklePitch", &RobotControl::setRightAnklePitch, this, RTT::OwnThread)
            .doc("Set Right Ankle Pitch")
            .arg("Value", "New ticks for right ankle pitch.");

    this->addOperation("setRightAnkleRoll", &RobotControl::setRightAnkleRoll, this, RTT::OwnThread)
            .doc("Set Right Ankle Roll")
            .arg("Value", "New ticks for right ankle roll.");
  }
  
  RobotControl::~RobotControl(){}

  void RobotControl::updateHook(){
    hubomsg::HuboCmd huboCmd = hubomsg::HuboCmd();
    hubomsg::CanMessage canMessage = hubomsg::CanMessage();
    if (NewData == this->canUpPort->read(canMessage)){
        //Received update from CanGateway
    }
    if (NewData == this->orOutPort->read(huboCmd)){
        //Recieved update from openRAVE
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

      return canMessage;
  }

  void RobotControl::setWaist(long ticks){
      //ros_gateway->transmit(0,ticks);
      canMsg* out = new canMsg(BNO_WAIST, TX_REF, CMD_NONE,
                               ticks, 0, 0, 0, 0);
      this->canDownPort->write(buildCanMessage(out));
  }

  void RobotControl::setNeck(long ticks, long one, long two){

      canMsg* out = new canMsg(BNO_NECK_YAW_1_2, TX_REF, CMD_NONE,
                               ticks, one, two, 0, 0);
      this->canDownPort->write(buildCanMessage(out));
     //ros_gateway->transmit(1,ticks);
  }

  void RobotControl::setLeftShoulderRoll(long ticks){
      canMsg* out = new canMsg(BNO_L_SHOULDER_PITCH_ROLL, TX_REF, CMD_NONE,
                               state.getMotorByName(LSR).getTicksPosition(), ticks, 0, 0, 0);
      this->canDownPort->write(buildCanMessage(out));
     //ros_gateway->transmit(3,ticks);
  }

  void RobotControl::setLeftShoulderPitch(long ticks){
      canMsg* out = new canMsg(BNO_L_SHOULDER_PITCH_ROLL, TX_REF, CMD_NONE,
                               ticks, state.getMotorByName(LSP).getTicksPosition(), 0, 0, 0);
      this->canDownPort->write(buildCanMessage(out));
     //ros_gateway->transmit(4,ticks);
  }

  void RobotControl::setLeftElbow(long ticks){
      //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftWristPitch(long ticks){
      //ros_gateway->transmit(8,ticks);
  }

  void RobotControl::setLeftWristYaw(long ticks){
      //ros_gateway->transmit(9,ticks);
  }

  void RobotControl::setRightShoulderRoll(long ticks){
      //ros_gateway->transmit(11,ticks);
  }

  void RobotControl::setRightShoulderPitch(long ticks){
      //ros_gateway->transmit(12,ticks);
  }

  void RobotControl::setRightElbow(long ticks){
      //ros_gateway->transmit(14,ticks);
  }

  void RobotControl::setRightWristPitch(long ticks){
      //ros_gateway->transmit(16,ticks);
  }

  void RobotControl::setRightWristYaw(long ticks){
      //ros_gateway->transmit(17,ticks);
  }

  void RobotControl::setLeftHipYaw(long ticks){
      //ros_gateway->transmit(19,ticks);
  }

  void RobotControl::setLeftHipRoll(long ticks){
      //ros_gateway->transmit(20,ticks);
  }

  void RobotControl::setLeftHipPitch(long ticks){
      //ros_gateway->transmit(21,ticks);
  }

  void RobotControl::setLeftKnee(long ticks){
      //ros_gateway->transmit(22,ticks);
  }

  void RobotControl::setLeftAnklePitch(long ticks){
      //ros_gateway->transmit(23,ticks);
  }

  void RobotControl::setLeftAnkleRoll(long ticks){
      //ros_gateway->transmit(24,ticks);
  }

  void RobotControl::setRightHipYaw(long ticks){
      //ros_gateway->transmit(26,ticks);
  }

  void RobotControl::setRightHipRoll(long ticks){
      //ros_gateway->transmit(27,ticks);
  }

  void RobotControl::setRightHipPitch(long ticks){
      //ros_gateway->transmit(28,ticks);
  }

  void RobotControl::setRightKnee(long ticks){
      //ros_gateway->transmit(29,ticks);
  }

  void RobotControl::setRightAnklePitch(long ticks){
      //ros_gateway->transmit(30,ticks);
  }

  void RobotControl::setRightAnkleRoll(long ticks){
      //ros_gateway->transmit(31,ticks);
  }

  void RobotControl::setRightHand(long f0, long f1, long f2, long f3, long f4){

  }

  void RobotControl::setLeftHand(long f0, long f1, long f2, long f3, long f4){

  }

ORO_CREATE_COMPONENT(RobotControl)
