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

    //Check for incoming data and respond if necessary.
  }

  void RobotControl::setWaist(double ticks){
      //ros_gateway->transmit(0,ticks);
  }

  void RobotControl::setNeck(double ticks, double one, double two){
      //ros_gateway->transmit(1,ticks);
  }

  void RobotControl::setLeftShoulderRoll(double ticks){
      //ros_gateway->transmit(3,ticks);
  }

  void RobotControl::setLeftShoulderPitch(double ticks){
      //ros_gateway->transmit(4,ticks);
  }

  void RobotControl::setLeftElbow(double ticks){
      //ros_gateway->transmit(6,ticks);
  }

  void RobotControl::setLeftWristPitch(double ticks){
      //ros_gateway->transmit(8,ticks);
  }

  void RobotControl::setLeftWristYaw(double ticks){
      //ros_gateway->transmit(9,ticks);
  }

  void RobotControl::setRightShoulderRoll(double ticks){
      //ros_gateway->transmit(11,ticks);
  }

  void RobotControl::setRightShoulderPitch(double ticks){
      //ros_gateway->transmit(12,ticks);
  }

  void RobotControl::setRightElbow(double ticks){
      //ros_gateway->transmit(14,ticks);
  }

  void RobotControl::setRightWristPitch(double ticks){
      //ros_gateway->transmit(16,ticks);
  }

  void RobotControl::setRightWristYaw(double ticks){
      //ros_gateway->transmit(17,ticks);
  }

  void RobotControl::setLeftHipYaw(double ticks){
      //ros_gateway->transmit(19,ticks);
  }

  void RobotControl::setLeftHipRoll(double ticks){
      //ros_gateway->transmit(20,ticks);
  }

  void RobotControl::setLeftHipPitch(double ticks){
      //ros_gateway->transmit(21,ticks);
  }

  void RobotControl::setLeftKnee(double ticks){
      //ros_gateway->transmit(22,ticks);
  }

  void RobotControl::setLeftAnklePitch(double ticks){
      //ros_gateway->transmit(23,ticks);
  }

  void RobotControl::setLeftAnkleRoll(double ticks){
      //ros_gateway->transmit(24,ticks);
  }

  void RobotControl::setRightHipYaw(double ticks){
      //ros_gateway->transmit(26,ticks);
  }

  void RobotControl::setRightHipRoll(double ticks){
      //ros_gateway->transmit(27,ticks);
  }

  void RobotControl::setRightHipPitch(double ticks){
      //ros_gateway->transmit(28,ticks);
  }

  void RobotControl::setRightKnee(double ticks){
      //ros_gateway->transmit(29,ticks);
  }

  void RobotControl::setRightAnklePitch(double ticks){
      //ros_gateway->transmit(30,ticks);
  }

  void RobotControl::setRightAnkleRoll(double ticks){
      //ros_gateway->transmit(31,ticks);
  }

  void RobotControl::setRightHand(double f0, double f1, double f2, double f3, double f4){

  }

  void RobotControl::setLeftHand(double f0, double f1, double f2, double f3, double f4){

  }

ORO_CREATE_COMPONENT(RobotControl)
