#include "RobotControl.h"

RobotControl::RobotControl(const std::string& name):
    TaskContext(name)
  {
    
    this->addEventPort(*ros_gateway->getInputPort());
    this->addPort(*ros_gateway->getOutputPort());

    this->addOperation("setTorsoYaw", &RobotControl::setTorsoYaw, this, RTT::OwnThread)
            .doc("Set Torso Yaw")
            .arg("Value", "New ticks for torso yaw.");
    
    this->addOperation("setHeadYaw", &RobotControl::setHeadYaw, this, RTT::OwnThread)
            .doc("Set Head Yaw")
            .arg("Value", "New ticks for head yaw.");
    
    this->addOperation("setLeftShoulderRoll", &RobotControl::setLeftShoulderRoll, this, RTT::OwnThread)
            .doc("Set Left Shoulder Roll")
            .arg("Value", "New ticks for left shoulder roll.");
    
    this->addOperation("setLeftShoulderPitch", &RobotControl::setLeftShoulderPitch, this, RTT::OwnThread)
            .doc("Set Left Shoulder Pitch")
            .arg("Value", "New ticks for left shoulder pitch.");
    
    this->addOperation("setLeftElbowRoll", &RobotControl::setLeftElbowRoll, this, RTT::OwnThread)
            .doc("Set Left Elbow Roll")
            .arg("Value", "New ticks for left elbow roll.");

    this->addOperation("setLeftElbowYaw", &RobotControl::setLeftElbowYaw, this, RTT::OwnThread)
            .doc("Set Left Elbow Yaw")
            .arg("Value", "New ticks for left elbow yaw.");

    this->addOperation("setLeftWristRoll", &RobotControl::setLeftWristRoll, this, RTT::OwnThread)
            .doc("Set Left Wrist Roll")
            .arg("Value", "New ticks for left wrist roll.");

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

    this->addOperation("setRightElbowRoll", &RobotControl::setRightElbowRoll, this, RTT::OwnThread)
            .doc("Set Right Elbow Roll")
            .arg("Value", "New ticks for right elbow roll.");

    this->addOperation("setRightElbowYaw", &RobotControl::setRightElbowYaw, this, RTT::OwnThread)
            .doc("Set Right Elbow Yaw")
            .arg("Value", "New ticks for right elbow yaw.");

    this->addOperation("setRightWristRoll", &RobotControl::setRightWristRoll, this, RTT::OwnThread)
            .doc("Set Right Wrist Roll")
            .arg("Value", "New ticks for right wrist roll.");

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

    this->addOperation("setLeftKneePitch", &RobotControl::setLeftKneePitch, this, RTT::OwnThread)
            .doc("Set Left Knee Pitch")
            .arg("Value", "New ticks for left knee pitch.");

    this->addOperation("setLeftFootPitch", &RobotControl::setLeftFootPitch, this, RTT::OwnThread)
            .doc("Set Left Foot Pitch")
            .arg("Value", "New ticks for left foot pitch.");

    this->addOperation("setLeftFootRoll", &RobotControl::setLeftFootRoll, this, RTT::OwnThread)
            .doc("Set Left Foot Roll")
            .arg("Value", "New ticks for left foot roll.");

    this->addOperation("setRightHipYaw", &RobotControl::setRightHipYaw, this, RTT::OwnThread)
            .doc("Set Right Hip Yaw")
            .arg("Value", "New ticks for right hip yaw.");

    this->addOperation("setRightHipRoll", &RobotControl::setRightHipRoll, this, RTT::OwnThread)
            .doc("Set Right Hip Roll")
            .arg("Value", "New ticks for right hip roll.");

    this->addOperation("setRightHipPitch", &RobotControl::setRightHipPitch, this, RTT::OwnThread)
            .doc("Set Right Hip Pitch")
            .arg("Value", "New ticks for right hip pitch.");

    this->addOperation("setRightKneePitch", &RobotControl::setRightKneePitch, this, RTT::OwnThread)
            .doc("Set Right Knee Pitch")
            .arg("Value", "New ticks for right knee pitch.");

    this->addOperation("setRightFootPitch", &RobotControl::setRightFootPitch, this, RTT::OwnThread)
            .doc("Set Right Foot Pitch")
            .arg("Value", "New ticks for right foot pitch.");

    this->addOperation("setRightFootRoll", &RobotControl::setRightFootRoll, this, RTT::OwnThread)
            .doc("Set Right Foot Roll")
            .arg("Value", "New ticks for right foot roll.");
  }
  
  ~RobotControl(){}

  void updateHook(){

    //Check for incoming data and respond if necessary.
    if(ros_gateway->recv()){

    }  
  }

  void setWaist(double ticks){
      ros_gateway->transmit(0,ticks);
  }

  void setNeck(double ticks, double one, double two){
      ros_gateway->transmit(1,ticks);
  }

  void setLeftShoulderRoll(double ticks){
      ros_gateway->transmit(3,ticks);
  }

  void setLeftShoulderPitch(double ticks){
      ros_gateway->transmit(4,ticks);
  }

  void setLeftElbowRoll(double ticks){
      ros_gateway->transmit(5,ticks);
  }

  void setLeftElbowYaw(double ticks){
      ros_gateway->transmit(6,ticks);
  }

  void setLeftWristRoll(double ticks){
      ros_gateway->transmit(7,ticks);
  }

  void setLeftWristPitch(double ticks){
      ros_gateway->transmit(8,ticks);
  }

  void setLeftWristYaw(double ticks){
      ros_gateway->transmit(9,ticks);
  }

  void setRightShoulderRoll(double ticks){
      ros_gateway->transmit(11,ticks);
  }

  void setRightShoulderPitch(double ticks){
      ros_gateway->transmit(12,ticks);
  }

  void setRightElbowRoll(double ticks){
      ros_gateway->transmit(13,ticks);
  }

  void setRightElbowYaw(double ticks){
      ros_gateway->transmit(14,ticks);
  }

  void setRightWristRoll(double ticks){
      ros_gateway->transmit(15,ticks);
  }

  void setRightWristPitch(double ticks){
      ros_gateway->transmit(16,ticks);
  }

  void setRightWristYaw(double ticks){
      ros_gateway->transmit(17,ticks);
  }

  void setLeftHipYaw(double ticks){
      ros_gateway->transmit(19,ticks);
  }

  void setLeftHipRoll(double ticks){
      ros_gateway->transmit(20,ticks);
  }

  void setLeftHipPitch(double ticks){
      ros_gateway->transmit(21,ticks);
  }

  void setLeftKnee(double ticks){
      ros_gateway->transmit(22,ticks);
  }

  void setLeftFootPitch(double ticks){
      ros_gateway->transmit(23,ticks);
  }

  void setLeftFootRoll(double ticks){
      ros_gateway->transmit(24,ticks);
  }

  void setRightHipYaw(double ticks){
      ros_gateway->transmit(26,ticks);
  }

  void setRightHipRoll(double ticks){
      ros_gateway->transmit(27,ticks);
  }

  void setRightHipPitch(double ticks){
      ros_gateway->transmit(28,ticks);
  }

  void setRightKnee(double ticks){
      ros_gateway->transmit(29,ticks);
  }

  void setRightFootPitch(double ticks){
      ros_gateway->transmit(30,ticks);
  }

  void setRightFootRoll(double ticks){
      ros_gateway->transmit(31,ticks);
  }

};
ORO_CREATE_COMPONENT(RobotControl)
