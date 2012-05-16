#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <vector>
#include "RosGateway.h"
#include "CanGateway.h"

using namespace RTT;

class RobotControl : public RTT::TaskContext{
private:  
  RosGateway *ros_gateway;
  CanGateway *can_gateway;

public:
  RobotControl(const std::string& name):
    TaskContext(name)
  {
    this->ros_gateway = new RosGateway("ros_error", "ros_cmd");
    this->can_gateway = new CanGateway();
     
    //Initialize input and output ports
    this->addEventPort(*ros_gateway->getInputPort());
    this->addPort(*ros_gateway->getOutputPort());

    this->addOperation("setTorsoYaw", &RobotControl::setTorsoYaw, this, RTT::OwnThread)
            .doc("Set Torso Yaw")
            .arg("Value", "New value for torso yaw.");
    
    this->addOperation("setHeadYaw", &RobotControl::setHeadYaw, this, RTT::OwnThread)
            .doc("Set Head Yaw")
            .arg("Value", "New value for head yaw.");
    
    this->addOperation("setLeftShoulderRoll", &RobotControl::setLeftShoulderRoll, this, RTT::OwnThread)
            .doc("Set Left Shoulder Roll")
            .arg("Value", "New value for left shoulder roll.");
    
    this->addOperation("setLeftShoulderPitch", &RobotControl::setLeftShoulderPitch, this, RTT::OwnThread)
            .doc("Set Left Shoulder Pitch")
            .arg("Value", "New value for left shoulder pitch.");
    
    this->addOperation("setLeftElbowRoll", &RobotControl::setLeftElbowRoll, this, RTT::OwnThread)
            .doc("Set Left Elbow Roll")
            .arg("Value", "New value for left elbow roll.");

    this->addOperation("setLeftElbowYaw", &RobotControl::setLeftElbowYaw, this, RTT::OwnThread)
            .doc("Set Left Elbow Yaw")
            .arg("Value", "New value for left elbow yaw.");

    this->addOperation("setLeftWristRoll", &RobotControl::setLeftWristRoll, this, RTT::OwnThread)
            .doc("Set Left Wrist Roll")
            .arg("Value", "New value for left wrist roll.");

    this->addOperation("setLeftWristPitch", &RobotControl::setLeftWristPitch, this, RTT::OwnThread)
            .doc("Set Left Wrist Pitch")
            .arg("Value", "New value for left wrist pitch.");

    this->addOperation("setLeftWristYaw", &RobotControl::setLeftWristYaw, this, RTT::OwnThread)
            .doc("Set Left Wrist Yaw")
            .arg("Value", "New value for left Wrist Yaw.");

    this->addOperation("setRightShoulderRoll", &RobotControl::setRightShoulderRoll, this, RTT::OwnThread)
            .doc("Set Right Shoulder Roll")
            .arg("Value", "New value for right shoulder roll.");

    this->addOperation("setRightShoulderPitch", &RobotControl::setRightShoulderPitch, this, RTT::OwnThread)
            .doc("Set Right Shoulder Pitch")
            .arg("Value", "New value for right shoulder pitch.");

    this->addOperation("setRightElbowRoll", &RobotControl::setRightElbowRoll, this, RTT::OwnThread)
            .doc("Set Right Elbow Roll")
            .arg("Value", "New value for right elbow roll.");

    this->addOperation("setRightElbowYaw", &RobotControl::setRightElbowYaw, this, RTT::OwnThread)
            .doc("Set Right Elbow Yaw")
            .arg("Value", "New value for right elbow yaw.");

    this->addOperation("setRightWristRoll", &RobotControl::setRightWristRoll, this, RTT::OwnThread)
            .doc("Set Right Wrist Roll")
            .arg("Value", "New value for right wrist roll.");

    this->addOperation("setRightWristPitch", &RobotControl::setRightWristPitch, this, RTT::OwnThread)
            .doc("Set Right Wrist Pitch")
            .arg("Value", "New value for right wrist pitch.");

    this->addOperation("setRightWristYaw", &RobotControl::setRightWristYaw, this, RTT::OwnThread)
            .doc("Set Right Wrist Yaw")
            .arg("Value", "New value for right wrist yaw.");

    this->addOperation("setLeftHipYaw", &RobotControl::setLeftHipYaw, this, RTT::OwnThread)
            .doc("Set Left Hip Yaw")
            .arg("Value", "New value for left hip yaw.");

    this->addOperation("setLeftHipRoll", &RobotControl::setLeftHipRoll, this, RTT::OwnThread)
            .doc("Set Left Hip Roll")
            .arg("Value", "New value for left hip roll.");

    this->addOperation("setLeftHipPitch", &RobotControl::setLeftHipPitch, this, RTT::OwnThread)
            .doc("Set Left Hip Pitch")
            .arg("Value", "New value for left hip pitch.");

    this->addOperation("setLeftKneePitch", &RobotControl::setLeftKneePitch, this, RTT::OwnThread)
            .doc("Set Left Knee Pitch")
            .arg("Value", "New value for left knee pitch.");

    this->addOperation("setLeftFootPitch", &RobotControl::setLeftFootPitch, this, RTT::OwnThread)
            .doc("Set Left Foot Pitch")
            .arg("Value", "New value for left foot pitch.");

    this->addOperation("setLeftFootRoll", &RobotControl::setLeftFootRoll, this, RTT::OwnThread)
            .doc("Set Left Foot Roll")
            .arg("Value", "New value for left foot roll.");

    this->addOperation("setRightHipYaw", &RobotControl::setRightHipYaw, this, RTT::OwnThread)
            .doc("Set Right Hip Yaw")
            .arg("Value", "New value for right hip yaw.");

    this->addOperation("setRightHipRoll", &RobotControl::setRightHipRoll, this, RTT::OwnThread)
            .doc("Set Right Hip Roll")
            .arg("Value", "New value for right hip roll.");

    this->addOperation("setRightHipPitch", &RobotControl::setRightHipPitch, this, RTT::OwnThread)
            .doc("Set Right Hip Pitch")
            .arg("Value", "New value for right hip pitch.");

    this->addOperation("setRightKneePitch", &RobotControl::setRightKneePitch, this, RTT::OwnThread)
            .doc("Set Right Knee Pitch")
            .arg("Value", "New value for right knee pitch.");

    this->addOperation("setRightFootPitch", &RobotControl::setRightFootPitch, this, RTT::OwnThread)
            .doc("Set Right Foot Pitch")
            .arg("Value", "New value for right foot pitch.");

    this->addOperation("setRightFootRoll", &RobotControl::setRightFootRoll, this, RTT::OwnThread)
            .doc("Set Right Foot Roll")
            .arg("Value", "New value for right foot roll.");
  }
  
  ~RobotControl(){}

  void updateHook(){

    //Check for incoming data and respond if necessary.
    if(ros_gateway->recv()){

    }  
  }

  void setTorsoYaw(double value){
      ros_gateway->transmit(0,value);
      can_gateway->transmit(0,value);
  }

  void setHeadYaw(double value){
      ros_gateway->transmit(1,value);
  }

  void setLeftShoulderRoll(double value){
      ros_gateway->transmit(3,value);
  }

  void setLeftShoulderPitch(double value){
      ros_gateway->transmit(4,value);
  }

  void setLeftElbowRoll(double value){
      ros_gateway->transmit(5,value);
  }

  void setLeftElbowYaw(double value){
      ros_gateway->transmit(6,value);
  }

  void setLeftWristRoll(double value){
      ros_gateway->transmit(7,value);
  }

  void setLeftWristPitch(double value){
      ros_gateway->transmit(8,value);
  }

  void setLeftWristYaw(double value){
      ros_gateway->transmit(9,value);
  }

  void setRightShoulderRoll(double value){
      ros_gateway->transmit(11,value);
  }

  void setRightShoulderPitch(double value){
      ros_gateway->transmit(12,value);
  }

  void setRightElbowRoll(double value){
      ros_gateway->transmit(13,value);
  }

  void setRightElbowYaw(double value){
      ros_gateway->transmit(14,value);
  }

  void setRightWristRoll(double value){
      ros_gateway->transmit(15,value);
  }

  void setRightWristPitch(double value){
      ros_gateway->transmit(16,value);
  }

  void setRightWristYaw(double value){
      ros_gateway->transmit(17,value);
  }

  void setLeftHipYaw(double value){
      ros_gateway->transmit(19,value);
  }

  void setLeftHipRoll(double value){
      ros_gateway->transmit(20,value);
  }

  void setLeftHipPitch(double value){
      ros_gateway->transmit(21,value);
  }

  void setLeftKneePitch(double value){
      ros_gateway->transmit(22,value);
  }

  void setLeftFootPitch(double value){
      ros_gateway->transmit(23,value);
  }

  void setLeftFootRoll(double value){
      ros_gateway->transmit(24,value);
  }

  void setRightHipYaw(double value){
      ros_gateway->transmit(26,value);
  }

  void setRightHipRoll(double value){
      ros_gateway->transmit(27,value);
  }

  void setRightHipPitch(double value){
      ros_gateway->transmit(28,value);
  }

  void setRightKneePitch(double value){
      ros_gateway->transmit(29,value);
  }

  void setRightFootPitch(double value){
      ros_gateway->transmit(30,value);
  }

  void setRightFootRoll(double value){
      ros_gateway->transmit(31,value);
  }

};
ORO_CREATE_COMPONENT(RobotControl)
