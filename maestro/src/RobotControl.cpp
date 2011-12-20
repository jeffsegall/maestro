#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <vector>
#include "RosGateway.h"

using namespace RTT;

class RobotControl : public RTT::TaskContext{
private:  
  RosGateway *gateway;

public:
  RobotControl(const std::string& name):
    TaskContext(name)
  {
    this->gateway = new RosGateway("ros_in", "hubo_cmd");
     
    //Initialize input and output ports
    this->addEventPort(*gateway->getInputPort());
    this->addPort(*gateway->getOutputPort());
  }
  ~RobotControl(){}
private:
  void updateHook(){

    //Check for incoming data and respond if necessary.
    if(gateway->recv()){

    }  
  }

  void setTorsoYaw(double value){
      gateway->transmit(0,value);
  }

  void setHeadYaw(double value){
      gateway->transmit(1,value);
  }

  void setLeftShoulderRoll(double value){
      gateway->transmit(3,value);
  }

  void setLeftShoulderPitch(double value){
      gateway->transmit(4,value);
  }

  void setLeftElbowRoll(double value){
      gateway->transmit(5,value);
  }

  void setLeftElbowYaw(double value){
      gateway->transmit(6,value);
  }

  void setLeftWristRoll(double value){
      gateway->transmit(7,value);
  }

  void setLeftWristPitch(double value){
      gateway->transmit(8,value);
  }

  void setLeftWristYaw(double value){
      gateway->transmit(9,value);
  }

  void setRightShoulderRoll(double value){
      gateway->transmit(11,value);
  }

  void setRightShoulderPitch(double value){
      gateway->transmit(12,value);
  }

  void setRightElbowRoll(double value){
      gateway->transmit(13,value);
  }

  void setRightElbowYaw(double value){
      gateway->transmit(14,value);
  }

  void setRightWristRoll(double value){
      gateway->transmit(15,value);
  }

  void setRightWristPitch(double value){
      gateway->transmit(16,value);
  }

  void setRightWristYaw(double value){
      gateway->transmit(17,value);
  }

  void setLeftHipYaw(double value){
      gateway->transmit(19,value);
  }

  void setLeftHipRoll(double value){
      gateway->transmit(20,value);
  }

  void setLeftHipPitch(double value){
      gateway->transmit(21,value);
  }

  void setLeftKneePitch(double value){
      gateway->transmit(22,value);
  }

  void setLeftFootPitch(double value){
      gateway->transmit(23,value);
  }

  void setLeftFootRoll(double value){
      gateway->transmit(24,value);
  }

  void setRightHipYaw(double value){
      gateway->transmit(26,value);
  }

  void setRightHipRoll(double value){
      gateway->transmit(27,value);
  }

  void setRightHipPitch(double value){
      gateway->transmit(28,value);
  }

  void setRightKneePitch(double value){
      gateway->transmit(29,value);
  }

  void setRightFootPitch(double value){
      gateway->transmit(30,value);
  }

  void setRIghtFootRoll(double value){
      gateway->transmit(31,value);
  }

};
ORO_CREATE_COMPONENT(RobotControl)
