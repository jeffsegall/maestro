#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include "RosGateway.h"

using namespace RTT;

class HuboCtrl : public RTT::TaskContext{
private:  
  RosGateway *gateway;

public:
  HuboCtrl(const std::string& name):
    TaskContext(name)
  {
    this->gateway = new RosGateway("ros_in", "hubo_cmd");
     
    //Initialize input and output ports

    this->addEventPort(*gateway->getInputPort());
    this->addPort(*gateway->getOutputPort());
    
  }
  ~HuboCtrl(){}
private:
  void updateHook(){

    //Check for incoming data and respond if necessary.
    if(gateway->recv()){

    }  
  }
};
ORO_CREATE_COMPONENT(HuboCtrl)
