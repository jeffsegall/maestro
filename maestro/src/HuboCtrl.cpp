/*
HuboCtrl.cpp

The main orocos component for HUBO control, situated inside a ROS node.
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <vector>
#include "RosGateway.h"

using namespace RTT;

class HuboCtrl : public RTT::TaskContext{
private:  
  RosGateway *ros_gateway;
  CanGateway *can_gateway;  

  int i;
  int joints[10];// = {1,4,12,21,28,1,4,12,21,28};
  double values[10];// = {0.0,1.0,-1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0};

public:
  HuboCtrl(const std::string& name):
    TaskContext(name)
  {
    this->ros_gateway = new RosGateway("ros_in", "hubo_cmd");
     
    //Initialize input and output ports
    //Test Comment for Commit Test

    this->addEventPort(*ros_gateway->getInputPort());
    this->addPort(*ros_gateway->getOutputPort());
    this->i = 0;
    this->joints[0]=1;
    this->joints[1]=4;
    this->joints[2]=12;
    this->joints[3]=21;
    this->joints[4]=28;
    this->joints[5]=1;
    this->joints[6]=4;
    this->joints[7]=12;
    this->joints[8]=21;
    this->joints[9]=28;

    this->values[0]=0.0;
    this->values[1]=1.0;
    this->values[2]=-1.0;
    this->values[3]=1.0;
    this->values[4]=1.0;
    this->values[5]=1.0;
    this->values[6]=0.0;
    this->values[7]=0.0;
    this->values[8]=0.0;
    this->values[9]=0.0;
  }
  ~HuboCtrl(){}
private:
  void updateHook(){
    if(i == 0){
        ros_gateway->transmit(joints[i],values[i]);
        i++;
    }else if(i == 10){
        i = 0;
    }else{
        ros_gateway->transmit(joints[i],values[i]);
        i++;
    }

        //Check for incoming data and respond if necessary.
    if(ros_gateway->recv()){

    }  
  }
};
ORO_CREATE_COMPONENT(HuboCtrl)
