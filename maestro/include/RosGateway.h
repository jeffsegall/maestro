/*rosgateway.h

  Acts as an interface between the Orocos Hubo ROS node and other subscribed ROS nodes.

  Author: Jeff Segall <js572@drexel.edu>
  Copyright 2011 Drexel University
*/

#ifndef ROSGATEWAY_H
#define ROSGATEWAY_H

#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <std_msgs/String.h>

using namespace RTT;

class RosGateway {

public:
    RosGateway(std::string inPortName, std::string outPortName);
    ~RosGateway();
    bool transmit(void* message);
    bool recv();
    InputPort<std_msgs::String>* getInputPort();
    OutputPort<std_msgs::String>* getOutputPort();

protected:
    InputPort<std_msgs::String> *inPort;
    OutputPort<std_msgs::String> *outPort;

};

#endif
