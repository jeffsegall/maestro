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
#include <hubomsg/typekit/HuboCmd.h>

using namespace RTT;

class RosGateway {

public:
    RosGateway(std::string inPortName, std::string outPortName);
    ~RosGateway();
    bool transmit(int joint, float angle);
    bool recv();
    InputPort<hubomsg::HuboCmd>* getInputPort();
    OutputPort<hubomsg::HuboCmd>* getOutputPort();

protected:
    InputPort<hubomsg::HuboCmd> *inPort;
    OutputPort<hubomsg::HuboCmd> *outPort;

};

#endif
