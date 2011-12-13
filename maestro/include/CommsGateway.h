/*commsgateway.h

  Acts as an interface between the Orocos Hubo ROS node and output channels,
  whether they be other ROS nodes or physical hardware.

  Author: Jeff Segall <js572@drexel.edu>
  Copyright 2011 Drexel University
*/

#ifndef COMMSGATEWAY_H
#define COMMSGATEWAY_H

#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <std_msgs/String.h>

class CommsGateway{

public:
    virtual ~CommsGateway();
    virtual void transmit(void* message) = 0;
    virtual void recv(void* data);
  
private:

};

#endif
