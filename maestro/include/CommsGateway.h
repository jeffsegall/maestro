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

class CommsGateway{

public:
    virtual ~CommsGateway();
    virtual bool transmit(void* message) = 0;
    virtual bool recv(void* data);
    
private:

};

#endif
