/*cangateway.h

  Acts as an interface between the Orocos Hubo ROS node and a CAN bus.

  Author: Jeff Segall <js572@drexel.edu>
  Copyright 2011 Drexel University
*/

#ifndef CANGATEWAY_H
#define CANGATEWAY_H

#include "CommsGateway.h"

class CanGateway : public CommsGateway{

public:
    CanGateway();
    virtual ~CanGateway();
    virtual void transmit(void* message);
    virtual void recv(void* data);
    virtual InputPort getInputPort();
    virtual OutputPort getOutputPort();

private:

};

#endif
