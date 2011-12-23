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
    void getHuboTx(unsigned char motorNum, double deg, unsigned char* tx);
    InputPort<unsigned char*> *inPort;
    OutputPort<unsigned char*> *outPort;
};

#endif
