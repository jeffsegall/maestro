/*cangateway.h

  Acts as an interface between the Orocos Hubo ROS node and a CAN bus.

  Author: Jeff Segall <js572@drexel.edu>
  Copyright 2011 Drexel University
*/

#ifndef CANGATEWAY_H
#define CANGATEWAY_H

#include "CommsGateway.h"
#include "can4linux.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string>
#include "huboCan.h"
#include <queue>

using namespace std;

class CanGateway : public CommsGateway{

public:
    CanGateway();
    virtual ~CanGateway();
    virtual void transmit(int joint, float angle);
    virtual void recv(void* data);
    virtual InputPort getInputPort();
    virtual OutputPort getOutputPort();

    int openCanConnection(char* path);
    int initConnection(int channel);
    void closeCanConnection(int channel);

private:
    void getHuboTx(unsigned char motorNum, double deg, unsigned char* tx);
    InputPort<unsigned char*> *inPort;
    OutputPort<unsigned char*> *outPort;

    char* strToSerial(string packet);

    int channel;

    bool transmit(canmsg_t packet);
    bool transmit(char* packet);

    canmsg_t buildCanPacket();
    string buildSerialPacket();

    queue outQueue;
    queue inQueue;

};

#endif
