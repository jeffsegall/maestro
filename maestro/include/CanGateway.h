/*cangateway.h

  Acts as an interface between the Orocos Hubo ROS node and a CAN bus.

  Author: Jeff Segall <js572@drexel.edu>
  Copyright 2011 Drexel University
*/

#ifndef CANGATEWAY_H
#define CANGATEWAY_H

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
//using namespace RTT;

class CanGateway{

public:
    CanGateway();
    ~CanGateway();
    void transmit(int joint, float angle);
    void recv(void* data);

    int openCanConnection(char* path);
    int initConnection(int channel);
    void closeCanConnection(int channel);

private:

    char* strToSerial(string packet);

    int channel;

    bool transmit(canmsg_t packet);
    bool transmit(char* packet);

    canmsg_t buildCanPacket(int joint, float angle);
    string buildSerialPacket(int joint, float angle);

    queue<unsigned char*> outQueue;
    queue<unsigned char*> inQueue;

};

#endif
