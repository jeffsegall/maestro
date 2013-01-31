/*cangateway.h

  Acts as an interface between the ROS and a CAN bus.

  Author: Jeff Segall <js572@drexel.edu>
  Copyright 2011-2012 Drexel University
*/

#ifndef CANGATEWAY_H
#define CANGATEWAY_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "can4linux.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string>
#include "huboCan.h"
#include <queue>
#include <map>
#include <vector>
#include <hubomsg/typekit/CanMessage.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace RTT;

class CanGateway : public RTT::TaskContext{

public:
    CanGateway(const std::string&);
    ~CanGateway();

    //ROS COMMUNICATION
    void recvFromRos();
    void transmitToRos();
    InputPort<hubomsg::CanMessage>* getInputPort();
    OutputPort<hubomsg::CanMessage>* getOutputPort();

    //HARDWARE COMMUNICATION
    int openCanConnection(char* path);
    void initConnection(int channel, int bitrate);
    void closeCanConnection(int channel);

    //RUN LOOP
    void runTick();
    void updateHook();

    bool startHook();
    void stopHook();

private:

    struct State { int values[5]; };

    char* strToSerial(string packet);

    int channel;

    bool transmit(canmsg_t* packet);
    bool transmit(char* packet);


    InputPort<hubomsg::CanMessage> *inPort;
    OutputPort<hubomsg::CanMessage> *outPort;

    queue<canMsg>* upQueue;
    queue<canMsg>* downQueue;

    map<boardNum, State > positions;
    map<boardNum, bool> flags;

    long tempYaw, tempRoll;
    bool rightHipEnabled;
    ofstream tempOutput;

};

#endif
