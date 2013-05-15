/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
