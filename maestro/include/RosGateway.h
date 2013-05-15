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
