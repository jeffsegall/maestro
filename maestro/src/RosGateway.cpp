#include "RosGateway.h"

using namespace RTT;

RosGateway::RosGateway(std::string inPortName, std::string outPortName){
    this->inPort = new InputPort<hubomsg::HuboCmd>(inPortName);
    this->outPort = new OutputPort<hubomsg::HuboCmd>(outPortName);
}

bool RosGateway::transmit(int joint, float angle){
    hubomsg::HuboCmd outCommand = hubomsg::HuboCmd();
    outCommand.joint = joint;
    outCommand.angle = angle;
    outCommand.msg = "cmd";
    this->outPort->write(outCommand);
    return true;
}

bool RosGateway::recv(){
    hubomsg::HuboCmd inCommand = hubomsg::HuboCmd();
    if(NewData==this->inPort->read(inCommand)){
        if (inCommand.msg == "cmd")
	    transmit(inCommand.joint, inCommand.angle);
        return true;
    }
    return false;
}

InputPort<hubomsg::HuboCmd>* RosGateway::getInputPort(){
    return this->inPort;
}

OutputPort<hubomsg::HuboCmd>* RosGateway::getOutputPort(){
    return this->outPort;
}
