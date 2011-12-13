#include "RosGateway.h"

using namespace RTT;

RosGateway::RosGateway(std::string inPortName, std::string outPortName){
    this->inPort = new InputPort<std_msgs::String>(inPortName);
    this->outPort = new OutputPort<std_msgs::String>(outPortName);
}

bool transmit(void* message){
     //this->outPort->write(message);
     return true;
}

bool RosGateway::recv(){
    std_msgs::String sdata;// = *static_cast<std_msgs::String>(data);
    if(NewData==this->inPort->read(sdata)){
        log(Info)<<"String in: "<<sdata<<endlog();
        this->outPort->write(sdata);
        return true;
    }
    return false;
}

InputPort<std_msgs::String>* RosGateway::getInputPort(){
    return this->inPort;
}

OutputPort<std_msgs::String>* RosGateway::getOutputPort(){
    return this->outPort;
}
