#include "CanGateway.h"

using namespace RTT;

CanGateway::CanGateway(std::string inPortName, std::string outPortName){
    this->inPort = new InputPort<unsigned char*>(inPortName);
    this->outPort = new OutputPort<unsigned char*>(outPortName);
}

bool CanGateway::transmit(int joint, float angle){
    unsigned char* message;
    

    return true;
}

bool CanGateway::recv(){

  
}

InputPort<unsigned char*>* CanGateway::getInputPort(){
    return this->inPort;
}

OutputPort<unsigned char*>* CanGateway::getOutputPort(){
    return this->outPort;
}


