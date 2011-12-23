#include "CanGateway.h"

using namespace RTT;

CanGateway::CanGateway(std::string inPortName, std::string outPortName){
    this->inPort = new InputPort<unsigned char*>(inPortName);
    this->outPort = new OutputPort<unsigned char*>(outPortName);
}

bool CanGateway::transmit(int joint, float angle){
    unsigned char* message;
    getHuboTx((unsigned char)joint, angle, message);
    this->outPort->write(message);
    return true;
}

bool CanGateway::recv(){
    hubomsg::HuboCmd inCommand = hubomsg::HuboCmd();
  
    //If a command message comes in from an external source, rebroadcast it.
    if(NewData==this->inPort->read(inCommand)){
        if (inCommand.msg == "cmd")
	    transmit(inCommand.joint, inCommand.angle);
        return true;
    }
    return false;
}

InputPort<unsigned char*>* CanGateway::getInputPort(){
    return this->inPort;
}

OutputPort<unsigned char*>* CanGateway::getOutputPort(){
    return this->outPort;
}

void getHuboTx( unsigned char motorNum, double deg, unsigned char* tx)
{
    bool isNeg = false;					

    int degInt	= std::floor(std::abs(deg));		

    int degRem = (std::abs(deg) - degInt) * 100;		

    tx[0] = 255; 		// header
    tx[1] = 65;			// char(65) = A, this says to use advanced mode
    tx[2] = 1;			// number of motors to send to
    tx[3] = motorNum;	

    if(deg < 0)
    {
        isNeg = true;
    }

    tx[4] = (unsigned char)(( degInt >> 8 ) & 127);
    tx[5] = (unsigned char)(degInt & 255);

    if( degRem > 100)
    {
        degRem = 99;    
    }

    tx[6] = (unsigned char)(degRem & 255);

    if(isNeg)
    {		
        tx[4] = tx[4] | 128;
    }

    tx[7] = (unsigned char)( (((unsigned int)tx[3] +
                           (unsigned int)tx[4] +
                           (unsigned int)tx[5] +
                           (unsigned int)tx[6])*tx[2])
                        & 255);
}


