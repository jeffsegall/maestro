#include "CanGateway.h"

#define SERIALTEST 1

CanGateway::CanGateway(){

    if (SERIALTEST){
        inQueue = queue<unsigned char*>();
        outQueue = queue<unsigned char*>();
    }
//    else{
//        inQueue = new queue<canmsg_t*>();
//        outQueue = new queue<canmsg_t*>();
//    }
}

canmsg_t buildCanPacket(int joint, float angle){
    return canmsg_t();
}

string buildSerialPacket(int joint, float angle){
    return canMsg.buildSetTicksPacket(boardNum(joint), angle).toSerial();
}

void CanGateway::transmit(int joint, float angle){

    if (SERIALTEST){
        //Make serial message and transmit. 
        unsigned char* message;    
        message = strToSerial( buildSerialPacket(joint, angle) );
    }
    else{
        //Make CAN message and transmit.
        canmsg_t message;
        message = buildCanPacket(joint, angle);
    }
    
    transmit(message);
}

char* CanGateway::strToSerial(string packet){
    char* data = new char[packet.length() + 1];

    strcpy(data, packet.c_str());

    data[packet.length()] = (char) 0x0D;
    
    return data;
}

bool CanGateway::transmit(canmsg_t packet){
    int sent = 0;

    //Make sure there's an outbound channel
    if (this.channel > 0){
        sent = write(this.channel, &packet, 1);
    }

    if (sent < 1){
        //Not all the data was sent
        return false;
    }

    return true;
}

bool CanGateway::transmit(char* packet){
    int sent = 0;

    //Make sure there's an outbound channel
    if (this.channel > 0){
        sent = write(this.channel, packet, strlen(packet));
    }

    if (sent < strlen(packet)){
        //Not all the data was sent
        return false;
    }

    return true;
}

int CanGateway::openCanConnection(char* path){
    //Read/Write and non-blocking.  Should be the same for
    //serial or CAN hardware.
    int channel = open(path, O_RDWR | O_NONBLOCK);
    return channel;
}

int CanGateway::initConnection(int channel){
    if (SERIALTEST){
        //Send speed and open packet
        transmit(stringToSerial("s8"));
	transmit(stringToSerial("O"));
    } 
}

void CanGateway::closeCanConnection(int channel){
    if (SERIALTEST){
        //Send close packet
        transmit(stringToSerial("C")); 
    }
    close(channel);
}

bool CanGateway::recv(){

}


