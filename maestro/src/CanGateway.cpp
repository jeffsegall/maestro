#include "CanGateway.h"

//Output to a serial CAN adapter instead of directly to a CAN bus
#define SERIALTEST 1

/******************************************************************
* CanGateway()
* Constructor
*
* Initializes the queues and ports necessary for communication
* with ROS.  Does not initialize CAN communication.
******************************************************************/
CanGateway::CanGateway(){

    this->upQueue = new queue<canMsg>();
    this->downQueue = new queue<canMsg>();

    this->inPort = new InputPort<hubomsg::CanMessage>("can_down");
    this->outPort = new OutputPort<hubomsg::CanMessage>("can_up");
}


/******************************************************************
* strToSerial()
* 
* Converts a std::string formatted CAN packet into a character
* array for communication over USB.  Adds a 0x0D (carriage return)
* onto the end of the array as per the protocol.
* 
* Protocol is tested for the EasySync USB2-F-7x01 adapter
*
* Paramters:
*    packet - a string representation of a CAN packet
*
* Returns a character array representation of the input string
* with a trailing carriage return.
******************************************************************/
char* CanGateway::strToSerial(string packet){
    char* data = new char[packet.length() + 1];

    strcpy(data, packet.c_str());

    data[packet.length()] = (char) 0x0D;
    
    return data;
}

/******************************************************************
* transmit()
* 
* Sends a canmsg_t (can4linux) packet over a hardware channel.
*
* Parameters:
*    packet - a canmsg_t formatted packet to transmit to hardware
*
* Returns true if data was sent successfully, false otherwise.
******************************************************************/
bool CanGateway::transmit(canmsg_t* packet){
    int sent = 0;

    //Make sure there's an outbound channel
    if (this->channel > 0){
        //Amount to send is always 1 for canmsg_t (see can4linux.h)
        sent = write(this->channel, &packet, 1);
    }

    if (sent < 1){
        //Not all the data was sent
        return false;
    }

    return true;
}

/******************************************************************
* transmit()
* 
* Sends a char* packet over a hardware channel.
*
* Parameters:
*    packet - a char* formatted packet to transmit to hardware
*
* Returns true if data was sent successfully, false otherwise.
******************************************************************/
bool CanGateway::transmit(char* packet){
    int sent = 0;

    //Make sure there's an outbound channel
    if (this->channel > 0){
        sent = write(this->channel, packet, strlen(packet));
    }

    if (sent < strlen(packet)){
        //Not all the data was sent
        return false;
    }

    return true;
}

/******************************************************************
* openCanConnection()
* 
* Opens a hardware channel file descriptor for communication.
* 
* Parameters:
*    path - the path to a file descriptor (e.g. /dev/ttyUSB0)
*
* Returns the channel number to use for communication.
******************************************************************/
int CanGateway::openCanConnection(char* path){
    //Read/Write and non-blocking.  Should be the same for
    //serial or CAN hardware.
    int channel = open(path, O_RDWR | O_NONBLOCK);
    return channel;
}

/******************************************************************
* initConnection()
*
* Initializes a CAN connection.  Sends the appropriate packets to
* set channel speed and open a connection, if necessary.
*
* Paramters:
*    channel - the channel to initialize.  obtained by a successful
*              call to openCanConnection()
******************************************************************/
void CanGateway::initConnection(int channel){
    if (SERIALTEST){
        //Send speed and open packet
        transmit(strToSerial("s8"));
	transmit(strToSerial("O"));
    } 
}

/******************************************************************
* closeCanConnection()
*
* Closes a CAN connection.  Sends the appropriate packets to close
* the connection if necessary.
*
* Parameters:
*    channel - the channel to close.  obtained by a successful call
*              to openCanConnection()
******************************************************************/
void CanGateway::closeCanConnection(int channel){
    if (SERIALTEST){
        //Send close packet
        transmit(strToSerial("C")); 
    }
    close(channel);
}

/******************************************************************
* recvFromRos()
* 
* Attempts to receive new data from the subscribed ROS topic.  Adds
* new data (if available) to the hardware send queue.
******************************************************************/
void CanGateway::recvFromRos(){
    hubomsg::CanMessage inMsg = hubomsg::CanMessage();
    canMsg can_message;

    //If a new message has come in from ROS, grab the CAN information

    if (NewData==this->inPort->read(inMsg)){
        //can_message = inMsg.can;
    }

    //Add message to queue
    this->downQueue->push(can_message); 
}

/******************************************************************
* transmitToRos()
* 
* Transmits all queued messages from hardware back up to ROS.
******************************************************************/
void CanGateway::transmitToRos(){

    //Flush our upstream queue out to the ROS bus

    for (int i = 0; i < upQueue->size(); i++){
        hubomsg::CanMessage upstream = hubomsg::CanMessage();
        //upstream.can = upQueue->front();
        upQueue->pop();
        this->outPort->write(upstream);
    }
}

/******************************************************************
* getInputPort()
*
* Returns the InputPort used for ROS communication.
******************************************************************/
InputPort<hubomsg::CanMessage>* CanGateway::getInputPort(){
    return this->inPort;
}

/******************************************************************
* getOutputPort()
*
* Returns the OutputPort used for ROS communication.
******************************************************************/
OutputPort<hubomsg::CanMessage>* CanGateway::getOutputPort(){
    return this->outPort;
}

/******************************************************************
* runTick()
* 
* Called each clock interval.  Sends the next message in the queue
* to the hardware.
******************************************************************/
void CanGateway::runTick(){
   
    //At each clock interval (50 ms?) send a message out to the hardware.
    canMsg out_message = this->downQueue->front();
    this->downQueue->pop();

    if (SERIALTEST){
        //Format message for serial output
        char* data = strToSerial(out_message.toSerial());
        this->transmit(data);
    }
    else {
        //Format message for CAN output
        canmsg_t* data = out_message.toCAN();
        this->transmit(data);
    } 

}
