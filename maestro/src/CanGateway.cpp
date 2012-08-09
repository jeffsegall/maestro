#include "CanGateway.h"

//Output to a serial CAN adapter instead of directly to a CAN bus
#define SERIALTEST 0
#define DEBUG 0

/******************************************************************
* CanGateway()
* Constructor
*
* Initializes the queues and ports necessary for communication
* with ROS.  Does not initialize CAN communication.
******************************************************************/
CanGateway::CanGateway(const std::string& name):
      TaskContext(name){

    this->upQueue = new queue<canMsg>();
    this->downQueue = new queue<canMsg>();

    this->inPort = new InputPort<hubomsg::CanMessage>("can_down");
    this->outPort = new OutputPort<hubomsg::CanMessage>("can_up");

    this->addEventPort(*inPort);
    this->addPort(*outPort);
}

CanGateway::~CanGateway(){

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
    else{
        Config_par_t cfg;
        volatile Command_par_t cmd;
        
        cmd.cmd = CMD_STOP;
        ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
    
        cfg.target = CONF_TIMING;
        cfg.val1 = (unsigned int)bitrate;
        ret = ioctl(fd, CAN_IOCTL_CONFIG, &cfg);
    
        cmd.cmd = CMD_START;
        ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
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
    else{
        volatile Command_par_t cmd;
        cmd.cmd = cmd.STOP;
        ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
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
        can_message = canMsg((boardNum)inMsg.bno, (messageType)inMsg.mType, (cmdType)inMsg.cmdType,
                             inMsg.r1, inMsg.r2, inMsg.r3, inMsg.r4, inMsg.r5, inMsg.r6, inMsg.r7, inMsg.r8);
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

    canMsg out;

    for (int i = 0; i < upQueue->size(); i++){
        hubomsg::CanMessage upstream = hubomsg::CanMessage();
        out = upQueue->front();
        upQueue->pop();
     
        //Set up ROS message parameters
        upstream.bno = out.getBNO();
        upstream.mType = out.getType();
        upstream.cmdType = out.getCmd();
        upstream.r1 = out.getR1();
        upstream.r2 = out.getR2();
        upstream.r3 = out.getR3();
        upstream.r4 = out.getR4();
        upstream.r5 = out.getR5();
        upstream.r6 = out.getR6();
        upstream.r7 = out.getR7();
        upstream.r8 = out.getR8();

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

    int messages_read = 0;
    //Also make an attempt to read in from hardware
    if (SERIALTEST){

    }
    else {
        messages_read = read(this->channel, &rx, 5);
        if (messages_read > 0){
            for (int i = 0; i < messages_read; i++){
                //Rebuild a canMsg and add it to the upstream buffer
                this->upQueue->push(canMsg.fromLineType(rx[i]));
            }
        } 
    }

}

/******************************************************************
* startHook()
* 
* Called at start.
******************************************************************/
bool CanGateway::startHook(){
    //@TODO: This should be some sort of parameter...not hardcoded
    this->channel = openCanConnection("/dev/can0");
    if (this->channel > -1){
        initConnection(this->channel);
        return 1;
    }
    return 0;
}

/******************************************************************
* updateHook()
* 
* Called each iteration.
******************************************************************/
void CanGateway::updateHook(){
    runTick();
}

/******************************************************************
* startHook()
* 
* Called at shutdown.
******************************************************************/
void CanGateway::stopHook(){
    closeCanConnection(this->channel);
}

ORO_LIST_COMPONENT_TYPE(CanGateway)
