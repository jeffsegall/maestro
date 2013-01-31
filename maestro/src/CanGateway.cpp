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

    tempYaw = 0;
    rightHipEnabled = false;
    tempRoll = 0;
    tempOutput.open("/home/hubo/maestro/outputlog.txt");
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
        sent = write(this->channel, packet, 1);
    }

    if (sent < 1){
        //Not all the data was sent
        std::cout << "Data transmission error" << std::endl;
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
void CanGateway::initConnection(int channel, int bitrate){
    if (SERIALTEST){
        //Send speed and open packet
        transmit(strToSerial("s8"));
	transmit(strToSerial("O"));
    }
    else{
        Config_par_t cfg;
        volatile Command_par_t cmd;
        
        cmd.cmd = CMD_STOP;
        ioctl(channel, CAN_IOCTL_COMMAND, &cmd);
    
        cfg.target = CONF_TIMING;
        cfg.val1 = (unsigned int)bitrate;
        ioctl(channel, CAN_IOCTL_CONFIG, &cfg);
    
        cmd.cmd = CMD_START;
        ioctl(channel, CAN_IOCTL_COMMAND, &cmd);
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
        cmd.cmd = CMD_STOP;
        ioctl(channel, CAN_IOCTL_COMMAND, &cmd);
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

    while (NewData==this->inPort->read(inMsg)){
        can_message = canMsg((boardNum)inMsg.bno, (messageType)inMsg.mType, (cmdType)inMsg.cmdType,
                             inMsg.r1, inMsg.r2, inMsg.r3, inMsg.r4, inMsg.r5, inMsg.r6, inMsg.r7, inMsg.r8);
        tempOutput << "Message received! r1: " << inMsg.r1 << std::endl;
        //Add message to queue
        //if (!this->downQueue->empty())
            //this->downQueue->pop();

        switch (inMsg.mType){
        case TX_REF:{
        	//tempOutput << "Reference Command Received!" << std::endl;
        	// Maximum of 5 arguments, unused args are 0. We'll worry about specific boards later.
        	State vals = { {inMsg.r1, inMsg.r2, inMsg.r3, inMsg.r4, inMsg.r5} };
        	positions[(boardNum)inMsg.bno] = vals;
        	break;
		} case TX_MOTOR_CMD:
        	switch (inMsg.cmdType){
        	case CMD_CONTROLLER_ON:
        		flags[(boardNum)inMsg.bno] = true; //Set a flag to enable sending positions to this board
        		break;
        	case CMD_CONTROLLER_OFF:
        		flags[(boardNum)inMsg.bno] = false; //Unset the flag to stop sending positions to this board
        		break;
        	default: break;
        	} // no break here - we want to fall into default case.
		default:
			this->downQueue->push(can_message);
			break;
        }
        /*
        if (inMsg.bno == BNO_R_HIP_YAW_ROLL && inMsg.mType == TX_REF && inMsg.cmdType == 2) {
        	tempYaw = inMsg.r1;
        	tempRoll = inMsg.r2;
        } else if (inMsg.bno == BNO_R_HIP_YAW_ROLL && inMsg.mType == TX_MOTOR_CMD && inMsg.cmdType == CMD_CONTROLLER_ON){
        	this->downQueue->push(can_message);
        	rightHipEnabled = true;
        } else if (inMsg.bno == BNO_R_HIP_YAW_ROLL && inMsg.mType == TX_MOTOR_CMD && inMsg.cmdType == CMD_CONTROLLER_OFF){
			this->downQueue->push(can_message);
			rightHipEnabled = false;
        } else
        	this->downQueue->push(can_message);
		*/
        //std::cout << this->downQueue->size() << std::endl;
    }

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
    canmsg_t** rx = new canmsg_t*[5]; 
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
                this->upQueue->push(canMsg::fromLineType(rx[i]));
                //std::cout << "read in a message from CAN" << std::endl;
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
    std::cout << "Opened CAN connection on channel " << channel << std::endl;
    if (this->channel > -1){
        initConnection(this->channel, 1000);
    canMsg name_info = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_SETREQ_BOARD_INFO,
                              0x05, 0, 0, 0, 0, 0, 0, 0);

    canMsg hip_on = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_HIP_ENABLE,
                           0x01, 0, 0, 0, 0, 0, 0, 0);
    canMsg run = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_CONTROLLER_ON,
                        0, 0, 0, 0, 0, 0, 0, 0);

        name_info.printme();
        hip_on.printme();
        run.printme();
//        transmit(name_info.toCAN());
//        transmit(hip_on.toCAN());
//        transmit(run.toCAN());      
//        write(channel, name_info.toCAN(), 1);
//        write(channel, hip_on.toCAN(), 1);
//        write(channel, run.toCAN(), 1);
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
    //runTick();
	static map<boardNum, bool >::iterator it = flags.begin();

    recvFromRos();
    if (it == flags.end()){
    	it = flags.begin();
    	//tempOutput << "Reached end of map! Returning to the beginning." << std::endl;
    } else if (downQueue->empty()) { // If we have nothing else to send, send a position.
    	while (!it->second && it != flags.end()) //Move to an enabled board in our map, or to the end of the list
    		it++;
    	if (it != flags.end()) {
    		//TODO: Search for key in positions, if not found then break. if found, send the packet. iterate iterator regardless.
    		tempOutput << "Found enabled board!" << std::endl;
    		map<boardNum, State>::iterator i = positions.find(it->first);
    		if (i != positions.end()){
				// we have found an enabled board, so let's send a packet and increment the iterator.
				switch (i->first){ // Which board are we talking to?
				case BNO_WAIST :
				case BNO_L_HIP_PITCH :
				case BNO_L_KNEE :
				case BNO_R_HIP_PITCH :
				case BNO_R_KNEE :  // 1 Motor Channel (treated as 2 motor channels)
					downQueue->push(canMsg(i->first, (messageType)TX_REF, (cmdType)2,
								i->second.values[0], 0, 0, 0, 0, 0, 0, 0)); // Send out a position command with preset arguments
					break;
				case BNO_L_SHOULDER_PITCH_ROLL :
				case BNO_L_SHOULDER_YAW_ELBOW :
				case BNO_L_WRIST_YAW_PITCH :
				case BNO_R_SHOULDER_PITCH_ROLL :
				case BNO_R_SHOULDER_YAW_ELBOW :
				case BNO_R_WRIST_YAW_PITCH :
				case BNO_L_HIP_YAW_ROLL :
				case BNO_L_ANKLE_PITCH_ROLL :
				case BNO_R_HIP_YAW_ROLL:
				case BNO_R_ANKLE_PITCH_ROLL :// 2 Motor Channels
					downQueue->push(canMsg(it->first, (messageType)TX_REF, (cmdType)2,
								i->second.values[0], i->second.values[1], 0, 0, 0, 0, 0, 0)); // Send out a position command with preset arguments
					break;
				case BNO_NECK_YAW_1_2 : // 3 Motor Channels
					downQueue->push(canMsg(it->first, (messageType)TX_REF, (cmdType)2,
								i->second.values[0], i->second.values[1], i->second.values[2], 0, 0, 0, 0, 0)); // Send out a position command with preset arguments
					break;
				case BNO_R_HAND :
				case BNO_L_HAND : // 5 Motor Channels
					downQueue->push(canMsg(it->first, (messageType)TX_REF, (cmdType)2,
								i->second.values[0], i->second.values[1], i->second.values[2],
								i->second.values[3], i->second.values[4], 0, 0, 0)); // Send out a position command with preset arguments
					break;
				default:
					break;
				}
			}
		}

		it++;
	}

    /*
    if (downQueue->empty() && rightHipEnabled){
		downQueue->push(canMsg((boardNum)BNO_R_HIP_YAW_ROLL, (messageType)TX_REF, (cmdType)2,
				tempYaw, tempRoll, 0, 0, 0, 0, 0, 0));
	}
	*/

    canmsg_t rx[5];

    if (!this->downQueue->empty()){
        transmit(this->downQueue->front().toCAN());
        tempOutput << "Transmitting message! r1: " << this->downQueue->front().getR1() << std::endl;
        this->downQueue->pop();
    }
    
    if (this->channel > 0){
        int messages_read = read(this->channel, &rx, 5);
        if (messages_read > 0){
            //std::cout << "read in a message from CAN" << std::endl;
            for (int i = 0; i < messages_read; i++){
                //Rebuild a canMsg and add it to the upstream buffer
                this->upQueue->push(canMsg::fromLineType(&rx[i]));
                //std::cout << "read in a message from CAN" << std::endl;
            }
        }
    } 

    transmitToRos();
}

/******************************************************************
* startHook()
* 
* Called at shutdown.
******************************************************************/
void CanGateway::stopHook(){
    closeCanConnection(this->channel);
    this->tempOutput.close();
}

ORO_LIST_COMPONENT_TYPE(CanGateway)
