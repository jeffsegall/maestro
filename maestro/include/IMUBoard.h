#ifndef IMUBOARD_H
#define IMUBOARD_H

#include "huboCan.h"
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/boost/Float64.h>
#include <std_msgs/boost/String.h>
#include <rtt/Component.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>

using namespace RTT;

class IMUBoard {

    private:
        boardNum BNO;
        
        //PUBLISH 
        OutputPort<hubomsg::CanMessage>* canDownPort;

        //SUBSCRIBE
        InputPort<hubomsg::CanMessage>* canUpPort; 

    public:

        IMUBoard();
        IMUBoard(boardNum BNO);
   
        //PROTOCOL COMMAND MESSAGES
        void setRequestBoardInfo(char CANR);
        void requestExecuteNULL();
        void requestExecuteCalib();
        void requestParameters(char PRF);
        void setNewBoardNumber(char NEW_BNO);

        //PROTOCOL READ MESSAGES
        void requestAngleAndRate();
};


#endif
