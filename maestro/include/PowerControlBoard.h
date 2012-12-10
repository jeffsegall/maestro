#ifndef POWERCONTROLBOARD_H
#define POWERCONTROLBOARD_H

#include "huboCan.h"
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/boost/Float64.h>
#include <std_msgs/boost/String.h>
#include <rtt/Component.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>

using namespace RTT;

class PowerControlBoard {

    private:
        boardNum BNO;
        
        //PUBLISH 
        OutputPort<hubomsg::CanMessage>* canDownPort;

        //SUBSCRIBE
        InputPort<hubomsg::CanMessage>* canUpPort; 
    public:

        PowerControlBoard();
        PowerControlBoard(boardNum BNO);

        void setRequestBoardInfo(char CANR);
        void setSwitchFunction(char SFUNC);
        void requestAlarm(char ALRM);
        void requestBeep(char BDUR);
        void requestVoltageAndCurrent();
        void requestTimeAndStatus();

};

#endif
