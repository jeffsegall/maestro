#ifndef IMUBOARD_H
#define IMUBOARD_H

#include "huboCan.h"
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>
#include <hubomsg/typekit/HuboState.h>

using namespace RTT;

class IMUBoard {

    private:
        boardNum BNO;
        string name;

        double xAcc, yAcc, zAcc;
        double xRot, yRot;
        
        //PUBLISH 
        OutputPort<hubomsg::HuboState>* huboDownPort;

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

        //NEW DATA
		string getName();
		double getXAcc();
		double getYAcc();
		double getZAcc();
		double getXRot();
		double getYRot();

		void update(double xAcc, double yAcc, double zAcc, double xRot, double yRot);
		void setName(string name);
};


#endif
