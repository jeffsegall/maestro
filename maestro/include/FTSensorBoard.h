#ifndef FTSENSORBOARD_H
#define FTSENSORBOARD_H

#include "huboCan.h"
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/boost/Float64.h>
#include <std_msgs/boost/String.h>
#include <rtt/Component.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>

using namespace RTT;

class FTSensorBoard {

    private:
        boardNum BNO;
        
        //PUBLISH 
        OutputPort<hubomsg::CanMessage>* canDownPort;

        //SUBSCRIBE
        InputPort<hubomsg::CanMessage>* canUpPort; 

    public:

        FTSensorBoard();
        FTSensorBoard(boardNum BNO);
   
        //PROTOCOL COMMAND MESSAGES
        void setRequestBoardInfo(char CANR);
        void requestBoardParameters(char PARM);
        void requestExecuteNULL(char EFS);
        void setFTMatrixConstant0(int SFT00, int SFT01, int SFT02);
        void setFTMatrixConstant1(int SFT10, int SFT11, int SFT12);
        void setFTMatrixConstant2(int SFT20, int SFT21, int SFT22);
        void setInclinoScaleFactor(int SIF0, int SIF1, int SIF2);
        void setNewBoardNumberFilterFreq(char NEW_BNO, int FREQ10);
        void initBoard();

        //PROTOCOL READ MESSAGES
        void requestFTTiltDigit();
        void requestFTScaleTiltScale();
        void requestFTScaleTiltDigit();
        void requestFTDigitTiltScale();
        void requestFTDigit();
        void requestFTScale();
        void requestTiltDigit();
        void requestTildScale();
        void requestGyroTempData();
};





#endif
