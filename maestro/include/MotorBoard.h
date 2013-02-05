#ifndef MOTORBOARD_H
#define MOTORBOARD_H

#include "HuboMotor.h"
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>
#include <string>
#include "huboCan.h"
#include "ros/ros.h"
#include <vector>
#include <queue>
#include <fstream>

#define DEFAULT_CHANNELS 2
using namespace RTT;
using std::vector;
using std::queue;

/* The MotorBoard class represents a single motor board in the HUBO+ */

class MotorBoard{// : public RTT::TaskContext {

    private:
        boardNum BNO; 
        int channels;
        vector<HuboMotor*> motors;

        //PUBLISH
        OutputPort<hubomsg::CanMessage>* canDownPort;
        OutputPort<hubomsg::HuboCmd>* orInPort;

        //SUBSCRIBE
        InputPort<hubomsg::CanMessage>* canUpPort;
        InputPort<hubomsg::HuboCmd>* orOutPort;

        queue<hubomsg::CanMessage>* outQueue;
 
    public:
        
        MotorBoard();
        MotorBoard(boardNum BNO, int channels, queue<hubomsg::CanMessage>* outQueue);
        MotorBoard(const MotorBoard& rhs);

        void addMotor(HuboMotor* motor, int channel);
        void removeMotor(HuboMotor* motor);
        void removeMotor(int channel);
        void setTicksPosition(vector<long> ticks);
        HuboMotor* getMotorByChannel(int channel);
        boardNum getBoardNumber();
        bool requiresMotion();

        // PROTOCOL MOTOR COMMANDS

        canMsg* setRequestBoardInfo(char CANR);
        canMsg* requestBoardStatus();
        canMsg* requestEncoderPosition(char FES);
        canMsg* requestCurrent();
        canMsg* resetEncoderToZero(char CH);
        canMsg* setPositionGain(char CH, int Kp, int Ki, int Kd);
        canMsg* setCurrentGain(char CH, int KPt, int KDt, int Kf);
        canMsg* setHIP(char HIP_EN);
        canMsg* openLoop(char PUL_ON, char DIR0, char DUTY0, char DIR1, char DUTY1);
        canMsg* openLoop(char PUL_ON, char D_DT0, char D_DT1, char D_DT2, char D_DT3, char D_DT4);
        canMsg* openLoop(char PUL_ON, char D_DT0, char D_DT1, char D_DT2);
        canMsg* enableController();
        canMsg* disableController();
        canMsg* setControlMode(char FBC);
        canMsg* goToHomeOffset(char CHD, char SDR, int H_OFFSET);
        canMsg* setDeadZone(char CH, char DZone);
        canMsg* requestBoardParams(char PARM);
        canMsg* setHomeSearchParams(char CH, char SRL, char SDR, int OFFSET);
        canMsg* setEncoderResolution(char CH, int ENC_RE);
        canMsg* setEncoderResolution(char CH, int ERS, char AS, char MD);
        canMsg* setMaxAccVel(char CH, int MACC, int MVEL);
        canMsg* setLowerPosLimit(char CH, char MPS, int MPOS1);
        canMsg* setUpperPosLimit(char CH, char MPS, int MPOS2);
        canMsg* setHomeVelAcc(char CH, char HMA, char HMV1, char HMV2, char SRM, char LIMD);
        canMsg* setGainOverride(char GOVW0, char GOVW1, int GDUR);
        canMsg* setNewBoardNum(char NEW_BNO, char CANR);
        canMsg* setJamPwmSatLim(int JAM_LIM, int PWM_LIM, char LIMD, char JAMD);
        canMsg* setErrorBound(int I_ERR, int B_ERR, int E_ERR);
        canMsg* initBoard();

        //PROTOCOL REFERENCE MESSAGES

        canMsg* sendPositionReference(vector<int> REF, int MAX_STEP = 75, int MIN_STEP = 5);
        canMsg* sendPositionReference(int MAX_STEP = 150, int MIN_STEP = 5);
        /*
        canMsg* sendPositionReference(char REF0, char REF1, char REF2);
        canMsg* sendPositionReference(char REF0, char REF1, char REF2, char REF3, char REF4);
		*/
        canMsg* sendPositionReferenceRadians(double rad0, double rad1);
        canMsg* sendPositionReferenceRadians(double rad0, double rad1, double rad2);
        canMsg* sendPositionReferenceRadians(double rad0, double rad1, double rad2, double rad3, double rad4);
         
};


#endif
