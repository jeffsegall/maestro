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
#include "huboCan.h"

#define DEFAULT_CHANNELS 2

/* The MotorBoard class represents a single motor board in the HUBO+ */

class MotorBoard {

    private:
        boardNum BNO; 
        int channels;
        HuboMotor** motors;

        //PUBLISH
        OutputPort<hubomsg::CanMessage>* canDownPort;
        OutputPort<hubomsg::HuboCmd>* orInPort;

        //SUBSCRIBE
        InputPort<hubomsg::CanMessage>* canUpPort;
        InputPort<hubomsg::HuboCmd>* orOutPort;
 
    public:
        
        MotorBoard();
        MotorBoard(int channels);

        void addMotor(HuboMotor* motor, int channel);
        void removeMotor(HuboMotor* motor);
        void removeMotor(int channel); 

        // PROTOCOL MOTOR COMMANDS

        void setRequestBoardInfo(char CANR);
        void requestBoardStatus();
        void requestEncoderPosition(char FES);
        void requestCurrent();
        void resetEncoderToZero(char CH);
        void setPositionGain(char CH, int Kp, int Ki, int Kd);
        void setCurrentGain(char CH, int KPt, int KDt, int Kf);
        void setHIP(char HIP_EN);
        void openLoop(char PUL_ON, char DIR0, char DUTY0, char DIR1, char DUTY1);
        void openLoop(char PUL_ON, char D_DT0, char D_DT1, char D_DT2, char D_DT3, char D_DT4);
        void openLoop(char PUL_ON, char D_DT0, char D_DT1, char D_DT2);
        void enableController();
        void disableController();
        void setControlMode(char FBC);
        void goToHomeOffset(char CHD, char SDR, int H_OFFSET);
        void setDeadZone(char CH, char DZone);
        void requestBoardParams(char PARM);
        void setHomeSearchParams(char CH, char SRL, char SDR, int OFFSET);
        void setEncoderResolution(char CH, int ENC_RE);
        void setEncoderResolution(char CH, int ERS, char AS, char MD);
        void setMaxAccVel(char CH, int MACC, int MVEL);
        void setLowerPosLimit(char CH, char MPS, int MPOS1);
        void setUpperPosLimit(char CH, char MPS, int MPOS2);
        void setHomeVelAcc(char CH, char HMA, char HMV1, char HMV2, char SRM, char LIMD);
        void setGainOverride(char GOVW0, char GOVW1, int GDUR);
        void setNewBoardNum(char NEW_BNO, char CANR);
        void setJamPwmSatLim(int JAM_LIM, int PWM_LIM, char LIMD, char JAMD);
        void setErrorBound(int I_ERR, int B_ERR, int E_ERR);
        void initBoard();

        //PROTOCOL REFERENCE MESSAGES

        void sendPositionReference(int REF0, int REF1);
        void sendPositionReference(char REF0, char REF1, char REF2);
        void sendPositionReference(char REF0, char REF1, char REF2, char REF3, char REF4);
         
};


#endif
