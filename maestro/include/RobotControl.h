#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>
#include <vector>
#include <queue>
#include "huboCan.h"
#include "HuboState.h"
#include "HuboMotor.h"
#include "MotorBoard.h"
#include <fstream>

using std::queue;
using std::vector;
using namespace RTT;

class RobotControl : public RTT::TaskContext{

public:
    RobotControl(const std::string&);
    ~RobotControl();

    void updateHook(); 
    hubomsg::CanMessage buildCanMessage(canMsg* msg);

    void initRobot(string path);

    //JOINT MOVEMENT API
    void setRightHipYaw(int ticks, int delay);
    void setRightHipRoll(int ticks, int delay);
    void setRightHipPitch(int ticks, int delay);
    void setRightKnee(int ticks, int delay);
    void setRightAnklePitch(int ticks, int delay);
    void setRightAnkleRoll(int ticks, int delay);
    void setLeftHipYaw(int ticks, int delay);
    void setLeftHipRoll(int ticks, int delay);
    void setLeftHipPitch(int ticks, int delay);
    void setLeftKnee(int ticks, int delay);
    void setLeftAnklePitch(int ticks, int delay);
    void setLeftAnkleRoll(int ticks, int delay);
    void setRightShoulderPitch(int ticks, int delay);
    void setRightShoulderRoll(int ticks, int delay);
    void setRightShoulderYaw(int ticks, int delay);
    void setRightElbow(int ticks, int delay);
    void setLeftShoulderPitch(int ticks, int delay);
    void setLeftShoulderRoll(int ticks, int delay);
    void setLeftShoulderYaw(int ticks, int delay);
    void setLeftElbow(int ticks, int delay);
    void setRightWristYaw(int ticks, int delay);
    void setRightWristPitch(int ticks, int delay);
    void setLeftWristYaw(int ticks, int delay);
    void setLeftWristPitch(int ticks, int delay);
    void setNeck(int yaw, int one, int two, int delay);
    void setWaist(int ticks, int delay);
    void setRightHand(int f0, int f1, int f2, int f3, int f4, int delay);
    void setLeftHand(int f0, int f1, int f2, int f3, int f4, int delay); 
    void enable(int board, int delay);
    void disable(int board, int delay);
    void requestEncoderPosition(int board, int delay);
    void getCurrentTicks(int board, int motor, int delay);
    void setCurrentTicks(int board, int motor, int ticks);
    void debugControl(int board, int operation);
    void runGesture(string path, int board);

private:

    //SUBSCRIBE
    InputPort<hubomsg::CanMessage>* canUpPort;
    InputPort<hubomsg::HuboCmd>* orOutPort;

    //PUBLISH
    OutputPort<hubomsg::CanMessage>* canDownPort;
    OutputPort<hubomsg::HuboCmd>* orInPort;

    HuboState* state;

    queue<hubomsg::CanMessage>* inputQueue;
    queue<hubomsg::CanMessage>* outputQueue;

    map< string, vector<float> > gestures;
   
    int written;
    bool needRequest;
};

#endif
