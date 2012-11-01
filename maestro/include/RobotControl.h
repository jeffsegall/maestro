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

    void foo(int ticks);
    void initRobot();

    //JOINT MOVEMENT API
    void setRightHipYaw(int ticks);
    void setRightHipRoll(int ticks);
    void setRightHipPitch(int ticks);
    void setRightKnee(int ticks);
    void setRightAnklePitch(int ticks);
    void setRightAnkleRoll(int ticks);
    void setLeftHipYaw(int ticks);
    void setLeftHipRoll(int ticks);
    void setLeftHipPitch(int ticks);
    void setLeftKnee(int ticks);
    void setLeftAnklePitch(int ticks);
    void setLeftAnkleRoll(int ticks);
    void setRightShoulderPitch(int ticks);
    void setRightShoulderRoll(int ticks);
    void setRightShoulderYaw(int ticks);
    void setRightElbow(int ticks);
    void setLeftShoulderPitch(int ticks);
    void setLeftShoulderRoll(int ticks);
    void setLeftShoulderYaw(int ticks);
    void setLeftElbow(int ticks);
    void setRightWristYaw(int ticks);
    void setRightWristPitch(int ticks);
    void setLeftWristYaw(int ticks);
    void setLeftWristPitch(int ticks);
    void setNeck(int yaw, int one, int two);
    void setWaist(int ticks);
    void setRightHand(int f0, int f1, int f2, int f3, int f4);
    void setLeftHand(int f0, int f1, int f2, int f3, int f4); 
    void enable(int board);
    void disable(int board);
    void runGesture(int board);

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
   
    int written;
};

#endif
