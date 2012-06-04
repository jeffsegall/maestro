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
#include "huboCan.h"
#include "HuboState.h"

using namespace RTT;

class RobotControl : public RTT::TaskContext{

public:
    RobotControl(const std::string&);
    ~RobotControl();

    void updateHook(); 
    hubomsg::CanMessage buildCanMessage(canMsg msg);

    //JOINT MOVEMENT API
    void setRightHipYaw(long ticks);
    void setRightHipRoll(long ticks);
    void setRightHipPitch(long ticks);
    void setRightKnee(long ticks);
    void setRightAnklePitch(long ticks);
    void setRightAnkleRoll(long ticks);
    void setLeftHipYaw(long ticks);
    void setLeftHipRoll(long ticks);
    void setLeftHipPitch(long ticks);
    void setLeftKnee(long ticks);
    void setLeftAnklePitch(long ticks);
    void setLeftAnkleRoll(long ticks);
    void setRightShoulderPitch(long ticks);
    void setRightShoulderRoll(long ticks);
    void setRightShoulder(long ticks);
    void setRightElbow(long ticks);
    void setLeftShoulderPitch(long ticks);
    void setLeftShoulderRoll(long ticks);
    void setLeftShoulderYaw(long ticks);
    void setLeftElbow(long ticks);
    void setRightWristYaw(long ticks);
    void setRightWristPitch(long ticks);
    void setLeftWristYaw(long ticks);
    void setLeftWristPitch(long ticks);
    void setNeck(long yaw, long one, long two);
    void setWaist(long ticks);
    void setRightHand(long f0, long f1, long f2, long f3, long f4);
    void setLeftHand(long f0, long f1, long f2, long f3, long f4); 
 
private:

    //SUBSCRIBE
    InputPort<hubomsg::CanMessage>* canUpPort;
    InputPort<hubomsg::HuboCmd>* orOutPort;

    //PUBLISH
    OutputPort<hubomsg::CanMessage>* canDownPort;
    OutputPort<hubomsg::HuboCmd>* orInPort;

    HuboState state;
};

#endif
