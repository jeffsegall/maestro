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

    //JOINT MOVEMENT API
    void setRightHipYaw(double ticks);
    void setRightHipRoll(double ticks);
    void setRightHipPitch(double ticks);
    void setRightKnee(double ticks);
    void setRightAnklePitch(double ticks);
    void setRightAnkleRoll(double ticks);
    void setLeftHipYaw(double ticks);
    void setLeftHipRoll(double ticks);
    void setLeftHipPitch(double ticks);
    void setLeftKnee(double ticks);
    void setLeftAnklePitch(double ticks);
    void setLeftAnkleRoll(double ticks);
    void setRightShoulderPitch(double ticks);
    void setRightShoulderRoll(double ticks);
    void setRightShoulder(double ticks);
    void setRightElbow(double ticks);
    void setLeftShoulderPitch(double ticks);
    void setLeftShoulderRoll(double ticks);
    void setLeftShoulderYaw(double ticks);
    void setLeftElbow(double ticks);
    void setRightWristYaw(double ticks);
    void setRightWristPitch(double ticks);
    void setLeftWristYaw(double ticks);
    void setLeftWristPitch(double ticks);
    void setNeck(double yaw, double one, double two);
    void setWaist(double ticks);
    void setRightHand(double f0, double f1, double f2, double f3, double f4);
    void setLeftHand(double f0, double f1, double f2, double f3, double f4); 
 
private:

    //SUBSCRIBE
    InputPort<hubomsg::CanMessage>* canUpPort;
    InputPort<hubomsg::HuboCmd>* orOutPort;

    //PUBLISH
    OutputPort<hubomsg::CanMessage>* canDownPort;
    OutputPort<hubomsg::HuboCmd>* orInPort;

};

#endif
