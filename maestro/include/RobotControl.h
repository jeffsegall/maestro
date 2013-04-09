#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#define CONFIG_PATH "/opt/ros/fuerte/stacks/maestro/test/config.txt"

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/scripting/ProgramInterface.hpp>
#include <rtt/scripting/ScriptingService.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>
#include <hubomsg/typekit/HuboState.h>
#include <hubomsg/typekit/HuboJointState.h>
#include <hubomsg/typekit/HuboCommand.h>
#include <hubomsg/typekit/HuboJointCommand.h>
#include <hubomsg/typekit/AchCommand.h>
#include <hubomsg/typekit/HuboIMU.h>
#include <hubomsg/typekit/HuboFT.h>
#include <vector>
#include <queue>
#include <map>
#include "huboCan.h"
#include "CommHandler.h"
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
    void buildHuboCommandMessage(vector<hubomsg::HuboJointCommand>& states, hubomsg::HuboCommand& message);
    void initRobot(string path);

    //JOINT MOVEMENT API
    void setRightHipYaw(int ticks, int delay);
    void setRightHipYawRad(double rads, int delay);
    void setRightHipRoll(int ticks, int delay);
    void setRightHipRollRad(double rads, int delay);
    void setRightHipPitch(int ticks, int delay);
    void setRightHipPitchRad(double rads, int delay);
    void setRightKnee(int ticks, int delay);
    void setRightKneeRad(double rads, int delay);
    void setRightAnklePitch(int ticks, int delay);
    void setRightAnklePitchRad(double rads, int delay);
    void setRightAnkleRoll(int ticks, int delay);
    void setRightAnkleRollRad(double rads, int delay);
    void setLeftHipYaw(int ticks, int delay);
    void setLeftHipYawRad(double rads, int delay);
    void setLeftHipRoll(int ticks, int delay);
    void setLeftHipRollRad(double rads, int delay);
    void setLeftHipPitch(int ticks, int delay);
    void setLeftHipPitchRad(double rads, int delay);
    void setLeftKnee(int ticks, int delay);
    void setLeftKneeRad(double rads, int delay);
    void setLeftAnklePitch(int ticks, int delay);
    void setLeftAnklePitchRad(double rads, int delay);
    void setLeftAnkleRoll(int ticks, int delay);
    void setLeftAnkleRollRad(double rads, int delay);
    void setRightShoulderPitch(int ticks, int delay);
    void setRightShoulderPitchRad(double rads, int delay);
    void setRightShoulderRoll(int ticks, int delay);
    void setRightShoulderRollRad(double rads, int delay);
    void setRightShoulderYaw(int ticks, int delay);
    void setRightShoulderYawRad(double rads, int delay);
    void setRightElbow(int ticks, int delay);
    void setRightElbowRad(double rads, int delay);
    void setLeftShoulderPitch(int ticks, int delay);
    void setLeftShoulderPitchRad(double rads, int delay);
    void setLeftShoulderRoll(int ticks, int delay);
    void setLeftShoulderRollRad(double rads, int delay);
    void setLeftShoulderYaw(int ticks, int delay);
    void setLeftShoulderYawRad(double rads, int delay);
    void setLeftElbow(int ticks, int delay);
    void setLeftElbowRad(double rads, int delay);
    void setRightWristYaw(int ticks, int delay);
    void setRightWristYawRad(double rads, int delay);
    void setRightWristPitch(int ticks, int delay);
    void setRightWristPitchRad(double rads, int delay);
    void setLeftWristYaw(int ticks, int delay);
    void setLeftWristYawRad(double rads, int delay);
    void setLeftWristPitch(int ticks, int delay);
    void setLeftWristPitchRad(double rads, int delay);
    void setNeck(int yaw, int one, int two, int delay);
    void setWaist(int ticks, int delay);
    void setWaistRad(double rads, int delay);
    void setRightHand(int f0, int f1, int f2, int f3, int f4, int delay);
    void setLeftHand(int f0, int f1, int f2, int f3, int f4, int delay); 
    void setJoint(string name, int ticks, int delay);
    void setJointRad(string name, double rads, int delay);
    void homeJoint(string name, int delay);
    void homeAll(string name, int delay);

    // Control Commands
    void enable(int board, int delay);
    void disable(int board, int delay);
    void enableJoint(string name, int delay);
    void disableJoint(string name, int delay);
    void enableAll(int delay);
    void disableAll(int delay);
    void debugControl(int board, int operation);
	void setDelay(int us);
	void runGesture(string name, int board);

    // Feedback Commands
    void requestEncoderPosition(int board, int delay);
    void getCurrentTicks(int board, int motor, int delay);
    void setCurrentTicks(int board, int motor, int ticks);
    void getCurrentGoal(int board, int motor, int delay);
    bool requiresMotion(int board, int motor, int delay);
    bool requiresMotionByName(string name, int delay);

    // Parameter Commands
    void setMaxAccVel(int board, int delay, int acc, int vel);
    void setPositionGain(int board, int motor, int kp, int ki, int kd);


    vector<string> getGestureScripts(string path);
    string getDefaultInitPath(string path);

private:


    //SUBSCRIBE
    InputPort<hubomsg::CanMessage>* canUpPort;
    InputPort<hubomsg::HuboCmd>* orOutPort;
    CommHandler* commHandler;

    //PUBLISH
    OutputPort<hubomsg::CanMessage>* canDownPort;
    OutputPort<hubomsg::HuboCmd>* orInPort;
    OutputPort<hubomsg::HuboCommand>* huboDownPort;
    OutputPort<hubomsg::AchCommand>* achDownPort;

    HuboState* state;

    queue<hubomsg::CanMessage>* inputQueue;
    queue<hubomsg::HuboCommand>* huboOutputQueue;
    queue<hubomsg::AchCommand>* achOutputQueue;

    map< string, vector<float> > gestures;
    map<boardNum, MotorBoard*>::iterator it;
    ofstream tempOutput;

   
    int written;
    bool printNow, enableControl;
    int delay;
};

#endif
