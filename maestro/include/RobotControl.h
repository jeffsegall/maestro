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
#include "IMUBoard.h"
#include "FTSensorBoard.h"
#include "Property.h"
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
    void buildHuboCommandMessage(hubomsg::HuboJointCommand& state, hubomsg::HuboCommand& message);
    void initRobot(string path);

    //JOINT MOVEMENT API
    void set(string name, string property, double value);
    void setProperties(string names, string properties, string values);

    // Control Commands
    void debugControl(int board, int operation);
	void setDelay(int us);
	void runGesture(string name, int board);
	int testFunction(int param1, int param2, int param3, int param4, int param5, int param6);
	void command(string name, string target);

    // Feedback Commands
    void requestEncoderPosition(int board, int delay);
    void getCurrentTicks(int board, int motor, int delay);
    void setCurrentTicks(int board, int motor, int ticks);
    void getCurrentGoal(int board, int motor, int delay);
    bool requiresMotion(string name);
    double get(string name, string property);
    void updateState();

    // Parameter Commands
    void setMaxAccVel(int board, int delay, int acc, int vel);
    void setPositionGain(int board, int motor, int kp, int ki, int kd);
    void setMode(string mode, bool value);


    vector<string> getGestureScripts(string path);
    vector<string> splitFields(string input);
    string getDefaultInitPath(string path);

private:


    //SUBSCRIBE
    InputPort<hubomsg::CanMessage>* canUpPort;
    InputPort<hubomsg::HuboCmd>* orOutPort;
    InputPort<hubomsg::HuboState>* huboUpPort;
    CommHandler* commHandler;

    //PUBLISH
    OutputPort<hubomsg::CanMessage>* canDownPort;
    OutputPort<hubomsg::HuboCmd>* orInPort;
    OutputPort<hubomsg::HuboCommand>* huboDownPort;
    OutputPort<hubomsg::AchCommand>* achDownPort;

    HuboState* state;

    queue<hubomsg::CanMessage>* inputQueue;
    queue<hubomsg::HuboState>*	huboInputQueue;
    queue<hubomsg::HuboCommand>* huboOutputQueue;
    queue<hubomsg::AchCommand>* achOutputQueue;

    map< string, vector<float> > gestures;
    map<boardNum, MotorBoard*>::iterator it;
    ofstream tempOutput;

    int written;
    bool printNow, enableControl;
    int delay;
    bool interpolation, override;
};

#endif
