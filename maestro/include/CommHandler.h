/*
 * CommHandler.h
 *
 *  Created on: Jan 10, 2013
 *      Author: Eric Rock
 */

#ifndef COMMHANDLER_H_
#define COMMHANDLER_H_

#include <rtt/Port.hpp>
#include <hubomsg/typekit/HuboCmd.h>
#include <hubomsg/typekit/CanMessage.h>
#include <hubomsg/typekit/HuboState.h>
#include "HuboState.h"

class CommHandler {

public:
	CommHandler (InputPort<hubomsg::CanMessage>* port1,
			InputPort<hubomsg::HuboCmd>* port2,
			InputPort<hubomsg::HuboState>* port3);
	~CommHandler();

	void update();
	hubomsg::CanMessage getMessage();
	hubomsg::HuboCmd getCmd();
	hubomsg::HuboState getState();
	bool isNew(int port);
private:
	hubomsg::HuboCmd currCmd;
	hubomsg::CanMessage currMessage;
	hubomsg::HuboState currState;

	//Subscribe
	InputPort<hubomsg::CanMessage>* canPort;
	InputPort<hubomsg::HuboCmd>* orPort;
	InputPort<hubomsg::HuboState>* achPort;

	bool newCanData;
	bool newORData;
	bool newAchData;
};


#endif /* COMMHANDLER_H_ */
