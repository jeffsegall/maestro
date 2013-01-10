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
#include "HuboState.h"

class CommHandler {

public:
	CommHandler (InputPort<hubomsg::CanMessage>* port1, InputPort<hubomsg::HuboCmd>* port2);
	~CommHandler();

	void update();
	hubomsg::CanMessage getMessage();
	hubomsg::HuboCmd getCmd();
	bool isNew();
private:
	hubomsg::HuboCmd currCmd;
	hubomsg::CanMessage currMessage;

	//Subscribe
	InputPort<hubomsg::CanMessage> canPort;
	InputPort<hubomsg::HuboCmd>* orPort;

	bool newCanData;
};


#endif /* COMMHANDLER_H_ */
