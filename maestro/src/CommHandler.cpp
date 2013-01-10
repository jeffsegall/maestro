/*
 * CommHandler.cpp
 *
 * Objectified access to the subscribe ports of the main control thread which other units may use to access messages.
 *
 *  Created on: Jan 10, 2013
 *      Author: maestro
 */

#include <iostream>
#include "CommHandler.h"

using namespace std;

	CommHandler::CommHandler (InputPort<hubomsg::CanMessage>* port1, InputPort<hubomsg::HuboCmd>* port2){
		canPort = port1;
		orPort = port2;

		newCanData = false;
	}

	void CommHandler::update(){
		hubomsg::HuboCmd huboCmd = hubomsg::HuboCmd();
		hubomsg::CanMessage canMessage = hubomsg::CanMessage();

		if (NewData == this->canPort->read(canMessage)){
			//Received update from CanGateway
			this->newCanData = true;
			this->currMessage = canMessage;

		}
		if (NewData == this->orPort->read(huboCmd)){
			this->currCmd = huboCmd;
		}
	}

	hubomsg::CanMessage CommHandler::getMessage(){
		newCanData = false;
		return currMessage;
	}

	hubomsg::HuboCmd CommHandler::getCmd(){
		return currCmd;
	}

	bool CommHandler::isNew(){
		return newCanData;
	}

