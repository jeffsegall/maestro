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

	CommHandler::CommHandler (InputPort<hubomsg::CanMessage>* port1,
			InputPort<hubomsg::HuboCmd>* port2,
			InputPort<hubomsg::HuboState>* port3){
		canPort = port1;
		orPort = port2;
		achPort = port3;

		newCanData = false;
		newAchData = false;
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
			this->newORData = true;
			this->currCmd = huboCmd;
		}
		if (NewData == this->achPort->read(achState)){
			this->newAchData = true;
			this->currState = achState;
		}
	}

	hubomsg::CanMessage CommHandler::getMessage(){
		newCanData = false;
		return currMessage;
	}

	hubomsg::HuboCmd CommHandler::getCmd(){
		newORData = false;
		return currCmd;
	}

	hubomsg::HuboState CommHandler::getState(){
		newAchData = false;
		return currState;
	}

	bool CommHandler::isNew(int port){
		switch (port){
		case 1:
			return newCanData;
		case 2:
			return newORData;
		case 3:
			return newAchData;
		}
		return false;
	}

