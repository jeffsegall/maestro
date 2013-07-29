/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
			InputPort<hubomsg::HuboState>* port3
                        InputPort<hubomsg::PythonMessage>* port4){
	
        	canPort = port1;
		orPort = port2;
		achPort = port3;
                pyPort = port4;

		newCanData = false;
		newAchData = false;

		currCmd = hubomsg::HuboCmd();
		currMessage = hubomsg::CanMessage();
		currState = hubomsg::HuboState();
		currPyMessage = hubomsg::PythonMessage();
	}

	void CommHandler::update(){
		hubomsg::HuboCmd huboCmd = hubomsg::HuboCmd();
		hubomsg::CanMessage canMessage = hubomsg::CanMessage();
		hubomsg::HuboState achState = hubomsg::HuboState();
                hubomsg::PythonMessage pyMessage = hubomsg::PythonMessage();

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
                if (NewData == this->pyPort->read(pyMessage)){
			this->newPyData = true;
			this->currPyMessage = pyMessage;
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
	
	hubomsg::PythonMessage CommHandler::getPyMessage(){
		newPyData = false;
		return currPyMessage;
	}

	bool CommHandler::isNew(int port){
		switch (port){
		case 1:
			return newCanData;
		case 2:
			return newORData;
		case 3:
			return newAchData;
		case 4: 
			return newPyData;
		}
		return false;
	}

