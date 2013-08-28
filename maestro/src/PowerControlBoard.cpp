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
#include "PowerControlBoard.h"

/******************************************************************************
* PowerControlBoard (Default Constructor)
*
* Initializes a power control board with BNO = 0
******************************************************************************/
PowerControlBoard::PowerControlBoard(){
    PowerControlBoard((boardNum)0);

    powerLookup.open(LOOKUP_TABLE_PATH);
    powerUsed = 0;

}

/******************************************************************************
* PowerControlBoard (Default Constructor)
*
* Initializes a power control board with the given BNO.
*
* @param	BNO		The BNO of this power control board
******************************************************************************/
PowerControlBoard::PowerControlBoard(boardNum BNO){
    this->BNO = BNO;
    this->huboDownPort = new OutputPort<hubomsg::HuboState>("Hubo/HuboState");
    this->canUpPort = new InputPort<hubomsg::CanMessage>("can_up");

    powerLookup.open(LOOKUP_TABLE_PATH);
	powerUsed = 0;
}

/******************************************************************************
* setRequestBoardInfo
*
* Sets CAN rate and requests board information.
*
* @param	CANR		CAN rate
******************************************************************************/
void PowerControlBoard::setRequestBoardInfo(char CANR){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SETREQ_BOARD_INFO,
                             CANR, 0, 0, 0, 0, 0, 0, 0);
    //this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setSwitchFunction
*
* Sets the switch function.
*
* @param	SFUNC		Switch function.  See protocol for details.
******************************************************************************/
void PowerControlBoard::setSwitchFunction(char SFUNC){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_SWITCH,
                             SFUNC, 0, 0, 0, 0, 0, 0, 0);
    //this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestAlarm
*
* Requests an alarm from the board.
*
* @param	ALRM		Alarm sound.  0 for alarm off, 1-4 for sound.
******************************************************************************/
void PowerControlBoard::requestAlarm(char ALRM){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_ALARM,
                             ALRM, 0, 0, 0, 0, 0, 0, 0);
    //this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestBeep
*
* Requests a beep from the board.
*
* @param	BDUR		Beep duration.  Beeps for (BDUR * 0.1) seconds.
******************************************************************************/
void PowerControlBoard::requestBeep(char BDUR){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_BEEP,
                             BDUR, 0, 0, 0, 0, 0, 0, 0);
    //this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestVoltageAndCurrent
*
* Requests the voltage and current information from the board.
******************************************************************************/
void PowerControlBoard::requestVoltageAndCurrent(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_VOLT_CUR,
                             0, 0, 0, 0, 0, 0, 0, 0);
    //this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestTimeAndStatus
*
* Requests the time and status information from the board.
******************************************************************************/
void PowerControlBoard::requestTimeAndStatus(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_TIME_STATUS,
                             0, 0, 0, 0, 0, 0, 0, 0);
    //this->canDownPort->write(buildCanMessage(out));
}

void PowerControlBoard::setInitialPower(double initialPower){
	powerUsed = initialPower;
}

double PowerControlBoard::getTotalPowerUsed(){
	return powerUsed;
}

bool PowerControlBoard::addMotionPower(string joint, double from, double to){

	string line;
	char dataJoint[10];
	double dataFrom = 0;
	double dataTo = 0;
	double delta = 0;
	int scanned = 0;
	do {
		getline(powerLookup, line, '\n');

		scanned = sscanf(line.c_str(), "%s %lf %lf %lf",
			&dataJoint, &dataFrom, &dataTo, &delta);
		std::cout << "Scanned: " << dataJoint << std::endl;
		if (strcmp(dataJoint, joint.c_str()) == 0 && from == dataFrom && to == dataTo){
			std::cout << "Power found!" << std::endl;
			powerUsed += delta;
			inputFile.close();
			inputFile.open(LOOKUP_TABLE_PATH);
			return true;
		}
	} while (scanned == 4);
	inputFile.close();
	inputFile.open(LOOKUP_TABLE_PATH);
	return false;
}
