#include "PowerControlBoard.h"

/******************************************************************************
* PowerControlBoard (Default Constructor)
*
* Initializes a power control board with BNO = 0
******************************************************************************/
PowerControlBoard::PowerControlBoard(){
    PowerControlBoard((boardNum)0);
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


