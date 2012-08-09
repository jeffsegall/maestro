#include "IMUBoard.h"

/******************************************************************************
* IMUBoard (Default Constructor)
*
* Initializes a IMU board with BNO = 0
******************************************************************************/
IMUBoard::IMUBoard(){
    IMUBoard((boardNum)0);
}

/******************************************************************************
* IMUBoard (Constructor)
*
* Initializes a IMU board with the given BNO.
*
* @param	BNO		The BNO of this IMU board
******************************************************************************/
IMUBoard::IMUBoard(boardNum BNO){
    this->BNO = BNO;
    this->canDownPort = new OutputPort<hubomsg::CanMessage>("can_down");
    this->canUpPort = new InputPort<hubomsg::CanMessage>("can_up");
}

/******************************************************************************
* setRequestBoardInfo
*
* Sets CAN rate and requests board information.
*
* @param	CANR		CAN rate
******************************************************************************/
void IMUBoard::setRequestBoardInfo(char CANR){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SETREQ_BOARD_INFO,
                             CANR, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out)); 
}

/******************************************************************************
* requestExecuteNULL
*
* When NULL is commanded, the IMU gathers data from the rate gyros for one
* second and then averages it for zero-levels.  Must be conducted before
* sending a message which requests angle and rate.  IMU will not respond to the
* request before NULL is finished.
******************************************************************************/
void IMUBoard::requestExecuteNULL(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_NULL,
                             0, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out)); 
}

/******************************************************************************
* requestExecuteCalib
*
* Requests calibration to determine the scale and the bias of the
* accelerometer.  See protocol for more information.
******************************************************************************/
void IMUBoard::requestExecuteCalib(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_CALIBRATE,
                             0, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestParameters
*
* Requests the parameters defined by PRF from the IMU board.
*
* @param	PRF		The parameter flag to return
******************************************************************************/
void IMUBoard::requestParameters(char PRF){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_BOARD_PARAM,
                             PRF, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setNewBoardNumber
*
* Sets this board's BNO to NEW_BNO.
*
* @param	NEW_BNO		The new board number
******************************************************************************/
void IMUBoard::setNewBoardNumber(char NEW_BNO){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_BNO_FREQ,
                             NEW_BNO, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}


