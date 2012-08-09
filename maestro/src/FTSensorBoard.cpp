#include "FTSensorBoard.h"

/******************************************************************************
* FTSensorBoard (Default Constructor)
*
* Initializes a FT sensor board with BNO = 0
******************************************************************************/
FTSensorBoard::FTSensorBoard(){
    FTSensorBoard((boardNum)0);
}

/******************************************************************************
* FTSensorBoard (Constructor)
*
* Initializes a FT sensor board with the given BNO.
*
* @param	BNO		The BNO of this IMU board
******************************************************************************/
FTSensorBoard::FTSensorBoard(boardNum BNO){
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
void FTSensorBoard::setRequestBoardInfo(char CANR){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SETREQ_BOARD_INFO,
                             CANR, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestBoardParameters
*
* Requests parameters designated by PARM
*
* @param	PARM		Parameter flag.  See protocol for details.
******************************************************************************/
void FTSensorBoard::requestBoardParameters(char PARM){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_BOARD_PARAM,
                             PARM, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestExecuteNULL
*
* Execute NULL on the FT sensor board.
*
* @param	EFS		0 to NULL the F/T sensors, 1 to NULL the inc.
******************************************************************************/
void FTSensorBoard::requestExecuteNULL(char EFS){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_NULL,
                             EFS, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setFTMatrixConstant0
*
* Sets the F/T matrix constants.
*
* @param	SFT00		Matrix (0,0) coefficient
* @param	SFT01		Matrix (0,1) coefficient
* @param	SFT02		Matrix (0,2) coefficient
******************************************************************************/
void FTSensorBoard::setFTMatrixConstant0(int SFT00, int SFT01, int SFT02){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_FT_0,
                             SFT00, SFT01, SFT02, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setFTMatrixConstant1
*
* Sets the F/T matrix constants.
*
* @param	SFT10		Matrix (1,0) coefficient
* @param	SFT11		Matrix (1,1) coefficient
* @param	SFT12		Matrix (1,2) coefficient
******************************************************************************/
void FTSensorBoard::setFTMatrixConstant1(int SFT10, int SFT11, int SFT12){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_FT_1,
                             SFT10, SFT11, SFT12, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setFTMatrixConstant2
*
* Sets the F/T matrix constants.
*
* @param	SFT20		Matrix (2,0) coefficient
* @param	SFT21		Matrix (2,1) coefficient
* @param	SFT22		Matrix (2,2) coefficient
******************************************************************************/
void FTSensorBoard::setFTMatrixConstant2(int SFT20, int SFT21, int SFT22){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_FT_2,
                             SFT20, SFT21, SFT22, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setInclinoScaleFactor
*
* Sets the Inclinometer (Accelerometer) scale factor.  See protocol for details
*
* @param	SIF0		Inclinometer X factor
* @param	SIF1		Inclinometer Y factor
* @param	SIF2		Inclinometer Z factor
******************************************************************************/
void FTSensorBoard::setInclinoScaleFactor(int SIF0, int SIF1, int SIF2){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_INCLINO_SCALE,
                             SIF0, SIF1, SIF2, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setNewBoardNumber
*
* Sets this board's BNO to NEW_BNO and filter frequency to FREQ10.
*
* @param	NEW_BNO		The new board number
* @param	FREQ10		The new filter frequency * 10
******************************************************************************/
void FTSensorBoard::setNewBoardNumberFilterFreq(char NEW_BNO, int FREQ10){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_BNO_FREQ,
                             NEW_BNO, FREQ10, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* initBoard
* 
* Initialize the board with default parameters.
******************************************************************************/
void FTSensorBoard::initBoard(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_INIT_BOARD,
                             0xAA, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}


