#include "MotorBoard.h"

/******************************************************************************
* @file	MotorBoard.cpp
* @author Jeff Segall <js572@drexel.edu>
*
* @section DESCRIPTION
* Represents a single motor board in the HUBO+
* Each motor board controls a number of motors (between 1 and 5).  This class
* provides the functionality described in the HUBO+ protocol for motor board
* messages.
******************************************************************************/


/******************************************************************************
* MotorBoard (Default Constructor)
*
* Creates a new motor board with default BNO and number of channels. 
******************************************************************************/
MotorBoard::MotorBoard(){
    MotorBoard(0, DEFAULT_CHANNELS);
}

/******************************************************************************
* MotorBoard (Constructor)
*
* Creates a new motor board with a given BNO and number of channels.  See
* huboCan.h for an enumeration of board numbers.
*
* @param	BNO		The Board Number of this motor board
* @param	channels	The number of motors (channels) this motor board
* 				controls.
******************************************************************************/
MotorBoard::MotorBoard(boardNum BNO, int channels){
    this->BNO = BNO;
    this->motors = new HuboMotor*[channels];
    this->canDownPort = new OutputPort<hubomsg::CanMessage>("can_down");
    this->orInPort = new OutputPort<hubomsg::HuboCmd>("or_in");
    this->canUpPort = new InputPort<hubomsg::CanMessage>("can_up");
    this->orOutPort = new InputPort<hubomsg::HuboCmd>("or_out");
}

/******************************************************************************
* addMotor
*
* Sets a channel of this motor board to a given motor.
*
* @param	motor		The motor for which to delegate control
* @param	channel		The board channel that controls the motor
******************************************************************************/
void MotorBoard::addMotor(HuboMotor* motor, int channel){
    this->motors[channel] = motor; 
}

/******************************************************************************
* removeMotor
*
* Releases control of a motor from this motor board.
* 
* @param	motor		The motor for which to relinquish control
******************************************************************************/
void MotorBoard::removeMotor(HuboMotor* motor){
    for (int i = 0; i < this->channels; i++){
        if (this->motors[channel] == motor){
            this->motors[channel] = NULL;
        }
    }  
}

/******************************************************************************
* removeMotor
*
* Releases control of a motor in a given channel from this motor board.
*
* @param	channel		The channel that houses the motor for which to 
* 				relinquish control
******************************************************************************/
void MotorBoard::removeMotor(int channel){
    this->motors[channel] = NULL;
}

/******************************************************************************
* setRequestBoardInfo
* 
* Requests information from this board.
* 
* @param	CANR		The CAN rate 
******************************************************************************/
void MotorBoard::setRequestBoardInfo(char CANR){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, 
                             CMD_SETREQ_BOARD_INFO, CANR,
                             0, 0, 0, 0, 0, 0, 0); 
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestBoardStatus
*
* Request the board to send the Status and Error flags.  The board
* automatically sends Status and Error flags without request if any change of
* the status is detected.
******************************************************************************/
void MotorBoard::requestBoardStatus(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_BOARD_STATUS);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestEncoderPosition
*
* Request the board to send the encoder position.
*
* @param	FES		Finger Position Send Flag.  Ignored if this
*				board doesn't control fingers.  If 0, sends
*				M0_POS, M1_POS, M2_POS.  If 1, sends
*				M3_POS, M4_POS.
******************************************************************************/
void MotorBoard::requestEncoderPosition(char FES){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_ENC_POS,
                             FES, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestCurrent
*
* Request the board to send the current.
******************************************************************************/
void MotorBoard::requestCurrent(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_CURR);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* resetEncoderToZero
*
* Sets the board encoder to zero, initializes internal parameters, and resets
* the Fault and Error flags.
*
* @param	CH		The channel to reset
******************************************************************************/
void MotorBoard::resetEncoderToZero(char CH){
    assert(CH < this->channels);

    //@TODO: This probably does other things.
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_RESET_ENC_ZERO,
                             CH, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setPositionGain
*
* Sets the PID gains for the given channel.  For 3CH and 5CH boards, sets gains
* for all channels.
*
* @param	CH		The channel to set
* @param	Kp		
* @param	Ki		
* @param	Kd		
******************************************************************************/
void MotorBoard::setPositionGain(char CH, int Kp, int Ki, int Kd){
    assert(CH < this->channels);

    //Set the values locally

    this->motors[CH]->setPositionGain(Kp, Ki, Kd);

    //Send the command to hardware

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_POS_GAIN_0 + CH,
                             Kp, Ki, Kd, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setCurrentGain
*
* Sets the PI and filter gains for the given channel.  For 3CH and 5CH boards,
* sets gains for all channels.
*
* @param	CH		The channel to set
* @param	Kpt		
* @param	Kdt		
* @param	Kf
******************************************************************************/
void MotorBoard::setCurrentGain(char CH, int KPt, int KDt, int Kf){
    assert(CH < this->channels);

    //Set the values locally

    this->motors[CH]->setPositionGain(KPt, KDt, Kf);

    //Send the command to hardware

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_CUR_GAIN_0 + CH,
                             KPt, KDt, Kf, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setHIP
*
* Enables and disables the motor driver for all channels on this board.
* Enabling the driver also disables position feedback.
*
* @param	HIP_EN		1 to enable driver, 0 to disable driver
******************************************************************************/
void MotorBoard::setHIP(char HIP_EN){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_HIP_ENABLE,
                             HIP_EN, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* openLoop 
*
* Disables motor opsition feedback.  If PUL_ON is set to 1, pulses out to run
* motor in specified PWM duty and direction.  If PUL_ON is set to 0, enforces
* zero duty to stop motor.  This method used for motor boards with 2 channels.
*
* @param	PUL_ON		see method description
* @param	DIR0		channel 0 direction
* @param	DUTY0		channel 0 duty
* @param	DIR1		channel 1 direction
* @param	DUTY1		channel 1 duty
******************************************************************************/
void openLoop(char PUL_ON, char DIR0, char DUTY0, char DIR1, char DUTY1){
    assert(this->channels == 2);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_OPEN_LOOP,
                             PUL_ON, DIR0, DUTY0, DIR1, DUTY1, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* openLoop 
*
* Disables motor opsition feedback.  If PUL_ON is set to 1, pulses out to run
* motor in specified PWM duty and direction.  If PUL_ON is set to 0, enforces
* zero duty to stop motor.  This method used for motor boards with 5 channels.
*
* @param	PUL_ON		see method description
* @param	D_DT0		channel 0 duty and direction
* @param	D_DT1		channel 1 duty and direction
* @param	D_DT2		channel 2 duty and direction
* @param	D_DT3		channel 3 duty and direction
* @param	D_DT4		channel 4 duty and direction
******************************************************************************/
void openLoop(char PUL_ON, char D_DT0, char D_DT1, char D_DT2, char D_DT3, char D_DT4){
    assert(this->channels == 5);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_OPEN_LOOP,
                             PUL_ON, D_DT0, D_DT1, D_DT2, D_DT3, D_DT4, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* openLoop 
*
* Disables motor opsition feedback.  If PUL_ON is set to 1, pulses out to run
* motor in specified PWM duty and direction.  If PUL_ON is set to 0, enforces
* zero duty to stop motor.  This method used for motor boards with 3 channels.
*
* @param	PUL_ON		see method description
* @param	D_DT0		channel 0 duty and direction
* @param	D_DT1		channel 1 duty and direction
* @param	D_DT2		channel 2 duty and direction
******************************************************************************/
void openLoop(char PUL_ON, char D_DT0, char D_DT1, char D_DT2){
    assert(this->channels == 3);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_OPEN_LOOP,
                             PUL_ON, D_DT0, D_DT1, D_DT2, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* enableController
*
* Activates the Feedback Controller.  To switch between position and current
* feedback modes, use setControlMode.  This also enables the motor driver.
******************************************************************************/
void MotorBoard::enableController(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_CONTROLLER_ON);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* disableController
*
* De-activates the feedback controller.  Also diables the motor drivers.
******************************************************************************/
void MotorBoard::disableController(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_CONTROLLER_OFF);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setControlMode
*
* Select one of two feedback controllers, position and current.
*
* @param	FBC		Feedback control: 1 for current control, 0 for
*				position control
******************************************************************************/
void MotorBoard::setControlMode(char FBC){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_CTRL_MODE,
                             FBC, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* goToHomeOffset
*
* Performs the following actions:
*    1) Find the limit switch and Index signal to get absolute position
*    2) Go to the Offset position from the Index
*    3) Set Encoder position value to Zero
*    4) Activate position feedback controller
*    5) If either Limit switch or Index signal is not found, set the fault bit
*       and deactivates the position controller
*    6) LED displays the search and go status as HME
*
* @param	CHD		Channel and direction.  See the protocol.
* @param	SDR		The search direction
* @param	H_OFFSET	The home offset
******************************************************************************/
void MotorBoard::goToHomeOffset(char CHD, char SDR, int H_OFFSET){

    //@TODO: Also set position to home offset?

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_GO_HOME_OFFSET,
                             CHD, SDR, H_OFFSET, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setDeadZone
*
* Sets the value of Dead zone to remove FET's PWM null.
*
* @param	CH		The channel to set
* @param	DZone		The dead zone value
******************************************************************************/
void MotorBoard::setDeadZone(char CH, char DZone){
    assert(CH < this->channels);

    this->motors[CH]->setDeadZone(DZone);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_DEAD_ZONE,
                             DZone, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* requestBoardParams
*
* Returns value requested by PARM.
*
* @param	PARM		The requested value.  See the protocol. 
******************************************************************************/
void MotorBoard::requestBoardParams(char PARM){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_REQ_BOARD_PARAM,
                             PARM, 0, 0, 0, 0, 0, 0 ,0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setHomeSearchParams
*
* Sets the value for the Home search parameter.
*
* @param	CH		The channel to set
* @param	SRL		The search limit.  SRL is the maximum number
*				of turns to find the limit switch.
* @param	SDR		The search direction
* @param	OFFSET		Offset from Index position
******************************************************************************/
void MotorBoard::setHomeSearchParams(char CH, char SRL, char SDR, int OFFSET){
    assert(CH < this->channels);

    this->motors[CH]->setHomeset1(OFFSET, SRL, SDR);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_HOME_SEARCH,
                             SRL, SDR, OFFSET, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setEncoderResolution
*
* Sets the encoder resolution.
*
* @param	CH		The channel to set
* @param	ENC_RE		A combination of ENC_RES, the encoder
*				resolution, AuS, the auto scale flag
                                (1 = auto scale), and MDR, the motor direction
                                (1 = clockwise, 0 = counterclockwise)
******************************************************************************/
void MotorBoard::setEncoderResolution(char CH, int ENC_RE){
    assert(CH < this->channels);

    //ENC_RE is formatted as:
    //b15 = md
    //b14 = as
    //b13-b0 = ers
    //Remove the 15th and 14th bit to get ERS
    this->motors[CH]->setEncoderResolution(ENC_RE & 0x00FFFFFFFFFFFFFF,
                                           canMsg.bitStrip(ENC_RE, 14),
                                           canMsg.bitStrip(ENC_RE, 15));

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_ENC_RESOLUTION + CH,
                             ENC_RE, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setEncoderResolution
*
* Sets the encoder resolution.
*
* @param	CH		The channel to set
* @param	ERS		The encoder resolution
* @param	AS		AutoScale flag
* @param	MD		Motor direction
******************************************************************************/
void MotorBoard::setEncoderResolution(char CH, int ERS, char AS, char MD){
    assert(CH < this->channels);

    this->motors[CH]->setEncoderResolution(ERS, AS, MD);

    int ENC_RE = (MD << 15) + (AS << 14) + ERS;

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_ENC_RESOLUTION + CH,
                             ENC_RE, 0, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setMaxAccVel
*
* Sets the value for maximum velocity and acceleration.
*
* @param	CH		The channel to set
* @param	MACC		The maximum acceleration
* @param	MVEL		The maximum velocity
******************************************************************************/
void MotorBoard::setMaxAccVel(char CH, int MACC, int MVEL){
    assert(CH < this->channels);

    this->motors[CH]->setSpeedLimit(MVEL, MACC);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_MAX_ACC_VEL + CH,
                             MACC, MVEL, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setLowerPosLimit
*
* Sets the value for the lower position limit
*
* @param	CH		The channel to set
* @param	MPS		Update/Enable flags.  See protocol.
* @param	MPOS1		The position limit
******************************************************************************/
void MotorBoard::setLowerPosLimit(char CH, char MPS, int MPOS1){
    assert(CH < this->channels);

    this->motors[CH]->setLowerLimit(MPOS1);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_LOWER_POS_LIMIT + CH,
                             MPS, MPOS1, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setUpperPosLimit
*
* Sets the value for the upper position limit
* 
* @param	CH		The channel to set
* @param	MPS		Update/Enable flags.  See protocol.
* @param	MPOS2		The position limit
******************************************************************************/
void MotorBoard::setUpperPosLimit(char CH, char MPS, int MPOS2){
    assert(CH < this->channels);

    this->motors[CH]->setUpperLimit(MPOS2);

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_UPPER_POS_LIMIT + CH,
                             MPS, MPOS2, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setHomeVelAcc
*
* Set the values for home search velocity and acceleration.
*
* @param	CH		The channel to set
* @param	HMA		Home search acceleration.  Actual acceleration
*				will be HMA/100.
* @param	HMV1		Maximum velocity to reach limit switch
* @param	HMV2		Maximum velocity to reach offset position
* @param	SRM		Search mode (0 = Limit switch & Index,
*				1 = Limit switch only, 2 = No limit switch)
* @param	LIMD		PWM duty for mechanical limit detection
******************************************************************************/
void MotorBoard::setHomeVelAcc(char CH, char HMA, char HMV1, char HMV2, char SRM, char LIMD){
    assert(CH < this->channels);

    this->motors[CH]->setHomeset2(HMV1, HMV2, HMA, SRM); 
 
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_HOME_VEL_ACC + CH,
                             HMA, HMV1, HMV2, SRM, LIMD, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setGainOverride
*
* Overall gain of the PID position controller changes to GOVWx in GDUR ms from
* the current value.
*
* @param	GOVW0		Percent override value of controller gain for
*				channel 0
* @param	GOVW1		Percent override value for controller gain for
*				channel 1
* @param	GDUR		Duration in milliseconds
******************************************************************************/
void MotorBoard::setGainOverride(char GOVW0, char GOVW1, int GDUR){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_GAIN_OVR,
                             GOVW0, GOVW1, GDUR, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setNewBoardNum
*
* Set the board number with NEW_BNO.
* 
* @param	NEW_BNO		New board number
* @param	CANR		CAN rate
******************************************************************************/
void MotorBoard::setNewBoardNum(char NEW_BNO, char CANR){
    //@TODO: What happens if duplicate?
    this->BNO = NEW_BNO;

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_NEW_BOARD_NUM,
                             NEW_BNO, CANR, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setJamPwmSatLim
*
* Set the JAM and PWM saturation limit.
*
* @param	JAM_LIM		JAM detection time in milliseconds
* @param	PWM_LIM		PWM saturation detection time in milliseconds
* @param	LIMD		PWM duty percentage for limit detection
* @param	JAMD		JAM limit in duty percentage
******************************************************************************/
void MotorBoard::setJamPwmSatLim(int JAM_LIM, int PWM_LIM, char LIMD, char JAMD){
    for (int i = 0; i < this->channels; i++){
       this->motors[i]->setJamPowerLimit(JAM_LIM, JAMD, PWM_LIM);
    } 

    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_JAM_PWM_SAT_LIMIT,
                             JAM_LIM, PWM_LIM, LIMD, JAMD, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* setErrorBound
*
* Sets the error bounds.
*
* @param	I_ERR		Maximum input difference error
* @param	B_ERR		Maximum error
* @param	T_ERR		Maximum temperature warning temperature
******************************************************************************/
void setErrorBound(int I_ERR, int B_ERR, int E_ERR){
    //@TODO: Some boards have channels with different error values but
    //       this doesn't take a channel parameter.
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_SET_ERR_BOUND,
                             I_ERR, B_ERR, E_ERR, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));  
}

/******************************************************************************
* initBoard
* 
* Initialize the board with default parameters.
******************************************************************************/
void initBoard(){
    canMsg* out = new canMsg(this->BNO, TX_MOTOR_CMD, CMD_INIT_BOARD);
    this->canDownPort->write(buildCanMessage(out));
}

//@TODO: Should we set the ticks position on this call or check the response?

/******************************************************************************
* sendPositionReference
* 
* Send a reference position to each channel on this motor board.  This method
* is for boards with 5 channels.
*
* @param	REF0		Reference position for channel 0
* @param	REF1		Reference position for channel 1
******************************************************************************/
void sendPositionReference(int REF0, int REF1){
    assert(this->channels <= 2);

    canMsg* out = new canMsg(this->BNO, TX_REF, (cmdType)2,
                             REF0, REF1, 0, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* sendPositionReference
* 
* Send a reference position to each channel on this motor board.  This method
* is for boards with 5 channels.
*
* @param	REF0		Reference position for channel 0
* @param	REF1		Reference position for channel 1
* @param	REF2		Reference position for channel 2
******************************************************************************/
void sendPositionReference(char REF0, char REF1, char REF2){
    assert(this->channels == 3);

    canMsg* out = new canMsg(this->BNO, TX_REF, (cmdType)2,
                             REF0, REF1, REF2, 0, 0, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}

/******************************************************************************
* sendPositionReference
* 
* Send a reference position to each channel on this motor board.  This method
* is for boards with 5 channels.
*
* @param	REF0		Reference position for channel 0
* @param	REF1		Reference position for channel 1
* @param	REF2		Reference position for channel 2
* @param	REF3		Reference position for channel 3
* @param	REF4		Reference position for channel 4
******************************************************************************/
void sendPositionReference(char REF0, char REF1, char REF2, char REF3, char REF4){
    assert(this->channels == 5);

    canMsg* out = new canMsg(this->BNO, TX_REF, (cmdType)2,
                             REF0, REF1, REF2, REF3, REF4, 0, 0, 0);
    this->canDownPort->write(buildCanMessage(out));
}


