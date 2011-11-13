
#include <math.h>
#include "RTX.h"
#include "CAN.h"
#include "mocap_functions.h"	// by Inhyeok


// for RTX
PSHARED_DATA	pSharedMemory;			// shared memory data
HANDLE			hSharedMemory;			// shared memory handler
HANDLE			hTimerCAN;				// timer handle for CAN sending
HANDLE			hTimer;					// timer handle for upper body control
HANDLE			hTimer1;				// timer handle for lower body control
unsigned long	timeIndex;				// timer index for TimerHandler
unsigned long	timeIndex1;				// timer index for TimerHandler1

// for CAN
MYCAN			CAN[2];					// CAN channel handler
MYMB			MB[MBSIZE];				// CAN message box

// for joints and sensors
JOINT			Joint[NO_OF_JOINT];		// Joint struct variable
FT				FTSensor[NO_OF_FT];		// FT sensor struct variable
IMU				IMUSensor[NO_OF_IMU];	// IMU sensor struct variable
unsigned char	ReadSensorFlag;			// read sensor data flag
unsigned char	NullSensorFlag;			// nulling sensor flag

float			ZMP[6];					// current ZMP value
float			FilteredZMP[6];			// filtered ZMP
float			InitZMP[2];				// initial ZMP




// for walking pattern
float			WalkingStep[4];				// walking step length
unsigned char	WalkingStartStop;			// walking start: 0x02, walking stop prepare: 0x01, walking stop: 0x00
WALKING_INFO	WalkingInfoSway;			// walking information for sway
WALKING_INFO	WalkingInfo[2][4];			// walking information for right(0), left(1) foot: 0->X, 1->Y, 2->Z
unsigned char	WalkingPhaseNextSway;		// next walking phase(next phase, not current phase) for sway
unsigned char	WalkingPhaseNext[4];		// next walking phase(next phase, not current phase) 0:x-direction, 1:y-direction, 2:z-direction, 3:yawing

float			dampingGain[6];			// damping controller gain

bool			dampingControlOnOff;
bool			dspControlOnOff;
bool			vibrationControlOnOff;



// for demo
unsigned char DemoFlag;


//Motion Capture JW
float			MotionCapture[NoOfMotion][MocapTimeLimit][NO_OF_JOINT_MC];
float			MotionCaptureVel[NoOfMotion][MocapTimeLimit][NO_OF_JOINT_MC];
float			MotionCaptureAcc[NoOfMotion][MocapTimeLimit][NO_OF_JOINT_MC];
int				MotionSetFlag;
int				TransitionFlag;
unsigned long	MotionTimeCurrent;
int				Motion;
int				tempMotion;
unsigned long	MotionPeriod[NoOfMotion];

unsigned int	jw_flag;
unsigned char	Control_Mode;
float	jw_test[10];

float TransitionCandidate[TfLimit][TfLimit][NO_OF_JOINT_MC];
float TransitionCandidateVel[TfLimit][TfLimit][NO_OF_JOINT_MC];
float TransitionCandidateAcc[TfLimit][TfLimit][NO_OF_JOINT_MC];
float TransitionCandidateVelMax[TfLimit][NO_OF_JOINT_MC];
float TransitionCandidateAccMax[TfLimit][NO_OF_JOINT_MC];
float error_M12[TfLimit][NO_OF_JOINT_MC];
float Q[TfLimit][NO_OF_JOINT_MC];
float Q_MIN[NO_OF_JOINT_MC];
unsigned int OptimalTfMax;
unsigned int OptimalTf[NO_OF_JOINT_MC];
unsigned int Ti;
float	test_Finger[MocapTimeLimit];
float	NKY_err_short_float[3];

//Handshake
float OLD_UpperMovement[NO_OF_JOINT_MC];
float OLD_UpperMovement1[NO_OF_JOINT_MC];

//Motion capture by Inhyeok
extern float _Qlb_19x1[20];	// [x,y,z,qPEL[4],rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar]
extern float _Qub_11x1[12];	// [wst, rsp,rsr,rsy,rep,rey, lsp,lsr,lsy,lep,ley]
extern float _RWP, _LWP;
extern unsigned int _MOTION_LENGTH;
extern unsigned int _mocap_count;
unsigned char	_WholeBodyMotionSetFlag;
int	_UpdatePassiveFlag;
int _recovery_count;
extern QUB_WIND _Qub_window;
 

void InitParameters(void);
bool LoadParameter(void);
bool SaveParameter(void);
bool SetJointParameter(unsigned char _jointID, JOINT _joint);
bool GetJointParameter(unsigned char _jointID, PJOINT _joint);
bool PrintJointParameter(unsigned char _jointID);
bool CheckDeviceCAN(unsigned int _boardID);
void FETDriverOnOff(unsigned int _boardID, unsigned char _enable);
bool SendRunStopCMD(unsigned char _boardID, unsigned char _runStop);
void GainSetting(unsigned int _jointID);
bool GoToLimitPos(unsigned int _jointID);
bool CheckCurrentState(unsigned int _jointID);
bool ZeroEncoder(unsigned char _boardID);
unsigned char GetMotorChannel(unsigned int _jointID);
unsigned int GetHDReduction(unsigned int _jointID);
unsigned int GetPulleyDrive(unsigned int _jointID);
unsigned int GetPulleyDriven(unsigned int _jointID);
unsigned char GetCANChannel(unsigned int _jointID);
unsigned char GetBoardID(unsigned char _jointID);
bool PositionLimitOnOff(unsigned char _jointID, unsigned char _mode);
bool Beep(unsigned char _mode);
bool GetStatus(unsigned char _boardID, unsigned char* _status);

// for JMC setting
bool SetMotorGain(JOINT _joint);
bool SetDeadZone(JOINT _joint);
bool SetEncoderResolution(JOINT _joint);
bool SetJamPwmSturation(JOINT _joint);
bool SetMaxAccVel(JOINT _joint);
bool SetControlMode(JOINT _joint);
bool SetHomeSearchParameter(JOINT _joint);
bool SetHomeMaxVelAcc(JOINT _joint);
bool SetUpperPositionLimit(JOINT _joint);
bool SetLowerPositionLimit(JOINT _joint);
bool SetErrorBound(JOINT _joint);

// Request parameters
bool RequestParameters(JOINT* _joint);


bool SetMoveJointAngle(unsigned char _jointID, float _angle, float _msTime, unsigned char _mode);
void MoveJointAngle(unsigned char _canChannel);


bool CheckBoardStatus(JOINT* _joint);


//Motion Capture JW
void UpperBodyMotionCapture(unsigned char _canChannel);
char FTN_half_1_cos(float mag, long time, long start, long during, int delay1, int delay2, float *result);
void Optimization(int _MotionNo);
void KinematicValueUpdate(int _MotionNo);
void SetMotionCapture0(void);	//Speach1
void SetMotionCapture1(void);	//Speach2
void SetMotionCapture2(void);	//Sign Language
void SetMotionCapture3(void);	//Taichi
void SetMotionCapture4(void);	//Welcome
void SetMotionCapture5(void);	//Goodbye
void SetMotionCapture6(void);	//Dance1
void SetMotionCapture7(void);	//Dance2
void SetMotionCapture8(void);	//Dance3
void SetMotionCapture9(void);	//Dance4
void SetMotionCapture10(void);	//Bow
void SetMotionCapture11(void);	//Right Up
void SetMotionCapture12(void);	//Right Forward
void SetMotionCapture13(void);	//Right Head
void SetMotionCapture14(void);	//Both Up
void SetMotionCapture15(void);	//Hand Wave
void SetMotionCapture16(void);	//Left Up
void SetMotionCapture17(void);	//Left Forward
void SetMotionCapture18(void);	//Left Head	
void SetMotionCapture19(void);	//HyunMin (MAYA)
void SetMotionCapture20(void);	//HM1(MAYA)
void SetMotionCapture21(void);	//HM2(MAYA)	
void SetMotionCapture22(void);	// Hand_Stone
void SetMotionCapture23(void);	// Hand_Paper
void SetMotionCapture24(void);	// Hand_Scissor
void SetMotionCapture25(void);	// Hand_14
void SetMotionCapture26(void);	// Hand_1
void SetMotionCapture27(void);	// Hand_RB
void SetMotionCapture28(void);	// HandNeck

void FingerControlModeChange(unsigned int _boardID, unsigned char _enable);

char FingerMove(int mag,int vel,int time, int start, int end, float *result);
void PROF_RFingerMotion1(long start_time, long stop_time, int _time, int _MotionNo); //RH Stone
void PROF_RFingerMotion2(long start_time, long stop_time, int _time, int _MotionNo); //RH Paper
void PROF_RFingerMotion3(long start_time, long stop_time, int _time, int _MotionNo); //RH Scissor
void PROF_RFingerMotion4(long start_time, long stop_time, int _time, int _MotionNo); //RH 1,4
void PROF_RFingerMotion5(long start_time, long stop_time, int _time, int _MotionNo); //RH 1
void PROF_RFingerMotion6(long start_time, long stop_time, int _time, int _MotionNo); //RH Rainbow
void PROF_RFingerMotion7(long start_time, long stop_time, int _time, int _MotionNo); //RH 0

void PROF_LFingerMotion1(long start_time, long stop_time, int _time, int _MotionNo); //LH Stone
void PROF_LFingerMotion2(long start_time, long stop_time, int _time, int _MotionNo); //LH Paper
void PROF_LFingerMotion3(long start_time, long stop_time, int _time, int _MotionNo); //LH Scissor
void PROF_LFingerMotion4(long start_time, long stop_time, int _time, int _MotionNo); //LH 1,4
void PROF_LFingerMotion5(long start_time, long stop_time, int _time, int _MotionNo); //LH 1
void PROF_LFingerMotion6(long start_time, long stop_time, int _time, int _MotionNo); //LH Rainbow
void PROF_LFingerMotion7(long start_time, long stop_time, int _time, int _MotionNo); //LH 0



// Whole body motion capture by Inhyeok
int WholeBodyMotionCapture(unsigned char canChannel);
void UpperBodyMotionCapture_mod(unsigned char _canChannel);
int WholeBodyMotionCapture_JW(unsigned char canChannel);



bool SetMoveTaskPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime, float _msDelayTime[2], float _userData[5], unsigned char _mode, unsigned char _function);
void MoveTaskPos(WALKING_INFO* _walkingInfoSway, WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _landingState, unsigned char _mode);
bool SetMoveTaskPosJointFF(JOINT *_joint, float _angle, float _msTime, float _msDelayTime[2], unsigned char _mode, unsigned char _function);
void MoveTaskPosJointFF(JOINT _joint[]);

void MoveJMC(unsigned char _boardID);
unsigned long SignConvention(long _input);
unsigned short SignConventionFinger(short _input,unsigned char _type);
void InverseKinematics(float _pos[6], float _angle[]);
void ForwardKinematics(float _angle[6], float _pos[]);
void ReadEncoder(unsigned char _canChannel);
void ReadFT(unsigned char _canChannel);
unsigned char GetZMP(FT _rightFootFTSensor, FT _leftFootFTSensor, WALKING_INFO _walkingInfo[2][4], float _zmp[], float _filteredZMP[]);
void ReadIMU(void);
bool RequestEncoder(unsigned char _boardID);
bool RequestCurrent(unsigned char _boardID);
bool RequestSensor(unsigned char _canChannel);
bool SetFTParameter(unsigned char _ftID, FT _ftSensor);
bool GetFTParameter(unsigned char _ftID, PFT _ftSensor);
bool NullFTSensor(unsigned char _ftID, unsigned char _mode);
bool NullFootAngleSensor(FT _ftSensor[]);
bool NullIMUSensor(unsigned char _imuID, unsigned char _mode);
bool PrintFTParameter(unsigned char _ftID);
bool SetIMUParameter(unsigned char _imuID, IMU _imuSensor);
bool GetIMUParameter(unsigned char _imuID, PIMU _imuSensor);
bool PrintIMUParameter(unsigned char _imuID);
void ZMPInitialization(float _refZMP[2], float _zmp[], IMU _imu, FT _rightFT, FT _leftFT,  WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _command);
void GotoWalkReadyPos(void);
void GotoHomePos(void);
unsigned char LandingStateCheck(FT _ftSensor[], WALKING_INFO _walkingInfo[][4]);

void DampingControl(IMU _imuSensor[], JOINT _joint[], unsigned char _command);
void DSPControl(float _ZMP[], float _refZMP[2], WALKING_INFO _walkingInfo[][4], unsigned char _command);
void VibrationControl(FT _ftSensor[], JOINT _joint[], unsigned char _command);


// under development
void TestFunction(void);


// walking sequence
void TestProfileSequence(void);

// motion and demo functions
bool GoToDemoOneLegSupport(void);
bool GoToDemoDownAndUp(unsigned char _rightLeft, unsigned char _times);
bool GoToDemoHomePosition(void);


//Handshake
float Torsion_mass_spring_damper_Mx(float Mx, int zero);
float Torsion_mass_spring_damper_My(float My, int zero);
float Wrist_mass_spring_damper_Fz(float Fz, int zero);
void ShakeHands();

/******************************************************************************/
void InitParameters(void)
{
	unsigned char i, j, k;

	// SharedMemory initialize
	pSharedMemory->CommandFlag = NO_ACT;
	pSharedMemory->MotorControlMode = CTRLMODE_NONE;

	// global variables initialize	
	ReadSensorFlag = 0x00;
	NullSensorFlag = 0x00;
	
	InitZMP[0] = 0.0f;
	InitZMP[1] = 0.0f;

	for(i=0 ; i<6 ; i++) ZMP[i] = FilteredZMP[i] = 0.0f;

	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++)
		{
			WalkingInfo[i][j].RefPatternCurrent		= 0.0f;
			WalkingInfo[i][j].RefPatternDelta		= 0.0f;
			WalkingInfo[i][j].RefPatternToGo		= 0.0f;
			WalkingInfo[i][j].RefPatternInitial		= 0.0f;
			WalkingInfo[i][j].GoalTimeCount			= 0;
			WalkingInfo[i][j].CurrentTimeCount		= 0;
			WalkingInfo[i][j].DelayTimeCount[0]		= 0;
			WalkingInfo[i][j].DelayTimeCount[1]		= 0;
			WalkingInfo[i][j].MoveFlag				= false;
			WalkingInfo[i][j].ControlDSPZMP			= 0.0f;
		}
	}

	for(i=X ; i<=Yaw ; i++) WalkingStep[j] = 0.0f;

	timeIndex =	0;
	timeIndex1 = 0;

	dampingGain[0] = 0.3f;		dampingGain[3] = 0.3f;
	dampingGain[1] = 1.0f;		dampingGain[4] = 1.0f;
	dampingGain[2] = 0.4f;		dampingGain[5] = 0.5f;
	
	//Motion Capture
	MotionSetFlag = 0;	TransitionFlag = 0;	Motion = 0;

/*	for(i=0;i<NoOfMotion;i++)
	{
		for(j=0;j<MocapTimeLimit;j++)
		{
			for(k=0;k<27;k++)
			{
				MotionCapture[i][j][k] = 0;
				MotionCaptureVel[i][j][k] = 0;
				MotionCaptureAcc[i][j][k] = 0;
			}
		}
		MotionPeriod[i] = 0;
	}
*/
	for(i=0;i<16;i++)
	{
		for(j=0;j<TfLimit;j++)
		{
			for(k=0;k<TfLimit;k++)
			{
				TransitionCandidate[k][j][i] = 0;
				TransitionCandidateVel[k][j][i] = 0;
				TransitionCandidateAcc[k][j][i]= 0;
			}
			TransitionCandidateVelMax[j][i] = 0;
			TransitionCandidateAccMax[j][i] = 0;
			error_M12[j][i]=0;
			Q[j][i]=0;
		}
		Q_MIN[i] = 1000000; 
		OptimalTf[i] = 0;
	}
	OptimalTfMax = 0;
	Ti = 0;


	dampingControlOnOff = true;
	dspControlOnOff = true;
	vibrationControlOnOff = true;

	DemoFlag = NO_DEMO;
}
/******************************************************************************/






/******************************************************************************/
bool LoadParameter(void)
{
	unsigned char i;
	unsigned char err_count = 0;
	
	for(i=RHY ; i<NO_OF_JOINT ; i++)
	{
		Joint[i].JMC				= GetBoardID(i);//Parameters.JMC[i];
		Joint[i].JointID			= i;
		Joint[i].CAN_channel		= GetCANChannel(i);//Parameters.CAN_channel[i];
		Joint[i].Ref_txdf			= REF_BASE_TXDF+Joint[i].JMC;//Parameters.Ref_txdf[i];
		Joint[i].Motor_channel		= GetMotorChannel(i);//Parameters.Motor_channel[i]-1;//Parameters.Motor_channel[i];
		Joint[i].HDReduction		= GetHDReduction(i);
		Joint[i].Pulley_drive		= GetPulleyDrive(i);
		Joint[i].Pulley_driven		= GetPulleyDriven(i);
	}

	for(i=RHY ; i<NO_OF_JOINT ; i++) if(RequestParameters(&Joint[i]) == false) err_count++;

	FTSensor[RFFT].Controller_NO	= FT0;
	FTSensor[RFFT].CAN_channel		= CAN0;
	FTSensor[RFFT].CutOffFeq		= 3.0f;
	FTSensor[RFFT].SF_Pitch			= 1.0f/0.0255f;
	FTSensor[RFFT].SF_Roll			= 1.0f/0.0255f;
		
	FTSensor[LFFT].Controller_NO	= FT1;
	FTSensor[LFFT].CAN_channel		= CAN0;
	FTSensor[LFFT].CutOffFeq		= 3.0f;
	FTSensor[LFFT].SF_Pitch			= 1.0f/0.0255f;
	FTSensor[LFFT].SF_Roll			= 1.0f/0.0255f;
		
	IMUSensor[CENTERIMU].Controller_NO	= IMU0;
	IMUSensor[CENTERIMU].CAN_channel	= CAN0;
	// #4
	//IMUSensor[CENTERIMU].RollOffset	= 1.1f;
	//IMUSensor[CENTERIMU].PitchOffset	= -0.7f;
	// #5
	IMUSensor[CENTERIMU].RollOffset		= 0.85f;
	IMUSensor[CENTERIMU].PitchOffset	= 0.0f;

	FTSensor[RWFT].Controller_NO	= FT2;
	FTSensor[RWFT].CAN_channel		= CAN1;
	FTSensor[RWFT].CutOffFeq		= 3.0f;

	FTSensor[LWFT].Controller_NO	= FT3;
	FTSensor[LWFT].CAN_channel		= CAN1;
	FTSensor[LWFT].CutOffFeq		= 3.0f;
		

	if(err_count == 0) RtWprintf(L"\n>>> Parameters are loaded successfully..!!");
	else RtWprintf(L"\n>>> Loading parameters is failed..!!");
	return true;
}
/******************************************************************************/




/******************************************************************************/
bool SaveParameter(void)
{
	unsigned char i;
	SAVE_PARAMETERS	Parameters;
	FILE *fp;
	
	fp = fopen("c:\\parameter.par", "wb");
	
	for(i=RHY ; i<NO_OF_JOINT ; i++)
	{
		Parameters.JMC[i]			= Joint[i].JMC;
		Parameters.CAN_channel[i]	= Joint[i].CAN_channel;
		Parameters.Ref_txdf[i]		= Joint[i].Ref_txdf;
		Parameters.Motor_channel[i]	= Joint[i].Motor_channel;
		Parameters.HDReduction[i]	= Joint[i].HDReduction;
		Parameters.Pulley_drive[i]	= Joint[i].Pulley_drive;
		Parameters.Pulley_driven[i]	= Joint[i].Pulley_driven;
		Parameters.Encoder_size[i]	= Joint[i].Encoder_size;
		Parameters.PPR[i]			= Joint[i].PPR;
	}

	for(i=RFFT ; i<NO_OF_FT ; i++)
	{
		Parameters.FTControllerNO[i]	= FTSensor[i].Controller_NO;
		Parameters.FTCANchannel[i]		= FTSensor[i].CAN_channel;
		Parameters.FTCutOff[i]			= FTSensor[i].CutOffFeq;
	}

	for(i=CENTERIMU ; i<NO_OF_IMU ; i++)
		{
			Parameters.IMUControllerNO[i]	= IMUSensor[i].Controller_NO;
			Parameters.IMUCANchannel[i]		= IMUSensor[i].CAN_channel;
		}

	if(fp == NULL) { RtWprintf(L"\n>>> File open error(SaveParameter)..!!"); return false; }
	else
	{
		fwrite(&Parameters, sizeof(SAVE_PARAMETERS), 1, fp);
		fclose(fp);
		RtWprintf(L"\n>>> File save success..!!");
		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
bool SetMotorGain(JOINT _joint)
{
	unsigned char tempData[8];	
	
	if(_joint.JointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong Joint ID(MotorGainSetting)..!!"); return false; }
	else
	{
		tempData[0] = _joint.JMC;

		switch (_joint.JointID)
		{
		case RHY:
		case RHP:
		case RKN:
		case RAP:
		case LHY:
		case LHP:
		case LKN:
		case LAP:
		case LAR:
		case RSP:
		case RSY:
		case LSP:
		case LSY:
		case RWY:
		case LWY:
		case NKY:
			if(_joint.MotorControlMode == 0x00) tempData[1] = SetPosGainA;
			else tempData[1] = SetTorqueGainA;
			break;
		case RHR:
		case RAR:
		case LHR:
		case RSR:
		case REB:
		case LSR:
		case LEB:
		case RWP:
		case LWP:
		case NK1:
		case NK2:
			if(_joint.MotorControlMode == 0x00) tempData[1] = SetPosGainB;
			else tempData[1] = SetTorqueGainB;
			break;
		}
	
		RtWprintf(L"\n>>> Joint(%d) motor gain is set..!!", _joint.JointID);
		if(_joint.MotorControlMode == 0x00)
		{
			tempData[2] = (unsigned char)(_joint.Position_Kp & 0xFF);
			tempData[3] = (unsigned char)((_joint.Position_Kp>>8) & 0xFF);	
			tempData[4] = (unsigned char)(_joint.Position_Ki & 0xFF);
			tempData[5] = (unsigned char)((_joint.Position_Ki>>8) & 0xFF);
			tempData[6] = (unsigned char)(_joint.Position_Kd & 0xFF);
			tempData[7] = (unsigned char)((_joint.Position_Kd>>8) & 0xFF);

			RtWprintf(L"\n>>> Kp:%d\t Ki:%d\t Kd:%d", _joint.Position_Kp, _joint.Position_Ki, _joint.Position_Kd);
		}
		else
		{
			tempData[2] = (unsigned char)(_joint.Torque_Kp & 0xFF);
			tempData[3] = (unsigned char)((_joint.Torque_Kp>>8) & 0xFF);	
			tempData[4] = (unsigned char)(_joint.Torque_Ki & 0xFF);
			tempData[5] = (unsigned char)((_joint.Torque_Ki>>8) & 0xFF);
			tempData[6] = (unsigned char)(_joint.Torque_Kd & 0xFF);
			tempData[7] = (unsigned char)((_joint.Torque_Kd>>8) & 0xFF);

			RtWprintf(L"\n>>> Kp:%d\t Ki:%d\t Kd:%d", _joint.Position_Kp, _joint.Position_Ki, _joint.Position_Kd);
		}

		CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);
		Sleep(20);

		
		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
bool SetJointParameter(unsigned char _jointID, JOINT _joint)
{
	if(_jointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong Joint ID(SetJointParameter)..!!"); return false; }
	else { Joint[_jointID] = _joint; return true; }
}
/******************************************************************************/




/******************************************************************************/
bool GetJointParameter(unsigned char _jointID, JOINT* _joint)
{
	if(_jointID > NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(GetJointParameter)..!!"); return false; }
	else { *_joint = Joint[_jointID];	return true; }
}
/******************************************************************************/




/******************************************************************************/
bool PrintJointParameter(unsigned char _jointID)
{
	if(_jointID > NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(PrintJointParameter)..!!"); return false; }
	else
	{
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Joint(%d) parameter information", _jointID);
		RtWprintf(L"\n>>> RefAngleCurrent: %d", (int)(Joint[_jointID].RefAngleCurrent*1000.0f));
		RtWprintf(L"\n>>> RefAngleToGo: %d", (int)(Joint[_jointID].RefAngleToGo*1000.0f));
		RtWprintf(L"\n>>> RefAngleInitial: %d", (int)(Joint[_jointID].RefAngleInitial*1000.0f));
		RtWprintf(L"\n>>> GoalTimeCount: %d", Joint[_jointID].GoalTimeCount);
		RtWprintf(L"\n>>> CurrentTimeCount: %d", Joint[_jointID].CurrentTimeCount);
		RtWprintf(L"\n>>> EncoderValue: %d", Joint[_jointID].EncoderValue);
		RtWprintf(L"\n>>> MoveFlag: %d", Joint[_jointID].MoveFlag);
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> PPR: %d", (int)(Joint[_jointID].PPR*1000.0f));
		RtWprintf(L"\n>>> JMC: %d", Joint[_jointID].JMC);
		RtWprintf(L"\n>>> Motor_channel: %d", Joint[_jointID].Motor_channel);
		RtWprintf(L"\n>>> CAN_channel: %d", Joint[_jointID].CAN_channel);
		RtWprintf(L"\n>>> Ref_txdf: %d", Joint[_jointID].Ref_txdf);
		RtWprintf(L"\n>>> Positive_dir: %d", Joint[_jointID].Positive_dir);
		RtWprintf(L"\n>>> Offset_angle: %d", (int)(Joint[_jointID].Offset_angle*1000.0f));
		RtWprintf(L"\n>>> Limit_rev: %d", Joint[_jointID].Limit_rev);
		RtWprintf(L"\n>>> Position_Kp: %d", Joint[_jointID].Position_Kp);
		RtWprintf(L"\n>>> Position_Kd: %d", Joint[_jointID].Position_Kd);
		RtWprintf(L"\n>>> Position_Ki: %d", Joint[_jointID].Position_Ki);
		RtWprintf(L"\n>>> Torque_Kp: %d", Joint[_jointID].Torque_Kp);
		RtWprintf(L"\n>>> Torque_Kd: %d", Joint[_jointID].Torque_Kd);
		RtWprintf(L"\n>>> Torque_Ki: %d", Joint[_jointID].Torque_Ki);
		RtWprintf(L"\n>>> Encoder_size: %d", Joint[_jointID].Encoder_size);
		RtWprintf(L"\n>>> HDReduction: %d", Joint[_jointID].HDReduction);
		RtWprintf(L"\n>>> Pulley_drive: %d", Joint[_jointID].Pulley_drive);
		RtWprintf(L"\n>>> Pulley_driven: %d", Joint[_jointID].Pulley_driven);
		
		GetJointParameter(_jointID, &pSharedMemory->Joint);

		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
bool CheckDeviceCAN(unsigned int _boardID)
{
	bool result = false;
	unsigned char tempData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	tempData[0] = _boardID;
	tempData[1] = NameInfo;	
	tempData[2] = (unsigned char)(INT_TIME & 0x00FF);

	switch(_boardID)
	{
		case JMC0:
			CanSendMsg(Joint[RHY].CAN_channel, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(Joint[RHY].CAN_channel, NAME0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC0..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC0..!! "); result = false; }
			break;
		case JMC1:
			CanSendMsg(0, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC1..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC1..!! "); result = false; }
			break;
		case JMC2:
			CanSendMsg(0, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC2..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC2..!! "); result = false; }
			break;
		case JMC3:
			CanSendMsg(0, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME3_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC3..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC3..!! "); result = false; }
			break;
		case JMC4:
			CanSendMsg(0, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME4_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC4..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC4..!! "); result = false; }
			break;
		case JMC5:
			CanSendMsg(0, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME5_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC5..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC5..!! "); result = false; }
			break;
		case JMC6:
			CanSendMsg(0, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME6_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC6..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC6..!! "); result = false; }
			break;
		case JMC7:
			CanSendMsg(0, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME7_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC7..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC7..!! "); result = false; }
			break;
		case JMC8:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME8_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC8..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC8..!! "); result = false; }
			break;
		case JMC9:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME9_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC9..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC9..!! "); result = false; }
			break;
		case JMC10:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME10_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC10..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with JMC10..!! "); result = false; }
			break;
		case JMC11:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME11_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC11..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC11..!! ");	result = false;	}
			break;
		case EJMC0:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC0..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC0..!! ");	result = false;	}
			break;
		case EJMC1:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC1..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC1..!! ");	result = false;	}
			break;
		case EJMC2:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 7, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC2..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC2..!! "); result = false; }
			break;
		case EJMC3:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(0, CMD_TXDF, tempData, 7, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_E3_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC3..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC3..!! ");	result = false;	}
			break;
		case EJMC4:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E4_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC4..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC4..!! ");	result = false;	}
			break;
		case EJMC5:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E5_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC5..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC5..!! ");	result = false;	}
			break;
		case FT0:
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_FT0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT0..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with FT0..!! "); result = false; }
			break;
		case FT1:
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_FT1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT1..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with FT1..!! "); result = false; }
			break;
		case IMU0:
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_IMU0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with IMU0..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with IMU0..!! "); result = false; }
			break;
		case IMU1:
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_IMU1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with IMU1..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with IMU1..!! "); result = false; }
			break;
		case IMU2:
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_IMU2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with IMU2..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with IMU2..!! "); result = false; }
			break;
		case FT2:
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_FT2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT2..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with FT2..!! "); result = false; }
			break;
		case FT3:
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_FT3_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT3..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with FT3..!! "); result = false; }
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(CheckDeviceCAN)..!!");
			result = false;
			break;
	}	

	return result;
}
/******************************************************************************/



/******************************************************************************/
void FETDriverOnOff(unsigned int _boardID, unsigned char _enable)
{
	unsigned char tempData[8];
	bool _enableFlag;

	if(_enable == 0x01) _enableFlag = true;
	else _enableFlag = false;

	tempData[0] = _boardID;
	tempData[1] = HipEnable;
	tempData[2] = _enable;		// enable : 1, disable : 0

	switch(_boardID)
	{
		case JMC0:
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			if(_enableFlag) RtWprintf(L"\n>>> JMC0 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC0 FET driver is disabled..!!");
			break;
		case JMC1:
			if(_enableFlag) RtWprintf(L"\n>>> JMC1 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC1 FET driver is disabled..!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC2:
			if(_enableFlag) RtWprintf(L"\n>>> JMC2 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC2 FET driver is disabled..!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC3:
			if(_enableFlag) RtWprintf(L"\n>>> JMC3 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC3 FET driver is disabled..!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC4:
			if(_enableFlag) RtWprintf(L"\n>>> JMC4 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC4 FET driver is disabled..!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC5:
			if(_enableFlag) RtWprintf(L"\n>>> JMC5 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC5 FET driver is disabled..!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC6:
			if(_enableFlag) RtWprintf(L"\n>>> JMC6 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC6 FET driver is disabled..!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC7:
			if(_enableFlag) RtWprintf(L"\n>>> JMC7 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC7 FET driver is disabled..!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC8:
			if(_enableFlag) RtWprintf(L"\n>>> JMC8 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC8 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC9:
			if(_enableFlag) RtWprintf(L"\n>>> JMC9 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC9 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC10:
			if(_enableFlag) RtWprintf(L"\n>>> JMC10 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC10 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC11:
			if(_enableFlag) RtWprintf(L"\n>>> JMC11 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC11 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC0:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC0 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC0 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC1:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC1 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC1 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC2:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC2 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC2 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC3:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC3 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> 3JMCE FET driver is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC4:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC4 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC4 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC5:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC5 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC5 FET driver is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(FETDriverOnOff)..!!");
			break;
	}
	RtSleep(15);
}
/******************************************************************************/



/******************************************************************************/
bool SendRunStopCMD(unsigned char _boardID, unsigned char _runStop)
{
	unsigned char tempData[8];
	bool _runStopFlag;

	tempData[0] = _boardID;
	if(_runStop == 0x01) { tempData[1] = RunCMD; _runStopFlag = true; }
	else { tempData[1] = StopCMD; _runStopFlag = false; }

	switch(_boardID)
	{
		case JMC0:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC0 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC0 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC1:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC1 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC1 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC2:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC2 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC2 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC3:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC3 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC3 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC4:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC4 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC4 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC5:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC5 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC5 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC6:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC6 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC6 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC7:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC7 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC7 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC8:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC8 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC8 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC9:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC9 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC9 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC10:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC10 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC10 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC11:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC11 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC11 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC0:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC0 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC0 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC1:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC1 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC1 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC2:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC2 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC2 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC3:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC3 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC3 motor control is disabled!!");
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC4:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC4 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC4 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC5:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC5 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC5 motor control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(SendRunStopCMD)..!!");
			return false;
			break;
	}
	RtSleep(15);

	return true;
}
/******************************************************************************/




/******************************************************************************/
void GainSetting(unsigned int _jointID)
{
	unsigned char tempData[8];

	tempData[0] = Joint[_jointID].JMC;	
	if(Joint[_jointID].Motor_channel == 0) tempData[1] = 0x07;
	else tempData[1] = 0x08;

	tempData[2] = (unsigned char)(Joint[_jointID].Position_Kp & 0xFF);
	tempData[3] = (unsigned char)((Joint[_jointID].Position_Kp>>8) & 0xFF);
	tempData[4] = (unsigned char)(Joint[_jointID].Position_Ki & 0xFF);
	tempData[5] = (unsigned char)((Joint[_jointID].Position_Ki>>8) & 0xFF);
	tempData[6] = (unsigned char)(Joint[_jointID].Position_Kd & 0xFF);
	tempData[7] = (unsigned char)((Joint[_jointID].Position_Kd>>8) & 0xFF);
	
	CanSendMsg(Joint[_jointID].CAN_channel, CMD_TXDF, tempData, 8, 0);
	RtSleep(10);
}
/******************************************************************************/





/******************************************************************************/
bool GoToLimitPos(unsigned int _jointID)
{
	unsigned char tempData[8];
	int itemp;

	// pulse input
	//itemp = Joint[_jointID].Positive_dir*(Joint[_jointID].Offset_pulse + Joint[_jointID].Offset_rev*Joint[_jointID].Encoder_size);
	// angle input
	//itemp = (int)(Joint[_jointID].Positive_dir*(Joint[_jointID].Offset_angle*Joint[_jointID].PPR));
	itemp = (int)(Joint[_jointID].Offset_angle*Joint[_jointID].PPR);
	
	if (itemp < 0) itemp = (((-itemp) & 0x0007FFFF) | (1<<19));

	tempData[0] = Joint[_jointID].JMC;
	tempData[1] = GoLimitPos;

	switch(_jointID)
	{
	case RHY:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RHY limit sensor searching..!!");
		break;
	case RHR:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RHR limit sensor searching..!!");
		break;
	case RHP:
		tempData[2] = 0x11;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RHP limit sensor searching..!!");
		break;
	case RKN:
		tempData[2] = 0x10;	
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RKN limit sensor searching..!!");
		break;
	case RAP:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RAP limit sensor searching..!!");
		break;
	case RAR:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RAR limit sensor searching..!!");
		break;
	case LHY:
		tempData[2] = 0x11;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LHY limit sensor searching..!!");
		break;
	case LHR:
		tempData[2] = 0x21;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LHR limit sensor searching..!!");
		break;
	case LHP:
		tempData[2] = 0x11;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LHP limit sensor searching..!!");
		break;
	case LKN:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LKN limit sensor searching..!!");
		break;
	case LAP:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LAP limit sensor searching..!!");
		break;
	case LAR:
		tempData[2] = 0x21;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LAR limit sensor searching..!!");
		break;
	case WST:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> WST limit sensor searching..!!");
		break;
	case RSP:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RSP limit sensor searching..!!");
		break;
	case RSR:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RSR limit sensor searching..!!");
		break;
	case RSY:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RSY limit sensor searching..!!");
		break;
	case REB:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> REB limit sensor searching..!!");
		break;
	case LSP:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LSP limit sensor searching..!!");
		break;
	case LSR:
		tempData[2] = 0x21;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LSR limit sensor searching..!!");
		break;
	case LSY:
		tempData[2] = 0x11;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LSY limit sensor searching..!!");
		break;
	case LEB:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LEB limit sensor searching..!!");
		break;
	case RWY:
		tempData[2] = 0x11;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RWY limit sensor searching..!!");
		break;
	case RWP:
		tempData[2] = 0x21;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RWP limit sensor searching..!!");
		break;
	case LWY:
		tempData[2] = 0x10;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LWY limit sensor searching..!!");
		break;
	case LWP:
		tempData[2] = 0x21;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LWP limit sensor searching..!!");
		break;
	case NKY:
		tempData[2] = 0xF3;
		tempData[3] = 0;//itemp & 0xFF;
		tempData[4] = 0;//(itemp >> 8) & 0xFF;
		tempData[5] = 0;//itemp2 & 0xFF;
		tempData[6] = 0;//itemp3 & 0xFF;
		tempData[7] = 0x00;
		RtWprintf(L"\n>>> NK limit sensor searching..!!");
		break;
	case RF1:
		tempData[2] = 0xF3;
		tempData[3] = 0;
		tempData[4] = 0;
		tempData[5] = 0;//((itemp>>8) & (0xFF));
		tempData[6] = 0;//(itemp & 0xFF);
		tempData[7] = 0;//(((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RF limit sensor searching..!!");
		break;
	case LF1:
		tempData[2] = 0xF3;
		tempData[3] = 0;
		tempData[4] = 0;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> LF limit sensor searching..!!");
		break;
	default:
		RtWprintf(L"\n>>> Wrong joint ID(GoToLimitPos)..!!");
		return false;
		break;
	}

	CanSendMsg(Joint[_jointID].CAN_channel, CMD_TXDF, tempData, 8, 0);

	return true;	
}
/******************************************************************************/






/******************************************************************************/
bool GoToLimitPosAll(unsigned char _upperLower)
{
	// _upperLower
	// 0x00 : All
	// 0x01 : Lower
	// 0x02 : Upper

	unsigned char tempData[8];

	if( (_upperLower==0x00) || (_upperLower==0x01) )
	{
		// RHY and RHR
		tempData[0] = Joint[RHY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RHY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RHP
		tempData[0] = Joint[RHP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RHP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RKN
		tempData[0] = Joint[RKN].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RKN].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RAP and RAR
		tempData[0] = Joint[RAP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RAP].CAN_channel, CMD_TXDF, tempData, 8, 0);
		
		// LHY and LHR
		tempData[0] = Joint[LHY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LHY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LHP
		tempData[0] = Joint[LHP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LHP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LKN
		tempData[0] = Joint[LKN].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LKN].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LAP and LAR
		tempData[0] = Joint[LAP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LAP].CAN_channel, CMD_TXDF, tempData, 8, 0);
	}

	if( (_upperLower==0x00) || (_upperLower==0x02) )
	{
		// WST
		tempData[0] = Joint[WST].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[WST].CAN_channel, CMD_TXDF, tempData, 8, 0);
		
		// RSP and RSR
		tempData[0] = Joint[RSP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RSP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RSY and REB
		tempData[0] = Joint[RSY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RSY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RWY and RWP
		tempData[0] = Joint[RWY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RWY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RF1, 2, 3, 4, and 5
		tempData[0] = Joint[RF1].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[RF1].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LSP and LSR
		tempData[0] = Joint[LSP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LSP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LSY and LEB
		tempData[0] = Joint[LSY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LSY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LWY and LWP
		tempData[0] = Joint[LWY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LWY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LF1, 2, 3, 4, and 5
		tempData[0] = Joint[LF1].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[LF1].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// NKY, NK1 and NK2
		tempData[0] = Joint[NKY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		CanSendMsg(Joint[NKY].CAN_channel, CMD_TXDF, tempData, 8, 0);
	}

	return true;	
}
/******************************************************************************/





/******************************************************************************/
bool CheckCurrentState(unsigned int _jointID)
{
	unsigned char _temp;

	switch(_jointID)
	{
	case RHY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RHY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHY limit sensor searching..failed!!"); return false; }
		break;
	case RHR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RHR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHR limit sensor searching..failed!!"); return false; }
		break;
	case RHP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RHP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHP limit sensor searching..failed!!"); return false; }
		break;
	case RKN:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RKN limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RKN limit sensor searching..failed!!"); return false; }
		break;
	case RAP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RAP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHP limit sensor searching..failed!!"); return false; }
		break;
	case RAR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RAR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RAR limit sensor searching..failed!!"); return false; }
		break;
	case LHY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LHY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LHY limit sensor searching..failed!!"); return false; }
		break;
	case LHR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LHR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LHR limit sensor searching..failed!!"); return false; }
		break;
	case LHP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LHP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LHP limit sensor searching..failed!!"); return false; }
		break;
	case LKN:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LKN limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LKN limit sensor searching..failed!!"); return false; }
		break;
	case LAP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LAP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LAP limit sensor searching..failed!!"); return false; }
		break;
	case LAR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LAR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LAR limit sensor searching..failed!!"); return false; }
		break;
	case WST:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> WST limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> WST limit sensor searching..failed!!"); return false; }
		break;
	case RSP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RSP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RSP limit sensor searching..failed!!"); return false; }
		break;
	case RSR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RSR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RSR limit sensor searching..failed!!"); return false; }
		break;
	case RSY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RSY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RSY limit sensor searching..failed!!"); return false; }
		break;
	case REB:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> REB limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> REB limit sensor searching..failed!!"); return false; }
		break;
	case LSP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LSP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LSP limit sensor searching..failed!!"); return false; }
		break;
	case LSR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LSR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LSR limit sensor searching..failed!!"); return false; }
		break;
	case LSY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LSY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LSY limit sensor searching..failed!!"); return false; }
		break;
	case LEB:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LEB limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LEB limit sensor searching..failed!!"); return false; }
		break;
	case RWY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RW limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RW limit sensor searching..failed!!"); return false; }
		break;
	case LWY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LW limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LW limit sensor searching..failed!!"); return false; }
		break;
	case NKY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> NK limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> NK limit sensor searching..failed!!"); return false; }
		break;
	case RF1:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RF limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RF limit sensor searching..failed!!"); return false; }
		break;
	case LF1:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LF limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LF limit sensor searching..failed!!"); return false; }
		break;
	}

	RtWprintf(L"\n>>> Wrong joint ID(GoToLimitPos)..!!");
	return false;	
}
/******************************************************************************/




/******************************************************************************/
bool ZeroEncoder(unsigned char _boardID)
{	
	unsigned char tempData[8];
	int i;

	tempData[0] = _boardID;
	tempData[1] = EncZero;
	tempData[2] = 0;		// zero all encoders

	switch(_boardID)
	{
		case JMC0:
			Joint[RHY].RefAngleCurrent =	0;
			Joint[RHY].RefAngleToGo =		0;
			Joint[RHY].RefAngleInitial =	0;
			Joint[RHY].RefAngleFF =			0;
			Joint[RHY].GoalTimeCount =		0;
			Joint[RHY].CurrentTimeCount =	0;
			Joint[RHY].EncoderValue =		0;
			Joint[RHY].MoveFlag =			false;

			Joint[RHR].RefAngleCurrent =	0;
			Joint[RHR].RefAngleToGo =		0;
			Joint[RHR].RefAngleInitial =	0;
			Joint[RHR].RefAngleFF =			0;
			Joint[RHR].GoalTimeCount =		0;
			Joint[RHR].CurrentTimeCount =	0;
			Joint[RHR].EncoderValue =		0;
			Joint[RHR].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC0(RHY, RHR) encoder zero..!!");
			break;
		case JMC1:
			Joint[RHP].RefAngleCurrent =	0;
			Joint[RHP].RefAngleToGo =		0;
			Joint[RHP].RefAngleInitial =	0;
			Joint[RHP].RefAngleFF =			0;
			Joint[RHP].GoalTimeCount =		0;
			Joint[RHP].CurrentTimeCount =	0;
			Joint[RHP].EncoderValue =		0;
			Joint[RHP].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC1(RHP) encoder zero..!!");
			break;
		case JMC2:
			Joint[RKN].RefAngleCurrent =	0;
			Joint[RKN].RefAngleToGo =		0;
			Joint[RKN].RefAngleInitial =	0;
			Joint[RKN].RefAngleFF =			0;
			Joint[RKN].GoalTimeCount =		0;
			Joint[RKN].CurrentTimeCount =	0;
			Joint[RKN].EncoderValue =		0;
			Joint[RKN].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC2(RKN) encoder zero..!!");
			break;
		case JMC3:
			Joint[RAP].RefAngleCurrent =	0;
			Joint[RAP].RefAngleToGo =		0;
			Joint[RAP].RefAngleInitial =	0;
			Joint[RAP].RefAngleFF =			0;
			Joint[RAP].GoalTimeCount =		0;
			Joint[RAP].CurrentTimeCount =	0;
			Joint[RAP].EncoderValue =		0;
			Joint[RAP].MoveFlag =			false;
			
			Joint[RAR].RefAngleCurrent =	0;
			Joint[RAR].RefAngleToGo =		0;
			Joint[RAR].RefAngleInitial =	0;
			Joint[RAR].RefAngleFF =			0;
			Joint[RAR].GoalTimeCount =		0;
			Joint[RAR].CurrentTimeCount =	0;
			Joint[RAR].EncoderValue =		0;
			Joint[RAR].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC3(RAP, RAR) encoder zero..!!");
			break;
		case JMC4:
			Joint[LHY].RefAngleCurrent =	0;
			Joint[LHY].RefAngleToGo =		0;
			Joint[LHY].RefAngleInitial =	0;
			Joint[LHY].RefAngleFF =			0;
			Joint[LHY].GoalTimeCount =		0;
			Joint[LHY].CurrentTimeCount =	0;
			Joint[LHY].EncoderValue =		0;
			Joint[LHY].MoveFlag =			false;
			
			Joint[LHR].RefAngleCurrent =	0;
			Joint[LHR].RefAngleToGo =		0;
			Joint[LHR].RefAngleInitial =	0;
			Joint[LHR].RefAngleFF =			0;
			Joint[LHR].GoalTimeCount =		0;
			Joint[LHR].CurrentTimeCount =	0;
			Joint[LHR].EncoderValue =		0;
			Joint[LHR].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC4(LHY, LHR) encoder zero..!!");
			break;
		case JMC5:
			Joint[LHP].RefAngleCurrent =	0;
			Joint[LHP].RefAngleToGo =		0;
			Joint[LHP].RefAngleInitial =	0;
			Joint[LHP].RefAngleFF =			0;
			Joint[LHP].GoalTimeCount =		0;
			Joint[LHP].CurrentTimeCount =	0;
			Joint[LHP].EncoderValue =		0;
			Joint[LHP].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC5(LHP) encoder zero..!!");
			break;
		case JMC6:
			Joint[LKN].RefAngleCurrent =	0;
			Joint[LKN].RefAngleToGo =		0;
			Joint[LKN].RefAngleInitial =	0;
			Joint[LKN].RefAngleFF =			0;
			Joint[LKN].GoalTimeCount =		0;
			Joint[LKN].CurrentTimeCount =	0;
			Joint[LKN].EncoderValue =		0;
			Joint[LKN].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC6(LKN) encoder zero..!!");
			break;
		case JMC7:
			Joint[LAP].RefAngleCurrent =	0;
			Joint[LAP].RefAngleToGo =		0;
			Joint[LAP].RefAngleInitial =	0;
			Joint[LAP].RefAngleFF =			0;
			Joint[LAP].GoalTimeCount =		0;
			Joint[LAP].CurrentTimeCount =	0;
			Joint[LAP].EncoderValue =		0;
			Joint[LAP].MoveFlag =			false;
			
			Joint[LAR].RefAngleCurrent =	0;
			Joint[LAR].RefAngleToGo =		0;
			Joint[LAR].RefAngleInitial =	0;
			Joint[LAR].RefAngleFF =			0;
			Joint[LAR].GoalTimeCount =		0;
			Joint[LAR].CurrentTimeCount =	0;
			Joint[LAR].EncoderValue =		0;
			Joint[LAR].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC7(LAP, LAR) encoder zero..!!");
			break;
		case JMC8:
			Joint[RSP].RefAngleCurrent =	0;
			Joint[RSP].RefAngleToGo =		0;
			Joint[RSP].RefAngleInitial =	0;
			Joint[RSP].RefAngleFF =			0;
			Joint[RSP].GoalTimeCount =		0;
			Joint[RSP].CurrentTimeCount =	0;
			Joint[RSP].EncoderValue =		0;
			Joint[RSP].MoveFlag =			false;
			
			Joint[RSR].RefAngleCurrent =	0;
			Joint[RSR].RefAngleToGo =		0;
			Joint[RSR].RefAngleInitial =	0;
			Joint[RSR].RefAngleFF =			0;
			Joint[RSR].GoalTimeCount =		0;
			Joint[RSR].CurrentTimeCount =	0;
			Joint[RSR].EncoderValue =		0;
			Joint[RSR].MoveFlag =			false;
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC8(RSP, RSR) encoder zero..!!");
			break;
		case JMC9:
			Joint[RSY].RefAngleCurrent =	0;
			Joint[RSY].RefAngleToGo =		0;
			Joint[RSY].RefAngleInitial =	0;
			Joint[RSY].RefAngleFF =			0;
			Joint[RSY].GoalTimeCount =		0;
			Joint[RSY].CurrentTimeCount =	0;
			Joint[RSY].EncoderValue =		0;
			Joint[RSY].MoveFlag =			false;
			
			Joint[REB].RefAngleCurrent =	0;
			Joint[REB].RefAngleToGo =		0;
			Joint[REB].RefAngleInitial =	0;
			Joint[REB].RefAngleFF =			0;
			Joint[REB].GoalTimeCount =		0;
			Joint[REB].CurrentTimeCount =	0;
			Joint[REB].EncoderValue =		0;
			Joint[REB].MoveFlag =			false;
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC9(RSY, REB) encoder zero..!!");
			break;
		case JMC10:
			Joint[LSP].RefAngleCurrent =	0;
			Joint[LSP].RefAngleToGo =		0;
			Joint[LSP].RefAngleInitial =	0;
			Joint[LSP].RefAngleFF =			0;
			Joint[LSP].GoalTimeCount =		0;
			Joint[LSP].CurrentTimeCount =	0;
			Joint[LSP].EncoderValue =		0;
			Joint[LSP].MoveFlag =			false;
			
			Joint[LSR].RefAngleCurrent =	0;
			Joint[LSR].RefAngleToGo =		0;
			Joint[LSR].RefAngleInitial =	0;
			Joint[LSR].RefAngleFF =			0;
			Joint[LSR].GoalTimeCount =		0;
			Joint[LSR].CurrentTimeCount =	0;
			Joint[LSR].EncoderValue =		0;
			Joint[LSR].MoveFlag =			false;
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC10(LSP, LSR) encoder zero..!!");
			break;
		case JMC11:
			Joint[LSY].RefAngleCurrent =	0;
			Joint[LSY].RefAngleToGo =		0;
			Joint[LSY].RefAngleInitial =	0;
			Joint[LSY].RefAngleFF =			0;
			Joint[LSY].GoalTimeCount =		0;
			Joint[LSY].CurrentTimeCount =	0;
			Joint[LSY].EncoderValue =		0;
			Joint[LSY].MoveFlag =			false;
			
			Joint[LEB].RefAngleCurrent =	0;
			Joint[LEB].RefAngleToGo =		0;
			Joint[LEB].RefAngleInitial =	0;
			Joint[LEB].RefAngleFF =			0;
			Joint[LEB].GoalTimeCount =		0;
			Joint[LEB].CurrentTimeCount =	0;
			Joint[LEB].EncoderValue =		0;
			Joint[LEB].MoveFlag =			false;
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC11(LSY, LEB) encoder zero..!!");
			break;
		case EJMC0:
			Joint[RWY].RefAngleCurrent =	0;
			Joint[RWY].RefAngleToGo =		0;
			Joint[RWY].RefAngleInitial =	0;
			Joint[RWY].RefAngleFF =			0;
			Joint[RWY].GoalTimeCount =		0;
			Joint[RWY].CurrentTimeCount =	0;
			Joint[RWY].EncoderValue =		0;
			Joint[RWY].MoveFlag =			false;
			
			Joint[RWP].RefAngleCurrent =	0;
			Joint[RWP].RefAngleToGo =		0;
			Joint[RWP].RefAngleInitial =	0;
			Joint[RWP].RefAngleFF =			0;
			Joint[RWP].GoalTimeCount =		0;
			Joint[RWP].CurrentTimeCount =	0;
			Joint[RWP].EncoderValue =		0;
			Joint[RWP].MoveFlag =			false;
			
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC0(RWY, RWP) encoder zero..!!");
			break;
		case EJMC1:
			Joint[LWY].RefAngleCurrent =	0;
			Joint[LWY].RefAngleToGo =		0;
			Joint[LWY].RefAngleInitial =	0;
			Joint[LWY].RefAngleFF =			0;
			Joint[LWY].GoalTimeCount =		0;
			Joint[LWY].CurrentTimeCount =	0;
			Joint[LWY].EncoderValue =		0;
			Joint[LWY].MoveFlag =			false;
			
			Joint[LWP].RefAngleCurrent =	0;
			Joint[LWP].RefAngleToGo =		0;
			Joint[LWP].RefAngleInitial =	0;
			Joint[LWP].RefAngleFF =			0;
			Joint[LWP].GoalTimeCount =		0;
			Joint[LWP].CurrentTimeCount =	0;
			Joint[LWP].EncoderValue =		0;
			Joint[LWP].MoveFlag =			false;
				
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC1(LWY, LWP) encoder zero..!!");
			break;
		case EJMC2:
			Joint[NKY].RefAngleCurrent =	0;
			Joint[NKY].RefAngleToGo =		0;
			Joint[NKY].RefAngleInitial =	0;
			Joint[NKY].RefAngleFF =			0;
			Joint[NKY].GoalTimeCount =		0;
			Joint[NKY].CurrentTimeCount =	0;
			Joint[NKY].EncoderValue =		0;
			Joint[NKY].MoveFlag =			false;
			
			Joint[NK1].RefAngleCurrent =	0;
			Joint[NK1].RefAngleToGo =		0;
			Joint[NK1].RefAngleInitial =	0;
			Joint[NK1].RefAngleFF =			0;
			Joint[NK1].GoalTimeCount =		0;
			Joint[NK1].CurrentTimeCount =	0;
			Joint[NK1].EncoderValue =		0;
			Joint[NK1].MoveFlag =			false;
				
			Joint[NK2].RefAngleCurrent =	0;
			Joint[NK2].RefAngleToGo =		0;
			Joint[NK2].RefAngleInitial =	0;
			Joint[NK2].RefAngleFF =			0;
			Joint[NK2].GoalTimeCount =		0;
			Joint[NK2].CurrentTimeCount =	0;
			Joint[NK2].EncoderValue =		0;
			Joint[NK2].MoveFlag =			false;
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC2(NKY, NK1, NK2) encoder zero..!!");
			break;
		case EJMC3:
			Joint[WST].RefAngleCurrent =	0;
			Joint[WST].RefAngleToGo =		0;
			Joint[WST].RefAngleInitial =	0;
			Joint[WST].RefAngleFF =			0;
			Joint[WST].GoalTimeCount =		0;
			Joint[WST].CurrentTimeCount =	0;
			Joint[WST].EncoderValue =		0;
			Joint[WST].MoveFlag =			false;
			CanSendMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC3(WST) encoder zero..!!");
			break;
		case EJMC4:
			for(i=0; i<5; i++)
			{
				Joint[RF1+i].RefAngleCurrent =	0;
				Joint[RF1+i].RefAngleToGo =		0;
				Joint[RF1+i].RefAngleInitial =	0;
				Joint[RF1+i].RefAngleFF =		0;
				Joint[RF1+i].GoalTimeCount =	0;
				Joint[RF1+i].CurrentTimeCount =	0;
				Joint[RF1+i].EncoderValue =		0;
				Joint[RF1+i].MoveFlag =			false;
			}
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC4(RF) encoder zero..!!");
			break;
		case EJMC5:
			for(i=0; i<5; i++)
			{
				Joint[LF1+i].RefAngleCurrent =	0;
				Joint[LF1+i].RefAngleToGo =		0;
				Joint[LF1+i].RefAngleInitial =	0;
				Joint[LF1+i].RefAngleFF =		0;
				Joint[LF1+i].GoalTimeCount =	0;
				Joint[LF1+i].CurrentTimeCount =	0;
				Joint[LF1+i].EncoderValue =		0;
				Joint[LF1+i].MoveFlag =			false;
			}
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC5(LF) encoder zero..!!");
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(ZeroEncoder)..!!");
			return false;
			break;
	}

	return true;
}
/******************************************************************************/









/******************************************************************************/
unsigned char GetMotorChannel(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 0;
			break;
		case RHR:
			return 1;
			break;
		case RHP:
			return 0;
			break;
		case RKN:
			return 0;
			break;
		case RAP:
			return 0;
			break;
		case RAR:
			return 1;
			break;
		case LHY:
			return 0;
			break;
		case LHR:
			return 1;
			break;
		case LHP:
			return 0;
			break;
		case LKN:
			return 0;
			break;
		case LAP:
			return 0;
			break;
		case LAR:
			return 1;
			break;
		case RSP:
			return 0;
			break;
		case RSR:
			return 1;
			break;
		case RSY:
			return 0;
			break;
		case REB:
			return 1;
			break;
		case RWY:
			return 0;
			break;
		case RWP:
			return 1;
			break;
		case LSP:
			return 0;
			break;
		case LSR:
			return 1;
			break;
		case LSY:
			return 0;
			break;
		case LEB:	
			return 1;
			break;
		case LWY:
			return 0;
			break;
		case LWP:
			return 1;
			break;
		case NKY:
			return 0;
			break;
		case NK1:
			return 1;
			break;
		case NK2:
			return 2;
			break;
		case WST:
			return 0;
			break;
		case RF1:
			return 0;
			break;
		case RF2:
			return 1;
			break;
		case RF3:
			return 2;
			break;
		case RF4:
			return 3;
			break;
		case RF5:
			return 4;
			break;
		case LF1:
			return 0;
			break;
		case LF2:
			return 1;
			break;
		case LF3:
			return 2;
			break;
		case LF4:
			return 3;
			break;
		case LF5:
			return 4;
			break;
	}

	return 100;
}
/******************************************************************************/








/******************************************************************************/
unsigned int GetHDReduction(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 100;
			break;
		case RHR:
			return 160;
			break;
		case RHP:
			return 160;
			break;
		case RKN:
			return 160;
			break;
		case RAP:
			return 100;
			break;
		case RAR:
			return 100;
			break;
		case LHY:
			return 100;
			break;
		case LHR:
			return 160;
			break;
		case LHP:
			return 160;
			break;
		case LKN:
			return 160;
			break;
		case LAP:
			return 100;
			break;
		case LAR:
			return 100;
			break;
		case RSP:
			return 100;
			break;
		case RSR:
			return 100;
			break;
		case RSY:
			return 100;
			break;
		case REB:
			return 100;
			break;
		case RWY:
			return 100;
			break;
		case RWP:
			return 100;
			break;
		case LSP:
			return 100;
			break;
		case LSR:
			return 100;
			break;
		case LSY:
			return 100;
			break;
		case LEB:	
			return 100;
			break;
		case LWY:
			return 100;
			break;
		case LWP:
			return 100;
			break;
		case NKY:
			return 100;
			break;
		case NK1:
			return 100;
			break;
		case NK2:
			return 100;
			break;
		case WST:
			return 100;
			break;
		case RF1:
			return 256;
			break;
		case RF2:
			return 256;
			break;
		case RF3:
			return 256;
			break;
		case RF4:
			return 256;
			break;
		case RF5:
			return 256;
			break;
		case LF1:
			return 256;
			break;
		case LF2:
			return 256;
			break;
		case LF3:
			return 256;
			break;
		case LF4:
			return 256;
			break;
		case LF5:
			return 256;
			break;
	}

	return 0;
}
/******************************************************************************/








/******************************************************************************/
unsigned int GetPulleyDrive(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 10;
			break;
		case RHR:
			return 324;
			break;
		case RHP:
			return 16;
			break;
		case RKN:
			return 16;
			break;
		case RAP:
			return 10;
			break;
		case RAR:
			return 324;
			break;
		case LHY:
			return 10;
			break;
		case LHR:
			return 324;
			break;
		case LHP:
			return 16;
			break;
		case LKN:
			return 16;
			break;
		case LAP:
			return 10;
			break;
		case LAR:
			return 324;
			break;
		case RSP:
			return 11;
			break;
		case RSR:
			return 1;
			break;
		case RSY:
			return 1;
			break;
		case REB:
			return 20;
			break;
		case RWY:
			return 1;
			break;
		case RWP:
			return 1;
			break;
		case LSP:
			return 11;
			break;
		case LSR:
			return 1;
			break;
		case LSY:
			return 1;
			break;
		case LEB:	
			return 20;
			break;
		case LWY:
			return 1;
			break;
		case LWP:
			return 1;
			break;
		case NKY:
			return 1;
			break;
		case NK1:
			return 1;
			break;
		case NK2:
			return 1;
			break;
		case WST:
			return 10;
			break;
		case RF1:
			return 1;
			break;
		case RF2:
			return 1;
			break;
		case RF3:
			return 1;
			break;
		case RF4:
			return 1;
			break;
		case RF5:
			return 1;
			break;
		case LF1:
			return 1;
			break;
		case LF2:
			return 1;
			break;
		case LF3:
			return 1;
			break;
		case LF4:
			return 1;
			break;
		case LF5:
			return 1;
			break;
	}

	return 0;
}
/******************************************************************************/







/******************************************************************************/
unsigned int GetPulleyDriven(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 25;
			break;
		case RHR:
			return 1024;
			break;
		case RHP:
			return 20;
			break;
		case RKN:
			return 16;
			break;
		case RAP:
			return 25;
			break;
		case RAR:
			return 1024;
			break;
		case LHY:
			return 25;
			break;
		case LHR:
			return 1024;
			break;
		case LHP:
			return 20;
			break;
		case LKN:
			return 16;
			break;
		case LAP:
			return 25;
			break;
		case LAR:
			return 1024;
			break;
		case RSP:
			return 16;
			break;
		case RSR:
			return 1;
			break;
		case RSY:
			return 1;
			break;
		case REB:
			return 24;
			break;
		case RWY:
			return 1;
			break;
		case RWP:
			return 1;
			break;
		case LSP:
			return 16;
			break;
		case LSR:
			return 1;
			break;
		case LSY:
			return 1;
			break;
		case LEB:	
			return 24;
			break;
		case LWY:
			return 1;
			break;
		case LWP:
			return 1;
			break;
		case NKY:
			return 1;
			break;
		case NK1:
			return 1;
			break;
		case NK2:
			return 1;
			break;
		case WST:
			return 25;
			break;
		case RF1:
			return 1;
			break;
		case RF2:
			return 1;
			break;
		case RF3:
			return 1;
			break;
		case RF4:
			return 1;
			break;
		case RF5:
			return 1;
			break;
		case LF1:
			return 1;
			break;
		case LF2:
			return 1;
			break;
		case LF3:
			return 1;
			break;
		case LF4:
			return 1;
			break;
		case LF5:
			return 1;
			break;
	}

	return 0;
}
/******************************************************************************/






/******************************************************************************/
unsigned char GetCANChannel(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return CAN0;
			break;
		case RHR:
			return CAN0;
			break;
		case RHP:
			return CAN0;
			break;
		case RKN:
			return CAN0;
			break;
		case RAP:
			return CAN0;
			break;
		case RAR:
			return CAN0;
			break;
		case LHY:
			return CAN0;
			break;
		case LHR:
			return CAN0;
			break;
		case LHP:
			return CAN0;
			break;
		case LKN:
			return CAN0;
			break;
		case LAP:
			return CAN0;
			break;
		case LAR:
			return CAN0;
			break;
		case RSP:
			return CAN1;
			break;
		case RSR:
			return CAN1;
			break;
		case RSY:
			return CAN1;
			break;
		case REB:
			return CAN1;
			break;
		case RWY:
			return CAN1;
			break;
		case RWP:
			return CAN1;
			break;
		case LSP:
			return CAN1;
			break;
		case LSR:
			return CAN1;
			break;
		case LSY:
			return CAN1;
			break;
		case LEB:	
			return CAN1;
			break;
		case LWY:
			return CAN1;
			break;
		case LWP:
			return CAN1;
			break;
		case NKY:
			return CAN1;
			break;
		case NK1:
			return CAN1;
			break;
		case NK2:
			return CAN1;
			break;
		case WST:
			return CAN0;
			break;
		case RF1:
			return CAN1;
			break;
		case RF2:
			return CAN1;
			break;
		case RF3:
			return CAN1;
			break;
		case RF4:
			return CAN1;
			break;
		case RF5:
			return CAN1;
			break;
		case LF1:
			return CAN1;
			break;
		case LF2:
			return CAN1;
			break;
		case LF3:
			return CAN1;
			break;
		case LF4:
			return CAN1;
			break;
		case LF5:
			return CAN1;
			break;
	}

	return 100;
}
/******************************************************************************/







/******************************************************************************/
unsigned char GetBoardID(unsigned char _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return JMC0;
			break;
		case RHR:
			return JMC0;
			break;
		case RHP:
			return JMC1;
			break;
		case RKN:
			return JMC2;
			break;
		case RAP:
			return JMC3;
			break;
		case RAR:
			return JMC3;
			break;
		case LHY:
			return JMC4;
			break;
		case LHR:
			return JMC4;
			break;
		case LHP:
			return JMC5;
			break;
		case LKN:
			return JMC6;
			break;
		case LAP:
			return JMC7;
			break;
		case LAR:
			return JMC7;
			break;
		case RSP:
			return JMC8;
			break;
		case RSR:
			return JMC8;
			break;
		case RSY:
			return JMC9;
			break;
		case REB:
			return JMC9;
			break;
		case RWY:
			return EJMC0;
			break;
		case RWP:
			return EJMC0;
			break;
		case LSP:
			return JMC10;
			break;
		case LSR:
			return JMC10;
			break;
		case LSY:
			return JMC11;
			break;
		case LEB:	
			return JMC11;
			break;
		case LWY:
			return EJMC1;
			break;
		case LWP:
			return EJMC1;
			break;
		case NKY:
			return EJMC2;
			break;
		case NK1:
			return EJMC2;
			break;
		case NK2:
			return EJMC2;
			break;
		case WST:
			return EJMC3;
			break;
		case RF1:
			return EJMC4;
			break;
		case RF2:
			return EJMC4;
			break;
		case RF3:
			return EJMC4;
			break;
		case RF4:
			return EJMC4;
			break;
		case RF5:
			return EJMC4;
			break;
		case LF1:
			return EJMC5;
			break;
		case LF2:
			return EJMC5;
			break;
		case LF3:
			return EJMC5;
			break;
		case LF4:
			return EJMC5;
			break;
		case LF5:
			return EJMC5;
			break;
	}

	return 100;
}
/******************************************************************************/







/******************************************************************************/
bool PositionLimitOnOff(unsigned char _jointID, unsigned char _mode)
{
	unsigned char tempData[8];
	
	tempData[0] = GetBoardID(_jointID);
	tempData[1] = 0x56 + GetMotorChannel(_jointID);
	tempData[2] = _mode;	
	CanSendMsg(GetCANChannel(_mode), CMD_TXDF, tempData, 3, 0);
	RtSleep(10);
		
	tempData[1] = 0x50 + GetMotorChannel(_jointID);
	CanSendMsg(GetCANChannel(_mode), CMD_TXDF, tempData, 3, 0);
	RtSleep(10);
	
	return true;
}
/******************************************************************************/






/******************************************************************************/
bool Beep(unsigned char _mode)
{
	unsigned char tempData[8];
	
	tempData[0] = 0x0E;
	tempData[1] = 0x82;
	tempData[2] = _mode;	
	CanSendMsg(CAN1, CMD_TXDF, tempData, 3, 0);
	RtSleep(10);
	
	return true;
}
/******************************************************************************/






/******************************************************************************/
bool SetDeadZone(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;	
	tempData[1] = 0x20 + _joint.Motor_channel;	
	tempData[2] = (_joint.Deadzone)&0x00FF;

	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 3, 0);

	RtWprintf(L"\n>>>Deadzone Set - Joint: %d\t%d", _joint.Motor_channel, tempData[2]);
	RtSleep(10);

	return true;
}
/******************************************************************************/






/******************************************************************************/
bool SetEncoderResolution(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;	
	tempData[1] = 0x38 + _joint.Motor_channel;
	tempData[2] = (unsigned char)(_joint.Encoder_size & 0xFF);
	tempData[3] = (unsigned char)((_joint.Encoder_size>>8) & 0xFF);
	tempData[3] |= ( (_joint.Positive_dir<<7) & 0x80 );

	RtWprintf(L"encoder input : %d", _joint.Encoder_size);
	RtWprintf(L"encoder input : %x", tempData[2]);
	RtWprintf(L"encoder input : %x", tempData[3]);

	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 6, 0);
	
	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetJamPwmSturation(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0xF2;
	tempData[2] = _joint.JAMmsTime & 0xFF;
	tempData[3] = (_joint.JAMmsTime>>8) & 0xFF;	// byte change
	tempData[4] = _joint.PWMmsTime & 0xFF;
	tempData[5] = (_joint.PWMmsTime>>8) & 0xFF;
	tempData[6] = _joint.JAMDuty;
	tempData[7] = _joint.PWMDuty;

	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/








/******************************************************************************/
bool SetMaxAccVel(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0x40 + _joint.Motor_channel;
	tempData[2] = _joint.MaxAcc & 0xFF;
	tempData[3] = (_joint.MaxAcc>>8) & 0xFF;	// byte change
	tempData[4] = _joint.MaxVel & 0xFF;
	tempData[5] = (_joint.MaxVel>>8) & 0xFF;
	
	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 6, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/








/******************************************************************************/
bool SetControlMode(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0x10;
	tempData[2] = _joint.MotorControlMode;
	
	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 3, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetHomeSearchParameter(JOINT _joint)
{
	unsigned char tempData[8];
	int itemp = (int)(_joint.Offset_angle*_joint.PPR);
	
	tempData[0] = _joint.JMC;
	tempData[1] = 0x30 + _joint.Motor_channel;
	tempData[2] = _joint.Limit_rev;
	tempData[3] = _joint.SearchDirection;
	tempData[4] = (itemp) & 0xFF;
	tempData[5] = (itemp>>8) & 0xFF;
	tempData[6] = (itemp>>16) & 0xFF;
	tempData[7] = (itemp>>24) & 0xFF;
	
	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/






/******************************************************************************/
bool SetHomeMaxVelAcc(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0x60 + _joint.Motor_channel;
	tempData[2] = _joint.MaxAccHome;
	tempData[3] = (_joint.MaxVelHome) & 0xFF;
	tempData[4] = (_joint.MaxVelHome>>8) & 0xFF;
	tempData[5] = _joint.HomeSearchMode;
	tempData[6] = _joint.PWMDuty;
	
	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 7, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetLowerPositionLimit(JOINT _joint)
{
	unsigned char tempData[8];
	int itemp = (int)(_joint.LowerPositionLimit*_joint.PPR);

	tempData[0] = _joint.JMC;
	tempData[1] = 0x50 + _joint.Motor_channel;
	tempData[2] = 0x02;
	tempData[3] = (itemp) & 0xFF;
	tempData[4] = (itemp>>8) & 0xFF;
	tempData[5] = (itemp>>16) & 0xFF;
	tempData[6] = (itemp>>24) & 0xFF;
	
	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 7, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetErrorBound(JOINT _joint)
{
	unsigned char tempData[8];
	unsigned int itemp[3];

	itemp[0] = _joint.I_ERR;
	itemp[1] = _joint.B_ERR;
	itemp[2] = _joint.E_ERR;

	tempData[0] = _joint.JMC;
	tempData[1] = 0xF3;
	tempData[2] = (itemp[0]) & 0xFF;
	tempData[3] = (itemp[0]>>8) & 0xFF;
	tempData[4] = (itemp[1]) & 0xFF;
	tempData[5] = (itemp[1]>>8) & 0xFF;
	tempData[6] = (itemp[2]) & 0xFF;
	tempData[7] = (itemp[2]>>8) & 0xFF;
	
	
	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetUpperPositionLimit(JOINT _joint)
{
	unsigned char tempData[8];
	int itemp = (int)(_joint.UpperPositionLimit*_joint.PPR);

	tempData[0] = _joint.JMC;
	tempData[1] = 0x56 + _joint.Motor_channel;
	tempData[2] = 0x02;
	tempData[3] = (itemp) & 0xFF;
	tempData[4] = (itemp>>8) & 0xFF;
	tempData[5] = (itemp>>16) & 0xFF;
	tempData[6] = (itemp>>24) & 0xFF;
	
	CanSendMsg(_joint.CAN_channel, CMD_TXDF, tempData, 7, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool RequestParameters(JOINT* _joint)
{
	unsigned char i, j;
	unsigned char sendData[8], receiveData[8];
	unsigned int resultData[9][8];
	unsigned char motorCH = _joint->Motor_channel;
	unsigned int  count = 0;
	unsigned char checkSum, check;

	unsigned char buff_no;
	bool		result = true;


	sendData[0] = _joint->JMC;	sendData[1] = RequestPara;
	
	if(motorCH < 3) sendData[2] = motorCH*6 + 1;
	else sendData[2] = motorCH*6 + 1 + 5;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 0);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		//if(ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData) == ERR_OK) count = 3500;
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[0][i] = (unsigned int)receiveData[i];
	

	if(motorCH < 3) sendData[2] = motorCH*6 + 2;
	else sendData[2] = motorCH*6 + 2 + 5;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 1);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		//if(ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData) == ERR_OK) count = 3500;
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[1][i] = (unsigned int)receiveData[i];


	if(motorCH < 3) sendData[2] = motorCH*6 + 3;
	else sendData[2] = motorCH*6 + 3 + 5;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 2);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[2][i] = (unsigned int)receiveData[i];


	if(motorCH < 3) sendData[2] = motorCH*6 + 4;
	else sendData[2] = motorCH*6 + 4 + 5;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 3);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[3][i] = (unsigned int)receiveData[i];


	if(motorCH < 3) sendData[2] = motorCH*6 + 5;
	else sendData[2] = motorCH*6 + 5 + 5;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 4);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[4][i] = (unsigned int)receiveData[i];
		

	if(motorCH < 3) sendData[2] = motorCH*6 + 6;
	else sendData[2] = motorCH*6 + 6 + 5;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 5);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[5][i] = (unsigned int)receiveData[i];


	sendData[2] = 20;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 6);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[6][i] = (unsigned int)receiveData[i];


	sendData[2] = 21;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 7);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[7][i] = (unsigned int)receiveData[i];

		
	sendData[2] = 22;
	count = 0;
	CanSendMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 8);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[8][i] = (unsigned int)receiveData[i];
	

	if( (_joint->JointID<=LWP) ||  (_joint->JointID==WST))
	{
		for(i=2 ; i<=7 ; i++)
		{
			checkSum = 0x00;
			for(j=0 ; j<5 ; j++) checkSum += resultData[j][i];
			check = checkSum&0xFF;
			if(check != resultData[5][i]) { RtWprintf(L"\n>>> CheckSum Error - Joint:%d\t No", _joint->JointID, i); result = false; }
		}
	}

	_joint->Position_Kp = resultData[0][0] | (resultData[0][1]<<8);
	_joint->Position_Ki = resultData[0][2] | (resultData[0][3]<<8);
	_joint->Position_Kd = resultData[0][4] | (resultData[0][5]<<8);
	_joint->Encoder_size = resultData[0][6] | (resultData[0][7]&0x3F)<<8;
	_joint->Positive_dir = (resultData[0][7]&0x80)>>7;
	_joint->Deadzone = resultData[1][0] | (resultData[1][1]<<8);
	_joint->SearchDirection = resultData[1][2];
	_joint->HomeSearchMode = resultData[1][3];
	_joint->Limit_rev = resultData[1][4] | (resultData[1][5]<<8);
	_joint->PPR	 = (float)_joint->HDReduction*((float)_joint->Pulley_driven/(float)_joint->Pulley_drive)*((float)_joint->Encoder_size)/360.0f;
	_joint->Offset_angle = (float)((int)resultData[1][6]|(int)(resultData[1][7]<<8)|(int)(resultData[2][0]<<16)|(int)(resultData[2][1]<<24))/_joint->PPR;
	_joint->LowerPositionLimit = (float)((int)resultData[2][2] | (int)(resultData[2][3]<<8) | (int)(resultData[2][4]<<16) | (int)(resultData[2][5]<<24))/_joint->PPR;
	_joint->UpperPositionLimit = (float)((int)resultData[2][6] | (int)(resultData[2][7]<<8) | (int)(resultData[3][0]<<16) | (int)(resultData[3][1]<<24))/_joint->PPR;
	_joint->MaxAcc = resultData[3][2] | (resultData[3][3]<<8);
	_joint->MaxVel = resultData[3][4] | (resultData[3][5]<<8);
	_joint->MaxPWM = resultData[3][6] | (resultData[3][7]<<8);
	_joint->MaxAccHome = resultData[6][6] | (resultData[6][7]<<8);
	_joint->MaxVelHome = resultData[7][0] | (resultData[7][1]<<8);
	_joint->JAMmsTime = resultData[7][4] | (resultData[7][5]<<8);
	_joint->PWMmsTime = resultData[7][6] | (resultData[7][7]<<8);
	_joint->PWMDuty = resultData[8][0];
	_joint->JAMDuty = resultData[8][1];
	_joint->I_ERR = resultData[8][2] | (resultData[8][3]<<8);
	_joint->B_ERR = resultData[8][4] | (resultData[8][5]<<8);
	_joint->E_ERR = resultData[8][6] | (resultData[8][7]<<8);
	
	return result;
}
/******************************************************************************/








/******************************************************************************/
bool CheckBoardStatus(JOINT* _joint)
{
	bool update = false;
	unsigned char i, receiveData[8];

	for(i=JMC0 ; i<=JMC11 ; i++)
	{
		switch(i)
		{
		case JMC0:
			if(ReadMBData(STAT_BASE_RXDF+JMC0, receiveData) == ERR_OK)
			{	
				_joint[RHY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RHY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RHY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RHY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RHY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	

				_joint[RHY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RHY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RHY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RHY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RHY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RHY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RHY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);

				_joint[RHY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RHY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RHY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RHY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RHY].TempError =			(bool)((receiveData[2]>>4)&0x01);


				_joint[RHR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RHR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RHR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RHR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RHR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RHR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RHR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RHR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RHR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RHR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RHR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RHR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RHR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RHR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RHR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RHR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RHR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC1:
			if(ReadMBData(STAT_BASE_RXDF+JMC1, receiveData) == ERR_OK) 
			{	
				_joint[RHP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RHP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RHP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RHP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RHP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RHP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RHP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RHP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RHP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RHP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RHP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RHP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RHP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RHP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RHP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RHP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RHP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC2:
			if(ReadMBData(STAT_BASE_RXDF+JMC2, receiveData) == ERR_OK) 
			{	
				_joint[RKN].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RKN].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RKN].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RKN].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RKN].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RKN].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RKN].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RKN].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RKN].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RKN].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RKN].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RKN].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RKN].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RKN].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RKN].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RKN].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RKN].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC3:
			if(ReadMBData(STAT_BASE_RXDF+JMC3, receiveData) == ERR_OK) 
			{	
				_joint[RAP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RAP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RAP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RAP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RAP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RAP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RAP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RAP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RAP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RAP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RAP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RAP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RAP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RAP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RAP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RAP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RAP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[RAR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RAR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RAR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RAR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RAR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RAR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RAR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RAR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RAR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RAR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RAR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RAR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RAR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RAR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RAR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RAR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RAR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC4:
			if(ReadMBData(STAT_BASE_RXDF+JMC4, receiveData) == ERR_OK) 
			{	
				_joint[LHY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LHY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LHY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LHY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LHY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LHY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LHY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LHY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LHY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LHY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LHY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LHY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LHY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LHY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LHY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LHY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LHY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LHR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LHR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LHR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LHR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LHR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LHR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LHR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LHR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LHR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LHR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LHR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LHR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[LHR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LHR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LHR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LHR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LHR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC5:
			if(ReadMBData(STAT_BASE_RXDF+JMC5, receiveData) == ERR_OK) 
			{				
				_joint[LHP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LHP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LHP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LHP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LHP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LHP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LHP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LHP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LHP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LHP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LHP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LHP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LHP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LHP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LHP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LHP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LHP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC6:
			if(ReadMBData(STAT_BASE_RXDF+JMC6, receiveData) == ERR_OK) 
			{				
				_joint[LKN].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LKN].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LKN].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LKN].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LKN].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LKN].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LKN].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LKN].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LKN].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LKN].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LKN].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LKN].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LKN].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LKN].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LKN].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LKN].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LKN].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC7:
			if(ReadMBData(STAT_BASE_RXDF+JMC7, receiveData) == ERR_OK) 
			{	
				_joint[LAP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LAP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LAP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LAP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LAP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LAP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LAP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LAP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LAP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LAP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LAP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LAP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LAP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LAP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LAP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LAP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LAP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LAR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LAR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LAR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LAR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LAR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LAR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LAR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LAR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LAR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LAR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LAR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LAR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[LAR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LAR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LAR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LAR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LAR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC8:
			if(ReadMBData(STAT_BASE_RXDF+JMC8, receiveData) == ERR_OK) 
			{	
				_joint[RSP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RSP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RSP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RSP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RSP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RSP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RSP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RSP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RSP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RSP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RSP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RSP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RSP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RSP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RSP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RSP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RSP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[RSR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RSR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RSR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RSR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RSR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RSR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RSR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RSR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RSR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RSR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RSR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RSR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RSR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RSR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RSR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RSR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RSR].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		case JMC9:
			if(ReadMBData(STAT_BASE_RXDF+JMC9, receiveData) == ERR_OK) 
			{				
				_joint[RSY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RSY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RSY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RSY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RSY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RSY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RSY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RSY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RSY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RSY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RSY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RSY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RSY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RSY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RSY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RSY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RSY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[REB].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[REB].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[REB].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[REB].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[REB].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[REB].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[REB].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[REB].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[REB].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[REB].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[REB].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[REB].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
		
				_joint[REB].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[REB].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[REB].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[REB].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[REB].TempError =			(bool)((receiveData[6]>>4)&0x01);
		
				update = true;
			}
			break;
		case JMC10:
			if(ReadMBData(STAT_BASE_RXDF+JMC10, receiveData) == ERR_OK) 
			{				
				_joint[LSP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LSP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LSP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LSP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LSP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LSP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LSP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LSP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LSP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LSP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LSP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LSP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LSP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LSP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LSP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LSP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LSP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LSR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LSR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LSR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LSR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LSR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
			
				_joint[LSR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LSR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LSR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LSR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LSR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LSR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LSR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
		
				_joint[LSR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LSR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LSR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LSR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LSR].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		case JMC11:
			if(ReadMBData(STAT_BASE_RXDF+JMC11, receiveData) == ERR_OK) 
			{				
				_joint[LSY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LSY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LSY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LSY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LSY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LSY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LSY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LSY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LSY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LSY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LSY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LSY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LSY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LSY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LSY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LSY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LSY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LEB].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LEB].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LEB].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LEB].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LEB].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LEB].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LEB].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LEB].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LEB].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LEB].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LEB].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LEB].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
			
				_joint[LEB].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LEB].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LEB].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LEB].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LEB].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		}
	}

	for(i=EJMC0 ; i<=EJMC5 ; i++)
	{
		switch(i)		
		{
		case EJMC0:
			if(ReadMBData(STAT_BASE_RXDF+EJMC0, receiveData) == NODATA) 
			{	
				_joint[RWY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RWY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RWY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RWY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RWY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RWY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RWY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RWY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RWY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RWY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RWY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RWY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RWY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RWY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RWY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RWY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RWY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[RWP].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RWP].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RWP].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RWP].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RWP].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RWP].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RWP].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RWP].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RWP].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RWP].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RWP].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RWP].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RWP].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RWP].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RWP].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RWP].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RWP].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		case EJMC1:
			if(ReadMBData(STAT_BASE_RXDF+EJMC1, receiveData) == ERR_OK) 
			{				
				_joint[LWY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LWY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LWY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LWY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LWY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LWY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LWY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LWY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LWY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LWY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LWY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LWY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LWY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LWY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LWY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LWY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LWY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LWP].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LWP].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LWP].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LWP].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LWP].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LWP].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LWP].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LWP].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LWP].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LWP].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LWP].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LWP].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[LWP].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LWP].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LWP].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LWP].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LWP].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case EJMC2:
			if(ReadMBData(STAT_BASE_RXDF+EJMC2, receiveData) == ERR_OK) 
			{
				_joint[NKY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[NKY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[NKY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[NKY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[NKY].JAMError =			(bool)((receiveData[0]>>4)&0x01);
				_joint[NKY].PWMError =			(bool)((receiveData[0]>>5)&0x01);
				_joint[NKY].BigError =			(bool)((receiveData[0]>>6)&0x01);
				_joint[NKY].EncoderError =		(bool)((receiveData[0]>>7)&0x01);

				_joint[NK1].FetOnOff =			(bool)(receiveData[1]&0x01);
				_joint[NK1].ControlOnOff =		(bool)((receiveData[1]>>1)&0x01);
				_joint[NK1].MotorControlMode =	(unsigned char)((receiveData[1]>>2)&0x01);
				_joint[NK1].LimitOnOff =		(bool)((receiveData[1]>>3)&0x01);
				_joint[NK1].JAMError =			(bool)((receiveData[1]>>4)&0x01);
				_joint[NK1].PWMError =			(bool)((receiveData[1]>>5)&0x01);
				_joint[NK1].BigError =			(bool)((receiveData[1]>>6)&0x01);
				_joint[NK1].EncoderError =		(bool)((receiveData[1]>>7)&0x01);

				_joint[NK2].FetOnOff =			(bool)(receiveData[2]&0x01);
				_joint[NK2].ControlOnOff =		(bool)((receiveData[2]>>1)&0x01);
				_joint[NK2].MotorControlMode =	(unsigned char)((receiveData[2]>>2)&0x01);
				_joint[NK2].LimitOnOff =		(bool)((receiveData[2]>>3)&0x01);
				_joint[NK2].JAMError =			(bool)((receiveData[2]>>4)&0x01);
				_joint[NK2].PWMError =			(bool)((receiveData[2]>>5)&0x01);
				_joint[NK2].BigError =			(bool)((receiveData[2]>>6)&0x01);
				_joint[NK2].EncoderError =		(bool)((receiveData[2]>>7)&0x01);

				update = true;
			}
			break;
		case EJMC3:
			if(ReadMBData(STAT_BASE_RXDF+EJMC3, receiveData) == ERR_OK) 
			{								
				_joint[WST].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[WST].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[WST].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[WST].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[WST].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[WST].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[WST].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[WST].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[WST].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[WST].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[WST].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[WST].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[WST].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[WST].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[WST].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[WST].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[WST].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case EJMC4:
			if(ReadMBData(STAT_BASE_RXDF+EJMC4, receiveData) == ERR_OK) 
			{
				_joint[RF1].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RF1].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RF1].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RF1].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RF1].JAMError =			(bool)((receiveData[0]>>4)&0x01);
				_joint[RF1].PWMError =			(bool)((receiveData[0]>>5)&0x01);
				_joint[RF1].BigError =			(bool)((receiveData[0]>>6)&0x01);
				_joint[RF1].EncoderError =		(bool)((receiveData[0]>>7)&0x01);
				
				_joint[RF2].FetOnOff =			(bool)(receiveData[1]&0x01);
				_joint[RF2].ControlOnOff =		(bool)((receiveData[1]>>1)&0x01);
				_joint[RF2].MotorControlMode =	(unsigned char)((receiveData[1]>>2)&0x01);
				_joint[RF2].LimitOnOff =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RF2].JAMError =			(bool)((receiveData[1]>>4)&0x01);
				_joint[RF2].PWMError =			(bool)((receiveData[1]>>5)&0x01);
				_joint[RF2].BigError =			(bool)((receiveData[1]>>6)&0x01);
				_joint[RF2].EncoderError =		(bool)((receiveData[1]>>7)&0x01);
				
				_joint[RF3].FetOnOff =			(bool)(receiveData[2]&0x01);
				_joint[RF3].ControlOnOff =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RF3].MotorControlMode =	(unsigned char)((receiveData[2]>>2)&0x01);
				_joint[RF3].LimitOnOff =		(bool)((receiveData[2]>>3)&0x01);
				_joint[RF3].JAMError =			(bool)((receiveData[2]>>4)&0x01);
				_joint[RF3].PWMError =			(bool)((receiveData[2]>>5)&0x01);
				_joint[RF3].BigError =			(bool)((receiveData[2]>>6)&0x01);
				_joint[RF3].EncoderError =		(bool)((receiveData[2]>>7)&0x01);

				_joint[RF4].FetOnOff =			(bool)(receiveData[3]&0x01);
				_joint[RF4].ControlOnOff =		(bool)((receiveData[3]>>1)&0x01);
				_joint[RF4].MotorControlMode =	(unsigned char)((receiveData[3]>>2)&0x01);
				_joint[RF4].LimitOnOff =		(bool)((receiveData[3]>>3)&0x01);
				_joint[RF4].JAMError =			(bool)((receiveData[3]>>4)&0x01);
				_joint[RF4].PWMError =			(bool)((receiveData[3]>>5)&0x01);
				_joint[RF4].BigError =			(bool)((receiveData[3]>>6)&0x01);
				_joint[RF4].EncoderError =		(bool)((receiveData[3]>>7)&0x01);
				
				_joint[RF5].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RF5].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RF5].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RF5].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RF5].JAMError =			(bool)((receiveData[4]>>4)&0x01);
				_joint[RF5].PWMError =			(bool)((receiveData[4]>>5)&0x01);
				_joint[RF5].BigError =			(bool)((receiveData[4]>>6)&0x01);
				_joint[RF5].EncoderError =		(bool)((receiveData[4]>>7)&0x01);
	
				update = true;
			}
			break;
		case EJMC5:
			if(ReadMBData(STAT_BASE_RXDF+EJMC5, receiveData) == ERR_OK) 
			{
				_joint[LF1].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LF1].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LF1].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LF1].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LF1].JAMError =			(bool)((receiveData[0]>>4)&0x01);
				_joint[LF1].PWMError =			(bool)((receiveData[0]>>5)&0x01);
				_joint[LF1].BigError =			(bool)((receiveData[0]>>6)&0x01);
				_joint[LF1].EncoderError =		(bool)((receiveData[0]>>7)&0x01);
				
				_joint[LF2].FetOnOff =			(bool)(receiveData[1]&0x01);
				_joint[LF2].ControlOnOff =		(bool)((receiveData[1]>>1)&0x01);
				_joint[LF2].MotorControlMode =	(unsigned char)((receiveData[1]>>2)&0x01);
				_joint[LF2].LimitOnOff =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LF2].JAMError =			(bool)((receiveData[1]>>4)&0x01);
				_joint[LF2].PWMError =			(bool)((receiveData[1]>>5)&0x01);
				_joint[LF2].BigError =			(bool)((receiveData[1]>>6)&0x01);
				_joint[LF2].EncoderError =		(bool)((receiveData[1]>>7)&0x01);
				
				_joint[LF3].FetOnOff =			(bool)(receiveData[2]&0x01);
				_joint[LF3].ControlOnOff =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LF3].MotorControlMode =	(unsigned char)((receiveData[2]>>2)&0x01);
				_joint[LF3].LimitOnOff =		(bool)((receiveData[2]>>3)&0x01);
				_joint[LF3].JAMError =			(bool)((receiveData[2]>>4)&0x01);
				_joint[LF3].PWMError =			(bool)((receiveData[2]>>5)&0x01);
				_joint[LF3].BigError =			(bool)((receiveData[2]>>6)&0x01);
				_joint[LF3].EncoderError =		(bool)((receiveData[2]>>7)&0x01);
				
				_joint[LF4].FetOnOff =			(bool)(receiveData[3]&0x01);
				_joint[LF4].ControlOnOff =		(bool)((receiveData[3]>>1)&0x01);
				_joint[LF4].MotorControlMode =	(unsigned char)((receiveData[3]>>2)&0x01);
				_joint[LF4].LimitOnOff =		(bool)((receiveData[3]>>3)&0x01);
				_joint[LF4].JAMError =			(bool)((receiveData[3]>>4)&0x01);
				_joint[LF4].PWMError =			(bool)((receiveData[3]>>5)&0x01);
				_joint[LF4].BigError =			(bool)((receiveData[3]>>6)&0x01);
				_joint[LF4].EncoderError =		(bool)((receiveData[3]>>7)&0x01);
				
				_joint[LF5].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LF5].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LF5].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LF5].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LF5].JAMError =			(bool)((receiveData[4]>>4)&0x01);
				_joint[LF5].PWMError =			(bool)((receiveData[4]>>5)&0x01);
				_joint[LF5].BigError =			(bool)((receiveData[4]>>6)&0x01);
				_joint[LF5].EncoderError =		(bool)((receiveData[4]>>7)&0x01);
				
				update = true;
			}
			break;
		}
	}

	for(i=0 ; i<NO_OF_JOINT ; i++)
	{
		if(_joint[i].JAMError == true) Beep(1);
		if(_joint[i].PWMError == true) Beep(2);
		if(_joint[i].BigError == true) Beep(3);
		if(_joint[i].EncoderError == true) Beep(4);
	}

	return update;
}
/******************************************************************************/








/******************************************************************************/
bool SetMoveJointAngle(unsigned char _jointID, float _angle, float _msTime, unsigned char _mode)
{
	if(_msTime <= 0) { RtWprintf(L"\n>>> goal time must be grater than zero..!!"); return false; }
	if(_jointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(SetMoveJointAngle)..!!"); return false; }
	
	switch(_mode)
	{
	case 0x00:	// relative mode
		Joint[_jointID].RefAngleToGo = Joint[_jointID].RefAngleCurrent + _angle;
		break;
	case 0x01:	// absolute mode
		Joint[_jointID].RefAngleToGo = _angle;
		break;
	default:
		RtWprintf(L"\n>>> Wrong reference mode(SetMoveJointAngle)..!!");
		return false;
		break;
	}

	Joint[_jointID].MoveFlag = false;
	Joint[_jointID].RefAngleInitial = Joint[_jointID].RefAngleCurrent;
	Joint[_jointID].CurrentTimeCount = 0;
	if(Joint[_jointID].CAN_channel == CAN0) Joint[_jointID].GoalTimeCount = (unsigned long)(_msTime/INT_TIME);
	else Joint[_jointID].GoalTimeCount = (unsigned long)(_msTime/INT_TIME1);
	Joint[_jointID].RefAngleDelta = Joint[_jointID].RefAngleToGo - Joint[_jointID].RefAngleCurrent;
	Joint[_jointID].MoveFlag = true;

	switch(_jointID)
	{
	case RHY:
		RtWprintf(L"\n>>> RHY joint reference is set..!!");
		break;
	case RHR:
		RtWprintf(L"\n>>> RHR joint reference is set..!!");
		break;
	case RHP:
		RtWprintf(L"\n>>> RHP joint reference is set..!!");
		break;
	case RKN:
		RtWprintf(L"\n>>> RKN joint reference is set..!!");
		break;
	case RAP:
		RtWprintf(L"\n>>> RAP joint reference is set..!!");
		break;
	case RAR: 
		RtWprintf(L"\n>>> RAR joint reference is set..!!");
		break;
	case LHY:
		RtWprintf(L"\n>>> LHY joint reference is set..!!");
		break;
	case LHR:
		RtWprintf(L"\n>>> LHR joint reference is set..!!");
		break;
	case LHP:
		RtWprintf(L"\n>>> LHP joint reference is set..!!");
		break;
	case LKN:
		RtWprintf(L"\n>>> LKN joint reference is set..!!");
		break;
	case LAP:
		RtWprintf(L"\n>>> LAP joint reference is set..!!");
		break;
	case LAR:
		RtWprintf(L"\n>>> LAR joint reference is set..!!");
		break;
	case RSP:
		RtWprintf(L"\n>>> RSP joint reference is set..!!");
		break;
	case RSR:
		RtWprintf(L"\n>>> RSR joint reference is set..!!");
		break;
	case RSY:
		RtWprintf(L"\n>>> RSY joint reference is set..!!");
		break;
	case REB:
		RtWprintf(L"\n>>> REB joint reference is set..!!");
		break;
	case LSP:
		RtWprintf(L"\n>>> LSP joint reference is set..!!");
		break;
	case LSR:
		RtWprintf(L"\n>>> LSR joint reference is set..!!");
		break;
	case LSY:
		RtWprintf(L"\n>>> LSY joint reference is set..!!");
		break;
	case LEB:
		RtWprintf(L"\n>>> LEB joint reference is set..!!");
		break;
	case RWY:
		RtWprintf(L"\n>>> RWY joint reference is set..!!");
		break;
	case RWP:
		RtWprintf(L"\n>>> RWP joint reference is set..!!");
		break;
	case LWY:
		RtWprintf(L"\n>>> LWY joint reference is set..!!");
		break;
	case LWP:
		RtWprintf(L"\n>>> LWP joint reference is set..!!");
		break;
	case NKY:
		RtWprintf(L"\n>>> NKY joint reference is set..!!");
		break;
	case NK1:
		RtWprintf(L"\n>>> NK1 joint reference is set..!!");
		break;
	case NK2:
		RtWprintf(L"\n>>> NK2 joint reference is set..!!");
		break;
	case WST:
		RtWprintf(L"\n>>> WST joint reference is set..!!");
		break;
	case RF1:
		RtWprintf(L"\n>>> RF1 joint reference is set..!!");
		break;
	case RF2:
		RtWprintf(L"\n>>> RF2 joint reference is set..!!");
		break;
	case RF3:
		RtWprintf(L"\n>>> RF3 joint reference is set..!!");
		break;
	case RF4:
		RtWprintf(L"\n>>> RF4 joint reference is set..!!");
		break;
	case RF5:
		RtWprintf(L"\n>>> RF5 joint reference is set..!!");
		break;
	case LF1:
		RtWprintf(L"\n>>> LF1 joint reference is set..!!");
		break;
	case LF2:
		RtWprintf(L"\n>>> LF2 joint reference is set..!!");
		break;
	case LF3:
		RtWprintf(L"\n>>> LF3 joint reference is set..!!");
		break;
	case LF4:
		RtWprintf(L"\n>>> LF4 joint reference is set..!!");
		break;
	case LF5:
		RtWprintf(L"\n>>> LF5 joint reference is set..!!");
		break;
	default:
		RtWprintf(L"\n>>> Wrong joint ID(SetMoveJointAngle)..!!");
		return false;
		break;
	}

	return true;
}
/******************************************************************************/




/******************************************************************************/
void MoveJointAngle(unsigned char _canChannel)
{
	unsigned char i;
	static float RefAngle_Last[NO_OF_JOINT] = {0.0f,};
	
	if(_canChannel == CAN0) 
	{
		for(i=RHY ; i<=LAR ; i++)
		{
			if(Joint[i].MoveFlag == true)
			{
				if(Joint[i].GoalTimeCount == Joint[i].CurrentTimeCount)
				{
					Joint[i].GoalTimeCount = Joint[i].CurrentTimeCount = 0;
					Joint[i].RefAngleCurrent = Joint[i].RefAngleToGo;
					Joint[i].MoveFlag = false;
				}
				else 
				{
					Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].RefAngleDelta*0.5f*(1.0f-cosf(PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
					Joint[i].CurrentTimeCount++;
				}
			}
		}
		for(i=JMC0 ; i<=JMC7 ; i++) MoveJMC(i);
		MoveJMC(EJMC3);
	}
	else if(_canChannel == CAN1)
	{
		for(i=RSP ; i<=RF5 ; i++)
		{
			if(Joint[i].MoveFlag == true)
			{
				if(Joint[i].GoalTimeCount == Joint[i].CurrentTimeCount)
				{
					Joint[i].GoalTimeCount = Joint[i].CurrentTimeCount = 0;
					Joint[i].RefAngleCurrent = Joint[i].RefAngleToGo;
					Joint[i].RefVelCurrent = 0.0f;
					Joint[i].MoveFlag = false;
				}
				else 
				{
					Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].RefAngleDelta*0.5f*(1.0f-cosf(PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
					Joint[i].RefVelCurrent = Joint[i].RefAngleCurrent - RefAngle_Last[i];
					RefAngle_Last[i] = Joint[i].RefAngleCurrent;
					Joint[i].CurrentTimeCount++;
				}
			}
		}
		for(i=JMC8 ; i<=JMC11 ; i++) MoveJMC(i);
		for(i=EJMC0 ; i<=EJMC2 ; i++) MoveJMC(i);
		for(i=EJMC4 ; i<=EJMC5 ; i++) MoveJMC(i);
	}
}
/******************************************************************************/




/******************************************************************************/
void UpperBodyMotionCapture(unsigned char _canChannel)
{
	if(MotionSetFlag == 0) return;
	
	if(TransitionFlag == 1)
	{
		if(MotionTimeCurrent == Ti+30 && Ti > OptimalTfMax)
		{
			Motion = tempMotion;
			MotionTimeCurrent = 0; 
		}
	}
	if(_canChannel == CAN0) 
	{
	}
	else if(_canChannel == CAN1)
	{
		Joint[NKY].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][1]-MotionCapture[Motion][MotionTimeCurrent-1][1];
		Joint[NK1].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][2]-MotionCapture[Motion][MotionTimeCurrent-1][2];
		Joint[NK2].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][26]-MotionCapture[Motion][MotionTimeCurrent-1][26];

		Joint[RSP].RefAngleCurrent = Joint[RSP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][ 9];	// R-Shoulder Pitch	
		Joint[RSR].RefAngleCurrent = Joint[RSR].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][10];	// R-Shoulder Roll	
		Joint[RSY].RefAngleCurrent = Joint[RSY].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][11];			// R-Shoulder Yaw
		Joint[REB].RefAngleCurrent = Joint[REB].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][12];	// R-Elbow	Pitch
		Joint[RWY].RefAngleCurrent = Joint[RWY].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][13];
		Joint[RWP].RefAngleCurrent = Joint[RWP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][14];
		//Joint[RHP].RefAngleCurrent = Joint[RHP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][15];

		Joint[RF1].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][16]-MotionCapture[Motion][MotionTimeCurrent-1][16];
		Joint[RF2].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][17]-MotionCapture[Motion][MotionTimeCurrent-1][17];
		Joint[RF3].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][18]-MotionCapture[Motion][MotionTimeCurrent-1][18];
		Joint[RF4].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][19]-MotionCapture[Motion][MotionTimeCurrent-1][19];
		Joint[RF5].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][20]-MotionCapture[Motion][MotionTimeCurrent-1][20];
		
		Joint[LSP].RefAngleCurrent = Joint[LSP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][3];	// L-Shoulder Pitch
		Joint[LSR].RefAngleCurrent = Joint[LSR].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][4];	// L-Shoulder Roll
		Joint[LSY].RefAngleCurrent = Joint[LSY].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][5];			// L-Shoulder Yaw
		Joint[LEB].RefAngleCurrent = Joint[LEB].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][6];	// L-Elbow	Pitch		
		Joint[LWY].RefAngleCurrent = Joint[LWY].WalkReadyAngle -MotionCapture[Motion][MotionTimeCurrent][7];
		Joint[LWP].RefAngleCurrent = Joint[LWP].WalkReadyAngle -MotionCapture[Motion][MotionTimeCurrent][8];
		//Joint[LHP].RefAngleCurrent = Joint[LHP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][15];
		
		Joint[LF1].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][21]-MotionCapture[Motion][MotionTimeCurrent-1][21];
		Joint[LF2].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][22]-MotionCapture[Motion][MotionTimeCurrent-1][22];
		Joint[LF3].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][23]-MotionCapture[Motion][MotionTimeCurrent-1][23];
		Joint[LF4].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][24]-MotionCapture[Motion][MotionTimeCurrent-1][24];
		Joint[LF5].RefVelCurrent = MotionCapture[Motion][MotionTimeCurrent][25]-MotionCapture[Motion][MotionTimeCurrent-1][25];

		MotionTimeCurrent++;
		
		if((MotionTimeCurrent == MotionPeriod[Motion]+2 && TransitionFlag != 1)|| MotionTimeCurrent >= MocapTimeLimit -10) MotionSetFlag = 0;
		if(MotionTimeCurrent == OptimalTfMax) TransitionFlag = 0;
	}
}
/*****************************************************************************/






/*****************************************************************************/
char FTN_half_1_cos(float mag, long time, long start, long during, int delay1, int delay2, float *result)
{
	long	temp,temp1;
	temp=start+delay1;
	temp1=start+during-delay2;
	
	if(time<temp)
	{
		*result=0;
		return 0;
	}
	else if((time>=temp)&&(time<=temp1))
	{
		*result=mag*(1-(float)cos(PI*(float)(time-temp)/(float)(during-(delay1+delay2))))/2;
		return 1;
	}
	else return 3;
}
/*****************************************************************************/






/*****************************************************************************/
void Optimization(int _MotionNo)
{
	static float a[NO_OF_JOINT_MC];
	static float b[NO_OF_JOINT_MC];
	static float c[NO_OF_JOINT_MC];
	static float d[NO_OF_JOINT_MC];
	static float e[NO_OF_JOINT_MC];
	static float f[NO_OF_JOINT_MC];
	unsigned int	JointIndex;
	//int	Ti;
	unsigned int Tf;
	unsigned int M1, M2;
	unsigned int t;

	M1 = Motion;
	M2 = _MotionNo;
	Ti = MotionTimeCurrent;
	OptimalTfMax = 0;
	///*
	for(JointIndex = 0;JointIndex<NO_OF_JOINT_MC;JointIndex++)
	{/*	
		a[JointIndex] = 0;
		b[JointIndex] = 0;
		c[JointIndex] = 0;
		d[JointIndex] = 0;
		for (Tf=0;Tf<TfLimit;Tf++)
		{
			for(t=0;t < Tf;t++)
			{
				TransitionCandidate[Tf][t][JointIndex] = 0;
				TransitionCandidateVel[Tf][t][JointIndex] = 0;
				TransitionCandidateAcc[Tf][t][JointIndex] = 0;
			}
			error_M12[Tf][JointIndex] = 0;
			TransitionCandidateVelMax[Tf][JointIndex] = 0;
			TransitionCandidateAccMax[Tf][JointIndex] = 0;
			Q[Tf][JointIndex] = 0;
		}
		Q_MIN[JointIndex] = 1000000;
		*/
		OptimalTf[JointIndex] = 0;
	}
	//*/
	for(JointIndex = 0;JointIndex<NO_OF_JOINT_MC;JointIndex++)
	{	
		for (Tf=0;Tf<TfLimit;Tf++)
		{
			///*
			a[JointIndex]=(float)(2./(Tf*Tf*Tf)*MotionCapture[M1][Ti+30][JointIndex]+1./(Tf*Tf)*MotionCaptureVel[M1][Ti+30][JointIndex]-2./(Tf*Tf*Tf)*MotionCapture[M2][Tf][JointIndex]+1./(Tf*Tf)*MotionCaptureVel[M2][Tf][JointIndex]);
			b[JointIndex]=(float)(-3./(Tf*Tf)*MotionCapture[M1][Ti+30][JointIndex]-2./Tf*MotionCaptureVel[M1][Ti+30][JointIndex]+3./(Tf*Tf)*MotionCapture[M2][Tf][JointIndex]-1./Tf*MotionCaptureVel[M2][Tf][JointIndex]);
			c[JointIndex]=(float)(MotionCaptureVel[M1][Ti+30][JointIndex]);
			d[JointIndex]=(float)(MotionCapture[M1][Ti+30][JointIndex]);
			//*/
			/*
			a[JointIndex] = (float)(-6./pow(Tf,5)*MotionCapture[M1][Ti+30][JointIndex]
							-3./pow(Tf,4)*MotionCaptureVel[M1][Ti+30][JointIndex]
							-1./2./pow(Tf,3)*MotionCaptureAcc[M1][Ti+30][JointIndex]
							+6./pow(Tf,5)*MotionCapture[M2][Tf][JointIndex]
							-3./pow(Tf,4)*MotionCaptureVel[M2][Tf][JointIndex]
							+1./2./pow(Tf,3)*MotionCaptureAcc[M2][Tf][JointIndex]);		
			b[JointIndex] = (float)(15./pow(Tf,4)*MotionCapture[M1][Ti+30][JointIndex]
							+8./pow(Tf,3)*MotionCaptureVel[M1][Ti+30][JointIndex]
							+3./2./pow(Tf,2)*MotionCaptureAcc[M1][Ti+30][JointIndex]
							-15./pow(Tf,4)*MotionCapture[M2][Tf][JointIndex]
							+7./pow(Tf,3)*MotionCaptureVel[M2][Tf][JointIndex]
							-1./pow(Tf,2)*MotionCaptureAcc[M2][Tf][JointIndex]);
			c[JointIndex] = (float)(-10./pow(Tf,3)*MotionCapture[M1][Ti+30][JointIndex]
							-6./pow(Tf,2)*MotionCaptureVel[M1][Ti+30][JointIndex]
							-3./2./Tf*MotionCaptureAcc[M1][Ti+30][JointIndex]
							+10./pow(Tf,3)*MotionCapture[M2][Tf][JointIndex]
							-4./pow(Tf,2)*MotionCaptureVel[M2][Tf][JointIndex]
							+1./2./Tf*MotionCaptureAcc[M2][Tf][JointIndex]);
			d[JointIndex] = (float)(1./2.*MotionCaptureAcc[M1][Ti+30][JointIndex]);
			e[JointIndex] = (float)(1.*MotionCaptureVel[M1][Ti+30][JointIndex]);
			f[JointIndex] = (float)(MotionCapture[M1][Ti+30][JointIndex]);
			*/
			
			for(t=0;t < Tf;t++)
			{
				
				TransitionCandidate[Tf][t][JointIndex] = a[JointIndex]*t*t*t+b[JointIndex]*t*t+c[JointIndex]*t+d[JointIndex];		
				//TransitionCandidate[Tf][t][JointIndex] = a[JointIndex]*t*t*t*t*t+b[JointIndex]*t*t*t*t+c[JointIndex]*t*t*t+d[JointIndex]*t*t+e[JointIndex]*t+f[JointIndex];		
				if(t>=1)
				{
					TransitionCandidateVel[Tf][t-1][JointIndex] = (float)(fabs(TransitionCandidate[Tf][t][JointIndex] - TransitionCandidate[Tf][t-1][JointIndex]));
					if(TransitionCandidateVelMax[Tf][JointIndex] < TransitionCandidateVel[Tf][t-1][JointIndex])
					TransitionCandidateVelMax[Tf][JointIndex] = TransitionCandidateVel[Tf][t-1][JointIndex];
				}
				if(t>=2)
				{
					TransitionCandidateAcc[Tf][t-2][JointIndex] = (float)(fabs(TransitionCandidateVel[Tf][t-1][JointIndex] - TransitionCandidateVel[Tf][t-2][JointIndex]));
					if(TransitionCandidateAccMax[Tf][JointIndex] < TransitionCandidateAcc[Tf][t-2][JointIndex])
					TransitionCandidateAccMax[Tf][JointIndex] = TransitionCandidateAcc[Tf][t-1][JointIndex];
				}
				
				
				error_M12[Tf][JointIndex] = (float)(error_M12[Tf][JointIndex]+fabs(TransitionCandidate[Tf][t][JointIndex]-MotionCapture[M2][t][JointIndex])/(t+1));
			}

			Q[Tf][JointIndex]=(float)(0.1*error_M12[Tf][JointIndex]+40.0*TransitionCandidateVelMax[Tf][JointIndex]+0.0*TransitionCandidateAccMax[Tf][JointIndex]);

			if(Q_MIN[JointIndex] > Q[Tf][JointIndex] && Tf > 60)
			{
				Q_MIN[JointIndex] = Q[Tf][JointIndex];				
				OptimalTf[JointIndex] = Tf;
			}
			
			Q[Tf][JointIndex]=0;
			TransitionCandidateVelMax[Tf][JointIndex]=0;
			TransitionCandidateAccMax[Tf][JointIndex]=0;
			error_M12[Tf][JointIndex] = 0;
		}
		Q_MIN[JointIndex]=100000000;
		if(OptimalTfMax<OptimalTf[JointIndex])
		{
			OptimalTfMax=OptimalTf[JointIndex];
		}

		if(Ti > OptimalTfMax)
		{
		///*
			a[JointIndex]=(float)(2./pow(OptimalTf[JointIndex],3)*MotionCapture[M1][Ti+30][JointIndex]
				+1./pow(OptimalTf[JointIndex],2)*MotionCaptureVel[M1][Ti+30][JointIndex]
				-2./pow(OptimalTf[JointIndex],3)*MotionCapture[M2][OptimalTf[JointIndex]][JointIndex]
				+1./pow(OptimalTf[JointIndex],2)*MotionCaptureVel[M2][OptimalTf[JointIndex]][JointIndex]);
			b[JointIndex]=(float)(-3./pow(OptimalTf[JointIndex],2)*MotionCapture[M1][Ti+30][JointIndex]
				-2./OptimalTf[JointIndex]*MotionCaptureVel[M1][Ti+30][JointIndex]
				+3./pow(OptimalTf[JointIndex],2)*MotionCapture[M2][OptimalTf[JointIndex]][JointIndex]
				-1./OptimalTf[JointIndex]*MotionCaptureVel[M2][OptimalTf[JointIndex]][JointIndex]);
			c[JointIndex]=MotionCaptureVel[M1][Ti+30][JointIndex];
			d[JointIndex]=MotionCapture[M1][Ti+30][JointIndex];
		//*/
		/*
		a[JointIndex] = (float)(-6./pow(OptimalTf[JointIndex],5)*MotionCapture[M1][Ti+30][JointIndex]
			-3./pow(OptimalTf[JointIndex],4)*MotionCaptureVel[M1][Ti+30][JointIndex]
			-1./2./pow(OptimalTf[JointIndex],3)*MotionCaptureAcc[M1][Ti+30][JointIndex]
			+6./pow(OptimalTf[JointIndex],5)*MotionCapture[M2][OptimalTf[JointIndex]][JointIndex]
			-3./pow(OptimalTf[JointIndex],4)*MotionCaptureVel[M2][OptimalTf[JointIndex]][JointIndex]
			+1./2./pow(OptimalTf[JointIndex],3)*MotionCaptureAcc[M2][OptimalTf[JointIndex]][JointIndex]);
		b[JointIndex] = (float)(15./pow(OptimalTf[JointIndex],4)*MotionCapture[M1][Ti+30][JointIndex]
			+8./pow(OptimalTf[JointIndex],3)*MotionCaptureVel[M1][Ti+30][JointIndex]
			+3./2./pow(OptimalTf[JointIndex],2)*MotionCaptureAcc[M1][Ti+30][JointIndex]
			-15./pow(OptimalTf[JointIndex],4)*MotionCapture[M2][OptimalTf[JointIndex]][JointIndex]
			+7./pow(OptimalTf[JointIndex],3)*MotionCaptureVel[M2][OptimalTf[JointIndex]][JointIndex]
			-1./pow(OptimalTf[JointIndex],2)*MotionCaptureAcc[M2][OptimalTf[JointIndex]][JointIndex]);
		c[JointIndex] = (float)(-10./pow(OptimalTf[JointIndex],3)*MotionCapture[M1][Ti+30][JointIndex]
			-6./pow(OptimalTf[JointIndex],2)*MotionCaptureVel[M1][Ti+30][JointIndex]
			-3./2./OptimalTf[JointIndex]*MotionCaptureAcc[M1][Ti+30][JointIndex]
			+10./pow(OptimalTf[JointIndex],3)*MotionCapture[M2][OptimalTf[JointIndex]][JointIndex]
			-4./pow(OptimalTf[JointIndex],2)*MotionCaptureVel[M2][OptimalTf[JointIndex]][JointIndex]
			+1./2./OptimalTf[JointIndex]*MotionCaptureAcc[M2][OptimalTf[JointIndex]][JointIndex]);
		d[JointIndex] = (float)(1./2.*MotionCaptureAcc[M1][Ti+30][JointIndex]);
		e[JointIndex] = (float)(1.*MotionCaptureVel[M1][Ti+30][JointIndex]);
		f[JointIndex] = (float)(MotionCapture[M1][Ti+30][JointIndex]);
		*/
		
			for(t=0;t<OptimalTf[JointIndex];t++)
			{
				MotionCapture[M2][t][JointIndex] = a[JointIndex]*t*t*t+b[JointIndex]*t*t+c[JointIndex]*t+d[JointIndex];
			//	MotionCapture[M2][t][JointIndex] = a[JointIndex]*t*t*t*t*t+b[JointIndex]*t*t*t*t+c[JointIndex]*t*t*t+d[JointIndex]*t*t+e[JointIndex]*t+f[JointIndex];		
				if(t>=1)
				MotionCaptureVel[M2][t-1][JointIndex] = MotionCapture[M2][t][JointIndex] - MotionCapture[M2][t-1][JointIndex];
				if(t>=2)
				MotionCaptureAcc[M2][t-2][JointIndex] = MotionCaptureVel[M2][t-1][JointIndex] - MotionCaptureVel[M2][t-2][JointIndex];
			}
		}
	}	

	if(Ti > OptimalTfMax) 
	{
		jw_flag = 10;
		TransitionFlag = 1;
		MotionSetFlag = 1;
	}
	else 
	{
		TransitionFlag = 0;
		jw_flag = 20;
	}


}
/******************************************************************************/






/******************************************************************************/

void KinematicValueUpdate(int _MotionNo)
{
	unsigned int t , j;
	for(j= 0 ; j<NO_OF_JOINT_MC; j++ )
	{
		for(t=0;t<MotionPeriod[_MotionNo];t++)
		{
			if(t>=1)
			MotionCaptureVel[_MotionNo][t-1][j] = MotionCapture[_MotionNo][t][j] - MotionCapture[_MotionNo][t-1][j]; 
			if(t>=2)
			MotionCaptureAcc[_MotionNo][t-2][j] = MotionCaptureVel[_MotionNo][t-1][j] - MotionCaptureVel[_MotionNo][t-2][j];
		}
	}
	
}
/*****************************************************************************/







/******************************************************************************/
void SetMotionCapture0(void) //Speach1
{
	FILE	*ifp;
	int		JointIndex;
	int		n;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][22];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][22];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f, -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 0;

	NoOfCoeff = 21; //should be changed
		
	MotionPeriod[MotionNo] = 930;// dim : 5 msec.		// T must be 3 times integer
		
		
	ifp = fopen("C:\\Rainbow\\MotionCapture\\taejin_6_1.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if (JointIndex<16)
		{
			for(n=0; n < NoOfCoeff+1; n++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][n]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][n]);
			}
		}
		else
		{
			for(n=0; n < NoOfCoeff; n++)
			{
				CosCoeff[JointIndex][n] = 0.0f;
				SinCoeff[JointIndex][n] = 0.0f;
			}
		}		
	}
	fclose(ifp);

	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(n=0; n < NoOfCoeff; n++)
			{
				if(n==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][n]*cos(2*(n)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][n]*sin(2*(n)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					if(JointIndex<16)
					{
						MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					}
					//PROF_RFingerMotion1(100, 250, t+j-3, MotionNo);
					PROF_RFingerMotion2(30, 120, t+j-3, MotionNo);

					PROF_RFingerMotion2(200, 450, t+j-3, MotionNo);
					PROF_LFingerMotion2(200, 450, t+j-3, MotionNo);

					PROF_LFingerMotion5(450, 650, t+j-3, MotionNo);

					PROF_RFingerMotion2(600, 700, t+j-3, MotionNo);

					PROF_RFingerMotion2(750, 850, t+j-3, MotionNo);
					PROF_LFingerMotion2(650, 850, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture1(void) //Speach2
{
	FILE *ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][23];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][23];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;

	//MotionSetFlag = 0;
	int MotionNo = 1;

	//jw_flag = 0;
	NoOfCoeff = 22; //should be changed
		
			//
	MotionPeriod[MotionNo] = 930;//2220;// dim : 5 msec.		// T must be 3 times integer
		
		
	ifp = fopen("C:\\Rainbow\\MotionCapture\\taejin_5_1.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex <16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	//jw_flag = 10;
	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion2(50, 200, t+j-3, MotionNo);

					PROF_RFingerMotion2(200, 300, t+j-3, MotionNo);
					PROF_LFingerMotion2(200, 300, t+j-3, MotionNo);
					
					PROF_RFingerMotion2(350, 450, t+j-3, MotionNo);
					PROF_LFingerMotion2(350, 450, t+j-3, MotionNo);

					PROF_RFingerMotion2(550, 650, t+j-3, MotionNo);
					PROF_LFingerMotion2(450, 600, t+j-3, MotionNo);

					PROF_RFingerMotion2(700, 850, t+j-3, MotionNo);

					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}

}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture2(void) //Sign Language
{
	int t =0; 
	int MotionNo = 2;
	int T;

	MotionPeriod[MotionNo] = 1080;// dim : 5 msec.		// T must be 3 times integer
	T = MotionPeriod[MotionNo];
		
	for(t=0 ; t < T ; t++)
	{ 
		PROF_RFingerMotion2(33, 151, t, MotionNo);
		PROF_LFingerMotion2(33, 151, t, MotionNo);

		PROF_RFingerMotion1(152, 280, t, MotionNo);
		PROF_LFingerMotion1(152, 280, t, MotionNo);

		PROF_RFingerMotion1(281, 420, t, MotionNo);
		PROF_LFingerMotion4(281, 420, t, MotionNo);

		PROF_RFingerMotion5(421, 726, t, MotionNo);
		PROF_LFingerMotion5(421, 726, t, MotionNo);

		PROF_RFingerMotion2(727, 1029, t, MotionNo);
		PROF_LFingerMotion2(727, 1029, t, MotionNo);
		MotionCapture[MotionNo][t][0] = (float)-(1.0*(0.01459+0.19317*cos(1*2*3.1415926536/T*t)-0.33252*sin(1*2*3.1415926536/T*t)+0.12461*cos(2*2*3.1415926536/T*t)+0.86735*sin(2*2*3.1415926536/T*t)-0.53675*cos(3*2*3.1415926536/T*t)-0.65848*sin(3*2*3.1415926536/T*t)+0.59734*cos(4*2*3.1415926536/T*t)-0.20580*sin(4*2*3.1415926536/T*t)-0.53853*cos(5*2*3.1415926536/T*t)+0.21083*sin(5*2*3.1415926536/T*t)+0.14495*cos(6*2*3.1415926536/T*t)-0.55091*sin(6*2*3.1415926536/T*t)+0.02945*cos(7*2*3.1415926536/T*t)+0.09739*sin(7*2*3.1415926536/T*t)-0.20674*cos(8*2*3.1415926536/T*t)-0.08358*sin(8*2*3.1415926536/T*t)+0.04206*cos(9*2*3.1415926536/T*t)-0.29507*sin(9*2*3.1415926536/T*t)-0.01511*cos(10*2*3.1415926536/T*t)-0.08828*sin(10*2*3.1415926536/T*t)+0.01114*cos(11*2*3.1415926536/T*t)+0.13896*sin(11*2*3.1415926536/T*t)+0.08292*cos(12*2*3.1415926536/T*t)+0.01731*sin(12*2*3.1415926536/T*t)+0.26415*cos(13*2*3.1415926536/T*t)-0.25936*sin(13*2*3.1415926536/T*t)-0.15390*cos(14*2*3.1415926536/T*t)+0.16082*sin(14*2*3.1415926536/T*t)-0.05337*cos(15*2*3.1415926536/T*t)+0.09202*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][1] = (float)-0.;
		MotionCapture[MotionNo][t][2] = (float)0.;
		MotionCapture[MotionNo][t][3] = (float)-(1.0*(52.42048-14.77716*cos(1*2*3.1415926536/T*t)-0.27491*sin(1*2*3.1415926536/T*t)-8.24491*cos(2*2*3.1415926536/T*t)-4.40976*sin(2*2*3.1415926536/T*t)-2.22229*cos(3*2*3.1415926536/T*t)+9.96050*sin(3*2*3.1415926536/T*t)-6.11357*cos(4*2*3.1415926536/T*t)-3.26441*sin(4*2*3.1415926536/T*t)-6.19268*cos(5*2*3.1415926536/T*t)+4.05147*sin(5*2*3.1415926536/T*t)+3.85722*cos(6*2*3.1415926536/T*t)+1.43477*sin(6*2*3.1415926536/T*t)-7.69654*cos(7*2*3.1415926536/T*t)+0.22663*sin(7*2*3.1415926536/T*t)+0.42973*cos(8*2*3.1415926536/T*t)-0.36342*sin(8*2*3.1415926536/T*t)-1.77698*cos(9*2*3.1415926536/T*t)-0.89126*sin(9*2*3.1415926536/T*t)-3.20521*cos(10*2*3.1415926536/T*t)-0.33271*sin(10*2*3.1415926536/T*t)-2.24534*cos(11*2*3.1415926536/T*t)+0.32785*sin(11*2*3.1415926536/T*t)-0.53319*cos(12*2*3.1415926536/T*t)+2.02711*sin(12*2*3.1415926536/T*t)+0.20589*cos(13*2*3.1415926536/T*t)-0.19368*sin(13*2*3.1415926536/T*t)-1.65484*cos(14*2*3.1415926536/T*t)-1.75469*sin(14*2*3.1415926536/T*t)-2.25061*cos(15*2*3.1415926536/T*t)-0.50211*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][4] = (float)-(1.0*(-17.13958+5.41798*cos(1*2*3.1415926536/T*t)+4.26447*sin(1*2*3.1415926536/T*t)+3.78580*cos(2*2*3.1415926536/T*t)+4.74588*sin(2*2*3.1415926536/T*t)+1.04780*cos(3*2*3.1415926536/T*t)-2.06160*sin(3*2*3.1415926536/T*t)+2.51821*cos(4*2*3.1415926536/T*t)-0.19955*sin(4*2*3.1415926536/T*t)-1.55012*cos(5*2*3.1415926536/T*t)-1.16044*sin(5*2*3.1415926536/T*t)-0.39319*cos(6*2*3.1415926536/T*t)-0.16738*sin(6*2*3.1415926536/T*t)+1.28034*cos(7*2*3.1415926536/T*t)-2.91777*sin(7*2*3.1415926536/T*t)+0.68446*cos(8*2*3.1415926536/T*t)-1.32314*sin(8*2*3.1415926536/T*t)+1.28991*cos(9*2*3.1415926536/T*t)+1.44549*sin(9*2*3.1415926536/T*t)+0.31708*cos(10*2*3.1415926536/T*t)-1.41642*sin(10*2*3.1415926536/T*t)+0.28386*cos(11*2*3.1415926536/T*t)+1.89437*sin(11*2*3.1415926536/T*t)+0.81120*cos(12*2*3.1415926536/T*t)-0.22995*sin(12*2*3.1415926536/T*t)+0.47046*cos(13*2*3.1415926536/T*t)-1.08273*sin(13*2*3.1415926536/T*t)+0.90027*cos(14*2*3.1415926536/T*t)+0.19679*sin(14*2*3.1415926536/T*t)+0.27552*cos(15*2*3.1415926536/T*t)+0.17818*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][5] = (float)-(1.0*(20.51788+0.29483*cos(1*2*3.1415926536/T*t)+0.06215*sin(1*2*3.1415926536/T*t)-6.49740*cos(2*2*3.1415926536/T*t)-3.16817*sin(2*2*3.1415926536/T*t)-5.58718*cos(3*2*3.1415926536/T*t)+5.98777*sin(3*2*3.1415926536/T*t)-1.50911*cos(4*2*3.1415926536/T*t)+1.37597*sin(4*2*3.1415926536/T*t)-6.28597*cos(5*2*3.1415926536/T*t)-1.29256*sin(5*2*3.1415926536/T*t)+0.70871*cos(6*2*3.1415926536/T*t)+3.17321*sin(6*2*3.1415926536/T*t)-4.13122*cos(7*2*3.1415926536/T*t)-5.50193*sin(7*2*3.1415926536/T*t)+1.09922*cos(8*2*3.1415926536/T*t)-1.50594*sin(8*2*3.1415926536/T*t)+3.33899*cos(9*2*3.1415926536/T*t)+3.08845*sin(9*2*3.1415926536/T*t)+0.69771*cos(10*2*3.1415926536/T*t)-3.67547*sin(10*2*3.1415926536/T*t)+0.33752*cos(11*2*3.1415926536/T*t)-0.88465*sin(11*2*3.1415926536/T*t)-4.04519*cos(12*2*3.1415926536/T*t)-1.87045*sin(12*2*3.1415926536/T*t)-2.65424*cos(13*2*3.1415926536/T*t)+1.78909*sin(13*2*3.1415926536/T*t)+1.09698*cos(14*2*3.1415926536/T*t)+1.83653*sin(14*2*3.1415926536/T*t)+2.61847*cos(15*2*3.1415926536/T*t)+0.29875*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][6] = (float)-(0.9*(47.03257-8.32549*cos(1*2*3.1415926536/T*t)-8.40287*sin(1*2*3.1415926536/T*t)-12.67584*cos(2*2*3.1415926536/T*t)-13.46296*sin(2*2*3.1415926536/T*t)-8.52232*cos(3*2*3.1415926536/T*t)+3.11797*sin(3*2*3.1415926536/T*t)-5.94568*cos(4*2*3.1415926536/T*t)-7.92439*sin(4*2*3.1415926536/T*t)-7.47225*cos(5*2*3.1415926536/T*t)-1.62522*sin(5*2*3.1415926536/T*t)+5.43119*cos(6*2*3.1415926536/T*t)-2.65534*sin(6*2*3.1415926536/T*t)-5.92960*cos(7*2*3.1415926536/T*t)+2.89742*sin(7*2*3.1415926536/T*t)-0.94423*cos(8*2*3.1415926536/T*t)+2.16083*sin(8*2*3.1415926536/T*t)-3.54262*cos(9*2*3.1415926536/T*t)-5.76285*sin(9*2*3.1415926536/T*t)-1.26978*cos(10*2*3.1415926536/T*t)+1.11205*sin(10*2*3.1415926536/T*t)+0.16006*cos(11*2*3.1415926536/T*t)-0.23855*sin(11*2*3.1415926536/T*t)+2.87645*cos(12*2*3.1415926536/T*t)+3.51522*sin(12*2*3.1415926536/T*t)+4.47132*cos(13*2*3.1415926536/T*t)-0.45633*sin(13*2*3.1415926536/T*t)-3.14193*cos(14*2*3.1415926536/T*t)-0.76232*sin(14*2*3.1415926536/T*t)-2.20183*cos(15*2*3.1415926536/T*t)-1.32685*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][7] = (float)-(0.5*(-12.47112-27.15057*cos(1*2*3.1415926536/T*t)+20.23762*sin(1*2*3.1415926536/T*t)-8.86864*cos(2*2*3.1415926536/T*t)-9.61629*sin(2*2*3.1415926536/T*t)+13.68356*cos(3*2*3.1415926536/T*t)-29.04642*sin(3*2*3.1415926536/T*t)+12.52147*cos(4*2*3.1415926536/T*t)-16.47691*sin(4*2*3.1415926536/T*t)+11.47096*cos(5*2*3.1415926536/T*t)-2.77302*sin(5*2*3.1415926536/T*t)+8.36439*cos(6*2*3.1415926536/T*t)+5.60558*sin(6*2*3.1415926536/T*t)-0.70774*cos(7*2*3.1415926536/T*t)+0.59176*sin(7*2*3.1415926536/T*t)-5.15710*cos(8*2*3.1415926536/T*t)-0.42620*sin(8*2*3.1415926536/T*t)-1.84009*cos(9*2*3.1415926536/T*t)+0.56546*sin(9*2*3.1415926536/T*t)+3.34522*cos(10*2*3.1415926536/T*t)-3.26821*sin(10*2*3.1415926536/T*t)+4.56892*cos(11*2*3.1415926536/T*t)-1.22414*sin(11*2*3.1415926536/T*t)+1.19679*cos(12*2*3.1415926536/T*t)+0.68394*sin(12*2*3.1415926536/T*t)+1.03284*cos(13*2*3.1415926536/T*t)+4.35796*sin(13*2*3.1415926536/T*t)-0.31364*cos(14*2*3.1415926536/T*t)+1.83963*sin(14*2*3.1415926536/T*t)+0.32474*cos(15*2*3.1415926536/T*t)-1.46278*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][8] = (float)(0.5*(-7.82932-2.27893*cos(1*2*3.1415926536/T*t)+0.33672*sin(1*2*3.1415926536/T*t)+1.69282*cos(2*2*3.1415926536/T*t)-0.95074*sin(2*2*3.1415926536/T*t)+2.43378*cos(3*2*3.1415926536/T*t)-2.22498*sin(3*2*3.1415926536/T*t)+1.15472*cos(4*2*3.1415926536/T*t)-1.72090*sin(4*2*3.1415926536/T*t)+1.27375*cos(5*2*3.1415926536/T*t)-0.60674*sin(5*2*3.1415926536/T*t)+1.17629*cos(6*2*3.1415926536/T*t)-0.08102*sin(6*2*3.1415926536/T*t)+0.09067*cos(7*2*3.1415926536/T*t)-0.56887*sin(7*2*3.1415926536/T*t)-0.08346*cos(8*2*3.1415926536/T*t)-0.29124*sin(8*2*3.1415926536/T*t)+0.33904*cos(9*2*3.1415926536/T*t)+0.25809*sin(9*2*3.1415926536/T*t)+0.39359*cos(10*2*3.1415926536/T*t)-0.25755*sin(10*2*3.1415926536/T*t)+0.49967*cos(11*2*3.1415926536/T*t)+0.14719*sin(11*2*3.1415926536/T*t)+0.40188*cos(12*2*3.1415926536/T*t)+0.45875*sin(12*2*3.1415926536/T*t)+0.55680*cos(13*2*3.1415926536/T*t)+0.07203*sin(13*2*3.1415926536/T*t)+0.27453*cos(14*2*3.1415926536/T*t)-0.14678*sin(14*2*3.1415926536/T*t)-0.09583*cos(15*2*3.1415926536/T*t)-0.13928*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][9] = (float)-(1.0*(46.21469+1.97112*cos(1*2*3.1415926536/T*t)-3.67580*sin(1*2*3.1415926536/T*t)-2.50654*cos(2*2*3.1415926536/T*t)+15.94167*sin(2*2*3.1415926536/T*t)-11.77427*cos(3*2*3.1415926536/T*t)+8.92226*sin(3*2*3.1415926536/T*t)+1.75016*cos(4*2*3.1415926536/T*t)-4.56626*sin(4*2*3.1415926536/T*t)-13.30535*cos(5*2*3.1415926536/T*t)+13.50937*sin(5*2*3.1415926536/T*t)+0.48987*cos(6*2*3.1415926536/T*t)-1.71222*sin(6*2*3.1415926536/T*t)-5.47172*cos(7*2*3.1415926536/T*t)+0.33531*sin(7*2*3.1415926536/T*t)-3.26382*cos(8*2*3.1415926536/T*t)+1.66818*sin(8*2*3.1415926536/T*t)-2.46420*cos(9*2*3.1415926536/T*t)+2.18946*sin(9*2*3.1415926536/T*t)-2.67993*cos(10*2*3.1415926536/T*t)-0.59222*sin(10*2*3.1415926536/T*t)-0.30207*cos(11*2*3.1415926536/T*t)-2.12193*sin(11*2*3.1415926536/T*t)-4.52572*cos(12*2*3.1415926536/T*t)+0.03794*sin(12*2*3.1415926536/T*t)-3.30338*cos(13*2*3.1415926536/T*t)-0.40777*sin(13*2*3.1415926536/T*t)+0.43012*cos(14*2*3.1415926536/T*t)+1.30013*sin(14*2*3.1415926536/T*t)-1.25896*cos(15*2*3.1415926536/T*t)-0.82083*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][10] = (float)-(1.0*(21.80232-4.14630*cos(1*2*3.1415926536/T*t)-0.47591*sin(1*2*3.1415926536/T*t)-1.95864*cos(2*2*3.1415926536/T*t)-2.19311*sin(2*2*3.1415926536/T*t)-0.78561*cos(3*2*3.1415926536/T*t)+5.24872*sin(3*2*3.1415926536/T*t)-5.26517*cos(4*2*3.1415926536/T*t)+1.11107*sin(4*2*3.1415926536/T*t)+0.96664*cos(5*2*3.1415926536/T*t)+2.27814*sin(5*2*3.1415926536/T*t)-0.55652*cos(6*2*3.1415926536/T*t)+0.91928*sin(6*2*3.1415926536/T*t)-1.52870*cos(7*2*3.1415926536/T*t)+2.76615*sin(7*2*3.1415926536/T*t)-0.48181*cos(8*2*3.1415926536/T*t)+1.43426*sin(8*2*3.1415926536/T*t)-2.71110*cos(9*2*3.1415926536/T*t)-1.47516*sin(9*2*3.1415926536/T*t)-1.83585*cos(10*2*3.1415926536/T*t)+1.66837*sin(10*2*3.1415926536/T*t)-0.31257*cos(11*2*3.1415926536/T*t)-1.22002*sin(11*2*3.1415926536/T*t)-1.01783*cos(12*2*3.1415926536/T*t)+0.79701*sin(12*2*3.1415926536/T*t)-0.14424*cos(13*2*3.1415926536/T*t)-0.37491*sin(13*2*3.1415926536/T*t)-0.50443*cos(14*2*3.1415926536/T*t)-0.15931*sin(14*2*3.1415926536/T*t)-1.52019*cos(15*2*3.1415926536/T*t)+0.11385*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][11] = (float)(1.0*(7.42599+12.17476*cos(1*2*3.1415926536/T*t)+2.87503*sin(1*2*3.1415926536/T*t)-7.05197*cos(2*2*3.1415926536/T*t)+5.20848*sin(2*2*3.1415926536/T*t)-8.98446*cos(3*2*3.1415926536/T*t)+1.88419*sin(3*2*3.1415926536/T*t)+7.36129*cos(4*2*3.1415926536/T*t)+6.19096*sin(4*2*3.1415926536/T*t)-13.71225*cos(5*2*3.1415926536/T*t)+3.92000*sin(5*2*3.1415926536/T*t)+4.91272*cos(6*2*3.1415926536/T*t)-1.15169*sin(6*2*3.1415926536/T*t)+0.50264*cos(7*2*3.1415926536/T*t)-3.31599*sin(7*2*3.1415926536/T*t)-1.69857*cos(8*2*3.1415926536/T*t)-4.51392*sin(8*2*3.1415926536/T*t)+4.08341*cos(9*2*3.1415926536/T*t)-2.95274*sin(9*2*3.1415926536/T*t)-4.49029*cos(10*2*3.1415926536/T*t)-2.09945*sin(10*2*3.1415926536/T*t)-3.37446*cos(11*2*3.1415926536/T*t)+6.44613*sin(11*2*3.1415926536/T*t)+1.02358*cos(12*2*3.1415926536/T*t)+0.17185*sin(12*2*3.1415926536/T*t)+4.53605*cos(13*2*3.1415926536/T*t)-1.27008*sin(13*2*3.1415926536/T*t)-0.76025*cos(14*2*3.1415926536/T*t)-1.39512*sin(14*2*3.1415926536/T*t)-1.94820*cos(15*2*3.1415926536/T*t)-1.61587*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][12] = (float)-(0.9*(43.93383+8.40254*cos(1*2*3.1415926536/T*t)-9.01301*sin(1*2*3.1415926536/T*t)-5.76628*cos(2*2*3.1415926536/T*t)+5.85990*sin(2*2*3.1415926536/T*t)-14.03558*cos(3*2*3.1415926536/T*t)+6.57783*sin(3*2*3.1415926536/T*t)+0.53582*cos(4*2*3.1415926536/T*t)-7.74823*sin(4*2*3.1415926536/T*t)-15.54849*cos(5*2*3.1415926536/T*t)+5.85044*sin(5*2*3.1415926536/T*t)+1.22584*cos(6*2*3.1415926536/T*t)-3.90839*sin(6*2*3.1415926536/T*t)-5.62729*cos(7*2*3.1415926536/T*t)+3.46387*sin(7*2*3.1415926536/T*t)-6.21032*cos(8*2*3.1415926536/T*t)+6.14777*sin(8*2*3.1415926536/T*t)-3.48649*cos(9*2*3.1415926536/T*t)-0.96189*sin(9*2*3.1415926536/T*t)+1.23852*cos(10*2*3.1415926536/T*t)+2.33153*sin(10*2*3.1415926536/T*t)+1.76022*cos(11*2*3.1415926536/T*t)-4.14099*sin(11*2*3.1415926536/T*t)-3.29508*cos(12*2*3.1415926536/T*t)-1.07375*sin(12*2*3.1415926536/T*t)-2.15535*cos(13*2*3.1415926536/T*t)+0.48573*sin(13*2*3.1415926536/T*t)-1.54443*cos(14*2*3.1415926536/T*t)+3.25374*sin(14*2*3.1415926536/T*t)+0.57255*cos(15*2*3.1415926536/T*t)-1.33630*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][13] = (float)-(0.5*(1.22621-5.74089*cos(1*2*3.1415926536/T*t)+26.86706*sin(1*2*3.1415926536/T*t)+1.02686*cos(2*2*3.1415926536/T*t)+10.23805*sin(2*2*3.1415926536/T*t)+8.10248*cos(3*2*3.1415926536/T*t)-5.83976*sin(3*2*3.1415926536/T*t)-0.35123*cos(4*2*3.1415926536/T*t)-0.97643*sin(4*2*3.1415926536/T*t)+2.40734*cos(5*2*3.1415926536/T*t)-1.44325*sin(5*2*3.1415926536/T*t)-0.11055*cos(6*2*3.1415926536/T*t)+2.92017*sin(6*2*3.1415926536/T*t)-1.63635*cos(7*2*3.1415926536/T*t)-4.43874*sin(7*2*3.1415926536/T*t)-1.27438*cos(8*2*3.1415926536/T*t)-3.16768*sin(8*2*3.1415926536/T*t)-1.30646*cos(9*2*3.1415926536/T*t)+1.41661*sin(9*2*3.1415926536/T*t)-1.84521*cos(10*2*3.1415926536/T*t)-2.54416*sin(10*2*3.1415926536/T*t)-1.87831*cos(11*2*3.1415926536/T*t)+4.00005*sin(11*2*3.1415926536/T*t)+0.62085*cos(12*2*3.1415926536/T*t)+2.45383*sin(12*2*3.1415926536/T*t)+2.54555*cos(13*2*3.1415926536/T*t)+0.17265*sin(13*2*3.1415926536/T*t)-0.49510*cos(14*2*3.1415926536/T*t)-2.29962*sin(14*2*3.1415926536/T*t)-1.29082*cos(15*2*3.1415926536/T*t)-0.88099*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][14] = (float)(0.5*(4.06122+0.04440*cos(1*2*3.1415926536/T*t)-2.91969*sin(1*2*3.1415926536/T*t)-2.28071*cos(2*2*3.1415926536/T*t)-2.03861*sin(2*2*3.1415926536/T*t)-1.83494*cos(3*2*3.1415926536/T*t)+0.66069*sin(3*2*3.1415926536/T*t)-0.17435*cos(4*2*3.1415926536/T*t)+0.56927*sin(4*2*3.1415926536/T*t)+0.02408*cos(5*2*3.1415926536/T*t)+0.29646*sin(5*2*3.1415926536/T*t)+0.05273*cos(6*2*3.1415926536/T*t)+0.43947*sin(6*2*3.1415926536/T*t)+0.20739*cos(7*2*3.1415926536/T*t)+1.00852*sin(7*2*3.1415926536/T*t)-0.00356*cos(8*2*3.1415926536/T*t)+0.52315*sin(8*2*3.1415926536/T*t)+0.21667*cos(9*2*3.1415926536/T*t)-0.45714*sin(9*2*3.1415926536/T*t)-0.06066*cos(10*2*3.1415926536/T*t)+0.10765*sin(10*2*3.1415926536/T*t)+0.06610*cos(11*2*3.1415926536/T*t)-0.33925*sin(11*2*3.1415926536/T*t)+0.21347*cos(12*2*3.1415926536/T*t)+0.00214*sin(12*2*3.1415926536/T*t)-0.35107*cos(13*2*3.1415926536/T*t)+0.14438*sin(13*2*3.1415926536/T*t)-0.12368*cos(14*2*3.1415926536/T*t)+0.13017*sin(14*2*3.1415926536/T*t)-0.05710*cos(15*2*3.1415926536/T*t)-0.20791*sin(15*2*3.1415926536/T*t)));
		MotionCapture[MotionNo][t][15] = (float)0.;
	}
	KinematicValueUpdate(MotionNo);
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture3(void) //Taichi
{
	float scale[NO_OF_JOINT_MC] = { -0.5f,  -0.8f,  0.8f, -1.0f,  -0.8f,  -0.8f,  -0.8f,  -0.5f,  0.5f, -0.9f, -0.7f, 0.8f,  -0.7f,  -0.5f, 0.5f, 0.8f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	Constant[NO_OF_JOINT_MC];
	float **CosCoeff;
	float **SinCoeff;
	float	sum;

	int		NoOfCoeff = 61;
	int		m;
	int		JointIndex;
	int		t;
	int		i, j;
	
	FILE *ifp;

	int MotionNo = 3;
	
	MotionPeriod[MotionNo] = 3300;

	CosCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	SinCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	for(i=0;i<NO_OF_JOINT_MC;i++)
	{
		CosCoeff[i]=(float *)malloc(sizeof(float)*NoOfCoeff);
		SinCoeff[i]=(float *)malloc(sizeof(float)*NoOfCoeff);
	}

	ifp = fopen("C:\\Rainbow\\MotionCapture\\taichi_short.txt", "r");
	fseek( ifp, 0L, SEEK_SET );

	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{	
		if(JointIndex<16)
		{
			for(m=0; m <= NoOfCoeff; m++)
			{
				if(m==0) fscanf(ifp,"%f",&Constant[JointIndex]);
				else	
				{
					fscanf(ifp,"%f",&CosCoeff[JointIndex][m-1]);
					fscanf(ifp,"%f",&SinCoeff[JointIndex][m-1]);
				}
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}

	fclose(ifp);

	for(t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0) sum = Constant[JointIndex];
				sum = (sum + CosCoeff[JointIndex][m]*cosf(2*(m+1)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sinf(2*(m+1)*PI/MotionPeriod[MotionNo]*t));
			}

			for(j=0;j<=3;j++)
			{
				if(t != 0) 
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					
					PROF_RFingerMotion2(int(5*0.3), int(1170*0.3), t+j-3, MotionNo);
					PROF_LFingerMotion2(int(5*0.3), int(1170*0.3), t+j-3, MotionNo);
					
					PROF_RFingerMotion1(int(1180*0.3), int(1740*0.3), t+j-3, MotionNo);
					PROF_LFingerMotion1(int(1180*0.3), int(1740*0.3), t+j-3, MotionNo);
					
					PROF_RFingerMotion2(int(1750*0.3), int(2860*0.3), t+j-3, MotionNo);
					PROF_LFingerMotion2(int(1750*0.3), int(2860*0.3), t+j-3, MotionNo);
					
					PROF_RFingerMotion6(int(3300*0.3), int(4100*0.3), t+j-3, MotionNo);
					
					PROF_RFingerMotion2(int(4680*0.3), int(6500*0.3), t+j-3, MotionNo);
					PROF_LFingerMotion2(int(4680*0.3), int(6500*0.3), t+j-3, MotionNo);
					
					PROF_RFingerMotion6(int(6500*0.3), int(7600*0.3), t+j-3, MotionNo);
					
					PROF_RFingerMotion2(int(8080*0.3), int(8470*0.3), t+j-3, MotionNo);
					PROF_LFingerMotion2(int(8080*0.3), int(8470*0.3), t+j-3, MotionNo);
					
					PROF_RFingerMotion1(int(8500*0.3), int(9300*0.3), t+j-3, MotionNo);
					PROF_LFingerMotion1(int(8500*0.3), int(9300*0.3), t+j-3, MotionNo);

					PROF_RFingerMotion2(int(10000*0.3),3200, t+j-3, MotionNo);
					PROF_LFingerMotion2(int(10000*0.3),3200, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
		if(t==360)	//must be larger than TfLimit
		{
			if(MotionSetFlag == 1)
			{
				Optimization(MotionNo);
				tempMotion = MotionNo;
			}
			else
			{
				MotionTimeCurrent = 0;
				Motion = MotionNo;
				MotionSetFlag = 1;
			}
		}
	}
	free(CosCoeff);
	free(SinCoeff);
}
/******************************************************************************/






/******************************************************************************/
void SetMotionCapture4(void) //Welcome
{
	// 0: -Torso yaw
	// 1: -Head yaw
	// 2: Head pitch\
	// 3: -LSP
	// 4: -LSY
	// 5: -LSR
	// 6: -LEB
	// 7: -LWR
	// 8: LWP
	// 9: -RSP
	// 10: -RSY
	// 11: RSR
	// 12: -REB
	// 13: -RWR
	// 14: RWP
	// 15: LRHP
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	Constant[NO_OF_JOINT_MC];
	float	CosCoeff[NO_OF_JOINT_MC][11];
	float	SinCoeff[NO_OF_JOINT_MC][11];
	float	scale[NO_OF_JOINT_MC] = { -0.8f,  -0.8f,  0.6f, -0.8f,  -0.8f,  -0.8f,  -0.8f,  -0.5f,  0.8f, -0.8f, -0.8f, 0.8f,  -0.8f,  -0.5f, 0.8f, 0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	float	tp_res[2];
	float	result1[2];
	float	result2[2];
	int MotionNo = 4;

	NoOfCoeff = 10; //should be changed
	
	MotionPeriod[MotionNo] = 1300;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\22coeff.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex < 16)
		{
			fscanf(ifp,"%f",&Constant[JointIndex]);
			for(m=0; m < NoOfCoeff; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
			}
			for(m=0; m < NoOfCoeff; m++)
			{
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			Constant[JointIndex] =0.0f;
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum = Constant[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			sum =scale[JointIndex]*sum;

			FTN_half_1_cos( 1.0,t,    0,100,0,0,&result1[0]);
			FTN_half_1_cos(-1.0,t,MotionPeriod[MotionNo]-100,100,0,0,&result2[0]);
			tp_res[0] = (float)(20.0*(result1[0]+result2[0]));	// shoulder pitch
			
			FTN_half_1_cos( 1.0,t,    0,50,0,0,&result1[1]);
			FTN_half_1_cos(-1.0,t,MotionPeriod[MotionNo]-100,100,0,0,&result2[1]);
			tp_res[1] = (float)(10.0*(result1[1]+result2[1]));	// shoulder roll
			
			if(JointIndex == 3 || JointIndex == 9)//-LSP // RSP
				sum = sum - tp_res[0];
			if(JointIndex == 4)//LSR
				sum = sum + tp_res[1];
			if(JointIndex == 10)//RSR
				sum = sum - tp_res[1];
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);			
					PROF_RFingerMotion2(335,1200,t+j-3,MotionNo);
					PROF_LFingerMotion2(335,1200,t+j-3,MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}

}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture5(void) //Goodbye
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	Constant[NO_OF_JOINT_MC];
	float	CosCoeff[NO_OF_JOINT_MC][17];
	float	SinCoeff[NO_OF_JOINT_MC][17];
	float	scale[NO_OF_JOINT_MC] = { 0.7f,  -1.0f,  1.0f, -1.0f,  -1.0f,  -1.0f,  -1.0f,  -0.5f,  1.0f, -1.0f,  -1.0f,  1.0f,  -1.5f,  -0.5f,  1.0f,  1.0f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	float	tp_res[2];
	float	result1[2];
	float	result2[2];

	int MotionNo = 5;

	NoOfCoeff = 16; //should be changed
	
	MotionPeriod[MotionNo] = 900;// dim : 5 msec.		// T must be 3 times integer
		
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Hand_wave_coeff.txt","r");
	fseek( ifp, 0L, SEEK_SET );

	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			fscanf(ifp,"%f",&Constant[JointIndex]);
			for(m=0; m < NoOfCoeff; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
			}
			for(m=0; m < NoOfCoeff; m++)
			{
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			Constant[JointIndex] = 0.0f;
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}

	fclose(ifp);

	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum = Constant[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			sum =scale[JointIndex]*sum;
			FTN_half_1_cos( 1.0,t,    0,100,0,0,&result1[0]);
			FTN_half_1_cos(-1.0,t,MotionPeriod[MotionNo]-100,100,0,0,&result2[0]);
			tp_res[0] = (float)(10.0*(result1[0]+result2[0]));	// shoulder pitch
			FTN_half_1_cos( 1.0,t,    0,100,0,0,&result1[1]);
			FTN_half_1_cos(-1.0,t,MotionPeriod[MotionNo]-100,100,0,0,&result2[1]);
			tp_res[1] = (float)(10.0*(result1[1]+result2[1]));	// shoulder roll
			
			if(JointIndex == 3) //LSP
				sum = sum - tp_res[0];
			if(JointIndex == 9) // RSP
				sum = sum - 11*tp_res[0];
			if(JointIndex == 4)//LSR
				sum = sum + tp_res[1];
			if(JointIndex == 10)//RSR
				sum = sum - tp_res[1];
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion2(100,MotionPeriod[MotionNo]-100,t+j-3,MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}

}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture6(void) //Dance1
{
	float scale[NO_OF_JOINT_MC] = { -0.8f,  -0.8f,  0.8f, -1.0f,  -0.8f,  -0.8f,  -0.8f,  -0.5f,  0.5f, -1.0f, -0.8f, 0.8f,  -0.8f,  -0.5f, 0.5f, 0.8f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	
	float	Constant[NO_OF_JOINT_MC];
	float **CosCoeff;
	float **SinCoeff;
	float	sum;

	int		NoOfCoeff = 56;
	int		m;
	int		JointIndex;
	int		t;
	int		i, j;
	
	FILE *ifp;

	//MotionSetFlag = 0;
	//MotionTimeCurrent =0;
	int MotionNo = 6;
	
	MotionPeriod[MotionNo] = 6000;

	CosCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	SinCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	for(i=0;i<NO_OF_JOINT_MC;i++)
	{
		CosCoeff[i]=(float *)malloc(sizeof(float)*56);
		SinCoeff[i]=(float *)malloc(sizeof(float)*56);
	}

	ifp = fopen("C:\\Rainbow\\MotionCapture\\1a.txt", "r");
	fseek( ifp, 0L, SEEK_SET );

	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{	
		if(JointIndex <16)
		{
			for(m=0; m <= NoOfCoeff; m++)
			{
				if(m==0) fscanf(ifp,"%f",&Constant[JointIndex]);
				else	
				{
					fscanf(ifp,"%f",&CosCoeff[JointIndex][m-1]);
					fscanf(ifp,"%f",&SinCoeff[JointIndex][m-1]);
				}
			}
		}
		else
		{
			Constant[JointIndex] = 0.0f;
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}

	fclose(ifp);

	for(t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0) sum = Constant[JointIndex];
				sum = (sum + CosCoeff[JointIndex][m]*cosf(2*(m+1)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sinf(2*(m+1)*PI/MotionPeriod[MotionNo]*t));
			}

			for(j=0;j<=3;j++)
			{
				if(t != 0) 
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
		if(t==TfLimit+10)	//must be larger than TfLimit
		{
			if(MotionSetFlag == 1)
			{
				Optimization(MotionNo);
				tempMotion = MotionNo;
			}
			else
			{
				MotionTimeCurrent = 0;
				Motion = MotionNo;
				MotionSetFlag = 1;
			}
		}
	}
	free(CosCoeff);
	free(SinCoeff);

}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture7(void) //Dance2
{
	float scale[NO_OF_JOINT_MC] = { -0.8f,  -0.8f,  0.8f, -0.8f,  -0.8f,  -0.5f,  -0.8f,  -0.5f,  0.8f, -0.8f, -0.8f, 0.5f,  -0.8f,  -0.5f, 0.8f, 0.8f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	
	float	Constant[NO_OF_JOINT_MC];
	float **CosCoeff;
	float **SinCoeff;
	float	sum;
	
	int		NoOfCoeff = 42;
	int		m;
	int		JointIndex;
	int		t;
	int		i, j;
	
	FILE *ifp;

	int MotionNo = 7;
	
	MotionPeriod[MotionNo] = 5001;
	
	CosCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	SinCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	for(i=0;i<NO_OF_JOINT_MC;i++)
	{
		CosCoeff[i]=(float *)malloc(sizeof(float)*42);
		SinCoeff[i]=(float *)malloc(sizeof(float)*42);
	}
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\1b.txt", "r");
	fseek( ifp, 0L, SEEK_SET );
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{	
		if(JointIndex<16)
		{
			for(m=0; m <= NoOfCoeff; m++)
			{
				if(m==0) fscanf(ifp,"%f",&Constant[JointIndex]);
				else	
				{
					fscanf(ifp,"%f",&CosCoeff[JointIndex][m-1]);
					fscanf(ifp,"%f",&SinCoeff[JointIndex][m-1]);
				}
			}
		}
		else
		{
			Constant[JointIndex] = 0.0f;
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	
	fclose(ifp);
	
	for(t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0) sum = Constant[JointIndex];
				sum = (sum + CosCoeff[JointIndex][m]*cosf(2*(m+1)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sinf(2*(m+1)*PI/MotionPeriod[MotionNo]*t));
			}
			
			for(j=0;j<=3;j++)
			{
				if(t != 0) 
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
		if(t==TfLimit+10)	//must be larger than TfLimit
		{
			if(MotionSetFlag == 1)
			{
				Optimization(MotionNo);
				tempMotion = MotionNo;
			}
			else
			{
				MotionTimeCurrent = 0;
				Motion = MotionNo;
				MotionSetFlag = 1;
			}
		}
	}
	free(CosCoeff);
	free(SinCoeff);
	
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture8(void) //Dance3
{
	float scale[NO_OF_JOINT_MC] = { -0.8f,  -0.8f,  0.8f, -0.8f,  -0.8f,  -0.5f,  -0.8f,  -0.5f,  0.8f, -0.8f, -0.8f, 0.5f,  -0.8f,  -0.5f, 0.8f, 0.3f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};	
	
	float	Constant[NO_OF_JOINT_MC];
	float **CosCoeff;
	float **SinCoeff;
	float	sum;
	
	int		NoOfCoeff = 44;
	int		m;
	int		JointIndex;
	int		t;
	int		i, j;
	
	FILE *ifp;
	
	int MotionNo = 8;
	
	MotionPeriod[MotionNo] = 5001;
	
	CosCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	SinCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	for(i=0;i<NO_OF_JOINT_MC;i++)
	{
		CosCoeff[i]=(float *)malloc(sizeof(float)*44);
		SinCoeff[i]=(float *)malloc(sizeof(float)*44);
	}
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\1c.txt", "r");
	fseek( ifp, 0L, SEEK_SET );
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{	
		if(JointIndex<16)
		{
			for(m=0; m <= NoOfCoeff; m++)
			{
				if(m==0) fscanf(ifp,"%f",&Constant[JointIndex]);
				else	
				{
					fscanf(ifp,"%f",&CosCoeff[JointIndex][m-1]);
					fscanf(ifp,"%f",&SinCoeff[JointIndex][m-1]);
				}
			}
		}
		else
		{
			Constant[JointIndex] = 0.0f;
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	
	fclose(ifp);
	
	for(t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0) sum = Constant[JointIndex];
				sum = (sum + CosCoeff[JointIndex][m]*cosf(2*(m+1)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sinf(2*(m+1)*PI/MotionPeriod[MotionNo]*t));
			}
			
			for(j=0;j<=3;j++)
			{
				if(t != 0) 
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
		if(t==TfLimit+10)	//must be larger than TfLimit
		{
			if(MotionSetFlag == 1)
			{
				Optimization(MotionNo);
				tempMotion = MotionNo;
			}
			else
			{
				MotionTimeCurrent = 0;
				Motion = MotionNo;
				MotionSetFlag = 1;
			}
		}
	}
	free(CosCoeff);
	free(SinCoeff);
	
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture9(void) //Dance4
{
	float scale[NO_OF_JOINT_MC] = { -0.8f,  -0.8f,  0.8f, -0.8f,  -0.8f,  -0.5f,  -0.8f,  -0.5f,  0.8f, -0.8f, -0.8f, 0.5f,  -0.8f,  -0.5f, 0.8f, 0.3f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	
	float	Constant[NO_OF_JOINT_MC];
	float **CosCoeff;
	float **SinCoeff;
	float	sum;
	
	int		NoOfCoeff = 38;
	int		m;
	int		JointIndex;
	int		t;
	int		i, j;
	
	FILE *ifp;
	
	int MotionNo = 9;
	
	MotionPeriod[MotionNo] = 5001;
	
	CosCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	SinCoeff = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	for(i=0;i<NO_OF_JOINT_MC;i++)
	{
		CosCoeff[i]=(float *)malloc(sizeof(float)*38);
		SinCoeff[i]=(float *)malloc(sizeof(float)*38);
	}
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\1d.txt", "r");
	fseek( ifp, 0L, SEEK_SET );
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{	
		if(JointIndex<16)
		{
			for(m=0; m <= NoOfCoeff; m++)
			{
				if(m==0) fscanf(ifp,"%f",&Constant[JointIndex]);
				else	
				{
					fscanf(ifp,"%f",&CosCoeff[JointIndex][m-1]);
					fscanf(ifp,"%f",&SinCoeff[JointIndex][m-1]);
				}
			}
		}
		else
		{
			Constant[JointIndex] = 0.0f;
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	
	fclose(ifp);
	
	for(t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0) sum = Constant[JointIndex];
				sum = (sum + CosCoeff[JointIndex][m]*cosf(2*(m+1)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sinf(2*(m+1)*PI/MotionPeriod[MotionNo]*t));
			}
			
			for(j=0;j<=3;j++)
			{
				if(t != 0) 
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
		if(t==TfLimit+10)	//must be larger than TfLimit
		{
			if(MotionSetFlag == 1)
			{
				Optimization(MotionNo);
				tempMotion = MotionNo;
			}
			else
			{
				MotionTimeCurrent = 0;
				Motion = MotionNo;
				MotionSetFlag = 1;
			}
		}
	}
	free(CosCoeff);
	free(SinCoeff);
	
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture10(void) //Bow
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][16];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][16];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -0.8f,  -1.0f,  -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  1.0f,  -1.0f,  -1.0f,  1.0f,  -1.0f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 10;

	NoOfCoeff = 15; //should be changed
	MotionPeriod[MotionNo] = 660;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_1.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);

	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion2(120, MotionPeriod[MotionNo]-130, t+j-3, MotionNo);
					PROF_LFingerMotion2(120, MotionPeriod[MotionNo]-130, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture11(void) //Right Up
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][9];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][9];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -1.0f,  -0.8f,  -1.0f,  -1.0f,  -1.0f,  1.0f,  -1.0f,  -1.0f,  1.0f,  -1.0f,  -1.0f,  1.0f,  -1.0f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 11;

	NoOfCoeff = 8; //should be changed
	MotionPeriod[MotionNo] = 450;// dim : 5 msec.		// T must be 3 times integer
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_3.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex <16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	//jw_flag = 10;
	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion2(70, MotionPeriod[MotionNo]-170, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}

}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture12(void) // Right Forward
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][10];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][10];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 12;

	NoOfCoeff = 9; //should be changed
	MotionPeriod[MotionNo] = 510;// dim : 5 msec.		// T must be 3 times integer
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_6.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex < 16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);

	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion2(50, MotionPeriod[MotionNo]-160, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture13(void) //Right Head
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][11];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][11];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 13;

	NoOfCoeff = 10; //should be changed
	
	//
	MotionPeriod[MotionNo] = 450;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_8.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);

	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion3(50, MotionPeriod[MotionNo]/2-50, t+j-3, MotionNo);
					PROF_RFingerMotion2(MotionPeriod[MotionNo]/2-50,MotionPeriod[MotionNo]-130, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture14(void) //Both Up
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][9];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][9];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 14;

	NoOfCoeff = 8; //should be changed
	
	MotionPeriod[MotionNo] = 510;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_5.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	//jw_flag = 10;
	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion2(50, MotionPeriod[MotionNo]-120, t+j-3, MotionNo);
					PROF_LFingerMotion2(50, MotionPeriod[MotionNo]-120, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture15(void) //Hand Wave
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][15];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][15];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 15;

	NoOfCoeff = 14; //should be changed
	
	MotionPeriod[MotionNo] = 810;// dim : 5 msec.		// T must be 3 times integer
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_2.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	//jw_flag = 10;
	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_RFingerMotion2(70, MotionPeriod[MotionNo]-300, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture16(void) //Left Up
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][8];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][8];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 16;

	NoOfCoeff = 7; //should be changed
	
	MotionPeriod[MotionNo] = 510;// dim : 5 msec.		// T must be 3 times integer
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_4.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);

	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_LFingerMotion2(50, MotionPeriod[MotionNo]-170, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}

	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);

		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
	//jw_flag = 40;
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture17(void) //Left Forward
{
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][10];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][10];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  1.0f,  -0.7f,  -1.0f,  -0.5f,  -1.0f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 17;

	NoOfCoeff = 9; //should be changed
	MotionPeriod[MotionNo] = 510;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_7.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_LFingerMotion2(30, MotionPeriod[MotionNo]-150, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture18(void) //Left Head
{
	// 0: Torso yaw
	// 1: Head yaw
	// 2: Head pitch\
	// 3: LSP
	// 4: LSY
	// 5: LSR
	// 6: LEB
	// 7: LWR
	// 8: LWP
	// 9: RSP
	// 10: RSY
	// 11: RSR
	// 12: REB
	// 13: RWR
	// 14: RWP
	// 15: LRHP
	FILE	*ifp;
	int		JointIndex;
	int		m;
	int		NoOfCoeff;
	float	CosCoeff[NO_OF_JOINT_MC][11];	// NoOfCoeff have to be less than 100
	float	SinCoeff[NO_OF_JOINT_MC][11];
	float	scale[NO_OF_JOINT_MC] = { -1.0f,  -1.0f,  0.8f,  -0.7f,  -1.0f,  -0.5f,  -1.5f,  -0.5f,  0.5f,  -0.7f,  -1.0f,  0.5f,  -1.0f,  -0.5f,  0.5f,  0.5f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	float	sum;
	int MotionNo = 18;

	NoOfCoeff = 10; //should be changed
	MotionPeriod[MotionNo] = 600;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\MotionCapture\\Jungwoo_9.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < NoOfCoeff+1; m++)
			{
				fscanf(ifp,"%f",&CosCoeff[JointIndex][m]);
				fscanf(ifp,"%f",&SinCoeff[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				CosCoeff[JointIndex][m] = 0.0f;
				SinCoeff[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	
	for(int t=0; t < (int)MotionPeriod[MotionNo]; t=t+3)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			for(m=0; m < NoOfCoeff; m++)
			{
				if(m==0)
				{
					sum =0;// Constant1[JointIndex];
				}
				sum = (float)(sum + CosCoeff[JointIndex][m]*cos(2*(m)*PI/MotionPeriod[MotionNo]*t) + SinCoeff[JointIndex][m]*sin(2*(m)*PI/MotionPeriod[MotionNo]*t));
			}
			for(int j=0;j<=3;j++)
			{
				if(t != 0)
				{
					if(t+j-3 == 0) MotionCapture[MotionNo][t+j-3][JointIndex] = 0.0f;
					MotionCapture[MotionNo][t+j-3][JointIndex] = (float)(MotionCapture[MotionNo][t-3][JointIndex] + (scale[JointIndex]*sum - MotionCapture[MotionNo][t-3][JointIndex])*(1./3.)*j);
					PROF_LFingerMotion3(90, MotionPeriod[MotionNo]/2-50, t+j-3, MotionNo);
					PROF_LFingerMotion2(MotionPeriod[MotionNo]/2-50, MotionPeriod[MotionNo]-170, t+j-3, MotionNo);
					if(t+j-3 >= 1) MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] = MotionCapture[MotionNo][t+j-3][JointIndex] - MotionCapture[MotionNo][t+j-3-1][JointIndex];
					if(t+j-3 >= 2) MotionCaptureAcc[MotionNo][t+j-3-2][JointIndex] = MotionCaptureVel[MotionNo][t+j-3-1][JointIndex] - MotionCaptureVel[MotionNo][t+j-3-2][JointIndex];
				}
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}

}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture19(void) //HyunMin
{
	FILE	*ifp;
	unsigned int		JointIndex;
	unsigned int		m;
	float	Maya[NO_OF_JOINT_MC][251];	
	float	scale[NO_OF_JOINT_MC] = { 1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f};
	
	int MotionNo = 19;
	
	MotionPeriod[MotionNo] = 251;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\Maya\\LeftUpDownTest.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex>=3 && JointIndex<7)
		{
			for(m=0; m < MotionPeriod[MotionNo]; m++)
			{
				fscanf(ifp,"%f",&Maya[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < MotionPeriod[MotionNo]; m++)
			{
				Maya[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);

	for(m=0; m < MotionPeriod[MotionNo]; m++)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			MotionCapture[MotionNo][m][JointIndex] = scale[JointIndex]*(Maya[JointIndex][m]);
			if(m >= 1) MotionCaptureVel[MotionNo][m-1][JointIndex] = MotionCapture[MotionNo][m][JointIndex] - MotionCapture[MotionNo][m-1][JointIndex];
			if(m >= 2) MotionCaptureAcc[MotionNo][m-2][JointIndex] = MotionCaptureVel[MotionNo][m-1][JointIndex] - MotionCaptureVel[MotionNo][m-2][JointIndex];
		}
	}
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/





/******************************************************************************/
void SetMotionCapture20(void) //HM1
{
	FILE	*ifp;
	unsigned int		JointIndex;
	unsigned int		m;
	
	float	**Maya;	
	float	scale[NO_OF_JOINT_MC] = { 1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f, 1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	
	int MotionNo = 20;
	
	Maya = (float **)malloc(sizeof(float*)*NO_OF_JOINT_MC);
	for(int i=0;i<NO_OF_JOINT_MC;i++)
	{
		Maya[i]=(float *)malloc(sizeof(float)*1178);
	}
	
	MotionPeriod[MotionNo] = 1178;// dim : 5 msec.		// T must be 3 times integer

	
	ifp = fopen("C:\\Rainbow\\Maya\\Maya1_hm.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < MotionPeriod[MotionNo]; m++)
			{
				fscanf(ifp,"%f",&Maya[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < MotionPeriod[MotionNo]; m++)
			{
				Maya[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	
	for(m=0; m < MotionPeriod[MotionNo]; m++)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			MotionCapture[MotionNo][m][JointIndex] = scale[JointIndex]*(Maya[JointIndex][m]);
			if(m >= 1) MotionCaptureVel[MotionNo][m-1][JointIndex] = MotionCapture[MotionNo][m][JointIndex] - MotionCapture[MotionNo][m-1][JointIndex];
			if(m >= 2) MotionCaptureAcc[MotionNo][m-2][JointIndex] = MotionCaptureVel[MotionNo][m-1][JointIndex] - MotionCaptureVel[MotionNo][m-2][JointIndex];
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
	//jw_flag = 40;
	free(Maya);
}
/******************************************************************************/





/******************************************************************************/
void SetMotionCapture21(void) //HM2
{
	FILE	*ifp;
	unsigned int		JointIndex;
	unsigned int		m;
	float	Maya[NO_OF_JOINT_MC][251];	// NoOfCoeff have to be less than 100
	float	scale[NO_OF_JOINT_MC] = { 1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,1.0f,  1.0f,  1.0f, 1.0f,	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f,};
	
	int MotionNo = 21;
	
	MotionPeriod[MotionNo] = 251;// dim : 5 msec.		// T must be 3 times integer
	
	
	ifp = fopen("C:\\Rainbow\\Maya\\Maya1_hm.txt","r");
	fseek( ifp, 0L, SEEK_SET );
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		if(JointIndex<16)
		{
			for(m=0; m < MotionPeriod[MotionNo]; m++)
			{
				fscanf(ifp,"%f",&Maya[JointIndex][m]);
			}
		}
		else
		{
			for(m=0; m < MotionPeriod[MotionNo]; m++)
			{
				Maya[JointIndex][m] = 0.0f;
			}
		}
	}
	fclose(ifp);
	
	for(m=0; m < MotionPeriod[MotionNo]; m++)
	{
		for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
		{
			MotionCapture[MotionNo][m][JointIndex] = scale[JointIndex]*(Maya[JointIndex][m]);
			if(m >= 1) MotionCaptureVel[MotionNo][m-1][JointIndex] = MotionCapture[MotionNo][m][JointIndex] - MotionCapture[MotionNo][m-1][JointIndex];
			if(m >= 2) MotionCaptureAcc[MotionNo][m-2][JointIndex] = MotionCaptureVel[MotionNo][m-1][JointIndex] - MotionCaptureVel[MotionNo][m-2][JointIndex];
		}
	}
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture22(void) //Stone
{
	int		JointIndex;
	int		MotionNo = 22;

	MotionPeriod[MotionNo] = 120;// dim : 5 msec.		// T must be 3 times integer
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		
		for(unsigned int t=0;t<= MotionPeriod[MotionNo];t++)
		{
			{
				if(t == 0) MotionCapture[MotionNo][t][JointIndex] = 0.0f;
				if(JointIndex<16)
				{
					MotionCapture[MotionNo][t][JointIndex] = 0;
				}
				
				PROF_RFingerMotion1(0, 120, t, MotionNo);
				PROF_LFingerMotion1(0, 120, t, MotionNo);
				if(t >= 1) MotionCaptureVel[MotionNo][t-1][JointIndex] = MotionCapture[MotionNo][t][JointIndex] - MotionCapture[MotionNo][t-1][JointIndex];
				if(t >= 2) MotionCaptureAcc[MotionNo][t-2][JointIndex] = MotionCaptureVel[MotionNo][t-1][JointIndex] - MotionCaptureVel[MotionNo][t-2][JointIndex];
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/






/******************************************************************************/
void SetMotionCapture23(void) //Paper
{
	int		JointIndex;
	int		MotionNo = 23;

	MotionPeriod[MotionNo] = 120;// dim : 5 msec.		// T must be 3 times integer
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		
		for(unsigned int t=0;t<= MotionPeriod[MotionNo];t++)
		{
			{
				if(t == 0) MotionCapture[MotionNo][t][JointIndex] = 0.0f;
				if(JointIndex<16)
				{
					MotionCapture[MotionNo][t][JointIndex] = 0;
				}
				
				PROF_RFingerMotion2(0, 120, t, MotionNo);
				PROF_LFingerMotion2(0, 120, t, MotionNo);
				if(t >= 1) MotionCaptureVel[MotionNo][t-1][JointIndex] = MotionCapture[MotionNo][t][JointIndex] - MotionCapture[MotionNo][t-1][JointIndex];
				if(t >= 2) MotionCaptureAcc[MotionNo][t-2][JointIndex] = MotionCaptureVel[MotionNo][t-1][JointIndex] - MotionCaptureVel[MotionNo][t-2][JointIndex];
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/





/******************************************************************************/
void SetMotionCapture24(void) //Scissor
{
	int		JointIndex;
	int		MotionNo = 24;
	
	MotionPeriod[MotionNo] = 120;// dim : 5 msec.		// T must be 3 times integer

	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		
		for(unsigned int t=0;t<= MotionPeriod[MotionNo];t++)
		{
			{
				if(t == 0) MotionCapture[MotionNo][t][JointIndex] = 0.0f;
				if(JointIndex<16)
				{
					MotionCapture[MotionNo][t][JointIndex] = 0;
				}
				
				PROF_RFingerMotion3(0, 120, t, MotionNo);
				PROF_LFingerMotion3(0, 120, t, MotionNo);
				if(t >= 1) MotionCaptureVel[MotionNo][t-1][JointIndex] = MotionCapture[MotionNo][t][JointIndex] - MotionCapture[MotionNo][t-1][JointIndex];
				if(t >= 2) MotionCaptureAcc[MotionNo][t-2][JointIndex] = MotionCaptureVel[MotionNo][t-1][JointIndex] - MotionCaptureVel[MotionNo][t-2][JointIndex];
			}
		}
	}
	
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/





/******************************************************************************/
void SetMotionCapture25(void) //Finger 14
{
	int		JointIndex;
	int		MotionNo = 25;
	
	MotionPeriod[MotionNo] = 120;// dim : 5 msec.		// T must be 3 times integer
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		
		for(unsigned int t=0;t<= MotionPeriod[MotionNo];t++)
		{
			{
				if(t == 0) MotionCapture[MotionNo][t][JointIndex] = 0.0f;
				if(JointIndex<16)
				{
					MotionCapture[MotionNo][t][JointIndex] = 0;
				}
				
				PROF_RFingerMotion4(0, 120, t, MotionNo);
				PROF_LFingerMotion4(0, 120, t, MotionNo);
				if(t >= 1) MotionCaptureVel[MotionNo][t-1][JointIndex] = MotionCapture[MotionNo][t][JointIndex] - MotionCapture[MotionNo][t-1][JointIndex];
				if(t >= 2) MotionCaptureAcc[MotionNo][t-2][JointIndex] = MotionCaptureVel[MotionNo][t-1][JointIndex] - MotionCaptureVel[MotionNo][t-2][JointIndex];
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/





/******************************************************************************/
void SetMotionCapture26(void) //Finger 1
{
	int		JointIndex;
	int MotionNo = 26;
	
	MotionPeriod[MotionNo] = 120;// dim : 5 msec.		// T must be 3 times integer
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		
		for(unsigned int t=0;t<= MotionPeriod[MotionNo];t++)
		{
			{
				if(t == 0) MotionCapture[MotionNo][t][JointIndex] = 0.0f;
				if(JointIndex<16)
				{
					MotionCapture[MotionNo][t][JointIndex] = 0;
				}				
				PROF_RFingerMotion5(0, 120, t, MotionNo);
				PROF_LFingerMotion5(0, 120, t, MotionNo);

				if(t >= 1) MotionCaptureVel[MotionNo][t-1][JointIndex] = MotionCapture[MotionNo][t][JointIndex] - MotionCapture[MotionNo][t-1][JointIndex];
				if(t >= 2) MotionCaptureAcc[MotionNo][t-2][JointIndex] = MotionCaptureVel[MotionNo][t-1][JointIndex] - MotionCaptureVel[MotionNo][t-2][JointIndex];
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture27(void) //Finger 1
{
	int		JointIndex;
	int MotionNo = 27;
	
	MotionPeriod[MotionNo] = 280;// dim : 5 msec.		// T must be 3 times integer
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		
		for(unsigned int t=0;t<= MotionPeriod[MotionNo];t++)
		{
			{
				if(t == 0) MotionCapture[MotionNo][t][JointIndex] = 0.0f;
				if(JointIndex<16)
				{
					MotionCapture[MotionNo][t][JointIndex] = 0;
				}
				
				PROF_RFingerMotion6(0, MotionPeriod[MotionNo], t, MotionNo);
				PROF_LFingerMotion6(0, MotionPeriod[MotionNo], t, MotionNo);

				if(t >= 1) MotionCaptureVel[MotionNo][t-1][JointIndex] = MotionCapture[MotionNo][t][JointIndex] - MotionCapture[MotionNo][t-1][JointIndex];
				if(t >= 2) MotionCaptureAcc[MotionNo][t-2][JointIndex] = MotionCaptureVel[MotionNo][t-1][JointIndex] - MotionCaptureVel[MotionNo][t-2][JointIndex];
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/




/******************************************************************************/
void SetMotionCapture28(void) //Finger 1
{
	// 0: -Torso yaw
	// 1: -Head yaw
	// 2: Head pitch
	// 3: -LSP
	// 4: -LSY
	// 5: -LSR
	// 6: -LEB
	// 7: -LWR
	// 8: LWP
	// 9: -RSP
	// 10: -RSY
	// 11: RSR
	// 12: -REB
	// 13: -RWR
	// 14: RWP
	// 15: LRHP
	int		JointIndex;
	int MotionNo = 28;
	float result1[5];
	float result2[5];
	float tp_res[2];

	MotionPeriod[MotionNo] = 2300;// dim : 5 msec.		// T must be 3 times integer
	
	for(JointIndex=0; JointIndex < NO_OF_JOINT_MC; JointIndex++)
	{
		
		for(unsigned int t=0;t<= MotionPeriod[MotionNo];t++)
		{
			{
				if(t == 0) MotionCapture[MotionNo][t][JointIndex] = 0.0f;
				if(JointIndex<16)
				{
					MotionCapture[MotionNo][t][JointIndex] = 0;
				}
				FTN_half_1_cos( 1.0,t,    0,100,0,0,&result1[0]);
				FTN_half_1_cos(-1.0,t,MotionPeriod[MotionNo]-100,100,0,0,&result2[0]);
				tp_res[0] = (float)(result1[0]+result2[0]);	// shoulder pitch
				MotionCapture[MotionNo][t][9] = -50.0f*tp_res[0];//RSP
				MotionCapture[MotionNo][t][12] = -20.0f*tp_res[0];//REB
				
				FTN_half_1_cos( 1.0,t,    160,100,0,0,&result1[1]);
				FTN_half_1_cos(-1.0,t,MotionPeriod[MotionNo]-100,100,0,0,&result2[1]);
				tp_res[1] = (float)(result1[1]+result2[1]);	// shoulder roll
				MotionCapture[MotionNo][t][3] = -50.0f*tp_res[1];//LSP
				MotionCapture[MotionNo][t][6] = -20.0f*tp_res[1];//LEB

				PROF_RFingerMotion6(100, 520,t,MotionNo);
				PROF_LFingerMotion6(260, 520,t,MotionNo);
				
				PROF_RFingerMotion2(521, 620,t,MotionNo);
				PROF_LFingerMotion2(521, 620,t,MotionNo);

				PROF_RFingerMotion1(631, 770,t,MotionNo);
				PROF_LFingerMotion1(631, 770,t,MotionNo);
				
				PROF_RFingerMotion2(781, 900,t,MotionNo);
				PROF_LFingerMotion2(781, 900,t,MotionNo);
				
				PROF_RFingerMotion7(1010, 1200,t,MotionNo);
				PROF_LFingerMotion7(1010, 1200,t,MotionNo);

				FTN_half_1_cos( 1.0,t,1425,100,0,0,&result1[2]);
				FTN_half_1_cos( -1.0,t,1925,100,0,0,&result2[2]);
				FTN_half_1_cos( -1.0,t,1525,100,0,0,&result1[3]);
				FTN_half_1_cos( 1.0,t,1625,100,0,0,&result2[3]);
				FTN_half_1_cos( -1.0,t,1725,100,0,0,&result1[4]);
				FTN_half_1_cos( 1.0,t,1825,100,0,0,&result2[4]);
				
				tp_res[2] = (float)(13.*((result1[2] + result2[2]) + 2.0*(result1[3] + result2[3] + result1[4] + result2[4])));
				tp_res[3] = (float)(20.*((result1[2] + result2[2]) + 2.0*(result1[3] + result2[3] + result1[4] + result2[4])));

				MotionCapture[MotionNo][t][1] = tp_res[2];//NKY
				MotionCapture[MotionNo][t][2] = tp_res[3];//NK1
				MotionCapture[MotionNo][t][26] = tp_res[3];//NK1


				if(t >= 1) MotionCaptureVel[MotionNo][t-1][JointIndex] = MotionCapture[MotionNo][t][JointIndex] - MotionCapture[MotionNo][t-1][JointIndex];
				if(t >= 2) MotionCaptureAcc[MotionNo][t-2][JointIndex] = MotionCaptureVel[MotionNo][t-1][JointIndex] - MotionCaptureVel[MotionNo][t-2][JointIndex];
			}
		}
	}
	
	if(MotionSetFlag == 1)
	{
		Optimization(MotionNo);
		tempMotion = MotionNo;
	}
	else
	{
		MotionTimeCurrent = 0;
		Motion = MotionNo;
		MotionSetFlag = 1;
	}
}
/******************************************************************************/






/******************************************************************************/

char FingerMove(int mag,int vel,int time, int start, int end, float *result)
{
	int T;
	T=end-start;
	if(vel>127)vel=127;
	
	if(mag<0)
	{
		vel=-vel;
	}

	if( int(T/2) >= int(mag/vel))
	{
		if(time<start)
		{
			*result=0.0f;
			return 0;
		}
		else if( time >= start && time < start + int(mag/vel) )
		{
			*result= (float)(vel*(time-start));
			return 1;
		}
		else if( time >= start+int(mag/vel) && time < end-int(mag/vel) )
		{
			*result= (float)(vel*(int(mag/vel)));
			return 2;
		}
		else if( time >= end-int(mag/vel) && time < end )
		{
			*result= (float)(-vel*(time-start-T+int(mag/vel)) + vel*int(mag/vel));
			return 3;
		}
		else  
		{	*result=0.0f;
			return 4;
		}
	}
	else
	{
		if(time<start)
		{
			*result=0.0f;
			return 0;
		}
		else if(time >= start && time < start + int(T/2))
		{
			*result= (float)(vel*(time-start));
			return 1;
		}
		else if(time >= start + int(T/2) && time < end)
		{
			*result= (float)(-vel*(time-start-int(T/2)) + T/2*vel);
			return 2;
		}
		else 
		{
			*result = 0.0f;
			return 3;
		}
	}

}
void PROF_RFingerMotion1(long start_time, long stop_time, int _time, int _MotionNo) // RH stone
{
	float	result1[3];

	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);

	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][16] = result1[2]; 
		MotionCapture[_MotionNo][_time][17] = result1[0];
		MotionCapture[_MotionNo][_time][18] = result1[0];
		MotionCapture[_MotionNo][_time][19] = result1[0];
		MotionCapture[_MotionNo][_time][20] = result1[0];
	}
}

void PROF_RFingerMotion2(long start_time, long stop_time, int _time, int _MotionNo) //RH paper
{
	float	result1[3];
	
	FingerMove(4000,127,_time, start_time, stop_time, &result1[0]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][16] = result1[0]; 
		MotionCapture[_MotionNo][_time][17] = result1[0];
		MotionCapture[_MotionNo][_time][18] = result1[0];
		MotionCapture[_MotionNo][_time][19] = result1[0];
		MotionCapture[_MotionNo][_time][20] = result1[0];
	}
}

void PROF_RFingerMotion3(long start_time, long stop_time, int _time, int _MotionNo) //RH scissor
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);

	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][16] = result1[2]; 
		MotionCapture[_MotionNo][_time][17] = result1[1];
		MotionCapture[_MotionNo][_time][18] = result1[1];
		MotionCapture[_MotionNo][_time][19] = result1[0];
		MotionCapture[_MotionNo][_time][20] = result1[0];
	}
}


void PROF_RFingerMotion4(long start_time, long stop_time, int _time, int _MotionNo) //RH 1,4
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][16] = result1[2]; 
		MotionCapture[_MotionNo][_time][17] = result1[1];
		MotionCapture[_MotionNo][_time][18] = result1[0];
		MotionCapture[_MotionNo][_time][19] = result1[0];
		MotionCapture[_MotionNo][_time][20] = result1[1];
	}
}


void PROF_RFingerMotion5(long start_time, long stop_time, int _time, int _MotionNo) //RH 1
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][16] = result1[2]; 
		MotionCapture[_MotionNo][_time][17] = result1[1];
		MotionCapture[_MotionNo][_time][18] = result1[0];
		MotionCapture[_MotionNo][_time][19] = result1[0];
		MotionCapture[_MotionNo][_time][20] = result1[0];
	}
}

void PROF_RFingerMotion6(long start_time, long stop_time, int _time, int _MotionNo) //RH Rainbow
{
	float	result1[5];
	//if(stop_time-start_time <=240+int(4000/127))
	//stop_time = start_time + 240+int(4000/127);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time+25, stop_time-25, &result1[1]);
	FingerMove(4000,127,_time, start_time+50, stop_time-50, &result1[2]);
	FingerMove(4000,127,_time, start_time+75, stop_time-75, &result1[3]);
	FingerMove(4000,127,_time, start_time+100, stop_time-100, &result1[4]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][16] = result1[0]; 
		MotionCapture[_MotionNo][_time][17] = result1[1];
		MotionCapture[_MotionNo][_time][18] = result1[2];
		MotionCapture[_MotionNo][_time][19] = result1[3];
		MotionCapture[_MotionNo][_time][20] = result1[4];
	}
}

void PROF_RFingerMotion7(long start_time, long stop_time, int _time, int _MotionNo) //RH 01
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][16] = result1[1]; 
		MotionCapture[_MotionNo][_time][17] = result1[1];
		MotionCapture[_MotionNo][_time][18] = result1[0];
		MotionCapture[_MotionNo][_time][19] = result1[0];
		MotionCapture[_MotionNo][_time][20] = result1[0];
	}
}




void PROF_LFingerMotion1(long start_time, long stop_time, int _time, int _MotionNo) //LH stone
{
	float	result1[3];

	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(-4000,127,_time, start_time+20, stop_time, &result1[2]);

	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][21] = result1[2]; 
		MotionCapture[_MotionNo][_time][22] = result1[0];
		MotionCapture[_MotionNo][_time][23] = result1[0];
		MotionCapture[_MotionNo][_time][24] = result1[0];
		MotionCapture[_MotionNo][_time][25] = result1[0];
	}
}

void PROF_LFingerMotion2(long start_time, long stop_time, int _time, int _MotionNo) //LH paper
{
	float	result1[3];
	
	FingerMove(4000,127,_time, start_time, stop_time, &result1[0]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][21] = result1[0]; 
		MotionCapture[_MotionNo][_time][22] = result1[0];
		MotionCapture[_MotionNo][_time][23] = result1[0];
		MotionCapture[_MotionNo][_time][24] = result1[0];
		MotionCapture[_MotionNo][_time][25] = result1[0];
	}
}

void PROF_LFingerMotion3(long start_time, long stop_time, int _time, int _MotionNo) //LH scissor
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);

	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][21] = result1[2]; 
		MotionCapture[_MotionNo][_time][22] = result1[1];
		MotionCapture[_MotionNo][_time][23] = result1[1];
		MotionCapture[_MotionNo][_time][24] = result1[0];
		MotionCapture[_MotionNo][_time][25] = result1[0];
	}
}


void PROF_LFingerMotion4(long start_time, long stop_time, int _time, int _MotionNo) //LH 1,4
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][21] = result1[2]; 
		MotionCapture[_MotionNo][_time][22] = result1[1];
		MotionCapture[_MotionNo][_time][23] = result1[0];
		MotionCapture[_MotionNo][_time][24] = result1[0];
		MotionCapture[_MotionNo][_time][25] = result1[1];
	}
}


void PROF_LFingerMotion5(long start_time, long stop_time, int _time, int _MotionNo) //LH 1
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][21] = result1[2]; 
		MotionCapture[_MotionNo][_time][22] = result1[1];
		MotionCapture[_MotionNo][_time][23] = result1[0];
		MotionCapture[_MotionNo][_time][24] = result1[0];
		MotionCapture[_MotionNo][_time][25] = result1[0];
	}
}


void PROF_LFingerMotion6(long start_time, long stop_time, int _time, int _MotionNo) //LH Rainbow
{
	float	result1[5];
	/*
	if(stop_time-start_time <=240+int(4000/127))
			stop_time = start_time + 240+int(4000/127);*/
	
	FingerMove(4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time+25, stop_time-25, &result1[1]);
	FingerMove(4000,127,_time, start_time+50, stop_time-50, &result1[2]);
	FingerMove(4000,127,_time, start_time+75, stop_time-75, &result1[3]);
	FingerMove(4000,127,_time, start_time+100, stop_time-100, &result1[4]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][21] = result1[0]; 
		MotionCapture[_MotionNo][_time][22] = result1[1];
		MotionCapture[_MotionNo][_time][23] = result1[2];
		MotionCapture[_MotionNo][_time][24] = result1[3];
		MotionCapture[_MotionNo][_time][25] = result1[4];
	}

}


void PROF_LFingerMotion7(long start_time, long stop_time, int _time, int _MotionNo) //LH 01
{
	float	result1[3];
	
	FingerMove(-4000,127,_time, start_time, stop_time, &result1[0]);
	FingerMove(4000,127,_time, start_time, stop_time, &result1[1]);
	FingerMove(-3000,127,_time, start_time+20, stop_time, &result1[2]);
	
	if(_time >= start_time)
	{
		MotionCapture[_MotionNo][_time][21] = result1[1]; 
		MotionCapture[_MotionNo][_time][22] = result1[1];
		MotionCapture[_MotionNo][_time][23] = result1[0];
		MotionCapture[_MotionNo][_time][24] = result1[0];
		MotionCapture[_MotionNo][_time][25] = result1[0];
	}
}

/******************************************************************************/
void FingerControlModeChange(unsigned int _boardID, unsigned char _enable)
{
	unsigned char tempData[8];
	bool _enableFlag;

	if(_enable == 0x01) _enableFlag = true;
	else _enableFlag = false;

	tempData[0] = _boardID;
	tempData[1] = ControlMode;
	tempData[2] = _enable;		// Current Mode: 1, Position Mode : 0

	switch(_boardID)
	{
		case EJMC4:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC4 Current Control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC4 Position Control is enabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC5:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC5 Current Control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC5 Position Control is disabled!!");
			CanSendMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(FingerControlModeChange)..!!");
			break;
	}
	RtSleep(15);
}
/******************************************************************************/







/******************************************************************************/
bool SetMoveTaskPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime, float _msDelayTime[2], float _userData[5], unsigned char _mode, unsigned char _function)
{
	unsigned char i;

	if(_msTime <= 0) { RtWprintf(L"\n>>> Goal time must be grater than zero(SetMoveTaskPos)..!!"); return false; }

	switch(_mode)
	{
	case 0x00:	// relative mode
		_walkingInfo->RefPatternToGo = _walkingInfo->RefPatternCurrent + _pos;
		break;
	case 0x01:	// absolute mode
		_walkingInfo->RefPatternToGo = _pos;
		break;
	default:
		RtWprintf(L"\n>>> Wrong reference mode(SetMoveTaskPos)..!!");
		return false;
		break;
	}

	_walkingInfo->MoveFlag = false;
	
	_walkingInfo->RefPatternInitial = _walkingInfo->RefPatternCurrent;
	_walkingInfo->CurrentTimeCount = 0;
	_walkingInfo->DelayTimeCount[0] = ((unsigned long)(_msDelayTime[0]))/INT_TIME;
	_walkingInfo->DelayTimeCount[1] = ((unsigned long)(_msDelayTime[1]))/INT_TIME;
	_walkingInfo->GoalTimeCount = ((unsigned long)(_msTime))/INT_TIME - _walkingInfo->DelayTimeCount[0] - _walkingInfo->DelayTimeCount[1];
	_walkingInfo->RefPatternDelta = _walkingInfo->RefPatternToGo - _walkingInfo->RefPatternCurrent;
	_walkingInfo->FunctionMode = _function;
	for(i=0 ; i<5 ; i++) _walkingInfo->UserData[i] = _userData[i];
	
	_walkingInfo->MoveFlag = true;

	return true;
}
/******************************************************************************/






/******************************************************************************/
void MoveTaskPos(WALKING_INFO* _walkingInfoSway, WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _landingState, unsigned char _mode)
{	
	unsigned char i, j;
	float tempPos[6], tempAngle[6];
	float tempTime, tempMixPos, tempMixTime, tempMag;
	float tempSideWalk, tempSidePeriod, tempSideTime, tempSideDelay;

	if(_mode == CTRLMODE_WALKING)
	{	
		//if(_landingState == RIGHT_FOOT_LATE_LANDING) {if(SwayControlFlag == 0x00) SwayControlFlag = 0x03; }	// right foot late landing
		//else if(_landingState == LEFT_FOOT_LATE_LANDING) {if(SwayControlFlag == 0x00) SwayControlFlag = 0x04; }	// left foot late landing

		// for basic sway
		if(_walkingInfoSway->MoveFlag == true)
		{
			if(_walkingInfoSway->DelayTimeCount[0] == 0) 
			{
				_walkingInfoSway->CurrentTimeCount++;
				if(_walkingInfoSway->GoalTimeCount <= _walkingInfoSway->CurrentTimeCount)
				{
					_walkingInfoSway->GoalTimeCount = _walkingInfoSway->CurrentTimeCount = 0;
					_walkingInfoSway->RefPatternCurrent = _walkingInfoSway->RefPatternToGo;
					if(_walkingInfoSway->DelayTimeCount[1] == 0) _walkingInfoSway->MoveFlag = false;	
					else _walkingInfoSway->DelayTimeCount[1]--; // tail time delay
				}
				else
				{
					tempTime = (float)_walkingInfoSway->CurrentTimeCount/(float)_walkingInfoSway->GoalTimeCount;
					// function mode: 0  ->  mag.*0.5*(1.0-cos(pi*t))
					if(_walkingInfoSway->FunctionMode == 0) _walkingInfoSway->RefPatternCurrent = _walkingInfoSway->RefPatternInitial+_walkingInfoSway->RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime));
				}	
			}	
			else _walkingInfoSway->DelayTimeCount[0]--;		// head time delay
		}

		// current walking position generator 
		for(i=RIGHT ; i<=LEFT ; i++)	// right and left
		{
			for(j=X ; j<=Yaw ; j++)	// X, Y, Z and Yaw
			{
				if(_walkingInfo[i][j].MoveFlag == true)
				{
					if(_walkingInfo[i][j].DelayTimeCount[0] == 0) 
					{
						_walkingInfo[i][j].CurrentTimeCount++;
						if(_walkingInfo[i][j].GoalTimeCount <= _walkingInfo[i][j].CurrentTimeCount)
						{
							_walkingInfo[i][j].GoalTimeCount = _walkingInfo[i][j].CurrentTimeCount = 0;
							_walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternToGo;
							if(_walkingInfo[i][j].DelayTimeCount[1] == 0) _walkingInfo[i][j].MoveFlag = false;	
							else _walkingInfo[i][j].DelayTimeCount[1]--; // tail time delay
						}
						else
						{
							if( (j==Z) && (_landingState==RIGHT_FOOT_EARLY_LANDING) && (WalkingStartStop == 0x02) )	// right leg early landing
							{
								_walkingInfo[RIGHT][Z].RefPatternInitial = _walkingInfo[RIGHT][Z].RefPatternToGo = _walkingInfo[RIGHT][Z].RefPatternCurrent; 
								_walkingInfo[RIGHT][Z].RefPatternDelta = 0.0f;		
							}
							else if( (j==Z) && (_landingState==LEFT_FOOT_EARLY_LANDING) && (WalkingStartStop == 0x02) )	// left leg early landing
							{
								_walkingInfo[LEFT][Z].RefPatternInitial = _walkingInfo[LEFT][Z].RefPatternToGo = _walkingInfo[LEFT][Z].RefPatternCurrent; 
								_walkingInfo[LEFT][Z].RefPatternDelta = 0.0f;
							} 
							else	// normal sequence
							{
								tempTime = (float)_walkingInfo[i][j].CurrentTimeCount/(float)_walkingInfo[i][j].GoalTimeCount;
								// function mode: 0  ->  mag.*0.5*(1.0-cos(pi*t))
								if(_walkingInfo[i][j].FunctionMode == 0) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime));
								// function mode: 1	->	mag.*0.5*(1.0-cos(0.5*pi*t)) ("1-cos" head)
								else if(_walkingInfo[i][j].FunctionMode == 1) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(1.0f-cosf(0.5f*PI*tempTime));
								// function mode: 2	->	mag.*0.5*(-cos(0.5(pi*(t+1))) ("1-cos" tail)
								else if(_walkingInfo[i][j].FunctionMode == 2) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(-cosf(0.5f*PI*(tempTime+1.0f)));
								// function mode: 3  ->  mag.*sin(0.5*pi*t)
								else if(_walkingInfo[i][j].FunctionMode == 3) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*sinf(0.5f*PI*tempTime);
								// function mode: 4  ->  mag.*(t-sin(pi*t)/pi)
								else if(_walkingInfo[i][j].FunctionMode == 4) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(tempTime-sinf(PI*tempTime)/PI);
								// function mode: 5  ->  mag.*(t+sin(pi*t)/pi)
								else if(_walkingInfo[i][j].FunctionMode == 5) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(tempTime+sinf(PI*tempTime)/PI);
								// function mode: 6	->	inti.* (t+sin(pi*t')/pi)-init.*(1-cos(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 6)
								{
									tempMag = _walkingInfo[i][j].RefPatternInitial;
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag*(tempMixTime+sinf(PI*tempMixTime)/PI);
									if(pSharedMemory->JW_temp[6] >= 0.0f)
									{
										if( tempMixPos >= _walkingInfo[i][j].RefPatternInitial+tempMag ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag;
									}
									else
									{
										if( tempMixPos <= _walkingInfo[i][j].RefPatternInitial+tempMag ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag;
									}
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos - tempMag*(1.0f-cosf(0.5f*PI*tempTime));
								}
								// function mode: 7	->	mag.*2*(t+sin(pi*t')/pi)-mag.*(sin(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 7)
								{
									tempMixTime = ((float)_walkingInfo[i][j].CurrentTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME)/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									if(tempMixTime < 0.0f) tempMixTime = 0.0f;
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*2.0f*(tempMixTime-sinf(PI*tempMixTime)/PI);
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos - _walkingInfo[i][j].RefPatternDelta*(sinf(0.5f*PI*tempTime));
								}
								// function mode: 8	->	mag.*2*(t+sin(pi*t')/pi)-mag.*(1-cos(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 8)
								{
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta*(tempMixTime+sinf(PI*tempMixTime)/PI);
									if(pSharedMemory->JW_temp[6] >= 0.0f)
									{
										if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									else
									{
										if( tempMixPos <= _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos - _walkingInfo[i][j].RefPatternDelta*(1.0f-cosf(0.5f*PI*tempTime));
								}
								// function mode: 9	->	mag.*(t+sin(pi*t')/pi)-mag.*(sin(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 9)
								{
									tempMag = _walkingInfo[i][j].RefPatternInitial;
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag*(tempMixTime-sinf(PI*tempMixTime)/PI);
									if(pSharedMemory->JW_temp[6] >= 0.0f)
									{
										if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial-tempMag) tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag;
									}
									else
									{
										if(tempMixPos <= _walkingInfo[i][j].RefPatternInitial-tempMag) tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag;
									}
									_walkingInfo[i][j].RefPatternCurrent = tempMag + tempMixPos - tempMag*(1.0f-sinf(0.5f*PI*tempTime));

								}
								// function mode: 10	->	mag.*(t+sin(pi*t')/pi)
								else if(_walkingInfo[i][j].FunctionMode == 10)
								{
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta*(tempMixTime+sinf(PI*tempMixTime)/PI);
									if(pSharedMemory->JW_temp[6] >= 0.0f)
									{
										if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									else
									{
										if(tempMixPos <= _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos;
								}
								else if(_walkingInfo[i][j].FunctionMode == 11)
								{
									tempSidePeriod = (float)_walkingInfo[i][j].GoalTimeCount/2.0f - _walkingInfo[i][j].UserData[1]/INT_TIME;
									tempSideTime = (float)_walkingInfo[i][j].CurrentTimeCount/tempSidePeriod;
									tempSideDelay = 2.0f*_walkingInfo[i][j].UserData[1]/INT_TIME/tempSidePeriod;

									if(tempSideTime < 1.0f) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f-cosf(PI*tempSideTime));
									else if( tempSideTime < (1.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0];
									else if( tempSideTime < (2.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f+cosf(PI*(tempSideTime-1.0f-tempSideDelay)));
									else tempSideWalk = 0.0f;
 
									_walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime)) + tempSideWalk;
								}
								else if(_walkingInfo[i][j].FunctionMode == 12)
								{
									tempSidePeriod = (float)_walkingInfo[i][j].GoalTimeCount/2.0f - _walkingInfo[i][j].UserData[1]/INT_TIME;
									tempSideTime = (float)_walkingInfo[i][j].CurrentTimeCount/tempSidePeriod;
									tempSideDelay = 2.0f*_walkingInfo[i][j].UserData[1]/INT_TIME/tempSidePeriod;

									if(tempSideTime < 1.0f) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f-cosf(PI*tempSideTime));
									else if( tempSideTime < (1.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0];
									else if( tempSideTime < (2.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f+cosf(PI*(tempSideTime-1.0f-tempSideDelay)));
									else tempSideWalk = 0.0f;
 
									_walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime)) - tempSideWalk;
								}
							}
						}	// if(_walkingInfo[i][j].GoalTimeCount <= _walkingInfo[i][j].CurrentTimeCount)
					}	// if(_walkingInfo[i][j].DelayTimeCount[0] == 0)
					else _walkingInfo[i][j].DelayTimeCount[0]--;		// head time delay
				}	// end of "if(WalkingInfo[i][j].MoveFlag == true)"
			}	// end of "for(j=X ; j<=Yaw ; j++)"
		}	// end of "for(i=RIGHT ; i<=LEFT ; i++)"	
	}
	else if(_mode == CTRLMODE_ZMP_INITIALIZATION)
	{
	}
	else;

	MoveTaskPosJointFF(_joint);

	// right leg joint angle reference
	tempPos[0] = _walkingInfo[RIGHT][X].RefPatternCurrent + _walkingInfo[RIGHT][X].RefPosFF + _walkingInfo[RIGHT][X].ControlDSPZMP;	// X
	tempPos[1] = _walkingInfoSway->RefPatternCurrent + _walkingInfo[RIGHT][Y].RefPatternCurrent + _walkingInfo[RIGHT][Y].RefPosFF + _walkingInfo[RIGHT][Y].ControlDSPZMP;	// Y
	tempPos[2] = _walkingInfo[RIGHT][Z].RefPatternCurrent + _walkingInfo[RIGHT][Z].RefPosFF;	// Z
	tempPos[3] = 0.0f;	// Roll
	tempPos[4] = 0.0f;	// Pitch
	tempPos[5] = _walkingInfo[RIGHT][Yaw].RefPatternCurrent;// + _walkingInfo[RIGHT][Yaw].RefPosFF;	// Yaw
	InverseKinematics(tempPos, tempAngle);
	_joint[RHY].WalkingRefAngle = tempAngle[0];
	_joint[RHR].WalkingRefAngle = tempAngle[1];
	_joint[RHP].WalkingRefAngle = tempAngle[2];
	_joint[RKN].WalkingRefAngle = tempAngle[3];
	_joint[RAP].WalkingRefAngle = tempAngle[4];
	_joint[RAR].WalkingRefAngle = tempAngle[5];

	_joint[RHY].RefAngleCurrent = _joint[RHY].WalkingRefAngle + _joint[RHY].RefAngleFF + _joint[RHY].ControlAngleFF;
	_joint[RHR].RefAngleCurrent = _joint[RHR].WalkingRefAngle + _joint[RHR].RefAngleFF + _joint[RHR].ControlVibrationAngle +  _joint[RHR].ControlAngleFF;
	_joint[RHP].RefAngleCurrent = _joint[RHP].WalkingRefAngle + _joint[RHP].RefAngleFF + _joint[RHP].ControlAngleFF;
	_joint[RKN].RefAngleCurrent = _joint[RKN].WalkingRefAngle + _joint[RKN].RefAngleFF + _joint[RKN].ControlAngleFF;
	_joint[RAP].RefAngleCurrent = _joint[RAP].WalkingRefAngle + _joint[RAP].RefAngleFF + _joint[RAP].ControlDampAngleCurent + _joint[RAP].ControlAngleFF;
	_joint[RAR].RefAngleCurrent = _joint[RAR].WalkingRefAngle + _joint[RAR].RefAngleFF + _joint[RAR].ControlDampAngleCurent + _joint[RAR].ControlAngleFF;


	// left leg joint angle reference
	tempPos[0] = _walkingInfo[LEFT][X].RefPatternCurrent + _walkingInfo[LEFT][X].RefPosFF + _walkingInfo[LEFT][X].ControlDSPZMP;	// X
	tempPos[1] = _walkingInfoSway->RefPatternCurrent + _walkingInfo[LEFT][Y].RefPatternCurrent + _walkingInfo[LEFT][Y].RefPosFF + _walkingInfo[LEFT][Y].ControlDSPZMP;	// Y
	tempPos[2] = _walkingInfo[LEFT][Z].RefPatternCurrent + _walkingInfo[LEFT][Z].RefPosFF;	// Z
	tempPos[3] = 0.0f;	// Roll
	tempPos[4] = 0.0f;	// Pitch
	tempPos[5] = _walkingInfo[LEFT][Yaw].RefPatternCurrent;// + _walkingInfo[LEFT][Yaw].RefPosFF;	// Yaw
	InverseKinematics(tempPos, tempAngle);
	_joint[LHY].WalkingRefAngle = tempAngle[0];
	_joint[LHR].WalkingRefAngle = tempAngle[1];
	_joint[LHP].WalkingRefAngle = tempAngle[2];
	_joint[LKN].WalkingRefAngle = tempAngle[3];
	_joint[LAP].WalkingRefAngle = tempAngle[4];
	_joint[LAR].WalkingRefAngle = tempAngle[5];

	_joint[LHY].RefAngleCurrent = _joint[LHY].WalkingRefAngle + _joint[LHY].RefAngleFF + _joint[LHY].ControlAngleFF;
	_joint[LHR].RefAngleCurrent = _joint[LHR].WalkingRefAngle + _joint[LHR].RefAngleFF + _joint[LHR].ControlVibrationAngle + _joint[LHR].ControlAngleFF;
	_joint[LHP].RefAngleCurrent = _joint[LHP].WalkingRefAngle + _joint[LHP].RefAngleFF + _joint[LHP].ControlAngleFF;
	_joint[LKN].RefAngleCurrent = _joint[LKN].WalkingRefAngle + _joint[LKN].RefAngleFF + _joint[LKN].ControlAngleFF;
	_joint[LAP].RefAngleCurrent = _joint[LAP].WalkingRefAngle + _joint[LAP].RefAngleFF + _joint[LAP].ControlDampAngleCurent + _joint[LAP].ControlAngleFF;
	_joint[LAR].RefAngleCurrent = _joint[LAR].WalkingRefAngle + _joint[LAR].RefAngleFF + _joint[LAR].ControlDampAngleCurent + _joint[LAR].ControlAngleFF;

	// goto reference angle
	for(i=JMC0 ; i<=JMC7 ; i++) MoveJMC(i);
}
/******************************************************************************/





/******************************************************************************/
bool SetMoveTaskPosJointFF(JOINT* _joint, float _angle, float _msTime, float _msDelayTime[2], unsigned char _mode, unsigned char _function)
{
	if(_msTime <= 0) { RtWprintf(L"\n>>> goal time must be grater than zero(SetMoveTaskPosJointFF)..!!"); return false; }
	
	switch(_mode)
	{
	case 0x00:	// relative mode
		_joint->ControlAngleFFToGo = _joint->ControlAngleFF + _angle;
		break;
	case 0x01:	// absolute mode
		_joint->ControlAngleFFToGo = _angle;
		break;
	default:
		RtWprintf(L"\n>>> Wrong reference mode(SetMoveTaskPosJointFF)..!!");
		return false;
		break;
	}

	_joint->ControlFFMoveFlag = false;
	_joint->ControlAngleFFDelta = _joint->ControlAngleFFToGo - _joint->ControlAngleFF;
	_joint->ControlAngleFFInitial = _joint->ControlAngleFF;
	_joint->ControlFFGoalTimeCount = (unsigned long)(_msTime/INT_TIME);
	_joint->ControlFFCurrentTimeCount = 0;
	_joint->ControlFFDelayTimeCount[0] = ((unsigned long)(_msDelayTime[0]))/INT_TIME;
	_joint->ControlFFDelayTimeCount[1] = ((unsigned long)(_msDelayTime[1]))/INT_TIME;
	_joint->ControlFFFunctionMode = _function;
	_joint->ControlFFMoveFlag = true;

	RtWprintf(L"\n>>> Joint reference is set(SetMoveTaskPosJointFF)..!!");

	return true;
}
/******************************************************************************/






/******************************************************************************/
void MoveTaskPosJointFF(JOINT _joint[])
{
	unsigned char i;
	float tempTime;
	
	for(i=RHY ; i<=LAR ; i++)
	{
	
		if(_joint[i].ControlFFMoveFlag == true)
		{
			if(_joint[i].ControlFFDelayTimeCount[0] == 0) 
			{
				_joint[i].ControlFFCurrentTimeCount++;
				if(_joint[i].ControlFFGoalTimeCount <= _joint[i].ControlFFCurrentTimeCount)
				{
					_joint[i].ControlFFGoalTimeCount = _joint[i].ControlFFCurrentTimeCount = 0;
					_joint[i].ControlAngleFF = _joint[i].ControlAngleFFToGo;
					if(_joint[i].ControlFFDelayTimeCount[1] == 0) _joint[i].ControlFFMoveFlag = false;	
					else _joint[i].ControlFFDelayTimeCount[1]--; // tail time delay
				}
				else
				{
					tempTime = (float)_joint[i].ControlFFCurrentTimeCount/(float)_joint[i].ControlFFGoalTimeCount;
					// function mode: 0  ->  mag.*0.5*(1.0-cos(pi*t))
					if(_joint[i].ControlFFFunctionMode == 0) _joint[i].ControlAngleFF = _joint[i].ControlAngleFFInitial+_joint[i].ControlAngleFFDelta*0.5f*(1.0f-cosf(PI*tempTime));
				}	
			}	
			else _joint[i].ControlFFDelayTimeCount[0]--;		// head time delay
		}
	}
}
/******************************************************************************/






/******************************************************************************/
void MoveJMC(unsigned char _boardID)
{
	unsigned char tempData[8];
	int tempPulse;
	unsigned int currentPulse;
	short short_temp;
	unsigned short short_currentPulse;
	char char_temp;

	switch(_boardID)
	{
		case JMC0:
			tempPulse = (int)(Joint[RHY].RefAngleCurrent*Joint[RHY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			tempPulse = (int)(Joint[RHR].RefAngleCurrent*Joint[RHR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			CanSendMsg(Joint[RHY].CAN_channel, Joint[RHY].Ref_txdf, tempData, 6, 0);
			break;
		case JMC1:
			tempPulse = (int)(Joint[RHP].RefAngleCurrent*Joint[RHP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			CanSendMsg(Joint[RHP].CAN_channel, Joint[RHP].Ref_txdf, tempData, 3, 0);
			break;
		case JMC2:
			tempPulse = (int)(Joint[RKN].RefAngleCurrent*Joint[RKN].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			CanSendMsg(Joint[RKN].CAN_channel, Joint[RKN].Ref_txdf, tempData, 3, 0);
			break;
		case JMC3:
			tempPulse = (int)(Joint[RAP].RefAngleCurrent*Joint[RAP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[RAR].RefAngleCurrent*Joint[RAR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			CanSendMsg(Joint[RAP].CAN_channel, Joint[RAP].Ref_txdf, tempData, 6, 0);
			break;
		case JMC4:
			tempPulse = (int)(Joint[LHY].RefAngleCurrent*Joint[LHY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LHR].RefAngleCurrent*Joint[LHR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			CanSendMsg(Joint[LHY].CAN_channel, Joint[LHY].Ref_txdf, tempData, 6, 0);
			break;			
		case JMC5:
			tempPulse = (int)(Joint[LHP].RefAngleCurrent*Joint[LHP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			CanSendMsg(Joint[LHP].CAN_channel, Joint[LHP].Ref_txdf, tempData, 3, 0);		
			break;
		case JMC6:
			tempPulse = (int)(Joint[LKN].RefAngleCurrent*Joint[LKN].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			CanSendMsg(Joint[LKN].CAN_channel, Joint[LKN].Ref_txdf, tempData, 3, 0);
			break;
		case JMC7:
			tempPulse = (int)(Joint[LAP].RefAngleCurrent*Joint[LAP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LAR].RefAngleCurrent*Joint[LAR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			CanSendMsg(Joint[LAP].CAN_channel, Joint[LAP].Ref_txdf, tempData, 6, 0);
			break;
		case EJMC3:
			tempPulse = (int)(Joint[WST].RefAngleCurrent*Joint[WST].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			CanSendMsg(Joint[WST].CAN_channel, Joint[WST].Ref_txdf, tempData, 3, 0);
			break;
		case JMC8:
			tempPulse = (int)(Joint[RSP].RefAngleCurrent*Joint[RSP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[RSR].RefAngleCurrent*Joint[RSR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			CanSendMsg(Joint[RSP].CAN_channel, Joint[RSP].Ref_txdf, tempData, 6, 0);
			break;
		case JMC9:
			tempPulse = (int)(Joint[RSY].RefAngleCurrent*Joint[RSY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[REB].RefAngleCurrent*Joint[REB].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			CanSendMsg(Joint[RSY].CAN_channel, Joint[RSY].Ref_txdf, tempData, 6, 0);
			break;
		case JMC10:
			tempPulse = (int)(Joint[LSP].RefAngleCurrent*Joint[LSP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LSR].RefAngleCurrent*Joint[LSR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			CanSendMsg(Joint[LSP].CAN_channel, Joint[LSP].Ref_txdf, tempData, 6, 0);
			break;
		case JMC11:
			tempPulse = (int)(Joint[LSY].RefAngleCurrent*Joint[LSY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LEB].RefAngleCurrent*Joint[LEB].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			CanSendMsg(Joint[LSY].CAN_channel, Joint[LSY].Ref_txdf, tempData, 6, 0);
			break;		
		case EJMC0:
			tempPulse = (int)(Joint[RWY].RefAngleCurrent*Joint[RWY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[RWP].RefAngleCurrent*Joint[RWP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			CanSendMsg(Joint[RWY].CAN_channel, Joint[RWY].Ref_txdf, tempData, 6, 0);	
			break;
		case EJMC1:
			tempPulse = (int)(Joint[LWY].RefAngleCurrent*Joint[LWY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LWP].RefAngleCurrent*Joint[LWP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			CanSendMsg(Joint[LWY].CAN_channel, Joint[LWY].Ref_txdf, tempData, 6, 0);	
			break;
		case EJMC2:
			//Joint[NKY].PPR = 100.f*128.f/360.f;
			//Joint[NK1].PPR = 100.f*128.f/360.f;
			//Joint[NK2].PPR = 100.f*128.f/360.f;
			
			jw_test[0]=Joint[NKY].RefVelCurrent;
			char_temp = (char)(Joint[NKY].RefVelCurrent*Joint[NKY].PPR);
			jw_test[1]=char_temp;
			//Truncation error compensation
			NKY_err_short_float[0] += (Joint[NKY].RefVelCurrent*Joint[NKY].PPR - (float)char_temp);	// mod by inhyeok
			if(NKY_err_short_float[0] >= 1.0f) 
			{
				if(NKY_err_short_float[0] > 0.)
				{
					char_temp += 1;
					NKY_err_short_float[0] -= 1.0f;
				}
				else
				{
					char_temp -= 1;
					NKY_err_short_float[0] += 1.0f;
				}
			}
			tempData[0] = char_temp;
			
			
			//-------------------------------------------------------			
			//short_temp = (short)(Joint[NK1].RefVelCurrent*Joint[NK1].PPR);			
			char_temp = (char)(Joint[NK1].RefVelCurrent*Joint[NK1].PPR);			
			
			//Truncation error compensation
			//NKY_err_short_float[1] += (Joint[NK1].RefVelCurrent*Joint[NK1].PPR - (float)short_temp);			
			NKY_err_short_float[1] += (Joint[NK1].RefVelCurrent*Joint[NK1].PPR - (float)char_temp);			
			
			while( fabs(NKY_err_short_float[1]) >= 1.0f) 
			{
				if(NKY_err_short_float[1] > 0.)
				{
					char_temp += 1;
					NKY_err_short_float[1] -= 1.0f;
				}
				else
				{
					char_temp -= 1;
					NKY_err_short_float[1] += 1.0f;
				}
				
			}
			tempData[1] = char_temp;
			//-------------------------------------------------------
			
			
			char_temp = (char)(Joint[NK2].RefVelCurrent*Joint[NK2].PPR);
			//Truncation error compensation
			NKY_err_short_float[2] += (Joint[NK2].RefVelCurrent*Joint[NK2].PPR - (float)char_temp);
			if(NKY_err_short_float[2] >= 1.0f) 
			{
				if(NKY_err_short_float[2] > 0.)
				{
					char_temp += 1;
					NKY_err_short_float[2] -= 1.0f;
				}
				else
				{
					char_temp -= 1;
					NKY_err_short_float[2] += 1.0f;
				}
			}			
			tempData[2] = char_temp;
			
			CanSendMsg(Joint[NKY].CAN_channel, Joint[NKY].Ref_txdf, tempData, 3, 0);
			
			
			break;		
				
		case EJMC4:
			short_temp = (short)(Joint[RF1].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[0] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[RF2].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[1] = (unsigned char)(short_currentPulse & 0x000000FF);
			
			short_temp = (short)(Joint[RF3].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[2] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[RF4].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[3] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[RF5].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[4] = (unsigned char)(short_currentPulse & 0x000000FF);
			/*
			if(MotionTimeCurrent>=1 && MotionTimeCurrent < 5)
							tempData[1] = 0x81;//(unsigned char)(short_currentPulse & 0x000000FF);
						else if(MotionTimeCurrent >=101 && MotionTimeCurrent <105)
							tempData[1] = 0x7F;
						else */
			

			CanSendMsg(Joint[RF1].CAN_channel, Joint[RF1].Ref_txdf, tempData, 5, 0);

			break;

		case EJMC5:
			short_temp = (short)(Joint[LF1].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[0] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[LF2].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[1] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[LF3].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[2] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[LF4].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[3] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[LF5].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,Control_Mode);
			tempData[4] = (unsigned char)(short_currentPulse & 0x000000FF);

			/*
			if(MotionTimeCurrent>=1 && MotionTimeCurrent < 5)
				tempData[1] = 0x81;//(unsigned char)(short_currentPulse & 0x000000FF);
			else if(MotionTimeCurrent >=101 && MotionTimeCurrent <105)
				tempData[1] = 0x7F;
			else
			*/

			CanSendMsg(Joint[LF1].CAN_channel, Joint[LF1].Ref_txdf, tempData, 5, 0);
			break;
		}
}
/******************************************************************************/



/******************************************************************************/
unsigned long SignConvention(long _input)
{
	if (_input < 0) return (unsigned long)( ((-_input)&0x007FFFFF) | (1<<23) );
	else return (unsigned long)_input;
			
}
/******************************************************************************/




/******************************************************************************/
unsigned short SignConventionFinger(short _input,unsigned char _type)
{
	if(_type == 0x00) // Position
	{
		if (_input < 0) return ((_input)&0x000000FF);
		else return (unsigned short)_input;
	}
	else if(_type == 0x01) // Current
	{
		if (_input < 0) return (unsigned short)( ((-_input)&0x0000007F) | (1<<7) );
		else return (unsigned short)_input;
	}
	else return 0x00;
}
/******************************************************************************/




/******************************************************************************/
void InverseKinematics(float _pos[6], float _angle[])
{
	unsigned char i;
	float angle[6];

	// knee
	angle[3] = PI-acosf((LENGTH_THIGH*LENGTH_THIGH+LENGTH_CALF*LENGTH_CALF-(_pos[0]*_pos[0]+_pos[1]*_pos[1]+_pos[2]*_pos[2]))/(2.0f*LENGTH_THIGH*LENGTH_CALF));
	// hip roll
	angle[1] = atan2f(_pos[1], -_pos[2]);
	// hip pitch
	angle[2] = asinf((-sinf(angle[3])*LENGTH_CALF*(-_pos[1]*sinf(angle[1])+_pos[2]*cosf(angle[1]))+_pos[0]*(cosf(angle[3])*LENGTH_CALF+LENGTH_THIGH))/(-_pos[0]*_pos[0]-(_pos[1]*sinf(angle[1])-_pos[2]*cosf(angle[1]))*(_pos[1]*sinf(angle[1])-_pos[2]*cosf(angle[1]))));
	// hip yaw
	angle[0] = DEG2RAD*_pos[5];
	// ankle roll
	angle[5] = -angle[1];
	// ankle pitch
	angle[4] = -angle[2]-angle[3];

	for(i=0 ; i<6 ; i++) _angle[i] = RAD2DEG*angle[i];
}
/******************************************************************************/





/******************************************************************************/
void ForwardKinematics(float _angle[6], float _pos[])
{
	
}
/******************************************************************************/





/******************************************************************************/
void ReadEncoder(unsigned char _canChannel)
{
	unsigned char tempData[8];

	if(_canChannel == CAN0)
	{
		ReadMBData(ENC0_RXDF, tempData);
		Joint[RHY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RHR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC1_RXDF, tempData);
		Joint[RHP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC2_RXDF, tempData);
		Joint[RKN].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC3_RXDF, tempData);
		Joint[RAP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RAR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC4_RXDF, tempData);
		Joint[LHY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LHR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC5_RXDF, tempData);
		Joint[LHP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC6_RXDF, tempData);
		Joint[LKN].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC7_RXDF, tempData);
		Joint[LAP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LAR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
	}
	else if(_canChannel == CAN1)
	{
		ReadMBData(ENC8_RXDF, tempData);
		Joint[RSP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RSR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC9_RXDF, tempData);
		Joint[RSY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[REB].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC10_RXDF, tempData);
		Joint[LSP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LSR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC11_RXDF, tempData);
		Joint[LSY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LEB].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC_E0_RXDF, tempData);
		Joint[RWY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RWP].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC_E1_RXDF, tempData);
		Joint[LWY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LWP].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC_E2_RXDF, tempData);
		Joint[NKY].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));
		Joint[NK1].EncoderValue = (int)((short)((tempData[2])|(tempData[3]<<8)));
		Joint[NK2].EncoderValue = (int)((short)((tempData[4])|(tempData[5]<<8)));

		ReadMBData(ENC_E3_RXDF, tempData);
		Joint[WST].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));
	}
	else;
	
	// only for debugging
	unsigned char i;
	for(i=RHY ; i<NO_OF_JOINT; i++) pSharedMemory->Joint_debug[i] = Joint[i];
}
/******************************************************************************/




/******************************************************************************/
void ReadFT(unsigned char _canChannel)
{
	unsigned char tempData[8];
	unsigned int buff_no;


	if(_canChannel == CAN0)	// for ankle F/T sensor
	{
		buff_no = GetBuffno(SENSOR_FT0_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_FT0_RXDF, tempData);
			FTSensor[RFFT].Mx = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
			FTSensor[RFFT].My = -(float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
			FTSensor[RFFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f; 
			FTSensor[RFFT].Filtered_Mx = FTSensor[RFFT].CutOffFeq*FTSensor[RFFT].Filtered_Mx + (1.0f-FTSensor[RFFT].CutOffFeq)*FTSensor[RFFT].Mx;
			FTSensor[RFFT].Filtered_My = FTSensor[RFFT].CutOffFeq*FTSensor[RFFT].Filtered_My + (1.0f-FTSensor[RFFT].CutOffFeq)*FTSensor[RFFT].My;
			FTSensor[RFFT].Filtered_Fz = FTSensor[RFFT].CutOffFeq*FTSensor[RFFT].Filtered_Fz + (1.0f-FTSensor[RFFT].CutOffFeq)*FTSensor[RFFT].Fz;
		}

		buff_no = GetBuffno(SENSOR_FT1_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_FT1_RXDF, tempData);
			FTSensor[LFFT].Mx = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
			FTSensor[LFFT].My = -(float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
			FTSensor[LFFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
			FTSensor[LFFT].Filtered_Mx = FTSensor[LFFT].CutOffFeq*FTSensor[LFFT].Filtered_Mx + (1.0f-FTSensor[LFFT].CutOffFeq)*FTSensor[LFFT].Mx;
			FTSensor[LFFT].Filtered_My = FTSensor[LFFT].CutOffFeq*FTSensor[LFFT].Filtered_My + (1.0f-FTSensor[LFFT].CutOffFeq)*FTSensor[LFFT].My;
			FTSensor[LFFT].Filtered_Fz = FTSensor[LFFT].CutOffFeq*FTSensor[LFFT].Filtered_Fz + (1.0f-FTSensor[LFFT].CutOffFeq)*FTSensor[LFFT].Fz;
		}

		buff_no = GetBuffno(SENSOR_AD0_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_AD0_RXDF, tempData);
			FTSensor[RFFT].dAccRoll =  ((tempData[0] | tempData[1] << 8) >> 0) - FTSensor[RFFT].dAccRoll_Offset;
			FTSensor[RFFT].dAccPitch = (((tempData[2] | tempData[3] << 8) >> 0)) - FTSensor[RFFT].dAccPitch_Offset;
			FTSensor[RFFT].AccRoll = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*(float)INT_TIME/1000.0f)*FTSensor[RFFT].AccRollOld + 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f*(float)FTSensor[RFFT].dAccRoll);
			FTSensor[RFFT].AccPitch = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[RFFT].AccPitchOld + 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f*FTSensor[RFFT].dAccPitch);
			FTSensor[RFFT].AccRollOld = FTSensor[RFFT].AccRoll;
			FTSensor[RFFT].AccPitchOld = FTSensor[RFFT].AccPitch;
			FTSensor[RFFT].Pitch = FTSensor[RFFT].AccPitch/FTSensor[RFFT].SF_Pitch;
			FTSensor[RFFT].Roll = FTSensor[RFFT].AccRoll/FTSensor[RFFT].SF_Roll;
			FTSensor[RFFT].VelRoll = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[RFFT].VelRollOld + INT_TIME/1000.0f*FTSensor[RFFT].AccRollOld);
			FTSensor[RFFT].VelRollOld = FTSensor[RFFT].VelRoll;
			FTSensor[RFFT].VelPitch = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[RFFT].VelPitchOld + INT_TIME/1000.0f*FTSensor[RFFT].AccPitchOld);
			FTSensor[RFFT].VelPitchOld = FTSensor[RFFT].VelPitch;
		}

		buff_no = GetBuffno(SENSOR_AD1_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_AD1_RXDF, tempData);
			FTSensor[LFFT].dAccRoll =  ((tempData[0] | tempData[1] << 8) >> 0) - FTSensor[LFFT].dAccRoll_Offset;
			FTSensor[LFFT].dAccPitch = (((tempData[2] | tempData[3] << 8) >> 0)) - FTSensor[LFFT].dAccPitch_Offset;
			FTSensor[LFFT].AccRoll = (float)((1.0f - 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].AccRollOld + 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f*FTSensor[LFFT].dAccRoll);
			FTSensor[LFFT].AccPitch = (float)((1.0f - 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].AccPitchOld + 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f*FTSensor[LFFT].dAccPitch);
			FTSensor[LFFT].AccRollOld = FTSensor[LFFT].AccRoll;
			FTSensor[LFFT].AccPitchOld = FTSensor[LFFT].AccPitch;
			FTSensor[LFFT].Pitch = FTSensor[LFFT].AccPitch/FTSensor[LFFT].SF_Pitch;
			FTSensor[LFFT].Roll = FTSensor[LFFT].AccRoll/FTSensor[RFFT].SF_Roll;
			FTSensor[LFFT].VelRoll = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].VelRollOld + INT_TIME/1000.0f*FTSensor[LFFT].AccRollOld);
			FTSensor[LFFT].VelRollOld = FTSensor[LFFT].VelRoll;
			FTSensor[LFFT].VelPitch = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].VelPitchOld + INT_TIME/1000.0f*FTSensor[LFFT].AccPitchOld);
			FTSensor[LFFT].VelPitchOld = FTSensor[LFFT].VelPitch;
		}
		
/*		ReadMBData(SENSOR_AD1_RXDF, tempData);
		FTSensor[LFFT].VelRoll =  (float)((int)(tempData[0]<<16|(tempData[1]<<24))>>16);
		FTSensor[LFFT].VelPitch = (float)((int)(tempData[2]<<16|(tempData[3]<<24))>>16);
*/
  }
	else if(_canChannel == CAN1)	// for wrist F/T sensor
	{
		buff_no = GetBuffno(SENSOR_FT2_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_FT2_RXDF, tempData);
			FTSensor[RWFT].Mx = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
			FTSensor[RWFT].My = -(float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
			FTSensor[RWFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
		}
/*		FTSensor[RWFT].dMx =  (short)((tempData[0])|(tempData[1]<<8));
		FTSensor[RWFT].dMy =  (short)((tempData[2])|(tempData[3]<<8));
		FTSensor[RWFT].dFz =  (short)((tempData[4])|(tempData[5]<<8));
		FTSensor[RWFT].Mx = FTSensor[RWFT].SF_Mx*(float)FTSensor[RWFT].dMx;
		FTSensor[RWFT].My = FTSensor[RWFT].SF_My*(float)FTSensor[RWFT].dMy;
		FTSensor[RWFT].Fz = FTSensor[RWFT].SF_Fz*(float)FTSensor[RWFT].dFz;
		FTSensor[RWFT].Filtered_Mx = FTSensor[RWFT].CutOffMx*FTSensor[RWFT].Filtered_Mx + (1.0f-FTSensor[RWFT].CutOffMx)*FTSensor[RWFT].Mx;
		FTSensor[RWFT].Filtered_My = FTSensor[RWFT].CutOffMy*FTSensor[RWFT].Filtered_My + (1.0f-FTSensor[RWFT].CutOffMy)*FTSensor[RWFT].My;
		FTSensor[RWFT].Filtered_Fz = FTSensor[RWFT].CutOffFz*FTSensor[RWFT].Filtered_Fz + (1.0f-FTSensor[RWFT].CutOffFz)*FTSensor[RWFT].Fz;

 */
		buff_no = GetBuffno(SENSOR_FT3_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_FT3_RXDF, tempData);
			FTSensor[LWFT].Mx = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
			FTSensor[LWFT].My = -(float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
			FTSensor[LWFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
		}
	}
	else;
			

	// only for debugging
	unsigned char i;
	for(i=RFFT ; i<NO_OF_FT; i++) pSharedMemory->FT_debug[i] = FTSensor[i];
}
/******************************************************************************/




/******************************************************************************/
unsigned char GetZMP(FT _rightFootFTSensor, FT _leftFootFTSensor, WALKING_INFO _walkingInfo[2][4], float _zmp[], float _filteredZMP[])
{
	// _zmp[0]: x-direction ZMP based on whole legs
	// _zmp[1]: y-direction ZMP based on whole legs
	// _zmp[2]: x-direction ZMP based on right leg
	// _zmp[3]: y-direction ZMP based on right leg
	// _zmp[3]: x-direction ZMP based on left leg
	// _zmp[4]: y-direction ZMP based on left leg
	
	unsigned char _state, i;
	float totalMx, totalMy;
	float filteredZMPOld[6];

	if(_rightFootFTSensor.Fz > 30.f)
	{			
		if(_leftFootFTSensor.Fz> 30.f)		// Double Support Phase
		{
			totalMx = _rightFootFTSensor.Mx+_leftFootFTSensor.Mx+(_walkingInfo[LEFT][Y].RefPatternCurrent+0.5f*PELVIS_WIDTH)*_leftFootFTSensor.Fz+(_walkingInfo[RIGHT][Y].RefPatternCurrent-0.5f*PELVIS_WIDTH)*_rightFootFTSensor.Fz;
			totalMy = _rightFootFTSensor.My+_leftFootFTSensor.My-_walkingInfo[LEFT][X].RefPatternCurrent*_leftFootFTSensor.Fz-_walkingInfo[RIGHT][X].RefPatternCurrent*_rightFootFTSensor.Fz;
			_zmp[0] = -1000.f*totalMy/(_leftFootFTSensor.Fz+_rightFootFTSensor.Fz);
			_zmp[1] = 1000.f*totalMx/(_leftFootFTSensor.Fz+_rightFootFTSensor.Fz);
			_zmp[2] = -1000.0f*_rightFootFTSensor.My/_rightFootFTSensor.Fz;
			_zmp[3] = 1000.0f*_rightFootFTSensor.Mx/_rightFootFTSensor.Fz;
			_zmp[4] = -1000.0f*_leftFootFTSensor.My/_leftFootFTSensor.Fz;
			_zmp[5] = 1000.0f*_leftFootFTSensor.Mx/_leftFootFTSensor.Fz;

			_state = DSP;
		}
		else	// Right leg Single Support Phase
		{
			_zmp[2] = -1000.0f*_rightFootFTSensor.My/_rightFootFTSensor.Fz;
			_zmp[3] = 1000.0f*_rightFootFTSensor.Mx/_rightFootFTSensor.Fz;
			_zmp[0] = _walkingInfo[RIGHT][X].RefPatternCurrent*1000.f + _zmp[2];
			_zmp[1] = (_walkingInfo[RIGHT][Y].RefPatternCurrent-0.5f*PELVIS_WIDTH)*1000.f + _zmp[3];
			_zmp[4] = _zmp[5] = 0.0f;

			_state = RIGHT_SSP;
		}			
	}
	else if(_leftFootFTSensor.Fz> 30.f)	// Left leg Single Support Phase
	{
		_zmp[4] = -1000.0f*_leftFootFTSensor.My/_leftFootFTSensor.Fz;
		_zmp[5] = 1000.0f*_leftFootFTSensor.Mx/_leftFootFTSensor.Fz;
		_zmp[0]  = _walkingInfo[LEFT][X].RefPatternCurrent*1000.0f + _zmp[4];
		_zmp[1]  = (_walkingInfo[LEFT][Y].RefPatternCurrent+0.5f*PELVIS_WIDTH)*1000.0f + _zmp[5];	
		_zmp[2] = _zmp[3] = 0.0f;

		_state = LEFT_SSP;
	}
	else { _zmp[0] = _zmp[1] = _zmp[2] = _zmp[3] = _zmp[4] = _zmp[5] = 0.0f; _state = NO_LANDING; }


	for(i=0 ; i<6 ; i++)
	{
		filteredZMPOld[i] = _filteredZMP[i];
		_filteredZMP[i] = (1.0f-2.0f*PI*5.0f*(float)INT_TIME/1000.0f)*filteredZMPOld[i] + (2.0f*PI*2.0f*(float)INT_TIME/1000.0f)*_zmp[i];
	}
		

	// only for debugging
	for(i=0 ; i<6 ; i++) pSharedMemory->ZMP[i] = _filteredZMP[i];


	return _state;
}
/******************************************************************************/





/******************************************************************************/
void ReadIMU(void)
{
	unsigned char tempData[8];
	int buff_no;

	buff_no = GetBuffno(SENSOR_IMU0_RXDF);
	if(MB[buff_no].status != NODATA)
	{
		ReadMBData(SENSOR_IMU0_RXDF, tempData);
		IMUSensor[CENTERIMU].Roll = (float)((short)((tempData[1]<<8)|tempData[0]))/100.0f + IMUSensor[CENTERIMU].RollOffset; 
		IMUSensor[CENTERIMU].Pitch = (float)((short)((tempData[3]<<8)|tempData[2]))/100.0f + IMUSensor[CENTERIMU].PitchOffset; 
		IMUSensor[CENTERIMU].Roll_Velocity = (float)((short)((tempData[5]<<8)|tempData[4]))/100.0f; 
		IMUSensor[CENTERIMU].Pitch_Velocity = (float)((short)((tempData[7]<<8)|tempData[6]))/100.0f; 
	}
	// only for debugging
	unsigned char i;
	for(i=CENTERIMU ; i<NO_OF_IMU; i++) pSharedMemory->IMU_debug[i] = IMUSensor[i];
}
/******************************************************************************/





/******************************************************************************/
bool RequestEncoder(unsigned char _boardID)
{
	unsigned char tempData[8];

	tempData[0] = _boardID;
	tempData[1] = SendEncoder;

	switch(_boardID)
	{
	case JMC0:
	case JMC1:
	case JMC2:
	case JMC3:
	case JMC4:
	case JMC5:
	case JMC6:
	case JMC7:
	case EJMC3:
		CanSendMsg(0, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	case JMC8:
	case JMC9:
	case JMC10:
	case JMC11:
	case EJMC0:
	case EJMC1:
	case EJMC2:
	case EJMC4:
	case EJMC5:
		CanSendMsg(1, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	}

	RtWprintf(L"\n>>> Wrong board ID(RequestEncoder)..!!");
	return false; 
}
/******************************************************************************/





/******************************************************************************/
bool RequestCurrent(unsigned char _boardID)
{
	unsigned char tempData[8];

	tempData[0] = _boardID;
	tempData[1] = SendCurrent;

	switch(_boardID)
	{
	case JMC0:
	case JMC1:
	case JMC2:
	case JMC3:
	case JMC4:
	case JMC5:
	case JMC6:
	case JMC7:
	case EJMC3:
		CanSendMsg(0, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	case JMC8:
	case JMC9:
	case JMC10:
	case JMC11:
	case EJMC0:
	case EJMC1:
	case EJMC2:
	case EJMC4:
	case EJMC5:
		CanSendMsg(1, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	}

	RtWprintf(L"\n>>> Wrong board ID(RequestCurrent)..!!");
	return false;
}
/******************************************************************************/





/******************************************************************************/
bool RequestSensor(unsigned char _canChannel)
{
	unsigned char tempData[8];
 	
 	if(_canChannel >= 2) { RtWprintf(L"\n>>> Wrong CAN channel number(RequestSensor)..!!"); return false; }

	if(_canChannel == CAN0)
	{
		tempData[0] = 0xFF;//0x00;//0xFF;//0x11;//0xFF;
		tempData[1] = 0x03;
		CanSendMsg(_canChannel, SEND_SENSOR_TXDF, tempData, 2, 0);

		tempData[0] = 0x03;
		tempData[1] = 0x00;
		CanSendMsg(_canChannel, SEND_SENSOR_TXDF, tempData, 2, 0);
	}
	else if(_canChannel == CAN1)
	{
		tempData[0] = 0xFF;
		tempData[1] = 0x00;
		CanSendMsg(_canChannel, SEND_SENSOR_TXDF, tempData, 2, 0);
	}
	else { RtWprintf(L"\n>>> Wrong CAN channel number(RequestSensor)..!!"); return false; }

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool SetFTParameter(unsigned char _ftID, FT _ftSensor)
{
	if(_ftID > NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(SetFTParameter)..!!"); return false; }
	else FTSensor[_ftID] = _ftSensor;
	return true;
}
/******************************************************************************/




/******************************************************************************/
bool GetFTParameter(unsigned char _ftID, FT* _ftSensor)
{
	if(_ftID > NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(GetFTParameter)..!!"); return false; }
	else *_ftSensor = FTSensor[_ftID];

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool NullFTSensor(unsigned char _ftID, unsigned char _mode)
{
	unsigned char tempData[8];

	if(_ftID>NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(NullFTSensor)..!!"); return false; }
	else
	{
		tempData[0] = FTSensor[_ftID].Controller_NO;
		tempData[1] = NullCMD;
		tempData[2] = _mode;

		CanSendMsg(FTSensor[_ftID].CAN_channel, CMD_TXDF, tempData, 3, 0);
	}

	return true;
}
/******************************************************************************/





/******************************************************************************/
bool NullFootAngleSensor(FT _ftSensor[])
{	
	unsigned char tempData[8];

	tempData[0] = _ftSensor[RFFT].Controller_NO;
	tempData[1] = NullCMD;
	tempData[2] = 0x04;
	CanSendMsg(_ftSensor[RFFT].CAN_channel, CMD_TXDF, tempData, 3, 0);
	
	Sleep(1);

	tempData[0] = _ftSensor[LFFT].Controller_NO;
	tempData[1] = NullCMD;
	tempData[2] = 0x04;
	CanSendMsg(_ftSensor[LFFT].CAN_channel, CMD_TXDF, tempData, 3, 0);

	return true;
}
/******************************************************************************/





/******************************************************************************/
bool NullIMUSensor(unsigned char _imuID, unsigned char _mode)
{
	unsigned char tempData[8];

	if(_imuID>NO_OF_IMU) { RtWprintf(L"\n>>> Wrong FT sensor ID(NullIMUSensor)..!!"); return false; }
	else
	{
		tempData[0] = IMUSensor[_imuID].Controller_NO;
		tempData[1] = NullCMD;
		tempData[2] = _mode;

		CanSendMsg(IMUSensor[_imuID].CAN_channel, CMD_TXDF, tempData, 3, 0);
	}

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool PrintFTParameter(unsigned char _ftID)
{
	if(_ftID > NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(PrintFTParameter)..!!"); return false; }
	else
	{
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> FT sensor(%d) parameter information", _ftID);
		RtWprintf(L"\n>>> Raw digit value Mx: %d", FTSensor[_ftID].dMx);
		RtWprintf(L"\n>>> Raw digit value My: %d", FTSensor[_ftID].dMy);
		RtWprintf(L"\n>>> Raw digit value Fz: %d", FTSensor[_ftID].dFz);
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Numerical value Mx: %d", (int)(FTSensor[_ftID].Mx*1000.0f));
		RtWprintf(L"\n>>> Numerical value My: %d", (int)(FTSensor[_ftID].My*1000.0f));
		RtWprintf(L"\n>>> Numerical value Fz: %d", (int)(FTSensor[_ftID].Fz*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Filtered value Mx: %d", (int)(FTSensor[_ftID].Filtered_Mx*1000.0f));
		RtWprintf(L"\n>>> Filtered value My: %d", (int)(FTSensor[_ftID].Filtered_My*1000.0f));
		RtWprintf(L"\n>>> Filtered value Fz: %d", (int)(FTSensor[_ftID].Filtered_Fz*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Cut-off frequency: %d", (int)(FTSensor[_ftID].CutOffFeq*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Decouple matrix");
		RtWprintf(L"\n>>> %d\t %d\t %d", (int)(FTSensor[_ftID].DCMat[0][0]*1000.0f), (int)(FTSensor[_ftID].DCMat[0][1]*1000.0f), (int)(FTSensor[_ftID].DCMat[0][2]*1000.0f));
		RtWprintf(L"\n>>> %d\t %d\t %d", (int)(FTSensor[_ftID].DCMat[1][0]*1000.0f), (int)(FTSensor[_ftID].DCMat[1][1]*1000.0f), (int)(FTSensor[_ftID].DCMat[1][2]*1000.0f));
		RtWprintf(L"\n>>> %d\t %d\t %d", (int)(FTSensor[_ftID].DCMat[2][0]*1000.0f), (int)(FTSensor[_ftID].DCMat[2][1]*1000.0f), (int)(FTSensor[_ftID].DCMat[2][2]*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> CAN channel number: %d", FTSensor[_ftID].CAN_channel);
		
		GetFTParameter(_ftID, &pSharedMemory->FTsensor);

		return true;
	}
}
/******************************************************************************/





/******************************************************************************/
bool SetIMUParameter(unsigned char _imuID, IMU _imuSensor)
{
	if(_imuID > NO_OF_IMU) { RtWprintf(L"\n>>> Wrong IMU ID(SetIMUParameter)..!!"); return false; }
	else IMUSensor[_imuID] = _imuSensor;

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool GetIMUParameter(unsigned char _imuID, IMU* _imuSensor)
{
	if(_imuID > NO_OF_IMU) { RtWprintf(L"\n>>> Wrong IMU ID(GetIMUParameter)..!!"); return false; }
	else *_imuSensor = IMUSensor[_imuID];

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool PrintIMUParameter(unsigned char _imuID)
{
	if(_imuID > NO_OF_IMU) { RtWprintf(L"\n>>> Wrong IMU ID(PrintIMUParameter)..!!"); return false; }
	else
	{
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> IMU(%d) parameter information", _imuID);
		RtWprintf(L"\n>>> Roll angle value: %d", (int)(IMUSensor[_imuID].Roll*1000.0f));
		RtWprintf(L"\n>>> Pitch angle value: %d", (int)(IMUSensor[_imuID].Pitch*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Roll angular velocity: %d", (int)(IMUSensor[_imuID].Roll_Velocity*1000.0f));
		RtWprintf(L"\n>>> Pitch angulat velocity: %d", (int)(IMUSensor[_imuID].Pitch_Velocity*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> CAN channel number: %d", IMUSensor[_imuID].CAN_channel);
		
		GetIMUParameter(_imuID, &pSharedMemory->IMUsensor);

		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
void ZMPInitialization(float _refZMP[2], float _zmp[], IMU _imu, FT _rightFT, FT _leftFT,  WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _command)
{
	unsigned char i;

	float KI;
	float zmpTemp[2], heightTemp, pitchHipTemp, pitchAnkleTemp, rollAnkleTemp[2];
	static float torsoSum[2] = {0.0f, 0.0f};
	static float heightSum = 0.0f;
	static float pitchHipSum = 0.0f;
	static float pitchAnkleSum = 0.0f;
	static float rollAnkleSum[2] = {0.0f, 0.0f};

	// _zmpInit[]
	// 0: torso x-direction
	// 1: torso y-direction
	// 2: z-direction of the foot
	// 3: hip pitch angle
	// 4: ankle pitch angle
	// 5: right ankle roll angle
	// 6: left ankle roll angle
	
	
	
	// ZMP initialization using the torso center position	
	//KI = 0.002f/INT_TIME;
	KI = 0.004f/INT_TIME;
	
	for(i=0 ; i<2 ; i++)
	{
		torsoSum[i] += _refZMP[i] - _zmp[i];

		if(torsoSum[i] > 90.0f/KI) torsoSum[i] = 90.0f/KI;
		else if(torsoSum[i] < -190.0f/KI) torsoSum[i] = -90.0f/KI;

		zmpTemp[i] = KI*torsoSum[i];

		if(_command == 0x00) { torsoSum[i] = 0.0f; zmpTemp[i] = 0.0f; }
	}

	_walkingInfo[RIGHT][X].RefPosFF = -zmpTemp[0]/1000.0f;
	_walkingInfo[LEFT][X].RefPosFF = -zmpTemp[0]/1000.0f;
	_walkingInfo[RIGHT][Y].RefPosFF = -zmpTemp[1]/1000.0f;
	_walkingInfo[LEFT][Y].RefPosFF = -zmpTemp[1]/1000.0f;
	
	

	// body roll angle initialization using the foot position(z-direction)
	//KI = 0.0004f;
	KI = 0.0008f;
	//heightTemp = 0.1f*_imu.Roll + KI*heightSum;
	heightTemp = 0.05f*_imu.Roll + KI*heightSum;
	heightSum += _imu.Roll;

	if(heightSum > 10.0f/KI) heightSum = 10.0f/KI;
	else if(heightSum < -10.0f/KI) heightSum = -10.0f/KI;
	if(heightTemp > 10.0f) heightTemp = 10.0f;
	else if(heightTemp < -10.0f) heightTemp = -10.0f;

	if(_command == 0x00) { heightSum = 0.0f; heightTemp = 0.0f; }

	_walkingInfo[RIGHT][Z].RefPosFF = -heightTemp/1000.0f;
	_walkingInfo[LEFT][Z].RefPosFF = heightTemp/1000.0f;



	// body pitch angle initialization using the hip pitch angles
	//KI = 0.0004f;
	KI = 0.0008f;
	pitchHipTemp = KI*pitchHipSum;
	pitchHipSum += _imu.Pitch;
	
	if(pitchHipSum > 10.0f/KI) pitchHipSum = 10.0f/KI;
	else if(pitchHipSum < -10.0f/KI) pitchHipSum = -10.0f/KI;
	
	if(pitchHipTemp > 10.0f) pitchHipTemp = 10.0f;
	else if(pitchHipTemp < -10.0f) pitchHipTemp = -10.0f;
	
	if(_command == 0x00) { pitchHipSum = 0.0f; pitchHipTemp = 0.0f; }

	_joint[RHP].RefAngleFF = pitchHipTemp;
	_joint[LHP].RefAngleFF = pitchHipTemp;
	
	



	// moment at ankle(My) initialization using ankle pitch angles
	//KI=0.0001f;
	KI = 0.0002f;
	pitchAnkleTemp = KI*pitchAnkleSum;
	pitchAnkleSum += _rightFT.My-_leftFT.My;
	
	if(pitchAnkleSum > 5.0f/KI) pitchAnkleSum = 5.0f/KI;
	else if(pitchAnkleSum < -5.0f/KI) pitchAnkleSum = -5.0f/KI;
	
	if(pitchAnkleTemp > 5.0f) pitchAnkleTemp = 5.0f;
	else if(pitchAnkleTemp < -5.0f) pitchAnkleTemp = -5.0f;

	if(_command == 0x00) { pitchAnkleSum = 0.0f; pitchAnkleTemp; }
	_joint[RAP].RefAngleFF = pitchAnkleTemp;




	// moment at ankle(Mx) initialization using ankle roll angles
	//KI = 0.01f/INT_TIME;
	KI = 0.02f/INT_TIME;
	for(i=0 ; i<2 ; i++)
	{
		rollAnkleTemp[i] = KI*rollAnkleSum[i];
		if(i==RIGHT) rollAnkleSum[i] += _rightFT.Mx;
		else rollAnkleSum[i] += _leftFT.Mx;
	
		if(rollAnkleSum[i] > 5.0f/KI) rollAnkleSum[i] = 5.0f/KI;
		else if(rollAnkleSum[i] < -5.0f/KI) rollAnkleSum[i] = -5.0f/KI;
	
		if(rollAnkleTemp[i] > 5.0f) rollAnkleTemp[i] = 5.0f;
		else if(rollAnkleTemp[i] < -5.0f) rollAnkleTemp[i] = -5.0f;
	
		if(_command == 0) { rollAnkleSum[i] = 0.0f; rollAnkleTemp[i] = 0.0f; }
	}
	_joint[RAR].RefAngleFF = rollAnkleTemp[RIGHT];	_joint[LAR].RefAngleFF = rollAnkleTemp[LEFT];		
}
/******************************************************************************/





/******************************************************************************/
void GotoWalkReadyPos(void)
{
	unsigned char i, j;
	float readyAngle[NO_OF_JOINT];
	float readyPos[6] = {0.0f, 0.0f, WALK_READY_Z_POS, 0.0f, 0.0f, 0.0f};
	
	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++) WalkingInfo[i][j].RefPatternCurrent = readyPos[j];
	}

	InverseKinematics(readyPos, readyAngle);
	for(i=RHY ; i<=RAR ; i++) 
	{
		SetMoveJointAngle(i, readyAngle[i], 2000.0f, 0x01);
		Joint[i].WalkReadyAngle = readyAngle[i];
	}
	for(i=LHY ; i<=LAR ; i++) 
	{
		SetMoveJointAngle(i, readyAngle[i-LHY], 2000.0f, 0x01);
		Joint[i].WalkReadyAngle = readyAngle[i-LHY];
	}
	readyAngle[RSP] = 10.0f;
	readyAngle[RSR] = -10.0f;
	readyAngle[RSY] = 0.0f;
	readyAngle[REB] = -30.0f;
	readyAngle[RWY] = 0.0f;
	readyAngle[RWP] = 0.0f;
	readyAngle[LSP] = 10.0f;
	readyAngle[LSR] = 10.0f;
	readyAngle[LEB] = -30.0f;
	readyAngle[LWY] = 0.0f;
	readyAngle[LWP] = 0.0f;
	readyAngle[NKY] = 0.0f;
	readyAngle[NK1] = 0.0f;
	readyAngle[NK2] = 0.0f;
	readyAngle[WST] = 0.0f;

	for(i=RSP ; i<=WST;i++)
	{	
		SetMoveJointAngle(i, readyAngle[i], 2000.0f, 0x01);
		Joint[i].WalkReadyAngle = readyAngle[i];
	}

}
/******************************************************************************/




/******************************************************************************/
void GotoHomePos(void)
{
	unsigned char i, j;
	float homePos[6] = {0.0f, 0.0f, HOME_Z_POS, 0.0f, 0.0f, 0.0f};

	// clear ZMP initialization values
	ZMPInitialization(InitZMP, ZMP, IMUSensor[CENTERIMU], FTSensor[RFFT], FTSensor[LFFT], WalkingInfo, Joint, 0);
	DSPControl(ZMP, InitZMP, WalkingInfo, 0);

	for(i=RHY ; i<=LAR ; i++)
	{ 
		SetMoveJointAngle(i, 0.0f, 3000.0f, 0x01); 
		Joint[i].ControlDampAngleCurent = 0.0f;
		Joint[i].ControlDampAnglePast = 0.0f;
		Joint[i].ControlVibrationAngle = 0.0f; 
		Joint[i].ControlAngleFF = 0.0f;
		Joint[i].ControlAngleFFDelta = 0.0f;
		Joint[i].ControlAngleFFToGo = 0.0f;
	}
	SetMoveJointAngle(RSP, 0.0f, 2000.0f, 0x01);
	SetMoveJointAngle(RSR, 0.0f, 2000.0f, 0x01);
	SetMoveJointAngle(REB, 0.0f, 2000.0f, 0x01);
	SetMoveJointAngle(LSP, 0.0f, 2000.0f, 0x01);
	SetMoveJointAngle(LSR, 0.0f, 2000.0f, 0x01);
	SetMoveJointAngle(LEB, 0.0f, 2000.0f, 0x01);
	SetMoveJointAngle(WST, 0.0f, 2000.0f, 0x01);
	for(i=X ; i<=Yaw ; i++)
	{
		WalkingInfo[RIGHT][i].RefPosFF = 0.0f; WalkingInfo[RIGHT][i].ControlDSPZMP = 0.0f;
		WalkingInfo[LEFT][i].RefPosFF = 0.0f; WalkingInfo[LEFT][i].ControlDSPZMP = 0.0f;
	}

	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++) WalkingInfo[i][j].RefPatternCurrent = homePos[j];
	}
}
/******************************************************************************/






/******************************************************************************/
unsigned char LandingStateCheck(FT _ftSensor[], WALKING_INFO _walkingInfo[][4])
{	
	if( (_walkingInfo[RIGHT][Z].CurrentSequence==RIGHT_FOOT_DOWN_SET) || (_walkingInfo[RIGHT][Z].CurrentSequence==RIGHT_FOOT_DOWN_FORWARD_SET) )
	{
		if( (_walkingInfo[RIGHT][Z].MoveFlag==true) && (_ftSensor[RFFT].Fz>50.0f) ) return RIGHT_FOOT_EARLY_LANDING;
		else if( (_walkingInfo[RIGHT][Z].GoalTimeCount==_walkingInfo[RIGHT][Z].CurrentTimeCount) && (_ftSensor[RFFT].Fz<50.0f) ) return RIGHT_FOOT_LATE_LANDING;
		else return NORMAL_LANDING;
	}
	else if( (_walkingInfo[LEFT][Z].CurrentSequence==LEFT_FOOT_DOWN_SET) || (_walkingInfo[LEFT][Z].CurrentSequence==LEFT_FOOT_DOWN_FORWARD_SET) )
	{
		if( (_walkingInfo[LEFT][Z].MoveFlag==true) && (_ftSensor[LFFT].Fz>50.0f) ) return LEFT_FOOT_EARLY_LANDING;
		else if( (_walkingInfo[LEFT][Z].GoalTimeCount==_walkingInfo[LEFT][Z].CurrentTimeCount) && (_ftSensor[LFFT].Fz<50.0f) ) return LEFT_FOOT_LATE_LANDING;
		else return NORMAL_LANDING;
	}
	else return NORMAL_LANDING;
}
/******************************************************************************/





/******************************************************************************/
void DSPControl(float _ZMP[], float _refZMP[2], WALKING_INFO _walkingInfo[][4], unsigned char _command)
{
	unsigned char i;
	static float x1new[2] = {0.0f, 0.0f}, x2new[2] = {0.0f, 0.0f};
	static float x1[2] = {0.0f, 0.0f}, x2[2] = {0.0f, 0.0f};
	float delZMP[2] = {_ZMP[0]+_refZMP[0], _ZMP[1]+_refZMP[1]};
	float controlOutput[2];
	float zmpLimit = 45.0f;
	float dspLimit = 60.0f;

	const float adm[2][4] = {0.519417298104f, -2.113174135817f, 0.003674390567f, 0.994144954175f,		0.734481292627f, -1.123597885586f, 0.004304846434f, 0.997046817458f};
	const float bdm[2][2] = {0.003674390567f, 0.000010180763f,		0.004304846434f, 0.000011314544f};
	const float cdm[2][2] = {9.698104154920f, -126.605319064208f,		-0.555683927701f, -77.652283056185f};

	for(i=X ; i<=Y ; i++)
	{
		x1new[i] = adm[i][0]*x1[i] + adm[i][1]*x2[i] + bdm[i][0]*delZMP[i];
		x2new[i] = adm[i][2]*x1[i] + adm[i][3]*x2[i] + bdm[i][1]*delZMP[i];
		controlOutput[i] = cdm[i][0]*x1new[i] + cdm[i][1]*x2new[i];

		x1[i] = x1new[i];	x2[i] = x2new[i];

		if(controlOutput[i] > dspLimit) controlOutput[i] = dspLimit;
		else if(controlOutput[i] < -dspLimit) controlOutput[i] = -dspLimit;
		else;

		if(_command == 0x00) { x1[i] = x2[i] = x1new[i] = x2new[i] = controlOutput[i] = 0.0f; }
	}

	_walkingInfo[RIGHT][X].ControlDSPZMP = 1.0f*controlOutput[X]/1000.0f;
	_walkingInfo[RIGHT][Y].ControlDSPZMP = 1.0f*controlOutput[Y]/1000.0f;
	_walkingInfo[LEFT][X].ControlDSPZMP = 1.0f*controlOutput[X]/1000.0f;
	_walkingInfo[LEFT][Y].ControlDSPZMP = 1.0f*controlOutput[Y]/1000.0f;



	// ZMP initialization using the torso center position
	float zmpTemp[2];
	static float torsoSum[2] = {0.0f, 0.0f};
	
	//float KI = 0.002f/INT_TIME;
	float KI = 0.005f/INT_TIME;
	
	for(i=0 ; i<2 ; i++)
	{
		torsoSum[i] += _refZMP[i] - _ZMP[i];

		if(torsoSum[i] > zmpLimit/KI) torsoSum[i] = zmpLimit/KI;
		else if(torsoSum[i] < -zmpLimit/KI) torsoSum[i] = -zmpLimit/KI;

		zmpTemp[i] = KI*torsoSum[i];

		if(_command == 0x00) { torsoSum[i] = 0.0f; zmpTemp[i] = 0.0f; }
	}

	_walkingInfo[RIGHT][X].RefPosFF = -zmpTemp[0]/1000.0f;
	_walkingInfo[LEFT][X].RefPosFF = -zmpTemp[0]/1000.0f;
	_walkingInfo[RIGHT][Y].RefPosFF = -zmpTemp[1]/1000.0f;
	_walkingInfo[LEFT][Y].RefPosFF = -zmpTemp[1]/1000.0f;
}
/******************************************************************************/





/******************************************************************************/
void DampingControl(IMU _imuSensor[], JOINT _joint[], unsigned char _command)
{
	float tempControlAngle[2];
	float limitAngle;
	float gain[6];
	float controlDampCutoff = 1.0f;

	// without case(pitch	roll)
// 	gain[0] = 0.2f;		gain[3] = 0.2f;
// 	gain[1] = 1.0f;		gain[4] = 1.0f;
// 	gain[2] = 0.4f;		gain[5] = 0.5f;
	// with case
// 	gain[0] = 0.3f;		gain[3] = 0.3f;
// 	gain[1] = 1.0f;		gain[4] = 1.0f;
// 	gain[2] = 0.4f;		gain[5] = 0.5f;

	unsigned char i;
	for(i=0 ; i<6 ; i++) gain[i] = dampingGain[i];
	

	tempControlAngle[0] = gain[0]*(gain[1]*-_imuSensor[CENTERIMU].Pitch + gain[2]*_imuSensor[CENTERIMU].Pitch_Velocity);
	tempControlAngle[1] = gain[3]*(gain[4]*-_imuSensor[CENTERIMU].Roll + gain[5]*_imuSensor[CENTERIMU].Roll_Velocity);
	
	
	

	switch(_command)
	{
	case RIGHT_SSP:
		_joint[RAP].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[RAP].ControlDampAnglePast = _joint[RAP].ControlDampAngleCurent;
		_joint[RAR].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[RAR].ControlDampAnglePast = _joint[RAR].ControlDampAngleCurent;
		break;
	case LEFT_SSP:
		_joint[LAP].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[LAP].ControlDampAnglePast = _joint[LAP].ControlDampAngleCurent;
		_joint[LAR].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[LAR].ControlDampAnglePast = _joint[LAR].ControlDampAngleCurent;
		break;
	case DSP:
		tempControlAngle[0] = tempControlAngle[1] = 0.0f;
		_joint[RAP].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[RAP].ControlDampAnglePast = _joint[RAP].ControlDampAngleCurent;
		_joint[RAR].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[RAR].ControlDampAnglePast = _joint[RAR].ControlDampAngleCurent;
		_joint[LAP].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[LAP].ControlDampAnglePast = _joint[LAP].ControlDampAngleCurent;
		_joint[LAR].ControlDampAngleCurent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[LAR].ControlDampAnglePast = _joint[LAR].ControlDampAngleCurent;
		break;
	default:
		break;
	}

	limitAngle = 10.0f;
	if(_joint[RAP].ControlDampAngleCurent > limitAngle) _joint[RAP].ControlDampAngleCurent = limitAngle;
	else if(_joint[RAP].ControlDampAngleCurent < -limitAngle) _joint[RAP].ControlDampAngleCurent = -limitAngle;
	if(_joint[LAP].ControlDampAngleCurent > limitAngle) _joint[LAP].ControlDampAngleCurent = limitAngle;
	else if(_joint[LAP].ControlDampAngleCurent < -limitAngle) _joint[LAP].ControlDampAngleCurent = -limitAngle;
	
	if(_joint[RAR].ControlDampAngleCurent > limitAngle) _joint[RAR].ControlDampAngleCurent = limitAngle;
	else if(_joint[RAR].ControlDampAngleCurent < -limitAngle) _joint[RAR].ControlDampAngleCurent = -limitAngle;
	if(_joint[LAR].ControlDampAngleCurent > limitAngle) _joint[LAR].ControlDampAngleCurent = limitAngle;
	else if(_joint[LAR].ControlDampAngleCurent < -limitAngle) _joint[LAR].ControlDampAngleCurent = -limitAngle;

	_joint[RAP].ControlDampAnglePast = _joint[RAP].ControlDampAngleCurent;
	_joint[LAP].ControlDampAnglePast = _joint[LAP].ControlDampAngleCurent;

}
/******************************************************************************/





/******************************************************************************/
void VibrationControl(FT _ftSensor[], JOINT _joint[], unsigned char _command)
{	
	float temp, limit;

	static float X_Roll_R[2] = {0.0f, 0.0f};
	static float X_Roll_New_R[2] = {0.0f, 0.0f};
	static float Y_Roll_R = 0.0f;

	static float X_Roll_L[2] = {0.0f, 0.0f};
	static float X_Roll_New_L[2] = {0.0f, 0.0f};
	static float Y_Roll_L = 0.0f;

	const float A_Roll[4] = {0.259455231689f, -3.299530027401f, 0.005542044942f, 0.979921074166f}; 
	const float B_Roll[2] = {0.005542044942f, 0.000033725503f}; 
	const float C_Roll[2] = {4.457995647858f, -29.104042013618f};

	limit = 5.0f;

	switch(_command)
	{
	case LEFT_SSP:
		temp = DEG2RAD*FTSensor[RFFT].VelRoll/FTSensor[RFFT].SF_Roll;
		X_Roll_New_R[0] = A_Roll[0]*X_Roll_R[0] + A_Roll[1]*X_Roll_R[1] + B_Roll[0]*temp;
		X_Roll_New_R[1] = A_Roll[2]*X_Roll_R[0] + A_Roll[3]*X_Roll_R[1] + B_Roll[1]*temp;
		Y_Roll_R = C_Roll[0]*X_Roll_New_R[0] + C_Roll[1]*X_Roll_New_R[1]; 
		
		X_Roll_R[0] = X_Roll_New_R[0];	X_Roll_R[1] = X_Roll_New_R[1];
		
		temp = -1.5f*10.0f*RAD2DEG*Y_Roll_R;
		if(temp > limit) temp = limit;
		else if(temp < -limit) temp = -limit;
	
		_joint[RHR].ControlVibrationAngle = temp;
		break;
	case RIGHT_SSP:
		temp = DEG2RAD*FTSensor[LFFT].VelRoll/FTSensor[LFFT].SF_Roll;
		X_Roll_New_L[0] = A_Roll[0]*X_Roll_L[0] + A_Roll[1]*X_Roll_L[1] + B_Roll[0]*temp;
		X_Roll_New_L[1] = A_Roll[2]*X_Roll_L[0] + A_Roll[3]*X_Roll_L[1] + B_Roll[1]*temp;
		Y_Roll_L = C_Roll[0]*X_Roll_New_L[0] + C_Roll[1]*X_Roll_New_L[1];  
		
		X_Roll_L[0] = X_Roll_New_L[0];	X_Roll_L[1] = X_Roll_New_L[1];
		
		temp = -1.5f*10.0f*RAD2DEG*Y_Roll_L;
		if(temp > limit) temp = limit;
		else if(temp < -limit) temp = -limit;
	
		_joint[LHR].ControlVibrationAngle = temp;
		break;
	case DSP:
		break;
	}
}
/******************************************************************************/






/******************************************************************************/
void main(void)
{
	RtxInit();
	InitParameters();
	StartCAN();
	InitGlobalMotionVariables();	// by Inhyeok

	while(true)
	{		
		RtSleep(1);
		
		CanReceiveMsg(0);	// CAN 1st channel
		CanReceiveMsg(1);	// CAN 2nd channel
		CheckBoardStatus(Joint);

		switch(pSharedMemory->CommandFlag)
		{
			unsigned int i;
			
		case EXIT_PROGRAM:	// exit program
			FreeGlobalMotionVariables();	// by Inhyeok
			RtxEnd();				
			return;
			break;
		case CAN_CH0_TX:	// CAN message from win32 app. transmitting procedure using channel 0
			CanSendMsg(CAN0, pSharedMemory->Tx_ID0, pSharedMemory->Tx_Data0, pSharedMemory->Tx_DLC0, 0);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case CAN_CH1_TX:	// CAN message from win32 app. transmitting procedure using channel 1
			CanSendMsg(CAN1, pSharedMemory->Tx_ID1, pSharedMemory->Tx_Data1, pSharedMemory->Tx_DLC1, 0);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_MOTOR_GAIN:
			//Joint[pSharedMemory->JointID].
			//SetMotorGainSetting(Joint[i]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_CURRENT_GAIN:
			//for(i=RHY ; i<NO_OF_JOINT ; i++) MotorGainSetting(Joint[i], 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case LOAD_PARAMETER:	// load parameters from the file.
			LoadParameter();
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SAVE_PARAMETER:	// save parameters to the file
			SaveParameter();
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_JOINT_PARAMETER:	// Set joint parameter
			SetJointParameter(pSharedMemory->JointID, pSharedMemory->Joint);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GET_JOINT_PARAMETER:	// Send joint parameter back to the win32 program
			GetJointParameter(pSharedMemory->JointID, &pSharedMemory->Joint);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PRINT_JOINT_PARAMETER:	// print out current joint parameter
			PrintJointParameter(pSharedMemory->JointID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case CHECK_DEVICE:	// Check each sub-controller's CAN communication and set it's initial parameters
			for(i=JMC0 ; i<=JMC11 ; i++) CheckDeviceCAN(i);
			for(i=EJMC0 ; i<=EJMC5 ; i++) CheckDeviceCAN(i);
			for(i=FT0 ; i<=FT1 ; i++) CheckDeviceCAN(i);
			for(i=FT2 ; i<=FT3 ; i++) CheckDeviceCAN(i);
			CheckDeviceCAN(IMU0);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GAIN_SETTING:	// motor controller gain setting
			for(i=RHY ; i<WST ; i++) GainSetting(i);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENABLE_FET:	// motor controller FET driver enable
			for(i=JMC0 ; i<=JMC11 ; i++) FETDriverOnOff(i, 0x01);
			for(i=EJMC0 ; i<=EJMC5 ; i++) FETDriverOnOff(i, 0x01);
			ReadSensorFlag = 0x01;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENABLE_FET_EACH:	// motor controller FET driver enable (each board)
			FETDriverOnOff(pSharedMemory->BoardID, 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DISABLE_FET:	// motor controller FET driver disable
			for(i=JMC0 ; i<=JMC11 ; i++) FETDriverOnOff(i, 0);
			for(i=EJMC0 ; i<=EJMC5 ; i++) FETDriverOnOff(i, 0);
			ReadSensorFlag = 0x00;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DISABLE_FET_EACH:	// motor controller FET driver disable (each board)
			FETDriverOnOff(pSharedMemory->BoardID, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case RUN_CMD:		// motor controller control-on
			for(i=JMC0 ; i<=JMC11 ; i++) { ZeroEncoder(i);	SendRunStopCMD(i, 0x01); }
			for(i=EJMC0 ; i<=EJMC5 ; i++) { ZeroEncoder(i);	SendRunStopCMD(i, 0x01); }
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case RUN_CMD_EACH:	// motor controller control-on (each board)
			ZeroEncoder(pSharedMemory->BoardID);
			SendRunStopCMD(pSharedMemory->BoardID, 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case STOP_CMD:		// motor controller control-off
			for(i=JMC0 ; i<=JMC11 ; i++) SendRunStopCMD(i, 0x00);
			for(i=EJMC0 ; i<=EJMC5 ; i++) SendRunStopCMD(i, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
		case STOP_CMD_EACH:	// motor controller control-off (each board)
			SendRunStopCMD(pSharedMemory->BoardID, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS:	// go to limit sensor and initial position
			GoToLimitPos(pSharedMemory->JointID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS_UPPER_ALL:
			GoToLimitPosAll(0x02);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS_LOWER_ALL:
			GoToLimitPosAll(0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS_ALL:
			GoToLimitPosAll(0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENCODER_ZERO:	// set encode value as zero
			for(i=JMC0 ; i<=JMC11 ; i++) ZeroEncoder(i);
			for(i=EJMC0 ; i<=EJMC5 ; i++) ZeroEncoder(i);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENCODER_ZERO_EACH:	// set encoder value as zero (each board)
			ZeroEncoder(pSharedMemory->BoardID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_ENCODER_RESOLUTION:
			Joint[pSharedMemory->JointID].Encoder_size = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].Positive_dir = pSharedMemory->CommandDataArray[1];
			SetEncoderResolution(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_DEADZONE:
			Joint[pSharedMemory->JointID].Deadzone = pSharedMemory->CommandData;
			SetDeadZone(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_JAMPWM_FAULT:
			Joint[pSharedMemory->JointID].JAMmsTime = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].PWMmsTime = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].JAMDuty = (unsigned char)(pSharedMemory->CommandDataArray[2]);
			Joint[pSharedMemory->JointID].PWMDuty = (unsigned char)(pSharedMemory->CommandDataArray[3]);
			SetJamPwmSturation(Joint[pSharedMemory->JointID]); 
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_MAX_VEL_ACC:
			Joint[pSharedMemory->JointID].MaxAcc = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].MaxVel = pSharedMemory->CommandDataArray[1];
			SetMaxAccVel(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_CONTROL_MODE:
			Joint[pSharedMemory->JointID].MotorControlMode = pSharedMemory->CommandData;
			SetControlMode(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_HOME_SEARCH_PARAMETER:
			Joint[pSharedMemory->JointID].Limit_rev = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].SearchDirection = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].Offset_angle = pSharedMemory->CommandDataFloat[0];
			SetHomeSearchParameter(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_HOME_MAX_VEL_ACC:
			Joint[pSharedMemory->JointID].MaxVelHome = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].MaxAccHome = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].HomeSearchMode = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].PWMDuty = pSharedMemory->CommandDataArray[0];
			SetHomeMaxVelAcc(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_POSITION_LIMIT:
			Joint[pSharedMemory->JointID].UpperPositionLimit = pSharedMemory->CommandDataFloat[0];
			Joint[pSharedMemory->JointID].LowerPositionLimit = pSharedMemory->CommandDataFloat[1];
			SetUpperPositionLimit(Joint[pSharedMemory->JointID]);
			SetLowerPositionLimit(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_ERROR_BOUND:
			Joint[pSharedMemory->JointID].I_ERR = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].B_ERR = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].E_ERR = pSharedMemory->CommandDataArray[2];
			SetErrorBound(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case REQUEST_PARAMETER:
			RequestParameters(&Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case POSITION_LIMIT_ONOFF:
			for(i=0 ; i<NO_OF_JOINT ; i++) PositionLimitOnOff(i, pSharedMemory->CommandData);
			//PositionLimitOnOff(LHP, pSharedMemory->CommandData);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case BEEP:
			Beep(pSharedMemory->CommandData);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case JOINT_REF_SET_RELATIVE:	// set joint position reference (relative mode)
			SetMoveJointAngle(pSharedMemory->JointID, pSharedMemory->GoalAngle, pSharedMemory->GoalTime, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case JOINT_REF_SET_ABS:	// set joint position reference (abs mode)
			SetMoveJointAngle(pSharedMemory->JointID, pSharedMemory->GoalAngle, pSharedMemory->GoalTime, 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_FT_PARAMETER:	// set FT sensor parameters
			SetFTParameter(pSharedMemory->FTsensorID, pSharedMemory->FTsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GET_FT_PARAMETER:	// get current FT sensor parameters
			GetFTParameter(pSharedMemory->FTsensorID, &pSharedMemory->FTsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case NULL_FT_SENSOR:
			for(i=RFFT ; i<NO_OF_FT ; i++) { NullFTSensor(i, 0x00); RtSleep(10); }
			pSharedMemory->CommandFlag = NO_ACT;
		case NULL_WRIST_FT_SENSOR:
			NullFTSensor(RWFT, 0x00); RtSleep(10);
			NullFTSensor(LWFT, 0x00); RtSleep(10);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case NULL_FOOT_ANGLE_SENSOR:
			RtWprintf(L"\n>>> FT angle nulling..!!");
			NullFootAngleSensor(FTSensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case NULL_IMU_SENSOR:
			RtWprintf(L"\n>>> IMU sensor nulling..!!");
			for(i=CENTERIMU ; i<NO_OF_IMU ; i++) { NullIMUSensor(i, 0x00); RtSleep(2); }
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PRINT_FT_PARAMETER:	// print out current FT sensor parameters
			PrintFTParameter(pSharedMemory->FTsensorID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_IMU_PARAMETER:		// set IMU sensor parameters
			SetIMUParameter(pSharedMemory->IMUsensorID, pSharedMemory->IMUsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GET_IMU_PARAMETER:		// get current IMU parameters
			GetIMUParameter(pSharedMemory->IMUsensorID, &pSharedMemory->IMUsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PRINT_IMU_PARAMETER:	// print out current IMU parameters
			PrintIMUParameter(pSharedMemory->IMUsensorID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_DAMPING_GAIN:
			if(pSharedMemory->CommandData == 0x00)
			{ for(i=0 ; i<3 ; i++) dampingGain[i] = pSharedMemory->CommandDataFloat[i]; }
			else { for(i=0 ; i<3 ; i++) dampingGain[i+3] = pSharedMemory->CommandDataFloat[i]; }
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_WALK_READY_POS:	// goto walk-ready position
			GotoWalkReadyPos();
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_HOME_POS:			// goto home position
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			GotoHomePos();
			GoToLimitPos(RF1);
			GoToLimitPos(LF1);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case START_ZMP_INITIALIZATION:	// start initialization control parameters(ZMP, Roll & Pitch angles..)
			pSharedMemory->MotorControlMode = CTRLMODE_ZMP_INITIALIZATION;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case STOP_ZMP_INITIALIZATION:	// stop initialization control parameters
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_FORWARD:			// start walking
			WalkingStartStop = 0x02;

			if(pSharedMemory->JW_temp[9]>=0)
			{
				WalkingPhaseNextSway = HIP_CENTER_TO_RIGHT_SET;
				WalkingPhaseNext[X] = FIRST_LEFT_FOOT_UP_FORWARD_SET;
				WalkingPhaseNext[Y] = FIRST_LEFT_FOOT_UP_SET;
				WalkingPhaseNext[Z] = FIRST_LEFT_FOOT_UP_SET;
				WalkingPhaseNext[Yaw] = FIRST_LEFT_FOOT_UP_SET;
			}
			else
			{	
				WalkingPhaseNextSway = HIP_CENTER_TO_LEFT_SET;
				WalkingPhaseNext[X] = FIRST_RIGHT_FOOT_UP_FORWARD_SET;
				WalkingPhaseNext[Y] = FIRST_RIGHT_FOOT_UP_SET;
				WalkingPhaseNext[Z] = FIRST_RIGHT_FOOT_UP_SET;
				WalkingPhaseNext[Yaw] = FIRST_RIGHT_FOOT_UP_SET;
			}
			pSharedMemory->MotorControlMode = CTRLMODE_WALKING;

			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case STOP_WALKING:		// stop walking
			WalkingStartStop = 0x01;
			pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_MOCAP:		// modified by Inhyeok
			if(TransitionFlag != 1)
			{
				if(MotionSetFlag != 1)
				{
					MotionSetFlag = 0;										
					_recovery_count = 0;					
					InitQubWindow(Joint[WST].RefAngleCurrent*D2R, Joint[RSP].RefAngleCurrent*D2R, Joint[RSR].RefAngleCurrent*D2R, Joint[RSY].RefAngleCurrent*D2R, Joint[REB].RefAngleCurrent*D2R, Joint[RWY].RefAngleCurrent*D2R, Joint[RWP].RefAngleCurrent*D2R,
						Joint[LSP].RefAngleCurrent*D2R, Joint[LSR].RefAngleCurrent*D2R, Joint[LSY].RefAngleCurrent*D2R, Joint[LEB].RefAngleCurrent*D2R, Joint[LWY].RefAngleCurrent*D2R, Joint[LWP].RefAngleCurrent*D2R,
						0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.);		
					
					UpdateGlobalMotionVariables();
					
					if(UpdatePassiveCoord_DSP())
					{
						_UpdatePassiveFlag = FALSE;
						wberror("Update Passive Coordinates failure");					
					}
					else
						_UpdatePassiveFlag = TRUE;					
				}
				
				if(_recovery_count == 0)
				{
					if(pSharedMemory->JW_MotionSet[0] == TRUE) SetMotionCapture0();
					else if(pSharedMemory->JW_MotionSet[1] == TRUE) SetMotionCapture1();
					else if(pSharedMemory->JW_MotionSet[2] == TRUE) SetMotionCapture2();
					else if(pSharedMemory->JW_MotionSet[3] == TRUE) SetMotionCapture3();
					else if(pSharedMemory->JW_MotionSet[4] == TRUE) SetMotionCapture4();
					else if(pSharedMemory->JW_MotionSet[5] == TRUE) SetMotionCapture5();
					else if(pSharedMemory->JW_MotionSet[6] == TRUE) SetMotionCapture6();
					else if(pSharedMemory->JW_MotionSet[7] == TRUE) SetMotionCapture7();
					else if(pSharedMemory->JW_MotionSet[8] == TRUE) SetMotionCapture8();
					else if(pSharedMemory->JW_MotionSet[9] == TRUE) SetMotionCapture9();
					else if(pSharedMemory->JW_MotionSet[10] == TRUE) SetMotionCapture10();
					else if(pSharedMemory->JW_MotionSet[11] == TRUE) SetMotionCapture11();
					else if(pSharedMemory->JW_MotionSet[12] == TRUE) SetMotionCapture12();
					else if(pSharedMemory->JW_MotionSet[13] == TRUE) SetMotionCapture13();
					else if(pSharedMemory->JW_MotionSet[14] == TRUE) SetMotionCapture14();
					else if(pSharedMemory->JW_MotionSet[15] == TRUE) SetMotionCapture15();
					else if(pSharedMemory->JW_MotionSet[16] == TRUE) SetMotionCapture16();
					else if(pSharedMemory->JW_MotionSet[17] == TRUE) SetMotionCapture17();
					else if(pSharedMemory->JW_MotionSet[18] == TRUE) SetMotionCapture18();
					//else if(pSharedMemory->JW_MotionSet[19] == TRUE)SetMotionCapture19();
					//else if(pSharedMemory->JW_MotionSet[20] == TRUE)SetMotionCapture20();
					//else if(pSharedMemory->JW_MotionSet[21] == TRUE)SetMotionCapture21();
					else if(pSharedMemory->JW_MotionSet[22] == TRUE) SetMotionCapture22();
					else if(pSharedMemory->JW_MotionSet[23] == TRUE) SetMotionCapture23();
					else if(pSharedMemory->JW_MotionSet[24] == TRUE) SetMotionCapture24();
					else if(pSharedMemory->JW_MotionSet[25] == TRUE) SetMotionCapture25();
					else if(pSharedMemory->JW_MotionSet[26] == TRUE) SetMotionCapture26();
					else if(pSharedMemory->JW_MotionSet[27] == TRUE) SetMotionCapture27();
					else if(pSharedMemory->JW_MotionSet[28] == TRUE) SetMotionCapture28();
					
					if(_UpdatePassiveFlag == TRUE)
					{
						if(MotionSetFlag == 1)
							pSharedMemory->MotorControlMode = CTRLMODE_JW_MOCAP;
					}
					else
						MotionSetFlag = 0;
				}				
			}
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case C_CONTROL_MODE:
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			Control_Mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case P_CONTROL_MODE:  // Position control of finger makes finger motors broken!! Don't use
			//FingerControlModeChange(EJMC4, 0x00);
			//FingerControlModeChange(EJMC5, 0x00);
			GoToLimitPos(pSharedMemory->JointID);
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			Control_Mode = 0x01; // Current Control mode
			//Control_Mode = 0x00; // Position Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_ON:
			for(i=RF1;i<=RF5;i++) Joint[i].RefVelCurrent = -pSharedMemory->CommandDataFloat[0];
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_OFF:
			for(i=RF1;i<=RF5;i++) Joint[i].RefVelCurrent = pSharedMemory->CommandDataFloat[0];
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_STOP:
			for(i=RF1;i<=RF5;i++) Joint[i].RefVelCurrent = 0;
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DEMO_FLAG:
			DemoFlag = pSharedMemory->CommandData;
			pSharedMemory->MotorControlMode = CTRLMODE_DEMO;
			break;
		case TEST_FUNCTION:
			//TestFunction();
			FETDriverOnOff(JMC3, 0x00);
			FETDriverOnOff(JMC7, 0x00);

			FETDriverOnOff(JMC3, 0x01);
			SendRunStopCMD(JMC3, 0x01);

			FETDriverOnOff(JMC7, 0x01);
			SendRunStopCMD(JMC7, 0x01);

			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case INIT_WB_MOCAP:		// by Inhyeok
			if(_WholeBodyMotionSetFlag == FALSE)
			{			
				UpdateGlobalMotionVariables();
				if(LoadMocapData(pSharedMemory->WB_MocapNo) == 0)
				{
					if(UpdatePassiveCoord_DSP())
					{
						_WholeBodyMotionSetFlag = FALSE;
						wberror("Updating Passive Coordinates failure!\n");
						ClearMocapData();					
					}
					else
					{
						pSharedMemory->Data_Debug_Index = 0;
						_mocap_count = 0;
						_WholeBodyMotionSetFlag = TRUE;
						pSharedMemory->MotorControlMode = CTRLMODE_WB_MOCAP;
					}				
				}			
				pSharedMemory->CommandFlag = NO_ACT;
			}
			else
			{
				wberror("Whole-body motion is running!\n");
				pSharedMemory->CommandFlag = NO_ACT;
			}
			break;
		}
	}
}
/******************************************************************************/





/******************************************************************************/
// timer interrupt service routine for lower body control
void RTFCNDCL TimerHandler(PVOID context)
{	
	// check CAN message from ch. 0
	CanReceiveMsg(CAN0);	// CAN 1st channel

	unsigned char supportPhase, landingState;
	if(ReadSensorFlag == 0x01)
	{
		// only for displaying
		unsigned char i;
		for(i=RHY ; i<NO_OF_JOINT; i++) pSharedMemory->Joint_debug[i] = Joint[i];

		ReadFT(CAN0);		// readout FT sensor value
		ReadIMU();			// readout IMU sensor value
//		ReadEncoder(CAN0);	// readout encoder value

		supportPhase = GetZMP(FTSensor[RFFT], FTSensor[LFFT], WalkingInfo, ZMP, FilteredZMP);
		landingState = LandingStateCheck(FTSensor, WalkingInfo);
	}

	switch(pSharedMemory->MotorControlMode)
	{
	case CTRLMODE_POSITION_CONTROL_WIN:
		// move each joint 
		// this mode is used normally in the home position setting
		MoveJointAngle(CAN0);
		break;
	case CTRLMODE_ZMP_INITIALIZATION:
		// initial posture setting
		ZMPInitialization(InitZMP, ZMP, IMUSensor[CENTERIMU], FTSensor[RFFT], FTSensor[LFFT], WalkingInfo, Joint, 1);
		
		// ZMP control at DSP
		if(supportPhase != NO_LANDING) DSPControl(ZMP, InitZMP, WalkingInfo, 1);
		
		// move
		MoveTaskPos(&WalkingInfoSway, WalkingInfo, Joint, landingState, CTRLMODE_ZMP_INITIALIZATION);
		break;
	case CTRLMODE_WALKING:
		// walking profile
		TestProfileSequence();

		// ZMP control at DSP (not walking)
		if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==WALK_READY) && (supportPhase!=NO_LANDING) ) 
		{
			DampingControl(IMUSensor, Joint, DSP);
			DSPControl(ZMP, InitZMP, WalkingInfo, 1);
		}
		else if(WalkingStartStop == 0x02) // during walking
		{
			// damping control
			//if(supportPhase != NO_LANDING) DampingControl(IMUSensor, Joint, supportPhase);
			DampingControl(IMUSensor, Joint, supportPhase);
			
			// foot vibration control
			VibrationControl(FTSensor, Joint, supportPhase);
		}
		else;

		// move
		MoveTaskPos(&WalkingInfoSway, WalkingInfo, Joint, landingState, CTRLMODE_WALKING);

		// for debugging //datasave
		if(pSharedMemory->Data_Debug_Index < 8000)
		{	
			pSharedMemory->Data_Debug[0][pSharedMemory->Data_Debug_Index] = ZMP[0];
			pSharedMemory->Data_Debug[1][pSharedMemory->Data_Debug_Index] = ZMP[1];
			pSharedMemory->Data_Debug[2][pSharedMemory->Data_Debug_Index] = ZMP[2];
			pSharedMemory->Data_Debug[3][pSharedMemory->Data_Debug_Index] = ZMP[3];
			pSharedMemory->Data_Debug[4][pSharedMemory->Data_Debug_Index] = ZMP[4];
			pSharedMemory->Data_Debug[5][pSharedMemory->Data_Debug_Index] = ZMP[5];

			pSharedMemory->Data_Debug[6][pSharedMemory->Data_Debug_Index] = FTSensor[RFFT].Mx;
			pSharedMemory->Data_Debug[7][pSharedMemory->Data_Debug_Index] = FTSensor[RFFT].My;
			pSharedMemory->Data_Debug[8][pSharedMemory->Data_Debug_Index] = FTSensor[RFFT].Fz;
			pSharedMemory->Data_Debug[9][pSharedMemory->Data_Debug_Index] = FTSensor[LFFT].Mx;
			pSharedMemory->Data_Debug[10][pSharedMemory->Data_Debug_Index] = FTSensor[LFFT].My;
			pSharedMemory->Data_Debug[11][pSharedMemory->Data_Debug_Index] = FTSensor[LFFT].Fz;

			pSharedMemory->Data_Debug[12][pSharedMemory->Data_Debug_Index] = supportPhase;
			
			pSharedMemory->Data_Debug_Index++;
			if(pSharedMemory->Data_Debug_Index >= 8000) pSharedMemory->Data_Debug_Index = 0;
		}	
		// for debugging -end-
		break;
	case CTRLMODE_DEMO:
		switch(DemoFlag)
		{
		case ONE_LEG_STAND:
			dspControlOnOff = false;	dampingControlOnOff = true;
			if( GoToDemoOneLegSupport() == false )
			{
				DemoFlag = pSharedMemory->CommandData = NO_DEMO;
				pSharedMemory->CommandFlag = NO_ACT;
			}
			break;
		case ONE_LEG_UP_AND_DOWN:
			dspControlOnOff = false;
			if( GoToDemoDownAndUp(RIGHT, 3) == false)
			{
				DemoFlag = pSharedMemory->CommandData = NO_DEMO;
				pSharedMemory->CommandFlag = NO_ACT;
			}
			break;
		case WALKREADY_POS:
			dspControlOnOff = true;	dampingControlOnOff = false;
			if( GoToDemoHomePosition() == false )
			{
				DemoFlag = pSharedMemory->CommandData = NO_DEMO;
				pSharedMemory->CommandFlag = NO_ACT;
			}
			break;
		case DSP_ON:
			dspControlOnOff = true;
			DemoFlag = pSharedMemory->CommandData = NO_DEMO;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DSP_OFF:
			dspControlOnOff = false;
			DemoFlag = pSharedMemory->CommandData = NO_DEMO;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		}

		// damping control 
		if(dampingControlOnOff) DampingControl(IMUSensor, Joint, DSP);
		// ZMP control
		if(dspControlOnOff) DSPControl(ZMP, InitZMP, WalkingInfo, 1);
		// foot vibration control
		if(vibrationControlOnOff) VibrationControl(FTSensor, Joint, supportPhase);

		// move
		MoveTaskPos(&WalkingInfoSway, WalkingInfo, Joint, landingState, CTRLMODE_WALKING);
		break;
	case CTRLMODE_WB_MOCAP:	// by Inhyeok
		WholeBodyMotionCapture(CAN0);				 
		break;
	case CTRLMODE_JW_MOCAP:	// by Inhyeok
		WholeBodyMotionCapture_JW(CAN0);		
		// for debugging
		if(pSharedMemory->Data_Debug_Index < N_ROW_SAVE)
		{	
			pSharedMemory->Data_Debug[0][pSharedMemory->Data_Debug_Index] = _Qub_window.dNK_JW_3x1[1];//_Qub_window.wind_WST[_Qub_window.tail_WST];
			pSharedMemory->Data_Debug[1][pSharedMemory->Data_Debug_Index] = _Qub_window.dNK_JW_3x1[2];//_Qub_window.wind_RSP[_Qub_window.tail_RSP];
			pSharedMemory->Data_Debug[2][pSharedMemory->Data_Debug_Index] = _Qub_window.dNK_JW_3x1[3];//_Qub_window.wind_RSR[_Qub_window.tail_RSR];
			pSharedMemory->Data_Debug[3][pSharedMemory->Data_Debug_Index] = Joint[NKY].RefVelCurrent;//_Qub_window.wind_RSY[_Qub_window.tail_RSY];
			pSharedMemory->Data_Debug[4][pSharedMemory->Data_Debug_Index] = Joint[NK1].RefVelCurrent;//_Qub_window.wind_REB[_Qub_window.tail_REB];
			pSharedMemory->Data_Debug[5][pSharedMemory->Data_Debug_Index] = Joint[NK2].RefVelCurrent;//_Qub_window.wind_RWY[_Qub_window.tail_RWY];
			pSharedMemory->Data_Debug[6][pSharedMemory->Data_Debug_Index] = jw_test[0];//_Qub_window.wind_RWP[_Qub_window.tail_RWP];
			pSharedMemory->Data_Debug[7][pSharedMemory->Data_Debug_Index] = jw_test[1];//_Qub_window.wind_LSP[_Qub_window.tail_LSP];
			pSharedMemory->Data_Debug[8][pSharedMemory->Data_Debug_Index] = _Qub_window.wind_LSR[_Qub_window.tail_LSR];
			pSharedMemory->Data_Debug[9][pSharedMemory->Data_Debug_Index] = _Qub_window.wind_LSY[_Qub_window.tail_LSY];
			pSharedMemory->Data_Debug[10][pSharedMemory->Data_Debug_Index] = _Qub_window.wind_LEB[_Qub_window.tail_LEB];
			pSharedMemory->Data_Debug[11][pSharedMemory->Data_Debug_Index] = _Qub_window.wind_LWY[_Qub_window.tail_LWY];			
			pSharedMemory->Data_Debug[12][pSharedMemory->Data_Debug_Index] = _Qub_window.wind_LWP[_Qub_window.tail_LWP];			
			pSharedMemory->Data_Debug[13][pSharedMemory->Data_Debug_Index] = _Qlb_19x1[14];
			pSharedMemory->Data_Debug[14][pSharedMemory->Data_Debug_Index] = _Qlb_19x1[15];
			pSharedMemory->Data_Debug[15][pSharedMemory->Data_Debug_Index] = _Qlb_19x1[16];
			pSharedMemory->Data_Debug[16][pSharedMemory->Data_Debug_Index] = _Qlb_19x1[17];
			pSharedMemory->Data_Debug[17][pSharedMemory->Data_Debug_Index] = _RWP;
			pSharedMemory->Data_Debug[18][pSharedMemory->Data_Debug_Index] = _LWP;
			
			pSharedMemory->Data_Debug[19][pSharedMemory->Data_Debug_Index] = _Qub_11x1[1];
			pSharedMemory->Data_Debug[20][pSharedMemory->Data_Debug_Index] = _Qub_11x1[2];
			pSharedMemory->Data_Debug[21][pSharedMemory->Data_Debug_Index] = _Qub_11x1[3];
			pSharedMemory->Data_Debug[22][pSharedMemory->Data_Debug_Index] = _Qub_11x1[4];
			pSharedMemory->Data_Debug[23][pSharedMemory->Data_Debug_Index] = _Qub_11x1[5];
			pSharedMemory->Data_Debug[24][pSharedMemory->Data_Debug_Index] = _Qub_11x1[6];
			pSharedMemory->Data_Debug[25][pSharedMemory->Data_Debug_Index] = _Qub_11x1[7];
			pSharedMemory->Data_Debug[26][pSharedMemory->Data_Debug_Index] = _Qub_11x1[8];
			pSharedMemory->Data_Debug[27][pSharedMemory->Data_Debug_Index] = _Qub_11x1[9];
			pSharedMemory->Data_Debug[28][pSharedMemory->Data_Debug_Index] = _Qub_11x1[10];
			pSharedMemory->Data_Debug[29][pSharedMemory->Data_Debug_Index] = _Qub_11x1[11];
			
			pSharedMemory->Data_Debug_Index++;
			if(pSharedMemory->Data_Debug_Index >= N_ROW_SAVE)
				pSharedMemory->Data_Debug_Index = (unsigned int)(N_ROW_SAVE-1);
		}
		// for debugging -end-*/
		break;
		case CTRLMODE_HANDSHAKE:		// by Inhyeok
			MoveJMC(EJMC3);
		break;	
	}

	if(ReadSensorFlag == 0x01) 
	{
		RequestSensor(CAN0);
	}

	timeIndex++;
}
/******************************************************************************/





/******************************************************************************/
// timer interrupt service routine for upper body control
void RTFCNDCL TimerHandler1(PVOID context)
{	

	// check CAN message from ch. 1
	CanReceiveMsg(CAN1);	// CAN 2nd channel

	if(ReadSensorFlag == 0x01) ReadFT(CAN1);		// readout FT sensor value

	switch(pSharedMemory->MotorControlMode)
	{
	case CTRLMODE_POSITION_CONTROL_WIN:
		// move each joint 
		// this mode is used normally in the home position setting
		MoveJointAngle(CAN1);
		break;
	case CTRLMODE_JW_MOCAP:					// by Inhyeok
		UpperBodyMotionCapture_mod(CAN1);
		WholeBodyMotionCapture_JW(CAN1);
		break;
	case CTRLMODE_WB_MOCAP:		// by Inhyeok
		WholeBodyMotionCapture(CAN1);		
		break;
	case CTRLMODE_HANDSHAKE:
		ShakeHands();
		for(int i=JMC8 ; i<=JMC11 ; i++) MoveJMC(i);
		for(i=EJMC0 ; i<=EJMC2 ; i++) MoveJMC(i);
		for(i=EJMC4 ; i<=EJMC5 ; i++) MoveJMC(i);
		break;
	}

	if(ReadSensorFlag == 0x01) RequestSensor(CAN1);

	timeIndex1++;
}
/******************************************************************************/






/******************************************************************************/
void TestProfileSequence(void)
{
	float tempDelayTime[2];
	float tempSway			= pSharedMemory->JW_temp[0];
	float tempSwayLeft		= pSharedMemory->JW_temp[1];
	float tempSwayRight		= pSharedMemory->JW_temp[2];
	float tempDelayY		= pSharedMemory->JW_temp[3];	// 60.0f;
	float tempDelayYLeft	= pSharedMemory->JW_temp[4];
	float tempDelayYRight	= pSharedMemory->JW_temp[5];
	float tempStep			= pSharedMemory->JW_temp[6];
	float tempStepLeft		= pSharedMemory->JW_temp[7];
	float tempStepRight		= pSharedMemory->JW_temp[8];
	float tempSideStep		= pSharedMemory->JW_temp[9];
	float tempSideStepLeft	= pSharedMemory->JW_temp[10];
	float tempSideStepRight = pSharedMemory->JW_temp[11];
	float tempYaw			= pSharedMemory->JW_temp[12];
	float tempDSP			= pSharedMemory->JW_temp[15];	// 20.0f;
	float tempPeriod		= pSharedMemory->JW_temp[16];	// 800.0f;
	int  tempCount			= (int)pSharedMemory->JW_temp[17];
	float tempStartSway		= pSharedMemory->JW_temp[18];
	float tempStopSway		= pSharedMemory->JW_temp[19];
	float tempSideDelayY		= pSharedMemory->JW_temp[20];	// 60.0f;
	float tempSideDelayYLeft	= pSharedMemory->JW_temp[21];
	float tempSideDelayYRight	= pSharedMemory->JW_temp[22];
//	float tempHipComp			= 0.0f;	// for hip compensation during side walk

	float tempUserData[5];


	// Stop walking command check
	if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==RIGHT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER;
	if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==LEFT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER;





	// x-direction sequence
	switch(WalkingPhaseNext[X])
	{
		case FIRST_RIGHT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod, tempDelayTime, tempUserData, 1, 0x04);
				SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			
				WalkingPhaseNext[X] = FIRST_RIGHT_FOOT_DOWN_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_RIGHT_FOOT_UP_FORWARD_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod, tempDelayTime, tempUserData, 1, 0x04);
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[X] = FIRST_LEFT_FOOT_DOWN_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_LEFT_FOOT_UP_FORWARD_SET;
			}
			break;
		case FIRST_RIGHT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
				SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);

				WalkingPhaseNext[X] = LEFT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_RIGHT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case FIRST_LEFT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				
				WalkingPhaseNext[X] = RIGHT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_LEFT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case RIGHT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(WalkingStartStop == 0x00)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					
					WalkingPhaseNext[X] = HIP_TO_CENTER;
				}
				else
				{
					tempUserData[0] = tempDSP;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x07);
					tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					WalkingPhaseNext[X] = RIGHT_FOOT_DOWN_FORWARD_SET;
				}
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = RIGHT_FOOT_UP_FORWARD_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
				
				tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				
				WalkingPhaseNext[X] = LEFT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = RIGHT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case LEFT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(WalkingStartStop == 0x00)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
					
					WalkingPhaseNext[X] = HIP_TO_CENTER;
				}
				else
				{
					tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					
					tempUserData[0] = tempDSP;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x07);
					
					WalkingPhaseNext[X] = LEFT_FOOT_DOWN_FORWARD_SET;
				}

				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = LEFT_FOOT_UP_FORWARD_SET;
			}
			break;
		case LEFT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				
				tempUserData[0] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
				
				WalkingPhaseNext[X] = RIGHT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = LEFT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(WalkingInfo[RIGHT][X].CurrentSequence == LEFT_FOOT_UP_FORWARD_SET)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
				}
				else if(WalkingInfo[RIGHT][X].CurrentSequence == RIGHT_FOOT_UP_FORWARD_SET)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				}
				WalkingPhaseNext[X] = WALK_READY;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false))
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = WALK_READY;
			break;
	}


	// y-direction sequence
	switch(WalkingPhaseNext[Y])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY/2.0f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);
			
				WalkingPhaseNext[Y] = FIRST_RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_RIGHT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY/2.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
			
				WalkingPhaseNext[Y] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_RIGHT_FOOT_DOWN_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{	
				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f+tempSideDelayYLeft*tempSideDelayY/2.0f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				WalkingPhaseNext[Y] = FIRST_LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.f+tempSideDelayYLeft*tempSideDelayY/2.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				WalkingPhaseNext[Y] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_LEFT_FOOT_DOWN_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					//tempDelayTime[0] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					//SetMoveTaskPos(LEFT, Y, tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	// if you change "function mode" here, also should change it in "HIP_TO_CENTER"
					
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						
						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else	// continuous walking
					{
						//SetMoveTaskPos(RIGHT, Y, -tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						tempDelayTime[0] = tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = RIGHT_FOOT_DOWN_SET;
					}

					//SetMoveTaskPosJointFF(&Joint[LHR], tempHipComp, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						
						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else
					{
						tempDelayTime[0] = tempDSP+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						WalkingPhaseNext[Y] = RIGHT_FOOT_DOWN_SET;
					}
				}
				
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	// DSP time + margin time
				//	SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
				//	SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				}
				
				WalkingPhaseNext[Y] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						tempDelayTime[0] = tempDSP+20.f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						tempDelayTime[0] = tempDSP+20.0f+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else
					{
						tempDelayTime[0] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					
						tempDelayTime[0] = tempDSP+20.f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						WalkingPhaseNext[Y] = LEFT_FOOT_DOWN_SET;
					}
				}
				else
				{
					//tempDelayTime[0] = tempDSP+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					//SetMoveTaskPos(RIGHT, Y, tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	// if you change "function mode" here, also should change it in "HIP_TO_CENTER"
					
					
					//tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// by jungwoo 2011.05.26
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else	// continuous walking
					{
						//SetMoveTaskPos(LEFT, Y, -tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = LEFT_FOOT_DOWN_SET;
					}

					//SetMoveTaskPosJointFF(&Joint[RHR], -tempHipComp, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			
			break;
		case LEFT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				}
				else
				{
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;	// DSP time + margin time
				//	SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
				//	SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingPhaseNext[Y] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{					
					//tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;
					//SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					//tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;
					//SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingPhaseNext[Y] = WALK_READY;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = WALK_READY;
			}
			break;
	}


	// z-direction sequence
	switch(WalkingPhaseNext[Z])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = tempDelayY;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				if(tempSideStep < 0.0f) SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = tempDelayY;	tempUserData[0] = 0.0f;
				if(tempSideStep > 0.0f) SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{	
				tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
				//if(tempSideStep >= 0.0f) SetMoveTaskPos(LEFT, Z, WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				//else SetMoveTaskPos(LEFT, Z, WALK_READY_Z_POS+tempPushLeg, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				if(tempSideStep < 0.0f) SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				WalkingPhaseNext[Z] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDelayY;	tempDelayTime[1] = tempDSP;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = tempDelayY;	tempDelayTime[1] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(tempSideStep > 0.0f) SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			break;
		case LEFT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDelayY;	tempDelayTime[1] = tempDSP;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				tempDelayTime[0] = tempDelayY;	tempDelayTime[1] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				WalkingPhaseNext[Z] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = WALK_READY;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false))
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = WALK_READY;
			break;
	}


	// yaw-direction sequence
	switch(WalkingPhaseNext[Yaw])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				if(tempYaw < 0.0f) WalkingStep[Yaw] = tempYaw;

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] > 0.0f)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
				}
				else
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				if(tempYaw >= 0.0f) WalkingStep[Yaw] = tempYaw;

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f;	tempDelayTime[1] = 0.0f;;
				if(WalkingStep[Yaw] > 0.0f)
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] >= 0.0f)
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				
				WalkingPhaseNext[Yaw] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				if(WalkingStep[Yaw] >= 0.0f) 
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

					if(tempYaw < 0.0f) WalkingStep[Yaw] = 0.0f;
					else WalkingStep[Yaw] = tempYaw;
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] > 0.0f) 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			break;
		case LEFT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				if(WalkingStep[Yaw] > 0.0f) 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}

					if(tempYaw >= 0.0f) WalkingStep[Yaw] = 0.0f;
					else WalkingStep[Yaw] = tempYaw;
				}
				
				WalkingPhaseNext[Yaw] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				WalkingPhaseNext[Yaw] = WALK_READY;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false))
			{
				WalkingStep[Yaw] = 0.0f;;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = WALK_READY;
			}
			break;
	}

		// basic sway movement
	switch(WalkingPhaseNextSway)
	{
	case HIP_CENTER_TO_LEFT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDSP = 40.0f;	tempDelayY = 0.0f;	tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;
			SetMoveTaskPos(&WalkingInfoSway, -tempStartSway*tempSway, tempPeriod, tempDelayTime,tempUserData, 1, 0);
			//bool SetMoveTaskPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime, float _msDelayTime[2], float _userData[5], unsigned char _mode, unsigned char _function)
			WalkingPhaseNextSway = HIP_LEFT_TO_RIGHT_SET;
			WalkingInfoSway.CurrentSequence = HIP_CENTER_TO_LEFT_SET;

			// for walking step count checking
			if(tempCount == 0) 
			{
				if(tempStep == 0.0f) WalkingStartStop = 0x00;
				else WalkingStartStop = 0x01;
			}
			else if(tempCount == -1) tempCount = -1;
			else pSharedMemory->JW_temp[17] -=1.0f;

			//SetMoveJointAngle(LSP, tempStep, tempPeriod, 0);
		}
		break;
	case HIP_CENTER_TO_RIGHT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDSP = 40.0f;	tempDelayY = 0.0f;	tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;
			SetMoveTaskPos(&WalkingInfoSway, tempStartSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
		
			WalkingPhaseNextSway = HIP_RIGHT_TO_LEFT_SET;
			WalkingInfoSway.CurrentSequence = HIP_CENTER_TO_RIGHT_SET;
			
			// for walking step count checking
			if(tempCount == 0) 
			{
				if(tempStep == 0.0f) WalkingStartStop = 0x00;
				else WalkingStartStop = 0x01;
			}
			else if(tempCount == -1) tempCount = -1;
			else pSharedMemory->JW_temp[17] -=1.0f;
		}
		break;
	case HIP_LEFT_TO_RIGHT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{	
			tempDelayTime[0] = tempDelayYLeft*tempDelayY;	tempDelayTime[1] = tempDelayYRight*tempDelayY;
			if(WalkingStartStop == 0x01)
			{
				tempCount = 0;	WalkingStartStop = 0x00;
				SetMoveTaskPos(&WalkingInfoSway, tempStopSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			}
			else 
			{
				SetMoveTaskPos(&WalkingInfoSway, tempSwayRight*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				// for walking step count checking
				if(tempCount == 0)
				{
					if(tempStep == 0.0f) WalkingStartStop = 0x00;
					else WalkingStartStop = 0x01;
				}
				else if(tempCount == -1) tempCount = -1;
				else pSharedMemory->JW_temp[17] -=1.0f;
			}

			WalkingPhaseNextSway = HIP_RIGHT_TO_LEFT_SET;
			WalkingInfoSway.CurrentSequence = HIP_LEFT_TO_RIGHT_SET;
		}
		break;
	case HIP_RIGHT_TO_LEFT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{	
			tempDelayTime[0] = tempDelayYRight*tempDelayY;	tempDelayTime[1] = tempDelayYLeft*tempDelayY;
			if(WalkingStartStop == 0x01)
			{
				tempCount = 0;	WalkingStartStop = 0x00;
				SetMoveTaskPos(&WalkingInfoSway, -tempStopSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			}
			else 
			{
				SetMoveTaskPos(&WalkingInfoSway, -tempSwayLeft*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				// for walking step count checking
				if(tempCount == 0)
				{
					if(tempStep == 0.0f) WalkingStartStop = 0x00;
					else WalkingStartStop = 0x01;
				}
				else if(tempCount == -1) tempCount = -1;
				else pSharedMemory->JW_temp[17] -=1.0f;
			}

			WalkingPhaseNextSway = HIP_LEFT_TO_RIGHT_SET;
			WalkingInfoSway.CurrentSequence = HIP_RIGHT_TO_LEFT_SET;
		}
		break;
	case HIP_TO_CENTER:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDelayTime[0] = tempDelayY;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
			WalkingPhaseNextSway = WALK_READY;
			WalkingInfoSway.CurrentSequence = HIP_TO_CENTER;
		}
		break;
	case WALK_READY:
		if(WalkingInfoSway.MoveFlag == false) WalkingInfoSway.CurrentSequence = WALK_READY;
		break;
	}
}
/******************************************************************************/






/******************************************************************************/
bool GoToDemoOneLegSupport(void)
{
	static unsigned char sequence = 0x00;
	float tempUserData[5];
	float tempDelayTime[2];

	if(sequence == 0x02) { sequence = 0x00; return false; }
	switch(sequence)
	{
	case 0x00:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.12f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			// z-direction sequence
			tempDelayTime[0] = 500.0f;	tempDelayTime[1] = 0.0f;;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT+0.04f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			sequence = 0x01;
		}
		break;
	case 0x01:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			sequence = 0x02;
		break;
	}

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool GoToDemoDownAndUp(unsigned char _rightLeft, unsigned char _times)
{
	static unsigned char sequence = 0x00;
	static unsigned char times = _times;
	float tempUserData[5];
	float tempDelayTime[2];
	float height = 0.04f;

	if(times == 0x00) { times = _times; return false; }

	// z-direction sequence
	switch(sequence)
	{
	case 0x00:	// down sequence
		if(_rightLeft == RIGHT)
		{
			if(WalkingInfo[RIGHT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+height, 800.0f, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x01;
			}
		}
		else if(_rightLeft == LEFT)
		{
			if(WalkingInfo[LEFT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+height, 800.0f, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x01;
			}
		}
		else
		{
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+height, 800.0f, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+height, 800.0f, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x01;
			}
		}
		break;
	case 0x01:
		if(_rightLeft == RIGHT)
		{
			if(WalkingInfo[RIGHT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, 800.0f, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x00;	times--;
			}
		}
		else if(_rightLeft == LEFT)
		{
			if(WalkingInfo[LEFT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, 800.0f, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x00;	times--;
			}
		}
		else
		{
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, 800.0f, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, 800.0f, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x00;	times--;
			}
		}
		break;
	}

	return true;
}
/******************************************************************************/






/******************************************************************************/
bool GoToDemoHomePosition(void)
{
	static unsigned char sequence = 0x00;
	float tempUserData[5];
	float tempDelayTime[2];

	if(sequence == 0x02) { sequence = 0x00; return false; }
	switch(sequence)
	{
	case 0x00:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.0f, 800.0f, tempDelayTime, tempUserData, 1, 0);
			// z-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 60.0f;	tempUserData[0] = 0.0f;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, 800.0f, tempDelayTime, tempUserData, 1, 0);

			sequence = 0x01;
		}
		break;
	case 0x01:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			sequence = 0x02;
		break;
	}

	return true;
}
/******************************************************************************/






/******************************************************************************/
void TestFunction(void)
{
	float tempDelayTime[2], tempUserData[5];

	tempDelayTime[0] = 400.0f;	tempDelayTime[1] = 60.0f;	tempUserData[0] = 0.0f;
	SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT+pSharedMemory->Temp[14], 800.0f, tempDelayTime, tempUserData, 1, 0);
	SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+pSharedMemory->Temp[14], 800.0f, tempDelayTime, tempUserData, 1, 0);
}
/******************************************************************************/



/******************************************************************************/	
int WholeBodyMotionCapture(unsigned char canChannel)		// by Inhyeok
{
	if(_WholeBodyMotionSetFlag == FALSE)
		return 0;
	
	if(canChannel == CAN0) 
	{
		MocapOnestep(_mocap_count);
		_mocap_count++;
		
		MoveJMC(JMC0);
		MoveJMC(JMC1);
		MoveJMC(JMC2);
		MoveJMC(JMC3);
		MoveJMC(JMC4);
		MoveJMC(JMC5);
		MoveJMC(JMC6);
		MoveJMC(JMC7);
		
		
		if(_mocap_count > _MOTION_LENGTH + 400)
		{
			_WholeBodyMotionSetFlag = FALSE;
			ClearMocapData();			
			RtWprintf(L"\n\n Whole-body motion ended.\n");
		}
		
		return 0;
	}
	else if(canChannel == CAN1)
	{		
		MoveJMC(JMC8);
		MoveJMC(JMC9);
		MoveJMC(JMC10);
		MoveJMC(JMC11);
		
		MoveJMC(EJMC0);	// Right wrist
		MoveJMC(EJMC1);	// Left wrist
		//		MoveJMC(EJMC2);	// Neck
		MoveJMC(EJMC3);	// Waist
		//		MoveJMC(EJMC4);	// Right fingers
		//		MoveJMC(EJMC5);	// Left fingers		
		
		return 0;
		
	}
	
	return 0;
	
}
/******************************************************************************/



/******************************************************************************/	
int WholeBodyMotionCapture_JW(unsigned char canChannel)		// by Inhyeok
{
	if(MotionSetFlag == FALSE)
		return 0;
	
	if(canChannel == CAN0) 
	{
		MocapOnestep_JW();
		
		MoveJMC(JMC0);
		MoveJMC(JMC1);
		MoveJMC(JMC2);
		MoveJMC(JMC3);
		MoveJMC(JMC4);
		MoveJMC(JMC5);
		MoveJMC(JMC6);
		MoveJMC(JMC7);
		
		
		return 0;
	}
	else if(canChannel == CAN1)
	{		
		MoveJMC(JMC8);
		MoveJMC(JMC9);
		MoveJMC(JMC10);
		MoveJMC(JMC11);
		
		MoveJMC(EJMC0);	// Right wrist
		MoveJMC(EJMC1);	// Left wrist
		MoveJMC(EJMC2);	// Neck
		MoveJMC(EJMC3);	// Waist
		MoveJMC(EJMC4);	// Right fingers
		MoveJMC(EJMC5);	// Left fingers		
		
		return 0;
		
	}
	
	return 0;	
}
/******************************************************************************/




/******************************************************************************/
void UpperBodyMotionCapture_mod(unsigned char _canChannel)		// modified by Inhyeok
{
	float wst, rsp, rsr, rsy, reb, rwy, rwp;
	float lsp, lsr, lsy, leb, lwy, lwp;
	float bp;
	float dNKY, dNK1, dNK2;
	float dRF[5], dLF[5];

	float wst_, rsp_, rsr_, rsy_, reb_, rwy_, rwp_;
	float lsp_, lsr_, lsy_, leb_, lwy_, lwp_;
	float bp_;

	if(MotionSetFlag == 0) 
		return;
	
	if(TransitionFlag == 1)
	{
		if(MotionTimeCurrent == Ti+30 && Ti > OptimalTfMax)
		{
			Motion = tempMotion;
			MotionTimeCurrent = 0; 
		}
	}
	
	if(_canChannel == CAN0) 
	{
	}
	else if(_canChannel == CAN1)
	{
		if(_recovery_count == 0)
		{		
			dNKY = MotionCapture[Motion][MotionTimeCurrent][1]-MotionCapture[Motion][MotionTimeCurrent-1][1];
			dNK1 = MotionCapture[Motion][MotionTimeCurrent][2]-MotionCapture[Motion][MotionTimeCurrent-1][2];
			dNK2 = MotionCapture[Motion][MotionTimeCurrent][26]-MotionCapture[Motion][MotionTimeCurrent-1][26];

			wst = (Joint[WST].WalkReadyAngle - MotionCapture[Motion][MotionTimeCurrent][0])*D2R;	

			rsp = (Joint[RSP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][ 9])*D2R;	// R-Shoulder Pitch	
			rsr = (Joint[RSR].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][10])*D2R;	// R-Shoulder Roll	
			rsy = (Joint[RSY].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][11])*D2R;			// R-Shoulder Yaw
			reb = (Joint[REB].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][12])*D2R;	// R-Elbow	Pitch
			rwy = (Joint[RWY].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][13])*D2R;
			rwp = (Joint[RWP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][14])*D2R;
			//Joint[RHP].RefAngleCurrent = Joint[RHP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][15];
			bp = (MotionCapture[Motion][MotionTimeCurrent][15])*D2R;

			dRF[0] = MotionCapture[Motion][MotionTimeCurrent][16]-MotionCapture[Motion][MotionTimeCurrent-1][16];
			dRF[1] = MotionCapture[Motion][MotionTimeCurrent][17]-MotionCapture[Motion][MotionTimeCurrent-1][17];
			dRF[2] = MotionCapture[Motion][MotionTimeCurrent][18]-MotionCapture[Motion][MotionTimeCurrent-1][18];
			dRF[3] = MotionCapture[Motion][MotionTimeCurrent][19]-MotionCapture[Motion][MotionTimeCurrent-1][19];
			dRF[4] = MotionCapture[Motion][MotionTimeCurrent][20]-MotionCapture[Motion][MotionTimeCurrent-1][20];
			
			lsp = (Joint[LSP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][3])*D2R;	// L-Shoulder Pitch
			lsr = (Joint[LSR].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][4])*D2R;	// L-Shoulder Roll
			lsy = (Joint[LSY].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][5])*D2R;			// L-Shoulder Yaw
			leb = (Joint[LEB].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][6])*D2R;	// L-Elbow	Pitch		
			lwy = (Joint[LWY].WalkReadyAngle -MotionCapture[Motion][MotionTimeCurrent][7])*D2R;
			lwp = (Joint[LWP].WalkReadyAngle -MotionCapture[Motion][MotionTimeCurrent][8])*D2R;
			//Joint[LHP].RefAngleCurrent = Joint[LHP].WalkReadyAngle + MotionCapture[Motion][MotionTimeCurrent][15];
			
			dLF[0] = MotionCapture[Motion][MotionTimeCurrent][21]-MotionCapture[Motion][MotionTimeCurrent-1][21];
			dLF[1] = MotionCapture[Motion][MotionTimeCurrent][22]-MotionCapture[Motion][MotionTimeCurrent-1][22];
			dLF[2] = MotionCapture[Motion][MotionTimeCurrent][23]-MotionCapture[Motion][MotionTimeCurrent-1][23];
			dLF[3] = MotionCapture[Motion][MotionTimeCurrent][24]-MotionCapture[Motion][MotionTimeCurrent-1][24];
			dLF[4] = MotionCapture[Motion][MotionTimeCurrent][25]-MotionCapture[Motion][MotionTimeCurrent-1][25];

			wst_ = 0.5f*(wst + _Qub_window.wind_WST[_Qub_window.head_WST]);
			rsp_ = 0.5f*(rsp + _Qub_window.wind_RSP[_Qub_window.head_RSP]);
			rsr_ = 0.5f*(rsr + _Qub_window.wind_RSR[_Qub_window.head_RSR] - OFFSET_RSR*D2R);
			rsy_ = 0.5f*(rsy + _Qub_window.wind_RSY[_Qub_window.head_RSY]);
			reb_ = 0.5f*(reb + _Qub_window.wind_REB[_Qub_window.head_REB]);
			rwy_ = 0.5f*(rwy + _Qub_window.wind_RWY[_Qub_window.head_RWY]);
			rwp_ = 0.5f*(rwp + _Qub_window.wind_RWP[_Qub_window.head_RWP]);
			lsp_ = 0.5f*(lsp + _Qub_window.wind_LSP[_Qub_window.head_LSP]);
			lsr_ = 0.5f*(lsr + _Qub_window.wind_LSR[_Qub_window.head_LSR] - OFFSET_LSR*D2R);
			lsy_ = 0.5f*(lsy + _Qub_window.wind_LSY[_Qub_window.head_LSY]);
			leb_ = 0.5f*(leb + _Qub_window.wind_LEB[_Qub_window.head_LEB]);
			lwy_ = 0.5f*(lwy + _Qub_window.wind_LWY[_Qub_window.head_LWY]);
			lwp_ = 0.5f*(lwp + _Qub_window.wind_LWP[_Qub_window.head_LWP]);
			bp_ = 0.5f*(bp + _Qub_window.wind_BP[_Qub_window.head_BP]);


			JW_motion_window(wst_, rsp_, rsr_, rsy_, reb_, rwy_, rwp_,
							 lsp_, lsr_, lsy_, leb_, lwy_, lwp_,
							 dNKY, dNK1, dNK2, dRF[0], dRF[1], dRF[2], dRF[3], dRF[4], dLF[0], dLF[1], dLF[2], dLF[3], dLF[4],
							 bp_);
			
			JW_motion_window(wst, rsp, rsr, rsy, reb, rwy, rwp,
							 lsp, lsr, lsy, leb, lwy, lwp,
							 dNKY, dNK1, dNK2, dRF[0], dRF[1], dRF[2], dRF[3], dRF[4], dLF[0], dLF[1], dLF[2], dLF[3], dLF[4],
							 bp);

			MotionTimeCurrent++;			
		}	
		

		if((MotionTimeCurrent == MotionPeriod[Motion]+2 && TransitionFlag != 1)|| MotionTimeCurrent >= MocapTimeLimit -10)
		{
			if(_recovery_count > 150)	// returns to the walk-ready position for 1.5 seconds  
			{
				MotionSetFlag = 0;
				_recovery_count = 0;
				RtWprintf(L"\n Motion Ended!\n");
			}
			else
			{
				wst = Joint[WST].WalkReadyAngle*D2R;

				rsp = Joint[RSP].WalkReadyAngle*D2R;	// R-Shoulder Pitch	
				rsr = Joint[RSR].WalkReadyAngle*D2R;	// R-Shoulder Roll	
				rsy = Joint[RSY].WalkReadyAngle*D2R;			// R-Shoulder Yaw
				reb = Joint[REB].WalkReadyAngle*D2R;	// R-Elbow	Pitch
				rwy = Joint[RWY].WalkReadyAngle*D2R;
				rwp = Joint[RWP].WalkReadyAngle*D2R;
				
				bp = 0;				
				
				lsp = Joint[LSP].WalkReadyAngle*D2R;	// L-Shoulder Pitch
				lsr = Joint[LSR].WalkReadyAngle*D2R;	// L-Shoulder Roll
				lsy = Joint[LSY].WalkReadyAngle*D2R;			// L-Shoulder Yaw
				leb = Joint[LEB].WalkReadyAngle*D2R;	// L-Elbow	Pitch		
				lwy = Joint[LWY].WalkReadyAngle*D2R;
				lwp = Joint[LWP].WalkReadyAngle*D2R;				
				
				JW_motion_window(wst, rsp, rsr, rsy, reb, rwy, rwp,
							 lsp, lsr, lsy, leb, lwy, lwp,
							 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., bp);

				JW_motion_window(wst, rsp, rsr, rsy, reb, rwy, rwp,
							 lsp, lsr, lsy, leb, lwy, lwp,
							 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., bp);
				_recovery_count++;	
			}
		}

		if(MotionTimeCurrent == OptimalTfMax)
			TransitionFlag = 0;		
	}
}
/*****************************************************************************/







/*****************************************************************************/
float Torsion_mass_spring_damper_Mx(float Mx, int zero)
{
	static float x1new,x2new, x1=0., x2=0.;
	static float filt;
	
	/*
	const float a[4] = {(float)0.92461509587868,  (float)-0.32069771753193,
						(float)0.00962093152596,   (float)0.99837557091103};
	
	const float b[2] = {(float)0.00962093152596, (float)0.00004873287267};
	const float c[2] = { (float)0.0,  (float)65.0};
	*/
	/*
	const float a[4] = {(float)0.97015145404280,  (float)-0.13134819764971,
						(float)0.00985111482373,   (float)0.99933994240940};
	
	const float b[2] = {(float)0.00985111482373,	(float)0.00004950431930};
	
	const float c[2] = {(float)0.0,  (float)14.81481481481481};
	*/

	const float a[4] = {(float)0.97765796618688,  (float)-0.07324804804199,
						(float)0.00988848648567,   (float)0.99963238059948};
	
	const float b[2] = {(float)0.00988848648567,	(float)0.00004962861907};
	
	const float c[2] = {(float)0.0,  (float)14.81481481481481};

	
	x1new = a[0]*x1 + a[1]*x2 + b[0]*Mx;
	x2new = a[2]*x1 + a[3]*x2 + b[1]*Mx;
	
	if(zero == 0) {x1new = 0.; x2new = 0.; filt = 0.;}
	
	//filt = c[0]*x1new + c[1]*x2new;

	Mx = (float)(Mx);
	//filt = (float)(filt + 0.1*Mx); 
	filt = (float)(filt + 0.007*Mx); 
	
	if(filt > 30.0) filt = 30.0;
	else if(filt < -15.0) filt = -15.0;
	
	x1=x1new;
	x2=x2new;
	
	return filt;
}
/*****************************************************************************/




/*****************************************************************************/
float Torsion_mass_spring_damper_My(float My, int zero)
{
	static float x1new,x2new, x1=0., x2=0.;
	static float filt;
	static float a1;
	static float a1_old;
	
	const float a[4] = {(float)0.97015145404280,  (float)-0.13134819764971,
						(float)0.00985111482373,   (float)0.99933994240940};
	
	const float b[2] = {(float)0.00985111482373,	(float)0.00004950431930};
	
	const float c[2] = {(float)0.0,  (float)14.81481481481481};
	
	x1new = a[0]*x1 + a[1]*x2 + b[0]*My;
	x2new = a[2]*x1 + a[3]*x2 + b[1]*My;
	
	if(zero == 0) {x1new = 0.; x2new = 0.; filt = 0.;}
	
	//filt = c[0]*x1new + c[1]*x2new;

	//if((My <= 10.0) && (My >= -10.0)) My = (float)0.;

	a1 = (float)((1. - 2.*PI*0.1*DELTA_T)*a1_old + DELTA_T*My);
	a1_old = a1;

//	pSharedMemory->temp_force2 = a1;

	//if(My > 10.0) My = (float)(My - 10.0);
	//else if(My < -10.0) My = (float)(My + 10.0);
	//else My = (float)0.;
	
//	My = (float)(My - 8.8);
	My = (float)(My);


//	pSharedMemory->temp_force1 = My;

	//filt = (float)(filt + 0.05*My); 
	//filt = (float)(filt + 0.008*My); 
	//filt = (float)(filt + 0.0055*My); 
	//filt = (float)(filt + 0.006*My); 
	//filt = (float)(filt + 0.007*My); 
	filt = (float)(filt + 0.0045*My); 
	
	if(filt > 15.0) filt = 15.0;
	else if(filt < -30.0) filt = -30.0;
	
	x1=x1new;
	x2=x2new;
	
	return filt;
}
/*****************************************************************************/




/*****************************************************************************/
float Wrist_mass_spring_damper_Fz(float Fz, int zero)
{
	static float x1new,x2new, x1=0., x2=0.;
	static float filt;
	
	const float a[4] = {(float)0.94503939283972,  (float)-0.29175327263157,
						(float)0.00972510908772,   (float)0.99852749282218};
	
	const float b[2] = {(float)0.00972510908772,	(float)0.00004908357259};
	
	const float c[2] = {(float)0.0,    (float)10.0};
	
	
	x1new = a[0]*x1 + a[1]*x2 + b[0]*Fz;
	x2new = a[2]*x1 + a[3]*x2 + b[1]*Fz;
	
	if(zero == 0) {x1new = 0.; x2new = 0.;}
	
	filt = c[0]*x1new + c[1]*x2new ;
	
	if(filt > 30.0) filt = 30.0;
	else if(filt < -30.0) filt = -30.0;
	
	x1=x1new;
	x2=x2new;
	
	return filt;
}
/*****************************************************************************/




/*****************************************************************************/
void ShakeHands()
{
	static float res1 = 0.0f;
	static float res2 = 0.0f;
	static float force_res = 0.0f;
	static float force_res1 = 0.0f;
	//MotionTimeCurrent for shake hands

//	res1 = 0.0f;
//	res2 = 0.0f;
//	force_res = 0.0f;
//	force_res1 = 0.0f;

	if(pSharedMemory->ShakeHandsFlag == 1) 
	{
		MotionTimeCurrent = 0;
		Wrist_mass_spring_damper_Fz(FTSensor[RWFT].Fz, 0);
		Torsion_mass_spring_damper_My(FTSensor[RWFT].My, 0);
		Torsion_mass_spring_damper_Mx(FTSensor[RWFT].Mx, 0);
		pSharedMemory->ShakeHandsFlag = 2;
	}
	if(pSharedMemory->ShakeHandsFlag == 2)
	{
		FTN_half_1_cos(1., MotionTimeCurrent, 0, 200, 10, 10, &res1);
		if(MotionTimeCurrent < 200) MotionTimeCurrent++;
		else MotionTimeCurrent = 200;	

	// 0: -Torso yaw
	// 1: -Head yaw
	// 2: Head pitch\
	// 3: -LSP
	// 4: -LSY
	// 5: -LSR
	// 6: -LEB
	// 7: -LWR
	// 8: LWP
	// 9: -RSP
	// 10: -RSY
	// 11: RSR
	// 12: -REB
	// 13: -RWR
	// 14: RWP
	// 15: LRHP
		Joint[RSP].RefAngleCurrent = (float)(Joint[RSP].WalkReadyAngle - 50.*res1);
		Joint[RSR].RefAngleCurrent = (float)(Joint[RSR].WalkReadyAngle + 10*res1);
		Joint[RSY].RefAngleCurrent = (float)(Joint[RSY].WalkReadyAngle - 5.*res1);		
		Joint[REB].RefAngleCurrent = (float)(Joint[REB].WalkReadyAngle - 5.*res1);
		Joint[RWP].RefAngleCurrent = (float)(Joint[RWP].WalkReadyAngle - 12.*res1);
		Joint[RWY].RefAngleCurrent = (float)(Joint[RWY].WalkReadyAngle + 0.*res1);
		Joint[WST].RefAngleCurrent = (float)(Joint[WST].WalkReadyAngle + 17.*res1);
		//Joint[NKY].RefAngleCurrent = (float)(-12.*res1);
		//Joint[NK1].RefAngleCurrent = (float)(-12.*res1);
		//Joint[NK2].RefAngleCurrent = (float)(12.*res1);

		OLD_UpperMovement1[RSP] = Joint[RSP].RefAngleCurrent;
		OLD_UpperMovement1[RSR] = Joint[RSR].RefAngleCurrent;
		OLD_UpperMovement1[RSY] = Joint[RSY].RefAngleCurrent;
		OLD_UpperMovement1[REB] = Joint[REB].RefAngleCurrent;
		OLD_UpperMovement1[RWY] = Joint[RWY].RefAngleCurrent;
		OLD_UpperMovement1[RWP] = Joint[RWP].RefAngleCurrent;
		OLD_UpperMovement1[WST] = Joint[WST].RefAngleCurrent;
		//OLD_UpperMovement1[NKY] = Joint[NKY].RefAngleCurrent;
		//OLD_UpperMovement1[NK1] = Joint[NK1].RefAngleCurrent;
		//OLD_UpperMovement1[NK2] = Joint[NK2].RefAngleCurrent;

		OLD_UpperMovement[RSP] = Joint[RSP].RefAngleCurrent;
		OLD_UpperMovement[RSR] = Joint[RSR].RefAngleCurrent;
		OLD_UpperMovement[RSY] = Joint[RSY].RefAngleCurrent;
		OLD_UpperMovement[REB] = Joint[REB].RefAngleCurrent;
		OLD_UpperMovement[RWY] = Joint[RWY].RefAngleCurrent;
		OLD_UpperMovement[RWP] = Joint[RWP].RefAngleCurrent;
		OLD_UpperMovement[WST] = Joint[WST].RefAngleCurrent;
		//OLD_UpperMovement[NKY] = Joint[NKY].RefAngleCurrent;
		//OLD_UpperMovement[NK1] = Joint[NK1].RefAngleCurrent;
		//OLD_UpperMovement[NK2] = Joint[NK2].RefAngleCurrent;
	}
	if(pSharedMemory->ShakeHandsFlag == 3)
	{
		//FT Wrist nulling
		pSharedMemory->ShakeHandsFlag = 4;
	}
	if(pSharedMemory->ShakeHandsFlag == 4)
	{
		FTN_half_1_cos(1., MotionTimeCurrent, 0, 200, 10, 10, &res1);
	
		if(MotionTimeCurrent < 200) MotionTimeCurrent++;
		else MotionTimeCurrent = 200;	

		force_res1 = Torsion_mass_spring_damper_My(FTSensor[RWFT].My,1);
		force_res = Torsion_mass_spring_damper_Mx(FTSensor[RWFT].Mx, 1);
		jw_test[0] =  force_res;
		jw_test[1] =  force_res1;
		Joint[RSR].RefAngleCurrent = (float)(OLD_UpperMovement1[RSR]- 0.0*force_res1);
		Joint[REB].RefAngleCurrent = (float)(OLD_UpperMovement1[REB]- 15.0*force_res);
		Joint[RWY].RefAngleCurrent = (float)(OLD_UpperMovement1[RWY]- 0.0*force_res1);

		OLD_UpperMovement[RSP] = Joint[RSP].RefAngleCurrent;
		OLD_UpperMovement[RSR] = Joint[RSR].RefAngleCurrent;
		OLD_UpperMovement[RSY] = Joint[RSY].RefAngleCurrent;
		OLD_UpperMovement[REB] = Joint[REB].RefAngleCurrent;
		OLD_UpperMovement[RWY] = Joint[RWY].RefAngleCurrent;
		OLD_UpperMovement[RWP] = Joint[RWP].RefAngleCurrent;
		OLD_UpperMovement[WST] = Joint[WST].RefAngleCurrent;
		//OLD_UpperMovement[NKY] = Joint[NKY].RefAngleCurrent;
		//OLD_UpperMovement[NK1] = Joint[NK1].RefAngleCurrent;
		//OLD_UpperMovement[NK2] = Joint[NK2].RefAngleCurrent;
	
	}

	if(pSharedMemory->ShakeHandsFlag == 5)
	{
		MotionTimeCurrent = 0;
		pSharedMemory->ShakeHandsFlag = 6;
	}

	if(pSharedMemory->ShakeHandsFlag == 6)
	{
		FTN_half_1_cos(1., MotionTimeCurrent, 0, 200, 10, 10, &res2); 

		if(MotionTimeCurrent < 200) MotionTimeCurrent++;
		else pSharedMemory->ShakeHandsFlag = 0; 	

		Joint[RSP].RefAngleCurrent = (float)(Joint[RSP].WalkReadyAngle + (OLD_UpperMovement[RSP]-Joint[RSP].WalkReadyAngle)*(1. - res2));
		Joint[RSR].RefAngleCurrent = (float)(Joint[RSR].WalkReadyAngle + (OLD_UpperMovement[RSR]-Joint[RSR].WalkReadyAngle)*(1. - res2));
		Joint[RSY].RefAngleCurrent = (float)(Joint[RSY].WalkReadyAngle + (OLD_UpperMovement[RSY]-Joint[RSY].WalkReadyAngle)*(1. - res2));
		Joint[REB].RefAngleCurrent = (float)(Joint[REB].WalkReadyAngle + (OLD_UpperMovement[REB]-Joint[REB].WalkReadyAngle)*(1. - res2));
		Joint[RWY].RefAngleCurrent = (float)(Joint[RWY].WalkReadyAngle + (OLD_UpperMovement[RWY]-Joint[RWY].WalkReadyAngle)*(1. - res2));
		Joint[RWP].RefAngleCurrent = (float)(Joint[RWP].WalkReadyAngle + (OLD_UpperMovement[RWP]-Joint[RWP].WalkReadyAngle)*(1. - res2));
		Joint[WST].RefAngleCurrent = (float)(Joint[WST].WalkReadyAngle + (OLD_UpperMovement[WST]-Joint[WST].WalkReadyAngle)*(1. - res2));
	
		//Joint[NKY].RefAngleCurrent = (float)(OLD_UpperMovement[NKY]*(1. - res2));
		//Joint[NK1].RefAngleCurrent = (float)(OLD_UpperMovement[NK1]*(1. - res2));
		//Joint[NK2].RefAngleCurrent = (float)(OLD_UpperMovement[NK2]*(1. - res2));
	}

}

