#ifndef COMMON_DEFINITIONDEFINITIONS_H
#define COMMON_DEFINITIONDEFINITIONS_H

#include "CANID.h"

#define	PI					3.141592f
#define RAD2DEG				(180.0f/PI)
#define DEG2RAD				(PI/180.0f)
#define CAN_TIME			0.4	// CAN timer interrupt frequency (msec)
#define INT_TIME			5	// timer interrupt frequncy (msec)

#define DELTA_T		(INT_TIME/1000.)		// 0.002 sec

#define	INT_TIME1			10	// second timer interrupt frequency (msec)
#define HOME_Z_POS			-0.56f	// z-position when home-pos
#define WALK_READY_Z_POS	-0.52f	// z-position when walk-ready
#define WALKING_Z_HEIGHT	0.035f	// z-direction position from the ground when walking

#define	PELVIS_WIDTH		0.177f
#define LENGTH_THIGH		0.28f
#define LENGTH_CALF			0.28f


#define N_COLUMN_SAVE	30		// by Inhyeok for data saving
#define N_ROW_SAVE		8000	// by Inhyeok for data saving



// Axis ID. ----------------------------------------------------------------------------------//
#define NO_OF_JOINT		38
#define NO_OF_JOINT_MC	28

#define		RHY		0		//	Right Hip Yaw
#define		RHR		1		//	Right Hip Roll
#define		RHP		2		//	Right Hip Pitch
#define		RKN		3		//	Right Knee Pitch
#define		RAP		4		//	Right Ankle Pitch
#define		RAR		5		//	Right Ankle Roll
#define		LHY		6		//	Left Hip Yaw
#define		LHR		7		//	Left Hip Roll
#define		LHP		8		//	Left Hip Pitch
#define		LKN		9		//	Left Knee Pitch
#define		LAP		10		//	Left Ankle Pitch
#define		LAR		11		//	Left Ankle Roll

#define		RSP		12		//	Right Shoulder Pitch
#define		RSR		13		//	Right Shoulder Pitch
#define		RSY		14		//	Right Shoulder Roll
#define		REB		15		//	Right Elbow Pitch
#define		RWY		16		// right wrist yaw
#define		RWP		17		// right wrist Pitch

#define		LSP		18		//	Left Shoulder Pitch
#define		LSR		19		//	Left Shoulder Yaw
#define		LSY		20		//	Left Shoulder Roll
#define		LEB		21		//	Left Elbow Pitch
#define		LWY		22		// left wrist yaw
#define		LWP		23		// left wrist pitch

#define		NKY		24		// neck yaw
#define		NK1		25		// neck 1
#define		NK2		26		// neck 2

#define		WST		27		//	Trunk Yaw

#define		RF1		28		//	Right Finger
#define		RF2		29		//	Right Finger
#define		RF3		30		//	Right Finger
#define		RF4		31		//	Right Finger
#define		RF5		32		//	Right Finger
#define		LF1		33		//	Left Finger
#define		LF2		34		//	Left Finger
#define		LF3		35		//	Left Finger
#define		LF4		36		//	Left Finger
#define		LF5		37		//	Left Finger
// -------------------------------------------------------------------------------------------//
// motion 
#define		NoOfMotion		40
#define		TfLimit			230
#define		MocapTimeLimit	6500




// Joint variable structure ------------------------------------------------------------------//
typedef struct _JOINT_
{
	// for "reference command"
	float			RefAngleCurrent;	// reference to move at this step
	float			RefAngleDelta;		// reference of the past step
	float			RefAngleToGo;		// goal position - initial position
	float			RefAngleInitial;	// initial position
	float			RefAngleFF;			// feed-forward reference (ZMP initialization)
	unsigned long	GoalTimeCount;		// the time at which the goal is reached
	unsigned long	CurrentTimeCount;	// current time count
	unsigned long	DelayTimeCount;		// delay time count
	bool			MoveFlag;			// move flag
	float			WalkingRefAngle;	// joint angle from walking pattern

	// for joint setting
	float			PPR;				// pulse per revolution (output axis)
	unsigned char	JMC;				// board ID
	unsigned char	JointID;			// joint ID
	unsigned char	Motor_channel;		// motor channel
	unsigned char	CAN_channel;		// CAN channel
	unsigned int	Ref_txdf;			// Reference CAN ID
	unsigned char	SearchDirection;	// limit search direction
	char			Positive_dir;		// positive direction
	float			Offset_angle;		// offset angle
	unsigned char	Limit_rev;			// maximum limit search rev.
	int				Position_Kp;		// position control p-gain
	int				Position_Kd;		// position control d-gain
	int				Position_Ki;		// position control i-gain
	int				Torque_Kp;			// torque control p-gain
	int				Torque_Kd;			// torque control d-gain
	int				Torque_Ki;			// torque control i-gain
	unsigned int	Encoder_size;		// motor encoder pulse per revolution (input axis)
	int				HDReduction;		// harmonic gear reduction ratio
	int				Pulley_drive;		// drive side pulley teeth 
	int				Pulley_driven;		// driven side pulley teeth
	unsigned int	JAMmsTime;			// JAM fault detection time in msec
	unsigned int	PWMmsTime;			// PWM fault detection time in msec
	unsigned char	JAMDuty;			// Duty for JAM fault check
	unsigned char	PWMDuty;			// Duty for PWM fault
	unsigned int	MaxVel;				// maximum velocity
	unsigned int	MaxAcc;				// maximum acceleration
	unsigned int	MaxPWM;				// maximum pwm
	unsigned int	MaxVelHome;			// maximum velocity for home searching
	unsigned int	MaxAccHome;			// maximum acceleration for home searching
	unsigned char	HomeSearchMode;		// home search mode
	float			LowerPositionLimit;	// lower position limit
	float			UpperPositionLimit;	// upper position limit
	unsigned char	LowerLimitEnabled;	// lower position limit enabled
	unsigned char	UpperLimitEnabled;	// upper position limit enabled
	unsigned char	Deadzone;			// deadzone 
	unsigned int	I_ERR;				// current error
	unsigned int	B_ERR;				// big error
	unsigned int	E_ERR;				// 
	unsigned char	MotorControlMode;	// motor control mode

	
	// for status
	int				EncoderValue;		// current encoder value


	// for damping control
	float			ControlDampAngleCurent;	// current damping control input
	float			ControlDampAnglePast;	// past damping control input
	
	// for vibration control
	float			ControlVibrationAngle;	// vibration control input angle (for HIP roll only)

	float			ControlAngleFF;			// feed-forward control input
	float			ControlAngleFFDelta;	// feed-forward (goal-start) angle
	float			ControlAngleFFToGo;		// feed-forward goal angle
	float			ControlAngleFFInitial;	// feed-forward start angle
	unsigned long	ControlFFGoalTimeCount;		// the time at which the goal is reached
	unsigned long	ControlFFCurrentTimeCount;	// current time count
	unsigned long	ControlFFDelayTimeCount[2];	// delay time count
	bool			ControlFFMoveFlag;		// feed-forward control move flag
	unsigned char	ControlFFFunctionMode;	// function mode
	
	//for motion capture
	float			WalkReadyAngle;
	float			RefVelCurrent;	// reference to move at this step

	// for joint status
	unsigned char	HomeStatus;
	bool			FetOnOff;
	bool			ControlOnOff;
	//bool			LimitOnOff;
	unsigned char	LimitOnOff;
	//bool			ControlMode;

	bool			JAMError;
	bool			PWMError;
	bool			BigError;
	bool			EncoderError;
	bool			FaultError;
	bool			Motor0Error;
	bool			Motor1Error;

	bool			ULimitError;
	bool			LLimitError;
	bool			VelError;
	bool			AccError;
	bool			TempError;

} JOINT, *PJOINT;
//--------------------------------------------------------------------------------------------//




// FT sensor ID ------------------------------------------------------------------------------//
#define NO_OF_FT	4

#define		RFFT	0	// right foot FT sensor
#define		LFFT	1	// left foot FT sensor
#define		RWFT	2	// right wrist FT sensor
#define		LWFT	3	// left wrist FT sensor
//--------------------------------------------------------------------------------------------//

// FT sensor variable structure --------------------------------------------------------------//
typedef struct _FT_
{
	short			dMx;		// raw digit value of "Mx"
	short			dMy;		// raw digit value of "My"
	short			dFz;		// raw digit value of "Fz"
	float			Mx;			// numerical value of "Mx"
	float			My;			// numerical value of "My"
	float			Fz;			// numerical value of "Fz"
	float			Filtered_Mx;	// filtered numerical value of "Mx"
	float			Filtered_My;	// filtered numerical value of "My"
	float			Filtered_Fz;	// filtered numerical value of "Fz"

	short			dAccRoll;			// raw digit value of "Acc. Roll"
	short			dAccPitch;			// raw digit value of "Acc. Pitch"
	short			dAccRollOld;		// raw digit old value of "Acc. Roll"
	short			dAccPitchOld;		// raw digit old value of "Acc. Pitch"
	short			dAccRoll_Offset;	// raw digit value of "Acc. Roll" offset
	short			dAccPitch_Offset;	// raw digit value of "Acc. Pitch" offset

	float			AccRoll;		// numerical value of "Acc. Roll"
	float			AccPitch;		// numerical value of "Acc. Pitch"
	float			AccRollOld;		// numerical old value of "Acc. Roll"
	float			AccPitchOld;	// numerical old value of "Acc. Pitch"

	float			Roll;		// numerical value of "Roll"
	float			Pitch;		// numerical value of "Pitch"

	float			VelRoll;		// numerical value of "Roll angular velocity"
	float			VelPitch;		// numerical value of "Pitch angular velocity"
	float			VelRollOld;		// numerical old value of "Roll angular velocity"
	float			VelPitchOld;	// numerical old value of "Pitch angular velocity"

	float			CutOffFeq;		// Cut-off frequency for all FT sensor filters
//	float			CutOffMx;		// Cut-off frequency of Mx filter  
//	float			CutOffMy;		// Cut-off frequency of My filter  
//	float			CutOffFz;		// Cut-off frequency of Fz filter 
//	float			CutOffACCRoll;	// Cut-off frequency of Acc. Roll filter  
//	float			CutOffACCPitch;	// Cut-off frequency of Acc. Pitch filter 
//	float			CutOffVelRoll;	// Cut-off frequency of Vel. Roll filter  
//	float			CutOffVelPitch;	// Cut-off frequency of Vel. Pitch filter 
	
	float			DCMat[3][3];	// decoupling matrix
	float			SF_Roll;	// scale factor of "Roll" : Digit->degree
	float			SF_Pitch;	// scale factor of "Pitch" : Digit->degree

	// Alpha			1/(1+2.*PI*CutOddMx*CONTROL_PERIOD_LOWER_ms/1000.)
	//float			Alpha;	// filter

	unsigned char	Controller_NO;	// Controller number			
	unsigned char	CAN_channel;		// CAN channel
} FT, *PFT;
//--------------------------------------------------------------------------------------------//



// IMU ID ------------------------------------------------------------------------------------//
#define		NO_OF_IMU	1
#define		CENTERIMU	0
//--------------------------------------------------------------------------------------------//



// IMU variable structure --------------------------------------------------------------------//
typedef struct _IMU_
{
	short			dRoll;
	short			dPitch;
	short			dRoll_Velocity;
	short			dPitch_Velocity;

	float			Roll;
	float			RollOffset;
	float			Pitch;
	float			PitchOffset;
	float			Roll_Velocity;
	float			Pitch_Velocity;
	
	unsigned char	Controller_NO;	// Controller number
	unsigned char	CAN_channel;		// CAN channel
} IMU, *PIMU;
//--------------------------------------------------------------------------------------------//




// commands from win32 to RTX ----------------------------------------------------------------//
typedef enum 
{	
	NO_ACT,
	EXIT_PROGRAM,
	CAN_CH0_TX,		
	CAN_CH1_TX,
	SET_MOTOR_GAIN,
	SET_CURRENT_GAIN,
	LOAD_PARAMETER,
	SAVE_PARAMETER,
	SET_JOINT_PARAMETER,
	GET_JOINT_PARAMETER,
	SET_BOARD_PARAMETER,
	GET_BOARD_PARAMETER,
	PRINT_JOINT_PARAMETER,
	CHECK_DEVICE,	
	GAIN_SETTING,	
	ENABLE_FET,
	ENABLE_FET_EACH,
	DISABLE_FET,	
	DISABLE_FET_EACH,
	RUN_CMD,
	RUN_CMD_EACH,
	STOP_CMD,	
	STOP_CMD_EACH,
	GOTO_LIMIT_POS,	
	GOTO_LIMIT_POS_UPPER_ALL,
	GOTO_LIMIT_POS_LOWER_ALL,
	GOTO_LIMIT_POS_ALL,
	ENCODER_ZERO,
	ENCODER_ZERO_EACH,
	SET_ENCODER_RESOLUTION,
	SET_DEADZONE,
	SET_JAMPWM_FAULT,
	SET_MAX_VEL_ACC,
	SET_CONTROL_MODE,
	SET_HOME_SEARCH_PARAMETER,
	SET_HOME_MAX_VEL_ACC,
	SET_POSITION_LIMIT,
	SET_ERROR_BOUND,
	REQUEST_PARAMETER,
	POSITION_LIMIT_ONOFF,
	BEEP,
	JOINT_REF_SET_RELATIVE,
	JOINT_REF_SET_ABS,
	SET_FT_PARAMETER,
	GET_FT_PARAMETER,
	NULL_FT_SENSOR,
	NULL_WRIST_FT_SENSOR,
	NULL_FOOT_ANGLE_SENSOR,
	NULL_IMU_SENSOR,
	PRINT_FT_PARAMETER,
	SET_IMU_PARAMETER,
	GET_IMU_PARAMETER,
	PRINT_IMU_PARAMETER,
	SET_DAMPING_GAIN,
	GOTO_WALK_READY_POS,
	GOTO_HOME_POS,
	START_ZMP_INITIALIZATION,
	STOP_ZMP_INITIALIZATION,
	GOTO_FORWARD,
	STOP_WALKING,
	SET_MOCAP,
	C_CONTROL_MODE,
	P_CONTROL_MODE,
	GRIP_ON,
	GRIP_OFF,
	GRIP_STOP,
	DEMO_FLAG,
	TEST_FUNCTION,
	INIT_WB_MOCAP	// by Inhyeok
} _COMMAND_FLAG;
//--------------------------------------------------------------------------------------------//
typedef enum
{
	NO_DEMO,
	ONE_LEG_STAND,
	ONE_LEG_UP_AND_DOWN,
	WALKREADY_POS,
	DSP_ON,
	DSP_OFF
} _DEMO_FLAG;


// commands status from RTX to win32 ---------------------------------------------------------//
typedef enum 
{	
	NO_STATUS	
} _COMMAND_FLAG_STATUS;
//--------------------------------------------------------------------------------------------//



// Joint control mode ------------------------------------------------------------------------//
typedef enum 
{	
	CTRLMODE_NONE,
	CTRLMODE_POSITION_CONTROL_WIN,
	CTRLMODE_WALKING,
	CTRLMODE_ZMP_INITIALIZATION,
	CTRLMODE_DEMO,
	CTRLMODE_WB_MOCAP,	// by Inhyeok
	CTRLMODE_JW_MOCAP,	// by Inhyeok
	CTRLMODE_HANDSHAKE
} _CONTROL_MODE;
//--------------------------------------------------------------------------------------------//



// Sensor control mode -----------------------------------------------------------------------//
typedef enum 
{
	SENMODE_NONE
} _SENSOR_MODE;
//--------------------------------------------------------------------------------------------//



#define RIGHT	0
#define LEFT	1
#define X		0
#define Y		1
#define Z		2
#define Yaw		3

#define NORMAL_LANDING				0
#define RIGHT_FOOT_EARLY_LANDING	1
#define LEFT_FOOT_EARLY_LANDING		2
#define RIGHT_FOOT_LATE_LANDING		3
#define LEFT_FOOT_LATE_LANDING		4

#define	NO_LANDING	0
#define DSP			1
#define RIGHT_SSP	2
#define LEFT_SSP	3


// walking information structure -------------------------------------------------------------//
typedef struct _WALKING_INFO_
{
	float			RefPatternCurrent;	// reference pattern to move at this step
	float			RefPatternDelta;	// reference pattern of the past step
	float			RefPatternToGo;		// goal pattern position - initial pattern position
	float			RefPatternInitial;	// initial pattern position

	unsigned long	GoalTimeCount;		// the time at which the goal is reached
	unsigned long	CurrentTimeCount;	// current time count
	unsigned long	DelayTimeCount[2];	// delay time count
	unsigned char	FunctionMode;		// move function type 0:cosine, 1:sine
	float			UserData[5];			// user defined data
	unsigned char	CurrentSequence;	// current move sequence
	bool			MoveFlag;			// move flag

	float			RefPosFF;			// feed-forward reference (ZMP initialization)
	float			ControlDSPZMP;		// ZMP control input at DSP
} WALKING_INFO, *PWALKING_INFO;
//--------------------------------------------------------------------------------------------//



// walking sequence information --------------------------------------------------------------//
typedef enum
{
	WALK_READY,//0
	// basic sway (hip sway)
	HIP_CENTER_TO_LEFT_SET,
	HIP_CENTER_TO_RIGHT_SET,
	HIP_LEFT_TO_RIGHT_SET,
	HIP_RIGHT_TO_LEFT_SET,
	HIP_TO_CENTER,
	// z-direction (foot upward)
	FIRST_RIGHT_FOOT_UP_SET,
	FIRST_RIGHT_FOOT_DOWN_SET,
	FIRST_LEFT_FOOT_UP_SET,
	FIRST_LEFT_FOOT_DOWN_SET,
	RIGHT_FOOT_UP_SET,
	RIGHT_FOOT_DOWN_SET,
	LEFT_FOOT_UP_SET,
	LEFT_FOOT_DOWN_SET,
	// x-direction (foot forward)
	FIRST_RIGHT_FOOT_UP_FORWARD_SET,
	FIRST_LEFT_FOOT_UP_FORWARD_SET,
	FIRST_RIGHT_FOOT_DOWN_FORWARD_SET,
	FIRST_LEFT_FOOT_DOWN_FORWARD_SET,
	RIGHT_FOOT_DOWN_FORWARD_SET,
	LEFT_FOOT_UP_FORWARD_SET,
	LEFT_FOOT_DOWN_FORWARD_SET,
	RIGHT_FOOT_UP_FORWARD_SET
} _WALKING_PHASE;
//--------------------------------------------------------------------------------------------//



// saving parameters -------------------------------------------------------------------------//
typedef struct _SAVE_PARAMETERS_
{
	// for joint parameter saving
	unsigned char	JMC[NO_OF_JOINT];				
	unsigned char	Motor_channel[NO_OF_JOINT];		
	unsigned char	CAN_channel[NO_OF_JOINT];		
	unsigned int	Ref_txdf[NO_OF_JOINT];
	int				HDReduction[NO_OF_JOINT];
	int				Pulley_drive[NO_OF_JOINT];
	int				Pulley_driven[NO_OF_JOINT];
	int				Encoder_size[NO_OF_JOINT];
	float			PPR[NO_OF_JOINT];					

	// for FT sensor parameter saving
	unsigned char	FTControllerNO[NO_OF_FT];
	unsigned char	FTCANchannel[NO_OF_FT];
	float			FTCutOff[NO_OF_FT];
	
	// for IMU sensor parameter saving
	unsigned char	IMUControllerNO[NO_OF_IMU];
	unsigned char	IMUCANchannel[NO_OF_IMU];
} SAVE_PARAMETERS, *PSAVE_PARAMETERS;
//--------------------------------------------------------------------------------------------//




#endif