#ifndef SHAREDMEMORY_H
#define SHAREDMEMORY_H

#include "CommonDefinition.h"
#include "CAN.h"

typedef struct _SHARED_DATA_
{	
// win32 -> RTX ---------------------------------------------------------------------------------//
	// CAN Message를 주고 받을때 사용되는 변수
	// 보내고자 하는 데이터를 아래의 변수에 할당 한 후 사용.
	// for Channel 0
	unsigned short		Tx_ID0;			// CAN ID.
	unsigned char		Tx_Data0[8];	// CAN data
	unsigned char		Tx_DLC0;		// data length code
	// for channel 1
	unsigned short		Tx_ID1;			// CAN ID.
	unsigned char		Tx_Data1[8];	// CAN data
	unsigned char		Tx_DLC1;		// data length code

	// for win32 command flag
	unsigned char		CommandFlag;			// command flag
	unsigned int		CommandData;			// command data variable used when the command flag needs
//	unsigned int		CommandDataInt;			// int command data variable used when the command flag needs
	float				CommandDataFloat[4];		// float command data variable used when the command flag needs
	unsigned int		CommandDataArray[4];	// array command data variable used when the command flag needs


	// getting board parameter
	unsigned int		WantedBoard;		
	unsigned int		ReturnedID;
	unsigned char		ReturnedData[8];

	// for joint control command
	//_CONTROL_MODE		MotorControlMode;	// control mode
	unsigned char		MotorControlMode;	// control mode
	float				GoalAngle;			// goal joint angle(deg)
	float				GoalTime;			// goal moving time(ms)
	unsigned char		JointID;			// control JointID
	unsigned char		BoardID;			// control BoardID
	JOINT				Joint;				// structure for each joint 

	// for sensor control command
	//_SENSOR_MODE		SensorControlMode;
	unsigned char		SensorControlMode;
	unsigned char		FTsensorID;
	unsigned char		IMUsensorID;
	FT					FTsensor;
	IMU					IMUsensor;
//-----------------------------------------------------------------------------------------------//


// RTX -> win32 ---------------------------------------------------------------------------------//
	_COMMAND_FLAG_STATUS CommandFlagStatus;
//-----------------------------------------------------------------------------------------------//


// win32 <-> RTX --------------------------------------------------------------------------------//
	 
//-----------------------------------------------------------------------------------------------//


// for debugging ---------------------------------------------------------------------------------//
	JOINT			Joint_debug[NO_OF_JOINT];
	FT				FT_debug[NO_OF_FT];
	IMU				IMU_debug[NO_OF_IMU];
	float			Data_Debug[30][8000];
	unsigned int	Data_Debug_Index;
	
	float			ZMP[6];
	float			Gain[6];
	float			Sway, ankleAdd;
	float			tempPeriod, tempDelayY, tempDSP, tempStep;
	float			TempRoll[3];

	float			Temp[15];
	unsigned char	TempChar;
	bool			tempBool;

	float			JW_temp[30];
	bool			JW_MotionSet[NoOfMotion];

	int				WB_MocapNo;		// by Inhyeok
//-----------------------------------------------------------------------------------------------//

//HandShake--------------------------------------------------------------------------------------//
	int				ShakeHandsFlag;

} SHARED_DATA, *PSHARED_DATA;

#endif
