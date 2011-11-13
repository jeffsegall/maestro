// file name: CAN.h
// object: CAN device(PCM-3680) setting
// by Jungho Lee (jungho77@rainbow.re.kr)
// All rights are reserved.

#ifndef CAN_H
#define CAN_H

#include "CANID.h"

/******************************************************************************/
// Definition of Registers for CAN controller
#define		MBSIZE			353		// Number of message
#define		IRQ_POLLING     0		// CAN receive message check method

#define		IMR1			0x21    // Interrupt Mask Register 1 (IMR1)
#define		IMR2			0xA1    // Interrupt Mask Register 2 (IMR2)

#define		EOI1			0x20    // End of Interrupt 1
#define		EOI2			0xA0    // End of Interrupt 2

#define		CR				0       // Control Register
#define		CMR				1       // Command Register
#define		SR				2       // Status Register
#define		IR				3       // Interrupt Register
#define		ACR				4       // Acceptance Code Register
#define		AMR				5       // Acceptance Mask Register
#define		BTR0			6       // Bus Timing Register 0
#define		BTR1			7       // Bus Timing Register 1
#define		OCR				8       // Output Control Regsiter
#define		TR				9       // Test Register

#define		TXID1			10      // Identifier 1 of Transmit Buffer
#define		TXID2			11      // Identifier 2 of Transmit Buffer
#define		TXDATA			12      // Transmit Data unsigned char 1

#define		RXID1			20      // Identifier 1 of Receive Buffer
#define		RXID2			21      // Identifier 2 of Receive Buffer
#define		RXDATA			22      // Receive Data unsigned char 1

#define		ERR_OK          0x00    // No Error
#define		ERR_SEG         0x10    // seg_addr set error
#define		ERR_RESET		0x11	// reset error
#define		ERR_IRQ         0x12    // irq_no set error
#define		ERR_BAUD		0x13	// Baudrate setting error
#define		ERR_HW          0xFF    // CAN hardware error

#define		ERR_NODATA      0x01    // No Received Data
#define		ERR_SEND        0x02    // Fail to send CAN msg
#define		ERR_CONFIG      0x03    // CAN Configuration fail
#define		ERR_RR          0x04    // No Reset Mode
#define		ERR_MB          0xFF    // No matching id is found

#define		NODATA          0x00    // No data is available
#define		NEWDATA         0x01    // New received data is available
#define		OVERRUN         0x02    // Data Overrun
#define		TRANSMITTED     0x03    // Data has been transmitted

#define		CAN0			0x00	// CAN channel 1
#define		CAN1			0x01	// CAN channel 2
// End of Definition for CAN Controller
/******************************************************************************/



/******************************************************************************/
// Definition of Data Structures for CAN Communication
typedef struct _MYCAN
{
	unsigned char irq;			// IRQ number
	unsigned char old_irq;		// Old IRQ number

	unsigned char acc_code;		// Acceptance Code Register (ACR)
	unsigned char acc_mask;		// Acceptance Mask Register (AMR)
	unsigned char btr0;			// Bus Timing Register 0 (BTR0)
	unsigned char btr1;     	// Bus Timing Register 1 (BTR1)
} MYCAN;
typedef struct _MYMB
{
	unsigned short	id;			// Identifier
	unsigned short	idread;		// read Identifier
	unsigned char	rtr;     	// Remote Transmission Request
	unsigned char	dlc;		// Data Length Code
	unsigned char	data[8];	// Data unsigned char
	unsigned char	status;		// status of message buffer
} MYMB;
enum Bitrate 
{ 
	BAUD_1M,     // =0: 1 Mbit/sec
	BAUD_500K,   // =1: 500 kbit/sec
	BAUD_250K,   // =2: 250 kbit/sec
	BAUD_125K,   // =3: 125 kbit/sec
	BAUD_100K    // =4: 100 kbit/sec
};
// End of definition for CAN communication
/******************************************************************************/

extern MYCAN CAN[2];
extern MYMB MB[MBSIZE];
extern unsigned char rBuf[2];


// CAN device setting functions
void			StartCAN(void);		// CAN device initialization procedure (InitCANHW, InitCAN and InitMB are called)
int				InitCANHW(void);	// CAN device setting
int				InitCAN(unsigned char channel, enum Bitrate BAUD);		// CAN channel setting
int				CanConfig(unsigned char channel);						// CAN device common register setting
int				CanNormalRun(unsigned char channel);					// set CAN channels as "normal run" mode
void			CanReset(unsigned char channel);						// reset register as hardware initial value
void			CanWrite(unsigned char channel, unsigned int addr, unsigned char data);		// CAN device register write 
unsigned char	CanRead(unsigned char channel, unsigned int offset);						// CAN device register read

// Transmit and receive CAN message function
int				CanSendMsg(unsigned char channel, unsigned int id, unsigned char *data,
						   unsigned char dlc, unsigned char rtr);							// send CAN message
int				CanReceiveMsg(unsigned char channel, unsigned int id);						// read CAN message (specific ID)
int				CanReceiveMsg(unsigned char channel);										// read CAN message

// Message Box related function
void			InitMB(void);		// Initialize CAN message box ( user setting function!! )
unsigned int	GetBuffno(unsigned int id);	// get MB related with input CAN ID
int				ReadMBData(unsigned int id, unsigned char *data);		// read message data related with input CAN ID
void			ConfigMB(unsigned char buff_no, unsigned int id);		// set message box with input id (with MB array position)
void			ConfigMB(unsigned int id);								// set message box with input id

#endif










