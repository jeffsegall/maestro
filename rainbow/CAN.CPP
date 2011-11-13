// file name: CAN.cpp
// object: CAN device(PCM-3680) setting
// by Jungho Lee (jungho77@rainbow.re.kr)
// All rights are reserved.


#include "CAN.H"
#include "RTX.h"

// virtual address from RTX
// PCM-3680 CAN device uses 'Memory Mapped IO' method.
// So if you write data to the specific address, CAN device IO shuld be operated.
extern PVOID vAddress;
unsigned int MBCounter = 0;		// message box index




/******************************************************************************/
// 메세지바이트를 할당하거나 초기화 할때 사용
// 새로운 메세지를 추가할 때 이 함수를 이용하여 할당해야함
void InitMB(void)
{	
	unsigned int i;

	// 0 ~ 79 for sensor MB. : 80EA
	for(i=0 ; i<16 ; i++)
	{
		ConfigMB(SENSOR_FT_BASE_RXDF+i);
		ConfigMB(SENSOR_AD_BASE_RXDF+i);
		ConfigMB(DAOFFSET_BASE_RXDF+i);
		ConfigMB(ADOFFSET_BASE_RXDF+i);
		ConfigMB(OFFSET_BASE_RXDF+i);
	}

	// 80 ~ 127 for Encoder MB : 48EA
	for(i=0 ; i<48 ; i++) ConfigMB(ENC_BASE_RXDF+i);

	// 128 ~ 223
	for(i=0 ; i<48 ; i++) 
	{
		//ConfigMB(CUR_BASE_RXDF+i);
		//ConfigMB(PM_BASE_RXDF+i);
	}

	// 224 ~ 287 for name MB : 64 EA
	for(i=0 ; i<64 ; i++) ConfigMB(NAME_BASE_RXDF+i);

	// 288 ~ 351 
	for(i=0 ; i<64 ; i++) ConfigMB(STAT_BASE_RXDF+i);

	for(i=0x00 ; i<0x0C ; i++) ConfigMB(SETTING_BASE_RXDF+i);
	for(i=0x20 ; i<0x26 ; i++) ConfigMB(SETTING_BASE_RXDF+i);
}
/******************************************************************************/

















/******************************************************************************/
// this function is used for enabling CAN device from RTX main program
void StartCAN(void)
{
	RtWprintf(L"\n>>> Initialize CAN hardware ... ");
	
    //------ set CAN card hardware setting
	if(InitCANHW() != ERR_OK )
    {
		RtWprintf(L"\n>>> Hardware initialization error occurred !!");
		ExitProcess(1);
    }
	
    CAN[0].old_irq = CAN[1].old_irq = IRQ_POLLING;
    if(InitCAN(CAN0, BAUD_1M) != ERR_OK)  ExitProcess(1);
    if(InitCAN(CAN1, BAUD_1M) != ERR_OK)  ExitProcess(1);
	
    InitMB(); 	// Initialize CAN message buffer
	
    RtWprintf(L"\n>>> CAN initialization O.K. ");
}
/******************************************************************************/



/******************************************************************************/
// CAN hardware initialization
int InitCANHW(void)
{
	unsigned char data;
	
	// Check memory base address for PCM-3680 : 0xC000 ~ 0xDF00
	if( baseAddr>0xdf00 || baseAddr<0xc000 || (baseAddr%0x100) ) return(ERR_SEG); // Return : segment address set error
	
	// CAN0 Init. 
	data = CanRead(CAN0, CR);			// read control register (CR)
	if(data & 0x01)					// check if "Reset Request" is set (Reset Mode)
	{
		// 1. CanWrite(0, CR, 0x60);     	// Set Transmit (TIE) & Receive (RIE) Interrupt Enable bits -> initial setting
		// 2. CanWrite(0, CR, 0x06);    	// Set Transmit (TIE) & Receive (RIE) Interrupt Enable bits -> by jungho77
		CanWrite(CAN0, CR, 0x00);     	// Set Transmit (TIE) & Receive (RIE) Interrupt Enable bits to 'Low'
		data = CanRead(CAN0, CR);
		if(data & 0x01)	return(ERR_RESET);	// if "Reset Request" is set to HIGH
											// Return : reset error
	}
	CanReset(CAN0);               		// Reset CAN port 0
	data = CanRead(CAN0, CR);
	if(! (data & 0x01) ) return(ERR_RESET);		// if "Reset Request" is disabled (LOW)
	else return(ERR_OK);						// if "Reset Request" is set to HIGH


	// CAN1 Init. 
	data = CanRead(CAN1, CR);			// read control register (CR)
	if(data & 0x01)					// check if "Reset Request" is set (Reset Mode)
	{
		// 1. CanWrite(1, CR, 0x60);     	// Set Transmit (TIE) & Receive (RIE) Interrupt Enable bits -> initial setting
		// 2. CanWrite(1, CR, 0x06);     	// Set Transmit (TIE) & Receive (RIE) Interrupt Enable bits -> by jungho77
		CanWrite(CAN1, CR, 0x00);     	// Set Transmit (TIE) & Receive (RIE) Interrupt Enable bits to 'Low'
		data = CanRead(CAN1, CR);
		if(data & 0x01) return(ERR_RESET);	// if "Reset Request" is set to HIGH
											// Return : reset error
	}
	CanReset(CAN1);               		// Reset CAN port 1
	data = CanRead(CAN1, CR);
	if(! (data & 0x01) ) return(ERR_RESET);		// if "Reset Request" is disabled (LOW)
	else return(ERR_OK);						// if "Reset Request" is set to HIGH

}
/******************************************************************************/



/******************************************************************************/
// initialize each CAN channel
// Advantech PCM-3680 has 2 CAN channels
int InitCAN(unsigned char channel, enum Bitrate BAUD)
{
	//------ reset CAN controller
	CanReset(channel);
	
    //------ CAN irq setting
	CAN[channel].irq = IRQ_POLLING;
	//------ CAN configuration
    CAN[channel].acc_code = 0x00;		// Write Acceptance Code
    CAN[channel].acc_mask = 0xFF;		// Write Acceptance Mask Register
	// *** 0xFF means "no bits are compared"
    CAN[channel].btr1 = 0x14; 			// Write Bus Timing Register 1
	//CAN[channel].btr1 = 0x32; 			// Write Bus Timing Register 1
	// Sampling = Single
	// Tseg1 = 6, Tseg2 = 2
	// Sampling Point @ 75%
    //------ Write Bus Timing Register 0
    //	SJW(Synchronization Jump Width)(b7,b6) = 4
    //  BRP(Baud Rate Pre-scaler)(b5..b0) = depends on Bitrate
    switch(BAUD) 
	{
	case BAUD_1M:	CAN[channel].btr0 = 0xC0;
		break;
	case BAUD_500K: CAN[channel].btr0 = 0xC1;
		break;
	case BAUD_250K:	CAN[channel].btr0 = 0xC3;
		break;
	case BAUD_125K: CAN[channel].btr0 = 0xC7;
		break;
	case BAUD_100K:	CAN[channel].btr0 = 0xC9;
		break;
	default:		return(ERR_BAUD);       // Bitrate setting error
    }
	
    if(CanConfig(channel) != ERR_OK) return(ERR_CONFIG);
	
    // Put CAN port to normal mode
    return(CanNormalRun(channel));		// successful initialization
}
/******************************************************************************/



/******************************************************************************/
int CanConfig(unsigned char channel)
{
	unsigned char temp;
	
	CanReset(channel);	// reset CAN controller
	
	//------ Write Control Register (CR)
	CanWrite(channel, CR, 0x01);	// Enter Reset Mode without setting RIE
	// and enter reset mode (Set Reset Request bit)
	
	//------ Write Bus Timing Register 0 & 1 (BTR0 & BTR1)
	// BTR can be accessed (read/write) if the reset mode is active
	CanWrite(channel, BTR0, CAN[channel].btr0); // Write Bus Timing Register 0 (BTR0)
	CanWrite(channel, BTR1, CAN[channel].btr1); // Write Bus Timing Register 1 (BTR1)
	
	temp = CanRead(channel, BTR0);
	if(temp != CAN[channel].btr0)			// Read BTR0 and confirm it
		return(ERR_CONFIG);					// fail to configure
	
	temp = CanRead(channel, BTR1);
	if(temp != CAN[channel].btr1)			// Read BTR1 and confirm it
		return(ERR_CONFIG);					// fail to configure
	
	//------ Write Acceptance Code Register (ACR) and
	//		 Acceptance Mask Register (AMR)
	CanWrite(channel, ACR, CAN[channel].acc_code);        // Write ACR
	CanWrite(channel, AMR, CAN[channel].acc_mask);        // Write AMR
	
	//------ Write Output Control Register (OCR)
	//   Set Normal Output Mode & Push-pull driver
	CanWrite(channel, OCR, 0xfa);
	
	return(ERR_OK);
}
/******************************************************************************/



/******************************************************************************/
// set CAN channel as "Normal" mode
int CanNormalRun(unsigned char channel)
{
	unsigned char v = CanRead(channel, CR);	// read Control Register (CR)
	
	if(v & 0x01)	        // if Reset Request(RR) is set (Reset Mode)
	{
		CanWrite(channel, CR, v&0xfe);
		// On the '1-to-0' transition of RR bit,  CAN controller returns to "Operating Mode"
		return(ERR_OK);
	}
	else return(ERR_RR); // Error = current mode is not Reset Mode
}
/******************************************************************************/



/******************************************************************************/
// CAN Reset
void CanReset(unsigned char channel)
{	
	unsigned char temp;
	temp = CanRead(channel, 0x0100);
	CanWrite(channel, 0x0100, temp);
	Sleep(100); 	// wait 100ms
}
/******************************************************************************/



/******************************************************************************/
// write data to CAN board(PCM-3680) memory(register)
// it's not a CAN message sending function
void CanWrite(unsigned char channel, unsigned int offset, unsigned char data)
{
	unsigned char *temp = (unsigned char *)vAddress;
	*(temp + offset + 0x200*channel) = data;
}
/******************************************************************************/



/******************************************************************************/
// read data to CAN board(PCM-3680) memory
// it's not a CAN message reading function
unsigned char CanRead(unsigned char channel, unsigned int offset)
{
	unsigned char *temp = (unsigned char *)vAddress;
	unsigned char data = *(temp + offset + 0x200*channel);
	return data;
}
/******************************************************************************/



/******************************************************************************/
// initialize MB(MessageByte)
void ConfigMB(unsigned char buff_no, unsigned int id)
{
	unsigned char i;
	
	MB[buff_no].id = id;	 		// assigned id
	
	MB[buff_no].idread = 0x0000;    // read id
	MB[buff_no].rtr = 0x00;     	// Remote Transmission Request
	MB[buff_no].dlc = 0x00;			// Data Length Code
	
	for(i=0; i<8; i++) MB[buff_no].data[8]=0x00;	// Data unsigned char
	MB[buff_no].status = NODATA;	// initial MB status = No Data
}
/******************************************************************************/



/******************************************************************************/
// initialize MB(MessageByte) : auto-counting
void ConfigMB(unsigned int id)
{
	unsigned char i;
	
	MB[MBCounter].id = id;	 			// assigned id
	
	MB[MBCounter].idread = 0x0000;		// read id
	MB[MBCounter].rtr = 0x00;     		// Remote Transmission Request
	MB[MBCounter].dlc = 0x00;			// Data Length Code
	
	for(i=0; i<8; i++) MB[MBCounter].data[8]=0x00;	// Data unsigned char
	MB[MBCounter].status = NODATA;		// initial MB status = No Data

	//RtWprintf(L"\n>>> MBCount : %d", MBCounter);
	//RtWprintf(L"\n>>> ID: %d", id);

	MBCounter++;
}
/******************************************************************************/



/******************************************************************************/
// send CAN message
int CanSendMsg(unsigned char channel, unsigned int id, unsigned char *data, unsigned char dlc, unsigned char rtr)
{
    unsigned char v, i, tempData[8];

	for(i=0 ; i<8 ; i++) tempData[i] = data[i];
	
    v = id >> 3;
    CanWrite(channel, TXID1, v);	// write Identifier (ID.10 .. ID.3)
    v = id & 0x07;        			// write Identifier (ID.2 .. ID.0) to b7..b5
    v <<= 5;
	
    if(rtr==1) v |= 0x10;  			// check Remote Transmit Request (RTR) bit
	
    v += dlc;            			// add Data Length Code (DLC)
    CanWrite(channel, TXID2, v);	// Write Identifier unsigned char 2
	
    for(i=0; i<dlc; i++) CanWrite(channel, TXDATA+i, *(data+i));	// write Data unsigned char
	//for(i=0; i<dlc; i++) CanWrite(channel, TXDATA+i, *(tempData+i));	// write Data unsigned char
	//for(i=0; i<dlc; i++) CanWrite(channel, TXDATA+i, tempData[i]);	// write Data unsigned char
	
    // Write TR of Command Register (CMR)
    CanWrite(channel, CMR, 0x01);	// Set Transmission Request (TR): message will be transmitted
	
    while(1)
    {
		v = CanRead(channel, SR);	// Read Status Register (SR)
		if(v & 0x40)				// If Error Status(ES) bit is set
		{
			//RtWprintf(L"\n >>> CAN send error..!!");
			return(ERR_SEND);		// fail to send
		}
		if(v & 0x08)				// when Transmission Complete Status (TCS) bit is set
			return(ERR_OK);	        // return (=successful)
    }
}
/******************************************************************************/



/******************************************************************************/
// receive CAN message(specific ID)
int CanReceiveMsg(unsigned char channel, unsigned int id)
{
	unsigned char i, buff_no, rBuf[2];
	unsigned short idread;
	
	if(CAN[channel].irq == IRQ_POLLING)	// polling
	{
		if(CanRead(channel, SR) & 0x01)	// Read Status Register (SR)
		{	// Check if Receive Buffer Status (RBS) is set
			// if RBS=1, one or more msg are available in RXFIFO
			rBuf[0] = CanRead(channel, RXID1);	// read Identifier (ID.10 .. ID.3)
			rBuf[1] = CanRead(channel, RXID2);	// read Identifier(ID.2 .. ID.0), RTR, DLC(=lower nibble)
			
			idread = rBuf[0];
			idread <<= 3;
			idread |= (rBuf[1]>>5);			// extract id from rBuf
			buff_no = GetBuffno(idread);    // get message buffer number
			
			//RtWprintf(L"\nbuff_no = %d", buff_no);
			MB[buff_no].idread = idread; 			// read id
			//RtWprintf(L"\nidread = %x", idread);
			MB[buff_no].rtr = (rBuf[1]>>4) & 0x01;	// RTR
			//RtWprintf(L"\nrtr = %x", MB[buff_no].rtr);
			MB[buff_no].dlc = rBuf[1] & 0x0f;     	// DLC
			//RtWprintf(L"\ndlc = %x", MB[buff_no].dlc);
			
			
			for(i=0; i<8; i++)
			{
				MB[buff_no].data[i] = CanRead(channel, RXDATA+i);	// read data unsigned char according to DLC(Data Length Code)
			}
			
			// release receive buffer (RRB) of Command Register (CMR)
			CanWrite(channel, CMR, 0x04); 	// release memory space in RXFIFO
			
			// Write NEWDATA flag to Status unsigned char
			if(MB[buff_no].status==NODATA) MB[buff_no].status = NEWDATA;
			else if(MB[buff_no].status==NEWDATA) MB[buff_no].status = OVERRUN;
			else;
		}
	}
	else;

	// Read CAN msg if new data is available
	buff_no = GetBuffno(id);
	if(MB[buff_no].status==NODATA) return(ERR_NODATA);		// Error = No Received Data
	else
	{
		MB[buff_no].status = NODATA;	// Set CAN status unsigned char to NODATA
		return(ERR_OK);					// No errors detected
	} 
}
/******************************************************************************/



/******************************************************************************/
// receive CAN message
int CanReceiveMsg(unsigned char channel)
{
	unsigned char i, canBox, buff_no, rBuf[2];
	unsigned short idread;
	
	if(CAN[channel].irq == IRQ_POLLING)	// polling
	{
		//if(CanRead(channel, SR) & 0x01)	// Read Status Register (SR)
		canBox = 0;
		while( ((CanRead(channel, SR) & 0x01) == 0x01) && (canBox < 32) )
		{								// Check if Receive Buffer Status (RBS) is set
			// if RBS=1, one or more msg are available in RXFIFO
			rBuf[0] = CanRead(channel, RXID1);	// read Identifier (ID.10 .. ID.3)
			rBuf[1] = CanRead(channel, RXID2);	// Read Identifier(ID.2 .. ID.0), RTR, DLC(=lower nibble)
			
			idread = rBuf[0];
			idread <<= 3;
			idread |= (rBuf[1]>>5);	// extract id from rBuf
			buff_no = GetBuffno(idread);    // get message buffer number
			//RtWprintf(L"\nbuff_no = %x", buff_no);
			MB[buff_no].idread = idread; 	// read id
			//RtWprintf(L"\nidrad = %x", idread);
			MB[buff_no].rtr = (rBuf[1]>>4) & 0x01;	// RTR
			//RtWprintf(L"\nrtr = %x", MB[buff_no].rtr);
			MB[buff_no].dlc = rBuf[1] & 0x0f;     	// DLC
			//RtWprintf(L"\ndlc = %x", MB[buff_no].dlc);
			
			for(i=0; i<MB[buff_no].dlc; i++)
			{
				MB[buff_no].data[i] = CanRead(channel, RXDATA+i);	// read data unsigned char according to DLC(Data Length Code)
			}
			
			// release receive buffer (RRB) of Command Register (CMR)
			CanWrite(channel, CMR, 0x04); 	// release memory space in RXFIFO
			
			if(MB[buff_no].status==NODATA) MB[buff_no].status = NEWDATA;    // Write NEWDATA flag to Status unsigned char
			else if(MB[buff_no].status==NEWDATA) MB[buff_no].status = OVERRUN;
			else;

			canBox++;
		}

		return MB[buff_no].status;
	}
	else return NODATA;
}
/******************************************************************************/



/******************************************************************************/
// check ID in the MB and return index
unsigned int GetBuffno(unsigned int id)
{
	unsigned int i;

	for(i=0 ; i < MBSIZE ; i++)
	{
		if(id == MB[i].id) return i;
	}

	return(MBSIZE-1);
}
/******************************************************************************/



/******************************************************************************/
// read MessageBox's data
// also check the data new or not
int ReadMBData(unsigned int id, unsigned char *data)
{
	// Read CAN msg if new data is available
	unsigned char i;
	unsigned char buff_no = GetBuffno(id);

	for(i=0; i<8 ; i++) *(data+i) = MB[buff_no].data[i];	// Read Data unsigned char from message unsigned char
	if(MB[buff_no].status==NEWDATA || MB[buff_no].status==OVERRUN)
	{
		MB[buff_no].status = NODATA;	// Set CAN status unsigned char to NODATA
		for(i=0; i<8 ; i++) MB[buff_no].data[i] = 0xFF;	// Read Data unsigned char from message unsigned char
		return(ERR_OK);					// No errors detected
	}
	else return(ERR_NODATA);			// Error = No Received Data
}
/******************************************************************************/