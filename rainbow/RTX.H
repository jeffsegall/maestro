#include <windows.h>
#include <rtapi.h>
#include <stdio.h>
#include "SharedMemory.h"
#include "CommonDefinition.h"

/******************************************************************************/
#define timeInt		10000*INT_TIME
#define	timeInt1	10000*INT_TIME1
#define baseAddr	0xDA00
/******************************************************************************/

/******************************************************************************/
extern PSHARED_DATA	pSharedMemory;	// shared memory data
extern HANDLE hSharedMemory;		// shared memory handler
extern HANDLE hTimer;				// timer handler for lower body control
extern HANDLE hTimer1;				// timer handler for upper body control
/******************************************************************************/

void RtxInit(void);		// RTX initialization (timer, shared memory and CAN device)
void RtxEnd(void);		// RTX close routine before termination

// timer interrupt service routine for lower body control
void RTFCNDCL TimerHandler(void *nContext);
// timer interrupt service routine for upper body control
void RTFCNDCL TimerHandler1(void *nContext);




















