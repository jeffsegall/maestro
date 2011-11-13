#include "RTX.h"

PVOID vAddress;
/******************************************************************************/
void RtxInit(void)
{
	unsigned long temp;
	LARGE_INTEGER physAddr;
	LARGE_INTEGER waiting, waiting1;
	LARGE_INTEGER liPeriod, liPeriod1;

	// Shared memory ----------------------------------------------------------//
	hSharedMemory = RtOpenSharedMemory(
										SHM_MAP_WRITE,				// access mode
										FALSE,						// don't care
										L"Can Shared Data",			// name of shared memory
										(VOID **)&pSharedMemory);	// shared memory data address
	if (hSharedMemory == NULL)
	{
		RtWprintf(L"\n>>> RtOpenSharedMemory error = %d\n", GetLastError());
		ExitProcess(1);
	}
	else RtWprintf(L"\n>>> RtOpenSharedMemory is OK..!!");
	// ------------------------------------------------------------------------//

	// Memory mapping for CAN device(Advantech PCM-3680) ----------------------//
	temp = baseAddr;
	temp = temp << 4;
	physAddr.QuadPart = temp;
	vAddress = RtMapMemory(physAddr, 0xDFFF, 1);
	if(vAddress==NULL) RtWprintf(L"\n>>> Failure on RtMapMemory..!\n");
	else;
	// ------------------------------------------------------------------------//



	// timer interrupt handler creation for lower body control ----------------//
	liPeriod.QuadPart = timeInt;
	waiting.QuadPart = 30000000;	// timer will be started after 3 sec later.
	if (! (hTimer = RtCreateTimer(
                                  NULL,            // security
                                  0,               // stack size - 0 uses default
                                  TimerHandler,    // timer handler
                                  NULL,            // NULL context (argument to handler)
                                  RT_PRIORITY_MAX, // priority
                                  CLOCK_2) ))      // RTX HAL timer
    {
        RtWprintf(L"\n>>> RtCreateTimer for %d msec error = %d\n", INT_TIME, GetLastError());
        ExitProcess(1);
    }
	else;
    if (! RtSetTimerRelative( hTimer,              // hTimer
                              &waiting,            // pExpiration
                              &liPeriod) )         // pInterval
    {
        RtWprintf(L"\n>>> RtSetTimerRelative error (%d msec) = %d\n", INT_TIME, GetLastError());
        ExitProcess(1);
    }
	else;
	// ------------------------------------------------------------------------//


	
	// timer interrupt handler creation for upper body control ----------------//
	liPeriod1.QuadPart = timeInt1;
	waiting1.QuadPart = 30000000;	// timer will be started after 3 sec later.
	if (! (hTimer1 = RtCreateTimer(
                                  NULL,            // security
                                  0,               // stack size - 0 uses default
                                  TimerHandler1,   // timer handler1
                                  NULL,            // NULL context (argument to handler)
                                  RT_PRIORITY_MAX, // priority
                                  CLOCK_2) ))      // RTX HAL timer
    {
        RtWprintf(L"\n>>> RtCreateTimer for %d msec error = %d\n", INT_TIME1, GetLastError());
        ExitProcess(1);
    }
	else;
    if (! RtSetTimerRelative( hTimer1,              // hTimer1
                              &waiting1,            // pExpiration1
                              &liPeriod1) )         // pInterval1
    {
        RtWprintf(L"\n>>> RtSetTimerRelative error (%d msec) = %d\n", INT_TIME1, GetLastError());
        ExitProcess(1);
    }
	else;
	// ------------------------------------------------------------------------//
}
/******************************************************************************/



/******************************************************************************/
void RtxEnd(void)
{
	RtCloseHandle(hTimer);
	RtCloseHandle(hTimer1);
	BOOL temp = RtUnmapMemory(vAddress);
	RtCloseHandle(hSharedMemory);
	RtWprintf(L"\n>>> This program is for experiment only..!!");
	RtWprintf(L"\n>>> If you have any questtion, e-mail me : jungho77@rainbow.re.kr");
	RtWprintf(L"\n>>> RAINBOW forever..!!");
}
/******************************************************************************/
