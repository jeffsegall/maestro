// RainbowGUIDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "RainbowGUIDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


// SharedMemory variable
	PSHARED_DATA pSharedMemory;

/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRainbowGUIDlg dialog

CRainbowGUIDlg::CRainbowGUIDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRainbowGUIDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CRainbowGUIDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRainbowGUIDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CRainbowGUIDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CRainbowGUIDlg, CDialog)
	//{{AFX_MSG_MAP(CRainbowGUIDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_EXIT, OnExit)
	ON_BN_CLICKED(IDC_RTXONOFF, OnRtxonoff)
	ON_BN_CLICKED(IDC_LOADPARAM, OnLoadparam)
	ON_BN_CLICKED(IDC_CHECKDEVICE, OnCheckdevice)
	ON_BN_CLICKED(IDC_CONTROLONOFF, OnControlonoff)
	ON_WM_TIMER()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRainbowGUIDlg message handlers

BOOL CRainbowGUIDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	// TODO: Add extra initialization here
	RTXStarted = false;

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRainbowGUIDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRainbowGUIDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRainbowGUIDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CRainbowGUIDlg::OnExit() 
{
	// TODO: Add your control notification handler code here
	if(RTXStarted == true) RTXOff();
	OnOK();
}

void CRainbowGUIDlg::OnRtxonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	GetDlgItem(IDC_RTXONOFF)->GetWindowText(strText);
	
	if(strText == "RTX ON")
	{
		RTXOn();
		GetDlgItem(IDC_RTXONOFF)->SetWindowText("RTX OFF");
		GetDlgItem(IDC_LOADPARAM)->EnableWindow(TRUE);
		RTXStarted = true;
	}
	else
	{
		RTXOff();
		GetDlgItem(IDC_RTXONOFF)->SetWindowText("RTX ON");
		RTXStarted = false;
	}
}

bool CRainbowGUIDlg::RTXOn()
{
	LPCTSTR RTSS_RUNNER = "..\\RAINBOW___Win32_RTSS_Release\\RAINBOW.rtss";
	PROCESS_INFORMATION pi;
	
	// initialize SharedMemory
	hSharedMemory = RtCreateSharedMemory(
											PAGE_READWRITE,				// access mode
											0,							// maximum size high
											sizeof(SHARED_DATA),		// maximum size low	
											"Can Shared Data",			// name of shared memory
											(VOID **)&pSharedMemory);	// shared memory data address 
	
	if (hSharedMemory == NULL) return false;

	if ( !RtCreateProcess(
					RTSS_RUNNER,	// path of RTX program
					NULL,			// command line to execute
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					&pi ))			// process information
	{
		RtWprintf(L"RtCreateProcess error = %d\n", GetLastError());
		return false;
	}
	else RtCloseHandle(pi.hProcess);
	
	Sleep(500);  // very important !!!
	return true;
}

bool CRainbowGUIDlg::RTXOff()
{
	pSharedMemory->CommandFlag = DISABLE_FET;
	Sleep(500);
	pSharedMemory->CommandFlag = EXIT_PROGRAM;
	RtCloseHandle(hSharedMemory);

	return true;
}

void CRainbowGUIDlg::OnLoadparam() 
{
	// TODO: Add your control notification handler code here
	if ( pSharedMemory->CommandFlag != NO_ACT) AfxMessageBox("Other Command is activated..!!");
	else 
	{
		// device check
		pSharedMemory->CommandFlag = CHECK_DEVICE;
		//GetDlgItem(IDC_CONTROLONOFF)->EnableWindow(TRUE);

		Sleep(1000);
	}

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = LOAD_PARAMETER;
		GetDlgItem(IDC_CHECKDEVICE)->EnableWindow(TRUE);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CRainbowGUIDlg::OnCheckdevice() 
{
	// TODO: Add your control notification handler code here
	if ( pSharedMemory->CommandFlag != NO_ACT) AfxMessageBox("Other Command is activated..!!");
	else 
	{
		// device check
		//pSharedMemory->CommandFlag = CHECK_DEVICE;
		GetDlgItem(IDC_CONTROLONOFF)->EnableWindow(TRUE);

//		Sleep(500);
		pSharedMemory->CommandFlag = ENABLE_FET;
	}
}

void CRainbowGUIDlg::OnControlonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	GetDlgItem(IDC_CONTROLONOFF)->GetWindowText(strText);

	if(strText == "CTRL. ON")
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->CommandFlag = RUN_CMD;
			GetDlgItem(IDC_CONTROLONOFF)->SetWindowText("CTRL. OFF");

			if(m_propertySheet.GetPageCount() <= 0)
			{
				// create property sheet and add property page
				m_propertySheet.AddPage(&m_initHUBO2Dlg);	//0
				m_propertySheet.AddPage(&m_walkingDlg);		//1
				m_propertySheet.AddPage(&m_setSensorDlg);	//2
				m_propertySheet.AddPage(&m_presetDlg);		//3
				//m_propertySheet.AddPage(&m_setJointDlg);
				m_propertySheet.AddPage(&m_jmcSettingDlg);	//4
				m_propertySheet.AddPage(&m_lStatusDlg);		//5
				m_propertySheet.AddPage(&m_uStatusDlg);		//6
				m_propertySheet.AddPage(&m_userDlg);		//7
				m_propertySheet.Create(this, WS_CHILD | WS_VISIBLE, 0);
				m_propertySheet.ModifyStyleEx (0, WS_EX_CONTROLPARENT);
				m_propertySheet.ModifyStyle(0, WS_TABSTOP);
				CRect rcSheet;
				GetDlgItem( IDC_PROPERTYSHEET )->GetWindowRect( &rcSheet );
				ScreenToClient( &rcSheet );
				m_propertySheet.SetWindowPos( NULL, rcSheet.left-7, rcSheet.top-7, 0, 0, SWP_NOZORDER | SWP_NOSIZE | SWP_NOACTIVATE );

				// set timer for display
				SetTimer(0, 100, NULL);
			}
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
	else
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->CommandFlag = STOP_CMD;
			GetDlgItem(IDC_CONTROLONOFF)->SetWindowText("CTRL. ON");
			
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
}

void CRainbowGUIDlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	
	int index;
	CString strText;
	switch(nIDEvent)
	{
	case 0:
		if(RTXStarted == true)
		{
			index = m_propertySheet.GetActiveIndex();
			switch(index)
			{
			case 0:	// initHUBO2 page
				strText.Format("%f", pSharedMemory->Joint_debug[RHY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RHY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RHR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RHR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RHP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RHP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RKN].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RKN)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RAP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RAP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RAR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RAR)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[LHY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LHY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LHR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LHR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LHP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LHP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LKN].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LKN)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LAP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LAP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LAR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LAR)->SetWindowText(strText);
				
				strText.Format("%f", pSharedMemory->Joint_debug[RSP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RSP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RSR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RSR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RSY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RSY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[REB].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_REB)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RWY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RWY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RWP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RWP)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[LSP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LSP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LSR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LSR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LSY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LSY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LEB].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LEB)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LWY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LWY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LWP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LWP)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[NKY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_NKY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[NK1].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_NK1)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[NK2].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_NK2)->SetWindowText(strText);
				
				strText.Format("%f", pSharedMemory->Joint_debug[WST].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_WST)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[RF1].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF1)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF2].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF2)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF3].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF3)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF4].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF4)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF5].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF5)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[LF1].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF1)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF2].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF2)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF3].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF3)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF4].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF4)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF5].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF5)->SetWindowText(strText);
				break;
			case 1:	// walking page
				break;
			case 2:	// sensor page
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_RMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].My);			m_setSensorDlg.GetDlgItem(IDC_RMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_RFZ)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Filtered_Mx);			m_setSensorDlg.GetDlgItem(IDC_RMX)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Filtered_My);			m_setSensorDlg.GetDlgItem(IDC_RMY)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Filtered_Fz);			m_setSensorDlg.GetDlgItem(IDC_RFZ)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].VelRoll);	m_setSensorDlg.GetDlgItem(IDC_RROLL)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].VelPitch);	m_setSensorDlg.GetDlgItem(IDC_RPITCH)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_LMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].My);			m_setSensorDlg.GetDlgItem(IDC_LMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_LFZ)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Filtered_Mx);			m_setSensorDlg.GetDlgItem(IDC_LMX)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Filtered_My);			m_setSensorDlg.GetDlgItem(IDC_LMY)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Filtered_Fz);			m_setSensorDlg.GetDlgItem(IDC_LFZ)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].VelRoll);	m_setSensorDlg.GetDlgItem(IDC_LROLL)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].VelPitch);	m_setSensorDlg.GetDlgItem(IDC_LPITCH)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->FT_debug[RWFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_RWMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RWFT].My);			m_setSensorDlg.GetDlgItem(IDC_RWMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RWFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_RWFZ)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->FT_debug[LWFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_LWMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LWFT].My);			m_setSensorDlg.GetDlgItem(IDC_LWMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LWFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_LWFZ)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Roll);				m_setSensorDlg.GetDlgItem(IDC_ROLL)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Pitch);			m_setSensorDlg.GetDlgItem(IDC_PITCH)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Roll_Velocity);	m_setSensorDlg.GetDlgItem(IDC_ROLLV)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Pitch_Velocity);	m_setSensorDlg.GetDlgItem(IDC_PITCHV)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->ZMP[0]);	m_setSensorDlg.GetDlgItem(IDC_ZMPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[1]);	m_setSensorDlg.GetDlgItem(IDC_ZMPY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[2]);	m_setSensorDlg.GetDlgItem(IDC_RZMPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[3]);	m_setSensorDlg.GetDlgItem(IDC_RZMPY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[4]);	m_setSensorDlg.GetDlgItem(IDC_LZMPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[5]);	m_setSensorDlg.GetDlgItem(IDC_LZMPY)->SetWindowText(strText);
				break;
			case 3:	// preset position
				break;
			case 4:	// JMC page
				break;
			case 5:	// LStatus page
				// Home status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RKN)->SetWindowText(strText);
				//strText.Format("%d", pSharedMemory->Joint_debug[RHY].LimitOnOff);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHP)->SetWindowText(strText);
				//strText.Format("%d", pSharedMemory->Joint_debug[RHR].LimitOnOff);	m_lStatusDlg.GetDlgItem(IDC_HOME_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LAR)->SetWindowText(strText);

				// JAM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LAR)->SetWindowText(strText);

				// PWM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LAR)->SetWindowText(strText);

				// Big error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LAR)->SetWindowText(strText);

				// Encoder error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LAR)->SetWindowText(strText);

				// FET driver error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LAR)->SetWindowText(strText);

				// Motor0 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LAR)->SetWindowText(strText);

				// Motor1 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LAR)->SetWindowText(strText);

				// Upper limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LAR)->SetWindowText(strText);

				// Lower limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LAR)->SetWindowText(strText);
				break;
			case 6:	// UStatus page
				// Home status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LWP)->SetWindowText(strText);

				// JAM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LWP)->SetWindowText(strText);

				// PWM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LWP)->SetWindowText(strText);

				// Big error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LWP)->SetWindowText(strText);

				// Encoder error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LWP)->SetWindowText(strText);

				// FET driver error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LWP)->SetWindowText(strText);

				// Motor0 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LWP)->SetWindowText(strText);

				// Motor1 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LWP)->SetWindowText(strText);

				// Upper limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LWP)->SetWindowText(strText);

				// Lower limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LWP)->SetWindowText(strText);
				break;
			case 7:	// user page
				break;
			default:
				break;
			}
		}
		break;
	}

	CDialog::OnTimer(nIDEvent);
}
