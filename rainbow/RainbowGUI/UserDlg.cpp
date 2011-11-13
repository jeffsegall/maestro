// UserDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "UserDlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"

// SharedMemory variable
extern PSHARED_DATA pSharedMemory;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CUserDlg property page

IMPLEMENT_DYNCREATE(CUserDlg, CPropertyPage)

CUserDlg::CUserDlg() : CPropertyPage(CUserDlg::IDD)
{
	//{{AFX_DATA_INIT(CUserDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}

CUserDlg::~CUserDlg()
{
}

void CUserDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CUserDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CUserDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CUserDlg)
	ON_BN_CLICKED(IDC_TEST1, OnTest1)
	ON_BN_CLICKED(IDC_TEST2, OnTest2)
	ON_BN_CLICKED(IDC_TEST3, OnTest3)
	ON_BN_CLICKED(IDC_TEST4, OnTest4)
	ON_BN_CLICKED(IDC_TEST5, OnTest5)
	ON_BN_CLICKED(IDC_BUTTON1, OnButton1)
	ON_BN_CLICKED(IDC_ROLL_DAMPING_SET, OnRollDampingSet)
	ON_BN_CLICKED(IDC_PITCH_DAMPING_SET, OnPitchDampingSet)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CUserDlg message handlers

void CUserDlg::OnTest1() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 1;
		pSharedMemory->CommandFlag = BEEP;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CUserDlg::OnTest2() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 2;
		pSharedMemory->CommandFlag = BEEP;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CUserDlg::OnTest3() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 3;
		pSharedMemory->CommandFlag = BEEP;
	}
	else AfxMessageBox("Other Command is activated..!!");	
}

void CUserDlg::OnTest4() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 4;
		pSharedMemory->CommandFlag = BEEP;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CUserDlg::OnTest5() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0;
		pSharedMemory->CommandFlag = BEEP;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CUserDlg::OnButton1() 
{
	// TODO: Add your control notification handler code here
	unsigned char i;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		for(i=0 ; i<=LWP ; i++)
		{
			pSharedMemory->JointID = i;
			pSharedMemory->CommandFlag = PRINT_JOINT_PARAMETER;
			Sleep(200);
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CUserDlg::OnRollDampingSet() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_ROLL_OVERALL)->GetWindowText(strText);
		pSharedMemory->CommandData = 0x01;
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
		GetDlgItem(IDC_ROLL_P)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
		GetDlgItem(IDC_ROLL_V)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[2] = (float)atof(strText);

		pSharedMemory->CommandFlag = SET_DAMPING_GAIN;
		
	}
	else AfxMessageBox("Other Command is activated..!!"); 
}

void CUserDlg::OnPitchDampingSet() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_PITCH_OVERALL)->GetWindowText(strText);
		pSharedMemory->CommandData = 0x01;
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
		GetDlgItem(IDC_PITCH_P)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
		GetDlgItem(IDC_PITCH_V)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[2] = (float)atof(strText);
		
		pSharedMemory->CommandFlag = SET_DAMPING_GAIN;
		
	}
	else AfxMessageBox("Other Command is activated..!!"); 
}

BOOL CUserDlg::OnInitDialog() 
{
	CPropertyPage::OnInitDialog();
	
	// TODO: Add extra initialization here
	
	GetDlgItem(IDC_PITCH_OVERALL)->SetWindowText("0.3");
	GetDlgItem(IDC_PITCH_P)->SetWindowText("1.0");
	GetDlgItem(IDC_PITCH_V)->SetWindowText("0.4");

	GetDlgItem(IDC_ROLL_OVERALL)->SetWindowText("0.3");
	GetDlgItem(IDC_ROLL_P)->SetWindowText("1.0");
	GetDlgItem(IDC_ROLL_V)->SetWindowText("0.5");

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}
