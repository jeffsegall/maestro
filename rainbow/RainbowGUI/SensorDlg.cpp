// SensorDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "SensorDlg.h"
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
// CSensorDlg property page

IMPLEMENT_DYNCREATE(CSensorDlg, CPropertyPage)

CSensorDlg::CSensorDlg() : CPropertyPage(CSensorDlg::IDD)
{
	//{{AFX_DATA_INIT(CSensorDlg)
	//}}AFX_DATA_INIT
}

CSensorDlg::~CSensorDlg()
{
}

void CSensorDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CSensorDlg)
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CSensorDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CSensorDlg)
	ON_BN_CLICKED(IDC_FTNULLING, OnFtnulling)
	ON_BN_CLICKED(IDC_FOOTANGLENULLING, OnFootanglenulling)
	ON_BN_CLICKED(IDC_IMUNULLING, OnImunulling)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CSensorDlg message handlers

void CSensorDlg::OnFtnulling() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = NULL_FT_SENSOR;
		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CSensorDlg::OnFootanglenulling() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = NULL_FOOT_ANGLE_SENSOR;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CSensorDlg::OnImunulling() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = NULL_IMU_SENSOR;
		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}
