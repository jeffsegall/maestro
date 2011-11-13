// InitHUBO2Dlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "InitHUBO2Dlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"


#define OFFSET_TIME	3000.0f

// SharedMemory variable
extern PSHARED_DATA pSharedMemory;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CInitHUBO2Dlg property page

IMPLEMENT_DYNCREATE(CInitHUBO2Dlg, CPropertyPage)

CInitHUBO2Dlg::CInitHUBO2Dlg() : CPropertyPage(CInitHUBO2Dlg::IDD)
{
	//{{AFX_DATA_INIT(CInitHUBO2Dlg)
	m_offsetPos = 0.0f;
	//}}AFX_DATA_INIT
}

CInitHUBO2Dlg::~CInitHUBO2Dlg()
{
}

void CInitHUBO2Dlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CInitHUBO2Dlg)
	DDX_Control(pDX, IDC_JOINTSEL, m_jointSelBox);
	DDX_Text(pDX, IDC_OFFSETPOS, m_offsetPos);
	DDV_MinMaxFloat(pDX, m_offsetPos, -90.f, 90.f);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CInitHUBO2Dlg, CPropertyPage)
	//{{AFX_MSG_MAP(CInitHUBO2Dlg)
	ON_BN_CLICKED(IDC_GOINITPOS, OnGoinitpos)
	ON_BN_CLICKED(IDC_ENCODERZERO, OnEncoderzero)
	ON_BN_CLICKED(IDC_GOOFFSETPOS, OnGooffsetpos)
	ON_BN_CLICKED(IDC_SAVEOFFSET, OnSaveoffset)
	ON_BN_CLICKED(IDC_ALLINITPOS, OnAllinitpos)
	ON_BN_CLICKED(IDC_LOWERINITPOS, OnLowerinitpos)
	ON_BN_CLICKED(IDC_UPPERINITPOS, OnUpperinitpos)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CInitHUBO2Dlg message handlers

BOOL CInitHUBO2Dlg::OnInitDialog() 
{
	CPropertyPage::OnInitDialog();
	
	// TODO: Add extra initialization here

	// Joint combobox list setting in the "Set Joints"
	m_jointSelBox.InsertString(0, "RHY");	m_jointSelBox.InsertString(1, "RHR");
	m_jointSelBox.InsertString(2, "RHP");	m_jointSelBox.InsertString(3, "RKN");
	m_jointSelBox.InsertString(4, "RAP");	m_jointSelBox.InsertString(5, "RAR");
	m_jointSelBox.InsertString(6, "LHY");	m_jointSelBox.InsertString(7, "LHR");
	m_jointSelBox.InsertString(8, "LHP");	m_jointSelBox.InsertString(9, "LKN");
	m_jointSelBox.InsertString(10, "LAP");	m_jointSelBox.InsertString(11, "LAR");
	m_jointSelBox.InsertString(12, "RSP");	m_jointSelBox.InsertString(13, "RSR");
	m_jointSelBox.InsertString(14, "RSY");	m_jointSelBox.InsertString(15, "REB");
	m_jointSelBox.InsertString(16, "RWY");	m_jointSelBox.InsertString(17, "RWP");
	m_jointSelBox.InsertString(18, "LSP");	m_jointSelBox.InsertString(19, "LSR");
	m_jointSelBox.InsertString(20, "LSY");	m_jointSelBox.InsertString(21, "LEB");
	m_jointSelBox.InsertString(22, "LWY");	m_jointSelBox.InsertString(23, "LWP");
	m_jointSelBox.InsertString(24, "NKY");	m_jointSelBox.InsertString(25, "NK1");
	m_jointSelBox.InsertString(26, "NK2");	m_jointSelBox.InsertString(27, "WST");
	m_jointSelBox.InsertString(28, "RF1");	m_jointSelBox.InsertString(29, "RF2");
	m_jointSelBox.InsertString(30, "RF3");	m_jointSelBox.InsertString(31, "RF4");
	m_jointSelBox.InsertString(32, "RF5");	m_jointSelBox.InsertString(33, "LF1");
	m_jointSelBox.InsertString(34, "LF2");	m_jointSelBox.InsertString(35, "LF3");
	m_jointSelBox.InsertString(36, "LF4");	m_jointSelBox.InsertString(37, "LF5");
	m_jointSelBox.SetCurSel(0);

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CInitHUBO2Dlg::OnGoinitpos() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->MotorControlMode = CTRLMODE_NONE;
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CInitHUBO2Dlg::OnEncoderzero() 
{
	// TODO: Add your control notification handler code here
	unsigned char jointID = m_jointSelBox.GetCurSel();
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->MotorControlMode = CTRLMODE_NONE;
		pSharedMemory->BoardID = GetBoardIDFromJointID(jointID);
		pSharedMemory->CommandFlag = ENCODER_ZERO_EACH;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

unsigned char CInitHUBO2Dlg::GetBoardIDFromJointID(unsigned char _jointID)
{
	switch(_jointID)
	{
	case RHY:
	case RHR:
		return JMC0;
	case RHP:
		return JMC1;
	case RKN:
		return JMC2;
	case RAP:
	case RAR:
		return JMC3;
	case LHY:
	case LHR:
		return JMC4;
	case LHP:
		return JMC5;
	case LKN:
		return JMC6;
	case LAP:
	case LAR:
		return JMC7;
	case RSP:
	case RSR:
		return JMC8;
	case RSY:
	case REB:
		return JMC9;
	case LSP:
	case LSR:
		return JMC10;
	case LSY:
	case LEB:
		return JMC11;
	case RWY:
	case RWP:
		return EJMC0;
	case LWY:
	case LWP:
		return EJMC1;
	case NKY:
	case NK1:
	case NK2:
		return EJMC2;
	case WST:
		return EJMC3;
	case RF1:
	case RF2:
	case RF3:
	case RF4:
	case RF5:
		return EJMC4;
	case LF1:
	case LF2:
	case LF3:
	case LF4:
	case LF5:
		return EJMC5;
	}

	return 0xFF;

}

void CInitHUBO2Dlg::OnGooffsetpos() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		pSharedMemory->CommandData = 0x00;
		pSharedMemory->CommandFlag = POSITION_LIMIT_ONOFF;

		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		pSharedMemory->GoalAngle = m_offsetPos;
		pSharedMemory->GoalTime = OFFSET_TIME;
		pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;

		Sleep(10);
		pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CInitHUBO2Dlg::OnSaveoffset() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch(m_jointSelBox.GetCurSel())
		{
		case RHY:
			pSharedMemory->JointID = RHY;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RHY)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RHR:
			pSharedMemory->JointID = RHR;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RHR)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RHP:
			pSharedMemory->JointID = RHP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RHP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RKN:
			pSharedMemory->JointID = RKN;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RKN)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RAP:
			pSharedMemory->JointID = RAP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RAP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RAR:
			pSharedMemory->JointID = RAR;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RAR)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LHY:
			pSharedMemory->JointID = LHY;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LHY)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LHR:
			pSharedMemory->JointID = LHR;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LHR)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LHP:
			pSharedMemory->JointID = LHP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LHP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LKN:
			pSharedMemory->JointID = LKN;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LKN)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LAP:
			pSharedMemory->JointID = LAP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LAP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LAR:
			pSharedMemory->JointID = LAR;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LAR)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RSP:
			pSharedMemory->JointID = RSP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RSP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RSR:
			pSharedMemory->JointID = RSR;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RSR)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RSY:
			pSharedMemory->JointID = RSY;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RSY)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case REB:
			pSharedMemory->JointID = REB;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_REB)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RWY:
			pSharedMemory->JointID = RWY;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RAP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case RWP:
			pSharedMemory->JointID = RWP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_RAR)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LSP:
			pSharedMemory->JointID = LSP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LSP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LSR:
			pSharedMemory->JointID = LSR;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LSR)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LSY:
			pSharedMemory->JointID = LSY;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LSY)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LEB:
			pSharedMemory->JointID = LEB;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LEB)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LWY:
			pSharedMemory->JointID = LWY;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LWY)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case LWP:
			pSharedMemory->JointID = LWP;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_LWP)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case WST:
			pSharedMemory->JointID = WST;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_WST)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case NKY:
			pSharedMemory->JointID = NKY;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_NKY)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case NK1:
			pSharedMemory->JointID = NK1;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_NK1)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		case NK2:
			pSharedMemory->JointID = NK2;
			pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;	Sleep(100);
			GetDlgItem(IDC_NK2)->GetWindowText(strText);
			pSharedMemory->CommandDataArray[0] = pSharedMemory->Joint.Limit_rev;	
			pSharedMemory->CommandDataArray[1] = pSharedMemory->Joint.SearchDirection;
			pSharedMemory->CommandDataFloat[0] = pSharedMemory->Joint.Offset_angle += (float)atof(strText);
			pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;	Sleep(100);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CInitHUBO2Dlg::OnAllinitpos() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS_ALL;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CInitHUBO2Dlg::OnLowerinitpos() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS_LOWER_ALL;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CInitHUBO2Dlg::OnUpperinitpos() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS_UPPER_ALL;
	}
	else AfxMessageBox("Other Command is activated..!!");
}
