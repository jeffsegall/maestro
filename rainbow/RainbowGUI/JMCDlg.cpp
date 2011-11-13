// JMCDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "JMCDlg.h"
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
// CJMCDlg property page

IMPLEMENT_DYNCREATE(CJMCDlg, CPropertyPage)

CJMCDlg::CJMCDlg() : CPropertyPage(CJMCDlg::IDD)
{
	//{{AFX_DATA_INIT(CJMCDlg)
	//}}AFX_DATA_INIT
}

CJMCDlg::~CJMCDlg()
{
}

void CJMCDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CJMCDlg)
	DDX_Control(pDX, IDC_JOINTSEL, m_jointSelBox);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CJMCDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CJMCDlg)
	ON_CBN_SELCHANGE(IDC_JOINTSEL, OnSelchangeJointsel)
	ON_BN_CLICKED(IDC_SETENCODER, OnSetencoder)
	ON_BN_CLICKED(IDC_SETHOMEVELACC, OnSethomevelacc)
	ON_BN_CLICKED(IDC_SETMODE, OnSetmode)
	ON_BN_CLICKED(IDC_SETGAIN, OnSetgain)
	ON_BN_CLICKED(IDC_SETPOSLIMIT, OnSetposlimit)
	ON_BN_CLICKED(IDC_SETMAXVELACC, OnSetmaxvelacc)
	ON_BN_CLICKED(IDC_SETSATURATION, OnSetsaturation)
	ON_BN_CLICKED(IDC_SETERRBOUND, OnSeterrbound)
	ON_BN_CLICKED(IDC_SETHOMEPARA, OnSethomepara)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CJMCDlg message handlers

BOOL CJMCDlg::OnInitDialog() 
{
	CPropertyPage::OnInitDialog();
	
	// TODO: Add extra initialization here
	
	// Joint combobox list setting
	m_jointSelBox.InsertString(0, "RHY");	m_jointSelBox.InsertString(1, "RHR");	m_jointSelBox.InsertString(2, "RHP");
	m_jointSelBox.InsertString(3, "RKN");	m_jointSelBox.InsertString(4, "RAP");	m_jointSelBox.InsertString(5, "RAR");
	m_jointSelBox.InsertString(6, "LHY");	m_jointSelBox.InsertString(7, "LHR");	m_jointSelBox.InsertString(8, "LHP");
	m_jointSelBox.InsertString(9, "LKN");	m_jointSelBox.InsertString(10, "LAP");	m_jointSelBox.InsertString(11, "LAR"); 

	m_jointSelBox.InsertString(12, "RSP");	m_jointSelBox.InsertString(13, "RSR");	m_jointSelBox.InsertString(14, "RSY");
	m_jointSelBox.InsertString(15, "REB");	m_jointSelBox.InsertString(16, "RWY");	m_jointSelBox.InsertString(17, "RWP");
	m_jointSelBox.InsertString(18, "LSP");	m_jointSelBox.InsertString(19, "LSR");	m_jointSelBox.InsertString(20, "LSY");
	m_jointSelBox.InsertString(21, "LEB");	m_jointSelBox.InsertString(22, "LWY");	m_jointSelBox.InsertString(23, "LWP"); 

	m_jointSelBox.InsertString(24, "NKY");	m_jointSelBox.InsertString(25, "NK1");	m_jointSelBox.InsertString(26, "NK2"); 
	m_jointSelBox.InsertString(27, "WST"); 

	m_jointSelBox.InsertString(28, "RF1");	m_jointSelBox.InsertString(29, "RF2");	m_jointSelBox.InsertString(30, "RF3"); 
	m_jointSelBox.InsertString(31, "RF4");	m_jointSelBox.InsertString(32, "RF5");	m_jointSelBox.InsertString(33, "LF1"); 
	m_jointSelBox.InsertString(34, "LF2");	m_jointSelBox.InsertString(35, "LF3");	m_jointSelBox.InsertString(36, "LF4"); 
	m_jointSelBox.InsertString(37, "LF5");	

	m_jointSelBox.SetCurSel(0);
	//OnSelchangeJointsel();

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CJMCDlg::OnSelchangeJointsel() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	unsigned char joint = m_jointSelBox.GetCurSel();

	// position or torque control gains
	if(pSharedMemory->Joint_debug[joint].MotorControlMode == 0x00)
	{
		strText.Format("%d", pSharedMemory->Joint_debug[joint].Position_Kp);
		GetDlgItem(IDC_PGAIN)->SetWindowText(strText);
		strText.Format("%d", pSharedMemory->Joint_debug[joint].Position_Ki);
		GetDlgItem(IDC_IGAIN)->SetWindowText(strText);
		strText.Format("%d", pSharedMemory->Joint_debug[joint].Position_Kd);
		GetDlgItem(IDC_DGAIN)->SetWindowText(strText);
	}
	else
	{
		strText.Format("%d", pSharedMemory->Joint_debug[joint].Torque_Kp);
		GetDlgItem(IDC_PGAIN)->SetWindowText(strText);
		strText.Format("%d", pSharedMemory->Joint_debug[joint].Torque_Ki);
		GetDlgItem(IDC_IGAIN)->SetWindowText(strText);
		strText.Format("%d", pSharedMemory->Joint_debug[joint].Torque_Kd);
		GetDlgItem(IDC_DGAIN)->SetWindowText(strText);
	}
	// Encoder resolution
	strText.Format("%d", pSharedMemory->Joint_debug[joint].Encoder_size);
	GetDlgItem(IDC_ENCRESOLUTION)->SetWindowText(strText);
	// Motor positive direction
	strText.Format("%d", pSharedMemory->Joint_debug[joint].Positive_dir);
	GetDlgItem(IDC_MOTORDIR)->SetWindowText(strText);
	// Motor deadzone
	strText.Format("%d", pSharedMemory->Joint_debug[joint].Deadzone);
	GetDlgItem(IDC_DEADZONE)->SetWindowText(strText);
	// limit search direction
	strText.Format("%d", pSharedMemory->Joint_debug[joint].SearchDirection);
	GetDlgItem(IDC_HOMESEARCHDIR)->SetWindowText(strText);
	// home search mode
	strText.Format("%d", pSharedMemory->Joint_debug[joint].HomeSearchMode);
	GetDlgItem(IDC_HOMESEARCHMODE)->SetWindowText(strText);
	// limit rotation
	strText.Format("%d", pSharedMemory->Joint_debug[joint].Limit_rev);
	GetDlgItem(IDC_HOMELIMITREV)->SetWindowText(strText);
	// home offset angle
	strText.Format("%f", pSharedMemory->Joint_debug[joint].Offset_angle);
	GetDlgItem(IDC_HOMEOFFSET)->SetWindowText(strText);
	// Lower position limit
	strText.Format("%f", pSharedMemory->Joint_debug[joint].LowerPositionLimit);
	GetDlgItem(IDC_LOWERLIMIT)->SetWindowText(strText);
	// Upper position limit
	strText.Format("%f", pSharedMemory->Joint_debug[joint].UpperPositionLimit);
	GetDlgItem(IDC_UPPERLIMIT)->SetWindowText(strText);
	// Max acc
	strText.Format("%d", pSharedMemory->Joint_debug[joint].MaxAcc);
	GetDlgItem(IDC_MAXACC)->SetWindowText(strText);
	// Max vel
	strText.Format("%d", pSharedMemory->Joint_debug[joint].MaxVel);
	GetDlgItem(IDC_MAXVEL)->SetWindowText(strText);
	// Max PWM
	strText.Format("%d", pSharedMemory->Joint_debug[joint].MaxPWM);
	GetDlgItem(IDC_HOMEPWM)->SetWindowText(strText);
	// Max acc home
	strText.Format("%d", pSharedMemory->Joint_debug[joint].MaxAccHome);
	GetDlgItem(IDC_HOMEMAXACC)->SetWindowText(strText);
	// Max vel home
	strText.Format("%d", pSharedMemory->Joint_debug[joint].MaxVelHome);
	GetDlgItem(IDC_HOMEMAXVEL)->SetWindowText(strText);
	// JAM time
	strText.Format("%d", pSharedMemory->Joint_debug[joint].JAMmsTime);
	GetDlgItem(IDC_JAMSATTIME)->SetWindowText(strText);
	// PWM time
	strText.Format("%d", pSharedMemory->Joint_debug[joint].PWMmsTime);
	GetDlgItem(IDC_PWMSATTIME)->SetWindowText(strText);
	// JAM pwm duty
	strText.Format("%d", pSharedMemory->Joint_debug[joint].PWMDuty);
	GetDlgItem(IDC_JAMDUTY)->SetWindowText(strText);
	// PWM pwm duty
	strText.Format("%d", pSharedMemory->Joint_debug[joint].PWMDuty);
	GetDlgItem(IDC_PWMDUTY)->SetWindowText(strText);
	// I Error
	strText.Format("%d", pSharedMemory->Joint_debug[joint].I_ERR);
	GetDlgItem(IDC_IERR)->SetWindowText(strText);
	// B Error
	strText.Format("%d", pSharedMemory->Joint_debug[joint].B_ERR);
	GetDlgItem(IDC_BERR)->SetWindowText(strText);
	// E Error
	strText.Format("%d", pSharedMemory->Joint_debug[joint].E_ERR);
	GetDlgItem(IDC_EERR)->SetWindowText(strText);
}

unsigned char CJMCDlg::GetBoardIDFromJointID(unsigned char _jointID)
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

unsigned char CJMCDlg::GetMotorChannelFromJointID(unsigned char _jointID)
{
	switch(_jointID)
	{
	case RHY:	return 0x00;
	case RHR:	return 0x01;
	case RHP:	return 0x00;
	case RKN:	return 0x00;
	case RAP:	return 0x00;
	case RAR:	return 0x01;
	case LHY:	return 0x00;
	case LHR:	return 0x01;
	case LHP:	return 0x00;
	case LKN:	return 0x00;
	case LAP:	return 0x00;
	case LAR:	return 0x01;
	case RSP:	return 0x00;
	case RSR:	return 0x01;
	case RSY:	return 0x00;
	case REB:	return 0x01;
	case LSP:	return 0x00;
	case LSR:	return 0x01;
	case LSY:	return 0x00;
	case LEB:	return 0x01;
	case RWY:	return 0x00;
	case RWP:	return 0x01;
	case LWY:	return 0x00;
	case LWP:	return 0x01;
	case NKY:	return 0x00;
	case NK1:	return 0x01;
	case NK2:	return 0x02;
	case WST:	return 0x00;
	case RF1:	return 0x00;
	case RF2:	return 0x01;
	case RF3:	return 0x02;
	case RF4:	return 0x03;
	case RF5:	return 0x04;
	case LF1:	return 0x00;
	case LF2:	return 0x01;
	case LF3:	return 0x02;
	case LF4:	return 0x03;
	case LF5:	return 0x04;
	}

	return 0xFF;
}

void CJMCDlg::OnSetencoder() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		GetDlgItem(IDC_DEADZONE)->GetWindowText(strText);
		pSharedMemory->CommandData = (unsigned int)(atoi(strText));
		pSharedMemory->CommandFlag = SET_DEADZONE;
	}
	else AfxMessageBox("Other Command is activated..!!");

	Sleep(500);

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		GetDlgItem(IDC_ENCRESOLUTION)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[0] = (unsigned int)(atoi(strText));

		GetDlgItem(IDC_MOTORDIR)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[1] = (unsigned int)(atoi(strText));
		
		pSharedMemory->CommandFlag = SET_ENCODER_RESOLUTION;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSethomevelacc() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		
		GetDlgItem(IDC_HOMEMAXVEL)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[0] = (unsigned int)(atoi(strText));
		GetDlgItem(IDC_HOMEMAXACC)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[1] = (unsigned int)(atoi(strText));
		GetDlgItem(IDC_HOMESEARCHMODE)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[2] = (unsigned int)(atoi(strText));
		GetDlgItem(IDC_HOMEPWM)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[3] = (unsigned int)(atoi(strText));

		pSharedMemory->CommandFlag = SET_ENCODER_RESOLUTION;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSetmode() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();

		GetDlgItem(IDC_SETMODE)->GetWindowText(strText);
		if(strText == "To C-Mode")
		{
			pSharedMemory->CommandData = 0x01;;
			pSharedMemory->CommandFlag = SET_CONTROL_MODE;

			GetDlgItem(IDC_SETMODE)->SetWindowText("To P-Mode");
		}
		else
		{
			pSharedMemory->CommandData = 0x00;;
			pSharedMemory->CommandFlag = SET_CONTROL_MODE;

			GetDlgItem(IDC_SETMODE)->SetWindowText("To C-Mode");
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSetgain() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();	

		GetDlgItem(IDC_PGAIN)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[0] = atoi(strText);
		GetDlgItem(IDC_IGAIN)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[1] = atoi(strText);
		GetDlgItem(IDC_DGAIN)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[3] = atoi(strText);
		
		pSharedMemory->CommandFlag = SET_MOTOR_GAIN;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSetposlimit() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();	

		GetDlgItem(IDC_LOWERLIMIT)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
		GetDlgItem(IDC_UPPERLIMIT)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
		
		pSharedMemory->CommandFlag = SET_POSITION_LIMIT;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSetmaxvelacc() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();	

		GetDlgItem(IDC_MAXVEL)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[0] = (unsigned int)atoi(strText);
		GetDlgItem(IDC_MAXACC)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[1] = (unsigned int)atoi(strText);
		
		pSharedMemory->CommandFlag = SET_MAX_VEL_ACC;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSetsaturation() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();	

		GetDlgItem(IDC_JAMSATTIME)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[0] = (unsigned int)atoi(strText);
		GetDlgItem(IDC_PWMSATTIME)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[1] = (unsigned int)atoi(strText);
		GetDlgItem(IDC_JAMDUTY)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[2] = (unsigned int)atoi(strText);
		GetDlgItem(IDC_PWMDUTY)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[3] = (unsigned int)atoi(strText);
		
		pSharedMemory->CommandFlag = SET_JAMPWM_FAULT;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSeterrbound() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();	

		GetDlgItem(IDC_IERR)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[0] = (unsigned int)atoi(strText);
		GetDlgItem(IDC_BERR)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[1] = (unsigned int)atoi(strText);
		GetDlgItem(IDC_EERR)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[2] = (unsigned int)atoi(strText);
		
		pSharedMemory->CommandFlag = SET_JAMPWM_FAULT;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJMCDlg::OnSethomepara() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		
		GetDlgItem(IDC_HOMELIMITREV)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[0] = (unsigned int)(atoi(strText));
		GetDlgItem(IDC_HOMESEARCHDIR)->GetWindowText(strText);
		pSharedMemory->CommandDataArray[1] = (unsigned int)(atoi(strText));
		GetDlgItem(IDC_HOMEOFFSET)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[0] = (float)(atof(strText));

		pSharedMemory->CommandFlag = SET_HOME_SEARCH_PARAMETER;
	}
	else AfxMessageBox("Other Command is activated..!!");	
}
