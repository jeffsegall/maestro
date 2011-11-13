// WalkingDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "WalkingDlg.h"
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
// CWalkingDlg property page

IMPLEMENT_DYNCREATE(CWalkingDlg, CPropertyPage)

CWalkingDlg::CWalkingDlg() : CPropertyPage(CWalkingDlg::IDD)
{
	//{{AFX_DATA_INIT(CWalkingDlg)
	m_swayValue = 0.063f;
	m_stopSway = 1.1f;
	m_stepCount = 0;
	m_startSway = 1.3f;
	m_sideStep = 0.0f;
	m_rotationStep = 0.0f;
	m_stepPeriod = 800;
	m_dspTime = 40;
	m_holdTime = 60;
	m_forwardStep = 0.0f;
	//}}AFX_DATA_INIT
}

CWalkingDlg::~CWalkingDlg()
{
}

void CWalkingDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CWalkingDlg)
	DDX_Control(pDX, IDC_HANDLIST, m_handSelBox);
	DDX_Control(pDX, IDC_MOTIONLIST, m_motionSelBox);
	DDX_Control(pDX, IDC_DEMOLIST, m_demoSelBox);
	DDX_Text(pDX, IDC_SWAY, m_swayValue);
	DDV_MinMaxFloat(pDX, m_swayValue, 4.e-002f, 8.e-002f);
	DDX_Text(pDX, IDC_STOPSWAY, m_stopSway);
	DDV_MinMaxFloat(pDX, m_stopSway, 0.5f, 1.5f);
	DDX_Text(pDX, IDC_STEPCOUNT, m_stepCount);
	DDV_MinMaxInt(pDX, m_stepCount, -1, 100);
	DDX_Text(pDX, IDC_STARTSWAY, m_startSway);
	DDV_MinMaxFloat(pDX, m_startSway, 0.5f, 1.5f);
	DDX_Text(pDX, IDC_SIDE, m_sideStep);
	DDV_MinMaxFloat(pDX, m_sideStep, -0.1f, 0.1f);
	DDX_Text(pDX, IDC_ROTATION, m_rotationStep);
	DDV_MinMaxFloat(pDX, m_rotationStep, -45.f, 45.f);
	DDX_Text(pDX, IDC_PERIOD, m_stepPeriod);
	DDV_MinMaxUInt(pDX, m_stepPeriod, 500, 1200);
	DDX_Text(pDX, IDC_DSP, m_dspTime);
	DDV_MinMaxUInt(pDX, m_dspTime, 0, 100);
	DDX_Text(pDX, IDC_HOLD, m_holdTime);
	DDV_MinMaxUInt(pDX, m_holdTime, 0, 200);
	DDX_Text(pDX, IDC_FORWARD, m_forwardStep);
	DDV_MinMaxFloat(pDX, m_forwardStep, -0.1f, 0.2f);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CWalkingDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CWalkingDlg)
	ON_BN_CLICKED(IDC_ANKLEONOFF, OnAnkleonoff)
	ON_BN_CLICKED(IDC_ZMPINIT, OnZmpinit)
	ON_BN_CLICKED(IDC_WALKREADY, OnWalkready)
	ON_BN_CLICKED(IDC_WALKINGSET, OnWalkingset)
	ON_BN_CLICKED(IDC_GO, OnGo)
	ON_BN_CLICKED(IDC_PLUSROTATION, OnPlusrotation)
	ON_BN_CLICKED(IDC_MINUSROTATION, OnMinusrotation)
	ON_BN_CLICKED(IDC_ZEROROTATION, OnZerorotation)
	ON_BN_CLICKED(IDC_STOP, OnStop)
	ON_BN_CLICKED(IDC_MOTION, OnMotion)
	ON_CBN_SELCHANGE(IDC_DEMOLIST, OnSelchangeDemolist)
	ON_BN_CLICKED(IDC_DEMO1, OnDemo1)
	ON_BN_CLICKED(IDC_DEMO2, OnDemo2)
	ON_BN_CLICKED(IDC_DEMO3, OnDemo3)
	ON_BN_CLICKED(IDC_DEMO4, OnDemo4)
	ON_BN_CLICKED(IDC_DATASAVE, OnDatasave)
	ON_CBN_SELCHANGE(IDC_HANDLIST, OnSelchangeHandlist)
	ON_BN_CLICKED(IDC_HAND1, OnHand1)
	ON_BN_CLICKED(IDC_HAND2, OnHand2)
	ON_BN_CLICKED(IDC_HAND3, OnHand3)
	ON_BN_CLICKED(IDC_HAND4, OnHand4)
	ON_BN_CLICKED(IDC_RANGEONOFF, OnRangeonoff)
	ON_BN_CLICKED(IDC_PLUSCURRENT, OnPluscurrent)
	ON_BN_CLICKED(IDC_MINUSCURRENT, OnMinuscurrent)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CWalkingDlg message handlers

void CWalkingDlg::OnAnkleonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	GetDlgItem(IDC_ANKLEONOFF)->GetWindowText(strText);

	if(strText == "Ankle OFF")
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = STOP_CMD_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = DISABLE_FET_EACH;
			Sleep(20);
	
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = STOP_CMD_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = DISABLE_FET_EACH;
			Sleep(20);

			GetDlgItem(IDC_ANKLEONOFF)->SetWindowText("Ankle ON");
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
	else
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = ENCODER_ZERO_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = ENABLE_FET_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = RUN_CMD_EACH;
			Sleep(20);
	
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = ENCODER_ZERO_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = ENABLE_FET_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = RUN_CMD_EACH;
			Sleep(20);

			GetDlgItem(IDC_ANKLEONOFF)->SetWindowText("Ankle OFF");
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
}

void CWalkingDlg::OnZmpinit() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_ZMPINIT)->GetWindowText(strText);
		if(strText == "ZMP Init. Start") 
		{
			pSharedMemory->CommandFlag = START_ZMP_INITIALIZATION;
			GetDlgItem(IDC_ZMPINIT)->SetWindowText("ZMP Init. Stop");
		}
		else
		{
			pSharedMemory->CommandFlag = STOP_ZMP_INITIALIZATION;
			GetDlgItem(IDC_ZMPINIT)->SetWindowText("ZMP Init. Start");
			//HUBO2Win->pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
		}
		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnWalkready() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_WALKREADY)->GetWindowText(strText);
		if(strText == "Walk Ready")
		{
			pSharedMemory->CommandFlag = GOTO_WALK_READY_POS;
			Sleep(10);
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			GetDlgItem(IDC_ZMPINIT)->EnableWindow(true);
			GetDlgItem(IDC_ANKLEONOFF)->EnableWindow(false);
			GetDlgItem(IDC_GO)->EnableWindow(true);

			GetDlgItem(IDC_WALKREADY)->SetWindowText("Goto Home");
		}
		else
		{
			pSharedMemory->CommandFlag = GOTO_HOME_POS;
			Sleep(10);
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			GetDlgItem(IDC_ZMPINIT)->EnableWindow(false);
			GetDlgItem(IDC_ANKLEONOFF)->EnableWindow(true);
			GetDlgItem(IDC_GO)->EnableWindow(false);

			GetDlgItem(IDC_WALKREADY)->SetWindowText("Walk Ready");
		}	
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnWalkingset() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);

	pSharedMemory->JW_temp[0]	= m_swayValue;
	pSharedMemory->JW_temp[3]	= (float)m_holdTime;
	pSharedMemory->JW_temp[6]	= m_forwardStep/2.0f;
	pSharedMemory->JW_temp[9]	= m_sideStep/2.0f;
	pSharedMemory->JW_temp[12]	= m_rotationStep/2.0f;
	pSharedMemory->JW_temp[15]	= (float)m_dspTime/2.0f;
	pSharedMemory->JW_temp[16]	= (float)m_stepPeriod;
	pSharedMemory->JW_temp[17]	= (float)m_stepCount;
	pSharedMemory->JW_temp[18]	= m_startSway;
	pSharedMemory->JW_temp[19]	= m_stopSway;

	pSharedMemory->JW_temp[1] = 1.0f;
	pSharedMemory->JW_temp[2] = 1.0f;
	pSharedMemory->JW_temp[4] = 1.0f;
	pSharedMemory->JW_temp[5] = 1.0f;
	pSharedMemory->JW_temp[7] = 0.0f;
	pSharedMemory->JW_temp[8] = 0.0f;
	pSharedMemory->JW_temp[10] = 1.0f;
	pSharedMemory->JW_temp[11] = 1.0f;
	
	pSharedMemory->JW_temp[13] = 1.0f;
	pSharedMemory->JW_temp[14] = 1.0f;
	
	pSharedMemory->JW_temp[20] = 60.0f;	// 60.0f;
	pSharedMemory->JW_temp[21] = 1.0f;
	pSharedMemory->JW_temp[22] = 1.0f;
	pSharedMemory->JW_temp[23] = 0.0f;
	pSharedMemory->JW_temp[24] = 0.0f;
	pSharedMemory->JW_temp[25] = 0.0f;
	pSharedMemory->JW_temp[26] = 0.0f;
}

void CWalkingDlg::OnGo() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->Data_Debug_Index = 0;
		pSharedMemory->CommandFlag = GOTO_FORWARD;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnPlusrotation() 
{
	// TODO: Add your control notification handler code here
	
	m_rotationStep = pSharedMemory->JW_temp[12] = 1.0f;
	UpdateData(false);
}

void CWalkingDlg::OnMinusrotation() 
{
	// TODO: Add your control notification handler code here
	m_rotationStep = pSharedMemory->JW_temp[12] = -1.0f;
	UpdateData(false);
}

void CWalkingDlg::OnZerorotation() 
{
	// TODO: Add your control notification handler code here
	m_rotationStep = pSharedMemory->JW_temp[12] = 0.0f;
	UpdateData(false);
}

void CWalkingDlg::OnStop() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = STOP_WALKING;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

BOOL CWalkingDlg::OnInitDialog() 
{
	CPropertyPage::OnInitDialog();
	
	// TODO: Add extra initialization here

	m_motionSelBox.InsertString(0, "Speech1");				m_motionSelBox.InsertString(1, "Speech2");
	m_motionSelBox.InsertString(2, "Sign language");		m_motionSelBox.InsertString(3, "Taichi");
	m_motionSelBox.InsertString(4, "Welcome");				m_motionSelBox.InsertString(5, "Goodbye");
	m_motionSelBox.InsertString(6, "Dance1");				m_motionSelBox.InsertString(7, "Dance2");
	m_motionSelBox.InsertString(8, "Dance3");				m_motionSelBox.InsertString(9, "Dance4");
	m_motionSelBox.InsertString(10, "Bow");					m_motionSelBox.InsertString(11, "Right hand-up");
	m_motionSelBox.InsertString(12, "Right hand-forward");	m_motionSelBox.InsertString(13, "Right hand-head");
	m_motionSelBox.InsertString(14, "Both hand-up");		m_motionSelBox.InsertString(15, "Hand wave");
	m_motionSelBox.InsertString(16, "Left hand-up");		m_motionSelBox.InsertString(17, "Left hand-forward");
	m_motionSelBox.InsertString(18, "Left hand-head");		m_motionSelBox.InsertString(19, "//");
	m_motionSelBox.InsertString(20, "//");					m_motionSelBox.InsertString(21, "//");

	m_motionSelBox.InsertString(22, "Hand Stone");			m_motionSelBox.InsertString(23, "Hand Paper");
	m_motionSelBox.InsertString(24, "Hand Scissor");		m_motionSelBox.InsertString(25, "Hand 14");
	m_motionSelBox.InsertString(26, "Hand 1");				m_motionSelBox.InsertString(27, "Hand RB");
	m_motionSelBox.InsertString(28, "HandNeck");;

	m_motionSelBox.InsertString(29, "//");			// by Inhyeok
	m_motionSelBox.InsertString(30, "Whole-body Bow");		// by Inhyeok
	m_motionSelBox.InsertString(31, "Whole-body Dance");	// by Inhyeok

	m_motionSelBox.SetCurSel(0);


	m_demoSelBox.InsertString(0, "One leg support");		m_demoSelBox.InsertString(1, "DSP Control");
	m_demoSelBox.InsertString(2, "//");						m_demoSelBox.InsertString(3, "//");
	m_demoSelBox.InsertString(4, "//");						m_demoSelBox.InsertString(5, "//");
	m_demoSelBox.SetCurSel(0);

	m_handSelBox.InsertString(0, "C-Grapsping");			m_handSelBox.InsertString(1, "//");//m_handSelBox.InsertString(1, "P-Grapsping");
	m_handSelBox.InsertString(2, "Handshake");				m_handSelBox.InsertString(3, "Salute");
	m_handSelBox.InsertString(4, "//");						m_handSelBox.InsertString(5, "//");
	m_handSelBox.SetCurSel(0);

	OnSelchangeDemolist();
	OnSelchangeHandlist();	

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CWalkingDlg::OnMotion()		// modified by Inhyeok
{
	int i;
	int flag_temp = FALSE;	// Inhyeok

	if( m_motionSelBox.GetCurSel() < 32)
	{
		
		for(i=0; i<NoOfMotion; i++) 
			pSharedMemory->JW_MotionSet[i] = FALSE;		
		
		if( m_motionSelBox.GetCurSel() < 30)		// JW motion
		{
			pSharedMemory->JW_MotionSet[m_motionSelBox.GetCurSel()] = TRUE;
			if(pSharedMemory->CommandFlag == NO_ACT)
			{
				pSharedMemory->CommandFlag = SET_MOCAP;
				Sleep(10);				
			}
			else
				AfxMessageBox("Other Command is activated..!!");
		}
		else		
		{
			if(pSharedMemory->CommandFlag == NO_ACT)
				switch(m_motionSelBox.GetCurSel())	// Inhyeok motion
				{
				case 30:
					pSharedMemory->WB_MocapNo = 1;		// bowing
					flag_temp = TRUE;
					break;
				case 31:
					pSharedMemory->WB_MocapNo = 2;		// dance
					flag_temp = TRUE;
					break;				
				}

			if(pSharedMemory->CommandFlag == NO_ACT )
			{
				if(flag_temp == TRUE)
					pSharedMemory->CommandFlag = INIT_WB_MOCAP;
				Sleep(10);
			}			
			else 
				AfxMessageBox("Other Command is activated..!!");
		}	
	}
}


void CWalkingDlg::OnSelchangeDemolist() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_demoSelBox.GetCurSel() )
		{
		case 0:		// one-leg stand and up-and-down motion
			GetDlgItem(IDC_DEMO1)->SetWindowText("One Leg");
			GetDlgItem(IDC_DEMO2)->SetWindowText("Up and Down");
			GetDlgItem(IDC_DEMO3)->SetWindowText("Go Init. Pos");
			GetDlgItem(IDC_DEMO1)->ShowWindow(true);
			GetDlgItem(IDC_DEMO2)->ShowWindow(true);
			GetDlgItem(IDC_DEMO3)->ShowWindow(true);
			GetDlgItem(IDC_DEMO4)->ShowWindow(false);
			break;
		case 1:
			GetDlgItem(IDC_DEMO1)->SetWindowText("DSP CTRL ON");
			GetDlgItem(IDC_DEMO2)->SetWindowText("DSP CTRL OFF");
			GetDlgItem(IDC_DEMO1)->ShowWindow(true);
			GetDlgItem(IDC_DEMO2)->ShowWindow(true);
			GetDlgItem(IDC_DEMO3)->ShowWindow(false);
			GetDlgItem(IDC_DEMO4)->ShowWindow(false);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo1() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_demoSelBox.GetCurSel() )
		{
		case 0:		// one-leg stand and up-and-down motion
			pSharedMemory->CommandData = ONE_LEG_STAND;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		case 1:
			pSharedMemory->CommandData = DSP_ON;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo2() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch(m_demoSelBox.GetCurSel())
		{
		case 0:		// one-leg stand and up-and-down motion
			pSharedMemory->CommandData = ONE_LEG_UP_AND_DOWN;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		case 1:
			pSharedMemory->CommandData = DSP_OFF;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo3() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_demoSelBox.GetCurSel() )
		{
		case 0:		// one-leg stand and up-and-down motion
			pSharedMemory->CommandData = WALKREADY_POS;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo4() 
{
	// TODO: Add your control notification handler code here
	
}

void CWalkingDlg::OnDatasave() 
{
	// TODO: Add your control notification handler code here
	FILE* fp;
	unsigned int i, j;
	fp = fopen("angleData.txt", "w");
	for(i=0 ; i<N_ROW_SAVE ; i++) 
	{
		for(j=0 ; j<N_COLUMN_SAVE ; j++) fprintf(fp, "%f\t", pSharedMemory->Data_Debug[j][i]);
		fprintf(fp, "\n");
	}
	fclose(fp);
}

void CWalkingDlg::OnSelchangeHandlist() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// c-mode grasping
			GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
			GetDlgItem(IDC_HAND2)->SetWindowText("Grasp On");
			GetDlgItem(IDC_HAND3)->SetWindowText("Grasp Off");
			GetDlgItem(IDC_HAND4)->SetWindowText("Grasp Stop");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND3)->ShowWindow(true);
			GetDlgItem(IDC_HAND4)->ShowWindow(true);

			pSharedMemory->JointID = RF1;
			pSharedMemory->CommandFlag = C_CONTROL_MODE;
			break;
/*		case 1:	// p-mode grasping
			GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
			GetDlgItem(IDC_HAND2)->SetWindowText("Grasp On");
			GetDlgItem(IDC_HAND3)->SetWindowText("Grasp Off");
			GetDlgItem(IDC_HAND4)->SetWindowText("Grasp Stop");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND3)->ShowWindow(true);
			GetDlgItem(IDC_HAND4)->ShowWindow(true);

			pSharedMemory->JointID = RF1;
			pSharedMemory->CommandFlag = P_CONTROL_MODE;
			break;
*/		case 2:
			GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
			GetDlgItem(IDC_HAND2)->SetWindowText("Start");
			GetDlgItem(IDC_HAND3)->SetWindowText("Grasp On");
			GetDlgItem(IDC_HAND4)->SetWindowText("Grasp Off");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND3)->ShowWindow(true);
			GetDlgItem(IDC_HAND4)->ShowWindow(true);
			break;
		case 3:
			GetDlgItem(IDC_HAND1)->SetWindowText("Salute");
			GetDlgItem(IDC_HAND2)->SetWindowText("Init. Pos");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND3)->ShowWindow(false);
			GetDlgItem(IDC_HAND4)->ShowWindow(false);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand1() 
{
	// TODO: Add your control notification handler code here
	
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->MotorControlMode = CTRLMODE_HANDSHAKE;
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : go to grasping position
		case 1:	// c-mode : go to grasping position
		case 2:
			GetDlgItem(IDC_HAND1)->GetWindowText(strText);
			if(strText == "Grasp Pos")
			{
				GetDlgItem(IDC_HAND1)->SetWindowText("Init. Pos");
				// goto handshake pos
				pSharedMemory->ShakeHandsFlag = 1;

				pSharedMemory->JointID = RF1;
				pSharedMemory->CommandFlag = C_CONTROL_MODE;

				Sleep(50);
				pSharedMemory->CommandFlag = GRIP_OFF;
				Sleep(1000);
				pSharedMemory->CommandFlag = GRIP_STOP;

				GetDlgItem(IDC_CURRENT)->SetWindowText("30.0");
			}
			else
			{
				GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
				// goto init pos.
				pSharedMemory->ShakeHandsFlag = 5;

				pSharedMemory->JointID = RF1;
				pSharedMemory->CommandFlag = P_CONTROL_MODE;
			}
			break;
		case 3:
			pSharedMemory->JointID = RSP;
			pSharedMemory->GoalAngle = -80.0f;
			pSharedMemory->GoalTime = 2000.0f;
			//pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
			Sleep(100);

			pSharedMemory->JointID = RSR;
			pSharedMemory->GoalAngle = -20.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
			Sleep(100);

			pSharedMemory->JointID = REB;
			pSharedMemory->GoalAngle = -105.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
			Sleep(100);

			pSharedMemory->JointID = RSY;
			pSharedMemory->GoalAngle = 40.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
			Sleep(100);
			
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;

			pSharedMemory->CommandFlag = GRIP_OFF;
			Sleep(1000);
			pSharedMemory->CommandFlag = GRIP_STOP;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand2() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : grasping on
		case 1:	// c-mode : grasping on
			GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
			pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
			Sleep(10);
			pSharedMemory->CommandFlag = GRIP_ON;
			Sleep(10);
			break;
		case 2:
			// hand FT nulling and start
			pSharedMemory->CommandFlag = NULL_WRIST_FT_SENSOR;
			Sleep(500);
			pSharedMemory->ShakeHandsFlag = 3;
			break;
		case 3:
			pSharedMemory->CommandFlag = GOTO_WALK_READY_POS;
			Sleep(10);
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			Sleep(100);

			pSharedMemory->CommandFlag = GRIP_ON;
			Sleep(1000);
			pSharedMemory->CommandFlag = GRIP_STOP;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand3() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : grasping off
		case 1:	// c-mode : grasping off
			GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
			pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
			Sleep(10);
			pSharedMemory->CommandFlag = GRIP_OFF;
			Sleep(10);
			break;
		case 2:
			// grasp on
			pSharedMemory->CommandFlag = GRIP_ON;
			Sleep(10);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand4() 
{
	// TODO: Add your control notification handler code here
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : grasping stop
		case 1:	// c-mode : grasping stop
			pSharedMemory->CommandFlag = GRIP_STOP;
			Sleep(10);
			break;
		case 2:
			pSharedMemory->CommandFlag = GRIP_OFF;
			Sleep(2000);
			pSharedMemory->CommandFlag = GRIP_STOP;
			Sleep(10);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnRangeonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_RANGEONOFF)->GetWindowText(strText);
		if(strText == "Range On")
		{
			pSharedMemory->CommandData = 0x01;
			pSharedMemory->CommandFlag = POSITION_LIMIT_ONOFF;
			GetDlgItem(IDC_RANGEONOFF)->SetWindowText("Range Off");
			GetDlgItem(IDC_WALKREADY)->EnableWindow(true);
		}
		else
		{
			pSharedMemory->CommandData = 0x00;
			pSharedMemory->CommandFlag = POSITION_LIMIT_ONOFF;
			GetDlgItem(IDC_RANGEONOFF)->SetWindowText("Range On");
			GetDlgItem(IDC_WALKREADY)->EnableWindow(false);
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnPluscurrent() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	float temp;
	
	GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
	temp = (float)atof(strText);
	temp += 5.0f;
	
	strText.Format("%f", temp);
	GetDlgItem(IDC_CURRENT)->SetWindowText(strText);
}

void CWalkingDlg::OnMinuscurrent() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	float temp;
	
	GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
	temp = (float)atof(strText);
	temp -= 5.0f;

	strText.Format("%f", temp);
	GetDlgItem(IDC_CURRENT)->SetWindowText(strText);
}
