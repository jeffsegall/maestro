// JointsDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "JointsDlg.h"
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
// CJointsDlg property page

IMPLEMENT_DYNCREATE(CJointsDlg, CPropertyPage)

CJointsDlg::CJointsDlg() : CPropertyPage(CJointsDlg::IDD)
{
	//{{AFX_DATA_INIT(CJointsDlg)
	m_hdReduction = 1;
	m_encoderPPR = 10;
	m_pulleyDrive = 1;
	m_pulleyDriven = 1;
	m_ftCutOff = 0.0f;
	m_ftDecouple00 = 0.0f;
	m_ftDecouple01 = 0.0f;
	m_ftDecouple02 = 0.0f;
	m_ftDecouple10 = 0.0f;
	m_ftDecouple11 = 0.0f;
	m_ftDecouple12 = 0.0f;
	m_ftDecouple20 = 0.0f;
	m_ftDecouple21 = 0.0f;
	m_ftDecouple22 = 0.0f;
	m_ftSFPitch = 0.0f;
	m_ftSFRoll = 0.0f;
	//}}AFX_DATA_INIT
}

CJointsDlg::~CJointsDlg()
{
}

void CJointsDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CJointsDlg)
	DDX_Control(pDX, IDC_IMUSEL, m_imuSelBox);
	DDX_Control(pDX, IDC_FTSEL, m_ftSelBox);
	DDX_Control(pDX, IDC_JOINTSEL, m_jointSelBox);
	DDX_Text(pDX, IDC_HDREDUCTION, m_hdReduction);
	DDV_MinMaxUInt(pDX, m_hdReduction, 1, 500);
	DDX_Text(pDX, IDC_ENCODERPPR, m_encoderPPR);
	DDV_MinMaxFloat(pDX, m_encoderPPR, -30.0f, 30.0f);
	DDX_Text(pDX, IDC_PULLEYDRIVE, m_pulleyDrive);
	DDV_MinMaxUInt(pDX, m_pulleyDrive, 1, 2000);
	DDX_Text(pDX, IDC_PULLEYDRIVEN, m_pulleyDriven);
	DDV_MinMaxUInt(pDX, m_pulleyDriven, 1, 2000);
	DDX_Text(pDX, IDC_FTCUTOFF, m_ftCutOff);
	DDV_MinMaxFloat(pDX, m_ftCutOff, 0.f, 100.f);
	DDX_Text(pDX, IDC_DECOUPLE00, m_ftDecouple00);
	DDX_Text(pDX, IDC_DECOUPLE01, m_ftDecouple01);
	DDX_Text(pDX, IDC_DECOUPLE02, m_ftDecouple02);
	DDX_Text(pDX, IDC_DECOUPLE10, m_ftDecouple10);
	DDX_Text(pDX, IDC_DECOUPLE11, m_ftDecouple11);
	DDX_Text(pDX, IDC_DECOUPLE12, m_ftDecouple12);
	DDX_Text(pDX, IDC_DECOUPLE20, m_ftDecouple20);
	DDX_Text(pDX, IDC_DECOUPLE21, m_ftDecouple21);
	DDX_Text(pDX, IDC_DECOUPLE22, m_ftDecouple22);
	DDX_Text(pDX, IDC_SFPITCH, m_ftSFPitch);
	DDX_Text(pDX, IDC_SFROLL, m_ftSFRoll);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CJointsDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CJointsDlg)
	ON_BN_CLICKED(IDC_STORE, OnStore)
	ON_BN_CLICKED(IDC_SAVE, OnSave)
	ON_CBN_SELCHANGE(IDC_JOINTSEL, OnSelchangeJointsel)
	ON_BN_CLICKED(IDC_FTSTORE, OnFtstore)
	ON_BN_CLICKED(IDC_IMUSTORE, OnImustore)
	ON_CBN_SELCHANGE(IDC_FTSEL, OnSelchangeFtsel)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CJointsDlg message handlers
void CJointsDlg::OnStore() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		UpdateData(true);

		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		pSharedMemory->Joint.HDReduction = m_hdReduction;
		pSharedMemory->Joint.Pulley_drive = m_pulleyDrive;
		pSharedMemory->Joint.Pulley_driven = m_pulleyDriven;
		//pSharedMemory->Joint.Encoder_size = m_encoderPPR;
		//pSharedMemory->Joint.PPR = (float)m_hdReduction*((float)m_pulleyDriven/(float)m_pulleyDrive)*((float)m_encoderPPR)/360.0f;
		switch(pSharedMemory->JointID)
		{
		case RHY:
			pSharedMemory->Joint.JMC = JMC0;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF0_TXDF;
			break;
		case RHR:
			pSharedMemory->Joint.JMC = JMC0;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF0_TXDF;
			break;
		case RHP:
			pSharedMemory->Joint.JMC = JMC1;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF1_TXDF;
			break;
		case RKN:
			pSharedMemory->Joint.JMC = JMC2;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF2_TXDF;
			break;
		case RAP:
			pSharedMemory->Joint.JMC = JMC3;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF3_TXDF;
			break;
		case RAR:
			pSharedMemory->Joint.JMC = JMC3;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF3_TXDF;
			break;
		case LHY:
			pSharedMemory->Joint.JMC = JMC4;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF4_TXDF;
			break;
		case LHR:
			pSharedMemory->Joint.JMC = JMC4;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF4_TXDF;
			break;
		case LHP:
			pSharedMemory->Joint.JMC = JMC5;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF5_TXDF;
			break;
		case LKN:
			pSharedMemory->Joint.JMC = JMC6;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF6_TXDF;
			break;
		case LAP:
			pSharedMemory->Joint.JMC = JMC7;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF7_TXDF;
			break;
		case LAR:
			pSharedMemory->Joint.JMC = JMC7;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF7_TXDF;
			break;
		case RSP:
			pSharedMemory->Joint.JMC = JMC8;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF8_TXDF;
			break;
		case RSR:
			pSharedMemory->Joint.JMC = JMC8;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF8_TXDF;
			break;
		case RSY:
			pSharedMemory->Joint.JMC = JMC9;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF9_TXDF;
			break;
		case REB:
			pSharedMemory->Joint.JMC = JMC9;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF9_TXDF;
			break;
		case RWY:
			pSharedMemory->Joint.JMC = EJMC0;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E0_TXDF;
			break;
		case RWP:
			pSharedMemory->Joint.JMC = EJMC0;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E0_TXDF;
			break;
		case LSP:
			pSharedMemory->Joint.JMC = JMC10;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF10_TXDF;
			break;
		case LSR:
			pSharedMemory->Joint.JMC = JMC10;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF10_TXDF;
			break;
		case LSY:
			pSharedMemory->Joint.JMC = JMC11;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF11_TXDF;
			break;
		case LEB:	
			pSharedMemory->Joint.JMC = JMC11;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF11_TXDF;
			break;
		case LWY:
			pSharedMemory->Joint.JMC = EJMC1;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E1_TXDF;
			break;
		case LWP:
			pSharedMemory->Joint.JMC = EJMC1;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E1_TXDF;
			break;
		case NKY:
			pSharedMemory->Joint.JMC = EJMC2;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E2_TXDF;
			break;
		case NK1:
			pSharedMemory->Joint.JMC = EJMC2;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E2_TXDF;
			break;
		case NK2:
			pSharedMemory->Joint.JMC = EJMC2;
			pSharedMemory->Joint.Motor_channel = 3;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E2_TXDF;
			break;
		case WST:
			pSharedMemory->Joint.JMC = EJMC3;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 0;
			pSharedMemory->Joint.Ref_txdf = REF_E3_TXDF;
			break;
		case RF1:
			pSharedMemory->Joint.JMC = EJMC4;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E4_TXDF;
			break;
		case RF2:
			pSharedMemory->Joint.JMC = EJMC4;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E4_TXDF;
			break;
		case RF3:
			pSharedMemory->Joint.JMC = EJMC4;
			pSharedMemory->Joint.Motor_channel = 3;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E4_TXDF;
			break;
		case RF4:
			pSharedMemory->Joint.JMC = EJMC4;
			pSharedMemory->Joint.Motor_channel = 4;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E4_TXDF;
			break;
		case RF5:
			pSharedMemory->Joint.JMC = EJMC4;
			pSharedMemory->Joint.Motor_channel = 5;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E4_TXDF;
			break;
		case LF1:
			pSharedMemory->Joint.JMC = EJMC5;
			pSharedMemory->Joint.Motor_channel = 1;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E5_TXDF;
			break;
		case LF2:
			pSharedMemory->Joint.JMC = EJMC5;
			pSharedMemory->Joint.Motor_channel = 2;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E5_TXDF;
			break;
		case LF3:
			pSharedMemory->Joint.JMC = EJMC5;
			pSharedMemory->Joint.Motor_channel = 3;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E5_TXDF;
			break;
		case LF4:
			pSharedMemory->Joint.JMC = EJMC5;
			pSharedMemory->Joint.Motor_channel = 4;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E5_TXDF;
			break;
		case LF5:
			pSharedMemory->Joint.JMC = EJMC5;
			pSharedMemory->Joint.Motor_channel = 5;
			pSharedMemory->Joint.CAN_channel = 1;
			pSharedMemory->Joint.Ref_txdf = REF_E5_TXDF;
			break;
		}
		pSharedMemory->CommandFlag = SET_JOINT_PARAMETER;


	}
	else AfxMessageBox("Other Command is activated..!!");
}

BOOL CJointsDlg::OnInitDialog() 
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

	m_ftSelBox.InsertString(0, "RFFT");		m_ftSelBox.InsertString(1, "LFFT");
	m_ftSelBox.InsertString(2, "RWFT");		m_ftSelBox.InsertString(3, "LWFT");
	m_ftSelBox.SetCurSel(0);

	m_imuSelBox.InsertString(0, "CENTERIMU");
	m_imuSelBox.SetCurSel(0);

	OnSelchangeJointsel();
	OnSelchangeFtsel();


	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CJointsDlg::OnSave() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SAVE_PARAMETER;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJointsDlg::OnSelchangeJointsel() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = m_jointSelBox.GetCurSel();
		pSharedMemory->CommandFlag = GET_JOINT_PARAMETER;
		Sleep(100);
		
		m_hdReduction = pSharedMemory->Joint.HDReduction;
		m_pulleyDrive = pSharedMemory->Joint.Pulley_drive;
		m_pulleyDriven = pSharedMemory->Joint.Pulley_driven;
		m_encoderPPR = pSharedMemory->Joint.PPR/4000.0f;
		

		UpdateData(false);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CJointsDlg::OnFtstore() 
{
	// TODO: Add your control notification handler code here
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		UpdateData(true);

		pSharedMemory->FTsensorID = m_ftSelBox.GetCurSel();
		pSharedMemory->FTsensor.CutOffFeq = m_ftCutOff;
		pSharedMemory->FTsensor.DCMat[0][0] = m_ftDecouple00;
		pSharedMemory->FTsensor.DCMat[0][1] = m_ftDecouple01;
		pSharedMemory->FTsensor.DCMat[0][2] = m_ftDecouple02;
		pSharedMemory->FTsensor.DCMat[1][0] = m_ftDecouple10;
		pSharedMemory->FTsensor.DCMat[1][1] = m_ftDecouple11;
		pSharedMemory->FTsensor.DCMat[1][2] = m_ftDecouple12;
		pSharedMemory->FTsensor.DCMat[2][0] = m_ftDecouple20;
		pSharedMemory->FTsensor.DCMat[2][1] = m_ftDecouple21;
		pSharedMemory->FTsensor.DCMat[2][2] = m_ftDecouple22;
		pSharedMemory->FTsensor.SF_Roll		= m_ftSFRoll;
		pSharedMemory->FTsensor.SF_Pitch	= m_ftSFPitch;

		switch(pSharedMemory->FTsensorID)
		{
		case RFFT:
			pSharedMemory->FTsensor.Controller_NO = FT0;
			pSharedMemory->FTsensor.CAN_channel = CAN0;
			break;
		case LFFT:
			pSharedMemory->FTsensor.Controller_NO = FT1;
			pSharedMemory->FTsensor.CAN_channel = CAN0;
			break;
		case RWFT:
			pSharedMemory->FTsensor.Controller_NO = FT2;
			pSharedMemory->FTsensor.CAN_channel = CAN1;
			break;
		case LWFT:
			pSharedMemory->FTsensor.Controller_NO = FT3;
			pSharedMemory->FTsensor.CAN_channel = CAN1;
			break;
		}
		pSharedMemory->CommandFlag = SET_FT_PARAMETER;
	}
}

void CJointsDlg::OnImustore() 
{
	// TODO: Add your control notification handler code here
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		UpdateData(true);

		pSharedMemory->IMUsensorID = m_imuSelBox.GetCurSel();

		switch(pSharedMemory->IMUsensorID)
		{
		case CENTERIMU:
			pSharedMemory->IMUsensor.CAN_channel		= 0;
			pSharedMemory->IMUsensor.Controller_NO		= IMU0;
			break;
		}
		pSharedMemory->CommandFlag = SET_IMU_PARAMETER;
	}
}

void CJointsDlg::OnSelchangeFtsel() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->FTsensorID = m_ftSelBox.GetCurSel();
		pSharedMemory->CommandFlag = GET_FT_PARAMETER;
		Sleep(100);
		
		m_ftCutOff		= pSharedMemory->FTsensor.CutOffFeq;
		m_ftDecouple00	= pSharedMemory->FTsensor.DCMat[0][0];
		m_ftDecouple01	= pSharedMemory->FTsensor.DCMat[0][1];
		m_ftDecouple02	= pSharedMemory->FTsensor.DCMat[0][2];
		m_ftDecouple10	= pSharedMemory->FTsensor.DCMat[1][0];
		m_ftDecouple11	= pSharedMemory->FTsensor.DCMat[1][1];
		m_ftDecouple12	= pSharedMemory->FTsensor.DCMat[1][2];
		m_ftDecouple20	= pSharedMemory->FTsensor.DCMat[2][0];
		m_ftDecouple21	= pSharedMemory->FTsensor.DCMat[2][1];
		m_ftDecouple22	= pSharedMemory->FTsensor.DCMat[2][2];
		m_ftSFRoll		= pSharedMemory->FTsensor.SF_Roll;
		m_ftSFPitch		= pSharedMemory->FTsensor.SF_Pitch;
		
		UpdateData(false);
	}
	else AfxMessageBox("Other Command is activated..!!");
}
