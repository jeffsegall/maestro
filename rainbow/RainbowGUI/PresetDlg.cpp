// PresetDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "PresetDlg.h"
#include <math.h>
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
// CPresetDlg property page

IMPLEMENT_DYNCREATE(CPresetDlg, CPropertyPage)

CPresetDlg::CPresetDlg() : CPropertyPage(CPresetDlg::IDD)
{
	//{{AFX_DATA_INIT(CPresetDlg)
	m_rhyPos = 0.0f;	m_rhrPos = 0.0f;	m_rhpPos = 0.0f;
	m_rknPos = 0.0f;	m_rapPos = 0.0f;	m_rarPos = 0.0f;
	m_lhyPos = 0.0f;	m_lhrPos = 0.0f;	m_lhpPos = 0.0f;
	m_lknPos = 0.0f;	m_lapPos = 0.0f;	m_larPos = 0.0f;
	m_rspPos = 0.0f;	m_rsrPos = 0.0f;	m_rsyPos = 0.0f;
	m_rebPos = 0.0f;	m_rwyPos = 0.0f;	m_rwpPos = 0.0f;
	m_lspPos = 0.0f;	m_lsrPos = 0.0f;	m_lsyPos = 0.0f;
	m_lebPos = 0.0f;	m_lwyPos = 0.0f;	m_lwpPos = 0.0f;
	m_nkyPos = 0.0f;	m_nk1Pos = 0.0f;	m_nk2Pos = 0.0f;
	m_wstPos = 0.0f;	m_rf1Pos = 0.0f;	m_rf2Pos = 0.0f;
	m_rf3Pos = 0.0f;	m_rf4Pos = 0.0f;	m_rf5Pos = 0.0f;
	m_lf1Pos = 0.0f;	m_lf2Pos = 0.0f;	m_lf3Pos = 0.0f;
	m_lf4Pos = 0.0f;	m_lf5Pos = 0.0f;
	m_timeToGo = 0.0f;
	//}}AFX_DATA_INIT
}

CPresetDlg::~CPresetDlg()
{
}

void CPresetDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CPresetDlg)
	DDX_Control(pDX, IDC_PRESETSEL, m_presetSelBox);
	DDX_Text(pDX, IDC_RHY, m_rhyPos);
	DDX_Text(pDX, IDC_RHR, m_rhrPos);
	DDX_Text(pDX, IDC_RHP, m_rhpPos);
	DDX_Text(pDX, IDC_RKN, m_rknPos);
	DDX_Text(pDX, IDC_RAP, m_rapPos);
	DDX_Text(pDX, IDC_RAR, m_rarPos);
	DDX_Text(pDX, IDC_LHY, m_lhyPos);
	DDX_Text(pDX, IDC_LHR, m_lhrPos);
	DDX_Text(pDX, IDC_LHP, m_lhpPos);
	DDX_Text(pDX, IDC_LKN, m_lknPos);
	DDX_Text(pDX, IDC_LAP, m_lapPos);
	DDX_Text(pDX, IDC_LAR, m_larPos);
	DDX_Text(pDX, IDC_RSP, m_rspPos);
	DDX_Text(pDX, IDC_RSR, m_rsrPos);
	DDX_Text(pDX, IDC_RSY, m_rsyPos);
	DDX_Text(pDX, IDC_REB, m_rebPos);
	DDX_Text(pDX, IDC_RWY, m_rwyPos);
	DDX_Text(pDX, IDC_RWP, m_rwpPos);
	DDX_Text(pDX, IDC_LSP, m_lspPos);
	DDX_Text(pDX, IDC_LSR, m_lsrPos);
	DDX_Text(pDX, IDC_LSY, m_lsyPos);
	DDX_Text(pDX, IDC_LEB, m_lebPos);
	DDX_Text(pDX, IDC_LWY, m_lwyPos);
	DDX_Text(pDX, IDC_LWP, m_lwpPos);
	DDX_Text(pDX, IDC_NKY, m_nkyPos);
	DDX_Text(pDX, IDC_NK1, m_nk1Pos);
	DDX_Text(pDX, IDC_NK2, m_nk2Pos);
	DDX_Text(pDX, IDC_WST, m_wstPos);
	DDX_Text(pDX, IDC_RF1, m_rf1Pos);
	DDX_Text(pDX, IDC_RF2, m_rf2Pos);
	DDX_Text(pDX, IDC_RF3, m_rf3Pos);
	DDX_Text(pDX, IDC_RF4, m_rf4Pos);
	DDX_Text(pDX, IDC_RF5, m_rf5Pos);
	DDX_Text(pDX, IDC_LF1, m_lf1Pos);
	DDX_Text(pDX, IDC_LF2, m_lf2Pos);
	DDX_Text(pDX, IDC_LF3, m_lf3Pos);
	DDX_Text(pDX, IDC_LF4, m_lf4Pos);
	DDX_Text(pDX, IDC_LF5, m_lf5Pos);
	DDX_Text(pDX, IDC_TIME, m_timeToGo);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CPresetDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CPresetDlg)
	ON_BN_CLICKED(IDC_SAVE, OnSave)
	ON_BN_CLICKED(IDC_STORE, OnStore)
	ON_CBN_SELCHANGE(IDC_PRESETSEL, OnSelchangePresetsel)
	ON_BN_CLICKED(IDC_RIGHTINVERSE, OnRightinverse)
	ON_BN_CLICKED(IDC_LEFTINVERSE, OnLeftinverse)
	ON_BN_CLICKED(IDC_GOPOS, OnGopos)
	ON_BN_CLICKED(IDC_GETCURRENT, OnGetcurrent)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CPresetDlg message handlers

void CPresetDlg::OnLoad() 
{
	// TODO: Add your control notification handler code here
	FILE *fp;
	fp = fopen("preset.per", "rb");

	if(fp == NULL){ AfxMessageBox("Can not open file(preset.per)..!!"); return; }
	else
	{
		fread(&presetPos, sizeof(PRESET_POS), 1, fp);
		OnSelchangePresetsel();
		fclose(fp);
	}
}

BOOL CPresetDlg::OnInitDialog() 
{
	CPropertyPage::OnInitDialog();
	
	// TODO: Add extra initialization here

	CString strText;
	unsigned char i;

	for(i=0 ; i<10 ; i++)
	{
		strText.Format("Preset%d", i);
		m_presetSelBox.InsertString(i, strText);
	}
	m_presetSelBox.SetCurSel(0);

	OnLoad();

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CPresetDlg::OnSave() 
{
	// TODO: Add your control notification handler code here
	FILE *fp;
	fp = fopen("preset.per", "wb");

	if(fp == NULL){ AfxMessageBox("Can not open file(preset.per)..!!"); return; }
	else
	{
		fwrite(&presetPos, sizeof(PRESET_POS), 1, fp);
		fclose(fp);
	}
}

void CPresetDlg::OnStore() 
{
	// TODO: Add your control notification handler code here
	int i = m_presetSelBox.GetCurSel();

	UpdateData(true);

	presetPos.Time[i] = m_timeToGo;
	presetPos.PresetPos[i][0] = m_rhyPos;
	presetPos.PresetPos[i][1] = m_rhrPos;
	presetPos.PresetPos[i][2] = m_rhpPos;
	presetPos.PresetPos[i][3] = m_rknPos;
	presetPos.PresetPos[i][4] = m_rapPos;
	presetPos.PresetPos[i][5] = m_rarPos;
	presetPos.PresetPos[i][6] = m_lhyPos;
	presetPos.PresetPos[i][7] = m_lhrPos;
	presetPos.PresetPos[i][8] = m_lhpPos;
	presetPos.PresetPos[i][9] = m_lknPos;
	presetPos.PresetPos[i][10] = m_lapPos;
	presetPos.PresetPos[i][11] = m_larPos;

	presetPos.PresetPos[i][12] = m_rspPos;
	presetPos.PresetPos[i][13] = m_rsrPos;
	presetPos.PresetPos[i][14] = m_rsyPos;
	presetPos.PresetPos[i][15] = m_rebPos;
	presetPos.PresetPos[i][16] = m_rwyPos;
	presetPos.PresetPos[i][17] = m_rwpPos;
	presetPos.PresetPos[i][18] = m_lspPos;
	presetPos.PresetPos[i][19] = m_lsrPos;
	presetPos.PresetPos[i][20] = m_lsyPos;
	presetPos.PresetPos[i][21] = m_lebPos;
	presetPos.PresetPos[i][22] = m_lwyPos;
	presetPos.PresetPos[i][23] = m_lwpPos;

	presetPos.PresetPos[i][24] = m_nkyPos;
	presetPos.PresetPos[i][25] = m_nk1Pos;
	presetPos.PresetPos[i][26] = m_nk2Pos;
	presetPos.PresetPos[i][27] = m_wstPos;
	
	presetPos.PresetPos[i][28] = m_rf1Pos;
	presetPos.PresetPos[i][29] = m_rf2Pos;
	presetPos.PresetPos[i][30] = m_rf3Pos;
	presetPos.PresetPos[i][31] = m_rf4Pos;
	presetPos.PresetPos[i][32] = m_rf5Pos;
	presetPos.PresetPos[i][33] = m_lf1Pos;
	presetPos.PresetPos[i][34] = m_lf2Pos;
	presetPos.PresetPos[i][35] = m_lf3Pos;
	presetPos.PresetPos[i][36] = m_lf4Pos;
	presetPos.PresetPos[i][37] = m_lf5Pos;
}

void CPresetDlg::OnSelchangePresetsel() 
{
	// TODO: Add your control notification handler code here
	int i = m_presetSelBox.GetCurSel();
	
	m_timeToGo = presetPos.Time[i];
	m_rhyPos = presetPos.PresetPos[i][0];
	m_rhrPos = presetPos.PresetPos[i][1];
	m_rhpPos = presetPos.PresetPos[i][2];
	m_rknPos = presetPos.PresetPos[i][3];
	m_rapPos = presetPos.PresetPos[i][4];
	m_rarPos = presetPos.PresetPos[i][5];
	m_lhyPos = presetPos.PresetPos[i][6];
	m_lhrPos = presetPos.PresetPos[i][7];
	m_lhpPos = presetPos.PresetPos[i][8];
	m_lknPos = presetPos.PresetPos[i][9];
	m_lapPos = presetPos.PresetPos[i][10];
	m_larPos = presetPos.PresetPos[i][11];

	m_rspPos = presetPos.PresetPos[i][12];
	m_rsrPos = presetPos.PresetPos[i][13];
	m_rsyPos = presetPos.PresetPos[i][14];
	m_rebPos = presetPos.PresetPos[i][15];
	m_rwyPos = presetPos.PresetPos[i][16];
	m_rwpPos = presetPos.PresetPos[i][17];
	m_lspPos = presetPos.PresetPos[i][18];
	m_lsrPos = presetPos.PresetPos[i][19];
	m_lsyPos = presetPos.PresetPos[i][20];
	m_lebPos = presetPos.PresetPos[i][21];
	m_lwyPos = presetPos.PresetPos[i][22];
	m_lwpPos = presetPos.PresetPos[i][23];

	m_nkyPos = presetPos.PresetPos[i][24];
	m_nk1Pos = presetPos.PresetPos[i][25];
	m_nk2Pos = presetPos.PresetPos[i][26];
	m_wstPos = presetPos.PresetPos[i][27];
	
	m_rf1Pos = presetPos.PresetPos[i][28];
	m_rf2Pos = presetPos.PresetPos[i][29];
	m_rf3Pos = presetPos.PresetPos[i][30];
	m_rf4Pos = presetPos.PresetPos[i][31];
	m_rf5Pos = presetPos.PresetPos[i][32];
	m_lf1Pos = presetPos.PresetPos[i][33];
	m_lf2Pos = presetPos.PresetPos[i][34];
	m_lf3Pos = presetPos.PresetPos[i][35];
	m_lf4Pos = presetPos.PresetPos[i][36];
	m_lf5Pos = presetPos.PresetPos[i][37];

	UpdateData(false);
}

void CPresetDlg::OnRightinverse() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	float pos[3], angle[6];

	GetDlgItem(IDC_RIGHTTASKX)->GetWindowText(strText);
	pos[0] = (float)atof(strText);
	GetDlgItem(IDC_RIGHTTASKY)->GetWindowText(strText);
	pos[1] = (float)atof(strText);
	GetDlgItem(IDC_RIGHTTASKZ)->GetWindowText(strText);
	pos[2] = (float)atof(strText);

	InverseKinematics(pos, angle);

	m_rhyPos = angle[0];	m_rhrPos = angle[1];	m_rhpPos = angle[2];
	m_rknPos = angle[3];	m_rapPos = angle[4];	m_rarPos = angle[5];
	
	UpdateData(false);
}

void CPresetDlg::OnLeftinverse() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	float pos[3], angle[6];

	GetDlgItem(IDC_LEFTTASKX)->GetWindowText(strText);
	pos[0] = (float)atof(strText);
	GetDlgItem(IDC_LEFTTASKY)->GetWindowText(strText);
	pos[1] = (float)atof(strText);
	GetDlgItem(IDC_LEFTTASKZ)->GetWindowText(strText);
	pos[2] = (float)atof(strText);

	InverseKinematics(pos, angle);

	m_lhyPos = angle[0];	m_lhrPos = angle[1];	m_lhpPos = angle[2];
	m_lknPos = angle[3];	m_lapPos = angle[4];	m_larPos = angle[5];
	
	UpdateData(false);
}

bool CPresetDlg::InverseKinematics(float _pos[], float _angle[])
{
	unsigned char i;
	float angle[6];

	// knee
	angle[3] = PI-acosf((LENGTH_THIGH*LENGTH_THIGH+LENGTH_CALF*LENGTH_CALF-(_pos[0]*_pos[0]+_pos[1]*_pos[1]+_pos[2]*_pos[2]))/(2.0f*LENGTH_THIGH*LENGTH_CALF));
	// hip roll
	angle[1] = atan2f(_pos[1], -_pos[2]);
	// hip pitch
	angle[2] = asinf((-sinf(angle[3])*LENGTH_CALF*(-_pos[1]*sinf(angle[1])+_pos[2]*cosf(angle[1]))+_pos[0]*(cosf(angle[3])*LENGTH_CALF+LENGTH_THIGH))/(-_pos[0]*_pos[0]-(_pos[1]*sinf(angle[1])-_pos[2]*cosf(angle[1]))*(_pos[1]*sinf(angle[1])-_pos[2]*cosf(angle[1]))));
	// hip yaw
	angle[0] = DEG2RAD*_pos[5];
	// ankle roll
	angle[5] = -angle[1];
	// ankle pitch
	angle[4] = -angle[2]-angle[3];

	for(i=0 ; i<6 ; i++) _angle[i] = RAD2DEG*angle[i];

	return true;
}

void CPresetDlg::OnGopos() 
{
	// TODO: Add your control notification handler code here

	UpdateData(true);
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
		
		pSharedMemory->GoalTime = m_timeToGo;
		
		pSharedMemory->JointID = RHY;	pSharedMemory->GoalAngle = m_rhyPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RHR;	pSharedMemory->GoalAngle = m_rhrPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RHP;	pSharedMemory->GoalAngle = m_rhpPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RKN;	pSharedMemory->GoalAngle = m_rknPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RAP;	pSharedMemory->GoalAngle = m_rapPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RAR;	pSharedMemory->GoalAngle = m_rarPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LHY;	pSharedMemory->GoalAngle = m_lhyPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LHR;	pSharedMemory->GoalAngle = m_lhrPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LHP;	pSharedMemory->GoalAngle = m_lhpPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LKN;	pSharedMemory->GoalAngle = m_lknPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LAP;	pSharedMemory->GoalAngle = m_lapPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		
		pSharedMemory->JointID = LAR;	pSharedMemory->GoalAngle = m_larPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RSP;	pSharedMemory->GoalAngle = m_rspPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RSR;	pSharedMemory->GoalAngle = m_rsrPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RSY;	pSharedMemory->GoalAngle = m_rsyPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = REB;	pSharedMemory->GoalAngle = m_rebPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RWY;	pSharedMemory->GoalAngle = m_rwyPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RWP;	pSharedMemory->GoalAngle = m_rwpPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LSP;	pSharedMemory->GoalAngle = m_lspPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LSR;	pSharedMemory->GoalAngle = m_lsrPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LSY;	pSharedMemory->GoalAngle = m_lsyPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LEB;	pSharedMemory->GoalAngle = m_lebPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LWY;	pSharedMemory->GoalAngle = m_lwyPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LWP;	pSharedMemory->GoalAngle = m_lwpPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		
		pSharedMemory->JointID = NKY;	pSharedMemory->GoalAngle = m_nkyPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = NK1;	pSharedMemory->GoalAngle = m_nk1Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = NK2;	pSharedMemory->GoalAngle = m_nk2Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = WST;	pSharedMemory->GoalAngle = m_wstPos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		
		pSharedMemory->JointID = RF1;	pSharedMemory->GoalAngle = m_rf1Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RF2;	pSharedMemory->GoalAngle = m_rf2Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RF3;	pSharedMemory->GoalAngle = m_rf3Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RF4;	pSharedMemory->GoalAngle = m_rf4Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = RF5;	pSharedMemory->GoalAngle = m_rf5Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LF1;	pSharedMemory->GoalAngle = m_lf1Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LF2;	pSharedMemory->GoalAngle = m_lf2Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LF3;	pSharedMemory->GoalAngle = m_lf3Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LF4;	pSharedMemory->GoalAngle = m_lf4Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
		pSharedMemory->JointID = LF5;	pSharedMemory->GoalAngle = m_lf5Pos;	pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CPresetDlg::OnGetcurrent() 
{
	// TODO: Add your control notification handler code here
	m_rhyPos = pSharedMemory->Joint_debug[RHY].RefAngleCurrent;
	m_rhrPos = pSharedMemory->Joint_debug[RHR].RefAngleCurrent;
	m_rhpPos = pSharedMemory->Joint_debug[RHP].RefAngleCurrent;
	m_rknPos = pSharedMemory->Joint_debug[RKN].RefAngleCurrent;
	m_rapPos = pSharedMemory->Joint_debug[RAP].RefAngleCurrent;
	m_rarPos = pSharedMemory->Joint_debug[RAR].RefAngleCurrent;
	m_lhyPos = pSharedMemory->Joint_debug[LHY].RefAngleCurrent;
	m_lhrPos = pSharedMemory->Joint_debug[LHR].RefAngleCurrent;
	m_lhpPos = pSharedMemory->Joint_debug[LHP].RefAngleCurrent;
	m_lknPos = pSharedMemory->Joint_debug[LKN].RefAngleCurrent;
	m_lapPos = pSharedMemory->Joint_debug[LAP].RefAngleCurrent;
	m_larPos = pSharedMemory->Joint_debug[LAR].RefAngleCurrent;

	m_rspPos = pSharedMemory->Joint_debug[RSP].RefAngleCurrent;
	m_rsrPos = pSharedMemory->Joint_debug[RSR].RefAngleCurrent;
	m_rsyPos = pSharedMemory->Joint_debug[RSY].RefAngleCurrent;
	m_rebPos = pSharedMemory->Joint_debug[REB].RefAngleCurrent;
	m_rwyPos = pSharedMemory->Joint_debug[RWY].RefAngleCurrent;
	m_rwpPos = pSharedMemory->Joint_debug[RWP].RefAngleCurrent;
	m_lspPos = pSharedMemory->Joint_debug[LSP].RefAngleCurrent;
	m_lsrPos = pSharedMemory->Joint_debug[LSR].RefAngleCurrent;
	m_lsyPos = pSharedMemory->Joint_debug[LSY].RefAngleCurrent;
	m_lebPos = pSharedMemory->Joint_debug[LEB].RefAngleCurrent;
	m_lwyPos = pSharedMemory->Joint_debug[LWY].RefAngleCurrent;
	m_lwpPos = pSharedMemory->Joint_debug[LWP].RefAngleCurrent;

	m_nkyPos = pSharedMemory->Joint_debug[NKY].RefAngleCurrent;
	m_nk1Pos = pSharedMemory->Joint_debug[NK1].RefAngleCurrent;
	m_nk2Pos = pSharedMemory->Joint_debug[NK2].RefAngleCurrent;
	m_wstPos = pSharedMemory->Joint_debug[WST].RefAngleCurrent;
	
	m_rf1Pos = pSharedMemory->Joint_debug[RF1].RefAngleCurrent;
	m_rf2Pos = pSharedMemory->Joint_debug[RF2].RefAngleCurrent;
	m_rf3Pos = pSharedMemory->Joint_debug[RF3].RefAngleCurrent;
	m_rf4Pos = pSharedMemory->Joint_debug[RF4].RefAngleCurrent;
	m_rf5Pos = pSharedMemory->Joint_debug[RF5].RefAngleCurrent;
	m_lf1Pos = pSharedMemory->Joint_debug[LF1].RefAngleCurrent;
	m_lf2Pos = pSharedMemory->Joint_debug[LF2].RefAngleCurrent;
	m_lf3Pos = pSharedMemory->Joint_debug[LF3].RefAngleCurrent;
	m_lf4Pos = pSharedMemory->Joint_debug[LF4].RefAngleCurrent;
	m_lf5Pos = pSharedMemory->Joint_debug[LF5].RefAngleCurrent;

	UpdateData(false);
}
