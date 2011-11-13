#if !defined(AFX_PRESETDLG_H__AA2C7B8A_3BB0_46E1_B5D6_E4F36721A6AE__INCLUDED_)
#define AFX_PRESETDLG_H__AA2C7B8A_3BB0_46E1_B5D6_E4F36721A6AE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// PresetDlg.h : header file
//


typedef struct _PRESET_POS_
{
	float		Time[10];
	float		PresetPos[10][38];
	char		PresetName[10];
} PRESET_POS, *pPRESET_POS;


/////////////////////////////////////////////////////////////////////////////
// CPresetDlg dialog

class CPresetDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CPresetDlg)

// Construction
public:
	CPresetDlg();
	~CPresetDlg();

	PRESET_POS presetPos;
// Dialog Data
	//{{AFX_DATA(CPresetDlg)
	enum { IDD = IDD_PRESET_DIALOG };
	CComboBox	m_presetSelBox;
	float	m_rhyPos;
	float	m_rhrPos;
	float	m_rhpPos;
	float	m_rknPos;
	float	m_rapPos;
	float	m_rarPos;
	float	m_lhyPos;
	float	m_lhrPos;
	float	m_lhpPos;
	float	m_lknPos;
	float	m_lapPos;
	float	m_larPos;
	float	m_rspPos;
	float	m_rsrPos;
	float	m_rsyPos;
	float	m_rebPos;
	float	m_rwyPos;
	float	m_rwpPos;
	float	m_lspPos;
	float	m_lsrPos;
	float	m_lsyPos;
	float	m_lebPos;
	float	m_lwyPos;
	float	m_lwpPos;
	float	m_nkyPos;
	float	m_nk1Pos;
	float	m_nk2Pos;
	float	m_wstPos;
	float	m_rf1Pos;
	float	m_rf2Pos;
	float	m_rf3Pos;
	float	m_rf4Pos;
	float	m_rf5Pos;
	float	m_lf1Pos;
	float	m_lf2Pos;
	float	m_lf3Pos;
	float	m_lf4Pos;
	float	m_lf5Pos;
	float	m_timeToGo;
	//}}AFX_DATA

// User functions
	bool InverseKinematics(float _pos[], float _angle[]);


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CPresetDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CPresetDlg)
	afx_msg void OnLoad();
	virtual BOOL OnInitDialog();
	afx_msg void OnSave();
	afx_msg void OnStore();
	afx_msg void OnSelchangePresetsel();
	afx_msg void OnRightinverse();
	afx_msg void OnLeftinverse();
	afx_msg void OnGopos();
	afx_msg void OnGetcurrent();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PRESETDLG_H__AA2C7B8A_3BB0_46E1_B5D6_E4F36721A6AE__INCLUDED_)


