#if !defined(AFX_JOINTSDLG_H__C735AE88_13DA_49E1_A3D9_2C9B74D0DC88__INCLUDED_)
#define AFX_JOINTSDLG_H__C735AE88_13DA_49E1_A3D9_2C9B74D0DC88__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// JointsDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CJointsDlg dialog

class CJointsDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CJointsDlg)

// Construction
public:
	CJointsDlg();
	~CJointsDlg();

// Dialog Data
	//{{AFX_DATA(CJointsDlg)
	enum { IDD = IDD_JOINTS_DIALOG };
	CComboBox	m_imuSelBox;
	CComboBox	m_ftSelBox;
	CComboBox	m_jointSelBox;
	UINT	m_hdReduction;
	float	m_encoderPPR;
	UINT	m_pulleyDrive;
	UINT	m_pulleyDriven;
	float	m_ftCutOff;
	float	m_ftDecouple00;
	float	m_ftDecouple01;
	float	m_ftDecouple02;
	float	m_ftDecouple10;
	float	m_ftDecouple11;
	float	m_ftDecouple12;
	float	m_ftDecouple20;
	float	m_ftDecouple21;
	float	m_ftDecouple22;
	float	m_ftSFPitch;
	float	m_ftSFRoll;
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CJointsDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CJointsDlg)
	afx_msg void OnStore();
	virtual BOOL OnInitDialog();
	afx_msg void OnSave();
	afx_msg void OnSelchangeJointsel();
	afx_msg void OnFtstore();
	afx_msg void OnImustore();
	afx_msg void OnSelchangeFtsel();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_JOINTSDLG_H__C735AE88_13DA_49E1_A3D9_2C9B74D0DC88__INCLUDED_)
