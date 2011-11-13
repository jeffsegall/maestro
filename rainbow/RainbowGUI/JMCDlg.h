#if !defined(AFX_JMCDLG_H__0079580C_1BA8_428A_9330_200D358C1049__INCLUDED_)
#define AFX_JMCDLG_H__0079580C_1BA8_428A_9330_200D358C1049__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// JMCDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CJMCDlg dialog

class CJMCDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CJMCDlg)

// Construction
public:
	CJMCDlg();
	~CJMCDlg();

	unsigned char GetBoardIDFromJointID(unsigned char _jointID);
	unsigned char GetMotorChannelFromJointID(unsigned char _jointID);

// Dialog Data
	//{{AFX_DATA(CJMCDlg)
	enum { IDD = IDD_JMC_DIALOG };
	CComboBox	m_jointSelBox;
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CJMCDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CJMCDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSelchangeJointsel();
	afx_msg void OnEncoderset();
	afx_msg void OnSetencoder();
	afx_msg void OnSethomevelacc();
	afx_msg void OnSetmode();
	afx_msg void OnSetgain();
	afx_msg void OnSetposlimit();
	afx_msg void OnSetmaxvelacc();
	afx_msg void OnSetsaturation();
	afx_msg void OnSeterrbound();
	afx_msg void OnSethomepara();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_JMCDLG_H__0079580C_1BA8_428A_9330_200D358C1049__INCLUDED_)
