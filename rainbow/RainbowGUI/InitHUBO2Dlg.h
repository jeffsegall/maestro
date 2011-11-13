#if !defined(AFX_INITHUBO2DLG_H__E838A54A_7F4D_4B4B_8C50_91E9029EE0DA__INCLUDED_)
#define AFX_INITHUBO2DLG_H__E838A54A_7F4D_4B4B_8C50_91E9029EE0DA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// InitHUBO2Dlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CInitHUBO2Dlg dialog

class CInitHUBO2Dlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CInitHUBO2Dlg)

// Construction
public:
	unsigned char GetBoardIDFromJointID(unsigned char _jointID);
	CInitHUBO2Dlg();
	~CInitHUBO2Dlg();

// Dialog Data
	//{{AFX_DATA(CInitHUBO2Dlg)
	enum { IDD = IDD_INITHUBO2_DIALOG };
	CComboBox	m_jointSelBox;
	float	m_offsetPos;
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CInitHUBO2Dlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CInitHUBO2Dlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnGoinitpos();
	afx_msg void OnEncoderzero();
	afx_msg void OnGooffsetpos();
	afx_msg void OnSaveoffset();
	afx_msg void OnAllinitpos();
	afx_msg void OnLowerinitpos();
	afx_msg void OnUpperinitpos();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_INITHUBO2DLG_H__E838A54A_7F4D_4B4B_8C50_91E9029EE0DA__INCLUDED_)
