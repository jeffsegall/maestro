#if !defined(AFX_USERDLG_H__53C63BFB_49FD_4033_90A9_0AB473811B7E__INCLUDED_)
#define AFX_USERDLG_H__53C63BFB_49FD_4033_90A9_0AB473811B7E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// UserDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CUserDlg dialog

class CUserDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CUserDlg)

// Construction
public:
	CUserDlg();
	~CUserDlg();

// Dialog Data
	//{{AFX_DATA(CUserDlg)
	enum { IDD = IDD_USER_DIALOG };
		// NOTE - ClassWizard will add data members here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CUserDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CUserDlg)
	afx_msg void OnTest1();
	afx_msg void OnTest2();
	afx_msg void OnTest3();
	afx_msg void OnTest4();
	afx_msg void OnTest5();
	afx_msg void OnButton1();
	afx_msg void OnDampingSet();
	afx_msg void OnRollDampingSet();
	afx_msg void OnPitchDampingSet();
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_USERDLG_H__53C63BFB_49FD_4033_90A9_0AB473811B7E__INCLUDED_)
