#if !defined(AFX_LSTATUSDLG_H__BD9C95DC_B8FD_4BED_A501_9FEAC089A621__INCLUDED_)
#define AFX_LSTATUSDLG_H__BD9C95DC_B8FD_4BED_A501_9FEAC089A621__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// LStatusDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CLStatusDlg dialog

class CLStatusDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CLStatusDlg)

// Construction
public:
	CLStatusDlg();
	~CLStatusDlg();

// Dialog Data
	//{{AFX_DATA(CLStatusDlg)
	enum { IDD = IDD_LSTATUS_DIALOG };
		// NOTE - ClassWizard will add data members here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CLStatusDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CLStatusDlg)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_LSTATUSDLG_H__BD9C95DC_B8FD_4BED_A501_9FEAC089A621__INCLUDED_)
