#if !defined(AFX_USTATUSDLG_H__2B6ABB29_8D6A_4DC3_AFDE_1D371BD6E871__INCLUDED_)
#define AFX_USTATUSDLG_H__2B6ABB29_8D6A_4DC3_AFDE_1D371BD6E871__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// UStatusDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CUStatusDlg dialog

class CUStatusDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CUStatusDlg)

// Construction
public:
	CUStatusDlg();
	~CUStatusDlg();

// Dialog Data
	//{{AFX_DATA(CUStatusDlg)
	enum { IDD = IDD_USTATUS_DIALOG };
		// NOTE - ClassWizard will add data members here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CUStatusDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CUStatusDlg)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_USTATUSDLG_H__2B6ABB29_8D6A_4DC3_AFDE_1D371BD6E871__INCLUDED_)
