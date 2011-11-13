#if !defined(AFX_JOINTDLG_H__45ECBB23_DB65_49BD_838F_8CE9D2AE1052__INCLUDED_)
#define AFX_JOINTDLG_H__45ECBB23_DB65_49BD_838F_8CE9D2AE1052__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// JointDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CJointDlg dialog

class CJointDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CJointDlg)

// Construction
public:
	CJointDlg();
	~CJointDlg();

// Dialog Data
	//{{AFX_DATA(CJointDlg)
	enum { IDD = IDD_JOINT_DIALOG };
		// NOTE - ClassWizard will add data members here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CJointDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CJointDlg)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_JOINTDLG_H__45ECBB23_DB65_49BD_838F_8CE9D2AE1052__INCLUDED_)
