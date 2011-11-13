#if !defined(AFX_SENSORDLG_H__8444AB40_8A39_457D_A6EB_7F9CF028736C__INCLUDED_)
#define AFX_SENSORDLG_H__8444AB40_8A39_457D_A6EB_7F9CF028736C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SensorDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CSensorDlg dialog

class CSensorDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CSensorDlg)

// Construction
public:
	CSensorDlg();
	~CSensorDlg();

// Dialog Data
	//{{AFX_DATA(CSensorDlg)
	enum { IDD = IDD_SENSOR_DIALOG };
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CSensorDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CSensorDlg)
	afx_msg void OnFtnulling();
	afx_msg void OnFootanglenulling();
	afx_msg void OnImunulling();
	afx_msg void OnTimer(UINT nIDEvent);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SENSORDLG_H__8444AB40_8A39_457D_A6EB_7F9CF028736C__INCLUDED_)
