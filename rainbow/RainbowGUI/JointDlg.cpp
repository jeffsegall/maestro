// JointDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "JointDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CJointDlg property page

IMPLEMENT_DYNCREATE(CJointDlg, CPropertyPage)

CJointDlg::CJointDlg() : CPropertyPage(CJointDlg::IDD)
{
	//{{AFX_DATA_INIT(CJointDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}

CJointDlg::~CJointDlg()
{
}

void CJointDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CJointDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CJointDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CJointDlg)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CJointDlg message handlers
