// LStatusDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "LStatusDlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"

// SharedMemory variable
extern PSHARED_DATA pSharedMemory;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CLStatusDlg property page

IMPLEMENT_DYNCREATE(CLStatusDlg, CPropertyPage)

CLStatusDlg::CLStatusDlg() : CPropertyPage(CLStatusDlg::IDD)
{
	//{{AFX_DATA_INIT(CLStatusDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}

CLStatusDlg::~CLStatusDlg()
{
}

void CLStatusDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CLStatusDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CLStatusDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CLStatusDlg)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CLStatusDlg message handlers
