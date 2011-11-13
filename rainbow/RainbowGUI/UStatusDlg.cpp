// UStatusDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "UStatusDlg.h"
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
// CUStatusDlg property page

IMPLEMENT_DYNCREATE(CUStatusDlg, CPropertyPage)

CUStatusDlg::CUStatusDlg() : CPropertyPage(CUStatusDlg::IDD)
{
	//{{AFX_DATA_INIT(CUStatusDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}

CUStatusDlg::~CUStatusDlg()
{
}

void CUStatusDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CUStatusDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CUStatusDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CUStatusDlg)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CUStatusDlg message handlers
