// RainbowGUI.h : main header file for the RAINBOWGUI application
//

#if !defined(AFX_RAINBOWGUI_H__8D0D3C82_BD76_4F96_B417_A3DF6BD6CE12__INCLUDED_)
#define AFX_RAINBOWGUI_H__8D0D3C82_BD76_4F96_B417_A3DF6BD6CE12__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CRainbowGUIApp:
// See RainbowGUI.cpp for the implementation of this class
//

class CRainbowGUIApp : public CWinApp
{
public:
	CRainbowGUIApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRainbowGUIApp)
	public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CRainbowGUIApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_RAINBOWGUI_H__8D0D3C82_BD76_4F96_B417_A3DF6BD6CE12__INCLUDED_)
