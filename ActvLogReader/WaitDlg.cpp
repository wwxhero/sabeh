// WaitDlg.cpp : implementation file
//

#include "stdafx.h"
#include "ActvLogReader.h"
#include "WaitDlg.h"
#include ".\waitdlg.h"

extern int g_progress;
extern bool g_LoadingFlag;
extern int g_total;

// CWaitDlg dialog

IMPLEMENT_DYNAMIC(CWaitDlg, CDialog)
CWaitDlg::CWaitDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CWaitDlg::IDD, pParent)
{
}

CWaitDlg::~CWaitDlg()
{
}

void CWaitDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PROGRESS1, m_pro);
}


BEGIN_MESSAGE_MAP(CWaitDlg, CDialog)
	ON_WM_TIMER()
END_MESSAGE_MAP()


// CWaitDlg message handlers

BOOL CWaitDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  Add extra initialization here
	m_Timer = SetTimer(1, 50, 0);
	if ( m_Timer == 0 ) {
		AfxMessageBox("Warning: could not get timer; problems ahead ...");
	}	
	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void CWaitDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	int cur, time100 = 100;
	cur = (g_progress * time100 ) / g_total;
	if(cur > 100) cur = 100;
	m_pro.SetPos(cur);
	if(g_LoadingFlag == false){
		CDialog::OnOK();
	}

	CDialog::OnTimer(nIDEvent);

}
