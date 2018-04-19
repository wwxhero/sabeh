#pragma once
#include "afxcmn.h"


// CWaitDlg dialog

class CWaitDlg : public CDialog
{
	DECLARE_DYNAMIC(CWaitDlg)

public:
	CWaitDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CWaitDlg();

// Dialog Data
	enum { IDD = IDD_DIALOG_wait };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	int m_Timer;
	CProgressCtrl m_pro;
};
