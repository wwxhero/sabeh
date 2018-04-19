// ListBoxEx.cpp : implementation file
//

#include "stdafx.h"
#include "ActvLogReader.h"
#include "ListBoxEx.h"
#include ".\listboxex.h"


// CListBoxEx

IMPLEMENT_DYNAMIC(CListBoxEx, CListBox)
CListBoxEx::CListBoxEx()
{
}

CListBoxEx::~CListBoxEx()
{
}


BEGIN_MESSAGE_MAP(CListBoxEx, CListBox)
	ON_WM_LBUTTONDOWN()
END_MESSAGE_MAP()



// CListBoxEx message handlers


void CListBoxEx::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default

	CListBox::OnLButtonDown(nFlags, point);
}
