// ActvLogReaderDlg.h : header file
//

#pragma once
#include "afxwin.h"

struct CActEvent {
	int count;
	int frm;
	CString name;
	CRootEvent *rp;
};

struct CEventById {
	int id;
	vector<int> m_id;
};

struct CEventByName {
	CString name;
	vector<int> m_name;
};

// CActvLogReaderDlg dialog
class CActvLogReaderDlg : public CDialog
{
// Construction
public:
	CActvLogReaderDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_ACTVLOGREADER_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnLbnSelchangeList();
	vector<CActEvent> m_eventList;
	vector<CEventById> m_eventbyid;
	vector<CEventByName> m_eventbyname;
	CListBox m_event;
	CListBox m_filtered;
	list<int> m_id;
	afx_msg void OnBnClickedButtonname();
	afx_msg void OnBnClickedButtonid();
	int m_type;
	afx_msg void OnBnClickedButtonfilter();
	afx_msg void OnBnClickedButtonunfilter();
	CListBox m_relate;
	afx_msg void OnLbnSelchangeListrelate();
	CString GetString(CString s, CString sBegin);
	CString GetString(CString s, CString sBegin, CString sEnd);
	CString GetString(CString &s, int m);

	int GetValue(CString s, CString sBegin);
	float GetFloat(CString s, CString sBegin);
	void GetDisplayData(CString s,float& vTime,int& vHcsmId,int& vCvedId,int& vCvedType,CString &sPos, CString &vName,int &vType);
	void RemoveFromEventList();

	void DisplayEvent();
	void fgDisplayEvent();
	void AddToEventList();
	void DisplayEventList(CString DisplayObject,int m,int frm, CString name, float vTime, int vHcsmId, int vCvedId, int vCvedType, CString sPos,CString vName,int vType);
	void DisplayEventList02(CString DisplayObject,int m,int frm, CString name, float vTime, int vHcsmId, CString sName,CString sDetail);
	void fgDisplayEventList(CString DisplayObject,int m,int counter,int frm, CString name, float vTime, int vHcsmId, int vCvedId, int vCvedType, CString sPos,CString vName,int vType, int eSeq);
	void DisplayOneEventRecord(int m);
	void fgDisplayOneEventRecord(int m, int counter);
	void fgRelateInit();
	void fgListInit();
	CString ToCString(int i);
	CString ToCString(float f);
	void ClearDisplay(CString vTableName);
	void fgRemoveFromEventList();
	void AppendOneRow(CString sName,int vRowSeq);
	void ResetDisplay();
	void DisplayRelatedEvent(int vRowNumber);
	void fgListResetContent();
	void listHideDuplicateRecord();
	void relateHideDuplicateRecord();
	void listDisplayDuplicateRecord();
	void relateDisplayDuplicateRecord();
	CString GetLogRecord(int m);
	CString GetDetail(CString s);
	void GetDisplayData02(CString s,float& vTime,int& vHcsmId,CString &sDetail);
	int GetValue(CString s, int m);
	int GetFirstValue(CString &s);
	void RefreshListDisplay();
	void RefreshListDisplay(vector<CString> &OrderByHcsmId);
//	void RefreshListDisplay(vector<CString> &OrderByHcsmId,vector<CRootEvent> &eOrderByHcsmId);
	CString GetName(int vHcsmId);
	CString RemoveNameFromDetail(CString sDetail,CString sName);
	CString GetHcsmId(CString s);
	void SortByName();
	void SortByFrameId();


public:
	afx_msg void OnLbnSelchangeListevent();

public:
	afx_msg void OnLbnDblclkListfilted();
public:
	afx_msg void OnLbnDblclkListevent();
public:
	afx_msg void OnLbnSelchangeListfilted();
public:
//	CMsflexgrid_list m_fgList;
public:
//	CMsflexgrid_list m_fgRelate;
public:
	DECLARE_EVENTSINK_MAP()
public:
	void RowColChangeMsflexgridList();
public:
	afx_msg void OnBnClickedCheckHideInstantiate();
public:
	afx_msg void OnBnClickedCheckHideDuplicate();
public:
	CButton m_cbDuplicateRecord;
public:
	CButton m_cbInstantiateRecord;
public:
	afx_msg void OnBnClickedButtonHcsmid();
public:
	afx_msg void OnLbnDblclkList();
public:
	afx_msg void OnBnClickedButtonrefresh();
};
