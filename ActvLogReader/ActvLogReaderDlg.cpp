// ActvLogReaderDlg.cpp : implementation file
//

#include "stdafx.h"
#include "ActvLogReader.h"
#include "ActvLogReaderDlg.h"
#include ".\actvlogreaderdlg.h"
#include "WaitDlgDisplay.h"
//#include ".\actvlogreaderdlg.h"
//#include <atldbcli>	//for DeleteItemStruct

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern int g_progress;
extern int g_total;
extern bool g_LoadingFlag;
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// CActvLogReaderDlg dialog



CActvLogReaderDlg::CActvLogReaderDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CActvLogReaderDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CActvLogReaderDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST_event, m_event);
	DDX_Control(pDX, IDC_LIST_filted, m_filtered);
	DDX_Control(pDX, IDC_LIST_relate, m_relate);
//	DDX_Control(pDX, IDC_MSFLEXGRID_LIST, m_fgList);
//	DDX_Control(pDX, IDC_MSFLEXGRID_RELATE, m_fgRelate);
//	DDX_Control(pDX, IDC_CHECK_HIDE_DUPLICATE, m_cbDuplicateRecord);
//	DDX_Control(pDX, IDC_CHECK_HIDE_INSTANTIATE, m_cbInstantiateRecord);
}

BEGIN_MESSAGE_MAP(CActvLogReaderDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON1, OnBnClickedButton1)
	ON_LBN_SELCHANGE(IDC_LIST, OnLbnSelchangeList)
	ON_BN_CLICKED(IDC_BUTTON_name, OnBnClickedButtonname)
	ON_BN_CLICKED(IDC_BUTTON_id, OnBnClickedButtonid)
	ON_BN_CLICKED(IDC_BUTTON_filter, OnBnClickedButtonfilter)
	ON_BN_CLICKED(IDC_BUTTON_unfilter, OnBnClickedButtonunfilter)
	ON_LBN_SELCHANGE(IDC_LIST_relate, OnLbnSelchangeListrelate)
	ON_LBN_DBLCLK(IDC_LIST_filted, &CActvLogReaderDlg::OnLbnDblclkListfilted)
	ON_LBN_DBLCLK(IDC_LIST_event, &CActvLogReaderDlg::OnLbnDblclkListevent)
//	ON_BN_CLICKED(IDC_CHECK_HIDE_INSTANTIATE, &CActvLogReaderDlg::OnBnClickedCheckHideInstantiate)
//	ON_BN_CLICKED(IDC_CHECK_HIDE_DUPLICATE, &CActvLogReaderDlg::OnBnClickedCheckHideDuplicate)
ON_BN_CLICKED(IDC_BUTTON_HcsmId, &CActvLogReaderDlg::OnBnClickedButtonHcsmid)
ON_LBN_DBLCLK(IDC_LIST, &CActvLogReaderDlg::OnLbnDblclkList)
ON_BN_CLICKED(IDC_BUTTON_refresh, &CActvLogReaderDlg::OnBnClickedButtonrefresh)
END_MESSAGE_MAP()


// CActvLogReaderDlg message handlers

BOOL CActvLogReaderDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	m_type = 2;
	m_relate.AddString("The events with same ID");

	//fgListInit();
	//fgRelateInit();
	//OnBnClickedButton1();
	
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CActvLogReaderDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CActvLogReaderDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CActvLogReaderDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CActvLogReaderDlg::OnBnClickedButton1()
{
	CFileDialog  odlg(TRUE, ".txt");

	if ( odlg.DoModal() != IDOK ) return;

	CEventTestEvent  e1;
	CEventTestEvent  e2;
	int *ip;

	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);

	CActvLog log2;
    istringstream in (ios::in | ios::out);
    ostream out (in.rdbuf ());

//	bool t=log2.Load("D:\\Program Files\\NADS\\Isat\\data\\actvlog_20080428_102027.log");
	if ( !log2.Load( (const char *)odlg.GetPathName() ) ) {
		AfxMessageBox("Could not load file");
		return;
	}

	ResetDisplay();

	BeginWaitCursor();
	g_LoadingFlag = true;
	g_total = log2.GetCount();

	_beginthread(CLoadStatus, 0, NULL);

	char buf[MAX_ACTIVITY_LOG_SIZE];
	char text[64];
	EActvLogType type;
	CRootEvent *pE;
	int i, frm, seleted;
	int count = 0;
	bool found = false;
	list<int>::iterator idp;
	m_eventList.clear();
	m_eventbyname.clear();
	m_eventbyid.clear();
	m_id.clear();

	if ( log2.Get(0, buf, type, frm) ) {
		
		pE = CRootEvent::CreateEventByType(type);
		pE->CopyData(buf);

		CActEvent aevent;
		aevent.count = count;
		aevent.frm = frm;
		aevent.name = pE->GetIdStr();
		aevent.rp = pE;
		m_eventList.push_back(aevent);

		CEventByName abyname;
		abyname.name = pE->GetIdStr();
		abyname.m_name.push_back(count);
		m_eventbyname.push_back(abyname);

		m_id.push_back(frm);

		CEventById abyid;
		ip = (int*) pE->GetData();
		abyid.id = *ip;
		abyid.m_id.push_back(count);
		m_eventbyid.push_back(abyid);

		g_progress = count;
		count++;

		while ( log2.GetNext(buf, type, frm) ) 
		{
			pE = CRootEvent::CreateEventByType(type);
			pE->CopyData(buf);

			CActEvent aevent;
			aevent.count = count;
			aevent.frm = frm;
			aevent.name = pE->GetIdStr();
			aevent.rp = pE;
			m_eventList.push_back(aevent);

			found = false;
			CString aname = pE->GetIdStr();
			for(i=0; i<m_eventbyname.size(); i++){
				if(aname == m_eventbyname[i].name){
					found = true;
					seleted = i;
					break;
				}
			}
			if(!found){
				CEventByName abyname;
				abyname.name = pE->GetIdStr();
				abyname.m_name.push_back(count);
				m_eventbyname.push_back(abyname);
			}
			else{
				m_eventbyname[seleted].m_name.push_back(count);
			}

			found = false;
			ip = (int*) pE->GetData();
			for(i=0; i<m_eventbyid.size(); i++){
				if(*ip == m_eventbyid[i].id){
					found = true;
					seleted = i;
					break;
				}
			}
			if(!found){
				CEventById abyid;
				abyid.id = *ip;
				abyid.m_id.push_back(count);
				m_eventbyid.push_back(abyid);
			}
			else{
				m_eventbyid[seleted].m_id.push_back(count);
			}

			found = false;
			idp = m_id.begin();
			while(idp != m_id.end()){
				if(frm == *idp){
					found = true;
					break;
				}
				idp++;
			}
			if(!found){
				m_id.push_back(frm);
			}
			g_progress = count;
			count++;
		}
	}

	DisplayEvent();
//	fgDisplayEvent();
//	if ((m_fgList.get_TextMatrix(1,2)!="") && (m_fgRelate.get_TextMatrix(1,2)==""))
		DisplayRelatedEvent(1);

	g_LoadingFlag = false;
	EndWaitCursor();

	sprintf(text, "%d events shown", pLb->GetCount());
	SetDlgItemText(IDC_TITLE, text);
}


////////////////////////////////////////////////////////////////////////////////
//
// This function is called to display event in ListBox
//
//
void CActvLogReaderDlg::DisplayEvent()
{
	for(int i=0; i<m_eventbyname.size(); i++)
	{
		if(m_eventbyname[i].name == "CvedCreate")
		{
			m_event.AddString(m_eventbyname[i].name);
			for(int j=0; j<m_eventbyname[i].m_name.size(); j++)
			{
				DisplayOneEventRecord(m_eventbyname[i].m_name[j]);
			}
		}
		else
		{
			m_filtered.AddString(m_eventbyname[i].name);
		}
	}
}

void CActvLogReaderDlg::fgDisplayEvent()
{
	int vGrandCounter=0;
	for(int i=0; i<m_eventbyname.size(); i++)
	{
		if(m_eventbyname[i].name == "CvedCreate")
		{
			m_event.AddString(m_eventbyname[i].name);
			for(int j=0; j<m_eventbyname[i].m_name.size(); j++)
			{
				vGrandCounter=vGrandCounter+1;
				fgDisplayOneEventRecord(m_eventbyname[i].m_name[j],vGrandCounter);
			}
		}
		else
		{
			m_filtered.AddString(m_eventbyname[i].name);
		}
	}
}

CString CActvLogReaderDlg::GetLogRecord(int m)
{
	CRootEvent *pE= m_eventList[m].rp;
//	pE = m_eventList[m_eventbyname[i].m_name[j]].rp;

	istringstream in  (ios::in | ios::out);
    ostream       out (in.rdbuf());
	int a=m_eventList[m].frm;	////////?? vector

	pE->Print(m_eventList[m].frm, out);
	CString s=in.str().c_str();

	return s;
}


void CActvLogReaderDlg::fgDisplayOneEventRecord(int m, int counter)
{
	if (counter>100)
		AppendOneRow("List",counter);

	CString sCurrent=GetLogRecord(m);
	CString sPrev="";

	if (m>0)
	{
		sPrev=GetLogRecord(m-1);
	}

//	if (sCurrent!=sPrev)
	{
		float vTime=0.0;
		int vHcsmId=0,vCvedId=0,vCvedType=0,vType=0;
		CString sPos="",sName="";
		GetDisplayData(sCurrent, vTime,vHcsmId,vCvedId,vCvedType,sPos,sName,vType);

		fgDisplayEventList("IDC_LIST",m,counter,m_eventList[m].frm, m_eventList[m].name,
			vTime, vHcsmId, vCvedId, vCvedType, sPos,sName,vType,m_eventList[m].count);
	}
}

void CActvLogReaderDlg::AppendOneRow(CString sName,int vRowSeq)
{
/*
	VARIANT myVariant;
	myVariant.intVal=1;
	myVariant.vt=VT_I4;

	char c[10];
	sprintf(c,"%i",vRowSeq);
	CString vSeq=c;
	myVariant.intVal=vRowSeq;
	const VARIANT myCol=myVariant;

	if (sName=="List")
	{
		m_fgList.AddItem(vSeq,myCol);
	}
	else if (sName=="Relate")
	{
		m_fgRelate.AddItem(vSeq,myCol);
	}
*/
}
void CActvLogReaderDlg::DisplayOneEventRecord(int m)
{
//	pE = m_eventList[m_eventbyname[i].m_name[j]].rp;
	CRootEvent *pE= m_eventList[m].rp;

	istringstream in  (ios::in | ios::out);
    ostream       out (in.rdbuf());
	int a=m_eventList[m].frm;	////////?? vector

	pE->Print(m_eventList[m].frm, out);
	CString s=in.str().c_str();

	CString sDetail="";
	float vTime=0.0;
	int vHcsmId=0;//,vCvedId=0,vCvedType=0,vType=0;
	//CString sPos="",sName="";
	//GetDisplayData(s, vTime,vHcsmId,vCvedId,vCvedType,sPos,sName,vType);
	//DisplayEventList("IDC_LIST",m,m_eventList[m].frm, m_eventList[m].name,
	//					vTime, vHcsmId, vCvedId, vCvedType, sPos,sName,vType);

	GetDisplayData02(s, vTime,vHcsmId,sDetail);

	CString sName=GetName(vHcsmId);

	DisplayEventList02("IDC_LIST",m,m_eventList[m].frm, m_eventList[m].name,
						vTime, vHcsmId, sName,sDetail);
}

CString CActvLogReaderDlg::GetName(int vHcsmId)
{
	CRootEvent *pE;
	CString vName="";
	int *ip;

	for (int i=0;i<m_eventList.size();i++)
	{
		if (m_eventList[i].name=="CreateHcsm")
		{
			pE= m_eventList[i].rp;

			ip = (int *)pE->GetData();
			if ((*ip)==vHcsmId)
			{

				istringstream in  (ios::in | ios::out);
			    ostream       out (in.rdbuf());
				//int a=m_eventList[i].frm;	////////?? vector

				pE->Print(m_eventList[i].frm, out);
				CString s=in.str().c_str();

				int j=s.Find("'");
				vName=s.Mid(j+1,s.GetLength()-j);

				j=vName.Find("'");
				vName=vName.Mid(0,j);

				break;
			}
		}
	}

	return vName;
}

void CActvLogReaderDlg::DisplayEventList02(CString DisplayObject,int m,int frm, CString sEventType, float vTime, int vHcsmId, CString sName,CString sDetail)
{
//	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);

	char text[1000];
	int count=0;

	if (sEventType=="CreateHcsm")
		sDetail=RemoveNameFromDetail(sDetail,sName);

	sprintf(text, "%6d  %23s  %8.3f %4d%20s   %s", frm, sEventType, vTime, vHcsmId,  sName, sDetail);
			
	int vDisplayObject;
	if (DisplayObject=="IDC_LIST")
	{
		vDisplayObject=IDC_LIST;
		CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
		count = pLb->AddString(text);
		pLb->SetItemDataPtr(count, m_eventList[m].rp);
	}
	else if (DisplayObject=="IDC_LIST_relate")
	{
		vDisplayObject=IDC_LIST_relate;
		count = m_relate.AddString(text);
		m_relate.SetItemDataPtr(count,m_eventList[m].rp);
	}
}

CString CActvLogReaderDlg::RemoveNameFromDetail(CString sDetail,CString sName)
{
	CString s0="", s1="",s2="";
	int i=sDetail.Find(sName);
	if (i>0)
	{
		CString cap="Name=";

		i=sDetail.Find(cap);

		s1=sDetail.Mid(0,i);

		s0=sDetail.Mid(i+cap.GetLength(),sDetail.GetAllocLength()-i-cap.GetLength());

		i=s0.Find(sName);

		s2=s0.Mid(i+sName.GetLength()+1,s0.GetLength()-i-sName.GetLength());

		s0= s1+s2;
	}

	return s0;

}

void CActvLogReaderDlg::DisplayEventList
	(CString DisplayObject,int m,int frm, CString name, float vTime, 
	 int vHcsmId, int vCvedId, int vCvedType, CString sPos,CString sTriggerName,int vType)
{
//	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);

	char text[1000];
	int count=0;
	sprintf(text, "%6d   %23s    %2.3f %4d      %2d    %5d%36s   %2d%20s", 
		    frm, name, vTime, vHcsmId, vCvedId, vCvedType, sPos, vType, sTriggerName);
			
	int vDisplayObject;
	if (DisplayObject=="IDC_LIST")
	{
		vDisplayObject=IDC_LIST;
		CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
		count = pLb->AddString(text);
		pLb->SetItemDataPtr(count, m_eventList[m].rp);
	}
	else if (DisplayObject=="IDC_LIST_relate")
	{
		vDisplayObject=IDC_LIST_relate;
		count = m_relate.AddString(text);
		m_relate.SetItemDataPtr(count,m_eventList[m].rp);
	}
}

void CActvLogReaderDlg::fgDisplayEventList
	(CString DisplayObject,int m,int counter,int frm, CString name, float vTime, 
	 int vHcsmId, int vCvedId, int vCvedType, CString sPos,CString sTriggerName,int vType, int eSeq)
{
/*
	if (DisplayObject=="IDC_LIST")
	{
		m_fgList.put_TextMatrix(counter,1,ToCString(frm));
		m_fgList.put_TextMatrix(counter,2,name);
		m_fgList.put_TextMatrix(counter,3,ToCString(vTime));
		m_fgList.put_TextMatrix(counter,4,ToCString(vHcsmId));
		m_fgList.put_TextMatrix(counter,5,sTriggerName);
		m_fgList.put_TextMatrix(counter,6,ToCString(vCvedId));
		m_fgList.put_TextMatrix(counter,7,ToCString(vCvedType));
		m_fgList.put_TextMatrix(counter,8,sPos);
		m_fgList.put_TextMatrix(counter,9,ToCString(vType));
		m_fgList.put_TextMatrix(counter,10,ToCString(eSeq));
	}
	else if (DisplayObject=="IDC_LIST_relate")
	{
		m_fgRelate.put_TextMatrix(counter,1,ToCString(frm));
		m_fgRelate.put_TextMatrix(counter,2,name);
		m_fgRelate.put_TextMatrix(counter,3,ToCString(vTime));
		m_fgRelate.put_TextMatrix(counter,4,ToCString(vHcsmId));
		m_fgRelate.put_TextMatrix(counter,5,sTriggerName);
		m_fgRelate.put_TextMatrix(counter,6,ToCString(vCvedId));
		m_fgRelate.put_TextMatrix(counter,7,ToCString(vCvedType));
		m_fgRelate.put_TextMatrix(counter,8,sPos);
		m_fgRelate.put_TextMatrix(counter,9,ToCString(vType));
		m_fgRelate.put_TextMatrix(counter,10,ToCString(eSeq));
	}
*/
}

CString CActvLogReaderDlg::ToCString(int i)
{
	char text[100];
	sprintf(text, "%d", i);
	CString sText=text;
	return sText;
}
CString CActvLogReaderDlg::ToCString(float i)
{
	char text[100];
	sprintf(text, "%.3f", i);
	CString sText=text;
	return sText;
}
void CActvLogReaderDlg::fgRelateInit()
{
/*
	char c[10];
	int i;
	CString vSeq;
	VARIANT myVariant;
	myVariant.intVal=1;
	myVariant.vt=VT_I4;

	for (i=1;i<=99;i++)
	{
		sprintf(c,"%i",i);
		vSeq=c;
		myVariant.intVal=i;
		const VARIANT myCol=myVariant;
		m_fgRelate.AddItem(vSeq,myCol);
	}

	//init the last row
	int vRowsCounter=m_fgRelate.get_Rows();
	sprintf(c,"%i",i);
	vSeq=c;
	m_fgRelate.put_TextMatrix(i,0,vSeq);

	m_fgRelate.put_Cols(11);
	m_fgRelate.put_FixedCols(1);

	m_fgRelate.put_TextArray(0,"Seq");			//0:event display sequence
	m_fgRelate.put_TextArray(1,"Frame");		//1
	m_fgRelate.put_TextArray(2,"Event Nme");	//2:
	m_fgRelate.put_TextArray(3,"Time");			//3
	m_fgRelate.put_TextArray(4,"HcsmId");		//4
	m_fgRelate.put_TextArray(5,"Trigger Name");	//5
	m_fgRelate.put_TextArray(6,"CvedId");		//6
	m_fgRelate.put_TextArray(7,"CvedType");		//7
	m_fgRelate.put_TextArray(8,"Pos");			//8
	m_fgRelate.put_TextArray(9,"Type");			//9
	m_fgRelate.put_TextArray(10,"eSeq");		//10:event sequence in m_event

	m_fgRelate.put_ColAlignment(1,5);	//right alignment
	m_fgRelate.put_ColAlignment(2,5);	//right alignment
	m_fgRelate.put_ColAlignment(3,5);	//right alignment
	m_fgRelate.put_ColAlignment(4,5);	//right alignment
	m_fgRelate.put_ColAlignment(5,5);	//right alignment
	m_fgRelate.put_ColAlignment(6,5);	//right alignment
	m_fgRelate.put_ColAlignment(7,5);	//right alignment
	m_fgRelate.put_ColAlignment(8,5);	//right alignment
	m_fgRelate.put_ColAlignment(9,5);	//right alignment

	m_fgRelate.put_ColWidth(0,200);
	m_fgRelate.put_ColWidth(1,1000);
	m_fgRelate.put_ColWidth(2,2000);	//event name
	m_fgRelate.put_ColWidth(3,1000);	//Time
	m_fgRelate.put_ColWidth(4,800);		//HcsmId
	m_fgRelate.put_ColWidth(5,2000);	//trigger name
	m_fgRelate.put_ColWidth(6,800);		//CvedId
	m_fgRelate.put_ColWidth(7,1000);		//CvedType
	m_fgRelate.put_ColWidth(8,3000);	//Pos
	m_fgRelate.put_ColWidth(9,800);		//Type
	m_fgRelate.put_ColWidth(10,0);		//

	m_fgRelate.put_GridLines(1);
//	m_fgRelate.SetColPosition(1,2);  //set column sequence
	m_fgRelate.put_Sort(1);  //sort on column:1 (0:n)

	m_fgRelate.put_AllowUserResizing(1);	//using resizing column width
//	m_fgRelate.SetAllowBigSelection(1);
//	m_fgRelate.put_CellAlignment(5);
//	m_fgRelate.put_FixedAlignment(0,5);
*/
}

void CActvLogReaderDlg::fgListInit()
{
/*
	char c[10];
	int i;
	CString vSeq;
	VARIANT myVariant;
	myVariant.intVal=1;
	myVariant.vt=VT_I4;

	for (i=1;i<=99;i++)
	{
		sprintf(c,"%i",i);
		vSeq=c;
		myVariant.intVal=i;
		const VARIANT myCol=myVariant;
		m_fgList.AddItem(vSeq,myCol);
	}

	//init the last row
	int vRowsCounter=m_fgList.get_Rows();
	sprintf(c,"%i",i);
	vSeq=c;
	m_fgList.put_TextMatrix(i,0,vSeq);

	m_fgList.put_Cols(11);
	m_fgList.put_FixedCols(1);

	m_fgList.put_TextArray(0,"Seq");			//0: event display sequence
	m_fgList.put_TextArray(1,"Frame");			// 1
	m_fgList.put_TextArray(2,"Event Nme");		// 2
	m_fgList.put_TextArray(3,"Time");			//3
	m_fgList.put_TextArray(4,"HcsmId");			//4
	m_fgList.put_TextArray(5,"Trigger Name");	//5
	m_fgList.put_TextArray(6,"CvedId");			//6
	m_fgList.put_TextArray(7,"CvedType");		//7
	m_fgList.put_TextArray(8,"Pos");			//8
	m_fgList.put_TextArray(9,"Type");			//9
	m_fgList.put_TextArray(10,"eSeq");			//10: event sequence in m_event

	m_fgList.put_ColAlignment(1,5);	//right alignment
	m_fgList.put_ColAlignment(2,5);	//right alignment
	m_fgList.put_ColAlignment(3,5);	//right alignment
	m_fgList.put_ColAlignment(4,5);	//right alignment
	m_fgList.put_ColAlignment(5,5);	//right alignment
	m_fgList.put_ColAlignment(6,5);	//right alignment
	m_fgList.put_ColAlignment(7,5);	//right alignment
	m_fgList.put_ColAlignment(8,5);	//right alignment
	m_fgList.put_ColAlignment(9,5);	//right alignment

	m_fgList.put_ColWidth(0,200);
	m_fgList.put_ColWidth(1,1000);
	m_fgList.put_ColWidth(2,2000);  //event name
	m_fgList.put_ColWidth(3,1000);	//Time
	m_fgList.put_ColWidth(4,800);	//HcsmId
	m_fgList.put_ColWidth(5,2000);	//trigger name
	m_fgList.put_ColWidth(6,800);	//CvedId
	m_fgList.put_ColWidth(7,1000);	//CvedType
	m_fgList.put_ColWidth(8,3000);	//Pos
	m_fgList.put_ColWidth(9,800);	//Type
	m_fgList.put_ColWidth(10,0);	//

	m_fgList.put_GridLines(1);
//	m_fgList.SetColPosition(1,2);  //set column sequence
	m_fgList.put_Sort(1);  //sort on column:1 (0:n)

	m_fgList.put_AllowUserResizing(1);	//using resizing column width
//	m_fgList.SetAllowBigSelection(1);
//	m_fgList.put_CellAlignment(5);
//	m_fgList.put_FixedAlignment(0,5);
*/
}


/*sample by Dave
				stringstream sstemp(sPos.GetString());
				float p1,p2,p3;
				char t1,t2,t3;
				sstemp>>p1>>t1>>p2>>t2>>p3;
*/

//CString s=pE->GetData();
//int *ip = (int*) pE->GetData();

void CActvLogReaderDlg::GetDisplayData(CString s,float& vTime,int& vHcsmId,int& vCvedId,int& vCvedType,CString &sPos, CString &vName,int &vType)
{
	//int vFrame=GetValue(s, "Frame");
	vTime=GetFloat(s, "time");
	vHcsmId=GetValue(s, "HcsmId");
	vCvedId=GetValue(s, "CvedId");
	vCvedType=GetValue(s, "CvedType");
	vType=GetValue(s, " Type");
	vName=GetString(s, "Name").Trim(" ").Trim("'");

	sPos=GetString(s, "(", ")");
}

void CActvLogReaderDlg::GetDisplayData02(CString s,float& vTime,int& vHcsmId,CString &sDetail)
{
	//int vFrame=GetValue(s, "Frame");
	vTime=GetFloat(s, "time");
	vTime=vTime*8.0;	//previous frequency is 240, now we use 30
	vHcsmId=GetValue(s, "HcsmId");
//	vName = GetString (s,
	sDetail=GetDetail(s);
}

CString CActvLogReaderDlg::GetDetail(CString s)
{
	CString s0="HcsmId =";
	int i=s.Find(s0);
	s=s.Mid(i+s0.GetLength(),s.GetLength()-i-s0.GetLength()-1);
	s=s.Trim(' ');
	i=s.FindOneOf("\n, ");	//get rid of the first number
	if (i>0)
	{
		s=s.Mid(i+1,s.GetLength()-i-1);
		s=s.Trim(' ');
		s.Replace("\n"," ");
		s.Replace("= ","=");
		s.Replace(" =","=");
		s.Replace("  "," ");
	}
	else	//empty string
	{
		s="";
	}
	return s;
}


CString CActvLogReaderDlg::GetString(CString s, CString sBegin)
{
	CString ss="";
	int b=s.Find(sBegin);
	if (b>=0)
	{
		s=s.Mid(b+sBegin.GetLength(),s.GetLength()-b-sBegin.GetLength());

		CString sSpace="=";
		b=s.Find(sSpace);
		s=s.Mid(b+sSpace.GetLength(),s.GetLength()-b-sSpace.GetLength());

		int c=s.FindOneOf("),\n");

		ss=s.Mid(0,c);
	}

	return ss;
}

CString CActvLogReaderDlg::GetString(CString s, CString sBegin, CString sEnd)
{
	int b=s.Find(sBegin);
	int c=s.Find(sEnd);
	CString ss=s.Mid(b+sBegin.GetLength()-1,c-b-sBegin.GetLength()+2);

	return ss;
}


int CActvLogReaderDlg::GetValue(CString s, CString sBegin)
{
	CString ss=GetString(s,sBegin);
	int vResult=atoi(ss);
	return vResult;
}

float CActvLogReaderDlg::GetFloat(CString s, CString sBegin)
{
	CString ss=GetString(s,sBegin);
	float vResult=atof(ss);
	return vResult;
}


////////////////////////////////////////////////////////////////////////////////
//
// This function is called each time the user clicks on an entry in the
// list box.
//
//
void CActvLogReaderDlg::OnLbnSelchangeList()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);

	int sel = pLb->GetCurSel();
	if ( sel == LB_ERR ) {
		SetDlgItemText(IDC_TEXT, "");
		return;
	}

	CRootEvent *pE;
	pE = (CRootEvent *)pLb->GetItemDataPtr(sel);

	CString str;
    istringstream in  (ios::in | ios::out);
    ostream       out (in.rdbuf());

	pLb->GetText(sel, str);
	pE->Print(atoi(str), out);

	CString myString=in.str().c_str();

	SetDlgItemText(IDC_TEXT, in.str().c_str());
	m_relate.ResetContent();

	int *pid;
	int index;
	pid = (int *) pE->GetData();
	for(int i=0; i<m_eventbyid.size(); i++)
	{
		if(*pid == m_eventbyid[i].id)
		{
			int c=m_eventbyid[i].m_id.size();
			for(int j=0; j<m_eventbyid[i].m_id.size(); j++)
			{

				pE = m_eventList[m_eventbyid[i].m_id[j]].rp;

				CString str;
			    istringstream in  (ios::in | ios::out);
			    ostream       out (in.rdbuf());

				pE->Print(atoi(str), out);

				CString s=in.str().c_str();

				float vTime=0.0;
				int vHcsmId=0; //int vCvedId=0,vCvedType=0,vType=0;
				CString sDetail;
				//CString sPos="", sName="";
				//GetDisplayData(s, vTime,vHcsmId,vCvedId,vCvedType,sPos,sName,vType);
				GetDisplayData02(s, vTime,vHcsmId,sDetail);	//

				CString sName=GetName(vHcsmId);
				vTime=1.0*m_eventList[m_eventbyid[i].m_id[j]].frm/240.0;			//	float vTime=GetFloat(s, "time");

				int m=m_eventbyid[i].m_id[j];
				//DisplayEventList("IDC_LIST_relate",m,m_eventList[m].frm, m_eventList[m].name,
				//		vTime, vHcsmId, vCvedId, vCvedType, sPos,sName,vType);
				DisplayEventList02("IDC_LIST_relate",m,m_eventList[m].frm, m_eventList[m].name,
						vTime, vHcsmId, sName,sDetail);
			}
			break;
		}
	}
}


void CActvLogReaderDlg::OnBnClickedButtonname()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	int max=pLb->GetCount();
	if (max>0)
	{
		SortByName();
	}
}

void CActvLogReaderDlg::SortByName()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);

	m_type = 2;
		BeginWaitCursor();
		pLb->ResetContent();
		g_LoadingFlag = true;
		if(m_eventList.size() > 10000)
			g_total = m_eventList.size() / 100;
		else
			g_total = m_eventList.size();
		g_progress = 0;
		_beginthread(CLoadStatus, 0, NULL);
		char text[64];
		int i, j, index;
		bool found = false;

		for(j=0; j<m_event.GetCount(); j++)
		{
			CString tmp;
			m_event.GetText(j,tmp);

			for(i=0; i<m_eventbyname.size(); i++)
			{
				if(m_eventbyname[i].name == tmp)
				{
					for(int k=0; k<m_eventbyname[i].m_name.size(); k++)
					{
						DisplayOneEventRecord(m_eventbyname[i].m_name[k]);
					}
				}
				if(m_eventList.size() > 10000)
					g_progress = (j * m_eventList.size() + i) / m_event.GetCount() / 100;
				else
					g_progress = (j * m_eventList.size() + i) / m_event.GetCount();
			}
		}
		g_LoadingFlag = false;
		EndWaitCursor();
		m_relate.ResetContent();
	}
/*
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);

	m_type = 2;
	BeginWaitCursor();
	pLb->ResetContent();
	g_LoadingFlag = true;
	if(m_eventList.size() > 10000)
		g_total = m_eventList.size() / 100;
	else
		g_total = m_eventList.size();
	g_progress = 0;
	_beginthread(CLoadStatus, 0, NULL);
	char text[64];
	int i, j, index,vGrandCounter=0;
	bool found = false;

	for(j=0; j<m_event.GetCount(); j++)
	{
		CString tmp;
		m_event.GetText(j,tmp);

		for(i=0; i<m_eventbyname.size(); i++)
		{
			if(m_eventbyname[i].name == tmp)
			{
				for(int k=0; k<m_eventbyname[i].m_name.size(); k++)
				{
					vGrandCounter++;
					DisplayOneEventRecord(m_eventbyname[i].m_name[k]);
					//fgDisplayOneEventRecord(m_eventbyname[i].m_name[k],vGrandCounter);
				}
				if(m_eventList.size() > 10000)
					g_progress = (j * m_eventList.size() + i) / m_event.GetCount() / 100;
				else
					g_progress = (j * m_eventList.size() + i) / m_event.GetCount();
			}
		}
	}

	g_LoadingFlag = false;
	EndWaitCursor();
*/

//reset all display for loading different data from text file 
void CActvLogReaderDlg::ResetDisplay()
{
	m_event.ResetContent();
	m_relate.ResetContent();
	m_filtered.ResetContent();

	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	pLb->ResetContent();
	//fgListInit();
	//fgRelateInit();
}

//clear content for display by different ordering (same data)
void CActvLogReaderDlg::fgListResetContent()
{
/*
	m_fgList.Clear();

	fgRelateInit();
	fgListInit();
*/
}



void gOnBnClickedButtonid()
{
/*
	fgListResetContent();

	m_type = 1;
	BeginWaitCursor();
	m_id.sort();
	g_LoadingFlag = true;
	if(m_eventList.size() > 10000)
		g_total = m_eventList.size() / 100;
	else
		g_total = m_eventList.size();
	g_progress = 0;
	_beginthread(CLoadStatus, 0, NULL);
	int j, count;
	int vGrandCounter=0;

	count = 0;

	for(int i=0; i<m_eventList.size(); i++)
	{
		vGrandCounter++;
		fgDisplayOneEventRecord(i,vGrandCounter);

		if(m_eventList.size() > 10000)
			g_progress = (count * m_eventList.size() + i) / m_id.size() / 100;
		else
			g_progress = (count * m_eventList.size() + i) / m_id.size();
	}

	g_LoadingFlag = false;
	EndWaitCursor();
*/
}

void CActvLogReaderDlg::OnBnClickedButtonid()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	int max=pLb->GetCount();
	if (max>0)
	{
		SortByFrameId();
	}
}

void CActvLogReaderDlg::SortByFrameId()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);

	m_type = 1;
		BeginWaitCursor();
		pLb->ResetContent();
		m_id.sort();
		g_LoadingFlag = true;
		if(m_eventList.size() > 10000)
			g_total = m_eventList.size() / 100;
		else
			g_total = m_eventList.size();
		g_progress = 0;
		_beginthread(CLoadStatus, 0, NULL);
		char text[64];
		int i, j, index, count;
		list<int>::iterator idp;

		idp = m_id.begin();
		count = 0;
		while(idp != m_id.end())
		{
			for(i=0; i<m_eventList.size(); i++)
			{
				if(m_eventList[i].frm == *idp)
				{
					for(j=0; j<m_event.GetCount(); j++)
					{
						CString tmp;
						m_event.GetText(j,tmp);
						if(m_eventList[i].name == tmp)
						{
							DisplayOneEventRecord(i);

							if(m_eventList.size() > 10000)
								g_progress = (count * m_eventList.size() + i) / m_id.size() / 100;
							else
								g_progress = (count * m_eventList.size() + i) / m_id.size();
	
							break;
						}
					}
				}
			}
			count++;
			idp++;
		}

		g_LoadingFlag = false;
		EndWaitCursor();

		m_relate.ResetContent();

//	CStatic* pText = (CStatic *)GetDlgItem(IDC_STATIC_FRAME);

	
}


void CActvLogReaderDlg::OnBnClickedButtonfilter()
{
	// TODO: Add your control notification handler code here
	if(m_event.GetCurSel() < 0){
		AfxMessageBox("Please select an item from unfiltrated events.");
	}
	else 
	{
		RemoveFromEventList();
	}
}

void CActvLogReaderDlg::RemoveFromEventList()
{
	char text[64];
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	CString del;
	m_event.GetText(m_event.GetCurSel(), del);
	m_event.DeleteString(m_event.GetCurSel());
	m_filtered.AddString(del);

	int m=pLb->GetCount();
	if (m>0)
	{
	BeginWaitCursor();
	g_LoadingFlag = true;
	g_total = m;
	g_progress = 0;
	_beginthread(CLoadStatus, 0, NULL);

	int index = 0;
	CString tmp;
	pLb->GetText(index, tmp);
	while(tmp.GetLength() > 0)
	{
		if(tmp.Find(del) != -1)
		{
			pLb->DeleteString(index);
		}
		else
		{
			index++;
		}
		if(index >= pLb->GetCount())
			break;
		pLb->GetText(index, tmp);
		g_total = pLb->GetCount();
		g_progress = index;
	}

	g_LoadingFlag = false;
	EndWaitCursor();
	}

	m_relate.ResetContent();
//	sprintf(text, "%d events shown", pLb->GetCount());
//	SetDlgItemText(IDC_TITLE, text);
}

void CActvLogReaderDlg::fgRemoveFromEventList()
{
/*
	char text[64];
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	CString del, vName;
	m_event.GetText(m_event.GetCurSel(), del);
	m_event.DeleteString(m_event.GetCurSel());
	m_filtered.AddString(del);

	BeginWaitCursor();
	g_LoadingFlag = true;
	g_total = pLb->GetCount();
	g_progress = 0;
	_beginthread(CLoadStatus, 0, NULL);

	g_total = m_fgList.get_Rows();
	int index=0;
	for (int i=(m_fgList.get_Rows()-1);i>0;i--)
	{
		vName=m_fgList.get_TextMatrix(i,2);
		if(vName == del)
		{
			m_fgList.RemoveItem(i);
		}
		else
		{
			index++;
		}
		g_progress = index;
	}

	g_LoadingFlag = false;
	EndWaitCursor();

	sprintf(text, "%d events shown", pLb->GetCount());
	SetDlgItemText(IDC_TITLE, text);
*/
}


void CActvLogReaderDlg::OnBnClickedButtonunfilter()
{
	// TODO: Add your control notification handler code here
	if(m_filtered.GetCurSel() < 0)
	{
		AfxMessageBox("Please select an item from filtrated events.");
	}
	else
	{
		AddToEventList();
	}
}

void CActvLogReaderDlg::AddToEventList()
{
	CString del;
	m_filtered.GetText(m_filtered.GetCurSel(), del);
	m_filtered.DeleteString(m_filtered.GetCurSel());
	m_event.AddString(del);

//	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
//	char text[64];

	SortByName();

	m_relate.ResetContent();

/*
	if(m_type == 0)
	{
		BeginWaitCursor();
		pLb->ResetContent();
		g_LoadingFlag = true;
		g_total = m_eventList.size() / 100;
		g_progress = 0;
		_beginthread(CLoadStatus, 0, NULL);
		int i, j, index, count;
		bool found = false;
		for(i=0; i<m_eventList.size(); i++)
		{
			found = false;
			for(j=0; j<m_event.GetCount(); j++)
			{
				CString tmp;
				m_event.GetText(j,tmp);
				if(m_eventList[i].name == tmp)
				{
					found = true;
					break;
				}
			}
			if(found)
			{
				DisplayOneEventRecord(i);
				g_progress = i / 100;
			}
		}
		g_LoadingFlag = false;
		EndWaitCursor();
	}
	else if(m_type == 1)
	{
		OnBnClickedButtonid();
	}
	else if(m_type == 2)
	{
		SortByName();
	}
*/
//	sprintf(text, "%d events shown", pLb->GetCount());
//	SetDlgItemText(IDC_TITLE, text);
}

void CActvLogReaderDlg::OnLbnSelchangeListrelate()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST_relate);

	int sel = pLb->GetCurSel();
	if ( sel == LB_ERR ) {
		SetDlgItemText(IDC_Text_Bottom, "");
		return;
	}

	CRootEvent *pE;
	pE = (CRootEvent *)pLb->GetItemDataPtr(sel);

	CString str;
    istringstream in  (ios::in | ios::out);
    ostream       out (in.rdbuf());

	pLb->GetText(sel, str);
	pE->Print(atoi(str), out);
	CString m=in.str().c_str();
	SetDlgItemText(IDC_Text_Bottom, in.str().c_str());
}

void CActvLogReaderDlg::OnLbnDblclkListfilted()
{
	AddToEventList();
}

void CActvLogReaderDlg::OnLbnDblclkListevent()
{
//	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
//	int max=pLb->GetCount();
//	if (max>0)
//	{
		RemoveFromEventList();
//	}
	//m_fgRelate.Clear();
}

BEGIN_EVENTSINK_MAP(CActvLogReaderDlg, CDialog)
	ON_EVENT(CActvLogReaderDlg, IDC_MSFLEXGRID_LIST, 70, CActvLogReaderDlg::RowColChangeMsflexgridList, VTS_NONE)
END_EVENTSINK_MAP()

void CActvLogReaderDlg::RowColChangeMsflexgridList()
{
/*
	int a=m_fgList.get_RowSel();

	DisplayRelatedEvent(a);
*/
}

void CActvLogReaderDlg::DisplayRelatedEvent(int a)
{
/*
	int counter=0;
	CString sHcsmId=m_fgList.get_TextMatrix(a,4);
	int vHcsmId=atoi(sHcsmId);

	ClearDisplay("Relate");

	for(int i=0; i<m_eventbyid.size(); i++)
	{
		if(vHcsmId == m_eventbyid[i].id)
		{
			int c=m_eventbyid[i].m_id.size();
			for(int j=0; j<m_eventbyid[i].m_id.size(); j++)
			{
				CRootEvent *pE = m_eventList[m_eventbyid[i].m_id[j]].rp;

				CString str;
			    istringstream in  (ios::in | ios::out);
			    ostream       out (in.rdbuf());

				pE->Print(atoi(str), out);

				CString s=in.str().c_str();

				float vTime=0.0;
				int vHcsmId=0,vCvedId=0,vCvedType=0,vType=0;
				CString sPos="", sName="";
				GetDisplayData(s, vTime,vHcsmId,vCvedId,vCvedType,sPos,sName,vType);

				vTime=1.0*m_eventList[m_eventbyid[i].m_id[j]].frm/240.0;			//	float vTime=GetFloat(s, "time");

				int m=m_eventbyid[i].m_id[j];
				counter++;
			
				fgDisplayEventList("IDC_LIST_relate",m,counter,m_eventList[m].frm, m_eventList[m].name,
					vTime, vHcsmId, vCvedId, vCvedType, sPos,sName,vType,m_eventList[m].count);

			}
			break;
		}
	}
*/
}

void CActvLogReaderDlg::ClearDisplay(CString vTableName)
{
/*
	if (vTableName=="List")
	{
		m_fgList.Clear();
		fgListInit();
	}
	else if (vTableName=="Relate")
	{
		m_fgRelate.Clear();
		fgRelateInit();
	}
*/
}

void CActvLogReaderDlg::OnBnClickedCheckHideInstantiate()
{
	
}

void CActvLogReaderDlg::OnBnClickedCheckHideDuplicate()
{
	int i=m_cbDuplicateRecord.GetCheck();

	if (i)
	{
		listHideDuplicateRecord();
		relateHideDuplicateRecord();
	}
	else
	{
		listDisplayDuplicateRecord();
		relateDisplayDuplicateRecord();
	}
}

void CActvLogReaderDlg::listHideDuplicateRecord()
{
/*
	int maxRow=m_fgList.get_Rows();
	maxRow--;

	for (int i=maxRow;i>0;i--)
	{
		if (m_fgList.get_TextMatrix(i,1) != "")	//skip empty rows
		{
			int vFrame01=atoi(m_fgList.get_TextMatrix(i,1));			// 1
			CString vEventName01=m_fgList.get_TextMatrix(i,2);	// 2
			float vTime01=atof(m_fgList.get_TextMatrix(i,3));			//3
			int vHcsmId01=atoi(m_fgList.get_TextMatrix(i,4));			//4
			CString vTriggerName01=m_fgList.get_TextMatrix(i,5);	//5
			int vCvedId01=atoi(m_fgList.get_TextMatrix(i,6));			//6
			int vCvedType01=atoi(m_fgList.get_TextMatrix(i,7));		//7
			CString vPos01=m_fgList.get_TextMatrix(i,8);			//8
			int vType01=atoi(m_fgList.get_TextMatrix(i,9));			//9

			if (i>1)
			{
				int vFrame02=atoi(m_fgList.get_TextMatrix(i-1,1));			// 1
				CString vEventName02=m_fgList.get_TextMatrix(i-1,2);	// 2
				float vTime02=atof(m_fgList.get_TextMatrix(i-1,3));			//3
				int vHcsmId02=atoi(m_fgList.get_TextMatrix(i-1,4));			//4
				CString vTriggerName02=m_fgList.get_TextMatrix(i-1,5);	//5
				int vCvedId02=atoi(m_fgList.get_TextMatrix(i-1,6));			//6
				int vCvedType02=atoi(m_fgList.get_TextMatrix(i-1,7));		//7
				CString vPos02=m_fgList.get_TextMatrix(i-1,8);			//8
				int vType02=atoi(m_fgList.get_TextMatrix(i-1,9));			//9

				if ((vFrame01 == vFrame02) && (vEventName01==vEventName02) &&
					(abs(vTime01-vTime02)<0.000001) && (vHcsmId01 == vHcsmId02) &&
					(vTriggerName01==vTriggerName02) && (vCvedId01==vCvedId02) &&
					(vCvedType01 ==vCvedType02) && (vPos01==vPos02) &&
					(vType01==vType02))
				{
					m_fgList.RemoveItem(i);
				}
			}
		}
	}
*/
}

void CActvLogReaderDlg::relateHideDuplicateRecord()
{
}

void CActvLogReaderDlg::listDisplayDuplicateRecord()
{
}

void CActvLogReaderDlg::relateDisplayDuplicateRecord()
{
}

void CActvLogReaderDlg::OnBnClickedButtonHcsmid()
{
	OnBnClickedButtonid();	//reload event list, otherwise it will sort on current remaining data

	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	CListBox* pLbDisplay = (CListBox *)GetDlgItem(IDC_LIST_DISPLAY);
	int max=pLb->GetCount();

	if (max>0)
	{
		m_type = 0;		//30 for Order by HcsmId

		BeginWaitCursor();

		g_LoadingFlag = true;
		if(m_eventList.size() > 10000)
			g_total = m_eventList.size() / 100;
		else
			g_total = m_eventList.size();
		g_progress = 0;
		_beginthread(CLoadStatus, 0, NULL);

		int count = 0,k=0;
		CString sSource="",sDest="";
		int vHcsmIdSou=0,vHcsmIdDest=0;
		bool done=false;
		CRootEvent *pE;
	
		for (int i=0;i<max;i++)
		{
			pLb->GetText(i,sSource);

			pE=(CRootEvent*)pLb->GetItemDataPtr(i);

			vHcsmIdSou=atoi(GetString(sSource, 4));
	
			if (pLbDisplay->GetCount()==0)
			{
				pLbDisplay->InsertString(0,sSource);
				pLbDisplay->SetItemDataPtr(0,pE);
			}
			else
			{
				for (int j=0;j<pLbDisplay->GetCount();j++)
				{
					pLbDisplay->GetText(j,sDest);
					vHcsmIdDest=atoi(GetString((sDest),4));

					if (vHcsmIdSou<=vHcsmIdDest)
					{
						k=pLbDisplay->InsertString(j,sSource);
						pLbDisplay->SetItemDataPtr(k,pE);
						done = true;
						break;
					}
				}
				if (!done)
				{
					k=pLbDisplay->AddString(sSource);	//add to last
					pLbDisplay->SetItemDataPtr(k,pE);
				}
			}
			done=false;

			if(m_eventList.size() > 10000)
				g_progress = (count * m_eventList.size() + i) / m_id.size() / 100;
			else	
				g_progress = (count * m_eventList.size() + i) / m_id.size();

			count++;
		}

		g_LoadingFlag = false;
		EndWaitCursor();

		RefreshListDisplay();
//	RefreshListDisplay(OrderByHcsmId,eOrderByHcsmId);
		m_relate.ResetContent();
	}
}
void CActvLogReaderDlg::RefreshListDisplay()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	CListBox* pLbDisplay = (CListBox *)GetDlgItem(IDC_LIST_DISPLAY);

	int count=0;
	int max=pLb->GetCount();
	CString sSource="";
	CRootEvent *pE;
	pLb->ResetContent();

	for (int i=0;i<max;i++)
	{
		pLbDisplay->GetText(i,sSource);
		pE=(CRootEvent*)pLbDisplay->GetItemDataPtr(i);

//		count=pLb->InsertString(i,sSource);
		count=pLb->AddString(sSource);
		pLb->SetItemDataPtr(count,pE);
	}
	pLbDisplay->ResetContent();
}

void CActvLogReaderDlg::RefreshListDisplay(vector<CString> &OrderByHcsmId)
{
	vector<CString>::iterator iterDest = OrderByHcsmId.begin();
//	vector<CRootEvent>::iterator eiterDest = eOrderByHcsmId.begin();

	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	pLb->ResetContent();
	int i=0;
	while(iterDest != OrderByHcsmId.end())
	{
		pLb->InsertString(i,(*iterDest));
//		pLb->SetItemDataPtr(i, (&eiterDest));


//		pLb->AddString(*iterDest);
		iterDest++;
//		eiterDest++;
		i++;
	}
}

CString CActvLogReaderDlg::GetString(CString &s, int m)
{
	CString a="-999";
	int vPos=0;
	CString ss=s;

	for (int i=1;i<=m;i++)
	{
		ss=ss.Trim(' ');
		vPos=ss.Find(" ");
		if ((vPos==-1) && (ss.GetLength()>0))
		{
			a=ss;
		}
		else
		{
			a=ss.Mid(0,vPos);
			ss=ss.Mid(vPos,ss.GetLength()-vPos);
		}
	}
	return a;
}


int CActvLogReaderDlg::GetValue(CString s, int m)
{
	int a=0;
	for (int i=0;i<m;i++)
	{
		a=GetFirstValue(s);
	}
	return a;
}

int CActvLogReaderDlg::GetFirstValue(CString &s)
{
	int k=-999;
	s=s.Trim(' ');
	int vPos=s.Find(" ");
	if ((vPos==-1) && (s.GetLength()>0))
	{
		k=atoi(s);
	}
	else
	{
		CString s1=s.Mid(0,vPos);
		s=s.Mid(vPos,s.GetLength()-vPos);
		k=atoi(s1);
	}

	return k;
}

void CActvLogReaderDlg::OnLbnDblclkList()
{
	CListBox* pLb = (CListBox *)GetDlgItem(IDC_LIST);
	CString s="";
	pLb->GetText(pLb->GetCurSel(),s);
	CString sHcsmId00=GetHcsmId(s);
	CString sHcsmId="";

	int m=pLb->GetCount();
	for (int i=m-1;i>=0;i--)
	{
		pLb->GetText(i,s);
		sHcsmId=GetHcsmId(s);

		if (sHcsmId==sHcsmId00)
		{
			pLb->DeleteString(i);
/*
			ASSERT(lpDeleteItemStruct->CtlType == ODT_LISTBOX);
			LPVOID lpszText = (LPVOID) lpDeleteItemStruct->itemData;
			ASSERT(lpszText != NULL);

			free(lpszText);

			//CListBox::DeleteItem(lpDeleteItemStruct);

			pLb->DeleteItem(lpDeleteItemStruct);
*/
		}
	}
	m_relate.ResetContent();
}

CString CActvLogReaderDlg::GetHcsmId(CString s)
{
	s=s.Trim(' ');
	//get the 4 column, which is HcsmId
	int i=s.Find(".");
	s=s.Mid(i,s.GetLength()-i);

	i=s.Find(" ");
	s=s.Mid(i,s.GetLength()-i);

	s=s.Trim(' ');

	i=s.Find(" ");
	s=s.Mid(0,i);

	return s;
}

void CActvLogReaderDlg::OnBnClickedButtonrefresh()
{
	SortByName();
}
