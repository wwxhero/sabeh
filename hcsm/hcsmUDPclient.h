//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmUDPclient.h,v 1.1 2018/05/17 16:27:08 IOWA\dheitbri Exp $
//
// Author(s):    Yiannis Papelis
//
// Date:         August, 1999
// Description:  Declaration of the hcsmclient library class.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef _HCSMUDPCLIENT_H_
#define _HCSMUDPCLIENT_H_

#include <cvedpub.h>
#ifdef _WIN32
#include <winhrt.h>
#endif
#include "hcsmcollection.h"
#include "hcsmconnect.h"

#ifdef _PowerMAXOS
#include <unistd.h>
#endif
class CUdpThread;
class CUdpSender;
/////////////////////////////////////////////////////////////////////////////
/// This class encapsulates the functionality of a client of the hcsm system. 
/// It allows programs that use this class to gain remote access to the
/// hcsm system and control its execution, or simply obtain information
/// about the virtual environment, while the hcsm system is running.  In
/// addition, the class provides facilities that allow remote control of
/// certain entities within the virtual environment.
///
/// The class operates in two modes, sycnhronous and free running.  In
/// synchronous mode, the class controls the pace of execution of
/// the hcsm system and requires the hcsmexec program to actually run
/// the behaviors.  In free running mode, the hcsm system should be running
/// at some computer that has network access.  In both cases, the library 
/// requires that the GatewayHCSM is running in the HCSM system.
/// Naturally, the library is tightly integrated with hcsmexec and the
/// GatewayHCSM.
///\ingroup HCSM
/////////////////////////////////////////////////////////////////////////////
class CHcsmUdpClient {
public:

	CHcsmUdpClient();
	~CHcsmUdpClient();

	enum TMode { EDynamicObjects =0 };
	enum TErrorCode {
		ENoError,
		EModeError,				// mode is incorrect for requested op
		EModeSocketComm,		// socket related comm error
		EModeExecProcError,		// cannot start a process
		EError
	};

	CHcsmUdpClient(TMode mode) : m_Mode(mode), m_ErrorCode(ENoError) {};
    void Tick();
private:
	CHcsmUdpClient(const CHcsmUdpClient &);				// no user access
	CHcsmUdpClient &operator=(const CHcsmUdpClient &);	// no user access

	///////////////////////////////////
	//
	// public data structures and types
	//
	///////////////////////////////////

public:
	// Debug mode options
	enum TDebugMode { 
		ENoDebug,				// No debugging information generated
		ETextDebug,				// The HCSM should provide text messages
		EGraphicsDebug,			// The HCSM should provide graphics commands
		EGraphicsTextDebug,		// The HCSM should provide both of the above
	};


	//
	// This structure is used by the HCSMs to specify some simple drawing
	// commands to be implemented by the ISAT.
	//
	struct TDrawCmd {
		int type;
		double p1, p2, p3, p4;
		int   color;
		short fill;
	};

	//
	// This structure contains remote control commands that travel
	// from a human user to one of the HCSMs that has been placed
	// in remote control mode.  Commands are meant to be high level, not
	// interactive, as there is no guarantee on the timeliness of delivery.
	//
	enum ERemCntrlCmd {
		eTURN_LEFT,
		eTURN_RIGHT,
		eTURN_STRAIGHT,
		eCHANGE_LANES_LEFT,
		eCHANGE_LANES_RIGHT,
		eFORCE_VEL,
		eMAX_VEL
	};

	struct TRemCntrlCmd {
		ERemCntrlCmd	command;
		int				val;
	};

public:
	bool SetMode(TMode);

	bool InitFreeRunning(const string &, int);
	bool Quit(void);

	void GetLastError(string &, TErrorCode &);
	bool GetLriFileName(string &);
	bool SetDebugMode(TDebugMode, const vector<int> &, int level = 1);

	bool GetDynamicObjs(vector<TObjDesc> &);
	bool GetDynamicObjs(TObjDescSet &);
	bool GetChangedStaticObjs(vector<TObjDesc> &);
	bool GetChangedStaticObjs(TObjDescSet &);
	bool GetInstancedObjs(vector<TObjDesc> &);
	bool GetDebugData(vector<CHcsmDebugItem> &);

	bool TakeControlOfObj(int, const TRemCntrlCmd &);
	bool ReleaseControlOfObj(int);
	bool ControlObj(int, const TRemCntrlCmd &);

private:
    void DecodeMessage(const TMessage &msg);
	// library state
	enum TState {
		EDorm, EReady, ERunning, EStopped
	};

	bool ConnectToDataSock(void);

	TMode			m_Mode;			// library mode
	TState          m_State;		// internal library state, see doc
	string			m_ErrorMsg;		// most recent error message
	TErrorCode		m_ErrorCode;	// most recent error code
	char            m_IpAddr[255];  // machine we are connected to
    TUDPMsgHeader   m_headerTemplate; 
#ifdef _M_IX86
	short           m_Timer;
#endif
    vector<TObjDesc> m_objects;
	double           m_GetDynObjTimeSend;
	double           m_GetDynObjTimeRecv;
    std::unique_ptr<CUdpThread>    m_udpWorker;
    std::unique_ptr<CUdpSender>    m_udpSender;
    TUDPMsgHeader    m_header;
};


#endif		// #ifndef _HCSMCLIENT_H_

