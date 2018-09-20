/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1999 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmconnect.h,v 1.19 2018/05/18 13:59:58 IOWA\dheitbri Exp $
// Author:       Yiannis Papelis
// Date:         August, 1999
//
// Description:   Header file for supporing connecting to the
// hcsm execution modules.
//
/////////////////////////////////////////////////////////////////////////////
#ifndef _HCSMCONNECT_H_
#define _HCSMCONNECT_H_

#include <cvedpub.h>
#include <cved.h>

using namespace CVED;

#ifdef _WIN32
#include <set>
#include <string>
#include <vector>
#elif __sgi

#include <unistd.h>
#include <fstream.h>
#include <map.h>
#include <set.h>
#include <string.h>
#include <strstream.h>
#include <vector.h>
#define ostringstream ostrstream

#elif _PowerMAXOS
#define ostringstream ostrstream
#include <strstream>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <ctype.h>
#endif

#include "debugitem.h"
#include "workerthread.h"
// where the hcsmexec server listens for connections
#define DEFAULT_PORT  3333

// what the start marker should be on all TCP/IP messages
#define START_MARK    0xCFAB

// maximum message size
#define MAX_HCSM_MESSAGE_SIZE  (12*1024)

struct TMsgHeader {
	short		m_StartMark;		// should always be START_BYTE
	short		m_OpCode;			// message opcode
	long		m_MsgLen;			// length of message, including header
};
struct TADOState
{
TU16b			  visualState;	/*   1: left turn signal        */
								/*   2: right turn signal       */
								/*   4: hazard lights           */
								/*   8: high beam lights        */
								/*  16: brake lights            */
								/*  32: operating lights        */
								/*  64: backup lights           */
float             laneOffset;
float             hldOffsetDist;
}; 
//
// This structure encapsulates the information associated with
typedef struct TCommObjState {
	TPoint3D          position;     /* where object is located */
	/* the following two vectors use right hand rule */
	TVector3D         tangent;      /* tangent vector (normalized) */
	TVector3D         lateral;      /* lateral vector (normalized) */
	TPoint3D          boundBox[2];  /* objects' bounding box */
	double            vel;          /* velocity along tangent */
	short             colorIndex;   /* Color index for vehicles */
	union Specific {
        TADOState         adoState;

		TU16b			  visualState;	/*   1: left turn signal        */
										/*   2: right turn signal       */
										/*   4: hazard lights           */
										/*   8: high beam lights        */
										/*  16: brake lights            */
										/*  32: operating lights        */
										/*  64: backup lights           */
		eCVTrafficLightState	state;	/* Current state of the        */
										/* traffic light               */
		struct Enviro {
			TPoint2D          vertex[cCV_MAX_ENVIRO_VERTS];
											/* array of vertices associated */
											/*  with the visual rep. of the */
											/*  environmental condition     */
			eCVEnviroType     enviroType;   /* type of enviroment condition */
		};
		int					optionNum;		/* Option number for static objects */
	} specific;
} TCommObjState;

//
// This structure encapsulates the information associated with
// one object in the virtual environment.
// It is used to communicate information about objects between the
// process running the HCSMs and the client using this library.
struct TObjDesc {
	int          id;
	cvEObjType   type;

	int          solId;
	int          hcsmId;

	TCommObjState  state;
	char		name[cOBJ_NAME_LEN];
};


//
// This is a function structure that allows us to use sets
// of TObjDesc.  (sets need a comparator function, and this is
// how we provide one). 
//
struct TObjDescComp
{
	bool operator()(const TObjDesc& pO1, const TObjDesc& pO2) const
	{
		if (pO1.id != pO2.id)
			return pO1.id < pO2.id;
		if (pO1.hcsmId != pO2.hcsmId)
			return pO1.hcsmId < pO2.hcsmId;
		if (pO1.solId != pO2.solId)
			return pO1.solId < pO2.solId;
		if (pO1.type != pO2.type)
			return pO1.type < pO2.type;
		return &pO1 < &pO2;
	}
};

// A type definition to make declaring sets of TObjDesc structures
// easier.
typedef set<TObjDesc, TObjDescComp> TObjDescSet;

// A structure that holds information about a message
//

struct TDebugItemComp 
{
	bool operator()(const CHcsmDebugItem::TItem& i1, const CHcsmDebugItem::TItem& i2) const
	{
		return i1.frame >= i2.frame;
	}
};

typedef set<CHcsmDebugItem::TItem, TDebugItemComp> TDebugItemSet;

//
//
// This structure lays out the message sent across the network.
// The header is always there, and the data area uses a union
// to allow easy access to the message contents using type
// specific arrays.  We assume that the allignment of the
// various machines communicating using this structure is the
// same.  The arrays are dimensioned as size 2, but they can
// be accessed beyond that, as long as the data is within the
// space allocated by the 'bytes' array.
//
struct TMessage {
	TMsgHeader     header;
	union  {
		unsigned char  bytes[MAX_HCSM_MESSAGE_SIZE - sizeof(TMsgHeader)];
		float          floats[2];			// can index past its end
		double         doubles[2];
		int            ints[2];
		short          shorts[2];
		TObjDesc       objs[2];
		CHcsmDebugItem::TItem     
					   dbgItems[2];
	} data;
};
struct TUDPMsgHeader{
    SOCKADDR_IN senderAddr;
    int msgSize;
};

class CUdpThread: public CAccumulatorBufferThread{
public:
    CUdpThread(unsigned short port);
    virtual void BufferLoop(TBuffRef) override;
    bool Start() override;
    bool Stop() override;
protected:
    unsigned short m_port;
    SOCKET m_socket;
};

class CUdpSender{
public:
    typedef std::unique_ptr<CUdpSender> TRef;
    CUdpSender();
    ~CUdpSender();
    void Send(const TUDPMsgHeader msg, const TMessage &msgs);
protected:
    SOCKET m_socket;
};


int ReadMessage(std::vector<char>& buff, TMessage &msgs, TUDPMsgHeader &header);

extern SOCKET WaitForConnection(int port);
extern SOCKET CreateNonBlockingSocket(int port);
extern SOCKET PollSocketForConnection(SOCKET sock);
extern int  ReadHeader(SOCKET sock, TMsgHeader &);
extern int  ReadMessage(SOCKET sock, const TMsgHeader &, TMessage &);
extern bool SendMessage(SOCKET sock, int cmd, const void *, int size);
extern int  RecvMessage(SOCKET sock, TMessage& msg);
extern SOCKET ConnectToSocket(const char ip[], int port);
extern bool StartNewProcess(char* const args[]);
extern bool CloseSocket(SOCKET sock);
extern void CvedDataToCommData( const cvTObjState&, TCommObjState&, cvEObjType, CCved* pCved, int id);
extern void CvedDataToCommDataLE( const cvTObjState&, TCommObjState&, cvEObjType, CCved* pCved, int id);
#define CMD_PING               1
#define CMD_SCENSCRIPT         2
#define CMD_SCENSCRIPTMULTI    3
#define CMD_SIMPARAMS          4
#define CMD_STARTSCEN          5
#define CMD_ACKOK              6
#define CMD_ACKERROR           7
#define CMD_RUNFRAMES          8
#define CMD_SETDEBUGMODE       9
#define CMD_GETDEBUGDATA      10
#define CMD_ENDSCENARIO       11
#define CMD_GETDYNAOBJS       12
#define CMD_GETINSTANCEDOBJS  13
#define CMD_TAKEOBJCONTROL    14
#define CMD_CONTROLOBJ        15
#define CMD_RELEASEOBJCONTROL 16
#define CMD_GETCHANGEDSTATICOBJS 17
#define CMD_SETDAIL           18
#define CMD_RESETDAIL         19

#define CMD_GETDYNAOBJS_RESP   118
#define CMD_TAKEOBJCONTROL_ACK 119

#define CMD_QUIT              99

#endif
