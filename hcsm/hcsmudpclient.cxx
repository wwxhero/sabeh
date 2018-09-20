//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmudpclient.cxx,v 1.5 2018/07/16 14:15:48 IOWA\dheitbri Exp $
//
// Author(s):    Yiannis Papelis
//
// Date:         August, 1999
// Description:  Implemention of the hcsmclient library class.
//
//////////////////////////////////////////////////////////////////////////////
// the following must appear to support precompiled headers  on the PC.
#include "hcsmudpclient.h"
#include "hcsmconnect.h"
#include "genhcsmglobal.h"

#ifdef _WIN32

#elif __sgi
#include <sys/types.h>
#include <netinet/in.h>
#elif _PowerMAXOS
#include <sys/types.h>
#include <netinet/in.h>
#endif

#undef  _DEBUG_LIBRARY_

/////////////////////////////////////////////////////////////////////////////
//
// Description: default constructor
//
// Remarks:
//
// Arguments:
//
// Returns:
//
CHcsmUdpClient::CHcsmUdpClient()
{
	m_ErrorCode = ENoError;
	m_State     = EDorm;


#if defined(_WIN32) && defined(_M_IX86)
	m_Timer     = hrt_timer_alloc(HRT_MICROSECOND, "CHcsmUdpClient");
#endif

	m_GetDynObjTimeSend = 0.0;
	m_GetDynObjTimeRecv = 0.0;
}

void CHcsmUdpClient::Tick(){
    TMessage msgs;
    auto getDynObjHeader = m_headerTemplate;
    getDynObjHeader.msgSize = sizeof(msgs.header);
    msgs.header.m_OpCode = CMD_GETDYNAOBJS;
    msgs.header.m_MsgLen = 0;

    TMessage testSteer;
    static int cnt=0;
    cnt++;
    if (cnt%120 == 0){
    stringstream ss;
        ss<<2<<" \"ForcedVelocity\"";
        testSteer.header.m_StartMark=cnt;
        m_udpSender->Send(getDynObjHeader,msgs);
        testSteer.header.m_OpCode = CMD_RESETDAIL;
        testSteer.header.m_MsgLen = (int)ss.str().size();
        memcpy(testSteer.data.bytes,ss.str().data(),testSteer.header.m_MsgLen);
        m_udpSender->Send(getDynObjHeader,testSteer);        
    }else{
        float angle =  sin((cnt)/162.0f)*15.0+15.0;
        stringstream ss;
        ss<<2<<" \"ForcedVelocity\" "<<"\"stream "<<angle<<"\"";
        testSteer.header.m_StartMark=cnt;
        m_udpSender->Send(getDynObjHeader,msgs);
        testSteer.header.m_OpCode = CMD_SETDAIL;
        testSteer.header.m_MsgLen = (int)ss.str().size();
        memcpy(testSteer.data.bytes,ss.str().data(),testSteer.header.m_MsgLen);
        m_udpSender->Send(getDynObjHeader,testSteer);
    }
    CAccumulatorBufferThread::TBufferList data;
    m_udpWorker->GetCurrentBuffer(data);
    for (auto itr = data.begin(); itr != data.end(); ++itr){
        TMessage msg;TUDPMsgHeader header;
        auto &currBuff = *itr->get();
        auto &byteBuff = (*currBuff.AsByteBuffer())();
        ReadMessage(byteBuff,msg,header);
        this->DecodeMessage(msg);
    }

}
/////////////////////////////////////////////////////////////////////////////
//
// Description: destructor
//
// Remarks:
//
// Arguments:
//
// Returns:
//
CHcsmUdpClient::~CHcsmUdpClient()
{
#if defined(_WIN32) && defined(_M_IX86)
	hrt_timer_free(m_Timer);
#endif
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Set the mode the library uses to control the hcsms
//
// Remarks:
// There are two possible mode, synchronous and free running.  In
// synchronous mode, the hcsmexec program should be located somewhere
// for execution, and the InitSynchronous function should be called.
// In free running mode, the hcsm system should be running somewhere
// on the network and should include the GatewayHCSM.  The mode-specific
// function InitFreeRunning should be called to provide the relevant
// information.
//
// Arguments:
// mode - the mode to set
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::SetMode(TMode mode)
{
	m_Mode  = mode;
	return true;
}




/////////////////////////////////////////////////////////////////////////////
//
// Description:  Initialize the library when in free running mode
//
// Remarks:
//
// Arguments:
// IpAddr - a string of the form XXX.XXX.XXX.XXX containing the IP
//          address of the computer that is running the behaviors
// Port - the socket port to use for contacting the hcsm system
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::InitFreeRunning(const string &IpAddr, int Port)
{
    m_headerTemplate.senderAddr.sin_addr.s_addr=inet_addr(IpAddr.c_str()); ;
    short port = DEFAULT_PORT+10;
    m_headerTemplate.senderAddr.sin_port = htons(port);
    m_headerTemplate.senderAddr.sin_family = AF_INET;
    port=DEFAULT_PORT+11;
    CUdpThread* pThread = new CUdpThread(port);
    m_udpWorker = TUdpThreadPtr(pThread);
    m_udpWorker->Start();
    m_udpSender = std::unique_ptr<CUdpSender>(new CUdpSender());

	return true;
}








/////////////////////////////////////////////////////////////////////////////
//
// Description: Terminate any connections to the HCSM system
//
// Remarks:
// This function terminates any open connections with the HCSM system
// and puts the class in the same state as after the default constructor.
//
// Arguments:
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::Quit(void)
{
    return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Obtain error information from the most recent function
//
// Remarks:
// This function provides access to the error message and error code
// generated from the  most recent member function of the class.
// Keep in mind that every function resets the error code so if
// a function returns false, call GetLastError immediately since
// the next function call simply reset the error message according to its
// execution status.
//
// Arguments:
// Msg - the error message
// Code - the error code
//
// Returns:
// Nothing.
//
void
CHcsmUdpClient::GetLastError(string &Msg, TErrorCode &Code)
{
	Msg  = m_ErrorMsg;
	Code = m_ErrorCode;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Get the name of the LRI file is use
//
// Remarks:
// This function provides the name of the LRI file that is currently
// in use by the virtual environment.
//
// Arguments:
// LriName - where to store the file name
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::GetLriFileName(string &LriName)
{
	m_ErrorCode = EModeError;
	m_ErrorMsg = "feature not implemented yet";
	return false;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: set the debug mode of one or more HCSMs
//
// Remarks:
// This function sets the HCSMs whose identifiers are contained in the
// HcsmSet set to the specified value.
//
// Arguments:
// Mode - the debug mode to set for the specified HCSMs
// HcsmSet - which HCSMs to affect. This is a set of HCSM identifiers.
// level - the debug level
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::SetDebugMode(TDebugMode Mode, const vector<int> &HcsmSet, int level)
{
	TMessage  msg;
	vector<int>::const_iterator  pI;
	int  i;
	int  maxIdInMsg = (sizeof(TMessage) - sizeof(TMsgHeader)) / sizeof(short);

	// For this message, we use the array of "short" integers to conserve
	// bandwidth.  The first slot contains an enumeration of the actual
	// mode, the second slot contains the debug level,
	// the thrid slot contains the number of HCSMs to affect and
	// the remaining slots contain the actual IDs of HCSMs on which to
	// apply the debug mode change.  As a special case, if only one
	// HCSM is specified, and its ID is -1, that means to apply the change
	// to all the HCSMs.
	msg.data.shorts[0] = (short)Mode;
	msg.data.shorts[1] = (short)level;
	msg.data.shorts[2] = (short)HcsmSet.size();
	i = 3;
	for ( pI = HcsmSet.begin(); pI != HcsmSet.end(); pI++ ) {
		if ( i < maxIdInMsg ) {
			msg.data.shorts[i++] = *pI;
		}
	}
    return true;

}


static void ntohf( double inputDouble, double& output )
{
	float input = static_cast<float>(inputDouble);
	unsigned long tmp;
	memcpy( &tmp, &input, sizeof(tmp) );
	tmp = ntohl( tmp );
	memcpy( &input, &tmp, sizeof(tmp) );
	output = input;
}



static void ntohf( double inputDouble, float& output )
{
	float input = static_cast<float>(inputDouble);
	unsigned long tmp;
	memcpy( &tmp, &input, sizeof(tmp) );
	tmp = ntohl( tmp );
	memcpy( &output, &tmp, sizeof(tmp) );
}



static void ntohf( float input, float& output )
{
	unsigned long tmp;
	memcpy( &tmp, &input, sizeof(tmp) );
	tmp = ntohl( tmp );
	memcpy( &output, &tmp, sizeof(tmp) );
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Get all dynamic objectets present in the virtual environment
//
// Remarks:
// This function gets the information about all objects that are currently
// active in the virtual environment of the HCSM system.  The information
// about the objects is placed in a vector.
//
// Keep in mind that when running in Free Running mode, the state of
// the environment could be changing at a rapid pace.  The TObjDesc
// structure contains the simulation frame during which this info was
// valid.  There is no guarantee that a particular object will even be
// alive at a later frame.
//
// Arguments:
// Objs - an array of object descriptions representing dynamic objects
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::GetChangedStaticObjs(vector<TObjDesc> &Objs)
{
	Objs.clear();
    return true;

}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Get all dynamic objectets present in the virtual environment
//
// Remarks:
// This function gets the information about all objects that are currently
// active in the virtual environment of the HCSM system.  The information
// about the objects is placed in a vector.
//
// Keep in mind that when running in Free Running mode, the state of
// the environment could be changing at a rapid pace.  The TObjDesc
// structure contains the simulation frame during which this info was
// valid.  There is no guarantee that a particular object will even be
// alive at a later frame.
//
// Arguments:
// Objs - an array of object descriptions representing dynamic objects
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::GetDynamicObjs(vector<TObjDesc> &Objs)
{
	vector<TObjDesc> empty;
    Objs = std::move(m_objects);

#ifdef _DEBUG_LIBRARY_
	cerr << "Enter CHcsmUdpClient::GetDynamicObjs" << endl;
#endif
    return true;
}

bool
CHcsmUdpClient::GetChangedStaticObjs(TObjDescSet &Objs)
{
	vector<TObjDesc>  vec;
	vector<TObjDesc>::iterator i;

	bool code = GetChangedStaticObjs(vec);
	if ( ! code ) return false;

	Objs.clear();
	for (i=vec.begin(); i != vec.end(); i++) {

		Objs.insert(*i);
	}
	return true;

}

bool
CHcsmUdpClient::GetDynamicObjs(TObjDescSet &Objs)
{
	vector<TObjDesc>  vec;
	vector<TObjDesc>::iterator i;

	bool code = GetDynamicObjs(vec);
	if ( ! code ) return false;

	Objs.clear();
	for (i=vec.begin(); i != vec.end(); i++) {

		Objs.insert(*i);
	}
	return true;

}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Get static objects that require a one time initialization
//
// Remarks:
// This function returns all static objects that have been instanced at
// runtime and all static relocatable objects whose option is set to
// a value other than the LRI default.  In effect, these are all the
// objects that require a one time initialization by a client
// such as the ISAT.
//
// Arguments:
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::GetInstancedObjs(vector<TObjDesc> &Objs)
{
    return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Get all available debug messages
//
// Remarks:
// This function returns all debug messages (text and graphics) that
// the HCSMs have issued and are stored in the HCSM system buffers.
// Messages read by this function are removed from the HCSM system buffer
// so they will not be available next time this function is called.
//
// Keep in mind that the HCSM collection has a finite amount of space
// in which to keep these messages, so unless they are removed often 
// enough by this function, they will automatically be discarded.
//
// Arguments:
// Msgs - where to store text messages
// Graphics - where to store graphic commands
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::GetDebugData(vector<CHcsmDebugItem>  &DebugItems)
{
	DebugItems.clear();
    return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Take control of an object
//
// Remarks:
// This function sends appropriate messages to the HCSM system to indicate
// that the HCSM controlling the object whose identifier is passed as
// an argument should go into remote control mode.  In addition, the
// initial command to the object is also specified.
//
// The function will fail if the object does not exist, of if the
// controlling HCSM does not support remote control operation.
//
// Once an object is placed in remote control, call the ControlObj
// function to sent it high level commands.
//
// Arguments:
// ObjId - the CVED identifier of the object to control
// InitialCmd - the initial command to send to the object
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::TakeControlOfObj(int ObjId, const TRemCntrlCmd &InitialCmd)
{
	m_ErrorCode = EModeError;
	m_ErrorMsg = "feature not implemented yet";
	return false;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Release control of an object.
//
// Remarks:
// This function releases the HCSM whose controlling object's id is 
// specified as an argument from remote control.
// The function will fail if the object id is not valid or if
// the specified object is not under remote control.
//
// Arguments:
// ObjId - the CVED identifier of the object to control
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::ReleaseControlOfObj(int ObjId)
{
	m_ErrorCode = EModeError;
	m_ErrorMsg = "feature not implemented yet";
	return false;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:  Send a remote control command to an object
//
// Remarks:
// This function sends a remote control command to the HCSM whose
// controlling object id is specified in the argument.  The object
// should have been placed in remote control mode by first calling the
// TakeControlOfObj function.
//
// Arguments:
// ObjId - the object to send the command to
// Cmd - the command to send.
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmUdpClient::ControlObj(int ObjId, const TRemCntrlCmd &Cmd)
{
	TMessage  msg;

	// while in synchronous mode we "talk" to the hcsmexec program
	// but if we are asynchronous mode we talk to the gateway HCSM.
	SOCKET sock;

	// For this message, we use the array of "int" integers to conserve
	// bandwidth.  The first slot contains an enumeration of the actual
	// mode, the second slot contains the debug level,
	// the thrid slot contains the number of HCSMs to affect and
	// the remaining slots contain the actual IDs of HCSMs on which to
	// apply the debug mode change.  As a special case, if only one
	// HCSM is specified, and its ID is -1, that means to apply the change
	// to all the HCSMs.
	msg.data.ints[0] = htonl(Cmd.command);
	msg.data.ints[1] = htonl(ObjId);
	msg.data.ints[2] = htonl(Cmd.val);

    return true;
}


bool
CHcsmUdpClient::ConnectToDataSock(void)
{

	return true;
}
void 
CHcsmUdpClient::DecodeMessage(const TMessage &msg){
    int opCode = msg.header.m_OpCode;
    if (opCode = CMD_GETDYNAOBJS_RESP){
        const TObjDesc &temp = (msg.data.objs[0]);
        m_objects.push_back(temp);
    }
}