//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmclient.cxx,v 1.31 2016/10/28 20:51:47 IOWA\dheitbri Exp $
//
// Author(s):    Yiannis Papelis
//
// Date:         August, 1999
// Description:  Implemention of the hcsmclient library class.
//
//////////////////////////////////////////////////////////////////////////////
// the following must appear to support precompiled headers  on the PC.
#include "hcsmclient.h"

#ifdef _WIN32

#elif __sgi
#include <sys/types.h>
#include <netinet/in.h>
#elif _PowerMAXOS
#include <sys/types.h>
#include <netinet/in.h>
#endif

#undef  _DEzBUG_LIBRARY_

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
CHcsmClient::CHcsmClient()
{
	m_Mode      = ENone;
	m_ErrorCode = ENoError;
	m_State     = EDorm;

	m_CmdSock   = -1;
	m_DataSock  = -1;

#if defined(_WIN32) && defined(_M_IX86)
	m_Timer     = hrt_timer_alloc(HRT_MICROSECOND, "CHcsmClient");
#endif

	m_GetDynObjTimeSend = 0.0;
	m_GetDynObjTimeRecv = 0.0;
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
CHcsmClient::~CHcsmClient()
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
CHcsmClient::SetMode(TMode mode)
{
	m_Mode  = mode;
	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:  Initialize the library when in synchronous mode
//
// Remarks:
// When in synchronous mode, this function initializes the library
// by starting the hcsmexec program.  The method used to start the
// program varies depending on the contents of the Directory argument.
// If the argument is of the form :manual:IP_ADDRESS then the
// function will assume that the program will be started manually
// and it will be residing at a machine whose address is listed
// immediately after the :manual: string.  Otherwise, the function
// will start the hcsmexec program as a separate process by first
// looking for it in the directory specified in the Directory argument.
//
// Either way, the function will open a socket to the hcsmexec program
// and initiate communications with that program.  If all goes well,
// the function returns, and the client library can be used
// to start and execute scenarios.
//
// Arguments:
// Directory - the location where to find the hcsmexec program.
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmClient::InitSynchronous(const string &Directory)
{
	//
	// Check mode; can only call this one in synchronous mode
	//
	if ( m_Mode != ESynchronous ) {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "Incompatible execution mode.";
		return false;
	}

	if ( m_State != EDorm ) {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "Cannot call InitSynchronous from current state.";
		return false;
	}

	// Initialize windows sockets
#ifdef _WIN32
	WSAData wsaData;
	if ( WSAStartup(MAKEWORD(1, 1), &wsaData) != 0 ) {
		gout << "tstclient: cannot initialize sockets." << endl;
		return false;
	}
#endif

	//
	// Start the hcsmexec program, if it needs to be started
	//
	if ( strncmp(Directory.c_str(), ":manual:", 8) )  { // needs starting ...
		string path;
        string binPath;
        NADS::GetEnvVar(binPath,"NADSSDC_BIN");
		if (binPath != "") {
			//  construct a new path including the %NADSSDC_BIN directory
#if 0
			strcpy_s( path, "PATH=" );
			strcat( path, getenv( "NADSSDC_BIN" ));
			strcat( path, ";");
			strcat( path, getenv( "PATH" ));
			putenv( path );
#endif
			path = binPath;
			path+= "\\hcsmexec.exe";

		} else {
			path =  "hcsmexec";
		}
		char*   arg[] = { &path[0], ",arg1", "arg2", 0 };

		if ( !StartNewProcess(arg) ) {
			m_ErrorCode = EModeExecProcError;
			m_ErrorMsg  = "Cannot start the hcsmexec program.  Make sure it is\n"
				"in the current directory or in a directory that is in your PATH.";

			return false;
		}
		strcpy_s(m_IpAddr, "localhost");

		// give it a second or so to start
#ifdef _WIN32
		Sleep(1000);
#else
		sleep(2);
#endif
	}
	else {   // manual start
		// find the ip addr within the directory string
		strncpy_s(m_IpAddr, Directory.c_str() + 8, sizeof(m_IpAddr)-1);
	}

	// at this point, hcsmexec is started so we connect
	gout << "Will connect to '" << m_IpAddr << "' ..." << endl;

	m_CmdSock = ConnectToSocket(m_IpAddr, DEFAULT_PORT);
	if ( m_CmdSock < 0  ) {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg  = "Cannot connect to socket (check address, server etc.)";
		return false;
	}

	gout << "Connection ok" << endl << flush;

	TMessage  pong;
	if ( SendMessage(m_CmdSock, CMD_PING, 0, 0) ) {
		if ( RecvMessage(m_CmdSock, pong) ) {
			return true;
		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "no reply to PING";
			return false;
		}
	}
	else {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "could not send PING";
		return false;
	}
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
CHcsmClient::InitFreeRunning(const string &IpAddr, int Port)
{
	if ( m_Mode != EFreeRunning ) {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "Incompatible execution mode.";
		return false;
	}

	// Initialize windows sockets
#ifdef _WIN32
	WSAData wsaData;
	if ( WSAStartup(MAKEWORD(1, 1), &wsaData) != 0 ) {
		gout << "tstclient: cannot initialize sockets." << endl;
		return false;
	}
#endif

	strncpy_s(m_IpAddr, IpAddr.c_str(), sizeof(m_IpAddr)-1);

	if ( !ConnectToDataSock() ) {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "Cannot connect to data socket";
		return false;
	}

	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:  Load a scenario script
//
// Remarks:
// This function can only be called when in synchronous mode.  It will 
// attempt to initialize the provided script on the hcsmexec program
// using the specified parameters.
// The function will not return until the scenario (and associated LRI file)
// have been loaded.
//
// Arguments:
// Script - the contents of the scenario file to execute
// LriDir - the directory on the remote system where LRI files reside
// Frequ - the execution frequency of the behaviors, in Hz
// DynaFrames - how many times to run the dynamic servers at each frame
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmClient::StartScenario(
		const string  &Script, 
		const string  &LriDir, 
		double         Frequ, 
		int            DynaFrames,
		const string  &scnDir)
{
	if ( m_Mode != ESynchronous ) {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "Incompatible execution mode.";
		return false;
	}

	int UsableSize = MAX_HCSM_MESSAGE_SIZE - sizeof(TMsgHeader);
	int ScriptSize = (int)Script.length()+1;

	gout << "Enter ScartScenario" << endl;


	//
	// First send the scenario file.  If the file is larger than
	// the maximum message we can sent, we break it up.
	//
	if ( ScriptSize <= UsableSize )  {
#ifdef _DEBUG_LIBRARY_
		cerr << "Small script, size is " << ScriptSize << endl;
#endif
		if ( !SendMessage(m_CmdSock, CMD_SCENSCRIPT, Script.c_str(), 
										ScriptSize) ) {
#ifdef _DEBUG_LIBRARY_
			cerr << "Could not send script at line " << __LINE__ << endl;
#endif
			return false;
		}
	}
	else {
		long BytesSent    = 0;

#ifdef _DEBUG_LIBRARY_
		cerr << "Long scenario, lenght " << Script.length() 
				<< " Usable size= " << UsableSize << flush << endl;
#endif

		while ( BytesSent < ScriptSize ) {
			if ( ScriptSize - BytesSent <= UsableSize ) { // last msg
				if ( !SendMessage(m_CmdSock, CMD_SCENSCRIPT,
							Script.c_str() + BytesSent,
							ScriptSize - BytesSent) ) {
#ifdef _DEBUG_LIBRARY_
					cerr << "Could not send last message" << endl;
#endif
					return false;
				}
				BytesSent += (ScriptSize - BytesSent);
			}
			else {		// one more part
#ifdef _DEBUG_LIBRARY_
				gout << "Full message " << endl;
#endif

				if ( !SendMessage(m_CmdSock, CMD_SCENSCRIPTMULTI,
								Script.c_str() + BytesSent, UsableSize) ) {
#ifdef _DEBUG_LIBRARY_
					cerr << "Could not send last message" << endl;
#endif
					return false;
				}

				BytesSent += UsableSize;

			}
		}
	}


	//
	// Now send the simulation parameters.  We pack
	// the frequency and dynamics rate in the first two
	// elements of an array and stuff the string at
	// the rest of the array.  We assume the lri directory
	// will not be larger than 256 characters in length
	// so that's how much space we allocate.
	//
	double data[2 + 512/sizeof(double)] = { Frequ, DynaFrames+0.1 };
	char   *pBuf = (char *)&data[2];
	strcpy_s(pBuf,2, LriDir.c_str());
	char   *pScnDirBuf = (char *)&data[256/sizeof(double)];
	strcpy_s(pScnDirBuf,2 + 512/sizeof(double), scnDir.c_str());
	int   totlen = 256 + scnDir.length() + 1;
	
	if ( !SendMessage(m_CmdSock, CMD_SIMPARAMS, data, totlen ) ) {
#ifdef _DEBUG_LIBRARY_
		cerr << "Could not send simulation parameters" << endl;
#endif
		return false;
	}

	//
	// Now send the command to start the scenario.  This will
	// cause the remote program to load CVED, and start
	// the HCSM system passing it the specified file as an
	// argument.  We will then read a message that indicates
	// the status of the initialization.
	//
	if ( !SendMessage(m_CmdSock, CMD_STARTSCEN, 0, 0) ) {
#ifdef _DEBUG_LIBRARY_
		cerr << "Could not send StartScenario command" << endl;
#endif
		return false;
	}

	TMessage response = { 0 };
	if ( !RecvMessage(m_CmdSock, response) ) {
#ifdef _DEBUG_LIBRARY_
		cerr << "Could not receive ack from StartScenario cmd" << endl;
#endif
		return false;
	}

	if ( response.header.m_OpCode == CMD_ACKOK ) {
		return true;
	}
	else 
	if ( response.header.m_OpCode == CMD_ACKERROR ) {
		m_ErrorMsg = (char *)response.data.bytes;
		return false;
	}
	else {
		char buf[200];
		sprintf_s(buf, "Received unexpected opcode %d instead of %d or %d",
			response.header.m_OpCode, CMD_ACKOK, CMD_ACKERROR);
		m_ErrorMsg = buf;
		return false;
	}
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Execute a specified number of frames
//
// Remarks:
// This function can only be called when in synchronous mode.  It will
// execute the behaviors for the specified number of frames.
//
// Arguments:
// Frames - the number of frames to execute
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmClient::RunFrames(int Frames, int Dyna)
{
	int data[2] = { Frames, Dyna };

	if ( m_Mode != ESynchronous ) {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "Incompatible execution mode.";
		return false;
	}
	
	if ( !SendMessage(m_CmdSock, CMD_RUNFRAMES, data, 2*sizeof(int))) {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "Could not send message.";
		return false;
	}
	else {
		TMessage dummy;

		if ( RecvMessage(m_CmdSock, dummy) > 0  ) {
			return true;
		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "Could not receive ack message.";
			return false;
		}
	}
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: End the currently executing scenario.
//
// Remarks:
// This function can only be called when in synchronous mode.  It will
// terminate execution of the HCSM system in effect causing the termination
// callbacks of all HCSMs to be called.
//
// Arguments:
//
// Returns:
// True to indicate success or false to indicate problems.  If the function
// returns false, call the GetLastError function for diagnostics.
//
bool
CHcsmClient::EndScenario(void)
{
	// we can only end a scenario when in synchronous mode
	if ( m_Mode != ESynchronous ) {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "Incompatible execution mode.";
		return false;
	}

	if ( !SendMessage(m_CmdSock, CMD_ENDSCENARIO, 0, 0))  {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "Could not send message.";
		return false;
	}
	else {
		TMessage dummy;

		if ( RecvMessage(m_CmdSock, dummy) > 0  ) {
			return true;
		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "Could not receive ack message.";
			return false;
		}
	}
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
CHcsmClient::Quit(void)
{
	// we can only terminate a scenario when in synchronous mode
	if ( m_Mode != ESynchronous ) {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "Incompatible execution mode.";
		return false;
	}

	if ( !SendMessage(m_CmdSock, CMD_QUIT, 0, 0)) {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "Could not send message.";
		return false;
	}
	else {
		TMessage dummy;

		if ( RecvMessage(m_CmdSock, dummy) > 0  ) {
			return true;
		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "Could not receive ack message.";
			return false;
		}
	}
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
CHcsmClient::GetLastError(string &Msg, TErrorCode &Code)
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
CHcsmClient::GetLriFileName(string &LriName)
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
CHcsmClient::SetDebugMode(TDebugMode Mode, const vector<int> &HcsmSet, int level)
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
	msg.data.shorts[1] = level;
	msg.data.shorts[2] = HcsmSet.size();
	i = 3;
	for ( pI = HcsmSet.begin(); pI != HcsmSet.end(); pI++ ) {
		if ( i < maxIdInMsg ) {
			msg.data.shorts[i++] = *pI;
		}
	}

	if ( !SendMessage(m_CmdSock, CMD_SETDEBUGMODE, &msg.data, i * sizeof(short)))  {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "Could not send message.";
		return false;
	}
	else {
		TMessage dummy;

		if ( RecvMessage(m_CmdSock, dummy) > 0  ) {
			return true;
		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "Could not receive ack message.";
			return false;
		}
	}
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
CHcsmClient::GetChangedStaticObjs(vector<TObjDesc> &Objs)
{
	Objs.clear();

#ifdef _DEBUG_LIBRARY_
	cerr << "Enter CHcsmClient::GetChangedStaticObjs" << endl;
#endif

	// while in synchronous mode we "talk" to the hcsmexec program
	// but if we are asynchronous mode we talk to the gateway HCSM.
	SOCKET sock;
	if ( m_Mode == ESynchronous ) {
		sock = m_CmdSock;
	}
	else
	if ( m_Mode == EFreeRunning ) {
		if ( m_DataSock == -1 ) {
			if ( ! ConnectToDataSock() ) {
				m_ErrorCode = EModeSocketComm;
				m_ErrorMsg  = "could not connect to data socket";
				return false;
			}
		}
		sock = m_DataSock;
	}
	else {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "mode has not been specified";
		return false;
	}

	TMessage    info;
	long rcode1, rcode2;

#if defined(_M_IX86) && defined(_WIN32)
	hrt_timer_start(m_Timer);
	rcode1 = SendMessage(sock, CMD_GETCHANGEDSTATICOBJS, 0, 0);
	hrt_timer_stop(m_Timer);
	m_GetDynObjTimeSend = hrt_timer_lastelapsedsecs(m_Timer);

	hrt_timer_start(m_Timer);
	rcode2 = RecvMessage(sock, info);
	hrt_timer_stop(m_Timer);
	m_GetDynObjTimeRecv = hrt_timer_lastelapsedsecs(m_Timer);
#else
	rcode1 = SendMessage(sock, CMD_GETCHANGEDSTATICOBJS, 0, 0);
	rcode2 = RecvMessage(sock, info);
#endif

	if ( rcode1 ) {
		if ( rcode2 ) {
			int*        pIntData = &info.data.ints[0];
			int NumObjs        = ntohl(pIntData[0]);
			int NumMessages    = ntohl(pIntData[1]);
			int ObjsPerMessage = ntohl(pIntData[2]);
			int ObjsInLastMsg  = ntohl(pIntData[3]);
			int msgInd;

#ifdef _DEBUG_LIBRARY_
			cerr << "CHcsmClient::GetChangedStaticObjs got response on GETDYN"<<endl;
			cerr << "OpCode is " << info.header.m_OpCode << endl
				<< "Length is " << info.header.m_MsgLen  
				<< "Data is " << endl
				<< "Total Objs       = " << NumObjs << endl
				<< "Num Messgs       = " << NumMessages << endl
				<< "Objs per Msg     = " << ObjsPerMessage << endl
				<< "Objs in Last Msg = " << ObjsInLastMsg << endl
				<< endl;
#endif

			for (msgInd = 0; msgInd < NumMessages; msgInd++) {
#if defined(_WIN32) && defined(_M_IX86)
				hrt_timer_start(m_Timer);
				int code = RecvMessage(sock, info);
				hrt_timer_stop(m_Timer);
				m_GetDynObjTimeRecv += hrt_timer_lastelapsedsecs(m_Timer);
#else
				int code = RecvMessage(sock, info);
#endif
				if ( code < 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "no follow on reply to GETDYNAOBJS";
					return false;
				}
				else
				if ( code == 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "RecvMessage return 0; should have blocked";
					return false;
				}

				// got message ok, determine how many objects are there
				// in the current message.  All messages should be full
				// with the exception of the last message.
				int        count;
				int        obj;
				TObjDesc*  pStore;

				count = (msgInd==NumMessages-1)?ObjsInLastMsg:ObjsPerMessage;
				for (obj=0; obj<count; obj++) {
					info.data.objs[obj].id = ntohl( info.data.objs[obj].id );
					info.data.objs[obj].type = (cvEObjType)ntohl( info.data.objs[obj].type );
					info.data.objs[obj].solId =
							ntohl( info.data.objs[obj].solId );
					info.data.objs[obj].hcsmId =
							ntohl( info.data.objs[obj].hcsmId );
					info.data.objs[obj].state.specific.optionNum = 
						ntohl( info.data.objs[obj].state.specific.optionNum );
					pStore = info.data.objs + obj;
					Objs.push_back(*pStore);
				}
			}
			return true;

		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "no reply to GETCHANGEDSTATICOBJS";
			return false;
		}
	}
	else {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "could not send PING";
		return false;
	}
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
CHcsmClient::GetDynamicObjs(vector<TObjDesc> &Objs)
{
	vector<TObjDesc> empty;
	Objs = empty;

#ifdef _DEBUG_LIBRARY_
	cerr << "Enter CHcsmClient::GetDynamicObjs" << endl;
#endif

	// while in synchronous mode we "talk" to the hcsmexec program
	// but if we are asynchronous mode we talk to the gateway HCSM.
	SOCKET sock;
	if ( m_Mode == ESynchronous ) {
		sock = m_CmdSock;
	}
	else
	if ( m_Mode == EFreeRunning ) {
		if ( m_DataSock == -1 ) {
			if ( ! ConnectToDataSock() ) {
				m_ErrorCode = EModeSocketComm;
				m_ErrorMsg  = "could not connect to data socket";
				return false;
			}
		}
		sock = m_DataSock;
	}
	else {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "mode has not been specified";
		return false;
	}

	TMessage    info;
	long rcode1, rcode2;

#if defined(_WIN32) && defined(_M_IX86)
	hrt_timer_start(m_Timer);
	rcode1 = SendMessage(sock, CMD_GETDYNAOBJS, 0, 0);
	hrt_timer_stop(m_Timer);
	m_GetDynObjTimeSend = hrt_timer_lastelapsedsecs(m_Timer);

	hrt_timer_start(m_Timer);
	rcode2 = RecvMessage(sock, info);
	hrt_timer_stop(m_Timer);
	m_GetDynObjTimeRecv = hrt_timer_lastelapsedsecs(m_Timer);
#else
	rcode1 = SendMessage(sock, CMD_GETDYNAOBJS, 0, 0);
	rcode2 = RecvMessage(sock, info);
#endif

	if ( rcode1 ) {
		if ( rcode2 ) {
			int*        pIntData = &info.data.ints[0];
			int NumObjs        = ntohl(pIntData[0]);
			int NumMessages    = ntohl(pIntData[1]);
			int ObjsPerMessage = ntohl(pIntData[2]);
			int ObjsInLastMsg  = ntohl(pIntData[3]);
			int msgInd;

#ifdef _DEBUG_LIBRARY_
			cerr << "CHcsmClient::GetDynamicObjs got response on GETDYN"<<endl;
			cerr << "OpCode is " << info.header.m_OpCode << endl
				<< "Length is " << info.header.m_MsgLen  
				<< "Data is " << endl
				<< "Total Objs       = " << NumObjs << endl
				<< "Num Messgs       = " << NumMessages << endl
				<< "Objs per Msg     = " << ObjsPerMessage << endl
				<< "Objs in Last Msg = " << ObjsInLastMsg << endl
				<< endl;
#endif

			for (msgInd = 0; msgInd < NumMessages; msgInd++) {
#if defined(_WIN32) && defined(_M_IX86)
				hrt_timer_start(m_Timer);
				int code = RecvMessage(sock, info);
				hrt_timer_stop(m_Timer);
				m_GetDynObjTimeRecv += hrt_timer_lastelapsedsecs(m_Timer);
#else
				int code = RecvMessage(sock, info);
#endif
				if ( code < 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "no follow on reply to GETDYNAOBJS";
					return false;
				}
				else
				if ( code == 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "RecvMessage return 0; should have blocked";
					return false;
				}

				// got message ok, determine how many objects are there
				// in the current message.  All messages should be full
				// with the exception of the last message.
				int        count;
				int        obj;
				TObjDesc*  pStore;

				count = (msgInd==NumMessages-1)?ObjsInLastMsg:ObjsPerMessage;
				for (obj=0; obj<count; obj++) {
					info.data.objs[obj].id = ntohl( info.data.objs[obj].id );
					info.data.objs[obj].type = (cvEObjType)ntohl( info.data.objs[obj].type );
					info.data.objs[obj].solId =
							ntohl( info.data.objs[obj].solId );
					info.data.objs[obj].hcsmId =
							ntohl( info.data.objs[obj].hcsmId );
					ntohf( info.data.objs[obj].state.position.x,
							info.data.objs[obj].state.position.x );
					ntohf( info.data.objs[obj].state.position.y,
							info.data.objs[obj].state.position.y );
					ntohf( info.data.objs[obj].state.position.z,
							info.data.objs[obj].state.position.z );
					ntohf( info.data.objs[obj].state.tangent.i,
							info.data.objs[obj].state.tangent.i );
					ntohf( info.data.objs[obj].state.tangent.j,
							info.data.objs[obj].state.tangent.j );
					ntohf( info.data.objs[obj].state.tangent.k,
							info.data.objs[obj].state.tangent.k );
					ntohf( info.data.objs[obj].state.lateral.i,
							info.data.objs[obj].state.lateral.i );
					ntohf( info.data.objs[obj].state.lateral.j,
							info.data.objs[obj].state.lateral.j );
					ntohf( info.data.objs[obj].state.lateral.k,
							info.data.objs[obj].state.lateral.k );
					ntohf( info.data.objs[obj].state.vel,
							info.data.objs[obj].state.vel );
					if (info.data.objs[obj].type == eCV_TRAFFIC_LIGHT) {
						info.data.objs[obj].state.specific.state = 
							(eCVTrafficLightState)ntohl( info.data.objs[obj].state.specific.state );
					}
					pStore = info.data.objs + obj;
					Objs.push_back(*pStore);
				}
			}
			return true;

		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "no reply to GETDYNAOBJS";
			return false;
		}
	}
	else {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "could not send PING";
		return false;
	}
}

bool
CHcsmClient::GetChangedStaticObjs(TObjDescSet &Objs)
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
CHcsmClient::GetDynamicObjs(TObjDescSet &Objs)
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
CHcsmClient::GetInstancedObjs(vector<TObjDesc> &Objs)
{
	vector<TObjDesc> empty;
	Objs = empty;

#ifdef _DEBUG_LIBRARY_
	cerr << "Enter CHcsmClient::GetInstancedObjs" << endl;
#endif

	SOCKET sock;
	if ( m_Mode == ESynchronous ) {
		sock = m_CmdSock;
	}
	else
	if ( m_Mode == EFreeRunning ) {
		if ( m_DataSock == -1 ) {
			if ( ! ConnectToDataSock() ) {
				m_ErrorCode = EModeSocketComm;
				m_ErrorMsg  = "could not connect to data socket";
				return false;
			}
		}
		sock = m_DataSock;
	}
	else {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "mode has not been specified";
		return false;
	}

	TMessage    info;

	if ( SendMessage(sock, CMD_GETINSTANCEDOBJS, 0, 0) ) {
		if ( RecvMessage(sock, info) ) {
			int*        pIntData = &info.data.ints[0];

#ifdef _DEBUG_LIBRARY_
			cerr << "OpCode is " << info.header.m_OpCode << endl
				<< "Length is " << info.header.m_MsgLen  
				<< "Data is " << endl
				<< "Slot 1 = " << pIntData[0] << endl
				<< "Slot 2 = " << pIntData[1] << endl
				<< "Slot 3 = " << pIntData[2] << endl
				<< "Slot 4 = " << pIntData[3] << endl
				<< endl;
#endif

			int NumObjs        = pIntData[0];
			int NumMessages    = pIntData[1];
			int ObjsPerMessage = pIntData[2];
			int ObjsInLastMsg  = pIntData[3];
			int msgInd;

			for (msgInd = 0; msgInd < NumMessages; msgInd++) {
				int code = RecvMessage(sock, info);
				if ( code < 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "no follow on reply to GETINSTBJS";
					return false;
				}
				else
				if ( code == 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "RecvMessage return 0; should have blocked";
					return false;
				}

				// got message ok, determine how many objects are there
				// in the current message.  All messages should be full
				// with the exception of the last message.
				int        count;
				int        obj;
				TObjDesc*  pStore;

				count = (msgInd==NumMessages-1)?ObjsInLastMsg:ObjsPerMessage;
				for (obj=0; obj<count; obj++) {
					pStore = info.data.objs + obj;

					Objs.push_back(*pStore);
				}
			}
			return true;

		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "no reply to GETDYNAOBJS";
			return false;
		}
	}
	else {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "could not send PING";
		return false;
	}
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
CHcsmClient::GetDebugData(vector<CHcsmDebugItem>  &DebugItems)
{
	DebugItems.clear();

#ifdef _DEBUG_LIBRARY_
	cerr << "Enter CHcsmClient::GetDebugData" << endl;
#endif

	SOCKET sock;
	if ( m_Mode == ESynchronous ) {
		sock = m_CmdSock;
	}
	else
	if ( m_Mode == EFreeRunning ) {
		if ( m_DataSock == -1 ) {
			if ( ! ConnectToDataSock() ) {
				m_ErrorCode = EModeSocketComm;
				m_ErrorMsg  = "could not connect to data socket";
				return false;
			}
		}
		sock = m_DataSock;
	}
	else {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "mode has not been specified";
		return false;
	}

	TMessage    info;

	if ( SendMessage(sock, CMD_GETDEBUGDATA, 0, 0) ) {
		if ( RecvMessage(sock, info) ) {
			int*        pIntData = &info.data.ints[0];

			// we now received a message; the first message
			// contains information about how many more messages
			// we are going to have to read
#ifdef _DEBUG_LIBRARY_
			cerr << "OpCode is " << info.header.m_OpCode << endl
				<< "Length is " << info.header.m_MsgLen  
				<< "Data is " << endl
				<< "Slot 1 = " << pIntData[0] << endl
				<< "Slot 2 = " << pIntData[1] << endl
				<< "Slot 3 = " << pIntData[2] << endl
				<< "Slot 4 = " << pIntData[3] << endl
				<< endl;
#endif
			int NumItems        = pIntData[0];
			int NumMessages     = pIntData[1];
			int ItemsPerMessage = pIntData[2];
			int ItemsInLastMsg  = pIntData[3];
			int msgInd;

			vector<CHcsmDebugItem::TItem>  Items;

			for (msgInd = 0; msgInd < NumMessages; msgInd++) {
				int code = RecvMessage(sock, info);
				if ( code < 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "no follow on reply to GETDEBUGDATA";
					return false;
				}
				else
				if ( code == 0 ) {
					m_ErrorCode = EModeSocketComm;
					m_ErrorMsg = "RecvMessage return 0; should have blocked";
					return false;
				}

				// got message ok,
				int          count;
				int          item;
				CHcsmDebugItem::TItem*  pStore;

				count = (msgInd==NumMessages-1)?ItemsInLastMsg:ItemsPerMessage;
				for (item=0; item<count; item++) {
					vector<CHcsmDebugItem::TItem>  subItems;

					pStore = info.data.dbgItems + item;
					if ( pStore->nParts > 1 ) {
						int j;

						int  numParts = pStore->nParts;

						for (j=0; j<numParts; j++)  {
							subItems.push_back(*pStore);
							item++;
							pStore = info.data.dbgItems + item;
						}
						item--;
					}
					else {
						subItems.push_back(*pStore);
					}

					CHcsmDebugItem   it;
					it.load(subItems);
					DebugItems.push_back(it);
				}
			}
			return true;
		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "no reply to GETDEBUGDATA";
			return false;
		}
	}
	else {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "could not send GETDEBUGDATA";
		return false;
	}
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
CHcsmClient::TakeControlOfObj(int ObjId, const TRemCntrlCmd &InitialCmd)
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
CHcsmClient::ReleaseControlOfObj(int ObjId)
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
CHcsmClient::ControlObj(int ObjId, const TRemCntrlCmd &Cmd)
{
	TMessage  msg;

	// while in synchronous mode we "talk" to the hcsmexec program
	// but if we are asynchronous mode we talk to the gateway HCSM.
	SOCKET sock;
	if ( m_Mode == ESynchronous ) {
		sock = m_CmdSock;
	}
	else
	if ( m_Mode == EFreeRunning ) {
		if ( m_DataSock == -1 ) {
			if ( ! ConnectToDataSock() ) {
				m_ErrorCode = EModeSocketComm;
				m_ErrorMsg  = "could not connect to data socket";
				return false;
			}
		}
		sock = m_DataSock;
	}
	else {
		m_ErrorCode = EModeError;
		m_ErrorMsg = "mode has not been specified";
		return false;
	}


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

	if ( !SendMessage(sock, CMD_CONTROLOBJ, &msg.data, 3 * sizeof(int)))  {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg = "Could not send message.";
		return false;
	}
	else {
		TMessage dummy;

		if ( RecvMessage(sock, dummy) > 0  ) {
			return true;
		}
		else {
			m_ErrorCode = EModeSocketComm;
			m_ErrorMsg = "Could not receive ack message.";
			return false;
		}
	}
}


bool
CHcsmClient::ConnectToDataSock(void)
{
	m_DataSock = ConnectToSocket(m_IpAddr, DEFAULT_PORT+1);
	if ( m_DataSock < 0  ) {
		m_ErrorCode = EModeSocketComm;
		m_ErrorMsg  = "Cannot connect to data socket (is HCSM running?)";
		return false;
	}
	return true;
}
