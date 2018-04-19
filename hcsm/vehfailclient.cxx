//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2000 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: vehfailclient.cxx,v 1.7 2000/04/21 01:18:03 schikore Exp $
//
// Author(s):    Yiannis Papelis
//
// Date:         January, 1999
// Description:  Implemention of the CVehFailClient library class.
//
//////////////////////////////////////////////////////////////////////////////
#include "vehfailclient.h"

#ifdef __sgi
#include <strstream.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#define ostringstream ostrstream
#elif _PowerMAXOS
#include <ctype.h>
#include <strstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#define ostringstream ostrstream
#elif _WIN32
#include <sstream>
#endif

static char* pNoError = "No error";

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
CVehFailClient::CVehFailClient()
{
	m_ErrorMsg  = pNoError;
	m_Socket    = -1;
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
CVehFailClient::~CVehFailClient()
{
}


bool
CVehFailClient::Initialize()
{
#ifdef _WIN32
	// Initialize windows sockets
	WSAData wsaData;
	if ( WSAStartup(MAKEWORD(1, 1), &wsaData) != 0 ) {
		m_ErrorMsg = "cannot initialize windows sockets";
		return false;
	}
#endif
	return true;
}

//
//
//
bool
CVehFailClient::Connect(const string& ip)
{
	strcpy( m_IpAddr, ip.c_str() );

	m_Socket = ConnectToSocket(m_IpAddr, VEH_FAILURES_PORT);
	if ( m_Socket < 0  ) {
		m_ErrorMsg  = "Cannot connect to socket (is HCSM running?)";
		return false;
	}

	m_ErrorMsg = pNoError;
	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: send the specified failure to the HCSM system
//
// Remarks:
// This function sends the failure specified by the sole argument
// to the hcsm system.  Once received by the hcsm system, the failure
// command will be communicated to the VVS, which will dispatch it to
// the appropriate simulator subsystem(s) for action.  Since this
// function uses TCP/IP sockets to send the message, it cannot
// guarantee real-time performance, or bound the delay, however,
// the messages are very short and in a lightly loaded network,
// such messages can be transmitted in less than one millisecond.
//
//
// Arguments:
// Frames - the number of frames to execute
//
// Returns:
// The function returns true to indicate that the message was successfully
// sent.  This is verified by receiving an acknowledgment from
// the hcsm system.  If the function failes, it returns false.
//
bool
CVehFailClient::SendFailure(const TVehFailure &info)
{
	bool status = true;
	if ( m_Socket < 0 ) {
		m_ErrorMsg = "Not connected to the simulator";
		return false;
	}
	if (status) {
		TVehFailure sendInfo;
		sendInfo.m_type  = (CVehFailClient::EVehFailureType)htonl(info.m_type);
		for (int n = 0; n < 11; n++)
		sendInfo.m_flags[n] = htonl(info.m_flags[n]);
		
		if ( !SendMessage(m_Socket, VEH_FAILURE_CMD, &sendInfo, sizeof(info))) {
			m_ErrorMsg = "Could not send message.";
			status = false;
		}
		else {
			TMessage dummy;

			if ( RecvMessage(m_Socket, dummy) > 0  ) {
				m_ErrorMsg = pNoError;
				status = true;
			}
			else {
				m_ErrorMsg = "Could not receive ack message.";
				status = false;
			}
		}
	}

	if (!status) {
#ifdef _WIN32
		if (WSAGetLastError() != WSAEWOULDBLOCK) {
#else
		if (errno != EWOULDBLOCK) {
#endif
			Disconnect();
		}
	}
	return status;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: disconnect from the hcsm system
//
// Remarks:
// This function closes the connection to the hcsm system.  After
// calling this function, no more failures can be sent.
//
// Arguments:
//
// Returns:
// True to indicate success or false to indicate problems.
//
bool 
CVehFailClient::Disconnect(void)
{
	bool b;

	if ( m_Socket < 0 ) {
		m_ErrorMsg = "Not connected to the simulator";
		return false;
	}

	b = CloseSocket(m_Socket);

	if ( b ) {
		m_ErrorMsg = pNoError;
	}
	else {
		m_ErrorMsg = "Could not disconnect";
	}
	m_Socket = -1;
	return b;
}

bool
CVehFailClient::IsConnected()
{
	return (m_Socket >= 0);
}



