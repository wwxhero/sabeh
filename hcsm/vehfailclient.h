//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2000 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: vehfailclient.h,v 1.9 2016/10/28 20:47:57 IOWA\dheitbri Exp $
//
// Author(s):    Yiannis Papelis
//
// Date:         January, 2000
// Description:  Declaration of the failclient library class.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef _VEHFAILCLIENT_H_
#define _VEHFAILCLIENT_H_

#include <cvedpub.h>
#ifdef _WIN32
#include <winhrt.h>
#endif
#include "hcsmcollection.h"
#include "hcsmconnect.h"
 
#ifdef _PowerMAXOS
#include <unistd.h>
#endif


#define VEH_FAILURES_PORT    5437

// The command code for sending a vehicle failure.  Since this is
// the only command, the actual value is irrelevant...
#define VEH_FAILURE_CMD  9921

///////////////////////////////////////////////////////////////////////////// 
/// This class allows any program to connect to the HCSM system and
/// communicate vehicle failures through a TCP/IP interface.
/// Through member functions of the class, a user program can connect
/// to the hcsm system and then send commands about various failures,
/// which then will be communicated to the appropriate simulator
/// subsystem for the actual implementation.
/// 
/// The receiving end is implemented within the VehFail HCSM, and
/// that HCSM should be running for this class to be able to
/// connect to it.
///
///\ingroup HCSM
/////////////////////////////////////////////////////////////////////////////
class CVehFailClient {
public:

	CVehFailClient();
	~CVehFailClient();


private:
	CVehFailClient(const CVehFailClient &);				// no user access
	CVehFailClient &operator=(const CVehFailClient &);	// no user access

	///////////////////////////////////
	//
	// public data structures and types
	//
	///////////////////////////////////

public:
	//
	// This enumeration describes the type of vehicle failure.
	// Associated with each type of failure is a set of variables
	// whose meaning depends on the failure type.  These variables
	// are collectively kept in CVehFailClient::TVehFailure structure.
	// Field descriptions and meanings are given in the NADS
	// failure injection working document.
	enum EVehFailureType {
		eNOFAILURE     = 0,
		eTIRE          = 1,	  // tire pressure too high or low
		eBRAKE         = 2,   // brake failure
		eSHOCKS        = 3,   // shock absorbers failing
		eSTEERING      = 4,   // steering failure
		eALERTS        = 5,   // various alerts, see flags for details
		eINFORMATION   = 6,   // information displays (tach/speed etc.
		eCABCOMPONENT  = 7    // lft/rgh signal, horn, fuel etc.
	};

	//
	// A structure to keep all information about vehicle failures.
	//
	struct TVehFailure {
		EVehFailureType   m_type;
		TU32b             m_flags[11];
	};

public:
	bool Connect( const string& ip );
	bool SendFailure(const TVehFailure &);
	bool Disconnect(void);
	void GetRecentErrorMsg(string &) const;
	bool Initialize();
	bool IsConnected();

private:
	bool ConnectToDataSock(void);

	string			m_ErrorMsg;		// most recent error message
	SOCKET          m_Socket;		// socket
	char            m_IpAddr[255];  // machine we are connected to
};


//////////////////////////////////////////////////////////////////////////////
//
// Description: get most recent error message
//
// Remarks:
// This function returns the error message associated with the most
// recently called member function.  If the most recently called function
// succeeded then the message will be empty.
//
// Args:
// msg - where to store the message
//
// Returns:
// True of the most recently called function failed, otherwise false.
//
//
inline void 
CVehFailClient::GetRecentErrorMsg(string &msg) const
{
	msg = m_ErrorMsg;
}



#endif		// #ifndef _VEHFAILCLIENT_H_

