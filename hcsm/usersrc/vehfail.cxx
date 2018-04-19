/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: vehfail.cxx,v 1.16 2012/10/04 15:31:21 IOWA\dheitbri Exp $
 *
 * Author:  Jillian Vogel, Yiannis Papelis
 *
 * Date:    January, 2000
 *
 * Description:  Contains code for the vehicle failure object
 *               (VehFail HCSM).
 *
 ****************************************************************************/
#include "../hcsmcollection.h"
#include "genhcsm.h"
#include "vehfailclient.h"


#ifdef __sgi
#include <strstream.h>
#include <sys/types.h>
#include <netinet/in.h>
#define ostringstream ostrstream
#elif _PowerMAXOS
#include <ctype.h>
#include <strstream>
#include <sys/types.h>
#include <netinet/in.h>
#define ostringstream ostrstream
#elif _WIN32
#include <sstream>
#endif


void CVehFail::UserCreation( 
			const CVehFailParseBlock* pSnoBlock
			)
{

	// print creation message with template name and hcsm id
	PrintCreationMessage();

	// Initialize windows sockets
#ifdef _WIN32
	WSAData wsaData;
	if ( WSAStartup(MAKEWORD(1, 1), &wsaData) != 0 ) {
		fprintf(stderr, 
			"Gateway:  cannot initialize Windows sockets; quiting ...\n");
		Suicide();
	}
#endif

	m_Socket = CreateNonBlockingSocket(VEH_FAILURES_PORT);
	m_CmdSock = -1;
	if ( m_Socket < 0 ) {
		fprintf(stderr, 
			"VehFail:  cannot create non-blocking socket; quiting...\n");
		Suicide();
	}
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Vehicle failure HCSM pre-activity
//
// Remarks:
// The function does the majority of the work for the
// vehicle failure HCSM.  Conceptually, the function collects all
// sources of vehicle failures and puts them in a queue.  Currently,
// there are two potential sources, the first is a TCP/IP socket connection
// from a client who may asynchronously send vehicle failure messages,
// and the second is by reading the dial on the HCSM.
//
// All failures are put in a queue and the post-activity is reponsible
// for implementing the failure.
//
void CVehFail::UserPreActivity( 
			const CVehFailParseBlock* pSnoBlock 
			)
{
	int        code;

	//
	// If there is no existing socket connection, check for one
	//
	if ( m_CmdSock < 0 ) { 
		code = PollSocketForConnection(m_Socket);
		if (code == 0 ) {  // no connection yet

		}
		else
		if ( code == INVALID_SOCKET) {
			fprintf(stderr, 
				"VehFailure: error while polling for connection; quiting...\n");
			Suicide();
		}
		else {
			m_CmdSock = code;
		}
	}

	TMsgHeader          msgHeader;

	if ( m_CmdSock >= 0 ) {
		code = ReadHeader(m_CmdSock, msgHeader);
		if ( code == 0 ) {	// no message waiting
		}
		else 
		if ( code < 0 ) {	// error
			CloseSocket(m_CmdSock);
			m_CmdSock = -1;
		}
		else {				// received a valid message
			TMessage                        msg;
			CVehFailClient::TVehFailure*    pFailure;

			if ( ReadMessage(m_CmdSock, msgHeader, msg) ) {
				SendMessage(m_CmdSock, 0, 0, 0);

				pFailure = (CVehFailClient::TVehFailure*) &msg.data;
				pFailure->m_type = (CVehFailClient::EVehFailureType)ntohl( pFailure->m_type );
				for (int n=0; n < 11; n++) {
					pFailure->m_flags[n] = ntohl( pFailure->m_flags[n] );
				}
				
				m_Failures.push(*pFailure);
			}
			else {
				fprintf(stderr, "Couldn't read whole message\n");
			}
		}
	}

	//
	// Now check the dial
	//

	if (m_dialFailure.HasValue()) {
		CVehFailClient::TVehFailure failure;
		string str = GetDialFailure();
		int type = CActionParseBlock::StringToFlags( str, failure.m_flags );
		failure.m_type = (CVehFailClient::EVehFailureType)type;
		m_Failures.push( failure );

		m_dialFailure.SetNoValue();
	}

}


void CVehFail::UserPostActivity( 
			const CVehFailParseBlock* pSnoBlock 
			)
{
	while (!m_Failures.empty()) {
		CVehFailClient::TVehFailure failure = m_Failures.front();
		m_Failures.pop();

		int n;
		switch ( failure.m_type ) {
			case CVehFailClient::eNOFAILURE:
				break;
			case CVehFailClient::eTIRE:
				for (n=0; n<10; n++)
					if ((int)failure.m_flags[n] != -1)
						CHcsmCollection::m_sTireCond[n] = failure.m_flags[n];
				break;
			case CVehFailClient::eBRAKE:
				for (n=0; n<11; n++)
					CHcsmCollection::m_sBrakeCond[n] = failure.m_flags[n];
				break;
			case CVehFailClient::eSHOCKS:
				break;
			case CVehFailClient::eSTEERING:
				CHcsmCollection::m_sSteeringCond = failure.m_flags[0];
				break;
			case CVehFailClient::eALERTS:
				CHcsmCollection::m_sAlertCond = failure.m_flags[0];
				break;
			case CVehFailClient::eINFORMATION:
				CHcsmCollection::m_sInfoCond = failure.m_flags[0];
				break;
			case CVehFailClient::eCABCOMPONENT:
				CHcsmCollection::m_sCabComponentCond = failure.m_flags[0];
				break;
		}
	}
}


void CVehFail::UserDeletion( 
			const CVehFailParseBlock* pSnoBlock
			)
{
	PrintDeletionMessage();

	if ( m_Socket >= 0 ) {
		CloseSocket(m_Socket);
	}

}
