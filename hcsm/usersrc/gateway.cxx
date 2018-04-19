/***************************************************************************** *
 * (C) Copyright 1999 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: gateway.cxx,v 1.26 2016/10/28 20:56:06 IOWA\dheitbri Exp $
 *
 * Author:       Yiannis Papelis
 * Date:         August, 1999
 *
 * Description:  The code for the hcsm gateway.  It is responsible
 *               for opening a socket and listening for commands.
 *
 ****************************************************************************/
// #include "genhcsm.h"
// #include "hcsmcollection.h"
#include "hcsmpch.h"
#include "hcsmclient.h"

#ifdef _WIN32

#elif __sgi
#include <sys/types.h>
#include <netinet/in.h>

#elif _PowerMAXOS
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif


#undef DEBUG_GATEWAY

 //   		void DeleteWorker (const CGatewayParseBlock* pBlock );
 //  		void Worker(void);
// Temporary!!

static int  Socket     = -1;
static int  DataSock[] = { -1, -1, -1, -1, -1 };

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Function called upon the creation of a Gateway HCSM
//
// Remarks:  The function does no parsing as there are no arguments
//           to it.  Its primary responsibility is to create a new
//           non-blocking socket which can later be "polled" for connections.
//
// Arguments: none
//
// Returns:  void
//
//////////////////////////////////////////////////////////////////////////////
void CGateway::CreateWorker( const CGatewayParseBlock* )
{

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

	int s = CreateNonBlockingSocket(DEFAULT_PORT+1);
	if ( s < 0 ) {
		fprintf(stderr, 
			"Gateway:  cannot create non-blocking socket; quiting...\n");
		Suicide();
	}
	Socket = s;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Process messages from the specified socket
//
// Remarks:
// This function is responsible for reading any pending messages
// in the specified sockets and then implementing the commands
// from these messages.
//
void
CGateway::DispatchMessage(int sock)
{
	int                 code;
	TMsgHeader          msgHeader;

	while ( 1 ) {
		code = ReadHeader(sock, msgHeader);
		if ( code == 0 ) {	// no message waiting
			break;
		}

		if ( code < 0 ) {  // error occurred, print error and return
			fprintf(stderr, "CGW::PM: read header failed\n");
			break;
		}

		int    opcode    = msgHeader.m_OpCode;

		switch ( opcode ) {
			case CMD_GETDYNAOBJS :
				HandleGetDynaObjsMsg( sock, msgHeader );
				break;

			case CMD_CONTROLOBJ :
				HandleControlObjMsg( sock, msgHeader );
				break;

			default :
				fprintf(stderr, "CGW::PM: got unexpected message\n");
				break;
		}
	}
}


void
CGateway::HandleGetDynaObjsMsg(int sock, TMsgHeader &head)
{
	// This message has no additional data so no need to do additional
	// reads.


	TMessage   msg;
	vector<int>  dynobjs;         // where to store object idendifiers
	vector<int>  driver;
	vector<int>  lights;
	vector<int>  staticObjs;
	int          NumObjs;			// number of dynamic objects
	int          ObjsPerMessage;  // how many objects fit in a message
	int          NumMessages;     // how many messages we will have to send
	int          ObjsInLastMsg;   // objects in last message
	int          package[4];      // storage message sending

	NumObjs = cved->GetNumDynamicObjs();

	CObjTypeMask driverMask;
	driverMask.Clear();
	driverMask.Set( eCV_EXTERNAL_DRIVER );
	cved->GetAllObjs( driver, driverMask );

// if no objects, no messages are sent
//	if ( NumObjs == 0 ) return ;

	cved->GetAllDynamicObjs(dynobjs);
//	cerr << "Got " << dynobjs.size() << " Dynobjs." << endl;
	CObjTypeMask mask;
	mask.Clear();
	mask.Set( eCV_TRAFFIC_LIGHT );
	if (driver.size() == 1) {
		cvTObjState state;
		cved->GetObjState( driver.front(), state );
//		cved->GetObjsNear( state.anyState.position, 2000, lights, mask );
		cerr << "@@@@ SABEH:CGateway: GetObjsNear no longer supported in CVED" << endl;
		assert( 0 );
	} else {
		cved->GetAllObjs( lights, mask );
	}
	dynobjs.insert( dynobjs.end(), lights.begin(), lights.end() );
//	cerr << "Got " << lights.size() << " lights." << endl;
//	cerr << "Got " << dynobjs.size() << " total objs." << endl;

	int startStatic = dynobjs.size();

//	cved->GetAllRuntimeObjs( staticObjs );
	cerr << "@@@@ SABEH:CGateway: GetAllRuntimeObjs no longer supported in CVED" << endl;
	assert( 0 );
	dynobjs.insert( dynobjs.end(), staticObjs.begin(), staticObjs.end() );

	NumObjs = dynobjs.size();
#if 0
	for (vector<int>::iterator i = lights.begin(); i != lights.end(); i++) {
		cerr << "\tSolId: " << cved->GetObjSolId(*i) << endl;
		cerr << "\tHCSM Id: " << cved->GetObjHcsmId(*i) << endl;
		cerr << "\tObjType: " << cved->GetObjType(*i) << endl;
	}
#endif

	ObjsPerMessage = sizeof(msg.data.bytes) / sizeof(TObjDesc);
	if ( (NumObjs % ObjsPerMessage) == 0 ) {
		NumMessages    = NumObjs / ObjsPerMessage;
		ObjsInLastMsg  = ObjsPerMessage;
	}
	else {
		NumMessages    = 1 + NumObjs / ObjsPerMessage;
		ObjsInLastMsg  = NumObjs % ObjsPerMessage;
	}

#if 0
	cerr << "In ProcessGetDynaObjs, " << endl
			<< "    sizeof(TObjDesc) ="<< sizeof( TObjDesc ) << endl
			<< "    NumObjs        = " << NumObjs << endl
			<< "    ObjsPerMessage = " << ObjsPerMessage << endl
			<< "    NumMessages    = " << NumMessages << endl
			<< "    ObjsInLastMsg  = " << ObjsInLastMsg << endl << flush;
#endif

// i1 - number of dynamic objects
// i2 - number of messages that will be send back
// i3 - number of objects per message, for all but the last message
// i4 - number of objects in the last message.
	package[0] = htonl(NumObjs);
	package[1] = htonl(NumMessages);
	package[2] = htonl(ObjsPerMessage);
	package[3] = htonl(ObjsInLastMsg);

#ifdef DEBUG_GATEWAY
	printf("Size of package is %d\n", sizeof(package));
#endif
	if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
		cerr << "Could not send message with size info" << endl;
		return ;
	}

	vector<int>::const_iterator pI;
	int num = 0;

	int  ObjsInMessage = 0;
	int objNum = 0;
	for (pI = dynobjs.begin(); pI != dynobjs.end(); pI++) {

#ifdef DEBUG_GATEWAY
		cerr << "  Object " << ObjsInMessage << endl;
#endif
		// fill in the data
		cvEObjType objType = cved->GetObjType( *pI );
		msg.data.objs[ObjsInMessage].solId  = htonl(cved->GetObjSolId(*pI));
		msg.data.objs[ObjsInMessage].hcsmId = htonl(cved->GetObjHcsmId(*pI));
		msg.data.objs[ObjsInMessage].type   = (cvEObjType)htonl(objType);
		msg.data.objs[ObjsInMessage].id     = htonl( *pI );
		if ((objNum >= startStatic) || (objType == eCV_TRAFFIC_LIGHT)) {
			// these objects have no hcsm id
//			msg.data.objs[ObjsInMessage].hcsmId = htonl( *pI );
			msg.data.objs[ObjsInMessage].hcsmId = htonl( -1 );
		}
		msg.data.objs[ObjsInMessage].name[0] = '\0';

		cvTObjState state;
		cved->GetObjState(*pI, state);
		CvedDataToCommData( state, msg.data.objs[ObjsInMessage].state, objType, cved, *pI );

#ifdef DEBUG_GATEWAY
		cerr << "      Sol:    " << msg.data.objs[ObjsInMessage].solId << endl;
		cerr << "      hcsmId: " << msg.data.objs[ObjsInMessage].hcsmId << endl;
		cerr << "      type:   " << msg.data.objs[ObjsInMessage].type<< endl;
		cerr << "      Pos:    " << 
			msg.data.objs[ObjsInMessage].state.position.x << ", " <<
			msg.data.objs[ObjsInMessage].state.position.y << ", " <<
			msg.data.objs[ObjsInMessage].state.position.z << endl;
#endif

		ObjsInMessage++;
		objNum++;

		if ( ObjsInMessage == ObjsPerMessage ) {
			int attempts = 0;
#ifdef DEBUG_GATEWAY
			cerr << "Sending message of size " << sizeof(msg.data) << endl;
#endif
			while ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
				cerr << "Could not send message with obj info" << endl;
				attempts++;
				if (attempts > 10)
					break;
//				return ;
			}
			ObjsInMessage = 0;
		}
		num++;
	}

	if ( ObjsInMessage != 0 ) { 	// must send last message
#ifdef DEBUG_GATEWAY
		cerr << "Sending message of size " << sizeof(msg.data) << endl;
#endif
		if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
			cerr << "Could not send last message with obj info" << endl;
			return ;
		}
	}
#if 0
	if ( SendMessage(sock, CMD_ACKOK, 0, 0) ) 
		return ;
	else
		return ;
#endif

}


void 
CGateway::HandleGetInstObjsMsg(int sock, TMsgHeader &head)
{
}


void 
CGateway::HandleTakeObjControlMsg(int sock, TMsgHeader &head)
{
}

void 
CGateway::HandleControlObjMsg(int sock, TMsgHeader &head)
{
	// Need to read the message body now
	TMessage message;
	ReadMessage( sock, head, message );

	SendMessage( sock, CMD_ACKOK, 0, 0 );

	int*        pIntData = &message.data.ints[0];

	CHcsm* pHcsm = m_pRootCollection->GetHcsm( ntohl(pIntData[1]) );

	if (!pHcsm) return;

	switch ( ntohl(pIntData[0]) ) {
	case CHcsmClient::eTURN_LEFT:
		pHcsm->SetButtonByName( "TurnLeft" );
		break;
	case CHcsmClient::eTURN_RIGHT:
		pHcsm->SetButtonByName( "TurnRight" );
		break;
	case CHcsmClient::eTURN_STRAIGHT:
		pHcsm->SetButtonByName( "GoStraight" );
		break;
	case CHcsmClient::eCHANGE_LANES_LEFT:
		pHcsm->SetButtonByName( "ChangeLaneLeft" );
		break;
	case CHcsmClient::eCHANGE_LANES_RIGHT:
		pHcsm->SetButtonByName( "ChangeLaneRight" );
		break;
	case CHcsmClient::eFORCE_VEL:
		pHcsm->SetDialByName( "ForcedVelocity", (int)ntohl(pIntData[2]) );
		break;
	case CHcsmClient::eMAX_VEL:
		pHcsm->SetDialByName( "TargetVelocity", (int)ntohl(pIntData[2]) );
		break;

	default:
		break;
	}
}

void 
CGateway::HandleReleaseObjCntrlMsg(int sock, TMsgHeader &head)
{
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Function called at each time step during the gateway's 
//				lifetime.
//
// Remarks:  Not sure what this function has to do. It's here just in case
//
// Arguments: None
//
// Returns:  void
//
//////////////////////////////////////////////////////////////////////////////
void CGateway::Worker( void )
{
	int        code;
	int        i;

	//
	// First monitor "primary" socket for new connections
	//
	code = PollSocketForConnection(Socket);
	if (code == 0 ) {  // no connection yet
	}
	else
	if ( code < 0 ) {
		fprintf(stderr, 
			"Gateway:  error while polling for a connection; quiting...\n");
		Suicide();
	}
	else {
		fprintf(stderr, "Gateway:  got a connection.\n");

		bool foundOne = false;
		for (i=0; i<sizeof(DataSock) / sizeof(DataSock[0]); i++) {
			if ( DataSock[i] == -1 ) {
				DataSock[i] = code;
				foundOne    = true;
				break;
			}
		}
		if (  ! foundOne ) {
			fprintf(stderr, "Gateway:  too many connections");
			Suicide();
		}
	}

	//
	// Now go through all the open connections and deal with 
	// any messages.
	//
	for (i=0; i<sizeof(DataSock) / sizeof(DataSock[0]); i++) {
		if ( DataSock[i] == -1 )  continue;

		DispatchMessage(DataSock[i]);
	}

}



//////////////////////////////////////////////////////////////////////////////
//
// Description: Function called at the end of the Gateways's lifetime.
//
// Remarks: This function cleans up the sockets
//
// Arguments: None
//
// Returns:  
//
//////////////////////////////////////////////////////////////////////////////
void CGateway::DeleteWorker( const CGatewayParseBlock* )
{
	int i;

	if ( Socket != -1 ) {
		if (!CloseSocket(Socket) ) {
			printf("Gateway:CloseSocket(%d) failed.\n", Socket);
		}
		Socket = -1;
	}

	for (i=0; i < sizeof(DataSock) / sizeof(DataSock[0]); i++ ) {
		if (DataSock[i] != -1) {
			if (!CloseSocket(DataSock[i]) ) {
				printf("Gateway: CloseSocket(%d) failed.\n", DataSock[i]);
			}
			DataSock[i] = -1;
		}
	}
	PrintDeletionMessage();
}


