/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1999 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmexec.cxx,v 1.38 2004/04/01 02:01:00 oahmad Exp $
// Author:       Yiannis Papelis
// Date:         August, 1999
//
// Description:  The HCSM client program that controls the hcsm execution
// in conjunction with the library that allows using the HCSM system 
// within the ISAT rehearsal mode
//
/////////////////////////////////////////////////////////////////////////////
#include <cvedpub.h>
#ifdef _WIN32
#include <winhrt.h>

#elif __sgi
#include <sys/types.h>
#include <netinet/in.h>

#elif _PowerMAXOS
#include <sys/types.h>
#include <netinet/in.h>
#endif


#include <hcsmcollection.h>
#include <hcsmspec.h>
#include <snoparse.h>
#include <hcsmclient.h>
#include <hcsmconnect.h>
#include <staticobjmanagerutil.h>

typedef CHcsmCollection* CHcsmCollectionPtr;

static short m_Timer;

#undef _DEBUG_ON_
#undef _SHOW_TIMING_

/////////////////////////////////////////////////////////////////////////////
//
// PING command processing.  Simply send back an empty message 
// containing the PING command code
//
static bool
ProcessPing(int sock, const TMsgHeader &head)
{
	if ( !SendMessage(sock, CMD_PING, 0, 0) ) {
#ifdef _DEBUG_ON_
		fprintf(stderr, "Could not send PING command");
#endif
		return false;
	}
	else {
		return true;
	}
}


/////////////////////////////////////////////////////////////////////////////
//
// Processes the "SendScenario" command.  There are two versions,
// the first one, where the command code is CMD_SCENSCRIPT, contains
// the whole scenario file in one message, the second, whose command
// code is CDM_SCENSCRIPTMULTI is used when the message is one part
// of a multiple message sequence to support files larger than the
// message size.
//
// Arguments:
//   sock  - the command socket
//   head  - the header of the incoming message
//   script - the string containing the SNO file
//
static bool
ProcessScript(int sock, const TMsgHeader &head, string &script)
{
	TMsgHeader more;
	TMessage   msg;

	//
	// Read the remainder of the message
	//
	unsigned int i;
	if ( !ReadMessage(sock, head, msg) ) return false;

	// clear the script variable
	script = string("");		// works on the Concurrent, others don't

	more = head;
	//
	// process the message; if the regular command, save the file
	// and we are done.  If the multi, read the remaining messages
	//
	if ( more.m_OpCode == CMD_SCENSCRIPT ) {
		// add character by character
		for (i=0; i<more.m_MsgLen - sizeof(more); i++) {
			char letter[2] = { msg.data.bytes[i], '\0' };
			script += string(letter);
		}
	}
	else
	// we have to keep reading messages
	if ( more.m_OpCode == CMD_SCENSCRIPTMULTI ) {

		while ( more.m_OpCode == CMD_SCENSCRIPTMULTI ) {
			for (i=0; i<more.m_MsgLen - sizeof(more); i++) {
				char letter[2] = { msg.data.bytes[i], '\0' };
				script += string(letter);
			}

			if ( !ReadHeader(sock, more) ) {
				return false;
			}
			if ( !ReadMessage(sock, more, msg) ) {
				return false;
			}
		}

		//
		// Once a SCENSCRIPTMULTI has been sent, the
		// last part of the file should come on a
		// SCENSCRIPT message.  If not, the other side
		// made a mistake so we check for that here.
		//
		if ( more.m_OpCode != CMD_SCENSCRIPT ) {
#ifdef _DEBUG_ON_
			cerr << "Hcsmexec:"
				<< "expected CMD_SCENSCRIPT but got " 
				<< more.m_OpCode << endl;
#endif
			return false;
		}

		for (i=0; i<more.m_MsgLen - sizeof(more); i++) {
			char letter[2] = { msg.data.bytes[i], '\0' };
			script += string(letter);
		}
	}
	else {
#ifdef _DEBUG_ON_
		cerr << "Hcsmexec: internal error " << __FILE__ << __LINE__ << endl;
#endif
		return false;
	}

	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Process the SimParams command.  Two double precision numbers
// are delivered, the first is the simulation frequency in Hz
// and the second is the # of times to run the dynamics per frame.
// The second number is naturally an integer but is sent as
// a double to simplify things.  Immediately after these numbers
// is the LriDirectory, as a string.
//
static bool
ProcessSimParams(int sock, const TMsgHeader &head, 
			double &f, int &n, string &LriDir, string& ScnDir)
{
	TMessage   msg;

	//
	// Read the remainder of the message
	//
	if ( !ReadMessage(sock, head, msg) ) return false;

	f = msg.data.doubles[0];
	n = (int)msg.data.doubles[1];

	char *pLriDir;
	pLriDir = (char *)&msg.data.doubles[2];
	LriDir  = string(pLriDir);

	char *pScnDir;
	pScnDir = (char *)&msg.data.bytes[256];
	ScnDir  = string(pScnDir);

	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Process the RunFrames command.  The data for this command is 
// an integer (4 byte) that indicates how many frames to run along with
// a second integer indicating how many iterations of the dynamics to run.
// The following table shows the interpretation of this two numbers:
//
// fr < 0,   dr > 0   => run dynamics only, dr times
// fr > 0,   dr = any => run behaviors and dynamics as per the initialization
// fr = 0,   dr = any => run behaviors and no dynamics
//
// If all works ok, the return message is an ACK
//
static bool
ProcessRunFrames(
	int                  sock,           // command socket
	const TMsgHeader&    head,           // message header
	int                  DynaRate,       // dynamics rate rel to behaviors
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg        // contains explanation about error
	)
{
	// read the remainder of the message
	TMessage msg;

	if ( !ReadMessage(sock, head, msg) ) return false;

	int numFrames = msg.data.ints[0];
	int numDyna   = msg.data.ints[1];

	if ( numFrames <= 0 && numDyna > 0 ) {
		int i;
		for (i=0; i<numDyna; i++) {
			Cved.ExecuteDynamicModels();
		}
	}
	else 
	if ( numFrames == 0 ) {
		pHcsm->ExecuteAllHcsm();
		Cved.Maintainer();
	}
	else {
		while ( numFrames-- ) {
			int i;

			for (i=0; i<DynaRate; i++) {
				Cved.ExecuteDynamicModels();
			}
			pHcsm->ExecuteAllHcsm();
			Cved.Maintainer();
		}
	}

	if ( SendMessage(sock, CMD_ACKOK, 0, 0) ) 
		return true;
	else
		return false;
}


/////////////////////////////////////////////////////////////////////////////
//
// Process the StartScenario command.  Load CVED, start a new
// hcsm collection class and load the scenario file.  Return
// a text error message if things don't go well.  Also, the
// caller expects a message back to indicate the status so we
// have to send a message (ACKOK or ACKERROR) before returning or
// the caller will hang.
//
// Arguments:  see argument list.
//
static bool
ProcessStartScenario(
	int                  sock,           // command socket
	const TMsgHeader&    head,           // message header
	string &             SnoScript,      // the scenario script
	string &             LriDir,         // where LRI files are stored
	double &             ExecFrequ,      // hcsm execution frequency
	int    &             DynamicsRate,   // exec rate of dynamics servers
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg,       // contains explanation about error
	string &			 ScnDir			 // where the SCN file is stored (for external references)
)
{
	//
	// parse the scenario script first
	// 
	CSnoParser             parser;
	parser.SetFilename( ScnDir );
	CSnoParser::TIterator  pBlock;
	if ( !parser.Parse(SnoScript) ) {
		ErrorMsg = "Cannot parse script";
		SendMessage(sock, CMD_ACKERROR, ErrorMsg.c_str(), ErrorMsg.length()+1);
		return false;
	}

	//
	// Initialize cved.  First we find the lri name from the 
	// sno parser, then combine it with the expected path and use
	// it along with the other parameters to initialize CVED.
	//
	string lri;

	pBlock = parser.Begin();
	if ( pBlock == parser.End() || pBlock->GetBlockName() != string("Header")) {
		ErrorMsg = "Cannot find header in the scenario script";
		SendMessage(sock, CMD_ACKERROR, ErrorMsg.c_str(), ErrorMsg.length()+1);
		return false;
	}

	CHeaderParseBlock hdrBlk(*pBlock);	
#ifdef _DEBUG_ON_
//	cout << "LRI file is " << hdrBlk.LriFile() << endl;
	cout << "LRI dir is " << LriDir << endl;
#endif

	if ( LriDir.length() == 0 ) {
		lri = hdrBlk.GetLriFile();
	}
	else
	if ( LriDir[LriDir.length()-1] != '/' ) {
		cout << "Check" << endl;
		lri = LriDir + "/";
		lri += hdrBlk.GetLriFile();
	}
	else {
		lri = LriDir + hdrBlk.GetLriFile();
	}

#ifdef _DEBUG_ON_
	cout << "Full LRI name is " << lri;
#endif
	cout << "ExecFrequ is " << ExecFrequ << endl;
	cout << "DynamicsRate is " << DynamicsRate << endl;

	Cved.Configure( CCved::eCV_SINGLE_USER, 1.0 / ExecFrequ, DynamicsRate);

	if ( ! Cved.Init(lri, ErrorMsg) ) {
		SendMessage(sock, CMD_ACKERROR, ErrorMsg.c_str(), ErrorMsg.length()+1);
		return false;
	}

	if ( hdrBlk.HasOwnVeh() ) {
		//
		// This SCN file has an external driver.  Make an ADO that will
		// represent the ExternalDriver in this scenario.
		//
		CAdoParseBlock block;
		block.SetName( "ExternalDriver" );
		block.SetSolName( "ChevyBlazerRed" );
		CRoadPos pos( Cved, hdrBlk.GetOwnVehPos() );
		if( pos.IsValid() ) 
		{
			block.SetRoadPos( pos.GetString() );
			block.SetPath( hdrBlk.GetPath() );
			parser.AddBlock( block );
		}
	}

	//
	// Create the hcsm collection and start all the HCSMs
	//
	pHcsm = new CHcsmCollection( 1.0 / ExecFrequ, &Cved);

	pBlock = parser.Begin();
	pBlock++;		// skip the header
	for (  ; pBlock != parser.End(); pBlock++) {
		CHcsm* pH;
		if ( (pH = pHcsm->CreateHcsm( pBlock->GetBlockName(), *pBlock )) == 0 ) {
			char buf[400];

			sprintf(buf, "CHcsmCollection::CreateHcsm failed on '%s'",
						pBlock->GetBlockName().c_str());
			ErrorMsg = buf;
			SendMessage(sock, CMD_ACKERROR, ErrorMsg.c_str(), 
											ErrorMsg.length()+1);
			return false;
		}
cout << "Creating " << pBlock->GetBlockName() << endl;
		if ( "StaticObjManager" == pH->GetName() ) {
			CSobjMngrParseBlock block( *pBlock );
			
			StaticObjManInitialSetup( &block, Cved, *pHcsm, pH );
		}
	}

	if ( SendMessage(sock, CMD_ACKOK, 0, 0) ) 
		return true;
	else
		return false;
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
// Process the GetDynamic Objects command.
//
// Arguments:  see argument list.
//
static bool
ProcessGetDynaObjs(
	int                  sock,           // command socket
	const TMsgHeader&    head,           // message header
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg        // contains explanation about error
)
{
	// This message has no additional data so no need to do additional
	// reads.

	TMessage   msg;
	vector<int>  dynobjs;         // where to store object idendifiers
	vector<int>  lights;         // where to store object idendifiers
	vector<int>  staticObjs;         // where to store object idendifiers
	int          NumObjs;			// number of dynamic objects
	int          ObjsPerMessage;  // how many objects fit in a message
	int          NumMessages;     // how many messages we will have to send
	int          ObjsInLastMsg;   // objects in last message
	int          package[4];      // storage message sending

	NumObjs = Cved.GetNumDynamicObjs();
	CObjTypeMask mask;
	mask.Clear();
	mask.Set( eCV_TRAFFIC_LIGHT );
	Cved.GetAllObjs( lights, mask );
	NumObjs += lights.size();

	if ( NumObjs == 0 ) {
		package[0] = 0;
		package[1] = 0;
		package[2] = 0;
		package[3] = 0;

		if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
#ifdef _DEBUG_ON_
			cerr << "Could not send message with size info (all 0's)" << endl;
#endif
			return false;
		}
		else {
			return true;
		}
	}

	Cved.GetAllDynamicObjs(dynobjs);
	dynobjs.insert( dynobjs.begin(), lights.begin(), lights.end() );

	int startStatic = dynobjs.size();

//	Cved.GetAllRuntimeObjs( staticObjs );
	cerr << "@@@@ Hcsmexec:ProcessGetDynaObjs: CVED doesn't support GetAllRuntimeObjs" << endl;
	assert( 0 );
	dynobjs.insert( dynobjs.end(), staticObjs.begin(), staticObjs.end() );
	NumObjs += staticObjs.size();

	int numStatic = staticObjs.size();

	int singleObjSize = sizeof( TObjDesc );
	int maxPacketSize = sizeof( msg.data.bytes );

	if ( singleObjSize > maxPacketSize ) {

		cerr << "Hcsmexec: message packets need to be a ";
		cerr << "larger size....exit" << endl;
		cerr << "  current size = " << maxPacketSize << endl;
		cerr << "  required minimum size = " << singleObjSize << endl;

		exit( -1 );

	}

	ObjsPerMessage = sizeof(msg.data.bytes) / sizeof(TObjDesc);
	if ( (NumObjs % ObjsPerMessage) == 0 ) {
		NumMessages    = NumObjs / ObjsPerMessage;
		ObjsInLastMsg  = ObjsPerMessage;
	}
	else {
		NumMessages    = 1 + NumObjs / ObjsPerMessage;
		ObjsInLastMsg  = NumObjs % ObjsPerMessage;
	}

#ifdef _DEBUG_ON_
	cerr << "In ProcessGetDynaObjs, " << endl
			<< "    NumObjs = " << NumObjs << endl
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
	if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
		cerr << "Could not send message with size info" << endl;
		return false;
	}

	vector<int>::const_iterator pI;

	int  ObjsInMessage = 0;
	int  objNum = 0;
	for (pI = dynobjs.begin(); pI != dynobjs.end(); pI++) {

#if 0
printf("---> object id is %d\n", *pI);
#endif

		// fill in the data
		msg.data.objs[ObjsInMessage].id     = htonl(*pI);
		msg.data.objs[ObjsInMessage].solId  = htonl(Cved.GetObjSolId(*pI));
		msg.data.objs[ObjsInMessage].hcsmId = htonl(Cved.GetObjHcsmId(*pI));
		msg.data.objs[ObjsInMessage].type   = (cvEObjType)htonl(Cved.GetObjType(*pI));
		strcpy( msg.data.objs[ObjsInMessage].name, Cved.GetObjName(*pI) );
		if ((objNum >= startStatic) || (Cved.GetObjType(*pI) == eCV_TRAFFIC_LIGHT)) {
			// these objects have no hcsm id
//			msg.data.objs[ObjsInMessage].hcsmId = htonl( *pI );
			msg.data.objs[ObjsInMessage].hcsmId = htonl( -1 );
		}
		cvTObjState state;
		Cved.GetObjState(*pI, state);
		CvedDataToCommData( state, msg.data.objs[ObjsInMessage].state, Cved.GetObjType(*pI), &Cved, *pI );

		ObjsInMessage++;
		objNum++;

		if ( ObjsInMessage == ObjsPerMessage ) {
#if 0
printf(" *** Sending a message with %d objects\n", ObjsInMessage);
#endif
			if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
#ifdef _DEBUG_ON_
				cerr << "Could not send message with obj info" << endl;
#endif
				return false;
			}
			ObjsInMessage = 0;
		}
	}

	if ( ObjsInMessage != 0 ) { 	// must send last message
#if 0
printf(" *** Flushing last message with %d objects\n", ObjsInMessage);
#endif
		if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
#ifdef _DEBUG_ON_
			cerr << "Could not send last message with obj info" << endl;
#endif
			return false;
		}
	}
	return true;
}

static bool
ProcessGetChangedStaticObjs(
	int                  sock,           // command socket
	const TMsgHeader&    head,           // message header
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg        // contains explanation about error
)
{
	TMessage     msg;
	vector<int>  runtimeObjs; 	  // runtime objects
	int          objsPerMessage;  // how many objects fit in a message
	int          numMessages;     // how many messages we will have to send
	int          objsInLastMsg;   // objects in last message
	int          package[4];      // storage message sending

	map<int, int> objs;
//	objs = Cved.GetStaticObjectsChanged();
	cerr << "@@@@ Hcsmexec:ProcessGetChangedStaticObjs: CVED doesn't support GetStaticObjectsChanged" << endl;
	assert( 0 );
	int numObjs = objs.size();
	
	if ( numObjs == 0 ) {
		package[0] = 0;
		package[1] = 0;
		package[2] = 0;
		package[3] = 0;

		if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
#ifdef _DEBUG_ON_
			cerr << "Could not send message with size info (all 0's)" << endl;
#endif
			return false;
		}
		else {
			return true;
		}
	}

	objsPerMessage = sizeof(msg.data.bytes) / sizeof(TObjDesc);
	if ( (numObjs % objsPerMessage) == 0 ) {
		numMessages    = numObjs / objsPerMessage;
		objsInLastMsg  = objsPerMessage;
	}
	else {
		numMessages    = 1 + numObjs / objsPerMessage;
		objsInLastMsg  = numObjs % objsPerMessage;
	}

#if 0
	cerr << "In ProcessGetChangedStaticObjs, " << endl
			<< "    numObjs = " << numObjs << endl
			<< "    ObjsPerMessage = " << objsPerMessage << endl
			<< "    NumMessages    = " << numMessages << endl
			<< "    ObjsInLastMsg  = " << objsInLastMsg << endl << flush;
#endif

// i1 - number of objects objects
// i2 - number of messages that will be send back
// i3 - number of objects per message, for all but the last message
// i4 - number of objects in the last message.
	package[0] = htonl(numObjs);
	package[1] = htonl(numMessages);
	package[2] = htonl(objsPerMessage);
	package[3] = htonl(objsInLastMsg);
	if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
#ifdef _DEBUG_ON_
		cerr << "Could not send message with size info" << endl;
#endif
		return false;
	}

	map<int, int>::const_iterator pI;

	int  objsInMessage = 0;
	for (pI = objs.begin(); pI != objs.end(); pI++) {
#ifdef _DEBUG_ON_
printf("---> object id is %d\n", *pI);
#endif
		// fill in the data
		msg.data.objs[objsInMessage].id     = pI->first;
		msg.data.objs[objsInMessage].solId  = Cved.GetObjSolId(pI->first);
		msg.data.objs[objsInMessage].hcsmId = Cved.GetObjHcsmId(pI->first);
		msg.data.objs[objsInMessage].type   = Cved.GetObjType(pI->first);
		strcpy( msg.data.objs[objsInMessage].name, Cved.GetObjName(pI->first) );
		msg.data.objs[objsInMessage].state.specific.optionNum = htonl(pI->second);
		objsInMessage++;


		if ( objsInMessage == objsPerMessage ) {
#ifdef _DEBUG_ON_
printf(" *** Sending a message with %d objects\n", objsInMessage);
#endif
			if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
#ifdef _DEBUG_ON_
				cerr << "Could not send message with obj info" << endl;
#endif
				return false;
			}
			objsInMessage = 0;
		}
	}

	if ( objsInMessage != 0 ) { 	// must send last message
#if 0
printf(" *** Flushing last message with %d objects\n", ObjsInMessage);
#endif
		if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
#ifdef _DEBUG_ON_
			cerr << "Could not send last message with obj info" << endl;
#endif
			return false;
		}
	}
	return true;

}

/////////////////////////////////////////////////////////////////////////////
//
// Process the GetInstanced Objects command.
//
// Arguments:  see argument list.
//
static bool
ProcessGetInstancedObjs(
	int                  sock,           // command socket
	const TMsgHeader&    head,           // message header
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg        // contains explanation about error
)
{
	// This message has no additional data so no need to do additional
	// reads.

	TMessage     msg;
	vector<int>  instObjs;        // where to store object idendifiers
	vector<int>  runtimeObjs; 	  // runtime objects
	int          NumObjs;		  // number of objects
	int          ObjsPerMessage;  // how many objects fit in a message
	int          NumMessages;     // how many messages we will have to send
	int          ObjsInLastMsg;   // objects in last message
	int          package[4];      // storage message sending

	vector<int>::const_iterator pO;

// 
// We are looking for the instanced objects, i.e., runtime
// objects that are not dynamic
//
	NumObjs = 0;
//	Cved.GetAllRuntimeObjs(runtimeObjs);
	cerr << "@@@@ Hcsmexec:ProcessGetInstancedObjs: CVED doesn't support GetAllRuntimeObjs" << endl;
	assert( 0 );
	for (pO=runtimeObjs.begin(); pO != runtimeObjs.end(); pO++) {
		if ( !Cved.IsDynObj(*pO) ) {
			instObjs.push_back(*pO);
			NumObjs++;
		}
	}

	if ( NumObjs == 0 ) {
		package[0] = 0;
		package[1] = 0;
		package[2] = 0;
		package[3] = 0;

		if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
#ifdef _DEBUG_ON_
			cerr << "Could not send message with size info (all 0's)" << endl;
#endif
			return false;
		}
		else {
			return true;
		}
	}


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
	cerr << "In ProcessGetInstObjs, " << endl
			<< "    NumObjs = " << NumObjs << endl
			<< "    ObjsPerMessage = " << ObjsPerMessage << endl
			<< "    NumMessages    = " << NumMessages << endl
			<< "    ObjsInLastMsg  = " << ObjsInLastMsg << endl << flush;
#endif

// i1 - number of objects objects
// i2 - number of messages that will be send back
// i3 - number of objects per message, for all but the last message
// i4 - number of objects in the last message.
	package[0] = NumObjs;
	package[1] = NumMessages;
	package[2] = ObjsPerMessage;
	package[3] = ObjsInLastMsg;
	if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
#ifdef _DEBUG_ON_
		cerr << "Could not send message with size info" << endl;
#endif
		return false;
	}

	vector<int>::const_iterator pI;

	int  ObjsInMessage = 0;
	for (pI = instObjs.begin(); pI != instObjs.end(); pI++) {
#ifdef _DEBUG_ON_
printf("---> object id is %d\n", *pI);
#endif
		// fill in the data
		msg.data.objs[ObjsInMessage].id     = *pI;
		msg.data.objs[ObjsInMessage].solId  = Cved.GetObjSolId(*pI);
		msg.data.objs[ObjsInMessage].hcsmId = Cved.GetObjHcsmId(*pI);
		msg.data.objs[ObjsInMessage].type   = Cved.GetObjType(*pI);
		strcpy( msg.data.objs[ObjsInMessage].name, Cved.GetObjName(*pI) );
		cvTObjState state;
		Cved.GetObjState(*pI, state);
		CvedDataToCommData( state, msg.data.objs[ObjsInMessage].state, msg.data.objs[ObjsInMessage].type, &Cved, *pI );
		ObjsInMessage++;


		if ( ObjsInMessage == ObjsPerMessage ) {
#ifdef _DEBUG_ON_
printf(" *** Sending a message with %d objects\n", ObjsInMessage);
#endif
			if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
#ifdef _DEBUG_ON_
				cerr << "Could not send message with obj info" << endl;
#endif
				return false;
			}
			ObjsInMessage = 0;
		}
	}

	if ( ObjsInMessage != 0 ) { 	// must send last message
#if 0
printf(" *** Flushing last message with %d objects\n", ObjsInMessage);
#endif
		if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
#ifdef _DEBUG_ON_
			cerr << "Could not send last message with obj info" << endl;
#endif
			return false;
		}
	}
	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Process the GetDebugData command.
//
// Arguments:  see argument list.
//
static bool
ProcessGetDebugData(
	int                  sock,           // command socket
	const TMsgHeader&    head,           // message header
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg        // contains explanation about error
)
{
	// This message has no additional data so no need to do additional
	// reads.

	TMessage     msg;
	int          NumItems;		   // number of objects
	int          ItemsPerMessage;  // how many objects fit in a message
	int          NumMessages;      // how many messages we will have to send
	int          ItemsInLastMsg;   // objects in last message
	int          package[4];       // storage message sending

	vector<CHcsmDebugItem>      debugItems;
// 
// We are looking for all messages currently in the queue.
// We first go through them to detemine how many messages we have
// to compose.  While we are doing this, we are also extracting
// the messages and we keep them in a vector for later usage
//
	CHcsmDebugItem   item;
	string           str;

	NumItems = 0;
	while ( pHcsm->GetDebugItem(item) ) {
		NumItems += 1;
		if ( item.GetType() == CHcsmDebugItem::eTEXT ) {
			item.GetText(str);
			NumItems += (strlen(str.c_str()) + 1 ) / DEBUG_TEXT_LEN;
		}
		debugItems.push_back(item);
	}

	if ( NumItems == 0 ) {
		package[0] = 0;
		package[1] = 0;
		package[2] = 0;
		package[3] = 0;

		if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
#ifdef _DEBUG_ON_
			cerr << "Could not send message with size info (all 0's)" << endl;
#endif
			return false;
		}
		else {
			return true;
		}
	}

	//
	// calculate number of messages and how many things per message
	//
	ItemsPerMessage = sizeof(msg.data.bytes) / sizeof(CHcsmDebugItem::TItem);
	if ( (NumItems % ItemsPerMessage) == 0 ) {
		NumMessages    = NumItems / ItemsPerMessage;
		ItemsInLastMsg  = ItemsPerMessage;
	}
	else {
		NumMessages    = 1 + NumItems / ItemsPerMessage;
		ItemsInLastMsg  = NumItems % ItemsPerMessage;
	}

#if 0
	cerr << "In ProcessGetDebugData, " << endl
			<< "    NumItems        = " << NumItems << endl
			<< "    ItemsPerMessage = " << ItemsPerMessage << endl
			<< "    NumMessages     = " << NumMessages << endl
			<< "    ItemsInLastMsg  = " << ItemsInLastMsg << endl << flush;
#endif

	// send the package with information on how many message are coming
	package[0] = NumItems;
	package[1] = NumMessages;
	package[2] = ItemsPerMessage;
	package[3] = ItemsInLastMsg;
	if ( !SendMessage(sock, CMD_ACKOK, package, sizeof(package)) ) {
#ifdef _DEBUG_ON_
		cerr << "Could not send message with size info" << endl;
#endif
		return false;
	}

	// now we go through the vector where we stored the items, and we package
	// them into messages and each time a message structure gets filled up
	// we sent it.

	int ItemsInMessage = 0;
	vector<CHcsmDebugItem>::const_iterator pI;

	for (pI = debugItems.begin(); pI != debugItems.end(); pI++) {
		vector<CHcsmDebugItem::TItem>  netItems;
		vector<CHcsmDebugItem::TItem>::const_iterator pItms;

		pI->store(netItems);
		
		for (pItms = netItems.begin(); pItms != netItems.end(); pItms++) {
			msg.data.dbgItems[ItemsInMessage] = *pItms;
			ItemsInMessage++;
			if ( ItemsInMessage == ItemsPerMessage ) {
				if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
					return false;
				}
				ItemsInMessage = 0;
			}
		}
	}

	if ( ItemsInMessage != 0 ) { 	// must send last message
#if 0
printf(" *** Flushing last message with %d objects\n", ObjsInMessage);
#endif
		if ( !SendMessage(sock, CMD_ACKOK, &msg.data, sizeof(msg.data))) {
#ifdef _DEBUG_ON_
			cerr << "Could not send last message with obj info" << endl;
#endif
			return false;
		}
	}
	return true;
}



/////////////////////////////////////////////////////////////////////////////
//
//
static bool
ProcessSetDebugMode(
	int                  sock,           // command socket
	const TMsgHeader&    head,           // message header
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg        // contains explanation about error
)
{
	// read the remainder of the message
	TMessage msg;

	if ( !ReadMessage(sock, head, msg) ) return false;

	int                      numHcsms;
	CHcsmClient::TDebugMode  Mode;
	EDebugMode               dbgMode;
	bool                     ApplyToAll;
	CHcsmDebugItem::ELevel   dbgLevel;

	Mode        = (CHcsmClient::TDebugMode)msg.data.shorts[0];
	dbgLevel    = (CHcsmDebugItem::ELevel)msg.data.shorts[1];
	numHcsms    = msg.data.shorts[2];
	ApplyToAll  = (numHcsms == 1) && (msg.data.shorts[3] == -1);

	char *s;
	switch ( Mode ) {
		case CHcsmClient::ENoDebug : 
			s       = "NoDebug"; 
			dbgMode = eDEBUG_NONE;
			break;

		case CHcsmClient::ETextDebug : 
			s       = "TextDebug"; 
			dbgMode = eDEBUG_TEXT;
			break;

		case CHcsmClient::EGraphicsDebug : 
			s       = "GraphicsDebug"; 
			dbgMode = eDEBUG_GRAPHICS;
			break;

		case CHcsmClient::EGraphicsTextDebug : 
			s       = "Text&GraphicsDebug"; 
			dbgMode = eDEBUG_TEXT_AND_GRAPHICS;
			break;

		default  : 
			s       = "Invalid debug mode"; 
			dbgMode = eDEBUG_NONE;
			break;
	}

#ifdef _DEBUG_ON_
	printf("SetDebugMode: Mode = %s\n", s);
	printf("SetDebugLevel: Level = %d", (int)dbgLevel);

	if ( ApplyToAll ) {
		printf(" applied to ALL HCSMs.\n");
	}
	else {
		printf(" applied to these %d HCSMS: ", numHcsms);
		int i;

		for (i=0; i<numHcsms; i++) {
			printf("%d ", (int) msg.data.shorts[i+3]);
		}
		printf("\n");
	}
#endif

	if ( ApplyToAll ) {
		pHcsm->SetHcsmDebugMode(dbgMode);
		pHcsm->SetHcsmDebugLevel(dbgLevel);
	}
	else {
		int i;
		for (i=0; i<numHcsms; i++) {
			pHcsm->SetHcsmDebugMode(msg.data.shorts[i+3], dbgMode);
			pHcsm->SetHcsmDebugLevel(msg.data.shorts[i+3], dbgLevel);
		}
	}

	if ( SendMessage(sock, CMD_ACKOK, 0, 0) ) 
		return true;
	else
		return false;
}

/////////////////////////////////////////////////////////////////////////////
//
// Deals with the end scenario command.  Terminate all the HCSMs
// and leave CVED in its initial state
//
static bool
ProcessEndScenario(int sock, 
	const TMsgHeader &   msgHeader,
	CCved  &             Cved,           // cved instance used for behaviors
	CHcsmCollectionPtr & pHcsm,          // the hcsm collection
	string &             ErrorMsg        // contains explanation about error
)
{
	if ( pHcsm ) pHcsm->DeleteAllHcsm();
	if ( SendMessage(sock, CMD_ACKOK, 0, 0) ) 
		return true;
	else
		return false;
}


/////////////////////////////////////////////////////////////////////////////
//
// Deals with the quit command.  Terminate HCSM and exit.
//
static bool
ProcessQuit(int sock, CHcsmCollectionPtr & pHcsm)
{
	if ( pHcsm ) pHcsm->DeleteAllHcsm();
	(void)SendMessage(sock, CMD_ACKOK, 0, 0);
	CloseSocket(sock);

	cout << "Hcsmexec received quit command.  All done.!" << endl;
	exit(0);
	return true;		// compiler complains without it
}


/////////////////////////////////////////////////////////////////////////////
//
// Default command processing.  Called when we don't know anything
// about the opcode.  The function reads the rest of the message,
// prints an error message and returns
//
static bool
ProcessDefault(int sock, const TMsgHeader &head)
{
	TMessage      msg;

	if ( !ReadMessage(sock, head, msg) ) return false;
	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Entry point for the Hcsmexec functionality.  The code creates a socket
// and waits for a connection; once a connection is made it goes into
// a loop receiving messages and processing commands.
//
int
RunHcsmExecMode(int argc, char **argv)
{
#ifdef _DEBUG_ON_
	cout << "In RunHcsmExecMode\n";
#endif

	cout << "Please wait ..." << flush;

// Initialize windows sockets
#ifdef _WIN32
	WSAData wsaData;
	if ( WSAStartup(MAKEWORD(1, 1), &wsaData) != 0 ) {
		cout << "Hcsmexec: cannot initialize sockets." << endl;
		return -1;
	}

	m_Timer = hrt_timer_alloc(HRT_MICROSECOND, "hcsmexec");

#endif

//
// Wait for a connection from a client
//
	int cmdSock = WaitForConnection(DEFAULT_PORT);
	if ( cmdSock < 0 ) return -1;


//
// Read and process commands
//
	double              frequ;
	int                 dynafrm;
	string              LriDir;
	string				ScnDir;
	CCved               cved;
	CHcsmCollectionPtr  pHcsm = 0;
	string              Scenario;
	string              error;

	TMsgHeader msgHeader;
	while ( 1 ) {
		if ( !ReadHeader(cmdSock, msgHeader) ) {
#ifdef _DEBUG_ON_
			cerr << "Read header at " << __FILE__ << ":" << __LINE__ 
				<< " failed." << endl;
#endif
			return -1;
		}

#ifdef _WIN32
		hrt_timer_start(m_Timer);
#endif

		int    opcode    = msgHeader.m_OpCode;
		bool   HaveError = false;

		switch ( opcode ) {
			case CMD_PING :
				HaveError = !ProcessPing(cmdSock, msgHeader);
#ifdef _DEBUG_ON_
				cout << "PING:" << HaveError << endl;
#endif
				break;

			case CMD_SCENSCRIPT :
			case CMD_SCENSCRIPTMULTI :
				HaveError = !ProcessScript(cmdSock, msgHeader, Scenario);
#ifdef _DEBUG_ON_
				cout << "SCENSCRIPT(actual=" << opcode << "):" 
						<< HaveError << endl;
#endif
				break;

			case CMD_SIMPARAMS :
				HaveError = !ProcessSimParams(cmdSock, msgHeader,
							frequ, dynafrm, LriDir, ScnDir);
#ifdef _DEBUG_ON_
				cout << "SIMPARAMS:" << HaveError << " F="
						<< frequ << ", DynaFrm=" << dynafrm 
						<< ", LriDir=\"" << LriDir << "\"" << endl;
#endif
				break;

			case CMD_STARTSCEN :
				HaveError = !ProcessStartScenario(cmdSock, msgHeader,
							Scenario, LriDir, frequ, dynafrm,
							cved, pHcsm, error, ScnDir);
#ifdef _DEBUG_ON_
				cout << "STARTSCEN:" << HaveError << endl;
#endif
				break;

			case CMD_RUNFRAMES :
				HaveError = !ProcessRunFrames(cmdSock, msgHeader,
						dynafrm, cved, pHcsm, error);
#ifdef _DEBUG_ON_
				cout << "CMD_RUNFRAMES, HaveError = " << HaveError << endl;
#endif
				break;

			case CMD_GETDYNAOBJS :
				HaveError = !ProcessGetDynaObjs(cmdSock, msgHeader,
							cved, pHcsm, error);
#ifdef _DEBUG_ON_
				cout << "CMD_GETDYNOBJS, HaveError = " << HaveError << endl;
#endif
#ifdef _SHOW_TIMING_
				hrt_timer_stop(m_Timer);
				float elapsedTime = hrt_timer_lastelapsedsecs(m_Timer);
				cout << "Time (mS): " << elapsedTime << endl;
#endif
				break;

			case CMD_GETINSTANCEDOBJS :
				HaveError = !ProcessGetInstancedObjs(cmdSock, msgHeader,
							cved, pHcsm, error);
#ifdef _DEBUG_ON_
				cout << "CMD_GETInSTANCEDOBJS, HaveError = " << HaveError 
								<< endl;
#endif
				break;

			case CMD_GETCHANGEDSTATICOBJS :
				HaveError = !ProcessGetChangedStaticObjs(cmdSock, msgHeader,
							cved, pHcsm, error);
#ifdef _DEBUG_ON_
				cout << "CMD_GETInSTANCEDOBJS, HaveError = " << HaveError 
								<< endl;
#endif
				break;

			case CMD_ENDSCENARIO :
				HaveError = !ProcessEndScenario(cmdSock, msgHeader,
							cved, pHcsm, error);
#ifdef _DEBUG_ON_
				cout << "CMD_ENDSCENARIO, HaveError = " << HaveError 
								<< endl;
#endif
				break;

			case CMD_QUIT :
#ifdef _DEBUG_ON_
				cout << "CMD_QUIT, HaveError = " << HaveError << endl;
#endif
				exit(0);
				HaveError = !ProcessQuit(cmdSock, pHcsm);
				break;

			case CMD_SETDEBUGMODE :
				HaveError = !ProcessSetDebugMode(cmdSock, msgHeader,
							cved, pHcsm, error);
#ifdef _DEBUG_ON_
				cout << "CMD_SETDEBUGMODE, HaveError = " << HaveError 
								<< endl;
#endif
				break;

			case CMD_GETDEBUGDATA :
				HaveError = !ProcessGetDebugData(cmdSock, msgHeader,
							cved, pHcsm, error);
#ifdef _DEBUG_ON_
				cout << "CMD_GETDEBUGDATA, HaveError = " << HaveError 
								<< endl;
#endif
				break;

			case CMD_TAKEOBJCONTROL :
			case CMD_CONTROLOBJ :
			case CMD_RELEASEOBJCONTROL :
				cout << "Hcsmexec: received command " 
					<< opcode << ".  Should be sent to the gateway"
					<< endl;
				HaveError = false;		// don't quit, continue
				break;

			default:
				HaveError = !ProcessDefault(cmdSock, msgHeader);
#ifdef _DEBUG_ON_
				cout << "Default:" << HaveError << endl;
#endif
				break;
		}
	}

	return 0;
}
