//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: tstclient.cxx,v 1.4 1999/09/02 16:00:04 yiannis Exp $
//
// Author(s):    Yiannis Papelis
//
// Date:         August, 1999
// Description:  Test program for the hcsm client library
//
//////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "hcsmclient.h"

main(int argc, char **argv)
{
	CHcsmClient              cli;
	CHcsmClient::TErrorCode  code;
	string                   msg;

	if ( argc != 3 ) {
		cerr << "Usage:\n" << argv[0] << " machine sno" << endl;
		cerr << "\tmachine - IP address or machine name for hcsmexec" << endl;
		cerr << "\tsno     - the scenario file to run" << endl;
		exit(-1);
	}
	cli.SetMode(CHcsmClient::ESynchronous);

	string Dir(":manual:");
	Dir += argv[1];
	if ( !cli.InitSynchronous(Dir) ) {
		cli.GetLastError(msg, code);
		cerr << "InitSynchrnous failed: " << msg << endl;
		return -1;
	}


	cout << " -- Test program -- \n Library initialization ok" << endl;

	//
	// open the scenario file and read it in memory
	//
	FILE *pF = fopen(argv[2], "r");
	if ( pF == NULL ) {
		cerr << "Cannot open scenario file " << argv[2] << endl;
		return -1;
	}
	fseek(pF, 0, SEEK_END);
	long fileSize = ftell(pF);
	rewind(pF);
	char *pBuf = new char[fileSize+1];
	fread(pBuf, fileSize, 1, pF);
	fclose(pF);


	//
	// Start the scenario
	//
	if ( !cli.StartScenario(pBuf, "./", 10.0, 3) ) {
		cli.GetLastError(msg, code);
		cerr << "StartScenario failed: " << msg << endl;
		return -1;
	}
	printf("Send & started scenario ok!\n");

	vector<TObjDesc> objs;
	int count = 0;
	
	while ( 1 ) {
		if ( count++ > 30 ) 
			break;
	//
	// Run one frame
	//
		cerr << "Running one frame ... " << flush;
		if ( !cli.RunFrames(1) ) {
			cerr << " failed." << endl << flush;
		}
		else {
			cerr << "  worked." << endl << flush;
		}

		cerr << "Running another frame ... " << flush;
		if ( !cli.RunFrames(1) ) {
			cerr << " failed." << endl << flush;
		}
		else {
			cerr << "  worked." << endl << flush;
		}

	//
	// Get the simulation data
	//  
		cerr << "Getting the dynamic objects ... " << flush;
		if ( !cli.GetDynamicObjs(objs) ) { 
			cerr << " failed." << endl << flush;
		}
		else {
			cerr << "  worked." << endl << flush;
			vector<TObjDesc>::const_iterator pI;

			for (pI=objs.begin(); pI != objs.end(); pI ++) {
				printf("--> Object, id = %d\n", pI->id);
				printf("       type   = %d\n", pI->type);
				printf("       solId  = %d\n", pI->solId);
				printf("       hcsmId = %d\n", pI->hcsmId);
				printf("       Pos    = %.1f, %.1f, %.1f\n",
							pI->state.anyState.position.x,
							pI->state.anyState.position.y,
							pI->state.anyState.position.z);
			}
		}

	//
	// Get the simulation data
	//  
		cerr << "Getting the instanced objects ... " << flush;
		if ( !cli.GetInstancedObjs(objs) ) { 
			cerr << " failed." << endl << flush;
		}
		else {
			cerr << "  worked." << endl << flush;
			vector<TObjDesc>::const_iterator pI;

			for (pI=objs.begin(); pI != objs.end(); pI ++) {
				printf(
"--> Object id,type,solId,hcsmId = %d,  %d, %d, %d\n", 
				pI->id, pI->type, pI->solId, pI->hcsmId);
				printf("       Pos    = %.1f, %.1f, %.1f\n",
							pI->state.anyState.position.x,
							pI->state.anyState.position.y,
							pI->state.anyState.position.z);
			}
		}
	}

	// Done, send end scenario and quit
	cerr << "Done with loop, sending END_SCENARIO & QUIT." << endl << flush;
	if ( cli.EndScenario() ) {
		if ( cli.Quit() ) {
			cerr << "OK, done." << endl;
			exit(0);
		}
		else {
			cerr << "Quit command failed." << endl;
		}
	}
	else {
		cerr << "End scenario command failed." << endl;
	}

	return 0;
}
