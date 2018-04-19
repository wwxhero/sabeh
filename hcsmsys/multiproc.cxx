/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1999 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: multiproc.cxx,v 1.6 2011/12/28 16:31:19 iowa\vhorosewski Exp $
// Author:       Yiannis Papelis
// Date:         August, 1999
//
// Description:  A "portable" interface to multi processing functions.
//
/////////////////////////////////////////////////////////////////////////////
#ifdef _WIN32

#include <stdio.h>
#include <process.h>

#elif __sgi

#include <sys/types.h>
#include <unistd.h>

#elif _PowerMAXOS

#include <sys/types.h>
#include <unistd.h>

#endif


///////////////////////////////////////////////////////////////
//
// Version for Concurrent
//
//////////////////////////////////////////////////////////////
#ifdef _PowerMAXOS
bool
StartNewProcess(char* const args[])
{
	int code;

	code = fork();

	if ( code == 0 ) {
		execvp(args[0], args);
		return false;
	}
	else
	if ( code < 0 ) {
		return false;
	}
	else {
		return true;
	}
}
#endif


#ifdef __sgi

bool
StartNewProcess(char* const args[])
{
	int code;

	code = fork();

	if ( code == 0 ) {
		execvp(args[0], args);
		return false;
	}
	else
	if ( code < 0 ) {
		return false;
	}
	else {
		return true;
	}
}

#endif

#ifdef _WIN32
bool
StartNewProcess(char *const args[])
{
	intptr_t code;

	code = _spawnvp(_P_NOWAIT, args[0], args);
	if ( code < 0 ) 
		return false;
	else
		return true;
}
#endif
