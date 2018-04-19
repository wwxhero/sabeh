/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: hcsminterface.cxx,v 1.14 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
 *
 * Author:  Jillian Vogel
 *
 * Date:    October, 1999
 *
 * Description:  Contains global functions for the HCSM system
 *
 ****************************************************************************/
#pragma warning( disable : 4786 )  
#include "hcsminterface.h"
#include "hcsmcollection.h"

void LogData( int streamNum, float val )
{
	// For now, just print a message
	if( CHcsmCollection::m_verbose )
	{
		gout << "LogData(): called on stream ";
		gout << streamNum << " with value " << val << endl;
	}

	if( streamNum > 0 && streamNum <= cNUM_LOG_STREAMS)
	{
		CHcsmCollection::m_sLogStreams[streamNum-1] = val;
	}
	else if( streamNum - cNUM_LOG_STREAMS > 0 && streamNum <= cNUM_LOG_STREAMS * 2){
		CHcsmCollection::m_sLogStreamsExt[streamNum-1 - cNUM_LOG_STREAMS ] = val;
	}
	else
	{
		gout << "LogData(): streamNum out of bounds" << endl;
	}
}

void MotionBasePreposition( void )
{
	if( CHcsmCollection::m_verbose )
	{
		gout << "MotionBasePreposition(): called" << endl;
	}
}

void MotionBaseTune( void )
{
	if( CHcsmCollection::m_verbose )
	{
		gout << "MotionBaseTune(): called" << endl;
	}
}

void TerminateSimulation( void )
{
	// Simply set a global flag declared in hcsmsys/sic to "1".
	CHcsmCollection::m_sSCC_Scenario_Stop_Ind = 1;
}
