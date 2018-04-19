/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: trafficdata.cxx,v 1.2 2004/04/27 19:15:13 schikore Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    May, 2001
 *
 * Description:  Contains code for the TrafficData HCSM.
 *
 ****************************************************************************/

#include "genericinclude.h"
#include "genhcsm.h"
#include "hcsmcollection.h"

#undef DEBUG_TD

void 
CTrafficData::UserCreation( const CTrafficDataParseBlock* pSnoBlock )
{
	PrintCreationMessage();
	m_logAllObjs = pSnoBlock->GetLogAllObjs();
	m_logObjName = pSnoBlock->GetLogObjName();
	m_logAccel = pSnoBlock->GetLogAccel();
	m_accelMin = pSnoBlock->GetAccelMin();
	m_accelMax = pSnoBlock->GetAccelMax();
}  // end of UserCreation


void 
CTrafficData::UserPreActivity( const CTrafficDataParseBlock* pSnoBlock )
{

#ifdef DEBUG_TD
	gout << MessagePrefix();
	gout << "=== TRAFFIC DATA =================" << endl;
#endif

}  // end of UserPreActivity


void 
CTrafficData::UserPostActivity( const CTrafficDataParseBlock* pSnoBlock )
{

}  // end of UserPostActivity


void CTrafficData::UserDeletion( const CTrafficDataParseBlock* pSnoBlock )
{
	PrintDeletionMessage();
}  // end of UserDeletion
