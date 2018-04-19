/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: environmentcontroller.cxx,v 1.18 2003/10/01 20:05:52 oahmad Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    November, 1999
 *
 * Description:  Contains code for the environment controller object
 *               (EnvironmentController HCSM).
 *
 ****************************************************************************/

#include "genericinclude.h"
#include "genhcsm.h"
#include "hcsmcollection.h"

#ifdef __sgi
#include <strstream.h>
#define ostringstream ostrstream
#elif _PowerMAXOS
#include <ctype.h>
#include <strstream>
#define ostringstream ostrstream
#elif _WIN32
#include <sstream>
#endif

//
// Debugging macros.
//

#undef DEBUG_LEVEL_ENVIRONMENTAL_CONTROLLER //0 
/////////////////////////////////////////////////////////////////////////////
//
// Description: convert the string to eCVEnviroType
//
// Remarks:
//
// Arguments:
// str - a string that will be converted
//
// Returns: a variable of type eCVEnviroType corresponding to str
//
/////////////////////////////////////////////////////////////////////////////
#if DEBUG_LEVEL_ENVIRONMENTAL_CONTROLLER > 0
	static numOfGettingCalled = 0;
#endif

void 
CEnvironmentController::Dump( void )
{
	cved->DumpEnvArea();
}



void 
CEnvironmentController::UserCreation( 
			const CEnvControlParseBlock* pSnoBlock
			)
{

	// print creation message with template name and hcsm id
	PrintCreationMessage();

	
	vector<CPoint2D> poly;
	CPoint2D origin;
	if( !pSnoBlock->IsGlobalEnv() )
	{
		origin = pSnoBlock->GetOrigin();
		poly = pSnoBlock->GetArea();
	}

	vector<cvTEnviroInfo> info;

	CSnoBlock::cTChildIterator pB;
	for( pB = pSnoBlock->BeginChild(); pB != pSnoBlock->EndChild(); pB++ )
	{
		CEnviroInfoParseBlock b( *pB );

		eCVEnviroType enviroType;
		cvTEnviroInfo cvInfo;

		CEnviroInfoParseBlock::TEnviroInfo eI = b.GetEnviroInfo();


		enviroType = cvStringToEnviroType(eI.type);
		cvInfo.type = enviroType;

		if( enviroType == eLIGHTNING )
		{
			cvInfo.info.Lightning.degree = 
				cvStringToLMHScale( eI.info.Lightning.degree );
		} 
		else if( enviroType == eVISIBILITY )
		{
			cvInfo.info.Visibility.dist = eI.info.Visibility.dist;
		} 
		else if( enviroType == eHAZE )
		{
			cvInfo.info.Haze.dist = eI.info.Haze.dist;
		}
		else if( enviroType == eFOG )
		{
			cvInfo.info.Fog.dist = eI.info.Fog.dist;
		}
		else if( enviroType == eSMOKE )
		{
			cvInfo.info.Smoke.degree = eI.info.Smoke.degree;
		}
		else if( enviroType == eCLOUDS ) 
		{
			cvInfo.info.Clouds.altitude = eI.info.Clouds.altitude;
			cvInfo.info.Clouds.type = eI.info.Clouds.type;
		}
		else if( enviroType == eGLARE )
		{
			cvInfo.info.Glare.degree = eI.info.Glare.degree;
		}
		else if( enviroType == eSNOW )
		{
			cvInfo.info.Snow.degree = 
						cvStringToLMHScale( eI.info.Snow.degree );
		}
		else if( enviroType == eRAIN )
		{
			cvInfo.info.Rain.degree = 
						cvStringToLMHScale( eI.info.Rain.degree );
		} 
		else if( enviroType == eWIND )
		{
			cvInfo.info.Wind.vel = eI.info.Wind.vel;
			cvInfo.info.Wind.gust = eI.info.Wind.gust;
			CVector2D v( eI.info.Wind.dirX, eI.info.Wind.dirY );
			v.Normalize();
			cvInfo.info.Wind.dir_i = v.m_i;
			cvInfo.info.Wind.dir_j = v.m_j;
		}
		info.push_back( cvInfo );
	}

	if( pSnoBlock->IsGlobalEnv() )
	{
		cved->SetGlobalEnviron( info );	
	}
	else
	{
		cved->CreateEnvArea( info, poly, origin );
	}
}


void 
CEnvironmentController::UserPreActivity( 
			const CEnvControlParseBlock* pSnoBlock 
			)
{	


}


void 
CEnvironmentController::UserPostActivity( 
			const CEnvControlParseBlock* pSnoBlock 
			)
{

}


void 
CEnvironmentController::UserDeletion( 
			const CEnvControlParseBlock* pSnoBlock
			)
{

	PrintDeletionMessage();

#if DEBUG_LEVEL_ENVIRONMENTAL_CONTROLLER > 0 
	if( numOfGettingCalled == 0 )
	{
		Dump();
	}
	numOfGettingCalled++;
#endif

}
