/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: trafficlightmanager.cxx,v 1.27 2016/10/28 20:56:06 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    November, 1999
 *
 * Description:  Contains code for the traffic light manager object
 *               (TrafficLightManager HCSM).
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
#define DEBUG_LEVEL_TRAFFIC_LIGHT_MANAGER 0

void 
CTrafficLightManager::MakeStringUpper( string& s )
{
	int i = 0;
	int diff = 'a' - 'A';
	for( i = 0; i < s.size(); i++ )
	{
		if( (s[i] <= 'z') && (s[i] >= 'a') )
		{
			s[i] -= diff;
		}
	}
}

void 
CTrafficLightManager::ParseDialString(
			const string& cStr, 
			string& name, 
			string& state, 
			double* pTime
			)
{
	const size_t strSize = cStr.size() + 1;
    char* pBuf = new char[strSize+1];
    char* pTokDel = " \t:";
    char* pTok;
	char* pCurrPos = NULL;
    strncpy( pBuf, cStr.c_str(),strSize );
    pTok = strtok_s( pBuf, pTokDel, &pCurrPos );
	if (!pTok)
		return;
    name = pTok;
    pTok = strtok_s( NULL, pTokDel, &pCurrPos );
	if (!pTok)
		return;
    state = pTok;
    pTok = strtok_s( NULL, pTokDel, &pCurrPos );
	if (!pTok)
		return;
    *pTime = atof( pTok );
}


void 
CTrafficLightManager::UserCreation( 
			const CTrafLghtMngrParseBlock* pSnoBlock
			)
{
	// print creation message with template name and hcsm id
	PrintCreationMessage();

	CSnoBlock::cTChildIterator pB;
	CClgParseBlock::TClgTable tClg;
	double currTimeInSecs = GetFrame() * GetTimeStepDuration();
	for( pB = pSnoBlock->BeginChild(); pB != pSnoBlock->EndChild(); pB++ )
	{
		CClgParseBlock b( *pB );
		CClg tempClg(b, *cved, currTimeInSecs);
		m_clgs.push_back(tempClg);
	}

#if DEBUG_LEVEL_TRAFFIC_LIGHT_MANAGER > 0
	vector<CClg>::const_iterator cItr = m_clgs.begin();
	gout<<"\n---------------------------------------"<<endl;	
	gout<<"time step"<<GetTimeStepDuration();
	for (; cItr != m_clgs.end(); cItr++)
		cItr->Dump(*cved);
	gout<<"---------------------------------------"<<endl;	
#endif 
}


void CTrafficLightManager::UserPreActivity( 
			const CTrafLghtMngrParseBlock* pSnoBlock 
			)
{
	double currTimeInSecs = GetFrame() * GetTimeStepDuration();
	vector<CClg>::iterator cItr;

	// to process here if a dial's been set 
	string lightName;
	string lightState;
	double  dTime;      // dial Time
	int    lightId;
	if( m_dialTrafficLight.HasValue() )
	{
		ParseDialString( GetDialTrafficLight(), lightName, lightState, &dTime );
		MakeStringUpper( lightState );
		eCVTrafficLightState state = cvStringToTrafficLightState( lightState.c_str() );
		bool isValidObj = cved->GetObj( lightName, lightId );
		if( isValidObj && state != eOFF )
		{
//			fprintf( stdout, "** currTimeInSecs = %.2f  dTime = %.2f  state = %d\n", currTimeInSecs, dTime, state );

			for( cItr = m_clgs.begin(); cItr != m_clgs.end(); cItr++ )
			{
				double timeToTargetLightState = cItr->TimeToLightState( currTimeInSecs, lightId, state );
//				fprintf( stdout, "%% Id %d:   timeToState = %.2f\n", lightId, timeToTargetLightState );

				if( timeToTargetLightState == 0.0 ) 
				{
					// current light state is one being asked for
					cItr->SetCycleStartTime( currTimeInSecs + dTime );
				}
				else if ( timeToTargetLightState > 0.0 )
				{
					double factor;
					if( dTime > 0.0 )
					{
						factor = timeToTargetLightState / dTime;
					}
					else
					{
						factor = 999999.9;
					}
					cItr->SetTargetStateAndFactor( lightId, state, factor , currTimeInSecs );
				}
			}
		}
		else
		{
			// obj not found
			fprintf(
				stderr, 
				"TLM: error  isValidObj = %d  state = %d\n",
				isValidObj, 
				state
				);
		}

		m_dialTrafficLight.SetNoValue();

#if DEBUG_LEVEL_TRAFFIC_LIGHT_MANAGER > 1
		gout<<"\n Dial Value : "<<GetDialTrafficLight();
		gout<<"\n LightName  : "<<lightName;
		gout<<"\n LightId    : "<<lightId;
		gout<<"\n LightState : "<<lightState;
		gout<<"\n dTime      : "<<dTime;
		if (lightFound)
			gout<<"\n lightFound";
		else
			gout<<"\n lightNotFound";
		gout<<endl;
#endif 
	}
		
	for( cItr = m_clgs.begin(); cItr != m_clgs.end(); cItr++ )
	{
		try 
		{ 
			cItr->Execute( currTimeInSecs, GetTimeStepDuration(), *cved );
		}
		catch ( ... ) 
		{
			cerr << "TrafficLightManager: caught exception in Clg::Execute";
			cerr << endl;
		}
	}

#if DEBUG_LEVEL_TRAFFIC_LIGHT_MANAGER > 1
	m_clgs[1].DumpStates(0, *cved, time);
	m_clgs[1].DumpStates(1, *cved, time);
#endif
}


void 
CTrafficLightManager::UserPostActivity( 
			const CTrafLghtMngrParseBlock* pSnoBlock 
			)
{

}


void 
CTrafficLightManager::UserDeletion( 
			const CTrafLghtMngrParseBlock* pSnoBlock
			)
{
	PrintDeletionMessage();
}
