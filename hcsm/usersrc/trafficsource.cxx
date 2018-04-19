/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: trafficsource.cxx,v 1.21 2016/01/04 16:38:40 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    November, 1999
 *
 * Description:  Contains code for the traffic source object
 *               (TrafficSource HCSM).
 *
 ****************************************************************************/

#include "genericinclude.h"
#include "genhcsm.h"
#include "hcsmcollection.h"
#include <strstream>
//#include <stdlib.h>
//#include <time.h>

//
// Debugging macros.
//
#undef DEBUG_TS

#ifdef DEBUG_TS
static int* cntrs = 0;
#endif

void 
CTrafficSource::UserCreation( const CTrafSrcParseBlock* cpBlock	)
{
	// print creation message with template name and hcsm id
	PrintCreationMessage();

	string prefix = MessagePrefix();
	strstream message;
	m_pTraffSource = nullptr;
	try {
		// Initialize CObjectInitCond object
		int curFrame = GetFrame();
		m_initCondition.SetObjectInitCond(
								*cved, 
								curFrame, 
								GetTimeStepDuration(),  
								cpBlock->GetCrRad(),  
								cpBlock->GetActvDel(), 
								cpBlock->GetLifetime()
								);

		// Initialize other local variables
		m_isRandomUniform = cpBlock->IsRandomUniform();
		m_times = cpBlock->GetTimes();

		m_periodicSequence = cpBlock->GetPeriodic();
		m_randomSequence = cpBlock->GetRandomizePeriodicFlag();
		m_curSequenceCount = 0;

		m_startFrame = curFrame;
		m_curTimeIdx = 0;
		m_randTime = -1;

		vector<CTrafSrcParseBlock::TWeight> weights = cpBlock->GetWeights();
		vector<CTrafSrcParseBlock::TWeight>::const_iterator wItr;
		CSnoBlock::cTChildIterator bItr;
		CSnoValue val;

		// Add all the child blocks to the traffic list once to insure that
		// even if an object is not listed in the Weights values, it is added
		// to the list.
		for( bItr = cpBlock->BeginChild(); bItr != cpBlock->EndChild(); bItr++ )
		{
			m_traffic.push_back( *bItr );
		}
        if (m_traffic.size() == 0){
            gout << MessagePrefix() << "Traffic Source Requires at least 1 Vehicle...";
            gout << "[SUICIDE]" << endl;
            Suicide();
            return;
        }

		for( wItr = weights.begin(); wItr != weights.end(); wItr++ ) 
		{
			for( bItr = cpBlock->BeginChild(); bItr != cpBlock->EndChild(); bItr++ ) 
			{
				// If the name field and its value matches the weight name
				bool addToList = (
							bItr->GetField( "Name", val ) && 
							val.GetStringValue() == wItr->name
							);
				if( addToList ) 
				{
					// Add it to the traffic list as many times as
					//	it is weighted - 1 (because all objects are 
					//	added once by default)
					int i;
					for( i = 1; i < wItr->weight; i++ )
					{
						m_traffic.push_back( *bItr );
					}

					// Move on to the next weight.
					break;
				}
			}
		}

		m_rng.SetAllSeeds( 2, 1 );
		m_rngTime = m_rng.GetStream();
		m_rngTraffic = m_rng.GetStream();
	}
	catch( CSnoBlock::TCountError e ) 
	{
		gout << MessagePrefix() << "UserCreation: caught SnoBlock error...";
		gout << "[SUICIDE]" << endl;

		Suicide();
		return;
	}
	catch(...) {
		gout << MessagePrefix() << "UserCreation: caught unknown error...";
		gout << "[SUICIDE]" << endl;

		Suicide();
		return;
	}

	SetDialStartStop( true );

#ifdef DEBUG_TS
	cntrs = new int[m_traffic.size()];
	int i;
	for( i = 0; i < m_traffic.size(); i++ )
	{
		cntrs[i] = 0;
	}

	gout << MessagePrefix() << "UserCreation" << endl;
	gout << "  position = " << cpBlock->GetPosition() << endl;
	gout << "  cr rad =   " << cpBlock->GetCrRad() << " ft" << endl;
	gout << "  lifetime = " << cpBlock->GetLifetime() << " secs" << endl;
	gout << "  actv del = " << cpBlock->GetActvDel() << " secs" << endl;
	vector<double>::const_iterator cItr;
	string times;
	char t[10];
	for( cItr = m_times.begin(); cItr != m_times.end(); cItr++ ) 
	{
		sprintf( t, "%lf", *cItr );
		times += t;
		times += "  ";
	}
	if( cpBlock->IsRandomUniform() )
		gout << "  random = " << times << endl;
	else
		gout << "  periodic = " << times << endl;

	vector<CTrafSrcParseBlock::TWeight> weights = cpBlock->GetWeights();
	vector<CTrafSrcParseBlock::TWeight>::const_iterator wItr;
	gout << "  weights =";
	for (wItr = weights.begin(); wItr != weights.end(); wItr++) {
		gout << "  " << wItr->name << ":" << wItr->weight;
	}
	gout << endl;

	vector<CSnoBlock>::iterator pTraf;
	CSnoValue val;
	gout << "  traffic = ";
	for( pTraf = m_traffic.begin(); pTraf != m_traffic.end(); pTraf++ ) 
	{
		if( pTraf->GetField( "Name", val ) )
		{
			gout << "  " << val.GetStringValue();
		}
	}
	gout << endl;
#endif

	if (m_times.size() > 1){
		double min = m_times[0];
		double max = m_times[1];
		m_randTime = m_rng.RandomDoubleRange( min, max, m_rngTime );
	}
	else if (m_times.size() == 1){
		m_randTime = m_times[0];
	}else{
		gout<<"Error Traffic Source needs at least 1 random time input"<<endl;
	}
#ifdef DEBUG_TS
		gout << endl << "########" << endl
			<< "Starting first run of sequence, size " << m_periodicSequence.size() << endl;
#endif
	if ( m_randomSequence )
	{
		m_workingSequence = m_periodicSequence;
		m_curSequenceIdx = m_rng.RandomLongRange( 0, m_workingSequence.size(), m_rngTime );
	}
}


void 
CTrafficSource::UserActivity( const CTrafSrcParseBlock* cpBlock )
{	
	string prefix = MessagePrefix();
	string message;

	// If the MakeTraffic button is pressed, then fire the action
	if( GetButtonMakeTraffic() ) 
	{
#ifdef DEBUG_TS
		gout << MessagePrefix() << "MakeTraffic was pressed" << endl;
#endif

		MakeTraffic();
		return;
	}

	// If the StartStop dial is set to false, then return 
	//	without making any traffic.
	if( !GetDialStartStop() ) 
	{
#ifdef DEBUG_TS
		gout << MessagePrefix() << "StartStop dial was set to 'STOP'" << endl;
#endif

		return;
	}

	// Get current time and position
	int actualTime = GetFrame();

	// Get the current state from the InitCondition object
	EInitCond state = m_initCondition.Execute(
										cpBlock->GetPosition(), 
										actualTime
										);
	// For use in eACTIVATE state.
	cvTObjAttr sourceAttr = { 0 };

	sourceAttr.hcsmId = m_pRootCollection->GetHcsmId( this );
	sourceAttr.xSize = 2.0;
	sourceAttr.ySize = 2.0;

	switch( state )
	{
		case eWAIT:
			// Don't do anything yet.
			break;

		case eACTIVATE:
				m_pTraffSource = dynamic_cast<CCoordinatorObjectObj *> (
								cved->CreateDynObj(
											cpBlock->GetName(),
											eCV_COORDINATOR, 
											m_typeId,
											sourceAttr,
											&cpBlock->GetPosition(),
											0, 
											0
											)
								);


			if( TimeToMakeTraffic( actualTime ) )  MakeTraffic();
			break;

		case eUNDER_CONTROL:
			
			if( TimeToMakeTraffic( actualTime ) )  MakeTraffic();
			break;

		case eDELETE:
			DeleteHcsm( this );
			break;

		case eEXIT:
			break;
	}
}


void 
CTrafficSource::UserDeletion( const CTrafSrcParseBlock* cpBlock	)
{
	PrintDeletionMessage();

	if( m_pTraffSource != 0 )  cved->DeleteDynObj( m_pTraffSource );

	//
	// Add an entry to the activity log for HCSM deletion.
	//
	m_pRootCollection->SetHcsmDeleteLog( 
				this, 
				cpBlock->GetPosition() 
				);

#ifdef DEBUG_TS
	int i;
	for( i = 0; i < m_traffic.size(); i++ )
	{
		gout << i << ": " << cntrs[i] << "  ";
	}
	gout << endl;
#endif
}

bool 
CTrafficSource::TimeToMakeTraffic( int curFrame ) 
{
	// Determine if it is time to make traffic based on the 
	//	specified distribution
	if( m_isRandomUniform ) 
	{
		// The RandomUniform distribution causes traffic to be generated 
		//	randomly during a specified interval.  Once the interval has 
		//	elapsed, it is repeated.  The bounds of the interval are stored 
		//	in cpBlock->GetTimes(), elements 0 and 1.
		double min = m_times[0];
		double max = m_times[1];

		// If the elapsed time exceeds the randomly selected duration, 
		//	return true to indicate that traffic should be generated.
		long elapsedFrames = curFrame - m_startFrame;
		double elapsedTime = GetTimeStepDuration() * elapsedFrames;
		double curTime = GetTimeStepDuration() * curFrame;

		bool timeToMakeTraffic = elapsedTime >= m_randTime;
		bool regenerateRandTime = timeToMakeTraffic || elapsedTime > max;
		if( regenerateRandTime ) 
		{
			m_startFrame = curFrame;
			elapsedTime = 0.0;
			m_randTime = m_rng.RandomDoubleRange( min, max, m_rngTime );

#ifdef DEBUG_TS
			gout << "** m_randTime = " << m_randTime << " secs" << endl;
#endif
		}

		if( timeToMakeTraffic ) 
		{
			return true;
		}
	}
	else 
	{
#if 0
		// If the current time index lies outside the acceptable range
		//	(through strangeness or through incrementing), then reset it
		//	to the beginning of the list of times.
		if( m_curTimeIdx >= m_times.size() || m_curTimeIdx < 0 )
		{
			m_curTimeIdx = 0;
		}

		// If the elapsed time exceeds the duration set in the parse block, 
		//	return true to indicate that traffic should be generated.
		long elapsedFrames = curFrame - m_startFrame;
		double elapsedTime = GetTimeStepDuration() * elapsedFrames;
		if( elapsedTime >= m_times[m_curTimeIdx] ) 
		{
			// Set up local vars for the next call
			m_startFrame = curFrame;
			m_curTimeIdx++;
			return true;
		}
#endif

		if( m_curSequenceCount >= m_periodicSequence.size() || m_curSequenceCount < 0 )
		{
			m_curSequenceCount = 0;
#ifdef DEBUG_TS
				gout << endl << "########" << endl 
					<< "Starting new run of sequence, size " << m_periodicSequence.size() << endl;
#endif
		}

		long elapsedFrames = curFrame - m_startFrame;
		double elapsedTime = GetTimeStepDuration() * elapsedFrames;
		if ( m_randomSequence )
		{
			// If the elapsed time exceeds the randomly selected duration from the 
			// current run of the periodic sequence, return true to indicate that 
			// traffic should be generated.
			bool timeToMakeTraffic = elapsedTime >= m_workingSequence.at( m_curSequenceIdx );
			if( timeToMakeTraffic ) 
			{
#ifdef DEBUG_TS
					gout << "Sequence " << m_curSequenceCount << ":" << m_curSequenceIdx<< " "
						<< m_workingSequence.at(m_curSequenceIdx) << "s" << endl;
#endif
				m_startFrame = curFrame;
				m_curSequenceCount++;
				m_workingSequence.erase( m_workingSequence.begin() + m_curSequenceIdx );
				if ( m_workingSequence.empty() )
					m_workingSequence = m_periodicSequence;
				m_curSequenceIdx = m_rng.RandomLongRange( 0, m_workingSequence.size(), m_rngTime );
				return true;
			}
		}
		else
		{
			// If the elapsed time exceeds the next duration from the current run of 
			// the periodic sequence, return true to indicate that traffic should be 
			// generated.
			if ( elapsedTime >= m_periodicSequence.at( m_curSequenceCount ) )
			{
#ifdef DEBUG_TS				
				gout << "Sequence " << m_curSequenceCount << " "
					<< m_periodicSequence.at(m_curSequenceCount) << "s" << endl;
#endif
				m_startFrame = curFrame;
				m_curSequenceCount++;
				return true;
			}
		}

	}

	return false;
}


void 
CTrafficSource::MakeTraffic( void )
{
	// TODO: Before creating a new HCSM, check to insure that there
	//	are no objects at or swiftly approaching the initial position.
	
	// Select a traffic element to create from the weighted array 
	//	of traffic elements.
	int idx = m_rng.RandomLongRange( 0, m_traffic.size(), m_rngTraffic );

#ifdef DEBUG_TS
	cntrs[idx]++;

	gout << MessagePrefix() << "MakeTraffic: creating ";
	gout << m_traffic[idx].GetBlockName() << endl;
#endif

	CHcsm* pHcsm = m_pRootCollection->CreateHcsm(
				m_traffic[idx].GetBlockName(), m_traffic[idx]
				);

	if( !pHcsm ) 
	{
		gout << MessagePrefix() << "MakeTraffic: unable to create HCSM ";
		gout << m_traffic[idx].GetBlockName() << endl;
	}
}
