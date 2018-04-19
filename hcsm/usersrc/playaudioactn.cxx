/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: playaudioactn.cxx,v 1.13 2011/07/15 20:44:19 iowa\dheitbri Exp $
// Author(s):   Jillian Vogel
// Date:        October, 1999
//
// Description: The definition of CPlayAudioActn, a subclass of CAction.  
//	This class causes the indicated objects to play the indicated audios.
//
/////////////////////////////////////////////////////////////////////////////

#include "playaudioactn.h"
#include "hcsmcollection.h"

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:	
//	pBlock - Pointer to the CActionParseBlock associated with this CAction.
//	pHC - Pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CPlayAudioActn::CPlayAudioActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	m_delay = cpBlock->GetDelay();
	m_audio = cpBlock->GetAudio();

	m_FrameCnt = -1;
	m_isFinished = false;
    m_started = false;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//  CPlayAudioActn
//
// Remarks: 
//
// Arguments: CPlayAudioActn to be copied into current CPlayAudioActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CPlayAudioActn::CPlayAudioActn( const CPlayAudioActn& cRhs )
{
	*this = cRhs;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The destructor.
//
// Remarks: 
//
// Arguments: none
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CPlayAudioActn::~CPlayAudioActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//  current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: reference to the CPlayAudioActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CPlayAudioActn&
CPlayAudioActn::operator=( const CPlayAudioActn& cRhs ) 
{
	if( this != &cRhs ) 
	{
		m_pHC			 = cRhs.m_pHC;
		m_audio			 = cRhs.m_audio;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function plays the audio for all the HCSMs that pass 
// 	through the candidate set filters.  If the instigator set is used, 
// 	then the filters are applied to the instigator set.  Otherwise, they 
// 	are applied to all the objects in simulation.
//
// Arguments: 
// 	instigators - optional parameter that contains the HCSM IDs of the
// 		objects which caused the trigger to fire.
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CPlayAudioActn::Execute( const set<CCandidate>* cpInstigators )
{
#if 0
	CHcsm* pHcsm;

	set<CCandidate> candidateSet;
	set<CCandidate>::const_iterator itr;

	m_candidateSet.GetCandidates( *(m_pHC->GetCved() ), candidateSet, *m_pHC );

	//
	// If we use the instigator set and the user gave us a set of 
	// instigators, then play the audio on the instigators that are in 
	// the candidate set.
	//
	if( m_useInstigators && cpInstigators != 0 )
	{
		// Iterate through set and play the audio.
		for( 
			itr = cpInstigators->begin();
			itr != cpInstigators->end(); 
			itr++
			) 
		{
			if( candidateSet.find( *itr ) != candidateSet.end() )
			{
				// Do nothing for now.
				pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
				if( pHcsm == 0 )
				{
					gout << "CPlayAudioActn::Execute: ERROR! "
						 << "Invalid HCSM " 
						 << itr->m_hcsmId << endl;
				}

			}
		}
	}
	//
	// If we don't use the instigator set, then disregard the 
	// instigators and play the audio of the candidates.
	//
	else 
	{
		// Execute action on the candidates in the set.
		for( 
			itr = candidateSet.begin();
			itr != candidateSet.end();
			itr++
			)
		{
			// Do nothing for now.
			pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
			if( pHcsm == 0 )
			{
				gout << "CPlayAudioActn::Execute: ERROR! "
					 << "Invalid HCSM " 
					 << itr->m_hcsmId << endl;
			}
		}
	}
#endif


	if (!m_started){
		CHcsmCollection::m_sPlayAudioText = m_audio;
		stringstream converter;
		int valOut = 0;
		converter<<m_audio;
		converter>>valOut;
		// write activity log
		CHcsmCollection::SetActionPlayAudioLog( m_triggerId );
		//CHcsmCollection::m_sAudio_Trigger = valOut;
		CHcsmCollection::m_sScenarioWriteCellData[CHcsmCollection::m_sScenarioWriteCellDataSize].cellName = "SCC_Audio_Trigger";
		CHcsmCollection::m_sScenarioWriteCellData[CHcsmCollection::m_sScenarioWriteCellDataSize].intData.clear();
		CHcsmCollection::m_sScenarioWriteCellData[CHcsmCollection::m_sScenarioWriteCellDataSize].intData.push_back(valOut);
		CHcsmCollection::m_sScenarioWriteCellData[CHcsmCollection::m_sScenarioWriteCellDataSize].cellData = m_audio;
		CHcsmCollection::m_sScenarioWriteCellData[CHcsmCollection::m_sScenarioWriteCellDataSize].frame = CHcsmCollection::m_frame;
		CHcsmCollection::m_sScenarioWriteCellDataSize++;
		m_FrameCnt = 0;
	}
	m_FrameCnt++;
	if (m_FrameCnt > 15){
		CHcsmCollection::m_sAudio_Trigger = 0;
		m_isFinished = true;
	}

}

