/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: transitiontrffclghtactn.cxx,v 1.12 2013/03/25 18:33:56 IOWA\dheitbri Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CTransitionTrffcLghtActn, a subclass 
//  of CAction.  This class sets the state of the indicated traffic lights.
//
/////////////////////////////////////////////////////////////////////////////

#include "transitiontrffclghtactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"

#define SET_TRFFC_LGHT_DEBUG	0

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:	
//	pBlock - pointer to the CActionParseBlock associated with
//			this CAction.
//	pHC - pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTransitionTrffcLghtActn::CTransitionTrffcLghtActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
{
	m_pHC = pHC;
	InitCandidateSet( cpBlock );
	m_candidateSet.SetType( "TrafficLight" );

	m_byNameSet = cpBlock->GetByNameSet();

	char tmpStateTime[20];
	sprintf_s( 
		tmpStateTime,
        20,
		":%s:%3.5lf", 
		cpBlock->GetState().c_str(), 
		cpBlock->GetTime()
		);
	m_stateTime = tmpStateTime;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CTransitionTrffcLghtActn
//
// Remarks: 
//
// Arguments: CTransitionTrffcLghtActn to be copied into current 
//  CTransitionTrffcLghtActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTransitionTrffcLghtActn::CTransitionTrffcLghtActn(
			const CTransitionTrffcLghtActn& cRhs
			)
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
CTransitionTrffcLghtActn::~CTransitionTrffcLghtActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CTransitionTrffcLghtActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTransitionTrffcLghtActn&
CTransitionTrffcLghtActn::operator=( const CTransitionTrffcLghtActn& cRhs ) 
{

	if( this != &cRhs )
	{
		m_pHC			 = cRhs.m_pHC;
		m_candidateSet	 = cRhs.m_candidateSet;
		m_useInstigators = cRhs.m_useInstigators;

		m_stateTime		= cRhs.m_stateTime;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function sets the state for all the traffic lights that pass 
// 	through the candidate set filters.  If the instigator set is used, 
// 	then the filters are applied to the instigator set.  Otherwise, they 
// 	are applied to all the traffic lights in simulation.
//
// Arguments:
// 	instigators - optional parameter that contains the CCandidate
// 		objects which caused the trigger to fire.
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CTransitionTrffcLghtActn::Execute( const set<CCandidate>* instigators )
{
#if SET_TRFFC_LGHT_DEBUG > 0
	gout << "CTransitionTrffcLghtActn::Execute: ";
	gout << "state = " << m_stateTime << endl;
#endif

	CHcsm* pHcsm = m_pHC->GetHcsm( "TrafficLightManager" );
	if( pHcsm == 0 )
	{
		cerr << "CTransitionTrffcLghtActn::Execute: ";
		cerr << "cannot find TrafficLightManager HCSM...return" << endl;

		return;
	}
	CTrafficLightManager* pTrffcLghtMngr = 
		dynamic_cast<CTrafficLightManager*> (pHcsm);
	if( !pTrffcLghtMngr )
	{
		cerr << "CTransitionTrffcLghtActn::Execute: ";
		cerr << "unable to dynamic cast TrafficLightManager HCSM...return";
		cerr << endl;

		return;
	}

#if SET_TRFFC_LGHT_DEBUG > 0
	gout << "CTransitionTrffcLghtActn::Execute: ";
	gout << "getting pointer to CVED" << endl;
#endif

	CCved* pCved = m_pHC->GetCved();

	set<CCandidate> candidateSet;
	if( !m_byNameSet.empty() )
	{
		int cvedId;
		pCved->GetObj( m_byNameSet[0], cvedId );

#if SET_TRFFC_LGHT_DEBUG > 0
	gout << "CTransitionTrffcLghtActn::Execute: ";
	gout << "insert obj " << cvedId << ":" << m_byNameSet[0];
	gout << " into candidate set" << endl;
#endif

		CCandidate cand;
		cand.m_cvedId = cvedId;
		candidateSet.insert( cand );
	}

#if SET_TRFFC_LGHT_DEBUG > 0
	gout << "CTransitionTrffcLghtActn::Execute: ";
	gout << "candidateSet has size = " << candidateSet.size() << endl;
#endif

	//
	// If we use the instigator set and the user gave us a set of 
	// instigators, then set the state on the instigators that are 
	// in the candidate set.
	//
	if( m_useInstigators && instigators != 0 )
	{
#if SET_TRFFC_LGHT_DEBUG > 0
		gout << "CTransitionTrffcLghtActn::Execute: ";
		gout << "set action on instigator" << endl;
#endif
		// Iterate through set and set the state.
		set<CCandidate>::const_iterator cItr;
		for(
			cItr = instigators->begin();
			cItr != instigators->end(); 
			cItr++
			)
		{
			if( candidateSet.find( *cItr ) != candidateSet.end() )
			{
				string dialValue = pCved->GetObjName( cItr->m_cvedId );
				dialValue += m_stateTime;

				pTrffcLghtMngr->SetDialTrafficLight( dialValue );

#if SET_TRFFC_LGHT_DEBUG > 0
				gout << "CTransitionTrffcLghtActn::Execute: set dial to "
					 << dialValue << endl;
#endif
			}
		}
	}
	//
	// If we don't use the instigator set, then disregard the 
	// instigators and set the state of the candidates.
	//
	else 
	{
#if SET_TRFFC_LGHT_DEBUG > 0
		gout << "CTransitionTrffcLghtActn::Execute: ";
		gout << "set action on candidate" << endl;
#endif
		// Execute action on the candidates in the set.
		set<CCandidate>::const_iterator cItr;
		for( 
			cItr = candidateSet.begin();
			cItr != candidateSet.end();
			cItr++
			)
		{
			string dialValue = pCved->GetObjName( cItr->m_cvedId );
			dialValue += m_stateTime;

			pTrffcLghtMngr->SetDialTrafficLight( dialValue );

#if SET_TRFFC_LGHT_DEBUG > 0
				gout << "CTransitionTrffcLghtActn::Execute: set dial to "
					 << dialValue << endl;
#endif
		}
	}
}
