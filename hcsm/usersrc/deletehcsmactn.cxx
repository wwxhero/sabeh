/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: deletehcsmactn.cxx,v 1.12 2004/06/30 19:30:19 schikore Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CDeleteHcsmActn, a subclass of CAction.  
// 	This class deletes the indicated HCSMs from the simulation.
//
/////////////////////////////////////////////////////////////////////////////

#include "deletehcsmactn.h"
#include "hcsmcollection.h"

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
CDeleteHcsmActn::CDeleteHcsmActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			) 
{
	m_pHC = pHC;
	InitCandidateSet( cpBlock );
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CDeleteHcsmActn
//
// Remarks: 
//
// Arguments: CDeleteHcsmActn to be copied into current CDeleteHcsmActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CDeleteHcsmActn::CDeleteHcsmActn( const CDeleteHcsmActn& cRhs )
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
CDeleteHcsmActn::~CDeleteHcsmActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CDeleteHcsmActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CDeleteHcsmActn &
CDeleteHcsmActn::operator=( const CDeleteHcsmActn& cRhs )
{

	if(this != &cRhs)
	{
		m_pHC			 = cRhs.m_pHC;
		m_candidateSet	 = cRhs.m_candidateSet;
		m_useInstigators = cRhs.m_useInstigators;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function deletes all the HCSMs that pass through the 
// 	candidate set filters.  If the instigator set is used, then the 
// 	filters are applied to the instigator set.  Otherwise, they are
// 	applied to all the objects in simulation.
//
// Arguments: 
// 	cpInstigators - An optional parameter that contains the HCSM IDs of the
//		objects which caused the trigger to fire.
//  sequentialActions - An optional parameters that is ignored.
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CDeleteHcsmActn::Execute( const set<CCandidate>* cpInstigators )
{
	CHcsm *pHcsm;

	set<CCandidate> candidateSet;
	set<CCandidate>::const_iterator itr;

	CHcsmCollection::SetActionDeleteLog( m_triggerId );

	m_candidateSet.GetCandidates( *(m_pHC->GetCved()), candidateSet, *m_pHC );

	//
	// If we use the instigator set and the user gave us a set of 
	// instigators, then delete the instigators that are in the 
	// candidate set.
	//
	if( m_useInstigators && cpInstigators != 0 )
	{
		// Iterate through set and delete the associated HCSM.
		for(
			itr = cpInstigators->begin();
			itr != cpInstigators->end(); 
			itr++
			) 
		{
			// Delete the HCSM.
			pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
			bool error = pHcsm == 0 || !m_pHC->DeleteHcsm( pHcsm );
			if( error ) 
			{
				gout << "CDeleteHcsmActn::Execute: "
					 << "ERROR!  Unable to delete HCSM "
					 << itr->m_hcsmId << endl;
			}
		}
	}
	//
	// If we don't use the instigator set, then disregard the 
	// instigators and delete the candidates.
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
			// Delete the HCSM.
			pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
			bool error = pHcsm == 0 || !m_pHC->DeleteHcsm( pHcsm );
			if( error )
			{
				gout << "CDeleteHcsmActn::Execute: ";
				gout << "ERROR!  Unable to delete HCSM ";
				gout << itr->m_hcsmId << endl;
			}
		}
	}
}

