/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: resetdialactn.cxx,v 1.8 2005/08/05 18:09:34 oahmad Exp $
// Author(s):   Omar Ahmad
// Date:        August, 2002
//
// Description:  Implementation of CResetDialActn class, which resets the 
// 	dial of the given name.
//
/////////////////////////////////////////////////////////////////////////////

#include "resetdialactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"

#define SET_DIAL_DEBUG 0

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks: The pointer to the root HCSM collection is where the new HCSMs
//			will be deleted.
//
// Arguments:	
//	pBlock - A pointer to the CActionParseBlock associated with	this CActn.
//	pHC - A pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CResetDialActn::CResetDialActn(
			const CActionParseBlock *cpBlock, 
			CHcsmCollection *pHC
			)
{
	m_pHC = pHC;
	CActionParseBlock::pbTDial dial = cpBlock->GetDial();
	m_dialName = dial.name;
	InitCandidateSet( cpBlock );
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CResetDialActn
//
// Remarks: 
//
// Arguments: CResetDialActn to be copied into current CResetDialActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CResetDialActn::CResetDialActn( const CResetDialActn& cRhs )
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
CResetDialActn::~CResetDialActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: reference to the CResetDialActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CResetDialActn&
CResetDialActn::operator=( const CResetDialActn& cRhs )
{

	if( this != &cRhs )
	{
		m_pHC            = cRhs.m_pHC;
		m_dialName       = cRhs.m_dialName;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function iterates through the children of its Actn Block
//  and deletes the HCSMs listed there.  If the HCSM specifications
//  are invalid, then an error message is displayed.
//
// Arguments: 
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
void 
CResetDialActn::Execute( const set<CCandidate>* cpInstigators )
{
	set<CCandidate> candidateSet;
	m_candidateSet.GetCandidates( *(m_pHC->GetCved()), candidateSet, *m_pHC );

	//
	// If we use the instigator set and the user gave us a set of 
	// instigators, then Execute the action on the instigators that 
	// are in the candidate set.
	//
	bool useInstigators = m_useInstigators && cpInstigators != 0;
	if( useInstigators )
	{
#if SET_DIAL_DEBUG > 1
		gout << "Using instigator set." << endl;
#endif

		set<CCandidate>::const_iterator itr;
		for( 
			itr = cpInstigators->begin(); 
			itr != cpInstigators->end(); 
			itr++ 
			)
		{
			CHcsm* pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
			bool unableToSetDial = ( 
						pHcsm == 0 || 
						!pHcsm->ResetDialByName( m_dialName ) 
						);
			if( unableToSetDial )
			{
				gout << "CResetDialActn::Execute: Error!"
					 << "Unable to reset dial "
					 << m_dialName << " on HCSM "
					 << itr->m_hcsmId << endl;
			}
#if SET_DIAL_DEBUG > 0
			else 
			{
				gout << m_dialName << " was reset on ";
				gout << pHcsm->GetName() << endl; 
			}
#endif
		}
	}

	//
	// If we don't use the instigator set, then disregard the 
	// instigators and Execute the action on the candidates.
	//
	else
	{
#if SET_DIAL_DEBUG > 1
		gout << "Not using instigator set." << endl;
#endif
	
		set<CCandidate>::const_iterator itr;
		for( itr = candidateSet.begin(); itr != candidateSet.end(); itr++ )
		{
			CHcsm* pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
			bool unableToSetDial = ( 
						pHcsm == 0 ||
						!pHcsm->ResetDialByName( m_dialName ) 
						);
			if( unableToSetDial )
			{
				gout << "CResetDialActn::Execute: Error!"
					 << "Unable to reset dial "
					 << m_dialName << " on HCSM "
					 << itr->m_hcsmId << endl;
			}
#if SET_DIAL_DEBUG > 0
			else 
			{
				gout << m_dialName << " was reset on ";
				gout << pHcsm->GetName() << endl; 
			}
#endif
		}
	}
	CHcsmCollection::SetActionResetDialLog( m_triggerId, m_dialName );
}

