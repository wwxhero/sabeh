/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setbuttonactn.cxx,v 1.12 2004/06/30 19:30:23 schikore Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description:  Implementation of CSetButtonActn class, which sets the 
// 				 button of the given name.
//
/////////////////////////////////////////////////////////////////////////////

#include "setbuttonactn.h"
#include "hcsmcollection.h"

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks: The pointer to the root HCSM collection is where the new HCSMs
//			will be deleted.
//
// Arguments:	
//	pBlock - pointer to the CActionParseBlock associated with
//			this CActn.
//	pHC - pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetButtonActn::CSetButtonActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
{
	m_pHC = pHC;
	m_buttonName = cpBlock->GetButton();
	InitCandidateSet( cpBlock );
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CSetButtonActn.
//
// Remarks: 
//
// Arguments: CSetButtonActn to be copied into current CSetButtonActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetButtonActn::CSetButtonActn( const CSetButtonActn &cRhs )
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
CSetButtonActn::~CSetButtonActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CSetButtonActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetButtonActn &
CSetButtonActn::operator=( const CSetButtonActn& cRhs ) 
{
	if( this != &cRhs )
	{
		m_pHC            = cRhs.m_pHC;
		m_candidateSet   = cRhs.m_candidateSet;
		m_useInstigators = cRhs.m_useInstigators;

		m_buttonName     = cRhs.m_buttonName;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function iterates through the children of its Actn Block
//			and deletes the HCSMs listed there.  If the HCSM specifications
//			are invalid, then an error message is displayed.
//
// Arguments: none
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CSetButtonActn::Execute( const set<CCandidate>* cpInstigators )
{
	set<CCandidate> candidateSet;
	set<CCandidate>::const_iterator itr;
	CHcsm* pHcsm;

	m_candidateSet.GetCandidates( *(m_pHC->GetCved() ), candidateSet, *m_pHC );

	CHcsmCollection::SetActionSetButtonLog( m_triggerId, m_buttonName );

	//
	// If we use the instigator set and the user gave us a set of 
	// instigators, then Execute the action on the instigators that 
	// are in the candidate set.
	//
	if( m_useInstigators && cpInstigators != 0 )
	{
		for(
			itr = cpInstigators->begin();
			itr != cpInstigators->end(); 
			itr++
			)
		{
			pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
			if( pHcsm == 0 || !pHcsm->SetButtonByName( m_buttonName ) )
			{
				cerr << "CSetButtonActn::Execute: Error!"
					 << "Unable to set button on HCSM "
					 << itr->m_hcsmId << endl;
			}
		}
	}
	//
	// If we don't use the instigator set, then disregard the 
	// instigators and Execute the action on the candidates.
	//
	else 
	{
		for(
			itr = candidateSet.begin();
			itr != candidateSet.end();
			itr++
			)
		{
			pHcsm = m_pHC->GetHcsm( itr->m_hcsmId );
			if( pHcsm == 0 || !pHcsm->SetButtonByName( m_buttonName ) )
			{
				cerr << "CSetButtonActn::Execute: Error!"
					 << "Unable to set button on HCSM "
					 << itr->m_hcsmId << endl;
			}
		}
	}
}
