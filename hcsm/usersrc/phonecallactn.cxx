/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: phonecallactn.cxx,v 1.11 2005/08/05 18:09:33 oahmad Exp $
// Author(s):   Matt Schikore
// Date:        August 2000
//
// Description: The definition of CPhoneCallActn, a subclass of CAction.  
//	This class causes a phone call to be placed.
//
/////////////////////////////////////////////////////////////////////////////

#include "phonecallactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"


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
CPhoneCallActn::CPhoneCallActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	m_delay = cpBlock->GetDelay();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//  CPhoneCallActn.
//
// Remarks: 
//
// Arguments: CPhoneCallActn to be copied into current CPhoneCallActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CPhoneCallActn::CPhoneCallActn( const CPhoneCallActn& cRhs )
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
CPhoneCallActn::~CPhoneCallActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//  current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: reference to the CPhoneCallActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CPhoneCallActn&
CPhoneCallActn::operator=( const CPhoneCallActn& cRhs )
{
	if( this != &cRhs)
	{
		m_pHC = cRhs.m_pHC;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The Execute function, which is called when the action
//  should be executed
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CPhoneCallActn::Execute( const set<CCandidate>* ) 
{
	CHcsmCollection::m_sPlacePhoneCallAge = 60;
	CHcsmCollection::m_sSCC_PlacePhoneCall = 1;

	// write the activity log
	CHcsmCollection::SetActionPhoneCallLog( m_triggerId );
}

