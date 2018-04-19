/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: usetrafmansetactn.cxx,v 1.4 2005/08/04 15:56:35 oahmad Exp $
//
// Author(s):   Matt Schikore
//
// Date:        January 2001
//
// Description: The definition of CUseTrafManSetActn, a subclass of CAction.  
//	This class causes the active set in the traffic manager to be changed.
//
/////////////////////////////////////////////////////////////////////////////

#include "usetrafmansetactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"

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
CUseTrafManSetActn::CUseTrafManSetActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	m_delay = cpBlock->GetDelay();
	m_setName = cpBlock->GetTrafManSet();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CUseTrafManSetActn
//
// Remarks: 
//
// Arguments: CUseTrafManSetActn to be copied into current CUseTrafManSetActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CUseTrafManSetActn::CUseTrafManSetActn( const CUseTrafManSetActn& cRhs )
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
CUseTrafManSetActn::~CUseTrafManSetActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CUseTrafManSetActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CUseTrafManSetActn&
CUseTrafManSetActn::operator=( const CUseTrafManSetActn& cRhs ) 
{
	if( this != &cRhs )
	{
		m_pHC = cRhs.m_pHC;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The Execute function, which is called when the action
//              should be executed
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CUseTrafManSetActn::Execute( const set<CCandidate>* ) 
{
	CTrafficManager* pTrafMan = 
		dynamic_cast<CTrafficManager*>(m_pHC->GetHcsm( "TrafficManager" ));

	if( pTrafMan ) 
	{
		pTrafMan->SetDialInputSet( m_setName );
	}
	CHcsmCollection::SetActionUseTrafManSetLog( this->m_triggerId, m_setName );
}
