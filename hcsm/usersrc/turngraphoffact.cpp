/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: turngraphoffact.cpp,v 1.1 2010/01/14 17:29:30 iowa\dheitbri Exp $
//
// Author(s):   Matt Schikore
//
// Date:        May 2004
//
// Description: The definition of CSetVarActn, a subclass of CAction.  
// 	This class causes an internal variable to be set to a specific value.
//
/////////////////////////////////////////////////////////////////////////////

#include "TurnGraphOnActn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"
#include <sstream>

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
CTurnGraphOffActn::CTurnGraphOffActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	
	m_delay = cpBlock->GetDelay();


	//vector< pair<string,string> > m_graphItems;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CSetVarActn.
//
// Remarks: 
//
// Arguments: CSetVarActn to be copied into current CSetVarActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTurnGraphOffActn::CTurnGraphOffActn( const CTurnGraphOffActn& cRhs )
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
CTurnGraphOffActn::~CTurnGraphOffActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CSetVarActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTurnGraphOffActn&
CTurnGraphOffActn::operator=( const CTurnGraphOffActn& cRhs )
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
CTurnGraphOffActn::Execute( const set<CCandidate>* ) 
{
	CHcsmCollection::m_sGraphIsOn = false;
}