/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: trmntsmltnactn.cxx,v 1.7 2005/08/05 18:09:34 oahmad Exp $
// Author(s):   Jillian Vogel
// Date:        October, 1999
//
// Description: The definition of CTrmntSmltnActn, a subclass of CAction.  
//	This class terminates the simulation.
//
/////////////////////////////////////////////////////////////////////////////

#include "trmntsmltnactn.h"
#include "hcsmcollection.h"
#include "hcsminterface.h"

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:	
//	cpBlock - Pointer to the CActionParseBlock associated with this CAction.
//	pHC - Pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTrmntSmltnActn::CTrmntSmltnActn( 
			const CActionParseBlock* cpBlock, 
			CHcsmCollection*
			)
{
	m_delay = cpBlock->GetDelay();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//  CTrmntSmltnActn.
//
// Remarks: 
//
// Arguments: CTrmntSmltnActn to be copied into current CTrmntSmltnActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTrmntSmltnActn::CTrmntSmltnActn( const CTrmntSmltnActn& cRhs )
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
CTrmntSmltnActn::~CTrmntSmltnActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//  current action to the parameter and returns a reference  to the current 
//  object.
//
// Remarks: 
//
// Arguments: reference to the CTrmntSmltnActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTrmntSmltnActn &
CTrmntSmltnActn::operator=( const CTrmntSmltnActn& cRhs )
{

	if( this != &cRhs )
	{
		;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Calls the global TerminateSimulation function.
//
// Remarks: 
//
// Arguments: ignores optional parameter
// 	
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrmntSmltnActn::Execute( const set<CCandidate>* )
{
	// Call global function
	TerminateSimulation();
}
