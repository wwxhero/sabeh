/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: mtnbasetuneactn.cxx,v 1.7 2005/08/04 15:54:21 oahmad Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CMtnBaseTuneActn, a subclass of CAction.  
//	This class tunes the motion base
//
/////////////////////////////////////////////////////////////////////////////

#include "mtnbasetuneactn.h"
#include "hcsmcollection.h"
#include "hcsminterface.h"

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
CMtnBaseTuneActn::CMtnBaseTuneActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection*
			)
{
	m_delay = cpBlock->GetDelay();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CMtnBaseTuneActn
//
// Remarks: 
//
// Arguments: CMtnBaseTuneActn to be copied into current CMtnBaseTuneActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CMtnBaseTuneActn::CMtnBaseTuneActn( const CMtnBaseTuneActn &cRhs )
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
CMtnBaseTuneActn::~CMtnBaseTuneActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CMtnBaseTuneActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CMtnBaseTuneActn&
CMtnBaseTuneActn::operator=( const CMtnBaseTuneActn &cRhs )
{

	if( this != &cRhs )
	{
		;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs.
//
// Remarks: This function tunes the motion base.
//
// Arguments: 
// 	ignores optional parameter
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CMtnBaseTuneActn::Execute( const set<CCandidate>* )
{
	// Call global function
	MotionBaseTune();
	CHcsmCollection::SetActionTuneMotionLog( m_triggerId );
}
