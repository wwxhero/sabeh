/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: vehfailactn.cxx,v 1.6 2005/08/04 15:56:35 oahmad Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CVehFailActn, a subclass of CAction.  
//	This class causes a particular failure on the own-vehicle.
//
/////////////////////////////////////////////////////////////////////////////

#include "vehfailactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"

#ifndef VEH_FAIL_DEBUG
#define VEH_FAIL_DEBUG 1

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
CVehFailActn::CVehFailActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	m_delay   = cpBlock->GetDelay();
	m_failure = cpBlock->GetFailure();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CVehFailActn
//
// Remarks: 
//
// Arguments: CVehFailActn to be copied into current CVehFailActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CVehFailActn::CVehFailActn( const CVehFailActn& cRhs )
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
CVehFailActn::~CVehFailActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CVehFailActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CVehFailActn&
CVehFailActn::operator=( const CVehFailActn& cRhs ) 
{
	if( this != &cRhs )
	{
		m_pHC = cRhs.m_pHC;
		m_failure = cRhs.m_failure;	
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
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
CVehFailActn::Execute( const set<CCandidate>* ) 
{
#if VEH_FAIL_DEBUG > 0
	gout << "CVehFailActn::Execute: failure = " << m_failure << endl;
#endif

	// Find the VehFail HCSM
	const CCved* pCved = m_pHC->GetCved();
	CVehFail* pVehFail = dynamic_cast<CVehFail*>(m_pHC->GetHcsm("VehFail"));

	if( !pVehFail ) 
	{
		gout << "CVehFailActn::Execute: Error! \n"
			 << "Cannot find VehFail HCSM, so the "
			 << "failure cannot be sent." << endl;
		return;
	}

	pVehFail->SetDialFailure( m_failure );

#if VEH_FAIL_DEBUG > 0
	gout << "CVehFailActn::Execute: set Failure dial to = "
		 << m_failure << endl;
#endif
}

#endif // VEH_FAIL_DEBUG
