/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: logdataactn.cxx,v 1.12 2005/08/05 18:09:32 oahmad Exp $
// Author(s):   Jillian Vogel
// Date:        October, 1999
//
// Description: The definition of CLogDataActn, a subclass of CAction.  
//	This class logs the simulation data to the indicated file.
//
/////////////////////////////////////////////////////////////////////////////

#include "logdataactn.h"
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
CLogDataActn::CLogDataActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
{
	m_delay     = cpBlock->GetDelay();
	m_pHC       = pHC;
	m_streamNum = cpBlock->GetStream();
	m_streamVal = cpBlock->GetStreamValue();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CLogDataActn
//
// Remarks: 
//
// Arguments: CLogDataActn to be copied into current CLogDataActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CLogDataActn::CLogDataActn( const CLogDataActn& cRhs )
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
CLogDataActn::~CLogDataActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CLogDataActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CLogDataActn&
CLogDataActn::operator=( const CLogDataActn& cRhs ) 
{
	if( this != &cRhs )
	{
		m_pHC       = cRhs.m_pHC;
		m_streamNum = cRhs.m_streamNum;
		m_streamVal = cRhs.m_streamVal;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function wries a stream id and an associated value to
//  a global function.
//
// Arguments: 
// 	ignores optional parameters
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CLogDataActn::Execute( const set<CCandidate>* )
{
	// Call global function
	LogData( m_streamNum, m_streamVal );

	// set the activity log
	CHcsmCollection::SetActionLogDataLog( m_triggerId, m_streamNum, m_streamVal );
}
