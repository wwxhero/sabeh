/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: createhcsmactn.cxx,v 1.12 2004/06/30 19:30:18 schikore Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        June, 1999
//
// Description: The definition of CCreateHcsmActn, a subclass of CAction.  
//				This class uses its CActionParseBlock to get the parser 
//				information for which HCSMs to create.
//
/////////////////////////////////////////////////////////////////////////////

#include "createhcsmactn.h"
#include "hcsmcollection.h"

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks: The pointer to the root HCSM collection is where the new HCSMs
//			will be created.
//
// Arguments:	
//	cpBlock - A pointer to the CActionParseBlock associated with
//		this CAction.
//  pHC - A pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CCreateHcsmActn::CCreateHcsmActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
{
	m_pHC = pHC;
	m_pActionBlock = cpBlock;
	InitCandidateSet( cpBlock );
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CCreateHcsmActn
//
// Remarks: 
//
// Arguments: CCreateHcsmActn to be copied into current CCreateHcsmActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CCreateHcsmActn::CCreateHcsmActn( const CCreateHcsmActn& cRhs )
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
CCreateHcsmActn::~CCreateHcsmActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CCreateHcsmActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CCreateHcsmActn&
CCreateHcsmActn::operator=( const CCreateHcsmActn& cRhs)
{
	if( this != &cRhs )
	{
		m_pActionBlock = cRhs.m_pActionBlock;
		m_pHC = cRhs.m_pHC;
		m_useInstigators = cRhs.m_useInstigators;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which creates the indicated HCSMs.
//
// Remarks: This function iterates through the children of its Action Block
//   and creates the HCSMs listed there.  If the HCSM specifications
//   are invalid, then an error message is displayed.
//
// Arguments: 
//   Ignores the optional parameters.
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CCreateHcsmActn::Execute( const set<CCandidate>* ) 
{
	// 
	// Iterate through all the HCSM children of the m_pActionBlock.
	//
	CSnoBlock::cTChildIterator i;
	CHcsm *pHcsm;

	CHcsmCollection::SetActionCreateLog( m_triggerId );

	for( 
		i = m_pActionBlock->BeginChild(); 
		i != m_pActionBlock->EndChild(); 
		i++
		)
	{
		// Create a new HCSM for each child
		pHcsm = m_pHC->CreateHcsm( i->GetBlockName(), *i );

		if( !pHcsm )
		{
			gout << "CCreateHcsmAction::Execute: Unable to create HCSM ";
			gout << i->GetBlockName() << endl;
		}
	}
}

