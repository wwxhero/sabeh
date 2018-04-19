//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmconcurrent.cxx,v 1.9 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CHcsmConcurrent class.
//
//////////////////////////////////////////////////////////////////////////////


#include "hcsmconcurrent.h"
#include "hcsmcollection.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHcsmConcurrent::CHcsmConcurrent( 
			CHcsmCollection* pRootCollection, 
			const CSnoBlock* cpSnoBlock,
			string hcsmName,
			bool rootState,
			int typeId
			):
	CHcsm( pRootCollection, cpSnoBlock, hcsmName, rootState, typeId )
{

}

CHcsmConcurrent::CHcsmConcurrent( 
			CHcsmCollection* pRootCollection, 
			string hcsmName,
			bool rootState,
			int typeId
			):
	CHcsm( pRootCollection, NULL, hcsmName, rootState, typeId )
{

}

CHcsmConcurrent::CHcsmConcurrent( const CHcsmConcurrent& cObjToCopy ):
	CHcsm(
		cObjToCopy.m_pRootCollection, 
		cObjToCopy.m_pSnoBlock, 
		cObjToCopy.m_name, 
		cObjToCopy.m_root
		)
{
	// call the assignment operator
	*this = cObjToCopy;
}

CHcsmConcurrent& CHcsmConcurrent::operator=( 
			const CHcsmConcurrent& cObjToCopy
			)
{
	// check to make sure that object passed in is not me
	if( this != &cObjToCopy ) 
	{
		// call parent's assignment operator
		CHcsm::operator=( cObjToCopy );

		// no members to copy
	}

	return *this;
}

CHcsmConcurrent::~CHcsmConcurrent()
{

}
