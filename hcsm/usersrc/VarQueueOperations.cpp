#include "hcsmspec.h"
#include "action.h"
#include "VarQueueOperations.h"


/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2013 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: 
// Author(s):   Heitbrink, David
// Date:        March 2013
//
// Description: This file is the implementation of the CVarQueueOperationAction class
//
/////////////////////////////////////////////////////////////////////////////



CVarQueueOperationAction::CVarQueueOperationAction( const CActionParseBlock* pBlock, CHcsmCollection* pColl)
{
    m_varOperation = pBlock->GetVarQueueOperation();
    m_delay = pBlock->GetDelay();
	assert(pBlock);
}
////////////////////////////////////////////////////////////////////////
///\remark
///		This stores values from to the specified text file
///\remark
///		variables are always to be doubles, variables are delimted by ;  
////////////////////////////////////////////////////////////////////////
void CVarQueueOperationAction::Execute( const set<CCandidate>* cObjs  ){
    CHcsmCollection::DoVarQueueOperation(m_varOperation);

}
CVarQueueOperationAction& 
CVarQueueOperationAction::operator=( const CVarQueueOperationAction& cRhs ){
	if (this != &cRhs){
		m_varOperation = cRhs.m_varOperation;
	}
	return *this;
}
CVarQueueOperationAction::CVarQueueOperationAction(const CVarQueueOperationAction&  cRhs){
	*this =  cRhs;
}
CVarQueueOperationAction::~CVarQueueOperationAction(void)
{
}
