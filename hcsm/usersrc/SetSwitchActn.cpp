/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2015 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: SetSwitchActn.cpp,v 1.2 2015/12/17 22:26:17 IOWA\dheitbri Exp $
// Author(s):   Omar Ahmad
// Date:        September, 2004
//
// Description: The definition of CSetSwitchActn, a subclass of CAction.  
// 	This class writes a value to variable that's mapped to a cell on the
//  scramnet.
//
/////////////////////////////////////////////////////////////////////////////

#include "action.h"
#include "hcsmcollection.h"
#include "genhcsm.h"
const char CSetSwitchActn::m_sName[] ="SetSwitch";

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:	
//	pBlock - A pointer to the CActionParseBlock associated with
//			this CAction.
//	pHC - A pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetSwitchActn::CSetSwitchActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	InitCandidateSet( cpBlock );
    auto val = cpBlock->GetSwitch();
    m_switchName = val.switchName;
    m_value = val.value;

}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CSetSwitchActn.
//
// Remarks: 
//
// Arguments: CSetSwitchActn to be copied into current CSetSwitchActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetSwitchActn::CSetSwitchActn( const CSetSwitchActn& cRhs )
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
CSetSwitchActn::~CSetSwitchActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: A reference to the CSetSwitchActn to assign.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetSwitchActn&
CSetSwitchActn::operator=( const CSetSwitchActn& cRhs )
{
	if( this != &cRhs )
	{
        m_switchName = cRhs.m_switchName;
        m_value = cRhs.m_value;
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
CSetSwitchActn::Execute( const set<CCandidate>* ) 
{
    CHcsmCollection::m_sLockVisualOptions.Lock();
	set<CCandidate> candidateSet;
 	m_candidateSet.GetCandidates( *(m_pHC->GetCved()), candidateSet, *m_pHC );
	int numObjects = candidateSet.size(); 
	int eventNum = CHcsmCollection::m_sSetSwitchCmdsDataSize;
	bool haveSpace = eventNum < cMAX_SCENARIO_WRITE_CELL_EVENTS;
	if( haveSpace )
	{
#if 0
		gout << "###[" << eventNum << "] inserting " << m_uniformName << endl;
		gout << "     type = " << m_cellType << endl;
#endif		
		auto &idSet = CHcsmCollection::m_sSetSwitchCmds[eventNum].cvedIDs;
		for (auto itr =candidateSet.begin(); itr != candidateSet.end();++itr){
			idSet.push_back(itr->m_cvedId);
		}
        CHcsmCollection::m_sSetSwitchCmds[eventNum].switchName = m_switchName;
        CHcsmCollection::m_sSetSwitchCmds[eventNum].id  = m_value;
		CHcsmCollection::m_sSetSwitchCmdsDataSize++;
	}
	else
	{
		gout << "CSetSwitchActn:Execute: out of space on set switch  event list" << endl;
	}
    CHcsmCollection::m_sLockVisualOptions.UnLock();
}
