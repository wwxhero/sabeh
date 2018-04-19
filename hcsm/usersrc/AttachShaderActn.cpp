/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2015 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: AttachShaderActn.cpp,v 1.1 2015/08/13 14:54:13 IOWA\dheitbri Exp $
// Author(s):   Omar Ahmad
// Date:        September, 2004
//
// Description: The definition of CAttachShaderActn, a subclass of CAction.  
// 	This class writes a value to variable that's mapped to a cell on the
//  scramnet.
//
/////////////////////////////////////////////////////////////////////////////

#include "AttachShaderActn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"
const char CAttachShaderActn::m_sName[] ="AttachShader";

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
CAttachShaderActn::CAttachShaderActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	InitCandidateSet( cpBlock );
	m_shaderName = cpBlock->GetShaderName();
	m_pragmas    = cpBlock->GetShaderOption();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CAttachShaderActn.
//
// Remarks: 
//
// Arguments: CAttachShaderActn to be copied into current CAttachShaderActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CAttachShaderActn::CAttachShaderActn( const CAttachShaderActn& cRhs )
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
CAttachShaderActn::~CAttachShaderActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: A reference to the CAttachShaderActn to assign.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CAttachShaderActn&
CAttachShaderActn::operator=( const CAttachShaderActn& cRhs )
{
	if( this != &cRhs )
	{
		m_shaderName = cRhs.m_shaderName;
		m_pragmas    = cRhs.m_pragmas;
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
CAttachShaderActn::Execute( const set<CCandidate>* ) 
{
//TAttachShaderCmd CHcsmCollection::m_sAttachShaderCmds[cMAX_SET_UNIFORM_CELL_EVENTS];
//int CHcsmCollection::m_sAttachShaderCmdsDataSize;
//
	set<CCandidate> candidateSet;
 	m_candidateSet.GetCandidates( *(m_pHC->GetCved()), candidateSet, *m_pHC );
	int numObjects = candidateSet.size(); 
	int eventNum = CHcsmCollection::m_sAttachShaderCmdsDataSize;
	bool haveSpace = eventNum < cMAX_SCENARIO_WRITE_CELL_EVENTS;
	if( haveSpace )
	{
#if 0
		gout << "###[" << eventNum << "] inserting " << m_uniformName << endl;
		gout << "     type = " << m_cellType << endl;
#endif		
		auto &idSet = CHcsmCollection::m_sAttachShaderCmds[eventNum].cvedIDs;
		for (auto itr =candidateSet.begin(); itr != candidateSet.end();++itr){
			idSet.push_back(itr->m_cvedId);
		}
		CHcsmCollection::m_sAttachShaderCmds[eventNum].cellName = m_shaderName;
		CHcsmCollection::m_sAttachShaderCmds[eventNum].pragmas  = m_pragmas;
		CHcsmCollection::m_sAttachShaderCmdsDataSize++;
	}
	else
	{
		gout << "CAttachShaderActn:Execute: out of space on cell write event list" << endl;
#if 1
		int i;
		for( i = 0; i < CHcsmCollection::m_sAttachShaderCmdsDataSize; i++ )
		{
			gout<< i << ":" 
				<< CHcsmCollection::m_sAttachShaderCmds[i].cellName << ", " 
				<< CHcsmCollection::m_sAttachShaderCmds[i].pragmas << endl;
		}
#endif
	}
}
