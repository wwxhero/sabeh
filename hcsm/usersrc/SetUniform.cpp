/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2015 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: SetUniform.cpp,v 1.3 2015/09/25 14:09:48 IOWA\dheitbri Exp $
// Author(s):   Omar Ahmad
// Date:        September, 2004
//
// Description: The definition of CWriteUniformActn, a subclass of CAction.  
// 	This class writes a value to variable that's mapped to a cell on the
//  scramnet.
//
/////////////////////////////////////////////////////////////////////////////

#include "SetUniform.h"
#include "hcsmcollection.h"
#include "genhcsm.h"
const char CWriteUniformActn::m_sName[] ="WriteUniform";

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
CWriteUniformActn::CWriteUniformActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	InitCandidateSet( cpBlock );
	m_uniformName = cpBlock->GetUniformName();
	m_cellType = cpBlock->GetCellType();
	m_isVariable = cpBlock->IsCellTypeVariable();
	if( !m_isVariable )
	{
		if( m_cellType == CActionParseBlock::eFLOAT )
		{
			m_floatData = cpBlock->GetCellDataFloat();
		}
		else if( m_cellType == CActionParseBlock::eSHORT )
		{
			m_shortData = cpBlock->GetCellDataShort();
		}
		else if( m_cellType == CActionParseBlock::eINT )
		{
			m_intData = cpBlock->GetCellDataInt();
		}
		else
		{
			gout << "WriteCellAction: unknown cell data type = " << m_cellType;
			gout << endl;
		}
	}

	m_varName = cpBlock->GetCellDataString();
#if 0
	if( m_floatData.size() > 0 )
		gout << "floatData = " << m_floatData.front() << endl;
	gout << "cellData  = " << m_varName << endl;
#endif
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CWriteUniformActn.
//
// Remarks: 
//
// Arguments: CWriteUniformActn to be copied into current CWriteUniformActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CWriteUniformActn::CWriteUniformActn( const CWriteUniformActn& cRhs )
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
CWriteUniformActn::~CWriteUniformActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: A reference to the CWriteUniformActn to assign.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CWriteUniformActn&
CWriteUniformActn::operator=( const CWriteUniformActn& cRhs )
{
	if( this != &cRhs )
	{
		m_pHC        = cRhs.m_pHC;
		m_uniformName   = cRhs.m_uniformName;
		m_cellType   = cRhs.m_cellType;
		m_floatData  = cRhs.m_floatData;
		m_intData    = cRhs.m_intData;
		m_shortData  = cRhs.m_shortData;
		m_isVariable = cRhs.m_isVariable;
		m_varName    = cRhs.m_varName;
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
CWriteUniformActn::Execute( const set<CCandidate>* ) 
{

	set<CCandidate> candidateSet;
 	m_candidateSet.GetCandidates( *(m_pHC->GetCved()), candidateSet, *m_pHC );
	int numObjects = candidateSet.size(); 
	int eventNum = CHcsmCollection::m_sScenarioWriteUniformDataSize;
    CHcsmCollection::m_sLockVisualOptions.Lock();
	bool haveSpace = eventNum < cMAX_SCENARIO_WRITE_CELL_EVENTS;
    try{
	    if( haveSpace )
	    {
    #if 0
		    gout << "###[" << eventNum << "] inserting " << m_uniformName << endl;
		    gout << "     type = " << m_cellType << endl;
    #endif
		    for (auto itr = candidateSet.begin(); itr != candidateSet.end(); itr++){
			    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].cvedIDs.push_back(itr->m_cvedId);
		    }
		
		    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].cellName = m_uniformName;
		    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].cellType = m_cellType;
		    if( m_isVariable )
		    {
			    string varName = m_varName;
			    double varVal = CHcsmCollection::GetExprVariable( varName );
			    if( m_cellType == CActionParseBlock::eFLOAT )
			    {
                    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].floatData.clear();
                    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].floatData.push_back( (float) varVal );
			    }
			    else if( m_cellType == CActionParseBlock::eSHORT )
			    {
                    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].shortData.clear();
                    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].shortData.push_back( (short) varVal );
			    }
			    else if( m_cellType == CActionParseBlock::eINT )
			    {
                    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].intData.clear();
                    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].intData.push_back( (int) varVal );
			    }
		    }
		    else
		    {
			    if( m_cellType == CActionParseBlock::eFLOAT )
			    {
				    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].floatData = m_floatData;

			    }
			    else if( m_cellType == CActionParseBlock::eSHORT )
			    {
				    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].shortData = m_shortData;
			    }
			    else if( m_cellType == CActionParseBlock::eINT )
			    {
				    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].intData = m_intData;
			    }
		    }

		    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].cellData = m_varName;
		    CHcsmCollection::m_sScenarioWriteUniformData[eventNum].frame = CHcsmCollection::m_frame;
		    CHcsmCollection::m_sScenarioWriteUniformDataSize++;
	    }
	    else
	    {
		    gout << "CWriteUniformActn:Execute: out of space on cell write event list" << endl;
    #if 1
		    int i;
		    for( i = 0; i < CHcsmCollection::m_sScenarioWriteUniformDataSize; i++ )
		    {
			    gout<< i << ":" << CHcsmCollection::m_sScenarioWriteUniformData[i].frame << ": " 
				    << CHcsmCollection::m_sScenarioWriteUniformData[i].cellName << ", " 
				    << CHcsmCollection::m_sScenarioWriteUniformData[i].cellType << ", " 
				    << CHcsmCollection::m_sScenarioWriteUniformData[i].cellData << endl;
		    }
    #endif
	    }
    }catch(...){
          gout << "CWriteUniformActn:Execute failed" << endl;
    }
    CHcsmCollection::m_sLockVisualOptions.UnLock();
}
