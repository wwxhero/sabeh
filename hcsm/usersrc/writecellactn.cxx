/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: writecellactn.cxx,v 1.12 2016/10/12 22:49:32 IOWA\dheitbri Exp $
// Author(s):   Omar Ahmad
// Date:        September, 2004
//
// Description: The definition of CWriteCellActn, a subclass of CAction.  
// 	This class writes a value to variable that's mapped to a cell on the
//  scramnet.
//
/////////////////////////////////////////////////////////////////////////////

#include "writecellactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"


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
CWriteCellActn::CWriteCellActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	InitCandidateSet( cpBlock );
	m_cellName = cpBlock->GetCellName();
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
		else if (m_cellType == CActionParseBlock::eSTRING){
			//we already store the as a string in m_cellData
		}
		else
		{
			gout << "WriteCellAction: unknown cell data type = " << m_cellType;
			gout << endl;
		}
	}

	m_cellData = cpBlock->GetCellDataString();
#if 0
	if( m_floatData.size() > 0 )
		gout << "floatData = " << m_floatData.front() << endl;
	gout << "cellData  = " << m_cellData << endl;
#endif
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CWriteCellActn.
//
// Remarks: 
//
// Arguments: CWriteCellActn to be copied into current CWriteCellActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CWriteCellActn::CWriteCellActn( const CWriteCellActn& cRhs )
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
CWriteCellActn::~CWriteCellActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: A reference to the CWriteCellActn to assign.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CWriteCellActn&
CWriteCellActn::operator=( const CWriteCellActn& cRhs )
{
	if( this != &cRhs )
	{
		m_pHC        = cRhs.m_pHC;
		m_cellName   = cRhs.m_cellName;
		m_cellType   = cRhs.m_cellType;
		m_floatData  = cRhs.m_floatData;
		m_intData    = cRhs.m_intData;
		m_shortData  = cRhs.m_shortData;
		m_cellData   = cRhs.m_cellData;
		m_isVariable = cRhs.m_isVariable;
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
CWriteCellActn::Execute( const set<CCandidate>* ) 
{
	int eventNum = CHcsmCollection::m_sScenarioWriteCellDataSize;
	bool haveSpace = eventNum < cMAX_SCENARIO_WRITE_CELL_EVENTS;
	if( haveSpace )
	{
		CHcsmCollection::m_sScenarioWriteCellData[eventNum].cellName = m_cellName;
		CHcsmCollection::m_sScenarioWriteCellData[eventNum].cellType = m_cellType;
		if( m_isVariable )
		{
			string varName = m_cellData;
			double varVal = CHcsmCollection::GetExprVariable( varName );
			if( m_cellType == CActionParseBlock::eFLOAT )
			{
                CHcsmCollection::m_sScenarioWriteCellData[eventNum].floatData.clear();
                CHcsmCollection::m_sScenarioWriteCellData[eventNum].floatData.push_back( (float) varVal );
			}
			else if( m_cellType == CActionParseBlock::eSHORT )
			{
                CHcsmCollection::m_sScenarioWriteCellData[eventNum].shortData.clear();
                CHcsmCollection::m_sScenarioWriteCellData[eventNum].shortData.push_back( (short) varVal );
			}
			else if( m_cellType == CActionParseBlock::eINT )
			{
                CHcsmCollection::m_sScenarioWriteCellData[eventNum].intData.clear();
                CHcsmCollection::m_sScenarioWriteCellData[eventNum].intData.push_back( (int) varVal );
			}
		}
		else
		{
			if( m_cellType == CActionParseBlock::eFLOAT )
			{
				CHcsmCollection::m_sScenarioWriteCellData[eventNum].floatData = m_floatData;

				//
				// Saves false alarm info so that it can be used by the collision warning
				// system.
				//
				if( m_cellName == "SCC_False_Alarm" )
				{
					vector<float>::iterator itr;
					int index = 0;
					for( itr = m_floatData.begin(); itr != m_floatData.end(); itr++ )
					{
						CHcsmCollection::m_sFalseAlarmInfo[index] = *itr;
						index++;
					}
				}
			}
			else if( m_cellType == CActionParseBlock::eSHORT )
			{
				CHcsmCollection::m_sScenarioWriteCellData[eventNum].shortData = m_shortData;
			}
			else if( m_cellType == CActionParseBlock::eINT )
			{
				CHcsmCollection::m_sScenarioWriteCellData[eventNum].intData = m_intData;
			}
		}

		CHcsmCollection::m_sScenarioWriteCellData[eventNum].cellData = m_cellData;
		CHcsmCollection::m_sScenarioWriteCellData[eventNum].frame = CHcsmCollection::m_frame;
		CHcsmCollection::m_sScenarioWriteCellDataSize++;
	}
	else
	{
		gout << "CWriteCellActn:Execute: out of space on cell write event list" << endl;
#if 1
		int i;
		for( i = 0; i < CHcsmCollection::m_sScenarioWriteCellDataSize; i++ )
		{
			gout << i << ":" << CHcsmCollection::m_sScenarioWriteCellData[i].frame << ": " << CHcsmCollection::m_sScenarioWriteCellData[i].cellName << ", " << CHcsmCollection::m_sScenarioWriteCellData[i].cellType << ", " << CHcsmCollection::m_sScenarioWriteCellData[i].cellData << endl;
		}
#endif
	}
}
