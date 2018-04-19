/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setposvaractn.cxx,v 1.1 2010/01/14 17:29:30 iowa\dheitbri Exp $
//
// Author(s):   Matt Schikore
//
// Date:        May 2004
//
// Description: The definition of CSetVarActn, a subclass of CAction.  
// 	This class causes an internal variable to be set to a specific value.
//
/////////////////////////////////////////////////////////////////////////////

#include "setposvaractn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"


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
CSetPosVarActn::CSetPosVarActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	m_delay = cpBlock->GetDelay();
	m_varName = cpBlock->GetVarName();
	m_varValue = cpBlock->GetVarValue();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CSetVarActn.
//
// Remarks: 
//
// Arguments: CSetVarActn to be copied into current CSetVarActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetPosVarActn::CSetPosVarActn( const CSetPosVarActn& cRhs )
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
CSetPosVarActn::~CSetPosVarActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CSetVarActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetPosVarActn&
CSetPosVarActn::operator=( const CSetPosVarActn& cRhs )
{
	if( this != &cRhs)
	{
		m_pHC = cRhs.m_pHC;
		m_varName = cRhs.m_varName;
		m_varValue = cRhs.m_varValue;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The Execute function, which is called when the action
//              should be executed
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////

void 
CSetPosVarActn::Execute( const set<CCandidate>* ) 
{
/*	enum eFlashingLightMode {
		eNO_LIGHTS = 0,
		eMIRROR_LIGHTS,
		eA_PILLAR_LIGHTS,
		eBOTH
	};

	enum eBUDCameraDisplayMode {
		eNO_DISPLAY = 0,
		eINSTRUMENT_PANEL,
		eINTERIOR_MIRROR
	};
*/
	double newValue = 0.0f;
	if( m_varValue == "OwnVehiclePos" )
	{
		CVED::CCved* pCved= m_pHC->GetCved();
		CPoint3D pos;
		if (pCved->GetOwnVehiclePos(pos))
			CHcsmCollection::SetExprPosVariable(m_varName,pos);

	}


}

