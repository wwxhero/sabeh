/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: mtnbaseprpstnactn.cxx,v 1.10 2005/08/05 18:09:32 oahmad Exp $
// Author(s):   Jillian Vogel
// Date:        October, 1999
//
// Description: The definition of CMtnBasePrpstnActn, a subclass of CAction.  
//	This class prepositions the motion base
//
/////////////////////////////////////////////////////////////////////////////

#include "mtnbaseprpstnactn.h"
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
CMtnBasePrpstnActn::CMtnBasePrpstnActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHc
			)
{
	m_delay = cpBlock->GetDelay();
	m_pos = CActionParseBlock::StringToPreposition(
									cpBlock->GetMotionPreposition()
									);
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CMtnBasePrpstnActnp.
//
// Remarks: 
//
// Arguments:
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CMtnBasePrpstnActn::CMtnBasePrpstnActn( const CMtnBasePrpstnActn& cRhs )
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
CMtnBasePrpstnActn::~CMtnBasePrpstnActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the current 
//  object.
//
// Remarks: 
//
// Arguments: reference to the CMtnBasePrpstnActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CMtnBasePrpstnActn&
CMtnBasePrpstnActn::operator=( const CMtnBasePrpstnActn& cRhs )
{

	if( this != &cRhs )
	{
		m_pos = cRhs.m_pos;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs.
//
// Remarks: This function prepositions the motion base.
//
// Arguments: ignores optional parameter
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
void 
CMtnBasePrpstnActn::Execute( const set<CCandidate>* )
{
	CHcsmCollection::m_sSCC_Scen_Pos_X_Crossbeam = m_pos.crossbeam;
	CHcsmCollection::m_sSCC_Scen_Pos_Y_Carriage = m_pos.carriage;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_X = m_pos.hexX;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Y = m_pos.hexY;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Z = m_pos.hexZ;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Roll = m_pos.hexRoll;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Pitch = m_pos.hexPitch;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Yaw = m_pos.hexYaw;
	CHcsmCollection::m_sSCC_Scen_Pos_TT = m_pos.turntable;

	// set the activity log
	CHcsmCollection::SetActionPreposMotionLog( m_triggerId );
}
