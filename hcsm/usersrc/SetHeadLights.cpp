//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2014 by National Advanced Driving Simulator and
// Simulation Center, The University of Iowa and the University of Iowa. 
// All rights reserved.
//
// Version:		$Id: SetHeadLights.cpp,v 1.4 2014/05/27 23:32:54 iowa\dheitbri Exp $
// Author(s):   David Heitbrink
// Date:        Spring 2014
//
// Description: This file contains imp for CSetHeadlights
//
/////////////////////////////////////////////////////////////////////////////
#include "hcsmspec.h"
#include "action.h"
#include "SetHeadlights.h"


CSetHeadlights::CSetHeadlights( const CActionParseBlock* pBlock, CHcsmCollection* pColl)
{
	assert(pBlock);
	m_headlightAction = pBlock->GetHeadlightControlCommand();
}
CSetHeadlights& 
CSetHeadlights::operator=( const CSetHeadlights& cRhs ){
	if (this != &cRhs){
		m_headlightAction = cRhs.m_headlightAction;
	}
	return *this;
}
CSetHeadlights::CSetHeadlights(const CSetHeadlights&  cRhs){
	*this =  cRhs;
}
////////////////////////////////////////////////////////////////////////
///\remark
///		copies headlight command to m_sHeadlightScenarioControl
////////////////////////////////////////////////////////////////////////
void CSetHeadlights::Execute( const set<CCandidate>* cObjs ){
	CHcsmCollection::m_sHeadlightScenarioControl = m_headlightAction;
}

CSetHeadlights::~CSetHeadlights(void)
{
}