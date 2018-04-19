/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setvaractn.cxx,v 1.23 2015/12/11 21:07:20 IOWA\dheitbri Exp $
//
// Author(s):   Matt Schikore
//
// Date:        May 2004
//
// Description: The definition of CSetVarActn, a subclass of CAction.  
// 	This class causes an internal variable to be set to a specific value.
//
/////////////////////////////////////////////////////////////////////////////

#include "setvaractn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"
#include <sstream>


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
CSetVarActn::CSetVarActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	m_delay = cpBlock->GetDelay();
	m_varName = cpBlock->GetVarName();
	m_varValue = cpBlock->GetVarValue();
	m_isExpr = cpBlock->GetIsVarValExpression();
    m_expr.cved = m_pHC->GetCved();
	if (m_isExpr)
		m_expr.Parse(m_varValue.c_str());
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
CSetVarActn::CSetVarActn( const CSetVarActn& cRhs )
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
CSetVarActn::~CSetVarActn() {
	m_varName.clear();
}

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
CSetVarActn&
CSetVarActn::operator=( const CSetVarActn& cRhs )
{
	if( this != &cRhs)
	{
		m_pHC = cRhs.m_pHC;
		m_varName = cRhs.m_varName;
		m_varValue = cRhs.m_varValue;
		m_isExpr = cRhs.m_isExpr;
		if (m_isExpr){
			string temps;
			cRhs.m_expr.Store(temps);
			m_expr.Load(temps);
		}
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
CSetVarActn::Execute( const set<CCandidate>* ) 
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
	if (m_isExpr){
		stringstream tval("");
        try {
            newValue = m_expr.Evaluate();
        }
        catch(CVED::cvCError cv){
            gout<<"CVED Exception"<<cv.m_msg<<endl;
            gout<<"While Evaluating"<<m_varValue<<endl;
        }catch(...){
            gout<<"Unkown excpetion while evaluating "<<m_varValue<<endl;
        }
		CHcsmCollection::SetExprVariable( m_varName, newValue );
		CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, newValue );
		return;
	}
	else if( m_varName == "VisualDisplayText" )
	{
		CHcsmCollection::SetVisualDisplayText( m_varValue );
		// TO DO: need to set the action variable log....
	}

	else if (m_varName == "HapticSeat"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		CHcsmCollection::m_sHapticSeat_IsEnabled = (int)newValue;
	}

	else if (m_varName == "SirenSpeed")
	{
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		CHcsmCollection::m_sSirenSpeed = (int)newValue * cMPH_TO_MS;
	}

	// warning system statuses
	else if (m_varName == "BswStatus"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue == 0)
			CHcsmCollection::m_sBswStatus = eBSW_NONE;
		else if (newValue == 1)
			CHcsmCollection::m_sBswStatus =  eBSW_LOW_LEFT;
		else if (newValue == 2)
			CHcsmCollection::m_sBswStatus =  eBSW_LOW_RIGHT;
		else if (newValue == 3)
			CHcsmCollection::m_sBswStatus =  eBSW_LOW_BOTH;
		else if (newValue == 4)
			CHcsmCollection::m_sBswStatus =  eBSW_HIGH_LEFT; 
		else if (newValue == 5)
			CHcsmCollection::m_sBswStatus =  eBSW_HIGH_RIGHT; 
		else if (newValue == 6)
			CHcsmCollection::m_sBswStatus =  eBSW_HIGH_BOTH;
	}
	else if (m_varName == "FcwStatus"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue == 0)
			CHcsmCollection::m_sFcwStatus = eFCW_NONE;
		else if (newValue == 1)
			CHcsmCollection::m_sFcwStatus =  eFCW_LOW;
		else if (newValue == 2)
			CHcsmCollection::m_sFcwStatus =  eFCW_HIGH;
	}
	else if (m_varName == "LdwStatus"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue == 0)
			CHcsmCollection::m_sLdwStatus = eLDW_NONE;
		else if (newValue == 1)
			CHcsmCollection::m_sLdwStatus =  eLDW_MONITORING;
		else if (newValue == 2)
			CHcsmCollection::m_sLdwStatus =  eLDW_LEFT;
		else if (newValue == 3)
			CHcsmCollection::m_sLdwStatus =  eLDW_RIGHT;
	}
	else if (m_varName == "BackUpDistance"){
		if (m_varValue == "--") {
			newValue = CHcsmCollection::m_sBackUpDistance - 1;
		}else{
			sscanf(m_varValue.c_str(), "%lf", &newValue);
		}
		CHcsmCollection::m_sBackUpDistance = (float)newValue;
	}
	// warning cue configuration
	else if (m_varName == "HapticSeat"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		CHcsmCollection::m_sHapticSeat_IsEnabled = (int)newValue;
	}
	else if (m_varName == "FlashingLightMode"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue == 0.0)
			CHcsmCollection::m_sFlashingLightMode = (CHcsmCollection::eNO_LIGHTS);
		else if (newValue == 1.0)
			CHcsmCollection::m_sFlashingLightMode = (CHcsmCollection::eMIRROR_LIGHTS);
		else if (newValue == 2.0)
			CHcsmCollection::m_sFlashingLightMode = (CHcsmCollection::eA_PILLAR_LIGHTS);
		else if (newValue == 3.0)
			CHcsmCollection::m_sFlashingLightMode = (CHcsmCollection::eBOTH);
	}
	else if (m_varName == "BUDCameraMode"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue == 0)
			CHcsmCollection::m_sBUDCameraDisplayMode =(CHcsmCollection::eNO_DISPLAY);
		else if (newValue == 1)
			CHcsmCollection::m_sBUDCameraDisplayMode =(CHcsmCollection::eINSTRUMENT_PANEL);
		else if (newValue == 2)
			CHcsmCollection::m_sBUDCameraDisplayMode =(CHcsmCollection::eINTERIOR_MIRROR);
	}
	else if (m_varName == "BSWCamera"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue > 0) {
			CHcsmCollection::m_sBSWCamera_IsEnabled = true;
		}else {
			CHcsmCollection::m_sBSWCamera_IsEnabled = false;
		}
	}
	else if (m_varName == "IPAlert"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue > 0) {
			CHcsmCollection::m_sIPAlert_IsEnabled = true;
		}else {
			CHcsmCollection::m_sIPAlert_IsEnabled = false;
		}
	}
	// warning system switch
	else if (m_varName == "BUD_IsOn"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue > 0){
			CHcsmCollection::m_sBUD_IsOn = true;
		}else{
			CHcsmCollection::m_sBUD_IsOn = false;
		}
	}
	else if (m_varName == "LCW_IsOn"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue > 0){
			CHcsmCollection::m_sLCW_IsOn = true;
		}else{
			CHcsmCollection::m_sLCW_IsOn = false;
		}
	}
	else if (m_varName == "BLS_IsOn"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue > 0){
			CHcsmCollection::m_sBLS_IsOn = true;
		}else{
			CHcsmCollection::m_sBLS_IsOn = false;
		}
	}
	else if (m_varName == "FCW_IsOn"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue > 0){
			CHcsmCollection::m_sFCW_IsOn = true;
		}else{
			CHcsmCollection::m_sFCW_IsOn = false;
		}
	}

	else if (m_varName == "Hour"){
		if ( m_varValue == "++" ){
			CHcsmCollection::m_sHour++;
			if (CHcsmCollection::m_sHour >= 24){
				CHcsmCollection::m_sHour = 0;
			}
		}
		else{
			sscanf(m_varValue.c_str(), "%lf", &newValue);
			CHcsmCollection::m_sHour = (int)newValue;
		}
		CHcsmCollection::SetExprVariable( m_varName, CHcsmCollection::m_sHour );
		CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, CHcsmCollection::m_sHour );
	}
	else if (m_varName == "Minute"){
		if ( m_varValue == "++" ){
			CHcsmCollection::m_sMinute++;
			if (CHcsmCollection::m_sMinute >= 60){
				CHcsmCollection::m_sMinute = 0;
				CHcsmCollection::m_sHour++;
				if (CHcsmCollection::m_sHour >= 24){
					CHcsmCollection::m_sHour = 0;
				}
			}
			CHcsmCollection::SetExprVariable( "Minute", CHcsmCollection::m_sMinute );
			CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, CHcsmCollection::m_sMinute );
			CHcsmCollection::SetExprVariable( "Hour", CHcsmCollection::m_sHour );
			CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, CHcsmCollection::m_sHour );
		}
		else{
			sscanf(m_varValue.c_str(), "%lf", &newValue);
			CHcsmCollection::m_sMinute = (int)newValue;
		}
	}
	else if (m_varName == "UpdateTime"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		if (newValue > 0){
			CHcsmCollection::m_sChangeTimeOfDay = true;
		}else{
			CHcsmCollection::m_sChangeTimeOfDay = false;
		}
	}
	else if (m_varName == "ACC_Range"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		CHcsmCollection::m_sSensor_Config[0] = (short)newValue;
	}
	else if (m_varName == "ACC_ConeSize"){
		sscanf(m_varValue.c_str(), "%lf", &newValue);
		CHcsmCollection::m_sSensor_Config[1] = (short)newValue;
	}
	else if (m_varName == "ACC_LaneCheck"){
		if (m_varValue == "true")
			CHcsmCollection::m_sSensor_Config[2] = 1;
		else if (m_varValue == "false")
			CHcsmCollection::m_sSensor_Config[2] = 0;
		else{
			sscanf(m_varValue.c_str(), "%lf", &newValue);
			CHcsmCollection::m_sSensor_Config[2] = (short)newValue;
		}
	}
	else
	{
		double oldValue = CHcsmCollection::GetExprVariable(m_varName);
		if (m_varValue == "++") {
			newValue = oldValue + 1;
		}
		else if (m_varValue == "--") {
			newValue = oldValue - 1;
		}
		else {
			sscanf(m_varValue.c_str(), "%lf", &newValue);
		}
		CHcsmCollection::SetExprVariable( m_varName, newValue );
		CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, newValue );
	}
}

