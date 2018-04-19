/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: action.cxx,v 1.40 2016/05/05 22:13:33 IOWA\dheitbri Exp $
// Author(s):   Jillian Vogel
// Date:        June, 1999
//
// Description: The implementation of CAction functions.  This is an 
//  abstract class and so cannot be instantiated.
//
/////////////////////////////////////////////////////////////////////////////

#include "action.h"

int CAction::m_sDataRedCounter = 0;

CAction::CAction()
{
	m_delay = 0.0;
	m_useInstigators = false;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CAction.
//
// Remarks: 
//
// Arguments: CPhoneCallActn to be copied into current CPhoneCallActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CAction::CAction( const CAction& cRhs )
{
	*this = cRhs;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the current 
//  object.
//
// Remarks: 
//
// Arguments: reference to the CPhoneCallActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CAction&
CAction::operator=( const CAction& cRhs )
{
	if( this != &cRhs )
	{
		m_candidateSet   = cRhs.m_candidateSet;
		m_useInstigators = cRhs.m_useInstigators;
		m_triggerId      = cRhs.m_triggerId;
		m_delay          = cRhs.m_delay;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The destructor.
//
// Remarks: 
//
// Arguments: 
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CAction::~CAction(){
}

void 
CAction::InitCandidateSet( const CActionParseBlock* cpBlock )
{
	//
	// Examine cpBlock and initialize any given candidate set filters.
	//
	vector<string> byNameSet = cpBlock->GetByNameSet();
	if( !byNameSet.empty() )  m_candidateSet.AddNames( byNameSet );

	vector<string> byTypeSet = cpBlock->GetByTypeSet();
	if( !byTypeSet.empty() )  m_candidateSet.AddTypes( byTypeSet );

	vector<pbTPstn> byPstnSet = cpBlock->GetByPstnSet();
	if( !byPstnSet.empty() )  m_candidateSet.AddPstns( byPstnSet );

	vector<pbTRoad> byRoadSet = cpBlock->GetByRoadSet();
	if( !byRoadSet.empty() )  m_candidateSet.AddRoads( byRoadSet );

	CActionParseBlock::TRelativeInfo info = cpBlock->GetRelativeSet();
	if( info.m_params.size() > 0 )
	{
		m_candidateSet.SetRelative( info );
	}
	
	m_useInstigators = cpBlock->GetInstigatorSet();
	m_delay = cpBlock->GetDelay();
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the delay associated with this action.
//
// Remarks:
//
// Arguments:
//
// Returns:  A double representing the delay.
//
//////////////////////////////////////////////////////////////////////////////
double
CAction::GetDelay() const
{
	return m_delay;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Function called to fill a vector of CAction* with the 
/// proper action types.
//
// Remarks: This function parses out the information in the 
//  CTriggerParseBlock sub-blocks that indicate what subclasses of 
//  CActions should fill the action vector.
//
// Arguments:
//  pBlock - Contains the Trigger parser information.
//	pHC - Points to the HCSM Collection required by the constructors for 
//        some subclasses of CAction.
//	actions - Vector of CAction * that is to be filled by this procedure.							.
//
// Returns:  void
//
//////////////////////////////////////////////////////////////////////////////
void 
GetActions(
			const CTriggerParseBlock* cpBlock, 
			CHcsmCollection* pHC, 
			CAction::TActionVec &actions,
			bool debug
			)
{
	// Iterate through the sub-blocks of the trigger parse block.
	CSnoBlock::cTChildIterator pChild;
	CActionParseBlock* pActnPBlock;

	for( 
		pChild = cpBlock->BeginChild(); 
		pChild != cpBlock->EndChild(); 
		pChild++
		)
	{	
		pActnPBlock = new CActionParseBlock( *pChild );
		// 
		// Examine the Name field to determine what kind of CAction 
		// to create.
		//
		if( pChild->GetBlockName() == "CreateHcsm" )
		{
			actions.push_back( CAction::TActionPtr(new CCreateHcsmActn( pActnPBlock, pHC ) ) );
		}
		else if( pChild->GetBlockName() == "DeleteHcsm" )
		{
			actions.push_back( CAction::TActionPtr(new CDeleteHcsmActn( pActnPBlock, pHC ) ) );
		}
		else if( pChild->GetBlockName() == "SetButton")
		{
			actions.push_back( CAction::TActionPtr(new CSetButtonActn( pActnPBlock, pHC ) ) );
		}
		else if( pChild->GetBlockName() == "SetDial")
		{
			actions.push_back( CAction::TActionPtr(new CSetDialActn( pActnPBlock, pHC, debug )) );
		}
		else if( pChild->GetBlockName() == "ResetDial")
		{
			actions.push_back( CAction::TActionPtr(new CResetDialActn( pActnPBlock, pHC )) );
		}
		else if( pChild->GetBlockName() == "PlayAudio")
		{
			actions.push_back( CAction::TActionPtr(new CPlayAudioActn( pActnPBlock, pHC ) ));
		}
        else if (pChild->GetBlockName() == "SetVisualDisplayText")
        {
            actions.push_back(CAction::TActionPtr(new CSetDisplayText(pActnPBlock, pHC) ));
        }
        else if( pChild->GetBlockName() == "TransitionTrffcLght")
		{
			actions.push_back( CAction::TActionPtr(new CTransitionTrffcLghtActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "LogData")
		{	
			actions.push_back( CAction::TActionPtr(new CLogDataActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "TrmntSmltn")
		{
			actions.push_back( CAction::TActionPtr(new CTrmntSmltnActn( pActnPBlock, pHC )) );
		}
		else if( pChild->GetBlockName() == "MtnBasePrpstn")
		{
			actions.push_back( CAction::TActionPtr(new CMtnBasePrpstnActn( pActnPBlock, pHC )) );
		}
		else if( pChild->GetBlockName() == "MtnBaseTune")
		{
			actions.push_back( CAction::TActionPtr(new CMtnBaseTuneActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "VehFailActn")
		{
			actions.push_back( CAction::TActionPtr(new CVehFailActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "PhoneCall")
		{
			actions.push_back( CAction::TActionPtr(new CPhoneCallActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "UseTrafManSet")
		{
			actions.push_back( CAction::TActionPtr(new CUseTrafManSetActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "StartDataRed" )
		{
			actions.push_back(CAction::TActionPtr( new CStartDataredActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "StopDataRed" )
		{
			actions.push_back( CAction::TActionPtr(new CStopDataredActn( pActnPBlock, pHC ) ));
		}
		else if( pChild->GetBlockName() == "SetVar" )
		{
			actions.push_back( CAction::TActionPtr(new CSetVarActn( pActnPBlock, pHC ) ) );
		}
		else if( pChild->GetBlockName() == "SetPosVar" )
		{
			actions.push_back( CAction::TActionPtr(new CSetPosVarActn( pActnPBlock, pHC ) ) );
		}
		else if( pChild->GetBlockName() == "WriteCell" )
		{
			actions.push_back( CAction::TActionPtr(new CWriteCellActn( pActnPBlock, pHC ) ) );
		}
		else if( pChild->GetBlockName() == "DisplayGraph" )
		{
			actions.push_back( CAction::TActionPtr(new CTurnGraphOnActn( pActnPBlock, pHC ) ) );
		}
		else if( pChild->GetBlockName() == "TurnOffGraph" )
		{
			actions.push_back( CAction::TActionPtr(new CTurnGraphOffActn( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == "SetVisualDisplayText"){
			actions.push_back( CAction::TActionPtr(new CSetDisplayText( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == "LoadVar"){
			actions.push_back( CAction::TActionPtr(new CLoadVar( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == "StoreVar"){
			actions.push_back( CAction::TActionPtr(new CStoreVar( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == "ChangeCabSetting"){
			actions.push_back( CAction::TActionPtr(new CChangeCabSetting( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == "VarQueueOperation"){
			actions.push_back( CAction::TActionPtr(new CVarQueueOperationAction( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == "SetHeadlight"){
			actions.push_back( CAction::TActionPtr(new CSetHeadlights( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == CCreateRandomGen::Name()){
			actions.push_back( CAction::TActionPtr(new CCreateRandomGen( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == CWriteUniformActn::Name()){
			actions.push_back( CAction::TActionPtr(new CWriteUniformActn( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == CAttachShaderActn::Name()){
			actions.push_back( CAction::TActionPtr(new CAttachShaderActn( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == CSetSwitchActn::Name()){
			actions.push_back( CAction::TActionPtr(new CSetSwitchActn( pActnPBlock, pHC ) ) );
		}
		else if ( pChild->GetBlockName() == CUpdateTodActn::Name()){
			actions.push_back( CAction::TActionPtr(new CUpdateTodActn( pActnPBlock, pHC ) ) );
		}

            
	}
}

