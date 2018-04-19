/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:  $Id: trffclghttrigger.cxx,v 1.20 2012/07/30 17:28:03 IOWA\dheitbri Exp $
 *
 * Author:   Jillian Vogel
 * Date:     October, 1999
 *
 * Description: Provides code for the 3 main functions of the 
 * 	TrffcLghtTrigger HCSM.
 *
 ****************************************************************************/

#include "genhcsm.h"
#ifdef _WIN32
#include <strstream>
#elif _PowerMAXOS
#include <strstream>
#define ostringstream ostrstream
#elif __sgi
#include <strstream.h>
#endif

void 
CTrffcLghtTrigger::UserCreation( const CTriggerParseBlock* cpBlock )
{
	PrintCreationMessage();
    m_priorityLevel = cpBlock->GetPriority();
	try 
	{
		// Initialize CObjectInitCond object
		m_initCondition.SetObjectInitCond(
									*cved, 
									GetFrame(), 
									GetTimeStepDuration(),  
									cpBlock->GetCrRad(),  
									cpBlock->GetActvDel(), 
									cpBlock->GetLifetime()
									);

		// Initialize CTriggerFireCond object
		m_fireCondition.InitFireCondition(
									GetTimeStepDuration(),
									cpBlock->GetFireDelFrames(),
									cpBlock->GetDebounce(),
									cpBlock->GetOneShot()
									);

		// Initialize other local variables
		m_prevPosition = cpBlock->GetPosition();
		m_prevState = eWAIT;
		m_pTrigger = 0;
		m_state = cvStringToTrafficLightState( cpBlock->GetState().c_str() );

		// Initialize the action vector
		GetActions( cpBlock, m_pRootCollection, m_pActionVector );
		m_sequentialActions = cpBlock->GetSequentialActions();
		m_pCurrentAction = m_pActionVector.end();
		m_delayFrameCount = 0;
		m_waitingForAction = false;

		InitCandidates( cpBlock );

		//
		// Make an entry into the activity log for HCSM creation.
		//
		m_pRootCollection->SetHcsmCreateLog( 
					this, 
					m_typeId, 
					m_name, 
					cpBlock->GetPosition() 
					);
	} 
	catch( CSnoBlock::TCountError e )
	{
		cerr << MessagePrefix() << "caught exception TCountError...[SUICIDE]";
		cerr << endl;

		Suicide();
		return;
	}
	catch( ... )
	{
		cerr << MessagePrefix() << "caught unknown exception...[SUICIDE]";
		cerr << endl;

		Suicide();
		return;
	}
}

void 
CTrffcLghtTrigger::UserActivity( const CTriggerParseBlock* cpBlock )
{
	// If the MakeTraffic button is pressed, then fire the action
	if( GetButtonFireTrigger() )
	{
		ExecuteConcurrentActions();
		return;
	}

	// Get current gmtrcpstn and position
	int actualTime = GetFrame();

	CPoint3D actualPosition;
	if( m_prevState == eUNDER_CONTROL )
	{
		actualPosition = m_pTrigger->GetPosImm();
	}
	else
	{
		actualPosition = m_prevPosition;
	}

	// Get the current state from the InitCondition object
	EInitCond state = m_initCondition.Execute( actualPosition, actualTime );
			
	// For use in eACTIVATE state
	cvTObjAttr triggerAttr = { 0 };
	switch( state )
	{
		//////////////////////////////////////////////////////////////////////
		// eWAIT
		//
		case eWAIT:
			// Don't do anything yet.
			break;	// case eWAIT


		//////////////////////////////////////////////////////////////////////
		// eACTIVATE
		//
		case eACTIVATE:
			{
				//
				// Initialize the CVED representation of the Trigger.  
				// Because we need to be able to fire it at any time 
				// using the FireTrigger button, it should exist in 
				// CVED for the duration of the HCSM.
				//
				triggerAttr.hcsmId = m_pRootCollection->GetHcsmId( this );
				triggerAttr.xSize = 2.0;
				triggerAttr.ySize = 2.0;

				m_pTrigger = dynamic_cast<CCoordinatorObjectObj *> 
								(
									cved->CreateDynObj(
												cpBlock->GetName(),
												eCV_COORDINATOR, 
												m_typeId,
												triggerAttr,
												&actualPosition,
												0, 
												0
												)
								);
				if( !m_pTrigger )
				{
					cerr << MessagePrefix() << cpBlock->GetName();
					cerr << " Unable to create self in CVED... [SUICIDE]";
					cerr << endl;

					Suicide();
					return;
				}

				// Set the actions' parents to be this trigger's HCSM id
				for (CAction::TActionIterator i = m_pActionVector.begin(); i != m_pActionVector.end(); i++) {
					(*i)->SetTriggerId( m_pTrigger->GetHcsmId() );
				}

				// Evaluate the predicate if the Fire Condition object 
				// says to.
				bool evalPredicate = m_fireCondition.GetState() == eEVAL_PRED;
				if( evalPredicate )
				{
					bool result = Evaluate();
					m_fireCondition.SetPredicate( result );
				}

				// Fire the action if the Fire Condition object says so.
				bool fire = m_fireCondition.Execute( actualTime ) == eFIRE;
				if( fire )
				{
					CHcsmDebugItem item;
					item.SetTriggerFire(this->m_pTrigger->GetId());
					m_pRootCollection->LogDebugItem(item);

					if( m_sequentialActions )
					{
						InitializeSequentialActions();
					}
					else
					{
						ExecuteConcurrentActions();
					}
				}

				if( m_sequentialActions )
				{
					ExecuteSequentialActions();
				}
			}

			break;	// case eACTIVATE

		//////////////////////////////////////////////////////////////////////
		// eUNDER_CONTROL
		//
		case eUNDER_CONTROL:
			{
				//
				// Evaluate the predicate if the Fire Condition object 
				// says to.
				//
				bool evalPredicate = m_fireCondition.GetState() == eEVAL_PRED;
				if( evalPredicate )
				{
					bool result = Evaluate();
					m_fireCondition.SetPredicate( result );
				}

				// Fire the action if the Fire Condition object says so.
				bool fire = m_fireCondition.Execute( actualTime ) == eFIRE;
				if( fire )
				{
					CHcsmDebugItem item;
					item.SetTriggerFire(this->m_pTrigger->GetId());
					m_pRootCollection->LogDebugItem(item);

					if( m_sequentialActions )
					{
						InitializeSequentialActions();
					}
					else
					{
						ExecuteConcurrentActions();
					}
				}

				if( m_sequentialActions )
				{
					ExecuteSequentialActions();
				}
			}

			break;	// case eUNDER_CONTROL

		//////////////////////////////////////////////////////////////////////
		// eDELETE
		//
		case eDELETE:
			DeleteHcsm( this );

			break;	// case eDELETE


		//////////////////////////////////////////////////////////////////////
		// eEXIT
		//
		case eEXIT:
			break;	// case eEXIT
	}

	// Set m_prev* local variables for next time step.
	m_prevState = state;
	m_prevPosition = actualPosition;
}

void 
CTrffcLghtTrigger::UserDeletion( const CTriggerParseBlock* cpBlock )
{
	PrintDeletionMessage();

	if( m_pTrigger != 0 )  cved->DeleteDynObj( m_pTrigger );

	//
	// Add an entry to the activity log for HCSM deletion.
	//
	m_pRootCollection->SetHcsmDeleteLog( 
				this, 
				cpBlock->GetPosition() 
				);
}

void
CTrffcLghtTrigger::InitCandidates( const CTriggerParseBlock* cpBlock )
{
	// Initialize the candidate sets associated with this trigger
	vector<string> byNameSet = cpBlock->GetByNameSet();
	if( !byNameSet.empty() )
	{
		m_candidateSet.AddNames( byNameSet );
		m_byNameSet = byNameSet;
	}

	vector<pbTPstn> byPstnSet = cpBlock->GetByPstnSet();
	if( !byPstnSet.empty() )
	{
		m_candidateSet.AddPstns( byPstnSet );
	}

	vector<pbTRoad> byRoadSet = cpBlock->GetByRoadSet();
	if( !byRoadSet.empty() )
	{
		m_candidateSet.AddRoads( byRoadSet );
	}
}

bool
CTrffcLghtTrigger::Evaluate( void )
{
	set<CCandidate> candidateSet;
	set<CCandidate>::const_iterator itr;

	if( !m_byNameSet.empty() )
	{
		CCandidate cand;
		int cvedId;
		cved->GetObj( m_byNameSet[0], cvedId );
		cand.m_cvedId = cvedId;
		candidateSet.insert( cand );
	}

	//
	// If one or more of the candidates is within Radius of the 
	// FirePosition, the the Trigger should fire.  Place the HCSM 
	// IDs of all the candidates that meet that criteria into the 
	// instigator set.
	//
	m_instigatorSet.clear();

	// For each candidate object
	for( itr = candidateSet.begin(); itr != candidateSet.end(); itr++ )
	{
		// If the state is set to the proper value, 
		// add the traffic light to the instigator set.
		if( m_state == cved->GetTrafficLightState( itr->m_cvedId ) )
		{
			m_instigatorSet.insert( *itr );
		}
	}

	//
	// If there's object(s) in the instigator set, then something 
	// caused the trigger to fire, so return true.  Otherwise, 
	// return false.
	//
	return !( m_instigatorSet.empty() );
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes the actions concurrently.  That is, all actions
//  associated with this trigger are executed at the same time.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CTrffcLghtTrigger::ExecuteConcurrentActions()
{
	//
	// Exeucte actions concurrently.
	//
	CAction::TActionIterator aI;
	for( aI = m_pActionVector.begin(); aI != m_pActionVector.end(); aI++ )
	{
		(*aI)->Execute( &m_instigatorSet );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the sequential actions if needed.  
//
// Remarks:  Sets the m_pCurrentAction member variable to point to the
//  first action on the action vector.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CTrffcLghtTrigger::InitializeSequentialActions()
{
	//
	// Execute actions sequentially.
	//
	bool needToInitialize = m_pCurrentAction == m_pActionVector.end();
	if( needToInitialize )
	{
		m_pCurrentAction = m_pActionVector.begin();
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes the actions sequentially.  That is, each action
//  is executed in sequence from the first action to the last.  
//
// Remarks:  In order for the next action to be executed, the first action
//  has to complete itself.  This is most applicable when setting dials on
//  an HCSM.  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CTrffcLghtTrigger::ExecuteSequentialActions()
{
	bool actionsRemaining = m_pCurrentAction != m_pActionVector.end();
	if( actionsRemaining )
	{
		if( m_waitingForAction )
		{
			bool isFinished = (*m_pCurrentAction)->IsFinished();
			if( isFinished )
			{
				m_waitingForAction = false;
				m_pCurrentAction++;
				actionsRemaining = m_pCurrentAction != m_pActionVector.end();
			}
		}
		if (actionsRemaining && !m_waitingForAction)
		{
			//
			// Wait to make sure that the delay counter has expired before
			// executing the current action.
			//
			bool readyToExecute = m_delayFrameCount <= 0;
			if( readyToExecute )
			{
				(*m_pCurrentAction)->Execute( &m_instigatorSet );
				m_waitingForAction = true;
				m_delayFrameCount = 
					static_cast<int>((*m_pCurrentAction)->GetDelay() / GetTimeStepDuration());
			}
			else
			{
				m_delayFrameCount--;
			}
		}
	}
}
