/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:  $Id: timetrigger.cxx,v 1.24 2012/07/30 17:28:03 IOWA\dheitbri Exp $
 *
 * Author:   Jillian Vogel
 * Date:     August, 1999
 *
 * Description: Provides code for the 3 main functions of the TimeTrigger
 *           HCSM.
 *
 ****************************************************************************/

#include "genhcsm.h"
#include "hcsmcollection.h"
#include "support.h"
#ifdef _WIN32
#include <strstream>
#elif _PowerMAXOS
#include <strstream>
#define ostringstream ostrstream
#elif __sgi
#include <strstream.h>
#endif

void CTimeTrigger::UserCreation( const CTriggerParseBlock* cpBlock )
{
	PrintCreationMessage();
    m_priorityLevel = cpBlock->GetPriority();
	string prefix = MessagePrefix();
	strstream message;

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
		m_time = cpBlock->GetTime();
		m_prevPosition = cpBlock->GetPosition();
		m_prevState = eWAIT;
		m_pTrigger = 0;
		
		// Initialize the action vector
		GetActions( cpBlock, m_pRootCollection, m_pActionVector );
		m_sequentialActions = cpBlock->GetSequentialActions();
		m_pCurrentAction = m_pActionVector.end();
		m_delayFrameCount = 0;
		m_waitingForAction = false;

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
		cerr << MessagePrefix() << "caught exception unknown...[SUICIDE]";
		cerr << endl;

		Suicide();
		return;
	}
}

void 
CTimeTrigger::UserActivity( const CTriggerParseBlock* cpBlock )
{
	// If the MakeTraffic button is pressed, then fire the action
	if( GetButtonFireTrigger() )
	{
		ExecuteConcurrentActions();
		return;
	}

	// Get current time and position
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
				// Initialize the CVED representation of the Trigger.  
				// Because we need to be able to fire it at any time 
				// using the FireTrigger button, it should exist in 
				// CVED for the duration of the HCSM.
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
					cerr << " Unable to create self in CVED... [SUICIDE]" << endl;

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
				// Evaluate the predicate if the Fire Condition object says
				// to.
				bool evalPredicate = m_fireCondition.GetState() == eEVAL_PRED;
				if( evalPredicate )
				{
					bool result = Evaluate();
					m_fireCondition.SetPredicate( result );
				}

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
CTimeTrigger::UserDeletion( const CTriggerParseBlock* cpBlock )
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

bool
CTimeTrigger::Evaluate( void ) 
{
	double curTime = GetFrame() * GetTimeStepDuration();
	return( curTime >= m_time );
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the trigger fire activty log.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CTimeTrigger::SetTriggerFireLog( CAction::TActionIterator& aI )
{
	set<CCandidate> candidateSet;
	(*aI)->m_candidateSet.GetCandidates( *cved, candidateSet, *m_pRootCollection );

	int candidateArr[16];
	int candidateArrSize = 0;
	PutCandidatesIntoArr( candidateSet, candidateArr, candidateArrSize );

	m_pRootCollection->SetTriggerFireLog(
				this,
				-1,
				candidateArr,
				candidateArrSize
				);
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
CTimeTrigger::ExecuteConcurrentActions()
{
	//
	// Exeucte actions concurrently.
	//
	CAction::TActionIterator aI;
	for( aI = m_pActionVector.begin(); aI != m_pActionVector.end(); aI++ )
	{
//		SetTriggerFireLog( aI );
		(*aI)->Execute();
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
CTimeTrigger::InitializeSequentialActions()
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
CTimeTrigger::ExecuteSequentialActions()
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
		if( actionsRemaining && !m_waitingForAction )
		{
			//
			// Wait to make sure that the delay counter has expired before
			// executing the current action.
			//
			bool readyToExecute = m_delayFrameCount <= 0;
			if( readyToExecute )
			{
//				SetTriggerFireLog( m_pCurrentAction );
				(*m_pCurrentAction)->Execute();
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
