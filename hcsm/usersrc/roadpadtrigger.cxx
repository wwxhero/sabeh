/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: roadpadtrigger.cxx,v 1.39 2012/07/30 17:28:03 IOWA\dheitbri Exp $
 *
 * Author:  Jillian Vogel
 * Date:    October, 1999
 *
 * Description: Provides code for the 3 main functions of the RoadPadTrigger 
 *   HCSM.
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

#define HLOG_RPT_0  15000

void 
CRoadPadTrigger::UserCreation( const CTriggerParseBlock* cpBlock )
{
	PrintCreationMessage();
    m_priorityLevel = cpBlock->GetPriority();
	strstream message;

	try 
	{
		// Initialize CObjectInitCond object.
		m_initCondition.SetObjectInitCond(
								*cved, 
								GetFrame(), 
								GetTimeStepDuration(),  
								cpBlock->GetCrRad(),  
								cpBlock->GetActvDel(), 
								cpBlock->GetLifetime()
								);

		// Initialize CTriggerFireCond object.
		m_fireCondition.InitFireCondition(
								GetTimeStepDuration(),
								cpBlock->GetFireDelFrames(),
								cpBlock->GetDebounce(),
								cpBlock->GetOneShot()
								);

		// Initialize other local variables.
		m_prevPosition = cpBlock->GetPosition();
		m_prevState = eWAIT;
		m_pTrigger = 0;
		m_path = CPath( *cved );
		m_path.SetString( cpBlock->GetPath() );

		m_nthFromStart = cpBlock->GetNthFromStart();
		m_nthFromEnd = cpBlock->GetNthFromEnd();
		m_nAhead = cpBlock->GetVehicleAhead();
		m_nBehind = cpBlock->GetVehicleBehind();

		// Initialize the action vector.
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
		message << 
			MessagePrefix()<< 
			"Suicide: error during CRoadPadTrigger::UserCreation: " <<
			e.msg;
		Suicide();
		return;
	}
	catch(...) 
	{

		message << 
			MessagePrefix() << 
			"Suicide: Unknown error during CRoadPadTrigger::UserCreation: ";
		Suicide();
		return;
	}

}

void 
CRoadPadTrigger::UserActivity( const CTriggerParseBlock* cpBlock )
{
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here0");

	// If the MakeTraffic button is pressed, then fire the action.
	if( GetButtonFireTrigger() ) 
	{
		ExecuteConcurrentActions();
		return;
	}

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here1");

	// Get current gmtrcpstn and position.
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

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here2");

	// Get the current state from the InitCondition object.
	EInitCond state = m_initCondition.Execute( actualPosition, actualTime );
			
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here3");

	// For use in eACTIVATE state.
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

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here4");

				m_pTrigger = dynamic_cast<CCoordinatorObjectObj *> (
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
					cerr << MessagePrefix();
					cerr << cpBlock->GetName();
					cerr << " unable to create self in CVED... [SUICIDE]";
					cerr << endl;

					Suicide();
					return;
				}

				// Set the actions' parents to be this trigger's HCSM id
				for (CAction::TActionIterator i = m_pActionVector.begin(); i != m_pActionVector.end(); i++) {
					(*i)->SetTriggerId( m_pTrigger->GetHcsmId() );
				}

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here5");

				// Evaluate the predicate if the Fire Condition object 
				// says to.
				bool evalPredicate = m_fireCondition.GetState() == eEVAL_PRED;
				if( evalPredicate )
				{
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here6");

					bool result = Evaluate();
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here7");

					m_fireCondition.SetPredicate( result );
				}

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here8");

				// Fire the action if the Fire Condition object says so.
				bool fire = m_fireCondition.Execute( actualTime ) == eFIRE;
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here9");

				if( fire )
				{
					CHcsmDebugItem item;
					item.SetTriggerFire(this->m_pTrigger->GetId());
					m_pRootCollection->LogDebugItem(item);
					if( m_sequentialActions )
					{
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here10");

						InitializeSequentialActions();
					}
					else
					{
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here11");

						ExecuteConcurrentActions();
					}
				}

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here12");

				if( m_sequentialActions )
				{
					ExecuteSequentialActions();
				}
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here13");

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
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here14");
				bool evalPredicate = m_fireCondition.GetState() == eEVAL_PRED;
				if( evalPredicate )
				{
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here15");

					bool result = Evaluate();
					m_fireCondition.SetPredicate( result );
				}

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here16");

				// Fire the action if the Fire Condition object says so.
				bool fire = m_fireCondition.Execute( actualTime ) == eFIRE;
				if( fire )
				{
					CHcsmDebugItem item;
					item.SetTriggerFire(this->m_pTrigger->GetId());
					m_pRootCollection->LogDebugItem(item);
					if( m_sequentialActions )
					{
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here17");

						InitializeSequentialActions();
					}
					else
					{
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here18");

						ExecuteConcurrentActions();
					}
				}

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here19");

				if( m_sequentialActions )
				{
					ExecuteSequentialActions();
				}
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here20");

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
CRoadPadTrigger::UserDeletion( const CTriggerParseBlock* cpBlock )
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
CRoadPadTrigger::InitCandidates( const CTriggerParseBlock* cpBlock )
{
	// Initialize the candidate sets associated with this trigger.
	vector<string> byNameSet = cpBlock->GetByNameSet();
	if( !byNameSet.empty() )  m_candidateSet.AddNames( byNameSet );

	vector<string> byTypeSet = cpBlock->GetByTypeSet();
	if( !byTypeSet.empty() )  m_candidateSet.AddTypes( byTypeSet );

	vector<pbTRoad> byRoadSet = cpBlock->GetByRoadSet();
	if( !byRoadSet.empty() )  m_candidateSet.AddRoads( byRoadSet );
}

bool
CRoadPadTrigger::Evaluate( void )
{
	//
	// Find the objects on the defined roadpad path.
	//
	vector<int> padObjs;
	m_path.GetObjectsOnPath( padObjs );
	if( 0 == padObjs.size() )  return false;

	//
	// Build a candidate set of objects on pad.
	//
	CCandidate tmpCand;
	vector<CCandidate> candidatePadObjs;
	vector<int>::const_iterator vecItr;
	for( vecItr = padObjs.begin(); vecItr != padObjs.end(); vecItr++ )
	{
		tmpCand.m_cvedId = *vecItr;
		tmpCand.m_hcsmId = cved->GetObjHcsmId( tmpCand.m_cvedId );
		candidatePadObjs.push_back( tmpCand );
	}

	//
	// Assemble the set of candidates and build the instigator set.
	//
	set<CCandidate> candidateSet;
	m_candidateSet.GetCandidates( *cved, candidateSet, *m_pRootCollection );
	
	//
	// If one or more of the candidates lies on the RoadPad  
	// then the Trigger should fire. Place the HCSM IDs of all 
	// the candidates that meet that criteria into the instigator 
	// set.
	//
	m_instigatorSet.clear();

	vector<CCandidate> inOrder;
	vector<CCandidate>::iterator i;
	for( i = candidatePadObjs.begin(); i != candidatePadObjs.end(); i++ )
	{
		if( candidateSet.find( *i ) != candidateSet.end() )
		{
			m_instigatorSet.insert( *i );
			inOrder.push_back( *i );

			if( m_pRootCollection->m_verbose )
			{
				if( i->m_cvedId == 0 )
				{
					gout << MessagePrefix();
					gout << "*** Trigger firing for DRIVER ***" << endl;
				}
			}
		}
	}

	// Now check for "Nth from start" or "Nth from end" flags.
	if( m_nthFromStart )
	{
		m_instigatorSet.clear();
		if( m_nthFromStart - 1 <= inOrder.size() && inOrder.size() > 0 )
		{
			m_instigatorSet.insert( inOrder[m_nthFromStart - 1] );
		}
	}
	else if( m_nthFromEnd )
	{
		m_instigatorSet.clear();
		if( m_nthFromEnd - 1 <= inOrder.size() && inOrder.size() > 0 )
		{
			m_instigatorSet.insert( inOrder[inOrder.size() - m_nthFromEnd] );
		}
	}

#if 0
	gout << MessagePrefix() << " candidates on path: ";
	set<CCandidate>::iterator itr;
	for( itr = m_instigatorSet.begin(); itr != m_instigatorSet.end(); itr++ )
	{
		gout << "[" << itr->hcsmId << " " << itr->cvedId << "]";
	}
	gout << endl;
#endif

	// If there's object(s) in the instigator set, then something 
	// caused the trigger to fire, so return true.  Otherwise, 
	// return false.
	return !( m_instigatorSet.empty() );
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
CRoadPadTrigger::SetTriggerFireLog( CAction::TActionIterator& aI )
{
	int instigatorHcsmId = GetInstigatorHcsmId( m_instigatorSet );

	set<CCandidate> candidateSet;
	(*aI)->m_candidateSet.GetCandidates( *cved, candidateSet, *m_pRootCollection);

	int candidateArr[16];
	int candidateArrSize = 0;
	if( (*aI)->m_useInstigators  )
	{
		PutCandidatesIntoArr( m_instigatorSet, candidateArr, candidateArrSize );
	}
	else
	{
		PutCandidatesIntoArr( candidateSet, candidateArr, candidateArrSize );
	}

	m_pRootCollection->SetTriggerFireLog(
				this,
				instigatorHcsmId,
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
CRoadPadTrigger::ExecuteConcurrentActions()
{
	//
	// Exeucte actions concurrently.
	//
	CAction::TActionIterator aI;
	for( aI = m_pActionVector.begin(); aI != m_pActionVector.end(); aI++ )
	{
		SetTriggerFireLog( aI );
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
CRoadPadTrigger::InitializeSequentialActions()
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
CRoadPadTrigger::ExecuteSequentialActions()
{
	bool actionsRemaining = m_pCurrentAction != m_pActionVector.end();
	if( actionsRemaining )
	{
		if( m_waitingForAction )
		{
#ifdef DEBUG
			gout << MessagePrefix() << "waiting for action to complete";
			gout << endl;
#endif

			bool isFinished = (*m_pCurrentAction)->IsFinished();
			if( isFinished )
			{
#ifdef DEBUG
				gout << MessagePrefix() << "action completed" << endl;
#endif

				m_waitingForAction = false;
				m_pCurrentAction++;
				actionsRemaining = m_pCurrentAction != m_pActionVector.end();
			}
		}
		if (actionsRemaining && !m_waitingForAction)
		{
#ifdef DEBUG
			gout << MessagePrefix() << "delay = " << m_delayFrameCount << endl;
#endif

			//
			// Wait to make sure that the delay counter has expired before
			// executing the current action.
			//
			bool readyToExecute = m_delayFrameCount <= 0;
			if( readyToExecute )
			{
#ifdef DEBUG
				gout << MessagePrefix() << "executing action" << endl;
#endif
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here21");

				SetTriggerFireLog( m_pCurrentAction );

//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, (*m_pCurrentAction)->GetName() );
				(*m_pCurrentAction)->Execute( &m_instigatorSet );
//	m_pRootCollection->MemLog( 0, HLOG_RPT_0, "Here23");
				m_waitingForAction = true;

				double delay = (*m_pCurrentAction)->GetDelay();
				m_delayFrameCount = (int)(delay / GetTimeStepDuration());

#ifdef DEBUG
				gout << "delay = " << delay << "secs    dur = " << GetTimeStepDuration() << endl;
				gout << "m_delayFrameCount = " << m_delayFrameCount << endl;
#endif
			}
			else
			{
				m_delayFrameCount--;
			}
		}
	}
}