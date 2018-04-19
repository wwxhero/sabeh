/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: followtrigger.cxx,v 1.25 2014/06/24 14:00:20 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 * Date:    March, 2002
 *
 * Description: Provides code for the FollowTrigger HCSM.
 *
 ****************************************************************************/

#include "genhcsm.h"
#include "hcsmcollection.h"
#ifdef _WIN32
#include <strstream>
#elif _PowerMAXOS
#include <strstream>
#define ostringstream ostrstream
#elif __sgi
#include <strstream.h>
#endif

#undef DEBUG_TRIGGER

void 
CFollowTrigger::UserCreation( const CTriggerParseBlock* cpBlock )
{
	PrintCreationMessage();
    m_priorityLevel = cpBlock->GetPriority();
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
		m_leaderInfo = cpBlock->GetLeaderInfo();
		m_followerInfo = cpBlock->GetFollowerInfo();
		if( cpBlock->GetIsTimeOrDistance() == 1)
			m_isFollowTime = true;
		else
			m_isFollowTime = false;
		m_followDist = cpBlock->GetFollowDist();
		m_followTime = cpBlock->GetFollowTime();
		m_toleranceMinus = cpBlock->GetToleranceMinus() / 100;
		m_tolerancePlus = cpBlock->GetTolerancePlus() / 100;
		m_minDuration = cpBlock->GetMinDuration();
		m_framesInRange = 0;
		m_requireSameLane = cpBlock->RequireSameLane();
		m_percentSpeedMatch = cpBlock->GetWithinPercentSpeed();
		
		m_isExpression = cpBlock->GetIsUseExpression();
		m_expression = cpBlock->GetFollowExpression();
		if (m_isExpression)
			m_exprEval.Parse(m_expression.c_str());
		m_exprEval.cved = cved;
#ifdef DEBUG_TRIGGER
		gout << "**the current path:" << endl;
		gout << m_path;

		gout << "Leader: ";
		if( m_leaderInfo .m_type == CTriggerParseBlock::eDRIVER )
			gout << "DRIVER";
		else if( m_leaderInfo .m_type == CTriggerParseBlock::eNAMED )
			gout << m_leaderInfo .m_name;
		else if( m_leaderInfo .m_type == CTriggerParseBlock::eAUTOMATIC )
			gout << "AUTO";
		else
			gout << "UNKNOWN";
		gout << endl;

		gout << "Follower: ";
		if( m_followerInfo.m_type == CTriggerParseBlock::eDRIVER )
			gout << "DRIVER";
		else if( m_followerInfo.m_type == CTriggerParseBlock::eNAMED )
			gout << m_followerInfo.m_name;
		else if( m_followerInfo.m_type == CTriggerParseBlock::eAUTOMATIC )
			gout << "AUTO";
		else
			gout << "UNKNOWN";
		gout << endl;

		gout << "IsFollowerTime = " << m_isFollowTime << endl;
		if( m_isFollowTime )
		{
			gout << "FollowTime = " << m_followTime << " s" << endl;
		}
		else
		{
			gout << "FollowDist = " << m_followDist << " ft" << endl;
		}

		gout << "Tolerance Minus = " << m_toleranceMinus << endl;
		gout << "Tolerance Plus  = " << m_tolerancePlus << endl;
#endif

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
CFollowTrigger::UserActivity( const CTriggerParseBlock* cpBlock )
{
	// If the MakeTraffic button is pressed, then fire the action.
	if( GetButtonFireTrigger() ) 
	{
		ExecuteConcurrentActions();
		return;
	}

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

	// Get the current state from the InitCondition object.
	EInitCond state = m_initCondition.Execute( actualPosition, actualTime );
			
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
CFollowTrigger::UserDeletion( const CTriggerParseBlock* cpBlock )
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
CFollowTrigger::InitCandidates( const CTriggerParseBlock* cpBlock )
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
CFollowTrigger::Evaluate( void )
{
#ifdef DEBUG_TRIGGER
	gout << MessagePrefix() << "=== Evaluate ============" << endl;
#endif
	if(m_isFollowTime){
		if (m_isExpression)
			m_followTime = m_exprEval.Evaluate();
	}
	else {
		if(m_isExpression)
			m_followDist = m_exprEval.Evaluate();
	}
	//
	// Find all objects on the defined roadpad path.
	//
	vector<int> tmpVec;
	m_path.GetObjectsOnPath( tmpVec );
	bool noObjectsOnPath = ( 0 == tmpVec.size() );
	if( noObjectsOnPath ) return false;

	vector<CCandidate> padObjs;
	CCandidate tmpCand;
	vector<int>::const_iterator vecItr;
	for( vecItr = tmpVec.begin(); vecItr != tmpVec.end(); vecItr++ )
	{
		tmpCand.m_cvedId = *vecItr;
		tmpCand.m_hcsmId = cved->GetObjHcsmId( tmpCand.m_cvedId );
		padObjs.push_back( tmpCand );
	}

#ifdef DEBUG_TRIGGER
	{
		gout << "objects on path:";
		vector<CCandidate>::iterator itr;
		for( itr = padObjs.begin(); itr != padObjs.end(); itr++ )
		{
			gout << " " << itr->m_cvedId;
		}
		gout << endl;
	}
#endif

	//
	// Get the leader and the follower object ids.
	//
	// Assuming the the padObjs in sorted in order with the first object
	// on the path being first on the list.
	//
	int followerId;
	int leaderId;
	bool followerFound = false;
	bool leaderFound = false;
    vector<int> autosetLeader;
    vector<int> autofollowerSet;
    if (m_followerInfo.m_type == CTriggerParseBlock::eAUTOMATIC &&  
        m_leaderInfo.m_type == CTriggerParseBlock::eAUTOMATIC ){
        gout<<"Error: Follwer and Leader can not be both Automatic"<<endl;
        return false;
    }
	vector<CCandidate>::iterator i;
	for( i = padObjs.begin(); i != padObjs.end(); i++ )
	{
		//
		// Find the follower first...and then find the leader.
		//
		if( !followerFound )
		{
			//
			// Find the follower.
			//
			if( m_followerInfo.m_type == CTriggerParseBlock::eDRIVER )
			{
				if( i->m_cvedId == 0 )
				{
					followerFound = true;
					followerId = 0;
				}
			}
			else if( m_followerInfo.m_type == CTriggerParseBlock::eNAMED )
			{
				if( m_followerInfo.m_name == cved->GetObjName( i->m_cvedId ) )
				{
					followerFound = true;
					followerId = i->m_cvedId;
				}
			}
		}
		if (!leaderFound)
		{
			//
			// Find the leader.
			//
			if( m_leaderInfo.m_type == CTriggerParseBlock::eDRIVER )
			{
				if( i->m_cvedId == 0 )
				{
					leaderFound = true;
					leaderId = 0;
				}
			}
			else if( m_leaderInfo.m_type == CTriggerParseBlock::eNAMED )
			{
				if( m_leaderInfo.m_name == cved->GetObjName( i->m_cvedId ) )
				{
					leaderFound = true;
					leaderId = i->m_cvedId;
				}
			}		
        }
	}

    //if our leader type is automatic, we need to find the first vehicle that is in front of the follower
	if (m_leaderInfo.m_type == CTriggerParseBlock::eAUTOMATIC && followerFound){
        float minDist = 1e16f;
	    vector<CCandidate>::iterator i;
	    const CDynObj* pFollowerObj = cved->BindObjIdToClass( followerId );
	    CRoadPos followerRoadPos( *cved, cved->GetObjPos( followerId ) );
        double followerDist    = m_path.GetLength( &followerRoadPos );
	    if( !followerRoadPos.IsValid() )  
            return false;
        for( i = padObjs.begin(); i != padObjs.end(); i++ )
	    {
	        if (i->m_cvedId  == followerId)
                continue;            
            const CDynObj* pLeaderObj   = cved->BindObjIdToClass( i->m_cvedId );
            CRoadPos leaderRoadPos( *cved, cved->GetObjPos( i->m_cvedId ) );
            double leaderDist      = m_path.GetLength( &leaderRoadPos );
            float dist = followerDist - leaderDist;
            if (dist > 0 && dist < minDist){
               minDist =  dist;
			   leaderFound = true;
			   leaderId = i->m_cvedId;
            }
        }
    }
    //if our leader type is automatic, we need to find the first vehicle that is in front of the follower
	if (m_followerInfo.m_type == CTriggerParseBlock::eAUTOMATIC && leaderFound){
        float minDist = 1e16f;
	    vector<CCandidate>::iterator i;
        const CDynObj* pLeaderObj   = cved->BindObjIdToClass( leaderId );
        CRoadPos leaderRoadPos( *cved, cved->GetObjPos( leaderId ) );
        double leaderDist      = m_path.GetLength( &leaderRoadPos );
	    if( !leaderRoadPos.IsValid() )  
            return false;
        for( i = padObjs.begin(); i != padObjs.end(); i++ )
	    {
	        if (i->m_cvedId  == leaderId)
                continue;
            const CDynObj* pFollowerObj = cved->BindObjIdToClass( i->m_cvedId );
	        CRoadPos followerRoadPos( *cved, cved->GetObjPos( i->m_cvedId) );
            double followerDist    = m_path.GetLength( &followerRoadPos );
            float dist = followerDist - leaderDist;
            if (dist > 0 && dist < minDist){
               minDist =  dist;
			   followerFound = true;
			   followerId = i->m_cvedId;
            }
        }
    }
	if( !leaderFound || !followerFound )  return false;

	const CDynObj* pFollowerObj = cved->BindObjIdToClass( followerId );
	const CDynObj* pLeaderObj   = cved->BindObjIdToClass( leaderId );

	CRoadPos leaderRoadPos( *cved, cved->GetObjPos( leaderId ) );
	if( !leaderRoadPos.IsValid() )  return false;
	CRoadPos followerRoadPos( *cved, cved->GetObjPos( followerId ) );
	if( !followerRoadPos.IsValid() )  return false;
	if (m_requireSameLane){//If the follwer and the leader have to be in the same lane
		if (leaderRoadPos.GetLane().GetId() != followerRoadPos.GetLane().GetId())
			return false;
	}
	double leaderDist      = m_path.GetLength( &leaderRoadPos );
	double followerDist    = m_path.GetLength( &followerRoadPos );
	double distBetweenObjs = followerDist - leaderDist;

#ifdef DEBUG_TRIGGER
	gout << "leaderId = " << leaderId << "   followerId = " << followerId << endl;
	gout << "leaderDist = " << leaderDist << "   followerDist = " << followerDist << endl;
	gout << "distBetweenObjs    = " << distBetweenObjs << " ft" << endl;
#endif

	//
	// Given either the FollowDist or the FollowTime, figure out how much
	// of a distance should be between the two vehicles.
	//
	double followDist;  // ft
	if( m_isFollowTime )
	{
		followDist = m_followTime * pFollowerObj->GetVelImm() * cMETER_TO_FEET;
	}
	else
	{
		followDist = m_followDist;
	}

	//
	// Figure the out the min and max distance based on the given
	// tolerance values.
	//
	double followDistMin = followDist - ( followDist * m_toleranceMinus );
	double followDistMax = followDist + ( followDist * m_tolerancePlus );

#ifdef DEBUG_TRIGGER
	gout << "followDistMin = " << followDistMin << " ft" << endl;
	gout << "followDistMax = " << followDistMax << " ft" << endl;
#endif

	//
	// Now figure out if the current distance between the 2 vehicles
	// falls within the range.
	//
	double notInDistRange = (
				distBetweenObjs < followDistMin ||
				distBetweenObjs > followDistMax
				);
	if( notInDistRange )
	{
		m_framesInRange = 0;
		return false;
	}
	else
	{
		bool velInRange = 0;
		if (m_percentSpeedMatch>0){
			//const double cVEL_RANGE_PERCENTAGE = 0.10;  // 10%
			double leaderVel = pLeaderObj->GetVelImm();
			double followerVel = pFollowerObj->GetVelImm();
			if (leaderVel > 0){
				double velDiff = fabs( leaderVel - followerVel );
				double percentageDiffFromLeader = velDiff / leaderVel;
				velInRange = percentageDiffFromLeader < m_percentSpeedMatch;
			}else if (leaderVel == 0){
				if (leaderVel == followerVel){
					m_percentSpeedMatch = 1.0;
					velInRange =true;
				}
			}
		}else{
			velInRange = true;
		}
#ifdef DEBUG_TRIGGER
		gout << "percentageDiff = " << percentageDiffFromLeader << endl;
		gout << "velInRange = " << velInRange << endl;
#endif

		if( velInRange )
		{
			m_framesInRange++;
		}
		else
		{
			m_framesInRange = 0;
			return false;
		}
	}

	double timeInRange = m_framesInRange * GetTimeStepDuration();  // seconds

#ifdef DEBUG_TRIGGER
	gout << "timeInRange = " << timeInRange << " s" << endl;
#endif

	if( timeInRange < m_minDuration )  return false;

	//
	// Initialize the instigator set.
	//
	m_instigatorSet.clear();
	if( m_leaderInfo.m_instigator )
	{
		CCandidate temp;
		temp.m_cvedId = leaderId;
		temp.m_hcsmId = pLeaderObj->GetHcsmId();
		m_instigatorSet.insert( temp );

#ifdef DEBUG_TRIGGER
		gout << "--inserting leader " << temp.m_cvedId << endl;
#endif
	}
	else if( m_followerInfo.m_instigator )
	{
		CCandidate temp;
		temp.m_cvedId = followerId;
		temp.m_hcsmId = pFollowerObj->GetHcsmId();
		m_instigatorSet.insert( temp );

#ifdef DEBUG_TRIGGER
		gout << "--inserting follower " << temp.m_cvedId << endl;
#endif
	}

	return true;
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
CFollowTrigger::ExecuteConcurrentActions()
{
	//
	// Exeucte actions concurrently.
	//
	CAction::TActionIterator aI;
	for( aI = m_pActionVector.begin(); aI != m_pActionVector.end(); aI++ )
	{
		(*aI)->Execute(&m_instigatorSet);
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
CFollowTrigger::InitializeSequentialActions()
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
CFollowTrigger::ExecuteSequentialActions()
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
				(*m_pCurrentAction)->Execute(&m_instigatorSet);
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
