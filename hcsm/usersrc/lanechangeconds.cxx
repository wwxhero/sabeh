
/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: lanechangeconds.cxx,v 1.70 2016/07/15 14:50:18 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad, Huidi Tang
 *
 * Date:    May, 2000
 *
 * Description:  Contains the implementation for the CLaneChangeConds class.
 *
 ****************************************************************************/

#include "lanechangeconds.h"
#include "exceptionsuicide.h"
#include "util.h"
#include "hcsm.h"
#include "hcsmcollection.h"
#include "ado_lc_data.h"
#include "support.h"


#undef DEBUG_CHECK_PATH_GUIDANCE            //17  // needs CVED id
#undef DEBUG_CHECK_LOSING_CORRIDOR          //17  // needs CVED id
#undef DEBUG_CHECK_HIGHWAY_MERGE
#undef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE //17  // needs CVED id
#undef DEBUG_CHECK_EXTERNAL                 //17  // needs CVED id
#undef DEBUG_CHECK_NON_PASSING_LANE         //17  // needs CVED id
#undef DEBUG_CHECK_SLOW_VEHICLE             //17  // needs CVED id
#undef DEBUG_CHECK_VERY_SLOW_VEHICLE        //3   // needs CVED id
#undef DEBUG_GAP_ACCEPT                     //7   //needs CVED id


//////////////////////////////////////////////////////////////////////
// Utility functions.
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//
// Description: Computes an urgency between 0 and 1.
//
// Remarks: The urgency gets higher as the distance to the intersection
//   gets smaller.
//
// Arguments:
//   cDistToIntrsctn - The distance to the next intersection in meters.
//   cUrgency - The default urgency.
//
// Returns:  The urgency.
//
//////////////////////////////////////////////////////////////////////////////
static double
ComputeUrgency( const double cDistToIntrsctn, const double cUrgency )
{

	double urgency = cUrgency;

	if( cDistToIntrsctn < 0.0 ) 
	{
		urgency = 1.0;
	}
	else if( cDistToIntrsctn < 50.0 )
	{
		urgency += ( 0.5 - ( ( MIN( cDistToIntrsctn, 50.0 ) / 50.0 ) * 0.5 ) );
	}

	return urgency;

}  // end of ComputeUrgency


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CLaneChangeConds::CLaneChangeConds() :
	m_pCved( NULL ),
	m_pRootCollection( NULL ),
	m_slowLcWaitStartFrame( -1 )
{

	//
	// Initialize the conditions map.  Insert an element for each type
	// of lane change condition.
	//
	CLaneChangeCond cond;

  	cond.SetType( eLC_EXTERNAL_COMMAND );
	InsertCondition( eLC_EXTERNAL_COMMAND, cond );

	cond.SetType( eLC_PATH_GUIDANCE );
	InsertCondition( eLC_PATH_GUIDANCE, cond );

	cond.SetType( eLC_LOSING_CORRIDOR );
	InsertCondition( eLC_LOSING_CORRIDOR, cond );

	cond.SetType( eLC_NONPASSING_LANE );
	InsertCondition( eLC_NONPASSING_LANE, cond );

	cond.SetType( eLC_SLOW_VEHICLE );
	InsertCondition( eLC_SLOW_VEHICLE, cond );

	cond.SetType( eLC_VERY_SLOW_VEHICLE );
	InsertCondition( eLC_VERY_SLOW_VEHICLE, cond );	

	cond.SetType( eLC_HIGHWAY_MERGE );
	InsertCondition( eLC_HIGHWAY_MERGE, cond );

	cond.SetType( eLC_AVOID_MERGING_VEHICLE );
	InsertCondition( eLC_AVOID_MERGING_VEHICLE, cond );


	//
	// NOTE:  It is required for the user to call SetCved and 
	//        SetHcsmCollection.
	//

}

CLaneChangeConds::CLaneChangeConds( const CLaneChangeConds& cObjToCopy )
{

	// call the assignment operator
	*this = cObjToCopy;

}

CLaneChangeConds& 
CLaneChangeConds::operator=( const CLaneChangeConds& cObjToCopy )
{
	// check to make sure that object passed in is not me
	if ( this != &cObjToCopy ) 
	{
		m_conditions = cObjToCopy.m_conditions;
		m_pCved = cObjToCopy.m_pCved;
		m_pRootCollection = cObjToCopy.m_pRootCollection;
		m_slowLcWaitStartFrame = cObjToCopy.m_slowLcWaitStartFrame;	
	}

	return *this;
}

CLaneChangeConds::~CLaneChangeConds()
{


}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets the pointer to the Cved instance.
//
// Remarks: The Cved pointer has to be set before this class can be
//   used.
//
// Arguments:
//   pCved - The pointer to a Cved instance.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::SetCved( CCved* pCved )
{
	m_pCved = pCved;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets the pointer to the HCSM root collection instance.
//
// Remarks: The root collection pointer has to be set before this class 
//   can be used.
//
// Arguments:
//   pRootCollection - The pointer to a HCSM root collection.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::SetHcsmCollection( CHcsmCollection* pRootCollection )
{
	m_pRootCollection = pRootCollection;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Inserts a condition.
//
// Remarks: Checks to see if the given type already has a condition
//   associated with it.  If so, it prints an error message and exits.
//
// Arguments:
//   cType - Type of the condition to insert.
//   cCond - The condition.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::InsertCondition( 
	const ELcCondition cType,
	const CLaneChangeCond& cCond
	)
{

	if( m_conditions.find( cType ) == m_conditions.end() ) 
	{
		// No previous condition of this type...insert it.
		m_conditions[cType] = cCond;
	}
	else 
	{
		cerr << "CLaneChangeConds::InsertCondition" << endl;
		cerr << "condition of type '" << cType << "' already exists!";
		cerr << endl;
	}

}  // end of InsertCondition


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets a condition.
//
// Remarks: Prints an error message if the given type doesn't have
//   a condition associated with it.
//
// Arguments:
//   cType - Type of the condition.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
CLaneChangeCond&
CLaneChangeConds::GetCondition( const ELcCondition cType )
{

	if( m_conditions.find( cType ) == m_conditions.end() ) 
	{
		// No condition associated with type.
		cerr << "CLaneChangeConds::GetCondition" << endl;
		cerr << "condition of type '" << cType << "' doesn't exist!";
		cerr << endl;

		CExceptionSuicide exception;
		throw exception;
	}

	return m_conditions[cType];
	
}  // end of GetCondition


//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets a condition.
//
// Remarks: If the given type already has a condition associated with 
//   it then it gets overwritten.
//
// Arguments:
//   cType - Type of the condition to insert.
//   cDond - The condition.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::SetCondition( 
	const ELcCondition cType,
	const CLaneChangeCond& cCond
	)
{

	m_conditions[cType] = cCond;

}  // end of SetCondition


//////////////////////////////////////////////////////////////////////////////
//
// Description: Finds out if any of the conditions are set to active.
//
// Remarks: 
//
// Arguments:
//
// Returns: A boolean indicating if any conditions are set to active.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLaneChangeConds::AnyConditionsActive()
{

	TLcConditions::const_iterator i;
	for( i = m_conditions.begin(); i != m_conditions.end(); i++ ) 
	{
		if ( i->second.GetActive() )  return true;
	}

	return false;

}  // end of AnyConditionsActive


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets the current lane change condition.
//
// Remarks: 
//
// Arguments:
//   cPastCond - The previous lane change condition.
//
// Returns: An element from an enumeration of possible lane changes.
//
//////////////////////////////////////////////////////////////////////////////
ELcCondition
CLaneChangeConds::GetCurrentCondition( const ELcCondition cPastCond )
{

	ELcCondition currCond = eLC_NONE;

	if( m_conditions[eLC_EXTERNAL_COMMAND].GetActive() ) 
	{
		currCond = eLC_EXTERNAL_COMMAND;
	}
	else if( m_conditions[eLC_PATH_GUIDANCE].GetActive() ) 
	{
		currCond = eLC_PATH_GUIDANCE;
	}
	else if( m_conditions[eLC_LOSING_CORRIDOR].GetActive() ) 
	{
		currCond = eLC_LOSING_CORRIDOR;
	}
	else if( m_conditions[eLC_NONPASSING_LANE].GetActive() ) 
	{
		currCond = eLC_NONPASSING_LANE;
	}
	else if( m_conditions[eLC_SLOW_VEHICLE].GetActive() ) 
	{
		currCond = eLC_SLOW_VEHICLE;
	}
	else if( m_conditions[eLC_VERY_SLOW_VEHICLE].GetActive() ) 
	{
		currCond = eLC_VERY_SLOW_VEHICLE;
	}
	else if( m_conditions[eLC_HIGHWAY_MERGE].GetActive() ) 
	{
		currCond = eLC_HIGHWAY_MERGE;
	}
	else if( m_conditions[eLC_AVOID_MERGING_VEHICLE].GetActive() ) 
	{
		currCond = eLC_AVOID_MERGING_VEHICLE;
	}

	//
	// Give utmost priority to path guidance...if path guidance
	// has already executed then let no-one else do anything except
	// for highway merges and external commands.
	//
	if( 
		cPastCond == eLC_PATH_GUIDANCE &&
		currCond != eLC_PATH_GUIDANCE &&
		//currCond != eLC_HIGHWAY_MERGE && 
		currCond != eLC_EXTERNAL_COMMAND
		)
	{
		currCond = eLC_NONE;
	}

	return currCond;

}  // end of GetCurrentcondition


//////////////////////////////////////////////////////////////////////////////
//
// Description: Resets the current lane change condition.
//
// Remarks: Resets the first condition that it finds active.
//
// Arguments:
//
// Returns: An element from an enumeration of possible lane changes.
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::ResetCurrentCondition( )
{

	TLcConditions::iterator i;
	for( i = m_conditions.begin(); i != m_conditions.end(); i++ ) 
	{
		if( i->second.GetActive() )
		{
			i->second.SetActive( false );
			return;
		}
	}

}  // end of GetCurrentcondition


//////////////////////////////////////////////////////////////////////////////
//
// Description: Determines if the lane change required from the starting
//   lane to the ending lane is to the left or not.
//
// Remarks:    
//
// Arguments:
//   cStartLane - The starting lane.
//   cEndLane   - The ending lane.
//
// Returns: A boolean indicating if it's to the left.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLaneChangeConds::IsLeftLaneChange( 
	const CLane& cStartLane, 
	const CLane& cEndLane
	)
{

	if( !cStartLane.IsLeftMostAlongDir() && cStartLane.GetLeft() == cEndLane ) 
	{	
		// making a left turn
		return true;
	}
	else if( !cStartLane.IsRightMost() && cStartLane.GetRight() == cEndLane ) 
	{
		// making a right turn
		return false;
	}
	else 
	{
		cerr << "CLaneChangeConds:IsLeftLaneChange: ";
		cerr << "unable to determine turn signal direction...";
		cerr << "returning right turn signal" << endl;
		cerr << "  startLane = " << cStartLane << endl;
		cerr << "  endLane   = " << cEndLane << endl;
		
		return false; 
	}

}  // end of IsLeftLaneChange

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Checks if a lane change is necessary due to an 
//   external command.
//
// Remarks:    
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions info. pointer.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLaneChangeConds::CheckExternalCommand(
			const CAdoInfo& cInfo,
			CLaneChangeCond& cond
			)
{
	//
    // Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_EXTERNAL
	int objId = cInfo.m_pObj->GetId();
	bool debugThisObj = objId == DEBUG_CHECK_EXTERNAL;
	if( debugThisObj ) 
	{
		gout << "[" << cInfo.m_pObj->GetId() << "] ";
		gout << "==== LANECHANGE CheckExternalCommand ==========" << endl;
		gout << "m_forcedLaneOffset = " << cInfo.m_forcedLaneOffset << endl;
		gout << "cInfo.m_rightLaneChangeButton = ";
		gout << cInfo.m_rightLaneChangeButton << endl;
		gout << "cInfo.m_leftLaneChangeButton = ";
		gout << cInfo.m_leftLaneChangeButton << endl;
	}
#endif

	//
	// Exit if no external command.
	//
	bool haveExternalCommand = (
				cInfo.m_leftLaneChangeButton ||
				cInfo.m_rightLaneChangeButton ||
				(
					cInfo.m_haveForcedLaneOffset &&
					fabs( cInfo.m_forcedLaneOffset - cInfo.m_roadPos.GetOffset() ) > 0.5
				)
				);
	if( !haveExternalCommand )
	{
#ifdef DEBUG_CHECK_EXTERNAL
		if( debugThisObj ) 
		{
			gout << "no external command...exit" << endl;
		}
#endif

		return;
	}

	if( cInfo.m_leftLaneChangeButton && cInfo.m_rightLaneChangeButton ) 
	{
#ifdef DEBUG_CHECK_EXTERNAL
		if( debugThisObj ) 
		{
			gout << "conflicting external commands...exit" << endl;
		}
#endif

		return;
	}
	
	if( cInfo.m_haveForcedLaneOffset )
	{
		//
		// Set lane change condition paramteres.
		//
		bool leftLaneChange = (
					cInfo.m_roadPos.GetOffset() > cInfo.m_forcedLaneOffset
					);
		cond.SetActive( true );
		cond.SetUrgency( cInfo.m_forcedLaneOffsetUrgency );
		cond.SetIsForcedLaneOffset( true);
		cond.SetForcedLaneOffset( cInfo.m_forcedLaneOffset );
		cond.SetSkipSignal( true );
		cond.SetLeftLaneChange( leftLaneChange );

#ifdef DEBUG_CHECK_EXTERNAL
		if( debugThisObj ) 
		{
			gout << "** initiate forcedLaneOffset lanechange" << endl;
			gout << "   leftLaneChange = " << leftLaneChange << endl;
			gout << "   targOffset = " << cInfo.m_forcedLaneOffset << endl;
		}
#endif
	}
	else
	{
		//
		// If it's not a left lane change then it must be a right lane
		// change given the error checking performed above.
		//
		bool isLeftLaneChange = cInfo.m_leftLaneChangeButton;

		//
		// Exit if inside intersection.
		//
		if( !cInfo.m_roadPos.IsRoad() ) 
		{
#ifdef DEBUG_CHECK_EXTERNAL
			if( debugThisObj ) 
			{
				gout << "inside intersection...exit" << endl;
			}
#endif
//           tgus us cide ti deal with crdrs, this is not ready
//           if (cInfo.m_roadPos.IsValid()){
//                auto crdr =  cInfo.m_roadPos.GetCorridor();
//                auto lane = crdr.GetDstntnLn();
//                bool haveLane = false;
//                CLane targLane;
//                if( isLeftLaneChange && !lane.IsLeftMost()) {
//                   targLane = lane.GetLeft();
//                   if (targLane.IsValid()){
//                       haveLane = true;
//                   }
//                }
//                else if( !isLeftLaneChange && !lane.IsRightMost()) {
//                    targLane = lane.GetRight();
//                    if (targLane.IsValid()){
//                       haveLane = true;
//                    }
//                }
//                if (haveLane){
//		            cond.SetActive( true );
//		            cond.SetUrgency( 0.5 );
//		            cond.SetTargLane( targLane );
//		            cond.SetLeftLaneChange( isLeftLaneChange );
//		            cond.SetIsForcedLaneOffset( false );
//		            cond.SetSkipSignal( false );
//                }
//            }

			return;
		}

		//
		// Compute target lane and perform more error checking.
		//
		CLane targLane;
		if( isLeftLaneChange ) 
		{
			//
			// Left lane change.
			//
			CLane curLane = cInfo.m_roadPos.GetLane();
			bool haveLane = !curLane.IsLeftMostAlongDir();
			if( haveLane ) 
			{
				targLane = curLane.GetLeft();
			}
			else 
			{	
#ifdef DEBUG_CHECK_EXTERNAL
				if( debugThisObj ) 
				{
					gout << "no lane exists to the left...exit" << endl;
				}
#endif

				return;
			}
		}
		else 
		{
			//
			// Right lane change.
			//
			CLane curLane = cInfo.m_roadPos.GetLane();
			bool haveLane = !( curLane.IsRightMost() );
			if ( haveLane ) {

				targLane = curLane.GetRight();

			}
			else 
			{	
#ifdef DEBUG_CHECK_EXTERNAL
				if( debugThisObj ) 
				{
					gout << "no lane exists to the right...exit" << endl;
				}
#endif

				return;
			}
		}

		//
		// Compute distance to intersection.
		//
		double distToIntrsctn = cInfo.m_pPath->GetDistToNextIntrsctn( 
			cInfo.m_roadPos 
			);
		distToIntrsctn = distToIntrsctn * cFEET_TO_METER;

		//
		// Compute urgency.
		//
		double urgency;
		bool useUrgencyFromDial = (
					cInfo.m_laneChangeUrgency >= 0.0 && 
					cInfo.m_laneChangeUrgency <= 1.0
					);
		if( useUrgencyFromDial ) 
		{
			urgency = cInfo.m_laneChangeUrgency;
		}
		else 
		{
			urgency = ComputeUrgency( distToIntrsctn, cInfo.m_lcInfo.urgency );
		}

#ifdef DEBUG_CHECK_EXTERNAL
		if( debugThisObj ) 
		{
			gout << "** initiate lanechange" << endl;
			gout << "   urgency = " << urgency << endl;
			gout << "   targLane = " << targLane << endl;
			gout << "   leftLaneChange = " << isLeftLaneChange << endl;
		}
#endif

		//
		// Set lane change condition paramteres.
		//
		cond.SetScenarioInducedAbort(false);
        cond.SetActive( true );
		cond.SetUrgency( urgency );
		cond.SetTargLane( targLane );
		cond.SetLeftLaneChange( isLeftLaneChange );
		cond.SetIsForcedLaneOffset( false );
		cond.SetSkipSignal( false );
	}

}  // end of CheckExternalCommand


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Checks if a lane change is necessary due to path
//   guidance.
//
// Remarks:    
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions info. pointer.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::CheckPathGuidance( 
			const CAdoInfo& cInfo, 
			CLaneChangeCond& cond
			)
{
	//
    // Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_PATH_GUIDANCE
	int objId = cInfo.m_pObj->GetId();
	bool debugThisObj = objId == DEBUG_CHECK_PATH_GUIDANCE;
	if( debugThisObj ) 
	{
		gout << "[" << cInfo.m_pObj->GetId() << "] ";
		gout << "==== LANECHANGE CheckPathGuidance ==========" << endl;
	}
#endif

	//
	// Exit if no path.
	//
	if( cInfo.m_pPath->Size() <= 0 )  return;


	//
	// CONDITION:
	// Check for path through non-dummy intersection.
	//
	CIntrsctn intrsctn = cInfo.m_pPath->GetNextIntrsctn( cInfo.m_roadPos );
	if( !intrsctn.IsValid() )
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "no upcoming intersection...exit" << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// Does the current lane fail to connect to the corridor that I
	// need to traverse the intersection?
	//
	CRoadPos junkRoadPos;
	CLane targLane;
	CPath::ETravelCode code;
#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << *(cInfo.m_pPath);
		gout << "input road pos = " << cInfo.m_roadPos << endl;
	}
#endif
	
	//
	// just checks to see if there exists a path between the
	// the curr rd and targ rd, retruns the code	
	//
	code = cInfo.m_pPath->Travel( 
						0.0, 
						cInfo.m_roadPos, 
						junkRoadPos, 
						targLane 
						);

	if( code != CPath::eCV_TRAVEL_LANE_CHANGE ) 
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "no lane change needed...exit" << endl;
		}
#endif

		return;
	}
#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << "lane change needed: targLane = " << targLane << endl;
	}
#endif

	//
	// Figure out if the target lane is adjacent to the current
	// lane.
	//
	CLane currLane = cInfo.m_roadPos.GetLane();
	bool adjacent = ( ( !currLane.IsLeftMostAlongDir() && 
						currLane.GetLeft() == targLane 
						) || 
					  ( !currLane.IsRightMost() &&
						currLane.GetRight() == targLane
						)
					  );
	int totalLaneChanges = 1;

	if( !adjacent )
	{
		//
		// Lanes are not adjancent.  Now figure out whether 
		// the target lane is to the left or right and then pick
		// the adjancent lane in that direction.
		//
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "target lane not adjancent to current lane" << endl;
		}
#endif
		
		if( currLane.IsValid() && targLane.IsValid() )
		{
			totalLaneChanges = abs( targLane.GetRelativeId() - currLane.GetRelativeId() );
#ifdef DEBUG_CHECK_PATH_GUIDANCE
			if( debugThisObj )
			{
				gout << "total # of lane changes = " << totalLaneChanges << endl;
			}
#endif
		}
		//
		// Is the target lane to the left of the current lane?
		//
		bool found = false;
		CLane tempLane = currLane;
		while( !found && !tempLane.IsLeftMostAlongDir() ) 
		{
			tempLane = tempLane.GetLeft();
			if( tempLane == targLane )
			{
				found = true;
				targLane = currLane.GetLeft();
			}
		}

		//
		// Or, is the target lane to the right of the current lane.
		//
		tempLane = currLane;
		while( !found && !tempLane.IsRightMost() )
		{
			tempLane = tempLane.GetRight();
			if( tempLane == targLane )
			{
				found = true;
				targLane = currLane.GetRight();
			}
		}
	}


	//
	// Calculate the distance and time to the next non-dummy intersection.
	//
	double distToIntrsctn = cInfo.m_pPath->GetDistToNextIntrsctn( 
													cInfo.m_roadPos 
													);	// ft

#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << "distToIntrsctn = " << distToIntrsctn << " ft" << endl;
	}
#endif

	double myVel = cInfo.m_currVel;
	double timeToIntrsctn;
	if( myVel > cNEAR_ZERO )
	{
		timeToIntrsctn = ( distToIntrsctn * cFEET_TO_METER ) / myVel;
	}
	else 
	{
		timeToIntrsctn = 1000.0;  // a large value
	}

#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << "timeToIntrsctn = " << timeToIntrsctn << " secs" << endl;
	}
#endif

	//
	// CONDITION:
	// Need to be at most 600 * totalLanechanges ft away from the
	// intersection.
	//
 	const double cFUDGE_DIST_TO_INTERSECTION = 600.0;  // ft
	double maxDistToIntrsctn = cFUDGE_DIST_TO_INTERSECTION * totalLaneChanges; 
	bool tooFarAwayFromIntersection = distToIntrsctn > maxDistToIntrsctn;
	if( tooFarAwayFromIntersection )
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << distToIntrsctn << " ft away from intersection, ";
			gout << "need to be at most " << maxDistToIntrsctn;
			gout << " ft away...exit" << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// Need to be within 30 seconds of the next intersection.
	//
	const double cTIME_TO_INTERSECTION = 30.0;  // seconds
	bool tooMuchTimeToIntrsctn = timeToIntrsctn > cTIME_TO_INTERSECTION; 
	if( tooMuchTimeToIntrsctn ) 
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << timeToIntrsctn << " secs away from intersection, ";
			gout << "need to be at most " << cTIME_TO_INTERSECTION;
			gout << " secs away...exit" << endl;
		}
#endif

		return;
	}


	//
	// CONDITION:
	// Target lane needs to have proper width to accomodate vehicle. 
	// Can give the okay to lane change about 3 seconds before the 
	// actual start of the lane change.
	//
	CRoadPos tempRoadPos = cInfo.m_roadPos;
	double distToTravel = 5.0 * cInfo.m_currVel * cMETER_TO_FEET;  // ft
	tempRoadPos.Travel( distToTravel );
	bool notEnoughRoom = (
					tempRoadPos.IsValid() && 
					targLane.GetWidth( tempRoadPos.GetDistance() ) < 0.5
					);
	if( notEnoughRoom )
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "currLane width =" << cInfo.m_currLane.GetWidth();
			gout << "ft" << endl;
			gout << "targLane width = " << targLane.GetWidth() << "ft" << endl;
			gout << "not enough room...exit" << endl;
		}
#endif
		
		return;
	}
	
	//
	// Compute urgency.
	//
	double urgency = ComputeUrgency( 
							distToIntrsctn * cFEET_TO_METER,
							cInfo.m_lcInfo.urgency
							);

	//
	// CONDITION:
	// The current distance to the non-dummy intersection is within 5
	// times the estimated forward distance needed to complete the lane
	// change.
	//
	double fwdDistNeeded;
	double fwdTimeNeeded;
	int solId = cInfo.m_pObj->GetSolId();

#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << "myVel = " << myVel * cMS_TO_MPH << "mph   ";
		gout << "urgency = " << urgency;
		gout << "   solId = " << solId << endl;
	}
#endif
	
	bool hasForwardDist = LcLookupDistTime( 
										myVel * cMS_TO_MPH, 
										urgency,
										solId,
										fwdDistNeeded,
										fwdTimeNeeded
										);

	if( !hasForwardDist )
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "no fwdDistNeeded obtained...return " << endl;
		}
#endif

		return;
	}
	
#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << "fwdDistNeeded = " << fwdDistNeeded << "ft" << endl;
		gout << "fwdTimeNeeded = " << fwdTimeNeeded << "s" << endl;
	}
#endif

	bool tooFarFromIntrsctn = distToIntrsctn > ( 5 * fwdDistNeeded );
	if( tooFarFromIntrsctn )
	{

#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "too far away from intersection...return" << endl;
		}
#endif

		return;
	}
	
	//
	// If distance reqd. to complete lanechange is greater than
	// the distance to an intersection, query the lc data files 
	// for urgency and check if by increasing urgency, lanechange
	// can be completed before going over the intersection.
	//
	const double cFUDGE_DIST = 20.0;  // ft
	//frwd dist + signalDist + fudge
	double distNeededForLaneChange =
		fwdDistNeeded + (2.0 * myVel * cMETER_TO_FEET) + cFUDGE_DIST; //ft
	
#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << "distNeededForLc = " << distNeededForLaneChange << "ft" << endl;
	}
#endif

	// Check Distance reqd. for LC infront of a intersection
	bool notEnoughDistForLc = distNeededForLaneChange > distToIntrsctn;

	if( notEnoughDistForLc )
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "not enough dist for lc...try to increase urgency" << endl;
		}
#endif
		const double cMAX_URGENCY = 1.0; 
		const double cURGENCY_INCREMENT = 0.1;
		bool foundUrgency = false;
		urgency += cURGENCY_INCREMENT;
		while( !foundUrgency && urgency <= cMAX_URGENCY )
		{
			bool foundDist = LcLookupDistTime( 
										myVel * cMS_TO_MPH, 
										urgency,
										solId,
										fwdDistNeeded,
										fwdTimeNeeded
										);
			if( foundDist )
			{
				distNeededForLaneChange = (
							fwdDistNeeded + 
							(2.0 * myVel * cMETER_TO_FEET) + 
							cFUDGE_DIST 
							);
			
				bool enoughDistForCompleteLC = ( 
							distToIntrsctn > distNeededForLaneChange 
							);

				if( enoughDistForCompleteLC  )
				{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
					if( debugThisObj )
					{
						gout << "new urgency = " << urgency;
					}
#endif
					foundUrgency = true;
					break;
				}
			}

			if( !foundUrgency )  urgency += cURGENCY_INCREMENT;
		}

		//
		// Set the urgency to maximum if unable to find suitable urgency.
		//
		if( !foundUrgency )
		{
			urgency = cMAX_URGENCY;

#ifdef DEBUG_CHECK_PATH_GUIDANCE
			if( debugThisObj )
			{
				gout << "no good solution...setting urgency to maximum = ";
				gout << urgency << endl;
			}
#endif
		}
	}
		
	//
	// CONDITION:
	// Check for proper gap acceptance.
	//
	bool hasProperGap;
	hasProperGap = GapAccept(
				cInfo,
				cInfo.m_backTimeThreshold,
				cInfo.m_fwdTimeThreshold,
				cInfo.m_backDistThreshold,
				cInfo.m_fwdDistThreshold,
				targLane
				);
	if ( !hasProperGap ) 
	{
#ifdef DEBUG_CHECK_PATH_GUIDANCE
		if( debugThisObj )
		{
			gout << "doesn't have proper gap....exit" << endl;
		}
#endif
		
		return;
	}

#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << " hasProperGap = " << hasProperGap << endl;
	}
#endif

	//
	// Compute direction of lane change.
	//
	currLane = cInfo.m_roadPos.GetLane();
	bool isLeftLaneChange = IsLeftLaneChange( currLane, targLane );

	//
	// Compute the lateral distance needed for this lane change.
	//
	// Wait to see if this should be computed here.
	//
	
#ifdef DEBUG_CHECK_PATH_GUIDANCE
	if( debugThisObj )
	{
		gout << "** initiate lanechange" << endl;
		gout << "   urgency = " << urgency << endl;
		gout << "   targLane = " << targLane << endl;
		gout << "   leftLaneChange = " << isLeftLaneChange << endl;
	}
#endif

	//
	// Set lane change condition paramteres.
	//
	cond.SetActive( true );
	cond.SetUrgency( urgency );
	cond.SetTargLane( targLane );
	cond.SetLeftLaneChange( isLeftLaneChange );
	cond.SetIsForcedLaneOffset( false );
	cond.SetSkipSignal( false );

}  // end of CheckPathGuidance


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Finds a corridor that has a destination lane that is to 
//   the left of the destination lane of the given corridor.
//
// Remarks: The function checks to make sure that the given corridor
//   and the resulting corridor have the same source road.
//
// Arguments:
//   intrsctn - Intersection in which to look for the corridor.
//   crdr     - The corridor from which to start looking for the left
//              corridor.
//   leftCrdr - (output) The corridor which has as its destination lane
//              the lane which is to the left of the given lane.
//
// Returns:  A boolean indicating if a corridor was found.
//
//////////////////////////////////////////////////////////////////////////////
static bool
FindLeftCorridor(
			const CIntrsctn& cIntrsctn,
			const CCrdr& cCrdr,
			CCrdr& targetCrdr
			)
{

	//
	// Find the lane to the left of the given lane.
	//
	CRoad startSrcRd    = cCrdr.GetSrcRd();
	CLane startDstntnLn = cCrdr.GetDstntnLn();

	CLane leftLane;
	bool noMoreLeftLanes = startDstntnLn.IsLeftMostAlongDir();
	if( noMoreLeftLanes )
	{
		// no more lanes available to the left....return false
		return false;
	}
	else
	{
		leftLane = startDstntnLn.GetLeft();
	}

	//
	// Get all the corridors inside the given intersection.
	//
	TCrdrVec allCrdrs;
	cIntrsctn.GetAllCrdrs( allCrdrs );

	//
	// Iterate the vector to find the corridor that 
	// has the same destination lane as the current
	// corridor. Then this other corridor is the
	// target corridor.
	//
	vector<CCrdr>::iterator i;
	bool foundCrdr = false;
	for ( i = allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		CCrdr aCrdr = *i;
		foundCrdr = (
			aCrdr.GetSrcRd() == startSrcRd &&
			aCrdr.GetDstntnLn() == leftLane 
			);

		if( foundCrdr )
		{

#ifdef DEBUG_CHECK_HIGHWAY_MERGE	
			gout << "  found the target corridor" << endl;
#endif

			targetCrdr = aCrdr;
			break;
		
		}
		
	}

	return foundCrdr;
}  // end of FindLeftCorridor


//////////////////////////////////////////////////////////////////////////////
//
// Description: Checks if the given corridor is merging into another 
//   corridor.
//
// Remarks:
//
// Arguments:
//   intrsctn - The intersection in which to look for merging corridors.
//   currCrdrId - The relative id of the corridor.
//   crdrsHaveSameSrcRoad - When true, the merging corridors must have 
//       the same source road for this function to return true.
// Returns: A boolean indicating if the given corridor merges with an
//   another corridor in the given intersection.
//
//////////////////////////////////////////////////////////////////////////////
static bool
OnMergingCorridor( 
			const CIntrsctn& cIntrsctn, 
			int currCrdrId, 
			bool crdrsHaveSameSrcRoad = false  
			)
{
	//
	// Hold all the corridors in the intersection and check if there 
	// are corridors that have the same destination lane. If so, have 
	// found at least one corridor is going to be merged.
	//
	vector<CCrdr> allCrdrs;
	cIntrsctn.GetAllCrdrs( allCrdrs );
	int i;
	for( i = 0; i < allCrdrs.size(); i++ )
	{
		int j;
		for( j = i + 1; j < allCrdrs.size(); j++ )
		{
			if( allCrdrs[i].GetDstntnLn() == allCrdrs[j].GetDstntnLn() )
			{
				//
				// Found 2 corridors that are merging into one.
				//
				bool onMergeCrdr = (
							currCrdrId == i || currCrdrId == j
							);
				if( onMergeCrdr)
				{
					if( crdrsHaveSameSrcRoad )
					{
						//
						// Check to make sure that these 2 corridors start from
						// the same road.
						//
						bool onLosingCrdr = (
									allCrdrs[i].GetSrcRd() == allCrdrs[j].GetSrcRd()
									);
						if( onLosingCrdr )  return true;
					}
					else
					{
						return true;
					}
				}
			}
		}
	}
	
	return false;
}  // end of OnMergingCorridor



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Checks if a corridor in an intersection is going be 
//	merged with another corridor.
//
// Remarks:    
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions info. pointer.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::CheckLosingCorridor( 
	const CAdoInfo& cInfo, 
	CLaneChangeCond& cond
	)
{
	//
	// Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_LOSING_CORRIDOR
	int objId = cInfo.m_pObj->GetId();
	bool debugThisObj = objId == DEBUG_CHECK_LOSING_CORRIDOR;
	if( debugThisObj ) 
	{
		gout << "[" << objId << "] ";
		gout << "==== LANECHANGE CheckLosingCorridor ==========" << endl;
	}
#endif

	//
	// Exit if no path.
	//
	if( cInfo.m_pPath->Size() <= 0 )  return;

	if( !cInfo.m_roadPos.IsRoad() )
	{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
		if( debugThisObj ) 
		{
			gout << "  temporarily disabled on intersetions...until";
			gout << "back object list works on intersections...exit" << endl;
		}
#endif
		return;
	}


	//
	// CONDITION:
	// On the ADO's path, there exists an intersection which merges
	// two or more corridors into one.
	//

	//
	// If on road, get the next intersection.
	//
	CIntrsctn intrsctn;
	int crdrId;
	bool onRoad = cInfo.m_roadPos.IsRoad();
	if( onRoad )
	{
		intrsctn = cInfo.m_pPath->GetNextIntrsctn( cInfo.m_roadPos );
		if( !intrsctn.IsValid() )
		{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
			if( debugThisObj ) 
			{
				gout << "  no upcoming intersection...exit" << endl;
			}
#endif
			return;
		}
		CCrdr nextCrdr = cInfo.m_pPath->GetNextCrdr(cInfo.m_roadPos);
		if( !nextCrdr.IsValid() )
		{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
			if( debugThisObj ) 
			{
				gout << "  unable to obtain next crdr...exit" << endl;
			}
#endif
			return;
		}
		crdrId = nextCrdr.GetRelativeId();
	}
	else
	{
		//
		// If on intersection, get the intersection.
		//
		intrsctn = cInfo.m_roadPos.GetIntrsctn();
		crdrId = cInfo.m_roadPos.GetCorridor().GetRelativeId();
	}

	if( !OnMergingCorridor( intrsctn, crdrId, true ) )
	{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
		if( debugThisObj ) 
		{
			gout << "  not on a merging corridor...exit" << endl;
		}
#endif

		return;
	}
	
	//
	// Calculate the distance and time to the next intersection.
	//
	double distToIntrsctn = 0.0;
	double timeToIntrsctn = 0.0;
	double myVel;
	if( onRoad )
	{
		distToIntrsctn = cInfo.m_pPath->GetDistToNextIntrsctn( 
												cInfo.m_roadPos
												);  // ft

		myVel = cInfo.m_currVel;
		if( myVel > cNEAR_ZERO ) 
		{
			timeToIntrsctn = ( distToIntrsctn * cFEET_TO_METER ) / myVel;
		}
		else 
		{
			timeToIntrsctn = 1000.0;  // a large value
		}
	}

#ifdef DEBUG_CHECK_LOSING_CORRIDOR
	if( debugThisObj ) 
	{
		gout << "  distToIntrsctn = " << distToIntrsctn << " ft " << endl;
		gout << "  timeToIntrsctn = " << timeToIntrsctn << " secs" << endl;
	}
#endif

	//
	// CONDITION:
	// Need to be at most 1000 feet away from the intersection.
	//
	const double cMAX_DIST_TO_INTERSECTION = 1000.0;  // feet
	if( distToIntrsctn > cMAX_DIST_TO_INTERSECTION )
	{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
		if( debugThisObj )
		{
			gout << distToIntrsctn << " ft away from intersection, ";
			gout << "need to be at most " << cMAX_DIST_TO_INTERSECTION;
			gout << " ft away...exit" << endl;
		}
#endif

		return;
	}


	//
	// CONDITION:
	// Need to be within 60 seconds of the next intersection.
	//
	const double cTIME_TO_INTERSECTION = 60.0;  // seconds
	if ( timeToIntrsctn > cTIME_TO_INTERSECTION ) {

#ifdef DEBUG_CHECK_LOSING_CORRIDOR
		if( debugThisObj ) 
		{
			gout << "  " << timeToIntrsctn;
			gout << " secs away from intersection, ";
			gout << "need to be at most " << cTIME_TO_INTERSECTION;
			gout << " secs away...exit" << endl;
		}
#endif

		return;

	}


	double urgency = ComputeUrgency( 
							distToIntrsctn * cFEET_TO_METER, 
							cInfo.m_lcInfo.urgency 
							);

	//
	// CONDITION:
	// The current distance to the non-dummy intersection is within 5
	// times the estimated forward distance needed to complete the lane
	// change.
	//
	double fwdDistNeeded;
	double fwdTimeNeeded;
	int solId = cInfo.m_pObj->GetSolId();

#ifdef DEBUG_CHECK_LOSING_CORRIDOR
	gout << " input before call: " << endl;
	gout << " myVel = " << myVel << endl;
	gout << " myVel * cMS_TO_MPH " << myVel * cMS_TO_MPH;
	gout << " urgency = " << urgency << endl;
	gout << " solId = " << solId << endl;
#endif
	
	bool hasForwardDist = LcLookupDistTime( 
										myVel * cMS_TO_MPH, 
										urgency,
										solId,
										fwdDistNeeded,
										fwdTimeNeeded
										);

	if( ! hasForwardDist )
	{

#ifdef DEBUG_CHECK_LOSING_CORRIDOR
		gout << " No fwdDistNeeded obtained...return " << endl;
#endif

		return;
	}
	
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
	gout << " fwdDistNeeded obtained in ft = " << fwdDistNeeded << endl;
	gout << " fwdDistNeeded obtained in meters = " << fwdDistNeeded * cFEET_TO_METER << endl;
	gout << " fwdTimeNeeded obtained = " << fwdTimeNeeded << endl;
#endif

//	if( ( distToIntrsctn * cFEET_TO_METER ) > ( 5 * fwdDistNeeded * cFEET_TO_METER ) )
	bool notEnoughRoom = distToIntrsctn < fwdDistNeeded;
	if( notEnoughRoom )
	{

#ifdef DEBUG_CHECK_LOSING_CORRIDOR
		gout << " Distance to intersection is not within limits of estimated forward dist...return";
		gout << endl;
#endif
		return;
	}


	//
	// Perform gap accept.
	//
	bool hasProperGap;
	CLane leftLane;
	CCrdr leftCrdr;
	if( onRoad )
	{
		CLane currLane = cInfo.m_roadPos.GetLane();
		if( currLane.IsLeftMostAlongDir() )
		{	
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
			if( debugThisObj ) 
			{
				gout << "  no more left lanes moving along same ";
				gout << "direction...exit";
				gout << endl;
			}
#endif

			return;
		}
		leftLane = currLane.GetLeft();

		//
		// Make sure this lane leads to the same road as the original
		// destination road.
		//
		CRoad origRoad = cInfo.m_pPath->GetNextCrdr( cInfo.m_roadPos ).GetDstntnLn().GetRoad();
		CIntrsctn nextIntrsctn = cInfo.m_pPath->GetNextIntrsctn( cInfo.m_roadPos );
		TCrdrVec crdrs;
		nextIntrsctn.GetCrdrsStartingFrom( leftLane, crdrs );
		bool foundRoad = false;
		TCrdrVec::iterator i;
		for( i = crdrs.begin(); i != crdrs.end(); i++ )
		{
			foundRoad = (*i).GetDstntnLn().GetRoad() == origRoad;
			if( foundRoad )  break;
		}

		if( !foundRoad )
		{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
			if( debugThisObj ) 
			{
				gout << "  no lane other lane to my left leads to the ";
				gout << "road that I need to get to...exit";
				gout << endl;
			}
#endif
			return;
		}

		//
		// CONDITION:
		// Target lane has proper gap acceptance.
		//
		const double cMAX_DIST_THRESHOLD = 80.0;  // ft.
		const double cMIN_DIST_THRESHOLD = 20.0;  // ft.
		const double cMAX_DIST_THRESHOLD_CUTOFF = 500.0;  // ft.
		const double cMIN_DIST_THRESHOLD_CUTOFF = 200.0;  // ft.
		
		double distThreshold = cMAX_DIST_THRESHOLD;
		if( distToIntrsctn < cMIN_DIST_THRESHOLD_CUTOFF )
		{
			distThreshold = cMIN_DIST_THRESHOLD;
		}
		else if( distToIntrsctn < cMAX_DIST_THRESHOLD_CUTOFF )
		{
			distThreshold = (
					cMIN_DIST_THRESHOLD + 
					( ( distToIntrsctn - cMIN_DIST_THRESHOLD_CUTOFF ) / 
					  ( cMAX_DIST_THRESHOLD_CUTOFF - cMIN_DIST_THRESHOLD_CUTOFF ) *
					  ( cMAX_DIST_THRESHOLD - cMIN_DIST_THRESHOLD )
					  )
					);
		}

		hasProperGap = GapAccept(
					cInfo,
					cInfo.m_backTimeThreshold,
					cInfo.m_fwdTimeThreshold,
					cInfo.m_backDistThreshold,
					cInfo.m_fwdDistThreshold,
					leftLane
					);
	}
	else
	{
		CCrdr currCrdr = cInfo.m_roadPos.GetCorridor();
		bool foundCrdr = currCrdr.GetLeftCrdrAlongDir( leftCrdr );

		if( !foundCrdr )
		{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
			if( debugThisObj ) 
			{
				gout << "  no corridor to the left...exit";
				gout << endl;
			}
#endif

			return;
		}

		//
		// Make sure this crdr leads to the same road as the original
		// destination road.
		//
		CRoad origRoad = currCrdr.GetDstntnLn().GetRoad();
		bool sameRoad = leftCrdr.GetDstntnLn().GetRoad() == origRoad;

		if( !sameRoad )
		{
#ifdef DEBUG_CHECK_LOSING_CORRIDOR
			if( debugThisObj ) 
			{
				gout << "  no lane other crdr to my left leads to the ";
				gout << "road that I need to get to...exit";
				gout << endl;
			}
#endif
			return;
		}

		//
		// CONDITION:
		// Target corridor has proper gap acceptance.
		//
		hasProperGap = GapAccept(
					cInfo,
					cInfo.m_fwdTimeThreshold,
					cInfo.m_fwdDistThreshold,
					leftCrdr
					);
	}

	if( !hasProperGap )
	{

#ifdef DEBUG_CHECK_LOSING_CORRIDOR
		if( debugThisObj ) 
		{
			gout << "  doesn't have proper gap....exit" << endl;
		}
#endif
		
		return;
	}


#ifdef DEBUG_CHECK_LOSING_CORRIDOR
	if( debugThisObj ) 
	{
		gout << "  ** initiate lanechange" << endl;
		gout << "     urgency = " << urgency << endl;
		if( onRoad )
		{
			gout << "     targLane = " << leftLane << endl;
		}
		else
		{
			gout << "     targCrdr = " << leftCrdr.GetRelativeId() << endl;
		}
		gout << "     leftLaneChange = " << true << endl;
	}
#endif

	//
	// Set lane change condition paramteres.
	//
	cond.SetActive( true );
	cond.SetUrgency( urgency );
	if( onRoad )
	{
		cond.SetTargLane( leftLane );
	}
	else
	{
		cond.SetTargCrdr( leftCrdr );
	}
	cond.SetLeftLaneChange( true );
	cond.SetIsForcedLaneOffset( false );
	cond.SetSkipSignal( false );


}  // end of CheckLosingCorridor


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Checks if a highway merge is possible.
//
// Remarks:  This function relies upon attributes being set correctly
//   in CVED to indicate highway on-ramps.  It also assumes that all 
//   highway merges are to the left.
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions info. pointer.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::CheckHighwayMerge( 
	const CAdoInfo& cInfo, 
	CLaneChangeCond& cond
	)

{
	//
	// Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_HIGHWAY_MERGE
	gout << "[" << cInfo.m_pObj->GetId() << "] ";
	gout << "==== LANECHANGE HighwayMerge ==========" << endl;
#endif

	//
	// Exit if no path.
	//
	if ( cInfo.m_pPath->Size() <= 0 )  return;

	const int ONRAMP_ATTR = 41;

	//
	// CONDITION:
	// The vehicle should be on a highway on-ramp.
	//
	if ( cInfo.m_roadPos.IsRoad() ) {
		CRoad currRd = cInfo.m_roadPos.GetRoad();
		vector<CAttr> attrs;
		currRd.QryAttr(attrs);

#ifdef DEBUG_CHECK_HIGHWAY_MERGE
		gout << "  attrs [size=" << attrs.size() << "]" << endl;
		vector<CAttr>::iterator i;
		for ( i = attrs.begin(); i != attrs.end(); i++ )
		{
			CAttr attr = *i;
			gout << "    " << attr.GetId() << " " << attr.GetName() << " ";
			gout << "from=" << attr.GetFrom() << "  to=" << attr.GetTo();
			gout << "  valid=" << attr.IsValid() << endl;
		}
#endif
		
#ifdef DEBUG_CHECK_HIGHWAY_MERGE
		gout << "  on a road...exit" << endl;
#endif

		return;
	}

	CCrdr currCrdr = cInfo.m_roadPos.GetCorridor();
	vector<CAttr> attrs;
	currCrdr.QryAttr( attrs );

#ifdef DEBUG_CHECK_HIGHWAY_MERGE
	gout << "  attrs [size=" << attrs.size() << "]" << endl;
#endif

	bool found = false;
	vector<CAttr>::iterator j;
	for ( j = attrs.begin(); j != attrs.end(); j++ )
	{
		CAttr attr = *j;
		if( attr.GetId() == ONRAMP_ATTR )
		{

#ifdef	DEBUG_CHECK_HIGHWAY_MERGE
			gout << " ADO is on highway onRamp. " << endl;
			gout << "    " << attr.GetId() << " " << attr.GetName() << " ";
			gout << "from=" << attr.GetFrom() << "  to=" << attr.GetTo();
			gout << "  valid=" << attr.IsValid() << endl;
#endif

			found = true;
			break;
		}
	}
	if( ! found )
	{

#ifdef DEBUG_CHECK_HIGHWAY_MERGE
		gout << " ADO is not on a highway onRamp...exit" << endl;
#endif

		return;
	}

	//
	// CONDITION
	// The vehicle should have a minimum speed of 35 mph.
	//
	const double MIN_SPEED = 35.0;
	double myVel = cInfo.m_currVel;
	bool underMinSpeed = ( myVel * cMS_TO_MPH ) < MIN_SPEED;
	if ( underMinSpeed )
	{

#ifdef DEBUG_CHECK_HIGHWAY_MERGE
		gout << "  speed is not fast enough to merge...exit" << endl;
#endif

		return;
	}

	double lengthOfCorridor = currCrdr.GetLength();
	double currentDistOnCord = cInfo.m_roadPos.GetDistance();
	double distToEndOfCorridor = lengthOfCorridor - currentDistOnCord;

	//
	// Compute urgency.
	//
	double urgency = ComputeUrgency( 
							distToEndOfCorridor * cFEET_TO_METER, 
							cInfo.m_lcInfo.urgency 
							);

	//
	// Find the destination road of the current corridor.
	//
	CLane dtntn = currCrdr.GetDstntnLn();

	//
	// Use a vector to hold all the corridors.
	//
	vector<CCrdr> allCrdrs;

	//
	// Get the intersection and return a vector
	// of all the corridors.
	//
	CIntrsctn intsc = cInfo.m_roadPos.GetIntrsctn();
	intsc.GetAllCrdrs( allCrdrs );
	
	//
	// Find the target corridor which is on the left of the
	// current corridor.
	//
	CCrdr targetCorridor;

	//
	// Iterate the vector to find the corridor that 
	// has the same destination lane as the current
	// corridor. Then this other corridor is the
	// target corridor.
	//
	vector<CCrdr>::iterator k;
	bool foundCrdr = false;
	for ( k = allCrdrs.begin(); k != allCrdrs.end(); k++ )
	{
		CCrdr aCrdr = *k;
		if( 
		  aCrdr.GetDstntnLn() == dtntn && 
		  aCrdr.GetRelativeId() != currCrdr.GetRelativeId() 
		  )
		{

#ifdef DEBUG_CHECK_HIGHWAY_MERGE	
			gout << "  found the target corridor" << endl;
#endif

			foundCrdr = true;
			targetCorridor = aCrdr;
			break;
		
		}
		
	}
	if( ! foundCrdr )
	{

#ifdef DEBUG_CHECK_HIGHWAY_MERGE	
		gout << "  unable to find target corridor or the ADO has its own corridor ";
		gout << " so that it's not necessaru to change lane ...exit." << endl;
#endif

		return;
	}

	//
	// CONDITION
	// The lane that the vehicle is going to be merged to
	// has proper gap acceptance.
	//
	bool hasProperGap;
	hasProperGap = GapAccept(
				cInfo,
				cInfo.m_fwdTimeThreshold,
				cInfo.m_fwdDistThreshold,
				targetCorridor
				);
	if ( !hasProperGap ) {

#ifdef DEBUG_CHECK_HIGHWAY_MERGE
		gout << "  doesn't have proper gap....exit" << endl;
#endif
		
		return;
	}

#ifdef DEBUG_CHECK_HIGHWAY_MERGE
	gout << "  ** initiate lanechange" << endl;
	gout << "     urgency = " << urgency << endl;
	gout << "     targCrdrId = " << targetCorridor.GetRelativeId() << endl;
	gout << "     leftLaneChange = " << true << endl;
#endif

	//
	// Set lane change condition parameters. 
	//
	cond.SetActive( true );
	cond.SetUrgency( urgency );
	cond.SetLeftLaneChange( true );	
	cond.SetTargCrdr( targetCorridor );
	cond.SetIsForcedLaneOffset( false );
	cond.SetSkipSignal( false );

}// end of CheckHighwayMerge

//////////////////////////////////////////////////////////////////////////////
//
// Description:	Checks if it is possible to avoid merging vehicles on the
//	highway on-ramp by changing to the lane on the left. 
//
// Remarks:    
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions info. pointer.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////

void
CLaneChangeConds::CheckToAvoidMergingVehicle( 
	const CAdoInfo& cInfo, 
	CLaneChangeCond& cond
	)

{
	//
	// Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
	int objId = cInfo.m_pObj->GetId();
	bool debugThisObj = objId == DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE;
	if( debugThisObj ) 
	{
		gout << "[" << cInfo.m_pObj->GetId() << "] ";
		gout << "==== LANECHANGE CheckToAvoidMergingVehicle ==========";
		gout << endl;
	}
#endif

	//
	// Exit if no path.
	//
	if ( cInfo.m_pPath->Size() <= 0 )  return;

	const int HIGHWAY_ATTR = 37;
	const int ONRAMP_ATTR = 41;

	//
	// CONDITION:
	// Check if the vehicle is on a highway. Need to check
	// both whether the vehicle is on road or in an 
	// intersection.
	//
	bool onRoad = cInfo.m_roadPos.IsRoad();
	if( onRoad )
	{
		// On road.
		CRoad currRd = cInfo.m_roadPos.GetRoad();
		vector<CAttr> attrs;
		currRd.QryAttr(attrs);

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  attrs [size=" << attrs.size() << "]" << endl;
		}
#endif
		bool roadOnHighway= false;
		vector<CAttr>::iterator itr;
		for ( itr = attrs.begin(); itr != attrs.end(); itr++ )
		{
			CAttr attr = *itr;
			if( attr.GetId() == HIGHWAY_ATTR )
			{

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
				if( debugThisObj ) 
				{
					gout << "  on a highway: ";
					gout << attr.GetId() << " " << attr.GetName();
					gout << " " << "val1=" << attr.GetVal1() << "  val2=";
					gout << attr.GetVal2();
					gout << "  valid=" << attr.IsValid() << endl;
				}
#endif

				roadOnHighway= true;
				break;
			}
		}
		if( !roadOnHighway)
		{

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  not a highway road....exit " << endl;
			}
#endif

			return;
		}
	}
	else
	{
		// On intersection.
		CIntrsctn currIntrsctn = cInfo.m_roadPos.GetIntrsctn();
		vector<CAttr> attrs;
		currIntrsctn.QryAttr(attrs);

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  attrs [size=" << attrs.size() << "]" << endl;
		}
#endif

		bool intrsctnOnHighway= false;
		vector<CAttr>::iterator itr;
		for ( itr = attrs.begin(); itr != attrs.end(); itr++ )
		{
			CAttr attr = *itr;
			if( attr.GetId() == HIGHWAY_ATTR )
			{

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
				if( debugThisObj ) 
				{
					gout << "  on a highway: ";
					gout << attr.GetId() << " " << attr.GetName() << " ";
					gout << "val1=" << attr.GetVal1() << "  val2=";
					gout << attr.GetVal2() << "  valid=" << attr.IsValid();
					gout << endl;
				}
#endif

				intrsctnOnHighway= true;
				break;
			}
		}
		if( !intrsctnOnHighway)
		{

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  not on a highway intersection...exit ";
				gout << endl;
			}
#endif

			return;
		}
	}

	//
	// CONDITION:
	// Am I on a merging lane?
	//
	CIntrsctn intrsctn;
	CCrdr currCrdr;
	if( onRoad )
	{

		//
		// On a road, get the next intersection and corridor by using
		// my path.
		//
		intrsctn = cInfo.m_pPath->GetNextIntrsctn( cInfo.m_roadPos );
		currCrdr = cInfo.m_pPath->GetNextCrdr( cInfo.m_roadPos );
		if( ! intrsctn.IsValid() )
		{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  no upcoming or invalid intersection...exit";
				gout << endl;
			}
#endif
			return;
		}
	}
	else
	{
		//
		// Already inside an intersection.
		//
		intrsctn = cInfo.m_roadPos.GetIntrsctn();
		currCrdr = cInfo.m_roadPos.GetCorridor();
	}
	
	if( ! currCrdr.IsValid() )
	{
		return;
	}
	//
	// Make sure the current corridor is not on a highway onRamp.
	//
	vector<CAttr> crdrAttrs;
	currCrdr.QryAttr( crdrAttrs );

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  curr/next crdr attrs [size=" << crdrAttrs.size() << "]";
		gout << endl;
	}
#endif

	bool foundOnRamp = false;
	vector<CAttr>::iterator k;
	for ( k = crdrAttrs.begin(); k != crdrAttrs.end(); k++ )
	{
		CAttr attr = *k;
		if( attr.GetId() == ONRAMP_ATTR )
		{

#ifdef	DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE		
			if( debugThisObj ) 
			{
				gout << "    on an onRamp: ";
				gout << "    " << attr.GetId() << " " << attr.GetName();
				gout << " from=" << attr.GetFrom() << "  to=" << attr.GetTo();
				gout << "  valid=" << attr.IsValid() << endl;
			}
#endif
			foundOnRamp = true;
			break;
			
		}
	}

	//
	// Use a vector to hold all the corridors, iterate the
	// vector to find another corridor that has the same 
	// destination lane as the current corridor.
	//
	CLane dtntnLn = ( currCrdr.GetDstntnLn() );
	vector<CCrdr> allCrdrs;
	intrsctn.GetAllCrdrs( allCrdrs );	

	vector<CCrdr>::iterator i;
	bool found = false;
	for( i = allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		CCrdr aCrdr = *i;
		if( (aCrdr.GetDstntnLn() == dtntnLn) && 
			(aCrdr.GetRelativeId() != currCrdr.GetRelativeId() ) &&
			(foundOnRamp == false) 
		)
		{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  currently on merging lane" << endl;
			}
#endif

			found = true;
			break;
		}
	}

	if(	!found )
	{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  not a merging lane, no need to change lane...exit";
			gout << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// Check if there exists a left lane so that the vehicle can
	// change to in order to avoid merging vehicle.

	CLane targetLane, currLane;
	CCrdr currentCrdr, targetCrdr;
	//
	// At this point, the vehicle knows it is on a merging lane.
	// If on a road, get the current lane and assign the 
	// target lane.
	//
	if( onRoad )
	{

		//
		// I'm on a road.  Get the lane to my left and make sure it's
		// traveling in the same direction.
		//
		currLane = cInfo.m_roadPos.GetLane();
		bool haveLane = ! currLane.IsLeftMostAlongDir();
		if( haveLane )
		{
			targetLane = currLane.GetLeft();

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  has a left lane = " << targetLane << endl;
			}
#endif

		}
		else
		{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  no lane to the left...exit" << endl;
			}
#endif

			return;
		}
	}
	else
	{

		//
		// I am inside an intersection.  
		//
		// Get the source lane of the current corridor and check if 
		// source lane has a lane to its left.  If so, find the corridor
		// that has this lane as its source lane.
		// 
		currentCrdr = cInfo.m_roadPos.GetCorridor();
		CLane srcLane = currentCrdr.GetSrcLn();

		if( ! srcLane.IsValid() )
		{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  unable to find source lane...exit" << endl;
			}
#endif

			return;
		}

		bool haveLane = ! srcLane.IsLeftMostAlongDir();
		if( ! haveLane )
		{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  no lane to the left...exit" << endl;
			}
#endif

			return;
		}

		CLane srcLaneLeftLn = srcLane.GetLeft();

		//
		// Use the vector declared earlier to find the corridor 
		// whose source lane matches the source lane's left lane.
		//
		bool foundCrdr = false;
		vector<CCrdr>::iterator j;
		for ( j = allCrdrs.begin(); j != allCrdrs.end(); j++ )
		{
			CCrdr aCrdr = *j;
			if( aCrdr.GetSrcLn() == srcLaneLeftLn )
			{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
				if( debugThisObj ) 
				{
					gout << "  found the target corridor" << endl;
				}
#endif

				foundCrdr = true;
				targetCrdr = aCrdr;
				break;
			}
		}

		if( ! foundCrdr )
		{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  unable to find target corridor...exit";
				gout << endl;
			}
#endif

			return;
		}
	}

	//
	// Compute distance. If it is on road, get the distance to the 
	// next intersection. If it is not on road, set the distance to 0.
	//
	double distanceToIntrsctn = 0.0;
	if( onRoad )
	{
		distanceToIntrsctn = cInfo.m_pPath->GetDistToNextIntrsctn(
													cInfo.m_roadPos 
													);
	}
	distanceToIntrsctn = distanceToIntrsctn * cFEET_TO_METER;

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  dist to intersection = " << distanceToIntrsctn << "m";
		gout << endl;
	}
#endif

	//
	// Compute urgency
	//
	double urgency = ComputeUrgency( 
							distanceToIntrsctn, 
							cInfo.m_lcInfo.urgency 
							);

	//
	// CONDITION
	// The lane that the vehicle is going to be changed to
	// has proper gap acceptance.
	//
	bool hasProperGap;

	if( onRoad )
	{	
		hasProperGap = GapAccept(
				cInfo,
				cInfo.m_backTimeThreshold,
				cInfo.m_fwdTimeThreshold,
				cInfo.m_backDistThreshold,
				cInfo.m_fwdDistThreshold,
				targetLane
				);
		if ( !hasProperGap ) {

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  doesn't have proper gap....exit" << endl;
			}
#endif
		
			return;
		}
	}
	else
	{
		hasProperGap = GapAccept(
				cInfo,
				cInfo.m_fwdTimeThreshold,
				cInfo.m_fwdDistThreshold,
				targetCrdr
				);
		if( !hasProperGap ) 
		{

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  doesn't have proper gap....exit" << endl;
			}
#endif
		
			return;
		}
	}

#ifdef DEBUG_CHECK_TO_AVOID_MERGING_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  ** initiate lanechange" << endl;
		gout << "     urgency = " << urgency << endl;
		if( onRoad )
		{
			gout << "     targLane = " << targetLane << endl;
		}
		else
		{
			gout << "     targCrdrId = " << targetCrdr.GetRelativeId() << endl;
		}
		gout << "     leftLaneChange = " << true << endl;
	}
#endif

	//
	// Set the lane change parameters.
	//
	cond.SetActive( true );
	cond.SetUrgency( urgency );
	cond.SetLeftLaneChange( true );
	if( onRoad )
	{
		cond.SetTargLane( targetLane );
	}
	else
	{
		cond.SetTargCrdr( targetCrdr );
	}
	cond.SetIsForcedLaneOffset( false );
	cond.SetSkipSignal( false );

} //end of CheckToAvoidMergingVehicle	




bool
CLaneChangeConds::GetLeadObjectId( 
			const CAdoInfo& cInfo, 
			vector<int>& objs, 
			int& leadObjId 
			)
{
	bool foundObj = false;
	vector<int>::iterator i;
	for( i = objs.begin(); i != objs.end(); i++ ) 
	{
		int objId = *i;

		// don't look at myself
		if( objId == cInfo.m_pObj->GetId() )  continue;

		// get the CVED object
		const CDynObj* pLeadObj = m_pCved->BindObjIdToClass( objId );

		// ignore invalid objects
		if( !pLeadObj->IsValid() )  continue;

		// ignore all objects except for vehicles for now
		cvEObjType leadObjType = pLeadObj->GetType();
		bool ignoreObject = (
					leadObjType != eCV_VEHICLE &&
					leadObjType != eCV_TRAJ_FOLLOWER &&
					leadObjType != eCV_EXTERNAL_DRIVER
					);
		if( ignoreObject )  continue;

		leadObjId = objId;
		foundObj = true;
		break;
	}

	return foundObj;
}  // end of GetLeadObjectId



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Checks if a lane change is necessary due to a slower
//   vehicle ahead of me.
//
// Remarks:    
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions info. pointer.
//   currentFrame - The simulation's current frame of execution.
//   freeDriveTargVel - The target velocity according to FreeDrive.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::CheckSlowVehicle( 
			const CAdoInfo& cInfo, 
			CLaneChangeCond& cond,
			long currentFrame,
			double freeDriveTargVel
			)
{

	//
    // Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_SLOW_VEHICLE
	int objId = cInfo.m_pObj->GetId();
	bool debugThisObj = objId == DEBUG_CHECK_SLOW_VEHICLE;
	if( debugThisObj ) 
	{
		gout << "[" << cInfo.m_pObj->GetId() << "] ";
		gout << "==== LANECHANGE CheckSlowVehicle ==========" << endl;
	}
#endif

	//
	// Exit if no path.
	//
	if ( cInfo.m_pPath->Size() <= 0 )  return;

	//
	// CONDITION:
	// At least 3 seconds have expired since the completion of the
	// previous lane change.
	//
	const double cMIN_TIME_SINCE_LC = 3.0;  // seconds
	if( cInfo.m_prevLcCompleteFrame >= 0 ) 
	{
		double framesExpired = currentFrame - cInfo.m_prevLcCompleteFrame;
		double timeExpired = framesExpired * cInfo.m_timeStepDuration;
		if( timeExpired < cMIN_TIME_SINCE_LC ) 
		{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
			if( debugThisObj )
			{
				gout << "  only " << timeExpired;
				gout << " seconds since last change....exit" << endl;
			}
#endif

			return;
		}
	}

	//
	// CONDITION:
	// The next non-dummy intersection is at least 900 feet away.
	//
	double distToIntrsctn = cInfo.m_pPath->GetDistToNextIntrsctn( 
													cInfo.m_roadPos 
													);

#ifdef DEBUG_CHECK_SLOW_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  distToIntrsctn = " << distToIntrsctn << " m" << endl;
	}
#endif

	const double cMAX_DIST_TO_INTRSCTN = 900.0;  // feet
	bool tooCloseToIntrsctn = distToIntrsctn < cMAX_DIST_TO_INTRSCTN;
	if( tooCloseToIntrsctn ) 
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  too close to interesction...exit" << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// There exists a lane to the left of the current lane that's
	// traveling in the same direction.
	//
	CLane currLane = cInfo.m_roadPos.GetLane();
	bool haveLane = !currLane.IsLeftMostAlongDir();

	if( !haveLane ) 
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  no lane to the left traveling in the same ";
			gout << "direction as the current lane...exit" << endl;
		}
#endif

		return;
	}

	CLane leftLane = currLane.GetLeft();
	if( leftLane.IsTurnLane() )
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  lane to left is turning lane...exit " << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// There is a vehicle in front me that's moving at least
	// 10 mph and at most 90% of my target velocity.
	//

	//
	// Use the path to get a list of vehicle.
	//
	vector<int> objs;
	int numObjs = cInfo.m_pPath->GetObjectsOnPathLane( cInfo.m_roadPos, objs );

	if ( numObjs <= 0 ) 
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  no objects to pass...exit" << endl;
		}
#endif

		return;
	}

	//
	// Find the object directly in front of me from the list
	// of objects returned by the path.
	//
	int leadObjId;
	bool foundObject = GetLeadObjectId( cInfo, objs, leadObjId );
	if( !foundObject )  return;

#ifdef DEBUG_CHECK_SLOW_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  found object " << leadObjId << endl;
	}
#endif

	// 
	// Use the lead object's id to get the CVED object.
	//
	const CDynObj* pLeadObj = m_pCved->BindObjIdToClass( leadObjId );
	if( !pLeadObj->IsValid() ) 
	{
		cerr << "  invalid lead object (id = ";
		cerr << leadObjId << ")" << endl;

		return;
	}

#ifdef DEBUG_CHECK_SLOW_VEHICLE
	if( debugThisObj ) 
	{
		gout << "    type = " << cvObjType2String( pLeadObj->GetType() );
		gout << "  name = " << pLeadObj->GetName();
		gout << "  hcsmid = " << pLeadObj->GetHcsmId() << endl;
		gout << "  pos = " << pLeadObj->GetPos() << endl;
	}
#endif

	//
	// Make sure the lead object is moving at least 10 mph.
	//
	const double cMIN_LEAD_OBJ_VEL = 10.0;                   // mph
	double leadObjVel = pLeadObj->GetVelImm();               // m/s
	bool leadObjTooSlow = leadObjVel * cMS_TO_MPH < cMIN_LEAD_OBJ_VEL;
	if( leadObjTooSlow )
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  lead object moving too slow (vel = ";
			gout << leadObjVel  * cMS_TO_MPH << " mph) ...exit" << endl;
		}
#endif

		return;
	}

	//
	// The lead object should be moving slower than 90% of my target
	// velocity.  If the lead object is not moving slower than 80%
	// of my target velocity, then wait 10 seconds before initiating
	// lane change.
	//
	double my90PercentTargVel = freeDriveTargVel * 0.90;
	double my80PercentTargVel = freeDriveTargVel * 0.80;

#ifdef DEBUG_CHECK_SLOW_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  my80Vel = " << my80PercentTargVel * cMS_TO_MPH;
		gout << " mph   my90Vel = " << my90PercentTargVel * cMS_TO_MPH;
		gout << " mph   leadObjVel = " << leadObjVel * cMS_TO_MPH;
		gout << " mph" << endl;
	}
#endif

	bool leadObjSlower = leadObjVel < my90PercentTargVel;
	if( leadObjSlower && leadObjVel > my80PercentTargVel ) 
	{
		const double cWAIT_TIME = 10.0;   // seconds
		// increment counter until cWAIT_TIME seconds is reached
		if( m_slowLcWaitStartFrame < 0 ) 
		{
			m_slowLcWaitStartFrame = currentFrame;
		}
		double waitTime = ( currentFrame - m_slowLcWaitStartFrame ) / 30.0;
		bool notLongEnough = waitTime < cWAIT_TIME;
		if( notLongEnough ) 
		{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
			if( debugThisObj ) 
			{
				gout << "  have to wait until " << cWAIT_TIME;
				gout << " seconds have expired...exit" << endl;
				gout << "    current wait time = " << waitTime;
				gout << " seconds" << endl;
			}
#endif

			return;
		}
	}
	else 
	{
		m_slowLcWaitStartFrame = -1;
	}

#ifdef DEBUG_CHECK_SLOW_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  m_slowLcWaitStartFrame = " << m_slowLcWaitStartFrame;
		gout << endl;
	}
#endif

	if( !leadObjSlower ) 
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  lead object moving over 90% of my velocity...exit";
			gout << endl;
			gout << "    lead vel         = " << leadObjVel * cMS_TO_MPH;
			gout << " mph" << endl;
			gout << "    my FreeDrive vel = ";
			gout << freeDriveTargVel * cMS_TO_MPH << " mph" << endl;
		}
#endif

		return;
	}

	double myVel = cInfo.m_currVel;   // m/s
	double leadObjAccel = GetAccel( pLeadObj );

	//
	// CONDITION:
	// The lead object should not be accelerating more than 0.05 m/s^2.
	//
	const double  cACCEL_THRESHOLD = 0.05;     // m/s^2
	bool leadObjAccelerating = leadObjAccel > cACCEL_THRESHOLD;
	if( leadObjAccelerating ) 
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  lead object is accelerating....exit   leadAccel = ";
			gout << leadObjAccel << " m/s^2" << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// Target lane has proper gap acceptance.
	//
	bool hasProperGap;
	hasProperGap = GapAccept(
				cInfo,
				cInfo.m_backTimeThreshold,
				cInfo.m_fwdTimeThreshold,
				cInfo.m_backDistThreshold,
				cInfo.m_fwdDistThreshold,
				leftLane
				);
	if( !hasProperGap ) 
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  doesn't have proper gap....exit" << endl;
		}
#endif
		
		return;
	}


	double urgency = ComputeUrgency( distToIntrsctn, cInfo.m_lcInfo.urgency );


#ifdef DEBUG_CHECK_SLOW_VEHICLE
	if( debugThisObj ) 
	{
		gout << "  ** initiate lanechange" << endl;
		gout << "     urgency = " << urgency << endl;
		gout << "     targLane = " << leftLane << endl;
		gout << "     leftLaneChange = " << true << endl;
	}
#endif

	//
	// Set lane change condition paramteres.
	//
	cond.SetActive( true );
	cond.SetUrgency( urgency );
	cond.SetTargLane( leftLane );
	cond.SetLeftLaneChange( true );
	cond.SetIsForcedLaneOffset( false );
	cond.SetSkipSignal( false );

	m_slowLcWaitStartFrame = -1;
	
}  // end of CheckSlowVehicle


//////////////////////////////////////////////////////////////////////////////
//
// Description:	Checks if a lane change is necessary due to a very slow 
//	vehicle or stopped vehicle ahead of me.
//
// Remarks:    
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions info. pointer.
//   currentFrame - The simulation's current frame of execution.
//   freeDriveTargVel - The target velocity according to FreeDrive.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::CheckVerySlowVehicle( 
	const CAdoInfo& cInfo, 
	CLaneChangeCond& cond,
	long currentFrame,
	double freeDriveTargVel
	)
{

	//
    // Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
	bool debugThisObj = DEBUG_CHECK_VERY_SLOW_VEHICLE == cInfo.m_pObj->GetId();
	if( debugThisObj )
	{
		gout << "[" << cInfo.m_pObj->GetId() << "] ";
		gout << "==== LANECHANGE CheckVerySlowVehicle ==========" << endl;
	}
#endif

	//
	// Exit if no path.
	//
	if ( cInfo.m_pPath->Size() <= 0 )  return;

	//
	// CONDITION:
	// The next non-dummy intersection is at least 50 meters away.
	//
	double distToIntrsctn = cInfo.m_pPath->GetDistToNextIntrsctn( 
													cInfo.m_roadPos 
													);

#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
	if( debugThisObj )
	{
		gout << "  distToIntrsctn = " << distToIntrsctn << " m" << endl;
	}
#endif

	const double cMAX_DIST_TO_INTRSCTN = 50.0;  // meters
	bool tooCloseToIntrsctn = distToIntrsctn < cMAX_DIST_TO_INTRSCTN;
	if( tooCloseToIntrsctn )
	{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
		if( debugThisObj )
		{
			gout << "  too close to interesction...exit" << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// There exists a lane to the left of the current lane that's 
	// traveling in the same direction.
	//
	CLane currLane = cInfo.m_roadPos.GetLane();
	bool haveLane = !currLane.IsLeftMostAlongDir();
	if( !haveLane ) 
	{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
		if( debugThisObj )
		{
			gout << "  no lane to the left traveling in the same ";
			gout << "  direction as the current lane...exit" << endl;
		}
#endif

		return;
	}

	CLane leftLane = currLane.GetLeft();
	if( leftLane.IsTurnLane() )
	{
#ifdef DEBUG_CHECK_SLOW_VEHICLE
		if( debugThisObj ) 
		{
			gout << "  lane to left is turning lane...exit " << endl;
		}
#endif

		return;
	}


	//
	// CONDITION:
	// There exists a vehicle in front of the ADO that's stopping or
	// moving at most 10 mph and not more than 90% of the ADO's velocity.
	// And it is not accelerating.
	//

	//
	// Use the path to get a list of vehicles.
	//
	vector<int> objs;
	int numObjs = cInfo.m_pPath->GetObjectsOnPathLane( cInfo.m_roadPos, objs );
	bool noObjsFound = numObjs <= 0;
	if( noObjsFound ) 
	{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
		if( debugThisObj )
		{
			gout << "  no objects to pass...exit" << endl;
		}
#endif

		return;
	}

	//
	// Find the object directly in front of me from the list
	// of objects returned by the path.
	//
	int leadObjId;
	bool foundObject = GetLeadObjectId( cInfo, objs, leadObjId );
	if( !foundObject )  
	{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
		if( debugThisObj )
		{
			gout << "  unable to find lead object id...exit" << endl;
		}
#endif

		return;
	}

#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
	if( debugThisObj )
	{
		gout << "  found object " << leadObjId << endl;
	}
#endif

	// 
	// Use the lead object's id to get the CVED object.
	//
	const CDynObj* pLeadObj = m_pCved->BindObjIdToClass( leadObjId );
	if( !pLeadObj->IsValid() ) 
	{
		cerr << "  invalid lead object (id = ";
		cerr << leadObjId << ")" << endl;

		return;
	}

#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
	if( debugThisObj )
	{
		gout << "  type = " << cvObjType2String( pLeadObj->GetType() );
		gout << "  name = " << pLeadObj->GetName();
		gout << "  hcsmid = " << pLeadObj->GetHcsmId() << endl;
		gout << "  pos = " << pLeadObj->GetPos() << endl;
	}
#endif

	const double cMAX_LEAD_OBJ_VEL = 10.0;                   // mph
	double leadObjVel = pLeadObj->GetVelImm();               // m/s
	bool leadObjTooFast = leadObjVel * cMS_TO_MPH > cMAX_LEAD_OBJ_VEL;
	if ( leadObjTooFast )
	{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
		if( debugThisObj )
		{
			gout << "  lead object moving too fast (vel = ";
			gout << leadObjVel  * cMS_TO_MPH << " mph) ...exit" << endl;
		}
#endif

		return;
	}

	double myVel = cInfo.m_currVel;                 // m/s
	const double cLEAD_OBJ_PERCENTAGE_VEL = 0.90;            // %
	if( myVel * cLEAD_OBJ_PERCENTAGE_VEL < leadObjVel ) 
	{
		//
		// Find out if the lead vehicle is accelerating....if not, 
		// then go ahead with pass if it's velocity is under my 
		// desired velocity from FreeDrive.
		//
		double accel = GetAccel( pLeadObj );
		const double cACCEL_THRESHOLD = 0.05;                // m/s^2
		bool leadObjAccelerating = accel > cACCEL_THRESHOLD;

		if( leadObjAccelerating ) 
		{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
			if( debugThisObj )
			{
				gout << "  lead object is accelerating....exit" << endl;
				gout << "    leadAccel = " << accel << " m/s^2" << endl;
			}
#endif

			return;
		}
		else 
		{
			double myAdjustedVel = freeDriveTargVel * cLEAD_OBJ_PERCENTAGE_VEL;
			bool leadObjTooFast = myAdjustedVel < leadObjVel;
			if( leadObjTooFast ) 
			{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
				if( debugThisObj )
				{
					gout << "  lead object moving over 90% of my velocity";
					gout << "...exit";
					gout << endl;
					gout << "    lead vel = " << leadObjVel * cMS_TO_MPH;
					gout << " mph" << endl;
					gout << "    my FreeDrive vel = ";
					gout << freeDriveTargVel * cMS_TO_MPH << " mph" << endl;
				}
#endif

				return;
			}
		}
	}

	//
	// CONDITION:
	// The lead vehicle is within 5 seconds or 200 feet.
	//
	const double cMAX_TTC              = 5.0;    // seconds
	const double cMAX_DIST_TO_LEAD_OBJ = 100.0;  // feet
	CHcsm* pLeadHcsm = m_pRootCollection->GetHcsm( pLeadObj->GetHcsmId() );
	if( !pLeadHcsm ) 
	{
		cerr << "[" << cInfo.m_pObj->GetId() << "] CheckVerySlowVehicle: ";
		cerr << "cannot get pointer maintain gap hcsm (cved id=";
		cerr << pLeadObj->GetId() << ")" << endl;

		return;
	}

	CRoadPos leadRoadPos;
	if( !pLeadHcsm->GetMonitorByName( "RoadPos", &leadRoadPos ) || !leadRoadPos.IsValid() ) 
	{
		cerr << "[" << cInfo.m_pObj->GetId() << "] CheckVerySlowVehicle: ";
		cerr << "unable to read RoadPos monitor from object ";
		cerr << "(cved id=" << pLeadObj->GetId() << ")";
		cerr << endl;

		return;
	}

	double distToLeadObj = cInfo.m_pPath->GetLength( 
											&cInfo.m_roadPos, 
											&leadRoadPos 
											);
	bool unableToGetDist = distToLeadObj < cNEAR_ZERO;
	if( unableToGetDist )
	{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
		if( debugThisObj )
		{
			gout << "  cannot compute distance to lead object...exit" << endl;
		}
#endif
		
		return;
	}
	double timeToLeadObj = ( distToLeadObj * cFEET_TO_METER ) / myVel;

#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
	if( debugThisObj )
	{
		gout << "  dist to lead obj = " << distToLeadObj << " ft" << endl;
		gout << "  time to lead obj = " << timeToLeadObj << " s" << endl;
	}
#endif

	bool leadObjClosingFast = timeToLeadObj < cMAX_TTC;
	if( !leadObjClosingFast ) 
	{
		bool closeEnough = distToLeadObj < cMAX_DIST_TO_LEAD_OBJ;
		if( !closeEnough ) 
		{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
			if( debugThisObj )
			{
				gout << "  lead object is too far away (" << timeToLeadObj;
				gout << " s, " << distToLeadObj << " ft)...exit" << endl;
			}
#endif

			return;
		}
	}

	//
	// CONDITION:
	// Target lane has proper gap acceptance.
	//
	bool hasProperGap;
	hasProperGap = GapAccept(
				cInfo,
				cInfo.m_backTimeThreshold,
				cInfo.m_fwdTimeThreshold,
				cInfo.m_backDistThreshold,
				cInfo.m_fwdDistThreshold,
				leftLane
				);
	if( !hasProperGap ) 
	{
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
		if( debugThisObj )
		{
			gout << "  doesn't have proper gap....exit" << endl;
		}
#endif
		
		return;
	}

	
	//
	// Compute urgency.
	//
	double urgency = ComputeUrgency( distToIntrsctn, cInfo.m_lcInfo.urgency );
	
#ifdef DEBUG_CHECK_VERY_SLOW_VEHICLE
	if( debugThisObj )
	{
		gout << "  ** initiate lanechange" << endl;
		gout << "     urgency = " << urgency << endl;
		gout << "     targLane = " << leftLane << endl;
		gout << "     leftLaneChange = 1" << endl;
	}
#endif

	//
	// Set lane change condition paramteres.
	//
	cond.SetActive( true );
	cond.SetUrgency( urgency );
	cond.SetTargLane( leftLane );
	cond.SetLeftLaneChange( true );
	cond.SetIsForcedLaneOffset( false );
	cond.SetSkipSignal( false );

}  // end of CheckVerySlowVehicle


//////////////////////////////////////////////////////////////////////////////
//
// Description: Checks if the given lane leads to a corridor that merges 
//   into another corridor.
//
// Remarks:
//
// Arguments:
//   cIntrsctn - The intersection in which to look for merging corridors.
//   cLane     - The lane to check.
//
// Returns: A boolean indicating if the given lane leads to a corridor
//   that merges with an another corridor in the given intersection.
//
//////////////////////////////////////////////////////////////////////////////
static bool
OnMergeLane( const CIntrsctn& cIntrsctn, const CLane& cLane )
{
	//
	// Hold all the corridors in the intersection and check if there 
	// are corridors that have the same destination lane. If so, have 
	// found at least one corridor is going to be merged.
	//
	vector<CCrdr> allCrdrs;
	cIntrsctn.GetAllCrdrs( allCrdrs );
	int i;
	for( i = 0; i < allCrdrs.size(); i++ )
	{
		int j;
		for( j = i + 1; j < allCrdrs.size(); j++ )
		{
			if( allCrdrs[i].GetDstntnLn() == allCrdrs[j].GetDstntnLn() )
			{
				//
				// Found 2 corridors that are merging into one.
				//
				bool onMergeLane = (
							allCrdrs[i].GetSrcLn() == cLane ||
							allCrdrs[j].GetSrcLn() == cLane
							);
				if( onMergeLane)  return true;
			}
		}
	}
	
	return false;
}  // end of OnMergeLane


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Checks if a lane change is necessary due moving into a
//   non-passing lane.
//
// Remarks:    
//
// Arguments:
//   cInfo  - Vehicle information.
//   pCond - The lane change conditions cInfo. pointer.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::CheckNonPassingLane( 
	const CAdoInfo& cInfo, 
	CLaneChangeCond& cond,
	long currentFrame	
	)
{

	//
    // Initialize.
	//
	cond.SetActive( false );

#ifdef DEBUG_CHECK_NON_PASSING_LANE
	int objId = cInfo.m_pObj->GetId();
	bool debugThisObj = objId == DEBUG_CHECK_NON_PASSING_LANE;
	if( debugThisObj ) 
	{
		gout << "[" << objId << "] ";
		gout << "==== LANECHANGE CheckNonPassingLane ==========" << endl;
	}
#endif

	//
	// Exit if no path.
	//
	if ( cInfo.m_pPath->Size() <= 0 )  return;

	//
	// Exit if on an intersection.
	//
	if ( !cInfo.m_roadPos.IsRoad() )  return;

	//
	// CONDITION:
	// The previous lane change shouldn't have been due to path guidance or
	// due to losing corridor.
	//
	bool alreadyHadHigherPriorityLaneChange = (
		cInfo.m_pastLcCond == eLC_PATH_GUIDANCE ||
		cInfo.m_pastLcCond == eLC_LOSING_CORRIDOR
		);
	if( alreadyHadHigherPriorityLaneChange )
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		if( debugThisObj ) 
		{
			gout << "  already had non-passing or losing corridor lc ";
			gout << "maneuver";
			gout << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// At least 5 seconds have expired since the completion of the
	// previous lane change.
	//
	const double cMIN_TIME_SINCE_LC = 5.0;  // seconds
	if ( cInfo.m_prevLcCompleteFrame >= 0 ) {

		double framesExpired = static_cast<double>(currentFrame - cInfo.m_prevLcCompleteFrame);
		double timeExpired = framesExpired * cInfo.m_timeStepDuration;
		if ( timeExpired < cMIN_TIME_SINCE_LC ) {

#ifdef DEBUG_CHECK_NON_PASSING_LANE
			if( debugThisObj )
			{
				gout << "  only " << timeExpired;
				gout << " seconds since last change....exit" << endl;
			}
#endif

			return;

		}

	}
	
	//
	// CONDITION:
	// The ADO should be on a highway and there should be a lane to the right 
	// moving in the same direction as the current lane. The lane to right 
	// should also not be an off-ramp.  
	//
	// If the ADO is the left-most lane then it should always attempt the 
	// maneuver if the rest of the conditions are true.  If the ADO is not 
	// in the left-most lane, the chances of ADO moving to a non-passing 
	// lane should decrease.
	// 
	CRoad currRoad = cInfo.m_roadPos.GetRoad();
	if( !currRoad.ActiveAttr( 37 ) )
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		if( debugThisObj ) 
		{
			gout << "  not on a highway...exit" << endl;
		}
#endif

		return;
	}

	//
	// Don't change lanes unless I'm in the left-most lane.
	// 
	CLane currLane = cInfo.m_roadPos.GetLane();
	if( !currLane.IsLeftMostAlongDir() )
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		if( debugThisObj ) 
		{
			gout << "  there's already a lane to my left that's ";
			gout << "moving in the same direction...exit" << endl;
		}
#endif

		return;
	}
	
	//
	// Find the lane to the right.
	//
	bool haveLane = !currLane.IsRightMost();

	if( !haveLane ) 
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		if( debugThisObj ) 
		{
			gout << "  no lane to the left traveling in the same ";
			gout << "  direction as the current lane...exit" << endl;
		}
#endif

		return;
	}

	CLane rightLane = currLane.GetRight();

	//
	// CONDITION:
	// The target lane shouldn't be a merge lane.
	//
	// NOTE:  It's debatable if the next intersection should be obtained
	//        using the path or the current lane.  If the path is not
	//        long enough, this check may fail on certain long roads.
	//        Perhaps we want it to fail on those long roads?????
	//
	//CIntrsctn nextIntrsctn = cInfo.m_pPath->GetNextIntrsctn(cInfo.m_roadPos);
	CIntrsctn nextIntrsctn = currLane.GetNextIntrsctn();
	if( nextIntrsctn.IsValid() )
	{
		if( OnMergeLane( nextIntrsctn, rightLane ) )
		{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
			if( debugThisObj ) 
			{
				gout << "  right lane is a merge lane...exit" << endl;
			}
#endif

			return;
		}
	}

	//
	// Calculate the distance and time to the next non-dummy intersection.
	//
	double distToIntrsctn = cInfo.m_pPath->GetDistToNextIntrsctn( 
		cInfo.m_roadPos 
		);	// ft

	double timeToIntrsctn;
	double myVel = cInfo.m_currVel;
	bool stopped = myVel > cNEAR_ZERO;
	if( stopped ) 
	{
		timeToIntrsctn = distToIntrsctn * cFEET_TO_METER / myVel;
	}
	else {
		timeToIntrsctn = 1000.0;  // a large value
	}

#ifdef DEBUG_CHECK_NON_PASSING_LANE
	if( debugThisObj ) 
	{
		gout << "  distToIntrsctn = " << distToIntrsctn << " ft" << endl;
		gout << "  timeToIntrsctn = " << timeToIntrsctn << " secs" << endl;
	}
#endif

	//
	// CONDITION:
	// Need to be at least 300 feet away from the intersection.
	//
	const double cMIN_DIST_TO_INTERSECTION = 300.0;  // feet
	bool tooCloseToIntrsctn = distToIntrsctn < cMIN_DIST_TO_INTERSECTION;
	if( tooCloseToIntrsctn ) 
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		if( debugThisObj ) 
		{
			gout << "  " << distToIntrsctn << " ft away from intersection, ";
			gout << "need to be at least " << cMIN_DIST_TO_INTERSECTION;
			gout << " ft away...exit" << endl;
		}
#endif

		return;
	}

	//
	// CONDITION:
	// Need to be at least 5 seconds away from the next intersection.
	//
	const double cTIME_TO_INTERSECTION = 10.0;  // seconds
	bool tooNearIntrsctn = timeToIntrsctn < cTIME_TO_INTERSECTION;
	if( tooNearIntrsctn ) 
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		if( debugThisObj ) 
		{
			gout << timeToIntrsctn << " secs away from intersection, ";
			gout << "need to be at least " << cTIME_TO_INTERSECTION;
			gout << " secs away...exit" << endl;
		}
#endif

		return;
	}


	//
	// Compute urgency.
	//
	double urgency = ComputeUrgency( 
							distToIntrsctn * cFEET_TO_METER, 
							cInfo.m_lcInfo.urgency 
							);

	//
	// CONDITION:
	// The current distance to the non-dummy intersection is within 5
	// times the estimated forward distance needed to complete the lane
	// change.
	//
	double fwdDistNeeded;
	double fwdTimeNeeded;
	int solId = cInfo.m_pObj->GetSolId();

	bool hasForwardDist = LcLookupDistTime( 
										myVel * cMS_TO_MPH, 
										urgency,
										solId,
										fwdDistNeeded,
										fwdTimeNeeded
										);

	if( ! hasForwardDist )
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		gout << "  unable to lookup estimated dist for lane change";
		gout << endl;
#endif

		return;
	}
	
#ifdef DEBUG_CHECK_NON_PASSING_LANE
	gout << "  fwdDistNeeded = " << fwdDistNeeded << " ft" << endl;
	gout << "  fwdTimeNeeded obtained = " << fwdTimeNeeded << " sec" << endl;
#endif

	bool atLeast3TimesAway = distToIntrsctn > ( 3 * fwdDistNeeded );
	if( !atLeast3TimesAway )
	{

#ifdef DEBUG_CHECK_NON_PASSING_LANE
		gout << "  within 3 times of estimated distance needed to ";
		gout << "complete lc" << endl;
#endif
		return;
	}
	

	//
	// CONDITION:
	// Check for proper gap acceptance.
	//
	bool hasProperGap;
	hasProperGap = GapAccept(
				cInfo,
				3.0,
				10.0,
				200.0,
				500.0,
				rightLane
				);
	if( !hasProperGap )
	{
#ifdef DEBUG_CHECK_NON_PASSING_LANE
		if( debugThisObj ) 
		{
			gout << "  doesn't have proper gap....exit" << endl;
		}
#endif
		
		return;
	}


	//
	// Compute direction of lane change.
	//
	bool isLeftLaneChange = false;

	//
	// Compute the lateral distance needed for this lane change.
	//
	// Wait to see if this should be computed here.
	//
	
#ifdef DEBUG_CHECK_NON_PASSING_LANE
	if( debugThisObj ) 
	{
		gout << "  ** initiate lanechange" << endl;
		gout << "     urgency = " << urgency << endl;
		gout << "     targLane = " << rightLane << endl;
		gout << "     leftLaneChange = " << isLeftLaneChange << endl;
	}
#endif

	//
	// Set lane change condition paramteres.
	//
	cond.SetActive( true );
	cond.SetUrgency( urgency );
	cond.SetTargLane( rightLane );
	cond.SetLeftLaneChange( isLeftLaneChange );
	cond.SetIsForcedLaneOffset( false );
	cond.SetSkipSignal( false );

}  // end of CheckNonPassingLane

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Check all lane change conditions.
//
// Remarks: Updates the m_conditions private member with the results.
//
// Arguments:
//   cInfo  - Vehicle information.
//   currentFrame - The simulation's current frame of execution.
//   freeDriveTargVel - The target velocity according to FreeDrive.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLaneChangeConds::CheckAllConditions( 
			const CAdoInfo& cInfo, 
			long currentFrame,
			double freeDriveTargVel
			)
{
	bool noCved = m_pCved == NULL;
	if( noCved ) 
	{
		cerr << "CLaneChangeConds: no valid pointer to Cved...exit" << endl;
		return;
	}

	bool noRootCollection = m_pRootCollection == NULL;
	if( noRootCollection ) 
	{
		cerr << "CLaneChangeConds: no valid pointer to HcsmCollection...exit";
		cerr << endl;
		return;
	}

	CheckExternalCommand( cInfo, m_conditions[eLC_EXTERNAL_COMMAND] );

	// no lane changes in the first couple of seconds after creation
	double ageTime = cInfo.m_ageFrame * cInfo.m_timeStepDuration;
	bool waitForInitDelay = ageTime < cInfo.m_lcInfo.initDelay;
	if( waitForInitDelay )  return;

	// don't do lane changes until status is okay to do so
	bool disableAllLaneChanges = cInfo.m_lcStatus != cLC_STATUS_DO_LC;
	if( disableAllLaneChanges )  return;

	CheckPathGuidance( cInfo, m_conditions[eLC_PATH_GUIDANCE] );
	if( cInfo.m_lcInfo.enableLosingCorridor )
	{
		CheckLosingCorridor( cInfo, m_conditions[eLC_LOSING_CORRIDOR] );
	}

	bool checkNonEssentialLaneChanges = (
				cInfo.m_lcInhibitCount < 0 && 
				cInfo.m_maintainGap.m_objId < 0
				);
	if( checkNonEssentialLaneChanges )
	{
		if( cInfo.m_lcInfo.enableNonPassingLane )
		{
			CheckNonPassingLane( 
						cInfo, 
						m_conditions[eLC_NONPASSING_LANE],
						currentFrame
						);
		}
		if( cInfo.m_lcInfo.enableSlowMovingObject )
		{
			CheckSlowVehicle( 
						cInfo, 
						m_conditions[eLC_SLOW_VEHICLE], 
						currentFrame,
						freeDriveTargVel
						);
		}
		if( cInfo.m_lcInfo.enableVerySlowMovingObject )
		{
			CheckVerySlowVehicle( 
						cInfo, 
						m_conditions[eLC_VERY_SLOW_VEHICLE], 
						currentFrame,
						freeDriveTargVel
						);
		}
		if( cInfo.m_lcInfo.enableHighwayMerge )
		{
			CheckHighwayMerge( cInfo, m_conditions[eLC_HIGHWAY_MERGE] );
		}
		if( cInfo.m_lcInfo.enableAvoidMergingObject )
		{
			CheckToAvoidMergingVehicle( 
						cInfo, 
						m_conditions[eLC_AVOID_MERGING_VEHICLE] 
						);
		}
	}
}  // end of CheckAllConditions


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Checks to see if a condition exists for an abort to the
//   current lane change.
//
// Remarks: 
//
// Arguments:
//   vehInfo       - Vehicle information.
//   externalAbort - An external request for an abort.
//
// Returns:  A boolean indicating if abort is necessary.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLaneChangeConds::AbortCondition( 
	const CAdoInfo& vehInfo, 
	bool externalAbort
	)
{

	
	return false;

}  // end of AbortCondition


//////////////////////////////////////////////////////////////////////////////
//
// Description:	Checks if there is enough room in the target lane 
//	when the vehicle needs to make a lane change.
// Remarks: 
//
// Arguments:	
//	cInfo - Vehicle information.
//	cTtcBack - Time to collision threshold for back.
//	cTtcFront - Time to collision threshold for front.
//	cDistBack - Bumper-to-bumper distance threshold back.
//	cDistFront - Bumper-to-bumper distance threshold front.
//	cTargLane - The lane that the vehicle wants to change to.
//
// Returns:  true or false.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLaneChangeConds::GapAccept( 
			const CAdoInfo& cInfo,
			const double& cTtcBack,
			const double& cTtcFront,
			const double& cDistBack,
			const double& cDistFront,
			const CLane& cTargLane
			)
{

#ifdef DEBUG_GAP_ACCEPT
	int objId = cInfo.m_pObj->GetId();
	bool debugThisObj = objId == DEBUG_GAP_ACCEPT;
	if( debugThisObj ) 
	{
		gout << "[" << objId  << "] ";
		gout << "==== LANECHANGE GapAccept ==========" << endl;
		gout << "  ttcBack = " << cTtcBack << "   ttcFront = " << cTtcFront;
		gout << endl;
		gout << "  distBack = " << cDistBack << "   distFront = ";
		gout << cDistFront << endl;
	}
#endif 
	
	//
	// If there are no objects behind the ADO in the target lane, 
	// then check for objects in front of the obj in the target lane.
	//
	if( cInfo.m_objsBack2.size() > 0 )
	{

		// 
		// Find the closest vehicle behind the ADO in the target lane.	
		//
		vector<CCved::TObjListInfo>::const_iterator i;
		vector<CCved::TObjListInfo> backObjsInTargLn;
		for( i = cInfo.m_objsBack2.begin(); i != cInfo.m_objsBack2.end(); i++ )
		{
			const CDynObj* pBackObj = m_pCved->BindObjIdToClass( i->objId );
			if( !pBackObj )
			{
				cerr << "Ado::GapAccept: unable to get pointer to object ";
				cerr << i->objId << " in objsBack2 list" << endl;
				continue;
			}
			
			CHcsm* pBackHcsm = m_pRootCollection->GetHcsm( pBackObj->GetHcsmId() );
			if( !pBackHcsm ) 
			{
				cerr << "Ado::GapAccept: cannot get pointer to back obj hcsm ";
				cerr << "(objId = " << i->objId << ")" << endl;
				continue;
			}

			CRoadPos backObjRoadPos;
			if( !pBackHcsm->GetMonitorByName( "RoadPos", &backObjRoadPos ) ) 
			{
				cerr << "Ado::GapAccept: failed to get back object's (id = ";
				cerr << i->objId << ") road position";
				cerr << endl;

				continue;
			}
            if (!backObjRoadPos.IsValid()){
				cerr << "Ado::GapAccept: get back object (id = ";
				cerr << i->objId << ") returned a invalid road position";
				cerr << endl;

				continue;                
            }
			if( backObjRoadPos.IsRoad() )
			{
				//
				// If on road, insert all the objects in the 
				// target lane in a vector.
				//
				if( backObjRoadPos.GetLane() == cTargLane )
				{
					backObjsInTargLn.push_back( *i );
				}		
			}else{
                vector< pair<int,double> > crds;
                backObjRoadPos.GetCorridors(crds);
                for (auto itr =  crds.begin() ;  itr != crds.end(); ++itr){
                    auto idx = backObjRoadPos.GetIntrsctn().GetCrdrIdx();
                    CVED::CCrdr crdr(*m_pCved,itr->first);
                    if (crdr.GetDstntnLn() == cTargLane){
                        backObjsInTargLn.push_back( *i );
                        break;
                    }
                }
            }
		
		}
		
		//
		// Since in the main back list, the objects are already sorted 
		// by distances between the object coming from behind and the lead
		// object, the closest object behind the ADO in the target lane
		// is the first element of the vector that has all the objects 
		// in the target lane.
		//

		// 
		// If there are objects in the target lane, then check bumper 
		// distance and time-to-collision value.If there is no obj behind 
		// the ADO in the target lane, then check if there are objs in front 
		// of the ADO in the target lane.
		//
		if( backObjsInTargLn.size() > 0 )
		{
			int closestObj = backObjsInTargLn[0].objId;				
			double dist = backObjsInTargLn[0].distFromOwner;
					
#ifdef DEBUG_GAP_ACCEPT
			if( debugThisObj ) 
			{
				gout << "  closest backObj = " << closestObj << endl;
				gout << "  backObj dist = ";
				gout << dist << " ft" << endl;
			}
#endif

			//
			// Calculate bumper-to-bumper distance, which can be obtained by
			// subtracting the proper lengths of the objects from the distance
			// between the centers of the objects.
			//
			double backObjLength;
			const CDynObj* pObj = m_pCved->BindObjIdToClass( closestObj );
			if( !pObj->IsValid() )
			{
				return false;
			}
			backObjLength = pObj->GetXSize();
			double bumperDist = (
						dist - 
						( cInfo.m_objLength * 0.5 ) - 
						( backObjLength * 0.5 )
						);

#ifdef DEBUG_GAP_ACCEPT
			if( debugThisObj ) 
			{
				gout << "  backObj bumperDist = " << bumperDist << endl;
			}
#endif
			
			bool noProperGap = bumperDist < cDistBack;
			if( noProperGap )
			{
#ifdef DEBUG_GAP_ACCEPT
				if( debugThisObj ) 
				{
					gout << "  not enough space to backObj...return false";
					gout << endl;
				}
#endif
				return false;
			}
			
			//
			// Calculate the velocity difference.
			//
			double ownerObjVel = cInfo.m_currVel;
			double backObjVel = pObj->GetVelImm();
			double velDiff = ownerObjVel - backObjVel;

#ifdef DEBUG_GAP_ACCEPT
			if( debugThisObj ) 
			{
				gout << "  myVel = " << ownerObjVel;
				gout << "  backObjVel = " << backObjVel << endl;
				gout << "  velDiff = " << velDiff << endl;
			}
#endif

			// check gap.
			if( velDiff < 0 )
			{ 
				double ttc;
				bool almostZeroVelDiff = fabs( velDiff ) < cNEAR_ZERO;
				if( almostZeroVelDiff )
				{
					ttc = 1000.0;
				}
				else
				{
					ttc = ( bumperDist*cFEET_TO_METER ) / fabs( velDiff );
				}

#ifdef DEBUG_GAP_ACCEPT
				if( debugThisObj ) 
				{
					gout << "  backObj ttc = " << ttc << endl;
				}
#endif

				bool tooClose = ttc < cTtcBack;
				if( tooClose )
				{
#ifdef DEBUG_GAP_ACCEPT
					if( debugThisObj ) 
					{
						gout << "  will collide with obj behind...return false";
						gout << endl;
					}
#endif

					return false;
				}
			}
		}

	}

	
	//
	// If no vehicle in front of the ADO in the target lane, then there
	// is proper gap to make a lane change.
	//
	if( cInfo.m_objsFwd.size() < 1 )
	{
#ifdef 	DEBUG_GAP_ACCEPT
		if( debugThisObj ) 
		{
			gout << "  no object in front...return true" << endl;
		}
#endif
		return true;
	}
	
	//
	// Find the closest object in front of the ADO in the 
	// target lane.
	//
	vector<CCved::TObjListInfo>::const_iterator j;
	vector<CCved::TObjListInfo> fwdObjsInTargLnCrdr;
	for( j = cInfo.m_objsFwd.begin(); j !=  cInfo.m_objsFwd.end(); j++ )
	{	
		const CDynObj* pFwdObj = m_pCved->BindObjIdToClass( j->objId );
		if( !pFwdObj )
		{
			cerr << "Ado::GapAccept: unable to get pointer to object ";
			cerr << j->objId;
			cerr << " in objsFwd list while looking on target lane";
			cerr << endl;

			continue;
		}
		
		CHcsm* pFwdHcsm = m_pRootCollection->GetHcsm( pFwdObj->GetHcsmId() );
		if( !pFwdHcsm ) 
		{
			cerr << "Ado::GapAccept: cannot get pointer to fwd hcsm ";
			cerr << "(objId = " << j->objId << ")" << endl;
			continue;
		}

		CRoadPos fwdObjRoadPos;
		if( !pFwdHcsm->GetMonitorByName( "RoadPos", &fwdObjRoadPos ) ) 
		{
			cerr << "Ado::GapAccept: failed to get Fwd object's (id = ";
			cerr << j->objId << ") road position";
			cerr << endl;

			continue;
		}

		if( fwdObjRoadPos.IsRoad() )
		{
			//
			// If on road, insert all the objects in the 
			// target lane in a vector.
			//
			if( fwdObjRoadPos.GetLane() == cTargLane )
			{
				fwdObjsInTargLnCrdr.push_back( *j );

			}
		}
		else	
		{
			// This is to ensure that an forward obj in the target lane 
			// will be seen even when it is on an intersection, by the Ado
			// attempting to make a lane change.
			if( fwdObjRoadPos.GetCorridor().GetSrcLn() == cTargLane )
			{
				fwdObjsInTargLnCrdr.push_back( *j );
			}
		}
	}
	

	//
	// Since in the main forward list, the objects are already sorted 
	// by distances between the objects, the closest object in front of
	// the ADO in the target lane or corridor is the first element of 
	// the vector that has all the objects in the target lane or corridor.
	//

	//
	// If there is no obj in front in the target lane or corridor, return.
	//
	if( fwdObjsInTargLnCrdr.size() == 0 )
	{

#ifdef DEBUG_GAP_ACCEPT
		if( debugThisObj ) 
		{
			gout << "  no fwd obj in target lane or corridor...return true";
			gout << endl;
		}
#endif
		return true;
	}
	int closestFwdObj = fwdObjsInTargLnCrdr[0].objId;				
	double distOfFwdObjs = fwdObjsInTargLnCrdr[0].distFromOwner;
				
#ifdef DEBUG_GAP_ACCEPT
	if( debugThisObj ) 
	{
		gout << "  the closest forward obj is " << closestFwdObj << endl;
		gout << "  fwdObj center dist = " << distOfFwdObjs << " ft";
		gout << endl;
	}
#endif

	//
	// Calculate bumper-to-bumper distance, which can be obtained by
	// subtracting the proper lengths of the objects from the distance
	// between the centers of the objects.
	//
	double fwdObjLength;
	const CDynObj* pObjFwd = m_pCved->BindObjIdToClass( closestFwdObj );
	if( !pObjFwd->IsValid() )
	{
		return false;
	}
	fwdObjLength = pObjFwd->GetXSize();
	double bumperDistOfFwdObjs = distOfFwdObjs - ( cInfo.m_objLength * 0.5 ) - 
								( fwdObjLength * 0.5 ) ;

#ifdef DEBUG_GAP_ACCEPT
	if( debugThisObj ) 
	{
		gout << "  fwdObj bumper dist = " << bumperDistOfFwdObjs;
		gout << endl;
	}
#endif

	// No proper gap.
	bool noProperGap = bumperDistOfFwdObjs < cDistFront;
	if( noProperGap )
	{
#ifdef DEBUG_GAP_ACCEPT
		if( debugThisObj ) 
		{
			gout << "  no proper gap because fwdObj bumper dist is < distThreshold, return false" << endl;
			gout << " distOfFwdObjs = " << distOfFwdObjs << endl;
			gout << " bumperDistOfFwdObjs = " << bumperDistOfFwdObjs << endl;		

		}
#endif
		return false;
	}

	//
	// Calculate the velocity difference.
	//
	double ownerFwdObjVel = cInfo.m_currVel;
	double fwdObjVel = pObjFwd->GetVelImm();
	double velDiff = ownerFwdObjVel - fwdObjVel;

#ifdef DEBUG_GAP_ACCEPT
	if( debugThisObj ) 
	{
		gout << "  myVel = " << ownerFwdObjVel;
		gout << "  backObjVel = " << fwdObjVel << endl;
		gout << "  velDiff = " << velDiff << endl;
	}
#endif

	// check gap.
	if( velDiff > 0 )
	{
		double divisor_2 = fabs ( velDiff );
		double ttcFwd;
		if( divisor_2 > cNEAR_ZERO )
		{
			ttcFwd = ( bumperDistOfFwdObjs * cFEET_TO_METER ) / divisor_2;
		}
		else
		{
			ttcFwd = 1000.0;
		}

#ifdef DEBUG_GAP_ACCEPT
		if( debugThisObj ) 
		{
			gout << "  fwdObj ttc = " << ttcFwd << endl;
		}
#endif

		bool tooClose = ttcFwd < cTtcFront;
		if( tooClose )
		{
#ifdef DEBUG_GAP_ACCEPT
			if( debugThisObj ) 
			{
				gout << "  will collide with fwd obj...return false";
				gout << endl;
			}
#endif

			return false;
		}
	}

#ifdef DEBUG_GAP_ACCEPT
	if( debugThisObj ) 
	{
		gout << "  reached end...return true" << endl;
	}
#endif

	return true;

}	// end of GapAccept


//////////////////////////////////////////////////////////////////////////////
//
// Description:	Checks if there is enough room in the given corridor
//   behind the owner.
//
// Remarks: 
//
// Arguments:	
//	cInfo - Vehicle information.
//	ttcThreshold - Time to collision threshold.
//	distThreshold - Bumper distance threshold.
//	targCorridor - The corridor that the vehicle wants to change to.
//
// Returns:  true or false.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLaneChangeConds::GapAcceptBackObjsOnCrdr( 
			const CAdoInfo& cInfo,
			const double ttcThreshold,
			const double distThreshold,
			const CCrdr& targCorridor
			)
{
	// 
	// Find the closest vehicle behind the ADO in the target corridor.	
	//
	vector<CCved::TObjListInfo>::const_iterator i;
	vector<CCved::TObjListInfo> backObjsInTargCrdr;
	for( i = cInfo.m_objsBack2.begin(); i != cInfo.m_objsBack2.end(); i++ )
	{

		const CDynObj* pBackObj = m_pCved->BindObjIdToClass( i->objId );
		if( !pBackObj )
		{
			cerr << "Ado::GapAcceptBackObjsOnCrdr: ";
			cerr << "unable to get pointer to object ";
			cerr << i->objId << " in backObj list" << endl;

			continue;
		}
		
		CHcsm* pBackHcsm = m_pRootCollection->GetHcsm( pBackObj->GetHcsmId() );
		if( !pBackHcsm ) 
		{
			cerr << "Ado::GapAcceptBackObjsOnCrdr: ";
			cerr << "cannot get pointer to back obj hcsm ";
			cerr << "(objId = " << i->objId << ")" << endl;

			continue;
		}

		CRoadPos backObjRoadPos;
		if( !pBackHcsm->GetMonitorByName( "RoadPos", &backObjRoadPos ) ) 
		{
			cerr << "Ado::GapAcceptBackObjsOnCrdr: ";
			cerr << "failed to get back object's (id = ";
			cerr << i->objId << ") road position";
			cerr << endl;

			continue;
		}

		if( !backObjRoadPos.IsRoad() )
		{
			//
			// If on intersection, insert all the objects in the 
			// target corridor in a vector.
			//
			int backObjCrdrId = backObjRoadPos.GetCorridor().GetRelativeId();
			int targCrdrId = targCorridor.GetRelativeId();
			int backObjIntrsctnId = backObjRoadPos.GetIntrsctn().GetId();
			int targCrdrIntrsctnId = targCorridor.GetIntrsctn().GetId();
			bool objOnTargCrdr = (
						backObjIntrsctnId == targCrdrIntrsctnId &&
						backObjCrdrId == targCrdrId
						);
			if( objOnTargCrdr )
			{
				backObjsInTargCrdr.push_back( *i );
			}		
		}
	
	}

	//
	// Since in the main back obj list, the objects are already sorted 
	// by distances between the approaching object and the approached 
	// object, the closest object behind the ADO in the target corridor
	// is the first element of the vector that has all the objects in the 
	// target corridor.
	//

	// 
	// If there are objects in the target corridor, then check bumper 
	// distance and time-to-collision value.If there is no obj behind 
	// the ADO in the target corridor, then check if there are objs in front 
	// of the ADO in the target corridor.
	//
	int currElem;
	for( currElem = 0; currElem < backObjsInTargCrdr.size(); currElem++ )
	{
		int closestObj = backObjsInTargCrdr[currElem].objId;				
		double dist = backObjsInTargCrdr[currElem].distFromOwner;
				
#ifdef DEBUG_GAP_ACCEPT
		if ( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
		{
			gout << "  The closest obj is: " << closestObj << endl;
			gout << "  The distance between the center of the objects is: ";
			gout << dist << " ft" << endl;
		}
#endif

		//
		// Calculate bumper-to-bumper distance, which can be obtained by
		// subtracting the proper lengths of the objects from the distance
		// between the centers of the objects.
		//
		double backObjLength;
		const CDynObj* pObj = m_pCved->BindObjIdToClass( closestObj );
		if( !pObj->IsValid() )  continue;

		backObjLength = pObj->GetXSize();
		double bumperDist = (
					dist - 
					( ( cInfo.m_objLength * 0.5 ) + ( backObjLength * 0.5 ) )
					);

		// No proper gap.
		bool haveEnoughGap = bumperDist > distThreshold;
		if( !haveEnoughGap )
		{
#ifdef DEBUG_GAP_ACCEPT
			if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
			{
				gout << "The bumper distance to the object behind the ADO is: ";
				gout << bumperDist << endl;
				gout << "Not sufficient bumper distance to obj behind. " << endl;
			}
#endif
			return false;
		}

		//
		// Calculate the velocity difference.
		//
		double ownerObjVel = cInfo.m_currVel;
		double backObjVel = pObj->GetVelImm();
		double velDiff = ownerObjVel - backObjVel;

#ifdef DEBUG_GAP_ACCEPT
		if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
		{
			gout << "The owner obj has this velocity: " << ownerObjVel << endl;
			gout << "The apprch obj has this velocity: "<< backObjVel << endl;
			gout  << "The vel difference to the obj behind is: " << velDiff << endl;
		}		
#endif

		// check gap.
		if( velDiff < 0 )
		{ 
			double divisor = fabs( velDiff );
			double ttc;
			if( divisor > cNEAR_ZERO )
			{
				ttc = ( bumperDist*cFEET_TO_METER ) / divisor;
#ifdef DEBUG_GAP_ACCEPT
				if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
				{
					gout << "The ttc for obj behind is: " << ttc << endl;
				}
#endif
			}
			else
			{
				ttc = 1000.0;
#ifdef DEBUG_GAP_ACCEPT
				if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
				{
					gout << "The ttc for obj behind is: "<< ttc << endl;
				}
#endif
			}
			if( ttc < ttcThreshold )
			{
#ifdef DEBUG_GAP_ACCEPT
				if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
				{
					gout << "Will collide with obj behind. " << endl;
				}
#endif

				return false;
			}
		}

		return true;
	}

	return true;
}  // GapAcceptBackObjsOnCrdr


//////////////////////////////////////////////////////////////////////////////
//
// Description:	Checks if there is enough room in the given corridor in.
//   front of the owner.
//
// Remarks: 
//
// Arguments:	
//	cInfo - Vehicle information.
//	ttcThreshold - Time to collision threshold.
//	distThreshold - Bumper distance threshold.
//	targCorridor - The corridor that the vehicle wants to change to.
//
// Returns:  true or false.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLaneChangeConds::GapAcceptFwdObjsOnCrdr( 
			const CAdoInfo& cInfo,
			const double ttcThreshold,
			const double distThreshold,
			const CCrdr& targCorridor
			)
{
	//
	// Find the closest object in front of the ADO in the target corridor.
	//
	vector<CCved::TObjListInfo>::const_iterator i;
	vector<CCved::TObjListInfo> fwdObjsInTargCrdr;
	for( i = cInfo.m_objsFwd.begin(); i !=  cInfo.m_objsFwd.end(); i++ )
	{	

		const CDynObj* pFwdObj = m_pCved->BindObjIdToClass( i->objId );
		if( !pFwdObj )
		{
			cerr << "Ado::GapAcceptFwdObjsOnCrdr: ";
			cerr << "unable to get pointer to object " << i->objId;
			cerr << " in objsFwd list while looking on target corridor";
			cerr << endl;

			continue;
		}
		
		CHcsm* pFwdHcsm = m_pRootCollection->GetHcsm( pFwdObj->GetHcsmId() );
		if( !pFwdHcsm ) 
		{
			cerr << "Ado::GapAcceptFwdObjsOnCrdr: ";
			cerr << "cannot get pointer to fwd obj hcsm ";
			cerr << "(objId = " << i->objId << ")" << endl;

			continue;
		}

		CRoadPos fwdObjRoadPos;
		if( !pFwdHcsm->GetMonitorByName( "RoadPos", &fwdObjRoadPos ) ) 
		{
			cerr << "Ado::GapAcceptFwdObjsOnCrdr: ";
			cerr << "failed to get fwd object's (id = ";
			cerr << i->objId << ") road position";
			cerr << endl;

			continue;
		}

		if( !fwdObjRoadPos.IsRoad() )
		{
			//
			// If on intersection, insert all the objects in the 
			// target corridor in a vector.
			//
			// If on intersection, insert all the objects in the 
			// target corridor in a vector.
			//
			int fwdObjCrdrId = fwdObjRoadPos.GetCorridor().GetRelativeId();
			int targCrdrId = targCorridor.GetRelativeId();
			int fwdObjIntrsctnId = fwdObjRoadPos.GetIntrsctn().GetId();
			int targCrdrIntrsctnId = targCorridor.GetIntrsctn().GetId();

			bool objOnTargCrdr = (
						fwdObjIntrsctnId == targCrdrIntrsctnId &&
						fwdObjCrdrId == targCrdrId
						);
			if( objOnTargCrdr )
			{
				fwdObjsInTargCrdr.push_back( *i );
			}		
		}
	}
		
	//
	// Since in the main forward list, the objects are already sorted 
	// by distances between the objects, the closest object in front of
	// the ADO in the target corridor is the first element of the vector
	// that has all the objects in the target corridor.
	//

	//
	// If there is no obj in front in the target corridor, return.
	//
	int currElem;
	for( currElem = 0; currElem < fwdObjsInTargCrdr.size(); currElem++ )
	{
		int closestFwdObj = fwdObjsInTargCrdr[currElem].objId;				
		double distOfFwdObjs = fwdObjsInTargCrdr[currElem].distFromOwner;
					
#ifdef DEBUG_GAP_ACCEPT
		if ( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
		{
			gout << "  The closest forward obj is: " << closestFwdObj << endl;
			gout << "  The distance between the center of the objects is: ";
			gout << distOfFwdObjs << " ft" << endl;
		}
#endif

		//
		// Calculate bumper-to-bumper distance, which can be obtained by
		// subtracting the proper lengths of the objects from the distance
		// between the centers of the objects.
		//
		double fwdObjLength;
		const CDynObj* pObjFwd = m_pCved->BindObjIdToClass( closestFwdObj );
		if( !pObjFwd->IsValid() )  continue;

		fwdObjLength = pObjFwd->GetXSize();
		double bumperDist = (
						distOfFwdObjs - 
						( ( cInfo.m_objLength * 0.5 ) + ( fwdObjLength * 0.5 ) )
						);

		// No proper gap.
		bool haveProperGap = bumperDist > distThreshold;
		if( !haveProperGap )
		{
#ifdef DEBUG_GAP_ACCEPT
			if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
			{
				gout << "The bumper distance to the forward obj is: ";
//				gout << bumperDistOfFwdObjs << endl;
				gout << "bumper distance not sufficient to the forward obj.";
				gout << endl;
			}
#endif
			return false;
		}

		//
		// Calculate the velocity difference.
		//
		double ownerFwdObjVel = cInfo.m_currVel;
		double fwdObjVel = pObjFwd->GetVelImm();
		double velDiffOfFwdObjs = ownerFwdObjVel - fwdObjVel;

#ifdef DEBUG_GAP_ACCEPT
		if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
		{
			gout << "The forward owner obj has this velocity: " << ownerFwdObjVel << endl;
			gout << "The obj in front has this velocity: " << fwdObjVel << endl;
			gout << "The vel diff is: " << velDiffOfFwdObjs <<endl;
		}
#endif

		// check gap.
		if( velDiffOfFwdObjs < 0 )
		{
			double divisor_2 = fabs ( velDiffOfFwdObjs );
			double ttcFwd;
			if( divisor_2 > cNEAR_ZERO )
			{
				ttcFwd = ( bumperDist * cFEET_TO_METER ) / divisor_2;
			}
			else
			{
				ttcFwd = 1000.0;
			}
			if( ttcFwd < ttcThreshold )
			{

	#ifdef DEBUG_GAP_ACCEPT
				if( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
				{
					gout << "The ttc to forward obj is: "<< ttcFwd << endl;
					gout << "Will collide with obj in front." << endl;
				}
	#endif

				return false;
			}
		}

		return true;
	}

	return true;
}  // GapAcceptFwdObjsOnCrdr

	
//////////////////////////////////////////////////////////////////////////////
//
// Description:	Checks if there is enough room in the target corridor
//	when the vehicle needs to make a lane change.
// Remarks: 
//
// Arguments:	
//	cInfo - Vehicle information.
//	ttcThreshold - Time to collision threshold.
//	distThreshold - Bumper distance threshold.
//	targCorridor - The corridor that the vehicle wants to change to.
//
// Returns:  true or false.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLaneChangeConds::GapAccept( 
			const CAdoInfo& cInfo,
			const double ttcThreshold,
			const double distThreshold,
			const CCrdr& targCorridor
			)
{
#ifdef DEBUG_GAP_ACCEPT
	if ( cInfo.m_pObj->GetId() == DEBUG_GAP_ACCEPT ) 
	{
		gout << "[" << cInfo.m_pObj->GetId()  << "] ";
		gout << "==== LANECHANGE GapAccept ==========" << endl;
	}
#endif 

	bool backSuccess = GapAcceptBackObjsOnCrdr( 
								cInfo,
								ttcThreshold,
								distThreshold,
								targCorridor 
								);

	bool fwdSuccess = GapAcceptFwdObjsOnCrdr(
								cInfo,
								ttcThreshold,
								distThreshold,
								targCorridor 
								);

	return backSuccess && fwdSuccess;
}	// end of GapAccept
