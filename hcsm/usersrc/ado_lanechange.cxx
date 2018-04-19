/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: ado_lanechange.cxx,v 1.85 2015/12/16 18:46:59 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad, Sunil Bulusu
 *
 * Date:    February, 2000
 * 
 * Description:  Contains code for the LaneChange HCSM.
 *
 ****************************************************************************/

#include "hcsmpch.h"
#include <math.h>

#include <pi_iostream>
#include <stack>
#include "ado_lc_data.h"
#include "controllers.h"

using namespace std;

//
// Debugging macros.
//

#undef DEBUG_LANECHANGE       //2  // needs CVED id
#undef DEBUG_CALCTOTALLATDIST //2   //needs CVED id
#undef DEBUG_OUTPUT		      // Outputting the Velocity, Urgency, Dist and Time 
							  // to a file 
#undef DEBUG_LcEXECUTE_STATES 	//


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the minimum lateral offset given a velocity.
//
// Remarks:  
//
// Arguments:
//   vel - The vehicle's velocity.
//
// Returns:  A double that represents the minimum lateral offset.
//
//////////////////////////////////////////////////////////////////////////////
static
double CalcMinLatOffset( double vel )
{
	double minOffset = 6.3356 * pow( vel * cMS_TO_MPH, -0.5306 );
	return minOffset;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the maximum lateral offset given a velocity.
//
// Remarks:  
//
// Arguments:
//   vel - The vehicle's velocity.
//
// Returns:  A double that represents the maximum lateral offset.
//
//////////////////////////////////////////////////////////////////////////////
static
double CalcMaxLatOffset( double vel,double max )
{
//	double maxOffset = 6.4286 * pow( vel * cMS_TO_MPH, -0.3266 );
	double maxOffset = max * pow( vel * cMS_TO_MPH, -0.3266 );
	return maxOffset;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the delta offset given the urgency.
//
// Remarks:  
//
// Arguments:
//   minOffset - The minimum lateral offset.
//   maxOffset - The maximum lateral offset.
//   urgency   - The urgency should be a value between 0.0 and 1.0.
//
// Returns:  A double that represents the delta offset.
//
//////////////////////////////////////////////////////////////////////////////
static
double CalcDeltaOffsetFromUrgency( 
			double minOffset, 
			double maxOffset, 
			double urgency 
			)
{
	double deltaOffset = minOffset + ( ( maxOffset - minOffset ) * urgency );
	return deltaOffset;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the curvature of the road given currRoadPos
//
// Remarks:  
//
// Arguments:
//   pI				- Pointer to vehicle information.
//   currRoadPos	- current road position.
//   angle			- referene to angle
//
// Returns:  bool 
//
//////////////////////////////////////////////////////////////////////////////
static bool 
CalcRoadAngle(
			CRoadPos currRoadPos, 
			const CRoadPos& targRoadPos, 
			double& roadAngle
			)
{
	bool foundAngle = false;

	//currRoadPos.SetOffset(0.0);
	CPoint3D currentPos	= currRoadPos.GetXYZ();
	if (!currRoadPos.IsValid())
	{
		gout << "CurrPos not valid " << endl;
		return false;
	}

	// From newTargRoadPos
	CPoint3D targetPos	= targRoadPos.GetVeryBestXYZ();
	
	// Tangent to the curve.
	CVector3D currTangent( currRoadPos.GetTangentInterpolated() );

	// Vector representing the vehicle direction 
	CVector3D vehicleDir(
		targetPos.m_x - currentPos.m_x, 
		targetPos.m_y - currentPos.m_y, 
		targetPos.m_z - currentPos.m_z
		);
	
	if( !(vehicleDir.Normalize() == 0.0 || currTangent.Normalize() == 0.0 ) )
	{
		double dotProduct =  currTangent.DotP( vehicleDir );
		double lengthProduct = currTangent.Length()* vehicleDir.Length();
		if( dotProduct / lengthProduct > 1.0 )
		{
			foundAngle = false;
		}
		else 
		{
			roadAngle = acos( dotProduct / lengthProduct );
			foundAngle = true;
		}
	}

	return foundAngle;
}
	


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Signals a lane change for an ADO.
//
// Remarks:  Sets the visual state of the ADO object.
//
// Arguments:
//   pI - Pointer to vehicle information.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void
SignalLaneChange( CAdoInfoPtr pI )
{

	if( pI->m_pCurrLcCond->IsLeftLaneChange() )
	{	
		// making a left turn
		pI->m_lightState |= cCV_LEFT_TURN_SIGNAL;
	}
	else 
	{
		// making a right turn
		pI->m_lightState |= cCV_RIGHT_TURN_SIGNAL;
	}

}  // end of SignalLaneChange


void
CLaneChange::NullifyOutputs()
{

	m_outputTargPos.SetNoValue();

}  // end of NullifyOutputs

void 
CLaneChange::Creation()
{

	m_conditions.SetCved( cved );
	m_conditions.SetHcsmCollection( m_pRootCollection );

}  // end of LaneChangeCreation


void 
CLaneChange::Deletion()
{

}  // end of LaneChangeCreation


void 
CLaneChange::PreActivity()
{
	
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LANECHANGE PRE-ACTIVITY ==============" << endl;
	}
#endif

	//
	// Set my children's inputs.
	//
	SetInputpIForLcMonitor( pI );
	SetInputpIForLcSignal( pI );
	SetInputpIForLcExecute( pI );
	SetInputpIForLcAbort( pI );

	SetInputpCondForLcMonitor( &m_conditions );
	SetInputpCondForLcSignal( &m_conditions );
	SetInputpCondForLcExecute( &m_conditions );
	SetInputpCondForLcAbort( &m_conditions );

	if( HasValueInputFreeDriveTargVel() )
	{
		//
		// Getting the Target Velocity from the FreeDrive behavior as Input
		//
		double freeDriveTargVel = GetInputFreeDriveTargVel();
		SetInputFreeDriveTargVelForLcMonitor( freeDriveTargVel );
	}

}  // end of LaneChangePreActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LaneChange HCSM post-activity function.
//
// Remarks:  This function is the LaneChange HCSM's post-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLaneChange::PostActivity()
{

	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LANECHANGE POST-ACTIVITY ==============" << endl;
	}
#endif


	//
	// Reset the outputs.
	//
	m_outputTargPos.SetNoValue();
	m_outputTargAccel.SetNoValue();

	if ( m_activeChild->GetName() == "LcExecute" ) 
	{
		if ( HasValueOutputTargPosFromLcExecute() ) 
		{
			SetOutputTargPos( GetOutputTargPosFromLcExecute() );
		}

		if ( HasValueOutputTargAccelFromLcExecute() ) 
		{
			SetOutputTargAccel( GetOutputTargAccelFromLcExecute() );	
		}
	}

#ifdef DEBUG_LANECHANGE
	if( debugThisObj )
	{
		gout << "current track lane = ";
		if( pI->m_trackLane.IsValid() )
		{	
			gout << pI->m_trackLane << endl;
		}
		else 
		{
			gout << "INVALID" << endl;
		}
	}
#endif

}  // end of LaneChangePostActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcMonitor HCSM pre-activity function.
//
// Remarks:  This function is the LcMonitor HCSM's pre-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcMonitor::PreActivity()
{

	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();
	CLcCondsPtr pCond = GetInputpCond();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcMonitor PRE-ACTIVITY ==============" << endl;
	}
#endif

	//
	// Read the FreeDriveTargVel input.
	//
	double freeDriveTargVel = -1;
	if( HasValueInputFreeDriveTargVel() )
	{
		freeDriveTargVel = GetInputFreeDriveTargVel();
	}

	//
	// Check all of possibilities for a lane change now.
	//
	pCond->CheckAllConditions( *pI, GetFrame(), freeDriveTargVel );

}  // end of PreActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcMonitor HCSM post-activity function.
//
// Remarks:  This function is the LcMonitor HCSM's post-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcMonitor::PostActivity()
{

	//
	// Nothing so far.
	//

}  // end of PostActivity

//////////////////////////////////////////////////////////////////////////////
//
// Description: Checks all the Lc conditions to see if lanechange is reqd.
//				To be used in the forced Lane Offset situations. 
//
// Remarks: This function is used along with LcMonitorLcExecutePredicate
//
// Arguments:
//
// Returns: bool
//
//////////////////////////////////////////////////////////////////////////////

static bool 
MonitorLcConditionsForSkipSignalling(	
			CAdoInfoPtr pI,
			CLcCondsPtr pCond
			)
{

	bool fire = (
			pI->m_lcStatus == cLC_STATUS_DO_LC &&
			pCond->AnyConditionsActive()
			);

	if( fire )
	{
		//
		// Update past and current lane change conditions.
		//
		ELcCondition currCond = pCond->GetCurrentCondition( pI->m_pastLcCond );

		fire = ( currCond != eLC_NONE );
		if ( fire )  pI->m_pCurrLcCond = &( pCond->GetCondition( currCond ) );			
	}
	return fire;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Represents the transition from the LcMonitor HCSM
//   to LcExecute HCSM.
//
// Remarks:  This predicate function represents the transition from
//   LcMonitor HCSM to LcExecute HCSM.  
//
// Arguments:
//
// Returns:  A boolean indicating whether the transition fired or not.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLcMonitor::LcMonitorLcExecutePredicate()
{
	CAdoInfoPtr pI = GetInputpI();
	CLcCondsPtr pCond = GetInputpCond();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcMonitor --> LcExecute PREDICATE =============";
		gout << endl;
	}
#endif
	
	//
	// Checking if the SkipSignal condition
	//
	
	bool fire = 
		MonitorLcConditionsForSkipSignalling( pI, pCond ) 
		&& pI->m_pCurrLcCond->IsSkipSignal();

#ifdef DEBUG_LANECHANGE
	if( debugThisObj )
	{
		gout << "  fire = " << fire << endl;
	}
#endif

	return fire;

}	// end of LcMonitorLcExecutePredicate


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Represents the transition from the LcMonitor HCSM
//   to LcSignal HCSM.
//
// Remarks:  This predicate function represents the transition from
//   LcMonitor HCSM to LcSignal HCSM.  
//
// Arguments:
//
// Returns:  A boolean indicating whether the transition fired or not.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLcMonitor::LcMonitorLcSignalPredicate()
{

	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();
	CLcCondsPtr pCond = GetInputpCond();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcMonitor --> LcSignal PREDICATE =============";
		gout << endl;
	}
#endif

	bool fire = (
			pI->m_lcStatus == cLC_STATUS_DO_LC &&
			pCond->AnyConditionsActive()
			);
/*	bool fire1 = pI->m_maintainGapObjId < 0;
	bool fire2 = pI->m_laneChangeStatus == 0;
	bool fire3 = pCond->AnyConditionsActive();

	gout << "== Maintain Gap obj ID: " << fire1 << endl;
	gout << "== Lane Change Status: " << fire2 << endl;
	gout << "== Any Conds: " << fire3 << endl;
*/
	if( fire )
	{
		//
		// Update past and current lane change conditions.
		//
		ELcCondition currCond = pCond->GetCurrentCondition( pI->m_pastLcCond );

		fire = ( currCond != eLC_NONE );
		if ( fire )  pI->m_pCurrLcCond = &( pCond->GetCondition( currCond ) );			
	}

#ifdef DEBUG_LANECHANGE
	if( debugThisObj)
	{
		gout << "  fire = " << fire << endl;
	}
#endif

	return fire;

}  // end of LcMonitorLcSignalPredicate




//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcSignal HCSM creation function.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcSignal::Creation()
{

	//
	// Initialize the signal counter.
	//
	m_signalFrame = 0;

}  // end of LcSignalCreation


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcSignal HCSM pre-activity function.
//
// Remarks:  This function is the LcSignal HCSM's pre-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcSignal::PreActivity()
{

	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();
	CLcCondsPtr pCond = GetInputpCond();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcSignal PRE-ACTIVITY ==============" << endl;
	}
#endif

	//
	// Signal intention to change lanes from current to target lane.
	//
	SignalLaneChange( pI );

	m_signalFrame++;

}  // end of LcSignal::PreActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcSignal HCSM post-activity function.
//
// Remarks:  This function is the LcSignal HCSM's post-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcSignal::PostActivity()
{

	//
	// Nothing so far.
	//

}  // end of LcSignal::PostActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Represents the transition from the LcSignal HCSM
//   to LcExecute HCSM.
//
// Remarks:  This predicate function represents the transition from
//   LcSignal HCSM to LcExecute HCSM.  
//
// Arguments:
//
// Returns:  A boolean indicating whether the transition fired or not.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLcSignal::LcSignalLcExecutePredicate()
{

	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcSignal --> LcExecute PREDICATE =============";
		gout << endl;
		gout << "  urgency = " << pI->m_pCurrLcCond->GetUrgency() << endl;
	}
#endif

	const double urgency = pI->m_pCurrLcCond->GetUrgency();
	
	//
	//
	// Signal time can be specified through Lane Change Dial or
	// randomly calculated turn signal time
	double signalTime = 0.0;
	if( pI->m_hasSignalTimeFromDial )
	{
		signalTime = pI->m_signalTimeFromDial; // Value from the dial
	}
	else
	{
		//
		// signal time does not change when urgency lies between 0.0 - 0.5
		// higher the urgency in the range ( 0.5 - 1.0 ), lower will be the
		// signal time
		//
		double turnSignalTime = pI->m_lcInfo.turnSignalTime;
		if( urgency < 0.5 )
		{
			signalTime = turnSignalTime;
		}
		else
		{
			signalTime = 2 * turnSignalTime * ( 1.0  - urgency );
		}
	}
	// final Check.
	if( signalTime < 0.0 ) signalTime = 0.0;
	
	double signaledSoFar = m_signalFrame * GetTimeStepDuration();
	bool fire = signaledSoFar >= signalTime;

	//
	// Reset signal counter if returning true.
	//
	if( fire )  m_signalFrame = 0;

#ifdef DEBUG_LANECHANGE

	if( debugThisObj )
	{
		gout << "  signaled so far time = " << signaledSoFar;
		gout << "  max signal time = " << signalTime << endl;
		gout << "  fire = " << fire << endl;
	}
#endif

	return fire;

}  // end of LcSignalLcExecutePredicate


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Represents the transition from the LcSignal HCSM
//   to LcAbort HCSM.
//
// Remarks:  This predicate function represents the transition from
//   LcSignal HCSM to LcAbort HCSM.  
//
// Arguments:
//
// Returns:  A boolean indicating whether the transition fired or not.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLcSignal::LcSignalLcAbortPredicate()
{

	//
	// ######   Needs to be implemented!!   ######
	//

	return false;

}  // end of LcSignalLcAbortPredicate

//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcExecute HCSM creation function.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcExecute::Creation()
{
	//
	// Initialize local variables.
	//
	m_firstFrame        = true;
	m_startLcFrame		= 0;
	m_maxRoadAngle		= 0.0;
    m_overShotLc        = false;
    m_currLookAheadTime = 1.0f;
	m_storeRoadAngle.clear();

}  // end of Creation


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the target road position for the lane change.
//
// Remarks:
//
// Arguments:
//   pI - Pointer to the vehicle info. structure.
//   targRoadPos - (output) Contains the target road position.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLcExecute::CalcTargRoadPos( CAdoInfoPtr& pI, CRoadPos& targRoadPos )
{
	//
	// Calculate a distance from the current position that points to
	// the next target position.
	//
	const double cSECONDS_AHEAD = 1.0;
	const double cMIN_DIST_AHEAD = 10.0;  //feet
	double currVel = pI->m_pObj->GetVelImm();
	double distForTargPos = ( currVel * m_currLookAheadTime * cMETER_TO_FEET);
    //our previous look ahead was only .32 seconds due to conversion error
    //we will want to presurve our same lc behavour so scenario's
    //will not need a large amount of tunning, so we will need to fade in
    //fade out our look ahead distance.
    double latDistRemaining;
    double minDist = cMIN_DIST_AHEAD + ( pI->m_objLength * 0.5 );
    double minLookahead = 0.5;
    if (pI->m_haveForcedLaneOffset)
     minLookahead = pI->m_forcedOffsetMinLookAhead;
 //   if( !m_firstFrame )
	//{
 //       latDistRemaining = pI->m_lcTotalLatDist - pI->m_lcLatDistTraveled;
 //       //fade in
 //       float progressScale =1.0f;// + max(0.25f, float(0.5 - minLookahead) * 4.0f);
 //       float progress = ( pI->m_lcLatDistTraveled / pI->m_lcTotalLatDist ) * (progressScale);
 //       if (progress < 0.25)
 //           distForTargPos = (distForTargPos * (1-minLookahead)) * (1-sin(progress*cPI*2)) + distForTargPos*minLookahead;
 //       else if (progress < 0.80)
 //            distForTargPos = distForTargPos*minLookahead;
 //       else
 //           distForTargPos = (distForTargPos * (1-minLookahead)) * sin(((progress-0.8)*2.5)*cPI) + distForTargPos*minLookahead;
 //       if( progress > 1.0 )  progress = 1.0;
 //   }
 //   
	
	if( distForTargPos < minDist )  distForTargPos = minDist;

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj )
	{
		gout << "  distToTravelForTargPos = " << distForTargPos << endl;
	}
#endif

	//
	// Use the path to obtain a target road position by traveling 
	// along my path from my current road position.  The distance to 
	// travel was computed above.
	// 
	bool haveValidPath = pI->m_pPath->Size() > 0;
	if( haveValidPath )
	{
		CPath::ETravelCode code;
		CRoadPos tempRoadPos = pI->m_roadPos;
		tempRoadPos.SetOffset( 0.0 );
		CLane lane;

		code = pI->m_pPath->GetRoadPos(
					distForTargPos, 
					tempRoadPos,
					targRoadPos
					);
		switch ( code ) {

		case CPath::eCV_TRAVEL_ERROR :
			
			gout << MessagePrefix( pI->m_pObj->GetId(), pI->m_objName );
			gout << "error with path travel   [SUICIDE]" << endl;
			gout << "  road position = " << tempRoadPos << endl;
			gout << "  cart position = " << tempRoadPos.GetXYZ() << endl;
			gout << "  distForTargPos = " << distForTargPos << " ft" << endl;
			gout << endl;

			gout << "  path:" << endl;
			gout << *pI->m_pPath << endl;
			
			Suicide();
			return;

		case CPath::eCV_TRAVEL_NOT_FOUND :

			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << "initial point not found" << endl;
			
			Suicide();
			return;

		case CPath::eCV_TRAVEL_OK :

			break;

		case CPath::eCV_TRAVEL_LANE_CHANGE :

			//
			// Need to do a lane change sometime.
			//

			break;

		case CPath::eCV_TRAVEL_END_OF_PATH :

			//
			// Reached end of path....need to extend path.
			//
			if ( !pI->m_pPath->Append( 300.0 ) ) {

				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "path append failed" << endl;
				
				Suicide();
				return;

			}
			break;

		default:

			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << "unknown ETravelCode case = " << code << endl;

			Suicide();
			return;
		}

	}
	else 
	{
		//
		// I don't have a path.
		//
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "I have no path...[SUICIDE]" << endl;

		Suicide();
		return;
	}

}  // end of CLcExecute::CalcTargRoadPos


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the total lateral distance between the current
//   offset and the center of the target lane.
//
// Remarks:
//
// Arguments:
//   pI - Pointer to the vehicle info. structure.
//   cCurrRoadPos - The current road position.
//   isRoadPos - is this the ADO's pos, or the Target Pos?
// Returns:  A double containing the total lateral distance.
//
//////////////////////////////////////////////////////////////////////////////
double
CLcExecute::CalcTotalLatDist( 
		CAdoInfoPtr& pI, 
		/*const CRoadPos&  cTargRoadPos,*/
		const CRoadPos& cCurrRoadPos,
        bool isAdoPos 
		)
{
    bool calcOverShot = isAdoPos && !m_overShotLc;
#ifdef DEBUG_CALCTOTALLATDIST
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_CALCTOTALLATDIST;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcExecute::CalcTotalLatDist ==============" << endl;
	}
#endif

	double totalLatDist;
	//
	// Forced lane Offset dial is 'SET'
	//
	if ( pI->m_pCurrLcCond->IsForcedLaneOffset() ) 
	{
		
		totalLatDist = 
			fabs( pI->m_pCurrLcCond->GetForcedLaneOffset() - cCurrRoadPos.GetOffset() );

	}
	else if ( pI->m_pCurrLcCond->IsTargLane() && cCurrRoadPos.IsRoad() ) 
	{
		//
		// I'm on a road and I have have a target lane.
		//
		CLane currLane = cCurrRoadPos.GetLane();
		CLane targLane = pI->m_pCurrLcCond->GetTargLane();

		bool notOnSameRoad = !( currLane.GetRoad() == targLane.GetRoad() );
		if( notOnSameRoad )
		{
			//
			// The lane change was initially started on a different road
			// than the one that I'm currently on.  Thus, change the 
			// TargLane in the current lane change condition to be the
			// target lane on the new road.
			//
			CLane origTargLane = targLane;
			CIntrsctn intrsctn = targLane.GetNextIntrsctn();
			if( !intrsctn.IsValid() )
			{
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "unable to find intersection between original ";
				gout << "target lane " << origTargLane << " and current lane ";
				gout << currLane << endl;

				Suicide();
				return -1.0;
			}

			TCrdrVec crdrs;
			intrsctn.GetCrdrsStartingFrom( origTargLane, crdrs );
			bool foundNewTargLane = false;
			TCrdrVec::iterator itr;
			for( itr = crdrs.begin(); itr != crdrs.end(); itr++ )
			{
				if( itr->GetDstntnRd() == currLane.GetRoad() )
				{
					targLane = itr->GetDstntnLn();
					foundNewTargLane = true;
					break;
				}
			}

			if( !foundNewTargLane )
			{
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "unable to find a new target lane from original target lane = ";
				gout << origTargLane << " thru intersection = " << intrsctn << endl;

				Suicide();
				return -1.0;
			}

#ifdef DEBUG_CALCTOTALLATDIST
			if( debugThisObj )
			{
				gout << "newTargLane = " << targLane << endl;
			}
#endif

			// set the new target lane
			pI->m_pCurrLcCond->SetTargLane( targLane );
		}

#ifdef DEBUG_CALCTOTALLATDIST
		if( debugThisObj )
		{
			gout << "  CurrPos on ROAD;  TargLane on ROAD" << endl;
			gout << "  currLane = " << currLane << endl;
			gout << "  targLane = " << targLane << endl;
			gout << "  currRoadPos = " << cCurrRoadPos << endl;
		}
#endif

		//
		// Compute the total lateral distance between my current position
		// and the target lane.
		//
		CRoadPos targRoadPos( 
					targLane.GetRoad(), 
					targLane, 
					cCurrRoadPos.GetDistance() 
					);
		totalLatDist = cCurrRoadPos.GetLateralDistance( targRoadPos );
		
        //we need to check to see if we have over-steered
        if (calcOverShot && fabs(totalLatDist) < 2.0f && !notOnSameRoad){
            if (!pI->m_pCurrLcCond->IsForcedLaneOffset()){
                if (pI->m_pCurrLcCond->IsLeftLaneChange()){
                    m_overShotLc = totalLatDist > 0;
                }else{
                    m_overShotLc = totalLatDist < 0;
                }
            }
        }
        totalLatDist = fabs( totalLatDist );
		//
		// Special case for when the target lane splits from the current
		// lane.  In this case, move to the left/right of the current
		// lane and wait until the lane splits.
		//
		if( pI->m_targLaneSplitsFromCurrLane )
		{
			double targLaneWidth = targLane.GetWidth( 
										targRoadPos.GetDistance() 
										);
			bool targLaneExists = targLaneWidth > 0.5;
			if( targLaneExists )
			{
				//
				// Target lane is now splitting off from the current
				// lane.
				//
				totalLatDist -= targLaneWidth * 0.5;
				totalLatDist = fabs( totalLatDist );
			}
			else
			{
				//
				// Target lane has yet to split off from the current lane.
				// Calculate the distance to the left/right edge of current
				// lane.
				//
				double currLaneWidth = currLane.GetWidth( 
											cCurrRoadPos.GetDistance()
											);
				double objWidth = pI->m_pObj->GetYSize();
				double maxDistToEdge = ( currLaneWidth - objWidth ) * 0.5;
				maxDistToEdge = maxDistToEdge - 2.0;
				if( maxDistToEdge < 0.0 )  maxDistToEdge = 0.0;
				CRoadPos tempRoadPos = cCurrRoadPos;
				if( pI->m_pCurrLcCond->IsLeftLaneChange() )
				{
					maxDistToEdge = -1.0 * maxDistToEdge;
					tempRoadPos.SetOffset( maxDistToEdge );
					totalLatDist = cCurrRoadPos.GetLateralDistance( 
														tempRoadPos 
														);

					// this function reports totalCalcDist as a postive value
					// and therefore if the ADO goes beyond the target lateral
					// dist, it starts to move off the road even more....this
					// sort of fixes that
					if( totalLatDist > 0.0 )  totalLatDist = 0.0;
				}
				else
				{
					tempRoadPos.SetOffset( maxDistToEdge );
					totalLatDist = cCurrRoadPos.GetLateralDistance( 
														tempRoadPos 
														);
					// this function reports totalCalcDist as a postive value
					// and therefore if the ADO goes beyond the target lateral
					// dist, it starts to move off the road even more....this
					// sort of fixes that
					if( totalLatDist < 0.0 )  totalLatDist = 0.0;
				}

				totalLatDist = fabs( totalLatDist );
			}
		}
	}  // end of CurrLane, TargLane
	else if( pI->m_pCurrLcCond->IsTargLane() && !cCurrRoadPos.IsRoad() )
	{
		//
		// I'm on an intersection and I have have a target lane.  Get
		// the source lane into the intersection.  This should be the lane
		// next to the target lane.
		//
		CCrdr currCrdr = cCurrRoadPos.GetCorridor();
#if 1
		CLane srcLane = currCrdr.GetSrcLn();
		if( !srcLane.IsValid() )
		{
			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << " LC: unable to get source lane from corridor...suicide";
			gout << endl;
			gout << "  crdrId = " << currCrdr.GetId() << endl;

			Suicide();
			return -1.0;
		}
#else
		CLane srcLane = pI->m_pCurrLcCond->GetSrcLane();
#endif

		CLane targLane = pI->m_pCurrLcCond->GetTargLane();
		bool sameRoad = srcLane.GetRoad() == targLane.GetRoad();
		if( !sameRoad )
		{
			if (currCrdr.GetDstntnLn().GetRoad() == targLane.GetRoad()){
				//at this point it looks our target pnt is past the crdr,
				//for calculating the offset lets call it the source
				srcLane = targLane;
			}else{
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << " LC: srcLane (" << srcLane << ") road not equal to ";
				gout << "targLane (" << targLane << ") road...suicide" << endl;
				Suicide();
				return -1.0;
			}

		}

#ifdef DEBUG_CALCTOTALLATDIST
		if( debugThisObj )
		{
			gout << "  CurrPos on INTRSCTN;  TargLane on ROAD" << endl;
			gout << "  srcLane = " << srcLane << endl;
			gout << "  targLane = " << targLane << endl;
			gout << "  currRoadPos = " << cCurrRoadPos << endl;
		}
#endif

		//
		// Compute the lateral distance that I need to cover between my 
		// current road position and the target lane.
		//
		bool leftLaneChange = pI->m_pCurrLcCond->IsLeftLaneChange();
		double currOffset = cCurrRoadPos.GetOffset();
		bool positiveOffset = currOffset >= 0.0;
		if( !leftLaneChange )  currOffset = -1.0 * currOffset;
		totalLatDist = fabs( srcLane.GetLateralDistance( targLane ) );
#ifdef DEBUG_CALCTOTALLATDIST
		if( debugThisObj )
		{
			gout << "  currOffset = " << cCurrRoadPos << " ft" << endl;
			gout << "  adjustedOffset = " << currOffset << " ft" << endl;
			gout << "  totalLatDist = " << totalLatDist << " ft" << endl;
		}
#endif
		if( targLane == srcLane )
		{
			totalLatDist = currOffset;
		}
		else
		{
			totalLatDist = totalLatDist + currOffset;
		}
#ifdef DEBUG_CALCTOTALLATDIST
		if( debugThisObj )
		{
			gout << "  totalLatDist = " << totalLatDist << " ft" << endl;
		}
#endif
	}  // end of CurrIntrsctn, TargLane
	else if( !pI->m_pCurrLcCond->IsTargLane() && !cCurrRoadPos.IsRoad() )
	{
		//
		// I'm on an intersection and I have have a target corridor.  
		// Either the source lanes into these corridors or the destination
		// lanes from these corridors have to been on the same road.  Find
		// the correct pair and then get the lateral distance that has to
		// be covered.
		//
		CCrdr currCrdr = cCurrRoadPos.GetCorridor();
		CLane srcLane = currCrdr.GetSrcLn();
		if( !srcLane.IsValid() )
		{
			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << " LC: unable to get source lane from corridor...suicide";
			gout << endl;
			gout << "  crdrId = " << currCrdr.GetId() << endl;

			Suicide();
			return -1.0;
		}

		CCrdr targCrdr = pI->m_pCurrLcCond->GetTargCrdr();
		CLane targLane = targCrdr.GetSrcLn();
		if( !targLane.IsValid() )
		{
			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << "LC: unable to obtain source lane from crdr...suicide";
			gout << endl;
			gout << "  crdrId = " << targCrdr.GetId() << endl;

			Suicide();
			return -1.0;
		}
		bool sameRoad = srcLane.GetRoad() == targLane.GetRoad();
		if( !sameRoad ) 
		{
			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << " LC: srcLane (" << srcLane << ") road not equal to ";
			gout << "targLane (" << targLane << ") road...suicide" << endl;

			Suicide();
			return -1.0;
		}

#ifdef DEBUG_CALCTOTALLATDIST
		if( debugThisObj )
		{
			gout << "  CurrPos on INTRSCTN;  TargLane on INTRSCTN" << endl;
			gout << "  srcLane = " << srcLane << endl;
			gout << "  targLane = " << targLane << endl;
			gout << "  currRoadPos = " << cCurrRoadPos << endl;
		}
#endif

		//
		// Compute the lateral distance that I need to cover between my 
		// current road position and the target lane.
		//
		totalLatDist = fabs( srcLane.GetLateralDistance( targLane ) );
		if( targLane == srcLane )
		{	
			totalLatDist = cCurrRoadPos.GetOffset();
		}
		else
		{
			totalLatDist += cCurrRoadPos.GetOffset();
		}
	}  // end of CurrIntrsctn, TargCrdr
	else if( !pI->m_pCurrLcCond->IsTargLane() && cCurrRoadPos.IsRoad() )
	{
		//
		// I'm on a road and I have a target corridor.  Get the destination
		// lane from the target corridor.  My current lane and that 
		// destination lane should be the same road.
		//
		CLane currLane = cCurrRoadPos.GetLane();
		CCrdr targCrdr = pI->m_pCurrLcCond->GetTargCrdr();

		//
		// The lane change was initially started on a corridor.  Since I'm
		// now on a road, change the TargCrdr to a TargLane.  This TargLane
		// should originate from the initial TargCrdr and should be to the
		// left/right of my current lane.
		//
		CLane targLane;
		if( pI->m_pCurrLcCond->IsLeftLaneChange() )
		{
			//
			// Left lane change.
			//
			if( currLane.IsValid() && !currLane.IsLeftMostAlongDir() )
			{
				targLane = currLane.GetLeft();

				if( !targLane.IsValid() )
				{
					gout << MessagePrefix( pI->m_pObj->GetId() );
					gout << " LC: unable to obtain destination lane from corridor";
					gout << "....suicide" << endl;
					gout << "  crdrId = " << targCrdr.GetId() << endl;

					Suicide();
					return -1.0;
				}

			}
			else
			{
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << " no lane to the left...this should be an abort";
				gout << "....[SUICIDE]" << endl;
				gout << "  currLane = " << currLane << endl;
				gout << "  targLane = " << targLane << endl;

				Suicide();
				return -1.0;
			}
		}
		else 
		{
			//
			// Right lane change.
			//
			if( !currLane.IsRightMost() )
			{
				targLane = currLane.GetRight();
				
				if( !targLane.IsValid() )
				{
					gout << MessagePrefix( pI->m_pObj->GetId() );
					gout << " LC: unable to obtain destination lane from corridor";
					gout << "....suicide" << endl;
					gout << "  crdrId = " << targCrdr.GetId() << endl;

					Suicide();
					return -1.0;
				}
			}
			else 
			{
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << " no lane to the right...this should be an abort";
				gout << "....[SUICIDE]" << endl;
				gout << "  currLane = " << currLane << endl;
				gout << "  targLane = " << targLane << endl;

				Suicide();
				return -1.0;
			}
		}

		if( !( targLane == targCrdr.GetDstntnLn() ) )
		{
			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << "new targLane doesn't originate from original ";
			gout << "targCrdr....[SUICIDE]" << endl;
			gout << "  currLane = " << currLane << endl;
			gout << "  targLane = " << targLane << endl;
			gout << "  targCrdr = " << targCrdr.GetRelativeId() << endl;

			Suicide();
			return -1.0;
		}

		bool sameRoad = currLane.GetRoad() == targLane.GetRoad();
		if( !sameRoad )
		{
			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << " LC: srcLane (" << currLane << ") road not equal to ";
			gout << "targLane (" << targLane << ") road...suicide" << endl;

			Suicide();
			return -1.0;
		}

#ifdef DEBUG_CALCTOTALLATDIST
		if( debugThisObj )
		{
			gout << "  CurrPos on ROAD;  TargLane on INTRSCTN" << endl;
			gout << "  targLane = " << targLane << endl;
			gout << "  currRoadPos = " <<cCurrRoadPos << endl;
		}
#endif

		//
		// Compute the lateral distance that I need to cover between my 
		// current road position and the target lane.
		//
		totalLatDist = fabs( cCurrRoadPos.GetLateralDistance( targLane ) );
	}  // end of CurrLane, TargCrdr
	else {

		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << " LC: unknown combination...suicide" << endl;

		Suicide();
		return -1.0;
	}

	return totalLatDist;

}  // end of CalcTotalLatDist


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the target lateral offset.
//
// Remarks:
//
// Arguments:
//   pI          - Pointer to the vehicle info. structure.
//   deltaOffset - The amount of offset to move to the left or right.
//
// Returns:  A double containing the total lateral distance.
//
//////////////////////////////////////////////////////////////////////////////
double
CLcExecute::CalcTargOffset( 
			CAdoInfoPtr& pI,
			double deltaOffset 
			)
{
	
	double targOffset;

	double currOffset = pI->m_roadPos.GetOffset();
	bool leftLaneChange = pI->m_pCurrLcCond->IsLeftLaneChange();
	if( leftLaneChange )
	{
		targOffset = currOffset - deltaOffset;	
		//gout << currOffset << " - " <<  deltaOffset 
		//	 << " = " << targOffset << endl;	
	}
	else 
	{
		targOffset = currOffset + deltaOffset;
		//gout << currOffset << " + " <<  deltaOffset 
		//	 << " = " << targOffset << endl;
	}
	return targOffset;
}  // end of CalcTargOffset


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Adjust the given target offset when the ADO is traveling
//   on overlapping lanes--i.e. where one lane splits off into two lanes.
//
// Remarks:  
//
// Arguments:
//   targOffset - (input/output) The target offset.
//
// Returns:  A boolean indicating if the ADO is currently on the target lane.
//
//////////////////////////////////////////////////////////////////////////////
static bool
AdjustTargOffsetForOverlappingLanes( 
			CAdoInfoPtr pI,
			const CRoadPos& cTargRoadPos,
			double& targOffset 
			)
{
	//
	// This part of the code deals with roads where a single lane
	// splits off into 2 lanes.
	//
	bool targLaneExists = false;
	bool onTargLane = false;
	if( pI->m_targLaneSplitsFromCurrLane )
	{
		CLane targLane = pI->m_pCurrLcCond->GetTargLane();
		onTargLane = targLane == pI->m_roadPos.GetLane();

		if( !onTargLane )
		{
			double targLaneWidth = targLane.GetWidth( 
												cTargRoadPos.GetDistance() 
												);
			targLaneExists = targLaneWidth > 0.5;
			if( targLaneExists )
			{
				if( pI->m_pCurrLcCond->IsLeftLaneChange() )
				{
					targOffset -= targLaneWidth * 0.5;
				}
				else
				{
					targOffset += targLaneWidth * 0.5;
				}
			}
		}
	}

	return onTargLane;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Adjust the given target offset when the ADO is travelling on a 
//				 on a curve. It 'increases' the offset when the vehicle and the road
//				 are curving in the same direction and 'decreases' the offset when
//				 the vehicle and the road are curving in different directions.
//
// Remarks:  
//
// Arguments:
//   pI				- Pointer to the vehicle info. structure.
//	 cTargRoadPos	- CRoadPos, input ref to target road position 
//   targOffset		- double (input/output) target offset.
//	 deque<double> storeRoadAngles - container to store roadAngles
//                    used to calculate the max Angle
//   maxRoadAngle   - stores the maxRoadAngle each frame
//
// Returns:  A boolean indicating if the ADO is currently on the target lane.
//
//////////////////////////////////////////////////////////////////////////////
static void 
AdjustTargOffsetForRoadCurvature( 
			CAdoInfoPtr pI,
			const CRoadPos& cTargRoadPos,
			deque<double>& storeRoadAngles,
			double& maxRoadAngle,
			double& targOffset
			)
{
	//
	// NOTE: This function needs to be updated for when the source
	// and target don't have the same curvature; such as corridors
	// that either meet or divert away from each other.
	//
	//
	// 1. Get a RoadPos 200ft in front of the vehicle.
	// 2. Calculate a vector using currPos and newTargRoadPos.
	// 3. Do a Cross product to get the direction of the curve.
	//
	CRoadPos newTargRoadPos = pI->m_roadPos;

	const double cDIST_TO_LOOK_AHEAD = 200.0f;   // feet
	CRoadPos::ETravelResult result = newTargRoadPos.Travel( 
														cDIST_TO_LOOK_AHEAD 
														);
	bool targRoadPosOnRoad = result == CRoadPos::eWITHIN_ROAD;
	if( targRoadPosOnRoad )
	{
		//
		// Computing a vector that represents the curve.
		//
		// Current position of the vehicle
		CRoadPos currRoadPos = pI->m_roadPos;
		currRoadPos.SetOffset( 0.0 );
	

		CPoint3D currentPos	= currRoadPos.GetVeryBestXYZ();		
		CPoint3D targetPos	= newTargRoadPos.GetVeryBestXYZ();
		CVector3D curveDir(
			targetPos.m_x - currentPos.m_x, 
			targetPos.m_y - currentPos.m_y, 
			targetPos.m_z - currentPos.m_z
			);

        double targetPosArcLength = curveDir.Length();

        //we are not on a sharp curve or recovering from a sharp curve
        CVector3D vec1 = pI->m_roadPos.GetTangentInterpolated();
        CVector3D vec2 = cTargRoadPos.GetTangentInterpolated();
        CVector3D tanDiff = vec1-vec2;
	    //check to see if we need to reduce our max offset due to lane width

	    CPoint3D currPos = pI->m_roadPos.GetXYZ();
	    CPoint3D targPos = cTargRoadPos.GetXYZ();	
			
	    CVector3D targetSeg(targPos - currPos);
        float laneWidth = 0;
   
	    //Scaling this by 1/3 the segment length
	    //was found through experimentations
	    //to work well, this should be parmaiterized 
	    tanDiff.Scale(targetSeg.Length()/3);


		CVector3D currTangent( currRoadPos.GetTangentInterpolated() );

		//
		// Normalize the curve vector and tangent and then
		// comptute the cross product to obtain the curve's 
		// direction.
		//
		curveDir.Normalize();
		currTangent.Normalize();
		CVector3D orthVector = curveDir.CrossP( currTangent );

		
		// Calculating the curves direction
		int curveDirection = 0; // 0->LEFT, 1->RIGHT
		//gout << "Orth Vector: " << orthVector.m_k;
		if( orthVector.m_k < -0.00 )
		{
			curveDirection = 0; // 0->LEFT
			//gout << " LEFT " << curveDirection << endl;
		}
		else
		{
			curveDirection = 1; // 1->RIGHT
			//gout << " RIGHT " << curveDirection << endl;
		}

		bool sameLcCurveDir = 
			( pI->m_pCurrLcCond->IsLeftLaneChange() && curveDirection == 0 ) ||
			( !(pI->m_pCurrLcCond->IsLeftLaneChange() ) && curveDirection == 1 );
		
/*		gout << "Vehicle Ori: " << (curveDirection==0?"LEFT":"RIGHT") 
			 << " " << "Lc Direction: "
			 << (pI->m_pCurrLcCond->IsLeftLaneChange()==true?"LEFT":"RIGHT")
			 << endl;
*/


		double rdAngle = 0.0;
		bool foundCurveAngle = CalcRoadAngle(
										currRoadPos, 
										cTargRoadPos, 
										rdAngle
										);
		if( !foundCurveAngle )
		{
			//
			// Unable to compute the curve's angle.  Not enough
			// info to compute offset...return.
			//
			return;
		}
		else
		{
			//
			// Pushing 20 rdAngles into deque
			//
			storeRoadAngles.push_back( rdAngle );
			if( storeRoadAngles.size() > 20 )
			{
				storeRoadAngles.pop_front();
			}

			deque<double>::iterator i;
			for( i = storeRoadAngles.begin(); i != storeRoadAngles.end(); i++ )
			{
				if( *i > maxRoadAngle)
				{
					maxRoadAngle = *i;
				}
			}
		}

		if( sameLcCurveDir )
		{
			double targOffsetAdjustSameDir = 15.0 * maxRoadAngle;
			targOffset += targOffsetAdjustSameDir;
			//gout << "Adjust Increase: " << targOffsetAdjustSameDir << endl;
			return;
		}
		else 
		{
			//
			// Decrement the targetOffset by looking at the curvature of the road
			// lane change direction of the vehicle
			//
			double targOffsetAdjustOppDir = 1.5 * maxRoadAngle;
			if( (targOffset - targOffsetAdjustOppDir) <= 0 )
			{
				//
				//  do not deduct targOffsetAdjustOppDir from targOffset
				//
				return;
			}
			else
			{			
                targOffset += tanDiff.Length();//targOffsetAdjustOppDir; 
				//gout << "Adjust Decrease:" << targOffsetAdjustOppDir << endl;			
				return;
			}
		}
	}
}



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Indicates if the lane change maneuver is complete.
//
// Remarks:  .
//
// Arguments:
//   pI - Pointer to the vehicle info. structure.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static bool
LaneChangeDone( CAdoInfoPtr pI )
{
	double cLC_DONE_LAT_DIST = 0.03;  // feet
    if (pI->m_pCurrLcCond->IsForcedLaneOffset() ){
        cLC_DONE_LAT_DIST = pI->m_forcedOffsetTolerance;
    }
    const double cLC_FINISHING_LAT_DIST = 0.5;
	double latDistRemaining = pI->m_lcTotalLatDist - pI->m_lcLatDistTraveled;
	bool laneChangeDone = latDistRemaining < cLC_DONE_LAT_DIST;
	if( pI->m_targLaneSplitsFromCurrLane )
	{
		CLane targLane = pI->m_pCurrLcCond->GetTargLane();
		bool onTargLane = targLane == pI->m_roadPos.GetLane();
		laneChangeDone = onTargLane;
	}
	if ( (latDistRemaining) < cLC_FINISHING_LAT_DIST){
        pI->m_pCurrLcCond->SetFinishing(true);
    }
	return laneChangeDone;
}


//////////////////////////////////////////////////////////////////////////////
///\brief
///  The LcExecute HCSM pre-activity function.
///
///\remark
///  This function is the LcExecute HCSM's pre-activity function.
///
///\todo needs to calculate take care of overshoots pI->m_lcLatDistTraveled 
////////////////////////////////////////////////////////////////////////////////
void 
CLcExecute::PreActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	//
	// Set LcExecute children's (Neutralize, Increment, Steady, Decrement) inputs.
	//
	SetInputpIForLcExecuteNeutralize( pI );
	SetInputpIForLcExecuteIncrement( pI );
	SetInputpIForLcExecuteSteady( pI );
	SetInputpIForLcExecuteDecrement( pI );


#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcExecute PRE-ACTIVITY ==============" << endl;
		gout << "  first frame = " << m_firstFrame << endl;
	}
#endif

#ifdef DEBUG_OUTPUT
	//
	// All the code inside 'DEBUG_OUTPUT' is for generating data files
	// for lane changes.  Hopefully, we'll replace this code sometime
	// in the near future with another HCSM that monitors and collects
	// this data from the outside.
	//
	// The static double implies that data can only
	static double startY;
#endif

	//
	// Calculate a target road position from the current position.
	//
	CalcTargRoadPos( pI, m_targRoadPos );

#ifdef DEBUG_LANECHANGE
	if( debugThisObj )  gout << "  m_targRoadPos = " << m_targRoadPos << endl;
#endif

	//
	// Initialize variables local to this HCSM.  The code inside the 
	// following IF statement executes once at the start of each
	// lane change.
	//
	
	if( m_firstFrame ) 
	{
		m_currLookAheadTime = 1.0f;
        m_maxRoadAngle		= 0.0;
	    m_storeRoadAngle.clear();
        pI->m_lcSinAngle = 0.0;
		pI->m_lcTotalLatDist = CalcTotalLatDist( pI, pI->m_roadPos, true );
        pI->m_pCurrLcCond->SetFinishing(false);
		if( !pI->m_pCurrLcCond->IsForcedLaneOffset() )
		{
			if( pI->m_pCurrLcCond->IsTargLane() )
			{
				CLane targLane = pI->m_pCurrLcCond->GetTargLane();
				double targLaneWidth = targLane.GetWidth( pI->m_roadPos.GetDistance() );
				pI->m_targLaneSplitsFromCurrLane = targLaneWidth < 0.5;
			}
		}
        m_overShotLc = false;

#ifdef DEBUG_OUTPUT
		startY = pI->m_roadPos.GetXYZ().m_y;
		m_startLcFrame = GetFrame();
		
#endif
	}

	//
	// Compute the total lateral distance between the current position
	// and the center of the target lane.
	//
	double currentLatDist = CalcTotalLatDist( pI,  pI->m_roadPos, true  );
	pI->m_lcLatDistTraveled = pI->m_lcTotalLatDist - currentLatDist;

#ifdef DEBUG_LANECHANGE
	if( debugThisObj ) 
	{
		gout << "  currentLatDist = " << currentLatDist << " ft" << endl;
		gout << "  m_targLaneSplitsFromCurrLane = ";
		gout << pI->m_targLaneSplitsFromCurrLane << endl;
	}
#endif

#ifdef DEBUG_OUTPUT
	
	double currentY =  pI->m_roadPos.GetXYZ().m_y;
	double currentX =  pI->m_roadPos.GetXYZ().m_x;
	if( LaneChangeDone( pI ) )
	{
		ofstream fileLongDist( "longDist.txt", ios::app);
		fileLongDist << pI->m_pObj->GetVelImm() * cMS_TO_MPH << " ";
		fileLongDist << pI->m_pCurrLcCond->GetUrgency() << " ";
		fileLongDist << currentY - startY << " ";
		fileLongDist.precision(2);
		fileLongDist << double((GetFrame() - m_startLcFrame)/30.0) << " ";
		fileLongDist << pI->m_pObj->GetSolId() << endl;
		fileLongDist.close();
	}
#endif

	// Initializing variables for LcExecuteNeutralize
	CRandLaneDevInfo& info = pI->m_lcNeutralizeOffset;
	if( info.m_RampStartTime <= 0 )
	{
		info.m_RampStartTime = GetFrame();
		// Read Values of RampAmp1 && RampAmp2
		info.m_RampAmp1 = pI->m_randLaneDev.m_RampAmp1;
		info.m_RampAmp2 = pI->m_randLaneDev.m_RampAmp2;

		// Rise and Fall depend on user specified lane deviation params
		// and are linearly interpolated from lane dev. params
		info.m_Rise = fabs( 0.288 * fabs(pI->m_roadPos.GetOffset()) + 0.27 ); // manually set
		info.m_Fall = info.m_Rise; // rise and fall are same
	}

}  // end of PreActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description: Adjusting the velocity to complete lane changes infront of 
//   intersections etc.
//
// Remarks:  
//
// Arguments: 
//   pI				- Pointer to the vehicle info. structure.
//   targVel		- (output) The target velocity needed to complete the 
//					  lane change in time (in m/s).
//   distToIntrsctn - (output) The distance to the nexst intersection 
//					  (in feet).
//
// Returns: A boolean to indicate if a target velocity was found.
//
//////////////////////////////////////////////////////////////////////////////
bool 
AdjustVelToCompleteLaneChange( 
			CAdoInfoPtr pI, 
			double& targVel, 
			double& distToIntrsctn 
			)
{
	double currVel = pI->m_pObj->GetVelImm() * cMS_TO_MPH; // mph
	double urgency = pI->m_pCurrLcCond->GetUrgency();
	int solId   = pI->m_pObj->GetSolId();
	
	// Get the dist to next intersection
	distToIntrsctn = pI->m_pPath->GetDistToNextIntrsctn( pI->m_roadPos ); //ft
	// Calculate the total lateral Dist

	double percentLatDistTraveled;
	if( pI->m_lcTotalLatDist > cNEAR_ZERO )
	{
		percentLatDistTraveled = pI->m_lcLatDistTraveled/pI->m_lcTotalLatDist;
	}
	else
	{
		return false;
	}

	double distReqdForLc;
	double timeReqdForLc;
	bool foundDist = LcLookupDistTime(
									currVel, 
									urgency,
									solId,
									distReqdForLc, 
									timeReqdForLc
									);
	bool enoughDistForLc = distToIntrsctn > distReqdForLc * percentLatDistTraveled;
	const double cDECREASE_VEL = 10.0;  // mph
	while( !enoughDistForLc && currVel >= 25.0  )
	{	
		currVel = currVel - cDECREASE_VEL; // decrease the velocity
		bool foundDist = LcLookupDistTime( 
									currVel, 
									urgency, 
									solId, 
									distReqdForLc, 
									timeReqdForLc
									);
		// recalculate the bool value again.
		enoughDistForLc = distToIntrsctn > distReqdForLc * percentLatDistTraveled;
		if( foundDist && !enoughDistForLc )
		{
#ifdef DEBUG_LANECHANGE
			gout << "found vel: " << currVel << " distReqdForLc: ";
			gout << distReqdForLc * percentLatDistTraveled;
			gout << " travelled: " << percentLatDistTraveled << "%" << endl;
#endif
			targVel = currVel * cMPH_TO_MS;
			return true;
		}
	}
	return false;
} // end of AdjustVelToCompleteLaneChange




//////////////////////////////////////////////////////////////////////////////
//
// Description: Adjusting the velocity to complete multiple lane changes before  
//				intersections etc.
//   
//
// Remarks:  
//
// Arguments: 
//   pI				- Pointer to the vehicle info. structure.
//   targVel		- (output) The target velocity needed to complete the lane
//					   change in time (in m/s).
//   distToIntrsctn - (output) The distance to the nexst intersection 
//					  (in feet).
//
// Returns: A boolean to indicate if a target velocity was found.
//
//////////////////////////////////////////////////////////////////////////////
bool 
AdjustVelForMultipleLaneChanges(
			CAdoInfoPtr pI, 
			double& targVel,
			double& distToIntrsctn 
			)
{
	
	// Current Lane
	CLane currLane = pI->m_roadPos.GetLane();
	// find the target lane from CPath::TravelCode
	CPath::ETravelCode code;
	CRoadPos tempRoadPos; // junk	
	// target Lane might be multiple lanes away from current lane
	CLane targLane; 
	CPath junkPath = *(pI->m_pPath);// using a junk path to update targLane and 
									// not change current path
	code = junkPath.Travel(
					0.0,
					pI->m_roadPos,
					tempRoadPos, 
					targLane
					);
	//
	// We are going to call this function from LcExecute::PostActivity
	// By this time, we have valid targetRoadPos.
	//
	if( code != CPath::eCV_TRAVEL_LANE_CHANGE )
	{
		return false; // exit out of this function
	}
	//
	// check if current lane and targLane are adjacent
	//
	bool adjacentLanes = ( ( !currLane.IsLeftMostAlongDir() && 
						currLane.GetLeft() == targLane 
						) || 
					  ( !currLane.IsRightMost() &&
						currLane.GetRight() == targLane
						)
					  );
	int totalLaneChanges = 0;
	if( currLane.IsValid() && targLane.IsValid() )
	{
		// if adjacent, # of lanechanges = 1
		if( adjacentLanes )
		{
			totalLaneChanges = 1;
		}
		// Calculating the total lane changes
		else
		{
			totalLaneChanges = abs( targLane.GetRelativeId() - currLane.GetRelativeId() );
		}
	}

	//gout << "Travel Code: " << code << endl;
	//gout << "Current Lane: " << currLane << endl;
	//gout << "Target Lane: " << targLane << endl;
	//gout << "Total Lane Changes: " << totalLaneChanges << endl;

	// Get the dist to next intersection
	distToIntrsctn = pI->m_pPath->GetDistToNextIntrsctn( 
													pI->m_roadPos
													);	// ft

	// Calculate the total Dist reqd for Multiple LaneChanges
	double currVel = pI->m_pObj->GetVelImm() * cMS_TO_MPH; // mph
	double urgency = pI->m_pCurrLcCond->GetUrgency();
	int solId   = pI->m_pObj->GetSolId();
	double distReqdForLc;
	double timeReqdForLc;
	bool foundDist = LcLookupDistTime(
									currVel, 
									urgency,
									solId,
									distReqdForLc, 
									timeReqdForLc
									);
	
	if( !foundDist ) return false;

	//
	// Single lane change
	//
	if( totalLaneChanges <= 1 )
	{
		double percentLatDistTraveled = 
					pI->m_lcLatDistTraveled/pI->m_lcTotalLatDist;
		bool enoughDistForLc = 
					distToIntrsctn > distReqdForLc * percentLatDistTraveled;
		const double cDECREASE_VEL = 10.0;  // mph
		while( !enoughDistForLc && currVel >= 25.0  )
		{	
			currVel = currVel - cDECREASE_VEL; // decrease the velocity
			bool foundDist = LcLookupDistTime( 
										currVel, 
										urgency, 
										solId, 
										distReqdForLc, 
										timeReqdForLc
										);
			// recalculate the bool value again.
			enoughDistForLc = 
						distToIntrsctn > distReqdForLc * percentLatDistTraveled;
			if( foundDist && !enoughDistForLc )
			{
#ifdef DEBUG_LANECHANGE
			gout << "found vel: " << currVel << " distReqdForLc: ";
			gout << distReqdForLc * percentLatDistTraveled;
			gout << " travelled: " << percentLatDistTraveled << "%" << endl;
#endif
				targVel = currVel * cMPH_TO_MS;
				return true;
			}
		}
	}
	else if( totalLaneChanges > 1 )
	{
		// Implement Code for multilane change
		const double cFUDGE_DIST = 30.0; // feet
		double totalDistReqdForMultiLaneChange = 
					distReqdForLc * totalLaneChanges + cFUDGE_DIST;
#ifdef DEBUG_LANECHANGE
		gout << "dist for multi-lanechange: ";
		gout << totalDistReqdForMultiLaneChange;
		gout << " distToIntrsctn: " << distToIntrsctn << endl;
#endif
		
		bool enoughDistForLc = 
					distToIntrsctn >= totalDistReqdForMultiLaneChange;
		if( !enoughDistForLc )
		{
			// Recalculate the redn in velocity
			// Query the data tables for a velocity that makes a
			// multiple lane changes and traverses the intrsctn
			// successfully w/o sudden redn in velocity

			double laneChangeDist = 
				(distToIntrsctn - cFUDGE_DIST) / totalLaneChanges;
			double reqdVel = 0.0;
			bool foundVel = LcLookupVelocityfromDist(
										laneChangeDist, 
										urgency, 
										solId, 
										reqdVel
										);
			if( foundVel )
			{
#ifdef DEBUG_LANECHANGE
				gout << "Velocity(mph): " << reqdVel << endl;
#endif
				targVel = reqdVel * cMPH_TO_MS;
				return true;
			}
			else if( !foundVel )
			{
				return false;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcExecute HCSM post-activity function.
//
// Remarks:  This function is the LcExecute HCSM's post-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcExecute::PostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	//
	// Get the targOffset from the active child.
	//
	double targOffset;
    bool isDecrement = false;
	if( m_activeChild->GetName() == "LcExecuteNeutralize" ) 
	{
		if( HasValueOutputTargOffsetFromLcExecuteNeutralize() )
		{
			targOffset = GetOutputTargOffsetFromLcExecuteNeutralize();
		}
	}
	else if( m_activeChild->GetName() == "LcExecuteIncrement" ) 
	{
		if( HasValueOutputTargOffsetFromLcExecuteIncrement() )
		{
			targOffset = GetOutputTargOffsetFromLcExecuteIncrement();
		}
        if (HasValueOutputTargLookAheadFromLcExecuteIncrement()){
            m_currLookAheadTime = GetOutputTargLookAheadFromLcExecuteIncrement();
        }
	}
	else if( m_activeChild->GetName() == "LcExecuteSteady" )
	{
		if( HasValueOutputTargOffsetFromLcExecuteSteady() )
		{
			targOffset = GetOutputTargOffsetFromLcExecuteSteady();
		}
        if (HasValueOutputTargLookAheadFromLcExecuteSteady()){
            m_currLookAheadTime = GetOutputTargLookAheadFromLcExecuteSteady();
        }
	}
	else
	{
		if( HasValueOutputTargOffsetFromLcExecuteDecrement() )
		{
			targOffset = GetOutputTargOffsetFromLcExecuteDecrement();
		}
        if (this->HasValueOutputTargLookAheadFromLcExecuteDecrement()){
            m_currLookAheadTime = GetOutputTargLookAheadFromLcExecuteDecrement();
        }
	}
	
	AdjustTargOffsetForOverlappingLanes( pI, m_targRoadPos, targOffset );
	AdjustTargOffsetForRoadCurvature( 
			pI, 
			m_targRoadPos, 
			m_storeRoadAngle, 
			m_maxRoadAngle, 
			targOffset
			);
	//m_lastTargetOffset = targOffset;
	double targVel;
	double targAccel;
	double distToIntrsctn;
	bool needToAdjustVel = AdjustVelForMultipleLaneChanges(
									pI, 
									targVel, 
									distToIntrsctn 
									);
	
	if( needToAdjustVel )
	{
		double targDist = distToIntrsctn * 0.2;
		targAccel = LinearDecelController(
							pI->m_currVel,
							targVel,
							targDist * cFEET_TO_METER
							);
		SetOutputTargAccel( targAccel );	
	}
	else
	{
		m_outputTargAccel.SetNoValue();
	}

	//
	// Apply the lateral offset to the target road position.
	//
	double actualOffset = CalcTargOffset(
								pI,
								targOffset 
								);
    //we have overshot our LC, just set the target to 0
    if (m_overShotLc){
        if (!pI->m_haveForcedLaneOffset){
            m_targRoadPos.SetOffset(0);
        }else{
            m_targRoadPos.SetOffset(pI->m_forcedLaneOffset);
        }
    }else{
	    m_targRoadPos.SetOffset( actualOffset );
    }
    m_lastTargetOffset = actualOffset;
    pI->m_prevForcedLaneOffset = actualOffset;
    pI->m_lcTargPointLatDistToTarget = CalcTotalLatDist( pI, m_targRoadPos, false);
    
	//
	// Write turn signal and target position information to the 
	// control inputs.
	//
	CPoint3D targPos = m_targRoadPos.GetVeryBestXYZ();
	//
	// Signal only if 'LaneChange' is taking place. If a forced LaneOffset 
	// dial is set, no need to signal. If we are basically done with the lc
	// and are just finishing centering the car, turn it off to, som
    if( !pI->m_haveForcedLaneOffset && !pI->m_pCurrLcCond->IsFinishing() )
	{
		SignalLaneChange( pI );
	}
	SetOutputTargPos( targPos );

	// gout << "** currPos = " << pI->m_roadPos.GetXYZ() << endl;
	// gout << "** targPos = " << targPos << endl;

	m_firstFrame = false;
}  // end of PostActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Represents the transition from the LcExecute HCSM
//   to LcAbort HCSM.
//
// Remarks:  This predicate function represents the transition from
//   LcExecute HCSM to LcAbort HCSM.  
//
// Arguments:
//
// Returns:  A boolean indicating whether the transition fired or not.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLcExecute::LcExecuteLcAbortPredicate()
{

	CAdoInfoPtr pI = GetInputpI();
#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcExecute --> LcAbort PREDICATE =============";
		gout << endl;
	}
#endif

	bool fire = false;
    //right now we only have a scenario induced abort.....
    bool abortActive = pI->m_pCurrLcCond->GetScenarioInducedAbort();
	if( abortActive )
	{
        auto child = GetActiveChild();
        if (child && child->GetName() == "LcExecuteNeutralize"){
            m_firstFrame = true;
            fire = true;
        }
        if (!child){
            fire = true;
        }
	} 
    //if our button has been hit, we want to let our children run 1 
    //frame so they can abort themselves, and we can get back to baseline.
    if (pI->m_reporjectAndResetLaneOffsetButton)
        pI->m_pCurrLcCond->SetScenarioInducedAbort(true);
	return fire;

}  // end of LcExecuteLcAbortPredicate


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Represents the transition from the LcExecute HCSM
//   to LcMonitor HCSM.
//
// Remarks:  This predicate function represents the transition from
//   LcExecute HCSM to LcMonitor HCSM.  
//
// Arguments:
//
// Returns:  A boolean indicating whether the transition fired or not.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLcExecute::LcExecuteLcMonitorPredicate()
{

	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_LANECHANGE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_LANECHANGE;
	if( debugThisObj ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== LcExecute --> LcMonitor PREDICATE =============";
		gout << endl;
	}
#endif

	bool fire = LaneChangeDone( pI ); //|| pI->m_reporjectAndResetLaneOffsetButton;

	if( fire )
	{
		//
		// Lane change is now complete...switch lanes in the path.
		//
		if( pI->m_roadPos.IsRoad() ) 
		{
			if( !pI->m_pCurrLcCond->IsForcedLaneOffset() )
			{
				if( pI->m_pCurrLcCond->IsTargLane() ) 
				{
					pI->m_pPath->SwitchLane( pI->m_pCurrLcCond->GetTargLane() );
				}
			}
		}

		//
		// Set the time of completion.
		//
		pI->m_prevLcCompleteFrame = GetFrame();

		//
		// Set past condition equal to current condition and reset current
		// condition.
		//
		pI->m_pastLcCond = pI->m_pCurrLcCond->GetType();
		pI->m_pCurrLcCond = NULL;
		CLcCondsPtr pCond = GetInputpCond();
		pCond->ResetCurrentCondition();

		//
		// Reset local variables.
		//
		m_firstFrame = true;

		// Reset Neutralize Start Frame
		CRandLaneDevInfo& info = pI->m_lcNeutralizeOffset;
		info.m_RampStartTime = 0;

		// In FREEDRIVE 're-randomize' the calculation of random lane deviation
		// values
		pI->m_randLaneDev.m_Timer = 0;
		pI->m_randLaneDev.m_Last = 0.0f;


	}

#ifdef DEBUG_LANECHANGE
	if( debugThisObj ) 
	{
		gout << "  fire = " << fire << endl;
	}
#endif

	return fire;

}  // end of LcExecuteLcMonitorPredicate

void 
CLcAbort::Creation(){
    m_abortCount = 0;
    m_firstFrame = true;
    m_scenarioTriggeredAbort = false;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcAbort HCSM pre-activity function.
//
// Remarks:  This function is the LcAbort HCSM's pre-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcAbort::PreActivity()
{

	//
	// Get the vehicle info pointer.
	//
   
	CAdoInfoPtr pI = GetInputpI();
	CLcCondsPtr pCond = GetInputpCond();
    if (pI->m_reporjectAndResetLaneOffsetButton){
         m_abortCount = 0;
        pI->m_pCurrLcCond->SetScenarioInducedAbort(true);
    }

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The LcAbort HCSM post-activity function.
//
// Remarks:  This function is the LcAbort HCSM's post-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CLcAbort::PostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();
	CLcCondsPtr pCond = GetInputpCond();
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Represents the transition from the LcAbort HCSM
//   to LcMonitor HCSM.
//
// Remarks:  This predicate function represents the transition from
//   LcAbort HCSM to LcMonitor HCSM.  
//
// Arguments:
//
// Returns:  A boolean indicating whether the transition fired or not.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CLcAbort::LcAbortLcMonitorPredicate()
{
    CAdoInfoPtr pI = GetInputpI();
    //This button indicates we are "reseting" the Lane change
    pI->m_pCurrLcCond->SetScenarioInducedAbort(false);
	return true;

}  // end of LcAbortLcMonitorPredicate



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculate the vehicle orientation 
//
// Remarks:  
//
// Arguments:
//   pI				- Pointer to the vehicle info. structure.
//
// Returns:  A boolean indicating if ADO's lane change direction is same as
//			 current vehicle orientation
//
//////////////////////////////////////////////////////////////////////////////
bool 
CalcVehicleOrientation( CAdoInfoPtr pI )
{
	//
	// 1. Get a RoadPos 20ft in front of the vehicle.
	// 2. Calculate a vector using currPos and newTargRoadPos.
	// 3. Do a Cross product to get the direction of the curve.
	//
	CRoadPos newTargRoadPos = pI->m_roadPos;
	const double cDIST_TO_LOOK_AHEAD = 20.0;   // feet
	double aheadDistUsed = cDIST_TO_LOOK_AHEAD;
	CRoadPos::ETravelResult result = newTargRoadPos.Travel( 
												cDIST_TO_LOOK_AHEAD 
												);
	bool targRoadPosOnRoad = result == CRoadPos::eWITHIN_ROAD;
	if( targRoadPosOnRoad )
	{
		// Computing a vector that represents the curve.
		CRoadPos currRoadPos = pI->m_roadPos;
		currRoadPos.SetOffset( 0.0 );
		CPoint3D currentPos	= currRoadPos.GetVeryBestXYZ();		
		CPoint3D targetPos	= newTargRoadPos.GetVeryBestXYZ();
		CVector3D vehDir(
			targetPos.m_x - currentPos.m_x, 
			targetPos.m_y - currentPos.m_y, 
			targetPos.m_z - currentPos.m_z
			);
		CVector3D currTangent( currRoadPos.GetTangent() );
		// Calculate orientation of veh
		double dotProduct =  currTangent.DotP( vehDir );
		double lengthProduct = currTangent.Length()* vehDir.Length();
		double vehAngle= 0.0; // initialization
		if( !(dotProduct / lengthProduct > 1.0) )
		{
			vehAngle= acos( dotProduct / lengthProduct );
		}

		//
		// Normalize the obj's direction vector and tangent 
		// then comptute the cross product to obtain the obj's 
		// direction.
		//
		vehDir.Normalize();
		currTangent.Normalize();
		CVector3D orthVector = currTangent.CrossP( vehDir );
		int vehDirection = 0; // 0->LEFT, 1->RIGHT
		if( orthVector.m_k < 0.00 ){
			vehDirection = 1; // 1->RIGHT
			//gout << "RIGHT";
		}
		else {
			vehDirection = 0; // 0->LEFT
			//gout << "LEFT";
		}
		
		bool sameLcDirectionAsDeviation = 
			( pI->m_pCurrLcCond->IsLeftLaneChange() && vehDirection == 0 ) ||
			( !(pI->m_pCurrLcCond->IsLeftLaneChange() ) && vehDirection == 1 );

#ifdef DEBUG_LcEXECUTE_STATES
		gout << "Vehicle Ori: " << (vehDirection==0?"LEFT":"RIGHT") 
			 << " " << "Lc Direction: "
			 << (pI->m_pCurrLcCond->IsLeftLaneChange()==true?"LEFT":"RIGHT")
			 << endl;
#endif
		return sameLcDirectionAsDeviation;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculate target offset in LcNeutralize state 
//
// Remarks:  
//
// Arguments:
//   pI				- Pointer to the vehicle info. structure.
//   currFrame      - current frame
//
// Returns:  A double having the calculated Offset value using ramps model
//
//////////////////////////////////////////////////////////////////////////////
double 
CalcOffsetRampsModel( 
	  CAdoInfoPtr   pI, 
	  int			currFrame
	  )
{
	CRandLaneDevInfo& info = pI->m_lcNeutralizeOffset;
	double   delta;			// time, in sec, since start of Ramp waveform
	double   ofs;
	
	info.m_RampAmp = fabs(pI->m_roadPos.GetOffset());
	delta = (currFrame - info.m_RampStartTime) / 30.0;

	if ( delta <= info.m_Rise ) {	// rising part
		ofs = info.m_RampAmp * delta / info.m_Rise;
	}
	else if ( delta <= info.m_Rise + info.m_Fall ) { // falling
		delta -= info.m_Rise;
		ofs = info.m_RampAmp - info.m_RampAmp * delta / info.m_Fall;
	}
	else {	// re-randomize for next waveform
		info.m_RampStartTime = currFrame;
		delta = (currFrame - info.m_RampStartTime) / 30.0; // re-calc. delta
		info.m_Rise = fabs( 0.288 * fabs(pI->m_roadPos.GetOffset()) + 0.27 ); // manually set
		info.m_Fall = info.m_Rise; // rise and fall are same
		ofs = 0.0;
	}
#ifdef DEBUG_LcEXECUTE_STATES
	gout << "Amp: " << info.m_RampAmp << " frame: " 
		 << currFrame << " RmpStTime: " << info.m_RampStartTime 
		 << " delta: " << delta << " Offset " << ofs << endl;
#endif

	return ofs;
}


void
CLcExecuteNeutralize::UserPostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	// calculating offset from RampModel
	double targOffset = CalcOffsetRampsModel( pI, GetFrame() );

#ifdef DEBUG_LcEXECUTE_STATES
	gout << MessagePrefix( pI->m_pObj->GetId() );
	gout << "======== LcExecuteNeutralize ==============" << endl;
	gout << "Target Offset: " << targOffset << endl;
#endif

	SetOutputTargOffset( targOffset );
}
////////////////////////////////////////////////////////
///\brief
///     this trans from baseline to lane change
///\todo
///   The steering controller does not handle banked
///   roads well at all, this is a complete hack to 
///   deal with this.
/////////////////////////////////////////////////////// 
bool
CLcExecuteNeutralize::LcNeutralizeIncrementPredicate()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI(); 
    LaneChangeDone( pI );

	bool sameLcDirectionAsDeviation = CalcVehicleOrientation( pI );

	// Look at current offset and decide if additional offset 
	// is necessary
	double cLcNEUTRALIZE_DONE_DIST = 0.1;
    if (pI->m_pCurrLcCond->IsForcedLaneOffset() ){
        cLcNEUTRALIZE_DONE_DIST = pI->m_forcedOffsetTolerance;
    } 
   
    float curve = pI->m_roadPos.GetCurvature();
    //float grade =  pI->m_roadPos.GetGrade(true);
    float doneDist = cLcNEUTRALIZE_DONE_DIST;
    auto rv = pI->m_roadPos.GetRightVecInterpolated(true);
    float grade = rv.m_k;
    doneDist +=min(2.0f,float(fabs(grade*100) * (2000.0f/ curve)));
    
	bool notNeedOffset = 
				fabs( pI->m_roadPos.GetOffset() ) < doneDist;

#ifdef DEBUG_LcEXECUTE_STATES
	gout << "======== [LcExecute] Neutralize-->Increment PREDICATE ========" << endl;
#endif

	/*** Omar: Comment below line and 'return true' to skip Neutralize ***/
	return notNeedOffset || sameLcDirectionAsDeviation;

}


void
CLcExecuteIncrement::UserPostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();


	//
	// Increment the sin angle.
	//
	const double cMAX_SIN_ANGLE = 90.0;   // degrees
	bool needToIncrement = pI->m_lcSinAngle < cMAX_SIN_ANGLE;
    double steerForce = 0;
    if (pI->m_pCurrLcCond->IsForcedLaneOffset()){
        steerForce = pI->m_forcedOffsetMaxSteerForce;
    }else{
        steerForce = pI->m_lcInfo.steeringForce;;
    }
	if( needToIncrement )  pI->m_lcSinAngle = pI->m_lcSinAngle + steerForce;
	pI->m_lcMaxSinAngle = pI->m_lcSinAngle;

	//
	// Calculate the target offset using the given urgency and the
	// vehicle's velocity.
	//
	double currVel = pI->m_pObj->GetVelImm();
	double minOffset = CalcMinLatOffset( currVel );
    double maxOffset;
    if (!pI->m_pCurrLcCond->IsForcedLaneOffset()){
        maxOffset = CalcMaxLatOffset( currVel, pI->m_lcInfo.maxLatOffset );
    }else{
        maxOffset = CalcMaxLatOffset( currVel, pI->m_forcedOffsetMaxSteerDistance );
    }
	double urgency = pI->m_pCurrLcCond->GetUrgency();
	double deltaOffsetFromUrgency = CalcDeltaOffsetFromUrgency(
											minOffset,
											maxOffset,
											urgency
											);
	double angle = sin( pI->m_lcSinAngle * cDEG_TO_RAD );
    double targOffset = (
				deltaOffsetFromUrgency * angle
				);

	float targLookAhead = 0.5 + 0.5 *(1 - angle);
	//
	// Set outputs.
	//
	SetOutputTargOffset( targOffset );
    SetOutputTargLookAhead(targLookAhead);
#ifdef DEBUG_LcEXECUTE_STATES
	gout << "======== LcExecuteIncrement ==============" << endl;
	gout << "steeringForce          = " << steerForce << endl;
	gout << "sinAngle               = " << pI->m_lcSinAngle << endl;
	gout << "minOffset              = " << minOffset << "  maxOffset: " << maxOffset; 
	gout << "urgency                = " << urgency << endl;
	gout << "deltaOffsetFromUrgency = " << deltaOffsetFromUrgency << endl;
	gout << "targOffset             = " << targOffset << endl << endl;
#endif
}

bool
CLcExecuteIncrement::LcIncrementSteadyPredicate()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();


	bool reachedMaxSinAngle = pI->m_lcSinAngle >= 90.0;
	bool reachedHalfwayPoint = (
				pI->m_lcLatDistTraveled >= 0.5 * pI->m_lcTotalLatDist
				);

#ifdef DEBUG_LcEXECUTE_STATES
	gout << "======== [LcExecute] Increment-->Steady PREDICATE ========" << endl;
	//gout << "Reached MaxSinAngle 1->true, 0->false: " << reachedMaxSinAngle << endl;
	//gout << "Reached Half: " << reachedHalfwayPoint << endl;
	//gout << "LatDistTravelled: " << pI->m_lcLatDistTraveled << endl << endl;
#endif

	return reachedMaxSinAngle || reachedHalfwayPoint;
}

bool
CLcExecuteIncrement::LcIncrementLcAbortPredicate(){
	CAdoInfoPtr pI = GetInputpI();
    if (pI->m_pCurrLcCond->GetScenarioInducedAbort()){
        return true;
    }
    return false;
}

void
CLcExecuteSteady::UserPostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	double currVel = pI->m_pObj->GetVelImm();
	double minOffset = CalcMinLatOffset( currVel );
	double maxOffset = 0;
    if (!pI->m_pCurrLcCond->IsForcedLaneOffset()){
        maxOffset = CalcMaxLatOffset( currVel, pI->m_lcInfo.maxLatOffset );
    }else{
        maxOffset = CalcMaxLatOffset( currVel, pI->m_forcedOffsetMaxSteerDistance );
    }
	double urgency = pI->m_pCurrLcCond->GetUrgency();
	double deltaOffsetFromUrgency = CalcDeltaOffsetFromUrgency(
											minOffset,
											maxOffset,
											urgency
											);

	double targOffset = (
				deltaOffsetFromUrgency * sin( pI->m_lcSinAngle * cDEG_TO_RAD )
				);

	double sinVal = sin( pI->m_lcSinAngle * cDEG_TO_RAD );

	float targLookAhead = 0.5 + 0.5 *(1 - sinVal);
    SetOutputTargLookAhead(targLookAhead);
	//
	// Set outputs.
	//
	SetOutputTargOffset( targOffset );

#ifdef DEBUG_LcEXECUTE_STATES
	gout << "======== LcExecuteSteady ==============" << endl;
	gout << "sinAngle               = " << pI->m_lcSinAngle << endl;
	gout << "min max offset         = " << minOffset << "   " << maxOffset << endl;
	gout << "deltaOffsetFromUrgency = " << deltaOffsetFromUrgency << endl;
	gout << "targOffset             = " << targOffset << endl;
	gout << "latDistTravelled       = " << pI->m_lcLatDistTraveled << endl << endl;
#endif
}

bool
CLcExecuteSteady::LcSteadyDecrementPredicate()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();
    double lastOffset = 0;
    if (m_outputTargOffset.HasValue()){
        lastOffset = m_outputTargOffset.GetValue();
    }
	bool reachedHalfwayPoint = (
				pI->m_lcLatDistTraveled + lastOffset >= pI->m_lcTotalLatDist 
				);

#ifdef DEBUG_LcEXECUTE_STATES
	gout << "======== [LcExecute] Steady-->Decrement PREDICATE ========" << endl;
	//gout << "Reached Half: " << reachedHalfwayPoint << endl << endl;
#endif

	return reachedHalfwayPoint;
}
bool
CLcExecuteSteady::LcSteadyLcAbortPredicate()
{
	CAdoInfoPtr pI = GetInputpI();
    if (pI->m_pCurrLcCond->GetScenarioInducedAbort()){
        return true;
    }
    return false;
}

void
CLcExecuteDecrement::UserPostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	//
	// Decrement the sin angle.
	// This now is just for moving the target point back to 1 second 
    // from 0.5 second for the look ahead.
	const double cMIN_SIN_ANGLE = 1.0;   // degrees

	double percentLatDist = 0.0;

    double steerForce = 0;
    if (pI->m_pCurrLcCond->IsForcedLaneOffset()){
        steerForce = pI->m_forcedOffsetMaxSteerForce;
    }else{
        steerForce = pI->m_lcInfo.steeringForce;;
    }
    steerForce = max(1.0,steerForce);
    double steerForceAdjust = max(steerForce-3,0.0)/3.0;
    bool needToDecrement = pI->m_lcSinAngle > (cMIN_SIN_ANGLE + steerForceAdjust ) ;
    double oldAngle = pI->m_lcSinAngle;
	if( needToDecrement && pI->m_lcLatDistTraveled <  pI->m_lcTotalLatDist )  
	{
		percentLatDist = 1 - pI->m_lcLatDistTraveled / pI->m_lcTotalLatDist;
		
		float distFromComplete = (pI->m_lcSinAngle);
        if (distFromComplete < 0)
            distFromComplete = 0;
        //bool farFromCompletion = (pI->m_lcSinAngle - ( ( 1 -  percentLatDist) * pI->m_lcMaxSinAngle) )>= 3.0;
		if( distFromComplete >  4.0 * steerForce) //far away
		{
			pI->m_lcSinAngle = pI->m_lcSinAngle - ( steerForce);
		}else if (distFromComplete > 3.0f){//cross fade between far and near
            float interDist = (distFromComplete - 3.0f)/(4.0 * steerForce -3.0f);
            pI->m_lcSinAngle = (pI->m_lcSinAngle - ( 2.0 * steerForce )) * interDist + 
                               ( ( percentLatDist ) * pI->m_lcMaxSinAngle *  (1-interDist));
        
        }else{ //near
			pI->m_lcSinAngle = ( percentLatDist ) * pI->m_lcMaxSinAngle;
		}
	}
    if (oldAngle < pI->m_lcSinAngle)
        pI->m_lcSinAngle = oldAngle;
    float sinVal = sin( pI->m_lcSinAngle * cDEG_TO_RAD );
    float targLookAhead = 0.5 + 0.5 *(1 - sinVal);
    SetOutputTargLookAhead(targLookAhead);

    
    SetOutputTargOffset(fabs(pI->m_lcLatDistTraveled - pI->m_lcTotalLatDist) );

#ifdef DEBUG_LcEXECUTE_STATES
	gout << "======== LcExecuteDecrement ==============" << endl;
	//gout << "SinAngle: " << pI->m_lcSinAngle << endl;
	//gout << "Percent SinAngle == " << percentLatDist * pI->m_lcMaxSinAngle << endl;
	//gout << "Difference == " << pI->m_lcSinAngle - percentLatDist * pI->m_lcMaxSinAngle << endl;
	//gout << "deltaOffsetFromUrgency: " << deltaOffsetFromUrgency << endl;
	//gout << "targOffset: " << targOffset << endl << endl;
#endif
}


bool
CLcExecuteDecrement::LcDecrementNeutralizePredicate()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_LcEXECUTE_STATES
	gout << "======== [LcExecute] Decrement-->Neutralize PREDICATE ========" << endl;
#endif

	return  LaneChangeDone( pI );
}

bool
CLcExecuteDecrement::LcDecrementLcAbortPredicate()
{
	CAdoInfoPtr pI = GetInputpI();
    if (pI->m_pCurrLcCond->GetScenarioInducedAbort()){
        return true;
    }
    return false;
}