/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: ado_freedrive.cxx,v 1.109 2018/08/17 22:39:34 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    November, 1999
 *
 * Description:  Contains code for the FreeDrive HCSM.
 *
 ****************************************************************************/
#include "hcsmpch.h"
#include "curvature.h"
#include "support.h"
#include "vehdyncommand.h"
#include "controllers.h"

#include <pi_iostream>
using namespace std;

//
// Debugging macros.
//
#undef	DEBUG_FREEDRIVE
#undef	DEBUG_VEH_DYN_COMMAND
#undef	DEBUG_OFFROAD
#undef	DEBUG_SPEEDLIMIT
#undef	DEBUG_MAINTAINGAP

//
// Constants.
//
const double cSPEED_RANDOMIZATION_MIN = 0.9;
const double cSPEED_RANDOMIZATION_MAX = 1.1;

void 
CFreeDrive::Creation()
{

	m_command.type = CVehDynCommand::eCMD_NONE;
	m_listeningToCurvature = false;
	m_prevTargAccel = 0.0;
	m_waitTimeFrame = 0;
	m_refreshSpeedRandomizationFrame = 0;
	m_speedRandomization = 1.0;
	m_laneOffsetPrevFrame = -1;
    m_laneOffsetPrev = 0;
	m_lastMaintainGapId = -1;
	m_currFollEngTime = -1;
	m_initFollDist = -1;
	m_maintainGapPrevActualDist = 0.0;
	m_maintainGapDurationCounter = 0;
	m_maintainGapCounter = 0;
	m_prevDistToCurv = 0.0;
	m_lastDistanceLookAhead = -1.0f;
}  // end of Creation


void 
CFreeDrive::PreActivity()
{
	
}  // end of PreActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function returns a destination road/lane out of 
//   an intersection that's connected to the given source road/lane into
//   the intersection.
//
// Remarks:  This function accepts a source road/lane into an intersection
//   as its argument.  It then looks at all the possible destination
//   road/lanes out of the intersection that are connected through 
//   corridors to the given source road/lane.  If more than one such
//   destination road/lanes exist, it picks one of them at random.
//
// Arguments:
//   sourceLane - The source lane into the intersection.
//
// Returns:  A boolean indicating if there is a destination road/lane.
//
//////////////////////////////////////////////////////////////////////////////
static bool 
GetRandomDestLane( const CLane& sourceLane, CLane& destLane )
{

	//
	// Get the next intersection.
	//
	CIntrsctn nextIntrsctn = sourceLane.GetNextIntrsctn();

#ifdef DEBUG_FREEDRIVE
	gout << "  the next intersection is " << nextIntrsctn.GetName() << endl;
#endif

	//
	// Get all the corridors at that intersection starting from
	// the input lane.
	//
	TCrdrVec crdrVec;
	nextIntrsctn.GetCrdrsStartingFrom( sourceLane, crdrVec );

	//
	// Pick one of the corridors (the first one for now).
	//
	TCrdrVec::iterator i = crdrVec.begin();
	if ( i != crdrVec.end() ) {

		destLane = (*i).GetDstntnLn();
		return true;

	}
	else {

		return false;

	}

}  // GetRandomDestLane


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets the current speed limit.
//
// Remarks:  This function uses the given road position to
//   determine the speed limit.  For now, it looks at the number
//   of lanes to determine the speed limit.
//
// Arguments:
//   roadPos - The road position.
//
// Returns:  The speed limit in meters/second.
//
//////////////////////////////////////////////////////////////////////////////
double 
CFreeDrive::GetSpeedLimit( const CRoadPos& roadPos )
{

	double speedLimit = roadPos.GetSpeedLimit();
	bool haveSpeedLimit = speedLimit >= 0.0;
	if( haveSpeedLimit )
	{
		return speedLimit * cMPH_TO_MS;
	}
	else 
	{
		CRoad road;
		if ( roadPos.IsRoad() ) 
		{
			// on a road
			road = roadPos.GetRoad();
		}
		else 
		{
			// on an intersection...get speed limit from next road
			CCrdr crdr = roadPos.GetCorridor();
			road = crdr.GetDstntnRd();	
		}

		int numLanes = road.GetNumLanes();

		if ( numLanes <= 2 ) 
		{
			// 35 mph
			return 35.0 * cMPH_TO_MS;
		}
		else 
		{
			// 55 mph
			return 55.0 * cMPH_TO_MS;
		}
	}
}  // end of GetSpeedLimit


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Randomizes the given velocity.
//
// Remarks:  This function regenerates the randomization every few
//   seconds.
//
// Arguments:
//   vel - The given velocity.
//
// Returns:  The randomized velocity in m/s.
//
//////////////////////////////////////////////////////////////////////////////
double
CFreeDrive::RandomizeVelocity( 
			CAdoInfoPtr pI, 
			const double vel, 
			int rngStreamId 
			)
{

	//
	// Is it time to regenerate randomization.
	//
	bool regenerateVel = m_refreshSpeedRandomizationFrame <= 0;
	if( regenerateVel ) 
	{
		if( pI->m_velCntrl.distribution.m_type == CAdoParseBlock::eUNIFORM )
		{
			m_speedRandomization = m_pRootCollection->m_rng.RandomDoubleRange(
						pI->m_velCntrl.distribution.m_param1,
						pI->m_velCntrl.distribution.m_param2,
						rngStreamId
						);
		}
		else if( pI->m_velCntrl.distribution.m_type == CAdoParseBlock::eNORMAL )
		{
			m_speedRandomization = m_pRootCollection->m_rng.NormalDouble(
						pI->m_velCntrl.distribution.m_param1,
						pI->m_velCntrl.distribution.m_param2,
						rngStreamId
						);
		}
		else
		{
			m_speedRandomization = vel;
		}

		m_refreshSpeedRandomizationFrame = static_cast<long>( 
					pI->m_velCntrl.refreshTime / 
					GetTimeStepDuration() 
					);
	}

	return m_speedRandomization;

}  // end of RandomizeVelocity

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes an acceleration given an urgency.
//
// Remarks:  This function computes an acceleration given an urgency
//   between 0 and 1.  The acceleration lies between 0.01g and 0.4g
//   for situations when the given target velocity exceeds the current
//   velocity.  Otherwise, the acceleration lies between 0.01g and 0.6g.
//
//   NOTE: In either case, the returned acceleration is a positive value.
//
// Arguments:
//   currVel - The current velocity in m/s.
//   targVel - The target velocity in m/s.
//   urgency - The target urgency (between 0.0 and 1.0).
//
// Returns:  The acceleration in m/s^2.
//
//////////////////////////////////////////////////////////////////////////////
static double
ComputeAccelFromUrgency(
			const double currVel, 
			const double targVel,
			const double urgency
			)
{
	double newG;

	bool needToAccelerate = targVel >= currVel;
	if( needToAccelerate ) 
	{
		// between 0.01g and 0.4g
		newG = 0.01 + ( urgency * 0.39 );
	}
	else 
	{
		// betwen 0.01g and 0.6g
		newG = 0.01 + ( urgency * 0.59 );
	}

	double accel = newG * cGRAVITY;
	return accel;
}  // ComputeAccelFromUrgency


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes a target distance.
//
// Remarks:  This function computes a target distance given the object's
//   current velocity, target velocity, and a desired acceleration.
//
// Arguments:
//   currVel - The current velocity.
//   targVel - The target velocity.
//   urgency - The urgency with which to achieve the target velocity.
//
// Returns:  The desired target distance.
//
//////////////////////////////////////////////////////////////////////////////
static double
ComputeTargetDistance( 
			const double currVel, 
			const double targVel,
			const double urgency
			)
{

	const double cMIN_TARG_DIST = 1.0;        // meters
	const double cCRUISING_TARG_DIST = 20.0;  // meters

	double accel = ComputeAccelFromUrgency( currVel, targVel, urgency );
	double deltaVel = targVel - currVel;
	double targDist;
	
	if( fabs( accel ) > cNEAR_ZERO ) 
	{
		bool almostReachedTargVel = fabs( targVel - currVel ) < 1.0;
		if( targVel > cNEAR_ZERO && almostReachedTargVel )
		{
			targDist = cCRUISING_TARG_DIST;
		}
		else
		{
			targDist = ( 0.5 * deltaVel * deltaVel ) / accel;
		}
	}
	else 
	{
		// a large value
		targDist = 1000.0;
	}

	bool nonZeroTargVel = fabs( targVel ) > cNEAR_ZERO;
	if( nonZeroTargVel ) 
	{
		if( targDist < cMIN_TARG_DIST ) 
		{	
			targDist = cMIN_TARG_DIST;
		}
	}

#if 0
	gout << "CTD:  ";
	gout << "cv=" << currVel * cMS_TO_MPH << "mph  tv=";
	gout << targVel * cMS_TO_MPH << "mph   a=" << accel / cGRAVITY;
	gout << "G   u=" << urgency << "   td=";
	gout << targDist * cMETER_TO_FEET << "ft" << endl;
#endif

	return targDist;

}  // ComputeTargetDistance



//////////////////////////////////////////////////////////////////////////////
//
// Description: When the vehicle is off-road, this function calculates
//   the longitudunal distance ahead that should be used to compute the
///  ADO's target position.
//
// Remarks:  Assumes that the vehicle is off-road.
//
// Arguments:
//   pI - Pointer to vehicle's main information structure.
//
// Returns: A double which reprents the distance (in feet).
//
//////////////////////////////////////////////////////////////////////////////
double
CFreeDrive::ComputeOffRoadDistForTargPos( CAdoInfoPtr pI )
{
	//
	// Assuming that the ADO is off-road.
	//
	// Compute distance between current cartesian position and
	// the guidance point on the road.
	//
	CPoint3D guidePos = pI->m_offroadGuideRoadPos.GetXYZ();
	double distSq = pI->m_offroadCartPos.DistSq( guidePos );  // feet

	const double cMIN_DIST_TO_OFFROAD_POINT = 10.0;  // feet
	double adjustedDist = cMIN_DIST_TO_OFFROAD_POINT + (pI->m_objLength * 0.5);
	bool insideMinDist = distSq < ( adjustedDist * adjustedDist );

#ifdef DEBUG_OFFROAD
	gout << "offroadCartPos = " << pI->m_offroadCartPos << endl;
	gout << "guidePos       = " << guidePos << endl;
	gout << "distToGuidePoint = " << sqrt( distSq );
	gout << "ft   adjustedDist = " << adjustedDist << "ft" << endl;
#endif

	double distForTargPos;
	if( insideMinDist ) 
	{
		//
		// Check to see if I'm on a piece of road yet and, if
		// so, then check to make sure that it's the same lane
		// as the target lane.
		//
		CRoadPos roadPos( *cved, pI->m_offroadCartPos );
        if( roadPos.IsValid() ){
            if (roadPos.IsRoad()) 
            {
                if (pI->m_offroadGuideRoadPos.IsRoad()){
                    CLane guideLane = pI->m_offroadGuideRoadPos.GetLane();
                    CLane currLane = roadPos.GetLane();
                    if( guideLane == currLane ) 
                    {
#ifdef DEBUG_OFFROAD
                        gout << "^^ roadPos = " << roadPos << endl;
#endif
                        pI->m_offroad = false;
                    }
                }
            }else if (!pI->m_offroadGuideRoadPos.IsRoad()){
                CIntrsctn& guideIntr = pI->m_offroadGuideRoadPos.GetIntrsctn();
                CIntrsctn& currIntr = roadPos.GetIntrsctn();
                if (guideIntr.GetId() == currIntr.GetId()){
                    vector< pair<int,double> > crds;
                    roadPos.GetCorridors(crds);
                    for (auto itr = crds.begin(); itr != crds.end(); itr++){
                        if (pI->m_offroadGuideRoadPos.HasCorridor(itr->first)){
                             pI->m_offroad = false;
                        }
                    }
                }
            }
            double distToRoadPos = sqrt( distSq );  // feet
            distForTargPos = 20.00 - distToRoadPos;
        }else{
            distForTargPos = 0.0;
        }
	}
	else 
	{
		distForTargPos = 0.0;
	}

#ifdef DEBUG_OFFROAD
	gout << "** dist to guidance point = ";
	gout << sqrt( distSq ) << "ft" << endl;
	gout << "           distForTargPos = ";
	gout << distForTargPos << "ft" << endl;
#endif

	return distForTargPos;
}

	
//////////////////////////////////////////////////////////////////////////////
//
// Description: Calculates the target acceleration based upon the 
//   speed limit.
//
// Remarks:  
//
// Arguments:
//   pI - Pointer to vehicle's main information structure.
//   targAccel - The target acceleration (m/s^2).
//   targVel - The target velocity (m/s).
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CFreeDrive::GetSpeedLimitTargAccel( 
			CAdoInfoPtr pI, 
			double& targAccel, 
			double& targVel 
			)
{
#ifdef DEBUG_SPEEDLIMIT
	gout << MessagePrefix( pI->m_pObj->GetId() );
	gout << "======== GetSpeedLimitTargAccel ==============" << endl;
#endif

	//
	// Calculate the target velocity according to the speed limit.
	//
	bool haveUserSpecifiedTargVel = pI->m_velCntrl.targetVel >= 0.0;
	if( haveUserSpecifiedTargVel ) 
	{
		targVel = pI->m_velCntrl.targetVel;
	}
	else if( pI->m_velCntrl.followSpeedLimit )
	{
		targVel = GetSpeedLimit( pI->m_roadPos );
		//gout << targVel << endl;
		targVel = RandomizeVelocity( pI, targVel, pI->m_rngStreamId );
	}
	else if( pI->m_velCntrl.distribution.m_type == CAdoParseBlock::eFIXED )
	{
		targVel = pI->m_velCntrl.distribution.m_param1;
	}
	else
	{
		targVel = GetSpeedLimit( pI->m_roadPos );
		targVel = RandomizeVelocity( pI, targVel, pI->m_rngStreamId );
	}

#ifdef DEBUG_SPEEDLIMIT
	gout << "speedLimit:  vel = ";
	gout << targVel * cMS_TO_MPH << "mph" << endl;
#endif

	targAccel = CalcTargAccelForNormalCruising(
							pI->m_currVel,
							pI->m_prevVel,
							targVel,
							pI->m_aggressiveness,
							m_pRootCollection->m_rng,
							pI->m_rngStreamId
							);
}  // end of GetSpeedLimitTargAccel


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the target acceleration and velocity based upon
//   the MaintainGap dial settings.
//
// Remarks:  
//
// Arguments:
//   pI - Pointer to vehicle's main information structure.
//   targAccel - The target acceleration (m/s^2).
//   targVel - The target velocity (m/s).
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CFreeDrive::GetMaintainGapTargAccel(		
		CAdoInfoPtr pI,
		double& targAccel,
		double& targVel
		)
{
#ifdef DEBUG_MAINTAINGAP
	gout << MessagePrefix( pI->m_pObj->GetId() );
	gout << "======== GetMaintainGapTargAccel ==============" << endl;
	gout << "maintainGapObjId = " << pI->m_maintainGap.m_objId << endl;
#endif

	//
	// Obtain CVED and HCSM pointers to the object from which 
	// the specified has to be maintained.  This object shall be
	// referred to as the MaintainGap object.
	//
	const CDynObj* pMaintainGapObj = cved->BindObjIdToClass( 
										pI->m_maintainGap.m_objId 
										);
	if( !pMaintainGapObj ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "unable to get pointer to cved object for id = ";
		gout << pI->m_maintainGap.m_objId << endl;

		Suicide();
		return;
	}
	CHcsm* pMaintainGapHcsm = m_pRootCollection->GetHcsm( 
										pMaintainGapObj->GetHcsmId() 
										);
	if( !pMaintainGapHcsm ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "cannot get pointer maintain gap hcsm (cved id=";
		gout << pI->m_maintainGap.m_objId << ")" << endl;

		Suicide();
		return;
	}

	//
	// Compute the velocity and distance errors between me and the
	// MaintainGap object.
	//
	double myCurrVel = pI->m_pObj->GetVelImm();             // m/s
	double velErr = pMaintainGapObj->GetVel() - myCurrVel;  // m/s
	double actualDist = 0.0;

	//
	// Get the MaintainGap object's current road position.
	//
	CRoadPos maintainRoadPos;
	if( !pMaintainGapHcsm->GetMonitorByName( "RoadPos", &maintainRoadPos ) ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "unable to read RoadPos monitor from object ";
		gout << "(cved id=" << pI->m_maintainGap.m_objId << ")";
		gout << endl;

		Suicide();
		return;
	}

	if( !maintainRoadPos.IsValid() )
	{
		targAccel = 0.0;
		targVel = pI->m_currVel;

		return;
	}

#ifdef DEBUG_MAINTAINGAP
	if( GetFrame() == 72 || GetFrame() >= 137 ) 
		gout << "---------- " << GetFrame() << " ----------------------" << endl;

	gout << "maintainRoadPos = " << maintainRoadPos << endl;
	gout << "myRoadPos = " << pI->m_roadPos << endl;
#endif
	
	//
	// Figure out if we're both on the same road or not.
	//
	bool onTheSameRoad = ( 
				pI->m_roadPos.IsRoad() &&
				maintainRoadPos.IsRoad() && 
				maintainRoadPos.GetRoad() == pI->m_currRoad
				);
	if( onTheSameRoad ) 
	{
		//
		// We're on the same road...computing the distance error is easy.
		//
		CLane maintainLane = maintainRoadPos.GetLane();
		CLane myLane = pI->m_roadPos.GetLane();

		if( myLane.GetDirection() == ePOS ) 
		{
			actualDist = pI->m_roadPos.GetDistance() - maintainRoadPos.GetDistance();
		}
		else 
		{
			actualDist = maintainRoadPos.GetDistance() - pI->m_roadPos.GetDistance();
		}
	}
	else 
	{
		bool onTheSameIntrsctn = (
					!pI->m_roadPos.IsRoad() &&
					!maintainRoadPos.IsRoad() && 
					maintainRoadPos.GetIntrsctn().GetId() == 
					pI->m_roadPos.GetIntrsctn().GetId()
					);
		if( onTheSameIntrsctn )
		{
			double maintainDist = maintainRoadPos.GetDistance();
			actualDist = pI->m_roadPos.GetDistance() - maintainDist;
		}
		else
		{
			//
			// We're not on the same road or on the same intersection.  
			// Now we have to employ the help
			// of the CPath class to figure out the distance between the two
			// of us.  And, it's still not clear if the MaintainGap object
			// is ahead of me or behind me.  So, CPath::Append should work if 
			// MaintainGap object is ahead of me; otherwise, CPath::Prepend
			// should work if the MaintainGap object is behind me.
			//
			CPath maintainPath( pI->m_roadPos );
			bool appendSuccess = maintainPath.Append( maintainRoadPos ); 
			if( appendSuccess )
			{
				actualDist = -1.0 * maintainPath.GetLength();
			}
			else
			{
				bool prependSuccess = maintainPath.Prepend( maintainRoadPos );
				if( prependSuccess )
				{
					actualDist = maintainPath.GetLength();
				}
				else 
				{
					gout << MessagePrefix( pI->m_pObj->GetId(), pI->m_objName );
					gout << "unable to build path for maintain ";
					gout << "object (cved id=";
					gout << pI->m_maintainGap.m_objId << ")";
					gout << endl;
					gout << "  myRoadPos       = " << pI->m_roadPos << endl;
					gout << "  maintainRoadPos = " << maintainRoadPos << endl;
					gout << maintainPath << endl;

					pI->m_maintainGap.Reset();
				}						
			}
		}
	}

#ifdef DEBUG_MAINTAINGAP
	gout << "  actualDist = " << actualDist << " ft" << endl;
	gout << "  velErr " << velErr * cMS_TO_MPH << " mph" << endl;
#endif

	// meaning of variables
	// m_currFollEngTime - how long we have been following the current leader
	// m_lastMaintainGapId - the object we were maintaining gap from during the last frame
	//
	double mgActualDist = -1.0 * actualDist * cFEET_TO_METER;
	if ( m_lastMaintainGapId == pI->m_maintainGap.m_objId ) {
		m_currFollEngTime++;
	}
	else {
		m_currFollEngTime = 0;
		m_initFollDist    = mgActualDist;
	}
	m_lastMaintainGapId = pI->m_maintainGap.m_objId;

	CFollowInfo fi;
	m_mgParams = pI->m_FollowParams;
	m_mgParams.maintainGap.distKp       = pI->m_maintainGap.m_distKp;
	m_mgParams.maintainGap.velKp        = pI->m_maintainGap.m_velKp;
	m_mgParams.maintainGap.posAccClip   = pI->m_maintainGap.m_maxAccel;
	m_mgParams.maintainGap.negAccClip   = pI->m_maintainGap.m_maxDecel;

	m_mgParams.engageThres   = 1000; // if the actual distance is
									   // larger than this, follow won't
									   // do anything
	m_mgParams.useReactDelay = false;
	m_mgParams.folTimeMode = false;
	m_mgParams.folValue = pI->m_maintainGap.m_value * cFEET_TO_METER;
	bool maintainGapAheadOfTarget = pI->m_maintainGap.m_value < 0.0;
	if( maintainGapAheadOfTarget )
	{
		m_mgParams.maintainGap.bumpStopDist = m_mgParams.folValue * 2;
	}

	double dummy;

	bool gotFollow = FollowController(
				GetFrame(),
				GetTimeStepDuration(),
				pI->m_maintainGap.m_objId,
				fi,
				m_mgParams,
				mgActualDist,
				m_maintainGapPrevActualDist,
				m_currFollEngTime,
				m_initFollDist,
				pMaintainGapObj->GetVelImm(),
				pI->m_currVel,
				pI->m_prevVel,
				GetAccel( pMaintainGapObj ),
				0.5,  // not used
				pI->m_maintainGap.m_maxSpeed,
				targAccel,
				dummy,			// don't care about it here
				true
				);

#if 0
	gout << "***** " << pI->m_objName << " ******" << endl;
	gout << "**mgActualDist = " << mgActualDist << " m" << endl;
	gout << "**targDist     = " << m_mgParams.folValue << " m" << endl;
	gout << "**targAccel    = " << targAccel << " m/s^2" << endl;
#endif

	//
	// Check for minimum speed.
	//
	bool belowMinSpeed = (
				pI->m_currVel < pI->m_maintainGap.m_minSpeed &&
				targAccel < 0.0
				);
	if( belowMinSpeed )
	{
//		gout << "** belowMinSpeed **" << endl;
		targAccel = 0.0;
	}

	//
	// Check for disable speed.
	//
	bool disableMaintainGap = (
				pI->m_maintainGap.m_disableSpeed > 0.0 &&
				pI->m_currVel > pI->m_maintainGap.m_disableSpeed
				);
	if( disableMaintainGap )
	{
//		gout << "## disable maintainGap ##" << endl;
		targAccel = 0.0;
		pI->m_maintainGap.Reset();
	} 
#ifdef DEBUG_MAINTAINGAP
	gout << "  mgDist = " << pI->m_maintainGap.m_value << " ft" << endl;
#endif

	bool percentAroundTargetActive = pI->m_maintainGap.m_percentAroundTarget >= 0.0;
	if( percentAroundTargetActive )
	{
		//
		// Check for percent around target and time spent.
		//
		double distFromTarget = fabs( actualDist + pI->m_maintainGap.m_value );
		double envelopeDist = fabs( pI->m_maintainGap.m_value ) * pI->m_maintainGap.m_percentAroundTarget;
		bool insideTargetDist = distFromTarget < envelopeDist;

#ifdef DEBUG_MAINTAINGAP
		gout << "  distFromTarget = " << distFromTarget << "   envelop = " << envelopeDist << endl;
#endif

		if( insideTargetDist )
		{
			m_maintainGapDurationCounter++;

			double insideTime = m_maintainGapDurationCounter * GetTimeStepDuration();
			bool reachedTime = insideTime >= pI->m_maintainGap.m_distDuration;
			if( reachedTime )
			{
				targAccel = 0.0;
				pI->m_maintainGap.Reset();
				m_maintainGapCounter = 0;
				m_maintainGapDurationCounter = 0;

#if 0
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "MaintainGap complete...reset" << endl;
#endif
			}
		}
		else
		{
			// reset counter
			m_maintainGapDurationCounter = 0;
		}
	}

	if( pI->m_maintainGap.m_duration >= 0.0 )
	{
		//
		// Reset when duration counter has expired.
		//
		m_maintainGapCounter++;

		double insideTime = m_maintainGapCounter * GetTimeStepDuration();
		bool reachedTime = insideTime >= pI->m_maintainGap.m_duration;
		if( reachedTime )
		{
			targAccel = 0.0;
			pI->m_maintainGap.Reset();
			m_maintainGapCounter = 0;
			m_maintainGapDurationCounter = 0;

#if 0
			gout << MessagePrefix( pI->m_pObj->GetId(), pI->m_objName );
			gout << "MaintainGap max duration expired...reset" << endl;
#endif
		}
	}


// 


	m_maintainGapPrevActualDist = mgActualDist;

	//
	// Have to fake a value for targVel.
	//
	if( fabs( targAccel ) < 0.1 ) 
	{
		targVel = pI->m_currVel;
	}
	else if( targAccel > 0.0 )
	{
		targVel = pI->m_currVel + 3.0;
	}
	else
	{
		targVel = pI->m_currVel - 3.0;
	}
    


#ifdef DEBUG_MAINTAINGAP
	gout << "  targAccel = " << targAccel << " m/s2    targVel = ";
	gout << targVel * cMS_TO_MPH << " mph" << endl;
#endif
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the target acceleration based upon the 
//   curvature.
//
// Remarks:  
//
// Arguments:
//   pI - Pointer to vehicle's main information structure.
//   targAccel - The target acceleration (m/s^2).
//   targVel - The target velocity (m/s).
//
// Returns: A boolean indicating if a valid target acceleration
//   were found.
//
//////////////////////////////////////////////////////////////////////////////
bool
CFreeDrive::GetCurvatureTargAccel(
		CAdoInfoPtr pI,
		double& targAccel,
		double& targVel
		)
{
	//
	// Get the velocity associated with the curvature.
	//
	// If the velocity chosen during the previous the frame was 
	// due to curvature, then use a lower acceleration threshold 
	// for gettting curvature.  This is to avoid oscillations 
	// where the ADO is slowing down due to curvature and then 
	// speeding up once it exceeds the acceleration threshold.
	//
	const double cSTART_MIN_ACCEL = -2.0;        // m/s^2
	const double cCONTINUE_MIN_ACCEL = 0.3;      // m/s^2
	double minAccel;
	if ( m_listeningToCurvature ) 
	{
		minAccel = cCONTINUE_MIN_ACCEL;
	}
	else 
	{
		minAccel = cSTART_MIN_ACCEL;
	}

	double targDist;
	bool gotCurv = pI->m_curvature.GetCurvature( minAccel, targVel, targDist);

	//
	// Now, check to see if the distance to the current curvature
	// point is close to the previous one.  If not, then use the
	// cSTART_MIN_ACCEL to obtain a new curvature point.
	//
	// This fixes problem with curvature where vehicle wouldn't 
	// speed up properly between 2 curvature points that required 
	// the vehicle to be slow but were placed far apart such that 
	// the vehicle should speed up in between.
	//
	if( m_listeningToCurvature )
	{
		//gout << "prevDist = " << m_prevDistToCurv * cMETER_TO_FEET;
		//gout << "ft   targDist = " << targDist * cMETER_TO_FEET << "ft";
		//gout << endl;
		const double cMIN_CURV_DIFF = 30.0;  // m
		double curvDiff = fabs( m_prevDistToCurv - targDist );
		bool restartCurv = curvDiff > cMIN_CURV_DIFF;
		if( restartCurv )
		{
			gotCurv = pI->m_curvature.GetCurvature( 
											cSTART_MIN_ACCEL, 
											targVel, 
											targDist
											);
		}
	}

	m_prevDistToCurv = targDist;

	if( gotCurv ) 
	{

#ifdef DEBUG_FREEDRIVE
		gout << "  curv:  vel = " << targVel * cMS_TO_MPH << "mph";
		gout << "   dist = " << targDist * cMETER_TO_FEET << "ft" << endl;
#endif

		//
		// HACK:  Making sure that the curvature velocity is a
		// minimum of 10 mph.  This hack has been place here since
		// CVED seems to be returning artificially low curvaturs
		// for 90 degree curves on roads.
		//
		const double cMIN_CURV_VEL = 10.0 * cMPH_TO_MS;
		double cMinCurvVel = cMIN_CURV_VEL;

		if( targVel < cMinCurvVel )  targVel = cMIN_CURV_VEL;

		//
		// Subtract half of the length for the curvature.  We want
		// the vehicle to achieve the curvature velocity by the time
		// the front edge of the vehicle hits the exact spot....this
		// is especially important for long vehicles like buses.
		//
#if 0
		gout << "objLength = " << pI->m_objLength << "    " <<;
		gout << pI->m_objLength * 0.5 * cFEET_TO_METER << endl;
#endif
		targDist -= pI->m_objLength * 0.5 * cFEET_TO_METER;
		if ( targDist < 0.0 )  targDist = 0.0;
		
		float aggr = pI->m_aggressiveness;
		aggr = max(min(1.0f,aggr),0.0f);
		aggr = aggr*1 + 0.5;
		targVel = targVel * aggr;

		aggr = pI->m_aggressiveness;
		aggr = max(min(1.0f,aggr),0.0f);
		aggr = (1.0-aggr)*1 + 0.5;  
		targDist = targDist * aggr;

		targAccel = LinearDecelController( 
								pI->m_currVel, 
								targVel, 
								targDist * cFEET_TO_METER 
								);


		return true;
	}

	return false;
}

CVector3D calcFowardDifferential (			
	        CAdoInfoPtr pI, 
			CRoadPos& targRoadPos,
			CVector3D& lastTargetTangent
			){
    //we are not on a sharp curve or recovering from a sharp curve
    CVector3D vec1 = pI->m_roadPos.GetTangentInterpolated();
	float yawRate = pI->m_pObj->GetYawRate();
    CVector3D vec2 = targRoadPos.GetTangentInterpolated()*0.1 + lastTargetTangent*0.9;
    CVector3D vec3 = (vec1-vec2);
	lastTargetTangent = vec2;
	return vec3;

}
//////////////////////////////////////////////////////////////////////////////
///\brief
///		adjust the target point for the curve of the road
///\remark
///		This function takes the current tangent (the road pos tangent not the
///		vehicles tangent), and the tangent of the target point, it takes the 
///		difference, and then bumps target point. It assumed the target point
///		is contained within the current path. It needs this to figure out 
///		what crdr to use.
///
///\par pI	- Pointer to vehicle's main information structure
///\par targRoadPos - The road position to adjust.
//////////////////////////////////////////////////////////////////////////////
void
CFreeDrive::AdjustTargRoadPosForCurves( 
			CAdoInfoPtr pI, 
			CRoadPos& targRoadPos 
			)
{
	const double cMAX_OFFSET = 4.9;
	const double cOFFSET_INCREMENT = 0.1;
	double currentMaxOffset = cMAX_OFFSET;

    //we are not on a sharp curve or recovering from a sharp curve
	CVector3D fowardDif;
	if (m_laneOffsetPrevFrame > 0) {
		fowardDif = calcFowardDifferential(pI, targRoadPos, m_tangentPrev);
	}
	else {
		CVector3D vec1 = pI->m_roadPos.GetTangentInterpolated();
		float yawRate = pI->m_pObj->GetYawRate();
		CVector3D vec2 = targRoadPos.GetTangentInterpolated();
		CVector3D vec3 = (vec1 - vec2);
		m_tangentPrev = vec2;
		fowardDif = vec3;
	}
	//check to see if we need to reduce our max offset due to lane width
	if (targRoadPos.IsRoad()){
		double laneWidth = targRoadPos.GetLane().GetWidth();
		if (laneWidth/2 - 0.5 < currentMaxOffset){
			currentMaxOffset = laneWidth/2 - 0.5;
		}
	}else{
		double dist = targRoadPos.GetDistance();
		double laneWidth = targRoadPos.GetCorridor().GetWidth(dist);
		if (laneWidth/2 - 0.5 < currentMaxOffset){
			currentMaxOffset = laneWidth/2 - 0.5;
		}
	}
	CPoint3D currPos = pI->m_roadPos.GetXYZ();
	CPoint3D targPos = targRoadPos.GetXYZ();	
			
	CVector3D targetSeg(targPos - currPos);
    float laneWidth = 0;
   
	//Scaling this by 1/3 the segment length
	//was found through experimentations
	//to work well, this should be parmaiterized 
	fowardDif.Scale(targetSeg.Length()/3);

    //if (fowardDif.Length() > currentMaxOffset){
    //    fowardDif.Scale(currentMaxOffset/fowardDif.Length());
    //}
    if (m_laneOffsetPrevFrame > 0){
        CVector3D vecDif  = fowardDif - m_curveOffsetPrev;
        double vec_length = vecDif.Length();
        if (vecDif.Length() > 0.2f){
            vecDif.Scale(0.2/vec_length);
        }
        fowardDif = m_curveOffsetPrev + vecDif;
    }
    CPoint3D projectedXYZ  = targRoadPos.GetBestXYZ();
	CRoadPos tempRoadPos(targRoadPos);
	tempRoadPos.SetXYZ(projectedXYZ + fowardDif); //set new target point
	if (tempRoadPos.IsValid()){
		targRoadPos = tempRoadPos;
		//now we will want to allign the new target point with the same crdr settings as 
		//the path and/or current pos.
		int crdId = -1;
        if (!targRoadPos.IsRoad()){
            crdId = targRoadPos.GetCorridor().GetId();
        }
        if (!targRoadPos.IsRoad() &&  !pI->m_roadPos.IsRoad() && targRoadPos.GetIntrsctn().GetId() == targRoadPos.GetIntrsctn().GetId() ){
			targRoadPos.SetCrdPriority(&pI->m_roadPos.GetCorridor());
		}else if (pI->m_roadPos.IsRoad() && !targRoadPos.IsRoad()){
			int crdrId = -1;
			//find the crdr the ADO is about to travel on 
			if (pI->m_pPath->GetCrdrFromIntrscn(targRoadPos.GetIntrsctn().GetId(),crdrId,&targRoadPos,pI->m_roadPos.GetLane().GetId())){
				CCrdr cdr(targRoadPos.GetIntrsctn(),crdrId);
				targRoadPos.SetCrdPriority(&cdr);
			}
		}
	}else{ //our projection is off the road, just use the same offset as last time......
		targRoadPos.SetOffset(m_laneOffsetPrev);
	}

	//rate limmit our change in target offset
	//instead of offset based, this should look at angle
	//between the target point and the driver's target
    //our problems are mostly from large changes in
    //curvature from crossing road crdr boundries
    //which make using the offset vary hard
    //so this code right now is not too usefull
	float currentOffset = targRoadPos.GetOffset();
	m_laneOffsetPrev = currentOffset; 
	m_laneOffsetPrevFrame = GetFrame();
	m_curveOffsetPrev = fowardDif;
        
}
		


/////////////////////////////////////////////////////////////////////////////
//
// Caclulates the instantaneous lane deviation for the 'Ramps' model
//
//
static double
ProduceDeviationRampsModel(
	  CAdoInfoPtr       pI,
	  int               frame		// current frame
	  )
{
	CRandLaneDevInfo& info = pI->m_randLaneDev;
	double   delta;			// time, in sec, since start of this waveform
	double   ofs;

	delta = (frame - info.m_RampStartTime) / 30.0;

//	printf("\nIn Ramp: delta = %.3f, ", delta);

	if ( delta <= info.m_Rise ) {	// rising part
//		printf("rising, ");
		ofs = info.m_RampAmp * delta * info.m_Pos / info.m_Rise;
	}
	else if ( delta <= info.m_Rise + info.m_Fall ) {
		delta -= info.m_Rise;
//		printf("falling, new delta=%.2f, ", delta);

		ofs = info.m_Pos * info.m_RampAmp - info.m_RampAmp * delta * info.m_Pos / info.m_Fall;
	}
	else if ( delta <= info.m_Rise + info.m_Fall + info.m_Idle ) {
//		printf("idle, ");
		ofs = 0.0;
	}
	else {	// re-randomize for next waveform
		info.m_RampStartTime = frame;

		info.m_Rise = info.m_Rnd.RandomDoubleRange(info.m_Rise1, info.m_Rise2, 0);
		info.m_Fall = info.m_Rnd.RandomDoubleRange(info.m_Fall1, info.m_Fall2, 0);
		info.m_Idle = info.m_Rnd.RandomDoubleRange(info.m_Idle1, info.m_Idle2, 0);
		info.m_RampAmp1 = info.m_Rnd.RandomDoubleRange(info.m_RampAmp1, info.m_RampAmp2, 0);
		info.m_Pos  = info.m_Rnd.RandomLongRange(0, 1, 0) ? 1.0 : -1.0;

//		printf(" ** re-randomized RAMP: rise/fall/idle/amp/sign = %.2f, %.2f, %.2f, %.2f, %.1f\n",
//			info.m_Rise, info.m_Fall, info.m_Idle, info.m_RampAmp, info.m_Pos);
		ofs = 0.0;
	}

//	printf("Ramp out = %.4f\n", ofs);
	return ofs;
}


/////////////////////////////////////////////////////////////////////////////
//
// Caclulates the instantaneous lane deviation for the 'Sin' model
//
//
static double
ProduceDeviationSinModel(
	  CAdoInfoPtr       pI,
	  int               frame
	 )
{
	CRandLaneDevInfo& info = pI->m_randLaneDev;
	return info.m_Bias + info.m_SinAmp * sin(info.m_Frequ * frame * 0.033 + info.m_Phase);
}


/////////////////////////////////////////////////////////////////////////////
//
// Implements the randomized lane deviation model.  It operates as follows:
// A counter determines when the whole model is re-randomized.  During
// randomization, the actual model and parameters are selected randomnly
// from within user specified intervals.  Then the actual model and
// values are used to compute the deviation which is placed in the
// targRoadPos.
//
// The function does not imlement any smoothing or any other variation based
// on the road type.
//
void
CFreeDrive::AdjustTargRoadPosForLaneDevVariance( 
			CAdoInfoPtr pI, 
			CRoadPos& targRoadPos 
			)
{
	double             ofs  = 0.0;
	CRandLaneDevInfo&  info = pI->m_randLaneDev;
	
//	printf("---> Offset in = %.2f, ", 	targRoadPos.GetOffset());
	double currOffset = targRoadPos.GetOffset();
	// First, check to see if we need to re-randomize the values
	// When we re-randomize, we do it for all the parameters
	if ( pI->m_randLaneDev.m_Timer <= 0 && fabs(pI->m_randLaneDev.m_Last) < 0.1 ) {
//		info.m_Model = (TLaneDevModel)pI->m_randLaneDev.m_Rnd.RandomLongRange(0, 2, 0);

		if ( info.m_Model == ELaneDevRamps || info.m_Model == ELaneDevRampsSin ) {
			info.m_Rise = info.m_Rnd.RandomDoubleRange(info.m_Rise1, info.m_Rise2, 0);
			info.m_Fall = info.m_Rnd.RandomDoubleRange(info.m_Fall1, info.m_Fall2, 0);
			info.m_Idle = info.m_Rnd.RandomDoubleRange(info.m_Idle1, info.m_Idle2, 0);
			info.m_RampAmp = info.m_Rnd.RandomDoubleRange(info.m_RampAmp1, info.m_RampAmp2, 0);
			info.m_Pos  = info.m_Rnd.RandomLongRange(0, 1, 0) ? 1.0 : -1.0;
		}

		if ( info.m_Model == ELaneDevSin || info.m_Model == ELaneDevRampsSin ) {
			info.m_Bias   = info.m_Rnd.RandomDoubleRange(info.m_Bias1, info.m_Bias2, 0);
			info.m_SinAmp = info.m_Rnd.RandomDoubleRange(info.m_SinAmp1, info.m_SinAmp2, 0);
			info.m_Frequ  = info.m_Rnd.RandomDoubleRange(info.m_Frequ1, info.m_Frequ2, 0);
			info.m_Phase  = info.m_Rnd.RandomDoubleRange(info.m_Phase1, info.m_Phase2, 0);
		}

		info.m_ReevalTime = info.m_Rnd.RandomDoubleRange(info.m_ReevalTime1, info.m_ReevalTime2, 0);

		info.m_Timer         = static_cast<long>(info.m_ReevalTime * 30);		// HACK HACK
		info.m_RampStartTime = GetFrame();
#if 0
		fprintf(stderr, "\n ** re-randomized at top, new model is %d, eval time %.1f\n", info.m_Model, info.m_ReevalTime);
		fprintf(stderr, "    Ramp: rise/fall/idle/amp = %.2f, %.2f, %.2f, %.2f, %.1f\n",
			info.m_Rise, info.m_Fall, info.m_Idle, info.m_RampAmp, info.m_Pos);
		fprintf(stderr, "    Sin: bias/amp/frequ/phase = %.2f, %.2f, %.2f, %.2f\n",
			info.m_Bias, info.m_SinAmp, info.m_Frequ, info.m_Phase);
#endif
	}

	if ( info.m_Model == ELaneDevRamps ) {
		ofs = ProduceDeviationRampsModel(pI, GetFrame());
	}
	else if ( info.m_Model == ELaneDevSin ) {
		ofs = ProduceDeviationSinModel(pI, GetFrame());
	}
	else if ( info.m_Model == ELaneDevRampsSin ) {
		int x = GetFrame();
		ofs = ProduceDeviationRampsModel(pI, x) + ProduceDeviationSinModel(pI, x);
	}

	pI->m_randLaneDev.m_Timer--;

//	printf("%d %.2f %.2f\n", pI->m_randLaneDev.m_Timer, ofs, targRoadPos.GetOffset());

	pI->m_randLaneDev.m_Last = ofs;
	targRoadPos.SetOffset(ofs+currOffset);
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Adjusts the offset of the given roadPos in reponse to the
//   setting of the ForcedLaneOffset dial.
//
// Remarks:  
//
// Arguments:
//   pI - Pointer to vehicle's main information structure.
//   targRoadPos - The road position to adjust.
//
// Returns: 
//
//////////////////////////////////////////////////////////////////////////////
void
CFreeDrive::AdjustTargRoadPosForForcedOffset( 
			CAdoInfoPtr pI, 
			CRoadPos& targRoadPos,
            double &MaxSteerRateAdjustment
			)
{
	//
	// If ForcedLaneOffset dial is set, modify the targRoadPos with
	// input forced offset.
	// 
	if( pI->m_haveForcedLaneOffset  && !pI->m_reporjectAndResetLaneOffsetButton)
	{
		//
		// Conditions to set forced offset:
		// 1. If lane change behaviors finish doing forcedOffset,
		//    we directly set forced offset in freedrive. 
		// 2. Freedrive should not set offset for the first
		//    10 forced Offset Frames.
		//
		bool setForcedLaneOffset = pI->m_forcedOffsetFrameCounter > pI->m_forcedOffsetDelay;
		// gout << "counter = " << pI->m_forcedOffsetFrameCounter << endl;
		if( setForcedLaneOffset ) 
		{
            targRoadPos.SetOffset( pI->m_forcedLaneOffset );
            MaxSteerRateAdjustment = (pI->m_forcedLaneOffsetTurnRate) * cDEG_TO_RAD;
		//gout << "** targRoadPos = " << targRoadPos << endl;
		}else{
            targRoadPos.SetOffset(pI->m_prevForcedLaneOffset);
        }
		pI->m_forcedOffsetFrameCounter++;
	}
}  // end of AdjustTargRoadPosForForcedOffset


/////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the vehicle's target velocity/distance and
//   target position.
//
// Remarks:  This function calculates the vehicle's target velocity and
//   distance based on the road's speed limit and curvature.  It 
//   the target position by calculating it to be x seconds ahead of the
//   current position.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CFreeDrive::PostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_FREEDRIVE
	gout << MessagePrefix( pI->m_pObj->GetId() );
	gout << "======== FREEDRIVE POST-ACTIVITY ==============" << endl;
#endif

#ifdef DEBUG_FREEDRIVE
	gout << "  currRoadPos = " << pI->m_roadPos << endl;
#endif

	//
	// Commonly used information.
	//
	double currVel = pI->m_pObj->GetVelImm();
	double distForTargPos;  // feet

	//
	//  Compute a distance to move ahead for the target position.
	//
	if( pI->m_offroad )
	{	
		distForTargPos = ComputeOffRoadDistForTargPos( pI );
	}
	else 
	{
		//
		// On the road.
		//
		// Calculate a distance from the current position that points to
		// the next target position.
		//
		// This code is problematic, we really should have about 2 seconds
		// of look ahead, but the lane change code is expecting a 1 second 
		// look ahead, and this could push the ideal target point in a 
		// different lane, that we are not prepaired to deal with
		//
		const double cSECONDS_AHEAD = 1.0; 
		const double cMIN_DIST_AHEAD = 2.0;  // feet

		const double cMAX_DIST_AHEAD = 102.5; // feet (around 70 mph)
		if (m_lastDistanceLookAhead < 0) {
			distForTargPos = currVel * cSECONDS_AHEAD * cMETER_TO_FEET;
		}
		else{
			pI->m_aggressiveness;
			float timeLookAhead = cSECONDS_AHEAD ;
			distForTargPos = currVel * timeLookAhead * cMETER_TO_FEET;
			distForTargPos =  distForTargPos*.035 +  m_lastDistanceLookAhead *.965;
			m_lastDistanceLookAhead =distForTargPos;
	    }
		double distToFrontBumper = pI->m_objLength * 0.5;
		double minDist = cMIN_DIST_AHEAD + distToFrontBumper;
		double maxDist = cMAX_DIST_AHEAD + distToFrontBumper;
		if( distForTargPos < minDist )  distForTargPos = minDist;
		if( distForTargPos > maxDist )  distForTargPos = maxDist;
	}

#ifdef DEBUG_FREEDRIVE
	gout << "  distToTravelForTargPos = " << distForTargPos << endl;
#endif

	//
	// Calculate a target position from the current position using the
	// distance calculated above.
	//
	CRoadPos targRoadPos;
	bool haveValidPath = pI->m_pPath->Size() > 0;

	if( !haveValidPath ) 
	{
		//
		// I don't have a path.
		//
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "I have no path...[SUICIDE]" << endl;

		Suicide();
		return;
	}
	else 
	{
		//
		// I have a path.
		//
		CPath::ETravelCode code;
		CRoadPos roadPosZeroOffset = pI->m_roadPos;
		roadPosZeroOffset.SetOffset( 0.0 );
		CLane lane;

		bool tryAgain = false;
		do
		{
			code = pI->m_pPath->Travel(
						distForTargPos, 
						roadPosZeroOffset,
						targRoadPos, 
						lane
						);
			switch ( code ) 
			{
			case CPath::eCV_TRAVEL_ERROR :
				{
					//
					// Try to regenerate the path.  This will keep the
					// ADO from dying.
					//
					if( !tryAgain )
					{
						gout << MessagePrefix( pI->m_pObj->GetId(), pI->m_objName );
						gout << "need to regenerate path..." << endl;
						gout << "  roadPos = " << pI->m_roadPos << endl;
						gout << "  roadPosZeroOffset = " << roadPosZeroOffset << endl;
						gout << "  distForTargPos = " << distForTargPos << endl;
						if( lane.IsValid() )  gout << "  lane = " << lane << endl;
						gout << "  oldPath[" << pI->m_pPath->Size() << "]:" << endl;
						gout << *pI->m_pPath;

						delete pI->m_pPath;
						pI->m_pPath = new CPath( *cved );

						NewPath( pI );
						ExtendPath( pI );
						pI->m_pPath->Prepend( 1.0 );
						if( !m_pRootCollection->m_sDisableCurvature )
						{
							pI->m_curvature.InitializeCurvature( 
													pI->m_roadPos, 
													*pI->m_pPath 
													);
							pI->m_curvature.ResetBuckets();
						}

						gout << "new path is..." << endl;
						gout << *pI->m_pPath;

						tryAgain = true;
					}
					else
					{
						gout << MessagePrefix( pI->m_pObj->GetId(), pI->m_objName );
						gout << "error with path travel   [SUICIDE]" << endl;
						gout << "  road position = " << roadPosZeroOffset << endl;
						gout << "  cart position = " << roadPosZeroOffset.GetXYZ() << endl;
						gout << "  distForTargPos = " << distForTargPos << " ft" << endl;
						gout << endl;

						gout << "  path:" << endl;
						gout << *(pI->m_pPath) << endl;

						Suicide();
						return;
					}
				}

				break;

			case CPath::eCV_TRAVEL_NOT_FOUND :

				tryAgain = false;

				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "initial point not found" << endl;

				gout << "dist from current pos = " << distForTargPos << endl;
				gout << "roadPos = " << roadPosZeroOffset << endl;
				gout << "cartPos = " << roadPosZeroOffset.GetXYZ() << endl;
				gout << "path:" << endl;
				gout << *(pI->m_pPath) << endl;

				Suicide();
				return;

			case CPath::eCV_TRAVEL_OK :

				tryAgain = false;
				break;

			case CPath::eCV_TRAVEL_LANE_CHANGE :

				//
				// Need to do a lane change sometime.
				//
				tryAgain = false;
				break;

			case CPath::eCV_TRAVEL_END_OF_PATH :
				{
					//
					// Reached end of path....need to extend path.
					//
					bool unableToAppendPath = !pI->m_pPath->Append( 300.0 );
					if( unableToAppendPath ) 
					{
						gout << MessagePrefix( pI->m_pObj->GetId() );
						gout << "path append failed" << endl;

						Suicide();
						return;
					}
					
					tryAgain = false;
				}
				break;

			default:

				tryAgain = false;

				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "unknown ETravelCode case = " << code << endl;

				Suicide();
				return;
			}
		
		}
		while( tryAgain );

	}




#ifdef DEBUG_FREEDRIVE
	gout << "  targRoadPos = " << targRoadPos << endl;
#endif

	//
	// Get the target velocity and distance.
	//
	double targAccel;      // m/s^2
	double targVel;        // m/s 
    double maxTurn = -1.0;        //rads/s

	if( !pI->m_offroad )
	{
		//prioritize the crdr based on the path	    
		//if we are both on a crdr
		if (!pI->m_offroad && !targRoadPos.IsRoad() && !pI->m_roadPos.IsRoad() && 
				targRoadPos.GetIntrsctn().GetId() == pI->m_roadPos.GetIntrsctn().GetId() ){
                auto laneID = targRoadPos.GetCorridor().GetSrcLn().GetId();
                auto intrID =targRoadPos.GetIntrsctn().GetId();
                int crdrId = 0;
                if (pI->m_pPath->GetCrdrFromIntrscn(intrID,crdrId,&targRoadPos,laneID)){
                    CCrdr cdr(targRoadPos.GetIntrsctn(),crdrId);
                    targRoadPos.SetCrdPriority(&cdr);
                }

                 //    GetCrdrFromIntrscn
				targRoadPos.SetCrdPriority( &pI->m_roadPos.GetCorridor());
		}else if (pI->m_roadPos.IsRoad() && !targRoadPos.IsRoad()){
			int crdrId = -1;
            auto laneID = targRoadPos.GetCorridor().GetSrcLn().GetId();
            auto intrID =targRoadPos.GetIntrsctn().GetId();
			//find the crdr the ADO is about to travel on 
			if (pI->m_pPath->GetCrdrFromIntrscn(intrID,crdrId,&targRoadPos,laneID)){
				CCrdr cdr(targRoadPos.GetIntrsctn(),crdrId);
				targRoadPos.SetCrdPriority(&cdr);
			}
		}

		vector<TAccelInfo> accelList;

		//
		// Calculate the speed limit velocity and dist.  Unless, 
		// the MaintainGap dial has been specified.  In that case,
		// compute speed and dist according to MaintainGap dial 
		// settings.
		//
		bool maintainGapActive = pI->m_maintainGap.IsActive();
		if( !maintainGapActive ) 
		{
			//
			// Get the speed limit or target velocity.
			//
			TAccelInfo node;
			GetSpeedLimitTargAccel( pI, node.accel, node.vel );
//			gout << "SL: vel = " << node.vel * cMS_TO_MPH << "mph   ";
//			gout << "accel = " << node.accel << "m/s2" << endl;
			accelList.push_back( node );
		}
		else 
		{
			//
			// The MaintainGap dial has been set.
			//
			TAccelInfo node;
			GetMaintainGapTargAccel( pI, node.accel, node.vel );
//			gout << "MG: vel = " << node.vel * cMS_TO_MPH << "mph   ";
//			gout << "accel = " << node.accel << "m/s2" << endl;
			accelList.push_back( node );
		}

		bool gotCurv = false;
		TAccelInfo node;
		if( !m_pRootCollection->m_sDisableCurvature )
		{
			//
			// Get the target velocity and distance associated with the
			// road curvature.
			//
			gotCurv = GetCurvatureTargAccel( pI, node.accel, node.vel );

			if( gotCurv )  
			{
				//gout << "CV: vel = " << node.vel * cMS_TO_MPH << "mph   ";
				//gout << "accel = " << node.accel << "m/s2" << endl;
				accelList.push_back( node );
			}
		}

		ResolveAccelInfoConservative( accelList, targAccel, targVel );

		if( gotCurv )
		{
			double accelDiff = fabs( targAccel - node.accel );
			m_listeningToCurvature = accelDiff < 0.01;
		}
		else
		{
			m_listeningToCurvature = false;
		}
		if ( m_listeningToCurvature ) {
			pI->m_maxAccelDueToCurv = node.accel;
		}
		else {
			pI->m_maxAccelDueToCurv = 100.0;
		}

	}  // end if not offroad
	else 
	{
		//
		// ADO is currently off the road.  Try to reach a velocity of
		// 10mph and maintain that velocity until ADO is on the road.
		//
		if( pI->m_currVel < 10.0 * cMPH_TO_MS )
		{
			targAccel = 0.1;  // m/s^2
		}
		else
		{
			targAccel = 0.0;
		}

		pI->m_maxAccelDueToCurv = 100.0;

		targVel = 10.0 * cMPH_TO_MS;  // m/s
	}  // end else offroad

	//
	// For sharp curves, the vehicle should try to hug the outside of the
	// curve in order to avoid cutting across other lanes or off-road areas.
	//
	//adjust the targ pos for curve

	AdjustTargRoadPosForCurves( pI, targRoadPos );

	//
	// Adjust the offset for the ForcedLaneOffset dial.
	//
	AdjustTargRoadPosForForcedOffset( pI, targRoadPos, maxTurn );
    
	//
	// Adjust the offset for randomizing lane deviation
	//
	bool dontUseLaneDev = 
		(pI->m_DisableLaneDevWhenForcedOffsetDial && pI->m_haveForcedLaneOffset) 
		|| pI->m_randLaneDev.m_Enable == false;



	if ( ! dontUseLaneDev ){
		AdjustTargRoadPosForLaneDevVariance( pI, targRoadPos );
		if (fabs(targRoadPos.GetCurvature()) > 100.0 &&(pI->m_roadPos.GetCurvature() < 35000.0f || targRoadPos.GetCurvature() < 35000.0f) && targRoadPos.IsRoad()){
			targRoadPos.SetXY(targRoadPos.GetXYZ());
		}else{
			targRoadPos.GetXYZ();
		}
	}



    CPoint3D targPos = targRoadPos.GetVeryBestXYZ();
	CRoadPos targPosP1(targRoadPos);
	CRoadPos targPosP2(targRoadPos);
	CLane tempLane;
	if (pI->m_pPath){
		if (CPath::eCV_TRAVEL_OK <= pI->m_pPath->Travel(1.5,targRoadPos,targPosP1,tempLane)){
			if (CPath::eCV_TRAVEL_OK <= pI->m_pPath->Travel(-1.5,targRoadPos,targPosP2,tempLane)){
				CPoint3D targPos1 = targPosP1.GetVeryBestXYZ();
				CPoint3D targPos2 = targPosP2.GetVeryBestXYZ();
				targPos.m_x = (targPos1.m_x + targPos.m_x + targPos2.m_x)/3.0;
				targPos.m_y = (targPos1.m_y + targPos.m_y + targPos2.m_y)/3.0;
				targPos.m_z = (targPos1.m_z + targPos.m_z + targPos2.m_z)/3.0;
			}
		}
	}

//#ifdef DEBUG_FREEDRIVE

	//
	// Write values to the HCSM's outputs.
	//
	SetOutputTargPos( targPos );

	//
	// Still need the TargVel for LaneChange.
	//
	SetOutputTargVel( targVel );

	//
	// These are the 2 new outputs:  TargAccel and TargSteer.  TargAccel
	// will replace TargVel and TargDist.  TargSteer will replace TargPos.
	//
	bool haveAccel = true;
	if( haveAccel )
	{
		SetOutputTargAccel( targAccel );
	}
	else
	{
		SetOutputTargAccelNoValue();
	}

	bool haveSteer = false;
	if( haveSteer )
	{
		double targSteer = 0.0;  // this should be a valid value
		SetOutputTargSteer( targSteer );
	}
	else
	{	
		SetOutputTargSteerNoValue();
	}
    if (maxTurn> 0){
        SetOutputMaxSteer(maxTurn);
    }else{
        SetOutputMaxSteerNoValue();
    }

	m_prevTargAccel = targAccel;

	m_refreshSpeedRandomizationFrame--;

#if 0
	gout << "FrDr: CurrPos: " << pI->m_pObj->GetPos() << endl;
	gout << "FrDr: CurrVel: " << pI->m_pObj->GetVelImm() << endl;
	gout << "FrDr: TargPos: " << targPos << endl;
	gout << "FrDr: TargAccel: " << targAccel << endl;
	gout << "FrDr: TargVel: " << targVel << endl << endl;
#endif



}  // PostActivity

