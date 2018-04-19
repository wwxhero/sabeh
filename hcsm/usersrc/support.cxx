/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: support.cxx,v 1.21 2013/10/31 17:29:49 iowa\oahmad Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    May, 2000
 *
 * Description:  Contains supporting code that may be used by any HCSM.
 *
 ****************************************************************************/

#include "controllers.h"
#include "support.h"
#include "util.h"

#include <pi_iostream>
using namespace std;

#undef DEBUG_EXTEND_PATH

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates an acceleration.
//
// Remarks:  Given an initial velocity, final velocity and a distance, 
//   this function calculates the  resulting acceleration.
//
// Arguments:
//   cInitVel  -- The object's (initial) velocity (in m/s).
//   cFinalVel -- The curvature's (final) velocity (in m/s).
//   cDist     -- Distance to the curvature (in meters).
//
// Returns: The acceleration in the units m/s^2.
//
//////////////////////////////////////////////////////////////////////////////
double
CalcAccel( 
			const double cInitVel, 
			const double cFinalVel,
			const double cDist
			)
{

	double velDiff = ( cFinalVel * cFinalVel ) - ( cInitVel * cInitVel );

	//
	// Protect from divide by zero.
	//
	double accel;
	if ( cDist > cNEAR_ZERO ) {

		accel = velDiff / ( 2.0 * cDist );

	}
	else {

		accel = 1000.0;  // a large value

	}

	return accel;

}  // end of CalcAccel


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates an acceleration for normal cruising situations.
//
// Remarks: 
//
// Arguments:
//   currVel   -- The current velocity (m/s).
//   prevVel   -- The previous velocity (m/s).
//   targVel   -- The target velocity (m/s/).
//   aggressiveness -- The aggressivenss of the vehicle (0.0 ---> 1.0).
//
// Returns: The acceleration in the units m/s^2.
//
//////////////////////////////////////////////////////////////////////////////
double
CalcTargAccelForNormalCruising(
			const double currVel,
			const double prevVel,
			const double targVel,
			const double aggressiveness,
			CRandNumGen& rndNumGen,
			const int rndStream
			)
{

	return SpeedMaintenanceController(
						currVel, 
						prevVel,
						targVel,
						aggressiveness);

#if 0
	//
	// Calculate the acceleration needed to get to the target velocity.
	//
	const double cMIN_VEL_DIFF = 5.0;  // mph
	const double cMIN_PERCENT_DIFF = 0.05;

	double velDiff = targVel - currVel;
	double velDiffPercent = velDiff / currVel;

	bool significantAccelNeeded = (
				velDiffPercent > cMIN_PERCENT_DIFF &&
				velDiff * cMS_TO_MPH > cMIN_VEL_DIFF
				);
	bool slowDownGradually = (
				velDiffPercent < -1.0 * cMIN_PERCENT_DIFF &&
				velDiff * cMS_TO_MPH < -1.0 * cMIN_VEL_DIFF
				);
#endif

#if 0
	gout << "currVel = " << currVel * cMS_TO_MPH;
	gout << "mph   targVel = " << targVel * cMS_TO_MPH;
	gout << "mph   velDiffPercent = " << velDiffPercent;
	gout << endl;

	gout << "Big A = " << significantAccelNeeded << endl;
	gout << "Small D = " << slowDownGradually << endl;
//	gout << "Constant = " << (!significantAccelNeeded) && (!slowDownGradually) << endl;
//	gout << "targVel = " << targVel * cMS_TO_MPH << endl;
#endif

#if 0
	double targAccel;
	if( significantAccelNeeded )
	{
		targAccel = SignificantAccelController( 
							currVel, 
							targVel, 
							aggressiveness,
							rndNumGen,
							rndStream
							);
	}
	else if( slowDownGradually )
	{
		targAccel = SlowDownGraduallyController(
							currVel, 
							targVel, 
							aggressiveness
							);
	}
	else
	{
		targAccel = SpeedMaintenanceController(
							currVel, 
							prevVel,
							targVel,
							aggressiveness
							);
	}

	return targAccel;
#endif
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets an object's acceleration.
//
// Remarks:  For objects that dont' have acceleration, this function
//   returns 0.0.
//
// Arguments:
//    cpObj - A pointer to the CVED object.
//
// Returns:  The object's acceleration.
//
//////////////////////////////////////////////////////////////////////////////
double
GetAccel( const CObj* cpObj )
{
	//
	// Get the acceleration.
	//
	double accel = 0.0;
	switch( cpObj->GetType() )
	{
	case eCV_VEHICLE:
		{
			const CVehicleObj* cpVehObj = 
							dynamic_cast<const CVehicleObj *>( cpObj );
			accel = cpVehObj->GetAccel();
		}
		break;
	case eCV_EXTERNAL_DRIVER:
		{
			const CExternalDriverObj* cpVehObj = 
							dynamic_cast<const CExternalDriverObj *>( cpObj );
			accel = cpVehObj->GetAccelHighFreq();
		}
		break;
	default:
		accel = 0.0;
	}

	return accel;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Looks at the incoming pairs of velocities and distances
//   and picks the most conservative pair.
//
// Remarks:  This function picks the most conservative pair of velocity and
//   and distance from the given vector of pairs.
//
//   It uses the equation:  a = (v1^2  - v0^2) / (2 * dist)
//
// Arguments:
//    cPairs   - The vector of velocity and distance pairs.
//    cInitVel - The initial velocity.
//    vel      - (output) The most conservative velocity.
//    dist     - (output) The most conservative distance.
//
// Returns:  The most conservative velocity and distance pair.
//
//////////////////////////////////////////////////////////////////////////////
void
ResolveAccelConservative( 
			const vector<TVelInfo>& cPairs,
			const double cInitVel,
			double& vel,
			double& dist
			)
{

	vector<TVelInfo>::const_iterator pMin = cPairs.end();

	if( cPairs.size() == 1 )
	{
		//
		// Only one pair....return this one.
		//
		pMin = cPairs.begin();
	}
	else if( cPairs.size() > 1 ) 
	{
		//
		// Now check each element in the given vector.
		//
		vector<TVelInfo>::const_iterator pElem;

		double lowAccel;
		for( pElem = cPairs.begin(); pElem != cPairs.end(); pElem++ )
		{
			TVelInfo elem = *pElem;

			if( elem.dist < 1.0 ) 
			{
				//
				// If any distance is near 0.0 then don't look any further.
				// This must be an emergency.
				//
				pMin = pElem;
				break;
			}
			else 
			{
				// 
				// Compute the ideal acceleration profile.
				//
				double currAccel = CalcAccel( cInitVel, elem.vel, elem.dist );

#if 0
				gout << "for (" << elem.vel << ", " << elem.dist << ") --  ";
				gout << "currAccel = " << currAccel << endl;
#endif

				if( pMin == cPairs.end() )
				{
					pMin     = pElem;
					lowAccel = currAccel;
				}
				else if( currAccel < lowAccel )
				{
					// 
					// Current element has a lower acceleration.
					//
					if( pMin->dist < pElem->dist ) 
					{
						//
						// Current element is farther away then the low
						// acceleration element.  If I was to ignore the
						// current element, check to see if what kind
						// of deceleration I would experience going from
						// min element to current element.
						//
						double distDiff = pElem->dist - pMin->dist;

#if 0
						gout << "### min:   v=" << pMin->vel  << "  d=" << pMin->dist  << endl;
						gout << "### elem:  v=" << pElem->vel << "  d=" << pElem->dist << endl;
#endif

						double minToCurrAccel = CalcAccel( 
													pMin->vel, 
													pElem->vel, 
													distDiff 
													);

#if 0
						gout << "### " << minToCurrAccel << endl;
#endif

						if( minToCurrAccel < -2.0 ) 
						{
							pMin     = pElem;
							lowAccel = currAccel;
						}
#if 0
						else 
						{
							gout << "** ignoring current element" << endl;
						}
#endif

					}
					else 
					{
						pMin     = pElem;
						lowAccel = currAccel;
					}

				} // end else if ( currAccel < lowAccel )

			}  // end else

		}  // end for

	}

	if( pMin == cPairs.end() ) 
	{
		//
		// No pairs found.
		//
		cerr << "ResolveAccelConservative: no (vel, dist) found!";
		cerr << endl;

		vel = 0.0;
		dist = 0.0;
	}
	else 
	{
		vel = pMin->vel;
		dist = pMin->dist;
	}

}  // end of ResolveAccelConservative


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Looks at the incoming accelerations and picks the smallest
//   one.
//
// Remarks:  
//
// Arguments:
//    cAccelList - The accelerations.
//    size       - the size of the accelerations array
//    which      - output - the entry with the min acceleration
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
ResolveAccelConservative2( 
			const double cAccelList[],
			int     size,			// array size
			int&    which			// which entry was used
			)
{
	int    i;
	double accel;

	if ( size == 1 ) {
		accel = cAccelList[0];
		which = 0;
		return;
	}

	accel = cAccelList[0];
	which = 0;
	for (i=0; i<size; i++) 	{
		if ( cAccelList[i] < accel )
		{
			accel = cAccelList[i];
			which = i;
		}
	}

}  // end of ResolveAccelConservative2



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Looks at the incoming accelerations and picks the smallest
//   one and also returns the associated velocity.
//
// Remarks:  
//
// Arguments:
//    cAccelList - The vector of accelerations.
//    accel      - (output) The most conservative acceleration.
//    vel        - (output) The associated velocity.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
ResolveAccelInfoConservative( 
			const vector<TAccelInfo>& cAccelList,
			double& accel,
			double& vel
			)
{
	if( cAccelList.size() <= 0 )
	{
		cerr << "ResolveAccelConservative: no accelerations in accel list";
		cerr << endl;

		return;
	}

	vector<TAccelInfo>::const_iterator min = cAccelList.begin();

	if( cAccelList.size() > 1 ) 
	{
		//
		// Now check each element in the given vector.
		//
		vector<TAccelInfo>::const_iterator i;
		for( i = min + 1; i != cAccelList.end(); i++ )
		{
			if( i->accel < min->accel )  min = i;
		}
	}

	accel = min->accel;
	vel   = min->vel;

}  // end of ResolveAccelConservative

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates a random Signal Time given the MaxSignal Time and
//				 MinSignalTime
//
// Remarks: 
//
// Arguments:
//   minSignalTime   -- Minimum Signal Time (sec).
//	 maxSignalTime   -- maximum Signal Time (sec).
//
// Returns: Randomly generated Signal Time
//
//////////////////////////////////////////////////////////////////////////////
double CalculateRandomSignalTime(
			const double minSignalTime,
			const double maxSignalTime,
			CRandNumGen& randNumGen,
			const int rndStream
			)
{
	//double CRandNumGen::RandomDoubleRange(double low, double high, int genId)

	double randomSignalTime = randNumGen.RandomDoubleRange( 
										minSignalTime,
										maxSignalTime,
										rndStream
										);
									
	return randomSignalTime;

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Extends the Ado's path.
//
// Remarks:  This function extends an Ado's path if one of the following
//   conditions is true: (1) the Ado's path is less than some specified
//   distance long, (2) the Ado can cover it's path is less than some
//   specified time, and (3) the Ado's path doesn't span some specified
//   number of intersections.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
ExtendPath( CAdoInfo* pI )
{
#ifdef DEBUG_EXTEND_PATH
	gout << "=== ExtendPath ===" << pI->m_ageFrame << "=========================" << endl;
	gout << "  avoidTurnsAtIntersection = " << pI->m_avoidTurnsAtIntersection;
	gout << endl;
	gout << *pI->m_pPath;
#endif

	const double cMIN_PATH_TIME       = 10.0;    // seconds
	const double cMIN_PATH_LENGTH     = 1000.0;  // feet
	const double cMAX_PATH_LENGTH     = 2500.0;  // feet
	const int   cMIN_INTERSECTIONS_IN_PATH = 2;

	bool invalidPath = (
				!pI->m_pPath->IsValid() || 
				pI->m_pPath->Size() <= 0
				);
	if( invalidPath ) pI->m_pPath->Initialize( pI->m_roadPos );

	//
	// Make sure that the path is at least some minimum length.
	//
	double currPathLength = pI->m_pPath->GetLength( &pI->m_roadPos );
	double distShortOfMinDist = cMIN_PATH_LENGTH - currPathLength;
	bool belowMinimumLength = distShortOfMinDist > 0.0;
	if( belowMinimumLength )
	{
		if( pI->m_avoidTurnsAtIntersection )
		{
			double distAppended = pI->m_pPath->Append( 
										cCV_STRAIGHT_TURN, 
										cCV_EPSILON, 
										distShortOfMinDist 
										);
#ifdef DEBUG_EXTEND_PATH
			gout << "  ** belowMinLength **" << endl;
			gout << "  wantToAppend = " << distShortOfMinDist << " ft" << endl;
			gout << "  distAppended = " << distAppended << " ft" << endl;
			gout << *pI->m_pPath;
#endif
			if( distAppended + 1.0 < distShortOfMinDist )
			{
				distShortOfMinDist -= distAppended;
				pI->m_pPath->Append( distShortOfMinDist );

#ifdef DEBUG_EXTEND_PATH
				gout << "  ####  not enough ####" << endl;
				gout << "  wantToAppend = " << distShortOfMinDist << " ft" << endl;
				gout << *pI->m_pPath;
#endif
			}
		}
		else
		{
			pI->m_pPath->Append( distShortOfMinDist );
		}
	}

	// 
	// Update length.
	//
	currPathLength = pI->m_pPath->GetLength( &pI->m_roadPos );

	//
	// Check to make sure that the path is long enough with respect
	// to my current speed.
	//
	double appendDist;
	bool iAmValidCvedObj = pI->m_pObj && pI->m_pObj->IsValid();
	if( iAmValidCvedObj )
	{
		double currVel = cMETER_TO_FEET * pI->m_pObj->GetVelImm(); // ft/s

		double currPathTime;
		bool nonZeroVel = currVel > cNEAR_ZERO;
		if( nonZeroVel )
		{
			currPathTime = currPathLength / currVel;
		}
		else 
		{
			currPathTime = 1000.0;  // a large value
		}

		double timeShortOfMinTime = cMIN_PATH_TIME - currPathTime;
		bool pathNotLongEnough = timeShortOfMinTime > 0.0;
		if( pathNotLongEnough ) 
		{
			appendDist = timeShortOfMinTime * currVel;
			if( pI->m_avoidTurnsAtIntersection )
			{
				double distAppended = pI->m_pPath->Append( 
											cCV_STRAIGHT_TURN, 
											cCV_EPSILON, 
											appendDist 
											);
				currPathLength += distAppended;
#ifdef DEBUG_EXTEND_PATH
				gout << "  ** timeShort **" << endl;
				gout << "  wantToAppend = " << appendDist << " ft" << endl;
				gout << "  distAppended = " << distAppended << endl;
				gout << *pI->m_pPath;
#endif
				if( distAppended + 1.0 < appendDist )
				{
					appendDist -= distAppended;
					pI->m_pPath->Append( appendDist );
				}
			}
			else
			{
				pI->m_pPath->Append( appendDist );
			}

			currPathLength += appendDist;
		}
	}  // end if iamValidCvedObj

	//
	// Check to make sure that the path spans the specified number of
	// intersections.
	//
	int numIntersections = pI->m_pPath->GetNumIntrsctns( &pI->m_roadPos );
	bool needMoreIntrsctns = (
				numIntersections < cMIN_INTERSECTIONS_IN_PATH &&
				currPathLength < cMAX_PATH_LENGTH
				);

	if( needMoreIntrsctns ) 
	{
		double distToNextIntersection = pI->m_pPath->GetDistToNextIntrsctn(
															pI->m_roadPos
															);

		appendDist = distToNextIntersection + 500.0;
		bool appendDistExceedsMaxLength = (
					currPathLength + appendDist > cMAX_PATH_LENGTH 
					);
		if( appendDistExceedsMaxLength )
		{
			appendDist = cMAX_PATH_LENGTH - currPathLength;
		}

		if( pI->m_avoidTurnsAtIntersection )
		{
			double distAppended = pI->m_pPath->Append( 
										cCV_STRAIGHT_TURN, 
										cCV_EPSILON, 
										appendDist 
										);

#ifdef DEBUG_EXTEND_PATH
			gout << "  ** more intersections **" << endl;
			gout << "  wantToAppend = " << appendDist << " ft" << endl;
			gout << "  distAppended = " << distAppended << endl;
			gout << *pI->m_pPath;
#endif
			if( distAppended + 1.0 < appendDist )
			{
				appendDist -= distAppended;
				pI->m_pPath->Append( appendDist );
			}
		}
		else
		{
			pI->m_pPath->Append( appendDist );
		}
	}
}  // end of ExtendPath


//////////////////////////////////////////////////////////////////////////////
//
// Description:  For new paths, this function extends the path to make
//   sure that there are no lane changes if the ADO is close to the 
//   intersection.
//
// Remarks:  The user should call ExtendPath immediately after calling this
//   function to make that the path meets minimum/maximum length standards.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
NewPath( CAdoInfo* pI )
{

	// 
	// Only need to execute this function if the ADO is on a road.
	//
	if( !pI->m_roadPos.IsRoad() )  return;

	bool invalidPath = !pI->m_pPath->IsValid() || pI->m_pPath->Size() <= 0;
	if( invalidPath )
	{
		pI->m_pPath->Initialize( pI->m_roadPos );
	}

	//
	// Calculate dist to next intersection.
	//
	cvELnDir currDir = pI->m_currLane.GetDirection();
	double distToIntrsctn;
	if( currDir == ePOS ) 
	{
		double roadLength = pI->m_currRoad.GetLinearLength();
		distToIntrsctn = roadLength - pI->m_roadPos.GetDistance();
	}
	else 
	{
		distToIntrsctn = pI->m_roadPos.GetDistance();
	}

	const double cMIN_DIST_TO_INTRSCTN = 150.0;  // ft.
	double minDistToIntrsctn = cMIN_DIST_TO_INTRSCTN;
	bool tooCloseToIntrsctn = distToIntrsctn < minDistToIntrsctn;
	if( tooCloseToIntrsctn )
	{
		//
		// The ADO is too close to the intersection.  Now make sure that
		// the path is built such that there are no lane changes.
		//
		bitset<cCV_MAX_CRDRS> laneCrdrMask;
		laneCrdrMask.set( pI->m_roadPos.GetLane().GetId() );
		bool success = pI->m_pPath->AppendNoLaneChange( laneCrdrMask );

		if( !success )
		{
			//
			// Find the corridor in the next intersection that has my
			// current lane as the source lane.
			//
			CIntrsctn intrsctn = pI->m_currLane.GetNextIntrsctn();
			TCrdrVec targCrdrs;
			intrsctn.GetCrdrsStartingFrom( pI->m_currLane, targCrdrs );

			if( targCrdrs.size() > 0 )
			{
				//
				// Build a RoadPos to the first corridor.
				//
				CCrdr targCrdr = targCrdrs[0];
				CRoadPos roadPos( intrsctn, targCrdr );
				if( !roadPos.IsValid() )  return;

				//
				// Use the newly built RoadPos and append to the path.
				// Now we've guaranteed no lane changes until the
				// next intersection.
				//
				pI->m_pPath->Append( roadPos );
			}
		}
	}

}  // end of NewPath


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Obtains the instigator object's HCSM id.
//
// Remarks:
//
// Arguments:
//   instigatorSet - The instigator set.
//
// Returns:  An integer that represents the HCSM's id.
//
//////////////////////////////////////////////////////////////////////////////
int
GetInstigatorHcsmId( const set<CCandidate>& cInstigatorSet )
{
	//
	// Simply return the first object in the instigator set.
	//
	int instigatorHcsmId = -1;
	set<CCandidate>::const_iterator cItr = cInstigatorSet.begin();
	if( cItr != cInstigatorSet.end() )  instigatorHcsmId = cItr->m_hcsmId;

	return instigatorHcsmId;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Copies the candidate set into an array that is compatible
//   with the activity log.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
PutCandidatesIntoArr( 
			const set<CCandidate>& cCandidateSet,
			int* pCandidateArr,
			int& candidateArrSize
			)
{
	set<CCandidate>::const_iterator i;
	for( i = cCandidateSet.begin(); i != cCandidateSet.end(); i++ )
	{
		pCandidateArr[candidateArrSize] = i->m_hcsmId;
		candidateArrSize++;
		if( candidateArrSize >= 16 )  break;
	}
}
