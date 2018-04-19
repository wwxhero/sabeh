/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: curvature.cxx,v 1.39 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    March, 2000
 *
 * Description:  Contains the implementation for the CCurvature class.
 *
 ****************************************************************************/

#include "curvature.h"
#include "util.h"

#undef DEBUG_REFRESH_CURVATURE

const double cMIN_CURV_TIME       = 10.0;    // seconds
const double cMIN_CURV_LENGTH     = 1000.0;  // feet
const double cMAX_CURV_LENGTH     = 2500.0;  // feet

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCurvature::CCurvature():
	m_cvedId( -1 ),
	m_latAccelms2( cDEFAULT_LAT_ACCEL )
{

	InitializeLimits();

}

CCurvature::CCurvature( const cvEObjType objType )
{
	m_latAccelms2 = cDEFAULT_LAT_ACCEL;
	InitializeLimits();
}
//////////////////////////////////////////////////////////////////////////////////////
///\brief
///		Updates the lateral acceleration limit, in Gs
//////////////////////////////////////////////////////////////////////////////////////
void   
CCurvature::SetCurvatureLatAccelLimit(const double& val){
	m_latAccelms2 = val;
}

CCurvature::CCurvature( const CCurvature& cObjToCopy )
{
	// call the assignment operator
	*this = cObjToCopy;
}

CCurvature& 
CCurvature::operator=( const CCurvature& cObjToCopy )
{
	// check to make sure that object passed in is not me
	if ( this != &cObjToCopy ) 
	{

	}

	return *this;
}

CCurvature::~CCurvature()
{
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the CVED id.
//
// Remarks:  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCurvature::SetCvedId( int cvedId )
{
	m_cvedId = cvedId;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Builds a list of curvature limits.
//
// Remarks:  The limits specify cut-off curvatures for the buckets.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCurvature::InitializeLimits()
{

	TCurvLimit elem;
	elem.code = 0;
	elem.lowerLimit = 10000;
	m_limits.push_back( elem );
	elem.code = 1;
	elem.lowerLimit = 5000;
	m_limits.push_back( elem );
	elem.code = 2;
	elem.lowerLimit = 1000;
	m_limits.push_back( elem );
	elem.code = 3;
	elem.lowerLimit = 500;
	m_limits.push_back( elem );
	elem.code = 4;
	elem.lowerLimit = 300;
	m_limits.push_back( elem );
	elem.code = 5;
	elem.lowerLimit = 100;
	m_limits.push_back( elem );
	elem.code = 6;
	elem.lowerLimit = 40;
	m_limits.push_back( elem );
	elem.code = 7;
	elem.lowerLimit = 0;
	m_limits.push_back( elem );

}  // end of InitializeLimits


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Builds a list of curvatures from the given road position
//   on the given path.
//
// Remarks:  This function builds a list curvatures from the given road
//   position on the given path.  It samples the path at a specified 
//   interval.
//
// Arguments:
//   cRoadPos - The road position at which to start with curvature.
//   cPath    - The path on which to look for curvature.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCurvature::InitializeCurvature(
			const CRoadPos& cRoadPos,
			const CPath& cPath
			)
{

	//
	// Error checking.
	//
	if( !cRoadPos.IsValid() ) 
	{
		gout << "[" << m_cvedId << "] ";
		gout << "CCurvature: invalid road position...exit" << endl;
		return;
	}
	if( !cPath.IsValid() ) 
	{
		gout << "[" << m_cvedId << "] ";
		gout << "CCurvature: invalid path...exit" << endl;
		return;
	}

	m_startRoadPos = cRoadPos;
	m_path         = cPath;

	//
	// Empty the curvature structure.
	//
	if( !m_curvature.empty() ) 
	{
		m_curvature.erase( m_curvature.begin(), m_curvature.end() );
	}

	CPath localPath = m_path;
	CRoadPos currRoadPos = m_startRoadPos;
	currRoadPos.SetOffset( 0.0 );

	double currDist = 0.0;
	CPath::ETravelCode code = CPath::eCV_TRAVEL_OK;

	int counter = 0;

	while( 
		( 
			code == CPath::eCV_TRAVEL_OK || 
			code == CPath::eCV_TRAVEL_LANE_CHANGE 
			) &&
		currDist < cMAX_CURV_LENGTH
		)
	{
		CRoadPos nextRoadPos;
		CLane newLane;

		code = localPath.Travel(
					cSAMPLE_DIST, 
					currRoadPos,
					nextRoadPos, 
					newLane
					);

		currDist += cSAMPLE_DIST * cFEET_TO_METER;
		TCurvature elem;

		switch ( code ) 
		{
		case CPath::eCV_TRAVEL_ERROR :

			gout << "[" << m_cvedId << "] ";
			gout << "InitializeCurvature: error with path travel" << endl;;
			if( currRoadPos.IsValid() )
			{
				gout << "  currRoadPos = " << currRoadPos << endl;
				gout << "  cartPos = " << currRoadPos.GetXYZ() << endl;
				gout << "  cSAMPLE_DIST = " << cSAMPLE_DIST << " ft" << endl;
			}
			else
			{
				gout << "  currRoadPos = INVALID" << endl;
			}
			
			if( localPath.IsValid() )
			{
				gout << localPath;
			}
			else
			{
				gout << "  path = INVALID" << endl;
			}

			break;

		case CPath::eCV_TRAVEL_NOT_FOUND :

			gout << "[" << m_cvedId << "] ";
			gout << "InitializeCurvature: initial point not found" << endl;
			gout << "  cSAMPLE_DIST = " << cSAMPLE_DIST << " ft" << endl;
			gout << "  currRoadPos = " << currRoadPos << endl;
			gout << "  cartPos = " << currRoadPos.GetXYZ() << endl;
			gout << "  path:" << endl;
			gout << localPath << endl;

			break;

		case CPath::eCV_TRAVEL_OK :

			//
			// Build element and insert into list.
			//
			elem.curvature = nextRoadPos.GetCurvatureInterpolated();
			elem.dist = currDist;
			elem.roadPos = nextRoadPos;
			m_curvature.push_back( elem );
			
			break;

		case CPath::eCV_TRAVEL_LANE_CHANGE :

			//
			// Build element and insert into list.
			//
			elem.curvature = nextRoadPos.GetCurvatureInterpolated();
			elem.dist = currDist;
			elem.roadPos = nextRoadPos;
			m_curvature.push_back( elem );
			
			//
			// Set new lane.
			//
			localPath.SwitchLane( newLane );
			nextRoadPos.SetLane( newLane );

			break;

		case CPath::eCV_TRAVEL_END_OF_PATH :

			break;

		default:

			gout << "[" << m_cvedId << "] ";
			gout << "InitializeCurvature: unknown ETravelCode case = ";
			gout << code << endl;

			break;
		}

#if 0
		gout << "[" << counter << "]  currRoadPos = ";
		if( currRoadPos.IsValid() )
			gout << currRoadPos << endl;
		else
			gout << "INVALID" << endl;
		gout << "[" << counter << "]  nextRoadPos = ";
		if( nextRoadPos.IsValid() )
			gout << nextRoadPos << endl;
		else
			gout << "INVALID" << endl;
#endif
		
		currRoadPos = nextRoadPos;
		
		counter++;
	}

}  // end of InitializeCurvature


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function returns the index of the curvature 
//   element that's closest to the specified distance and is greater
//   than the specified distance.
//
// Remarks:  
//
// Arguments:
//   cDist - The distance (in meters) for which to return the curvature 
//      element.
//
// Returns:  An integer that represents the index of the curvature 
//   element.
//
//////////////////////////////////////////////////////////////////////////////
int
CCurvature::FindElemFromDist( const double& cDist )
{
	int roadPosElem = 0;

	deque<TCurvature>::iterator i;
	for( i = m_curvature.begin(); i != m_curvature.end(); i++ ) 
	{
		if( (*i).dist < cDist ) 
		{
			roadPosElem++;
		}
		else 
		{
			break;
		}
	}

	return roadPosElem;
}  // end of FindElemFromDist


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Refreshes the list of curvatures by adding more 
//   curvature elements according to the path.
//
// Remarks:  This function builds upon the list of curvatures.  It
//   first eliminates the old curvatures located before the given
//   road position.  It then adds more curvature elements to the 
//   list according to the given path.
//
// Arguments:
//   cRoadPos - The road position at which to refresh the curvature.
//   cPath    - The path on which to look for curvature.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCurvature::RefreshCurvature(
			const CRoadPos& cRoadPos,
			const CPath& cPath,
			long frame
			)
{
	//
	// Error checking.
	//
	if( !cRoadPos.IsValid() ) 
	{
		gout << "CCurvature: invalid road position...exit" << endl;
		return;
	}
	if( !cPath.IsValid() ) 
	{
		gout << "CCurvature: invalid path...exit" << endl;
		return;
	}

#ifdef DEBUG_REFRESH_CURVATURE
	gout << "==== RefreshCurvature =====" << endl;
#endif

	//
	// Compute the distance from the start of the curvature list
	// to the given cRoadPos.
	//
	double distFromStart = m_path.GetLength( &m_startRoadPos, &cRoadPos ); // ft
	int roadPosElem = FindElemFromDist( distFromStart * cFEET_TO_METER );

#ifdef DEBUG_REFRESH_CURVATURE
	gout << "distToFirstElem = " << m_curvature.begin()->dist * cMETER_TO_FEET << " ft" << endl;
	gout << "distFromStart = " << distFromStart << " ft" << endl;
	gout << "roadPosElem = " << roadPosElem << endl;
	gout << m_curvature[roadPosElem-1].dist * cMETER_TO_FEET << " ft  ";
	gout << m_curvature[roadPosElem].dist * cMETER_TO_FEET << " ft";
	gout << endl;
#endif

	//
	// Empty the curvature structure up to the element associated with
	// the given roadPos.
	//
	deque<TCurvature>::iterator endErase = m_curvature.begin() + roadPosElem;
	if( roadPosElem > 0 ) 
	{
		m_curvature.erase( m_curvature.begin(), endErase );
	}

	//
	// Update the distance on each existing element of the curvature 
	// list to reflect the distance from the given roadPos.
	//
	deque<TCurvature>::iterator i;
	for( i = m_curvature.begin(); i != m_curvature.end(); i++ ) 
	{
		(*i).dist = (*i).dist - ( distFromStart * cFEET_TO_METER );
	}

#ifdef DEBUG_REFRESH_CURVATURE
	gout << "new starting dist = ";
	gout << (*m_curvature.begin()).dist * cMETER_TO_FEET << " ft" << endl;
#endif

	//
	// Reset member variables to keep track of new starting road
	// position and the path.
	//
	m_startRoadPos = cRoadPos;
	m_path         = cPath;

	CPath localPath = m_path;
	CRoadPos currRoadPos = m_startRoadPos;
	currRoadPos.SetOffset( 0.0 );

	//
	// Now travel along the path until arriving at the road position in
	// the last element of the curvature list.
	//
	double fullCurvLength = 0;
	if( m_curvature.size() > 0 )
	{
		fullCurvLength = m_curvature.back().dist;  // meters
	}
	double fullPathLength = localPath.GetLength();  // feet
	double remainingPathLength = localPath.GetLength( &currRoadPos );
	
#ifdef DEBUG_REFRESH_CURVATURE
	gout << localPath << endl;
	gout << "fullCurvLength = " << fullCurvLength * cMETER_TO_FEET << " ft";
	gout << endl;
	gout << "fullPathLength = " << localPath.GetLength() << " ft" << endl;
	gout << "remaining path length = " << remainingPathLength << " ft" << endl;
	gout << "curr roadPos = " << currRoadPos << endl;
#endif

	//
	// Check to see if we've already reached the path's end.  If so,
	// then return.
	//
	bool exceededPath = ( ( fullCurvLength * cMETER_TO_FEET ) + 10.0 >
						  fullPathLength
						  );
	if( exceededPath ) 
	{
#ifdef DEBUG_REFRESH_CURVATURE
		gout << "reached path end...no more elements to add...exit" << endl;
#endif

		return;
	}

	CPath::ETravelCode code;
	CRoadPos nextRoadPos;

	code = localPath.GetRoadPos(
				fullCurvLength * cMETER_TO_FEET, 
				currRoadPos,
				nextRoadPos,
				false
				);

#ifdef DEBUG_REFRESH_CURVATURE
	gout << "current path length = " << localPath.GetLength( &nextRoadPos );
	gout << endl;
	gout << "code = " << code << endl;
#endif

	//
	// Error checking.
	//
	if( code == CPath::eCV_TRAVEL_END_OF_PATH ) 
	{
		//
		// Already reached end of path.....return.
		//
#ifdef DEBUG_REFRESH_CURVATURE
		gout << "reached end of path....return" << endl;
#endif

		return;

	}
	else if( 
		code != CPath::eCV_TRAVEL_OK && 
		code != CPath::eCV_TRAVEL_LANE_CHANGE 
		) 
	{
		gout << "[" << m_cvedId << "] ";
		gout << "CCurvature:RefreshCurvature: something is wrong....";
		gout << "attempting to continue by calling InitializeCurvature";
		gout << endl;

		gout << "pathLength   full = " << fullPathLength << " ft   remaining = " << remainingPathLength << " ft" << endl;
		gout << "code = " << code << endl;
		gout << "dist = " << fullCurvLength * cMETER_TO_FEET << " ft" << endl;
		gout << "currRoadPos = " << currRoadPos << endl;
		gout << localPath;

		InitializeCurvature( cRoadPos, cPath );
		return;
	}

	double currDist = fullCurvLength;
	code = CPath::eCV_TRAVEL_OK;
	currRoadPos = nextRoadPos;
	if (currRoadPos.IsRoad())
		localPath.SwitchLane(currRoadPos.GetLane()); //we may have switched lanes

#ifdef DEBUG_REFRESH_CURVATURE
	gout << "endPos = " << (*(m_curvature.end() - 1)).roadPos << endl;
	gout << "nextRoadPos = " << nextRoadPos << endl;
#endif

	while( ( code == CPath::eCV_TRAVEL_OK || 
			  code == CPath::eCV_TRAVEL_LANE_CHANGE 
			  ) &&
			currDist < cMAX_CURV_LENGTH
			) 
	{
		CRoadPos nextRoadPos;
		CLane newLane;

		code = localPath.Travel(
					cSAMPLE_DIST, 
					currRoadPos,
					nextRoadPos, 
					newLane
					);

		currDist += cSAMPLE_DIST * cFEET_TO_METER;
		TCurvature elem;

#ifdef DEBUG_REFRESH_CURVATURE
		gout << "currDist = " << currDist * cMETER_TO_FEET << endl;
#endif

		switch( code ) 
		{

		case CPath::eCV_TRAVEL_ERROR :

			gout << "[" << frame << ":" << m_cvedId <<  "] ";
			gout << "RefreshCurvature: error with path travel" << endl;
			if( currRoadPos.IsValid() )
			{
				gout << "  currRoadPos = " << currRoadPos << endl;
				gout << "  cartPos = " << currRoadPos.GetXYZ() << endl;
				gout << "  cSAMPLE_DIST = " << cSAMPLE_DIST << endl;
			}
			else
			{
				gout << "  currRoadPos = INVALID" << endl;
			}
			
			if( localPath.IsValid() )
			{
				gout << localPath;
			}
			else
			{
				gout << "  path = INVALID" << endl;
			}

			break;

		case CPath::eCV_TRAVEL_NOT_FOUND :

			gout << "[" << frame << ":" << m_cvedId <<  "] ";
			gout << "RefreshCurvature: initial point not found";
			gout << endl;

			if( currRoadPos.IsValid() )
			{
				gout << "  currRoadPos = " << currRoadPos << endl;
				gout << "  cartPos = " << currRoadPos.GetXYZ() << endl;
				gout << "  cSAMPLE_DIST = " << cSAMPLE_DIST << endl;
			}
			else
			{
				gout << "  currRoadPos = INVALID" << endl;
			}
			
			if( localPath.IsValid() )
			{
				gout << localPath;
			}
			else
			{
				gout << "  path = INVALID" << endl;
			}

			break;

		case CPath::eCV_TRAVEL_OK :

			//
			// Build element and insert into list.
			//
			elem.curvature = nextRoadPos.GetCurvatureInterpolated();
			elem.dist = currDist;
			elem.roadPos = nextRoadPos;
			m_curvature.push_back( elem );

#ifdef DEBUG_REFRESH_CURVATURE
			gout << "dist = " << currDist * cMETER_TO_FEET;
			gout << " ft     roadPos = " << nextRoadPos << endl;
#endif

			break;

		case CPath::eCV_TRAVEL_LANE_CHANGE :

			//
			// Build element and insert into list.
			//
			elem.curvature = nextRoadPos.GetCurvatureInterpolated();
			elem.dist = currDist;
			elem.roadPos = nextRoadPos;
			m_curvature.push_back( elem );
			
#ifdef DEBUG_REFRESH_CURVATURE
			gout << "switching lanes to " << newLane << endl;
			gout << "dist = " << currDist * cMETER_TO_FEET;
			gout << " ft     roadPos = " << nextRoadPos << endl;
#endif

			//
			// Set new lane.
			//
			localPath.SwitchLane( newLane );
			nextRoadPos.SetLane( newLane );

			break;

		case CPath::eCV_TRAVEL_END_OF_PATH :

			break;

		default:

			gout << "[" << frame << ":" << m_cvedId <<  "] ";
			gout << "RefreshCurvature: unknown ETravelCode case = ";
			gout << code << endl;

			break;

		}

		currRoadPos = nextRoadPos;
	}

}  // end of RefreshCurvature


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Finds the code associated with a curvature.
//
// Remarks:  Given a curvature, this function finds the code associated
//   with it.  This function returns a -1 if it cannot find the code
//   associated with the given curvature.
//
// Arguments:
//   cCurvature -- The curvature for which to find a code.
//
// Returns: An integer with the code.  -1 for errors.
//
//////////////////////////////////////////////////////////////////////////////
int
CCurvature::FindCode( const double& cCurvature )
{
	vector<TCurvLimit>::iterator i;
	for( i = m_limits.begin(); i != m_limits.end(); i++ ) 
	{
		if( cCurvature > (*i).lowerLimit )  return (*i).code;
	}

	return -1;
}  // end of FindCode


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the desired velocity at a curvature.
//
// Remarks:  Given a curvature and a maximum tolerated lateral
//   acceleration, this function finds the desired velocity associated
//   with it.
//
// Arguments:
//   cCurvature -- The curvature ( in feet );
//
// Returns: The velocity needed for the curvature with the tolerated
//   lateral acceleration.  This value is in the units m/s.
//
//////////////////////////////////////////////////////////////////////////////
double 
CCurvature::CalcVel( const double& cCurvature )
{
	//
	// Calculate velocity in m/s units since it will be reported
	// to dynamics.
	//
	return sqrt( cCurvature * cFEET_TO_METER * m_latAccelms2 );
}  // end of CalcVel


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates an acceleration.
//
// Remarks:  Given the object's (initial) velocity, the curvature's 
//   (final) velocity and a distance, this function calculates the 
//   resulting acceleration.
//
// Arguments:
//   cObjVel -- The object's (initial) velocity (in m/s).
//   cCurvVel -- The curvature's (final) velocity (in m/s).
//   cDistToCurv -- Distance to the curvature (in meters).
//
// Returns: The acceleration in the units m/s^2.
//
//////////////////////////////////////////////////////////////////////////////
double
CCurvature::CalcAccel( 
			const double& cObjVel, 
			const double& cCurvVel,
			const double& cDistToCurv
			)
{

	double velDiff = ( cCurvVel * cCurvVel ) - ( cObjVel * cObjVel );

	//
	// Protect for divide by zero.
	//
	double accel;
	if( cDistToCurv > cNEAR_ZERO ) 
	{
		accel = velDiff / ( 2.0 * cDistToCurv );
	}
	else 
	{
		accel = 1000.0;  // a large acceleration
	}

	return accel;
}  // end of CalcAccel


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Builds the buckets.
//
// Remarks:  This function builds the buckets for efficient querying of
//   curvature.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CCurvature::ResetBuckets()
{
	//
	// Empty the buckets if needed.
	//
	if( !m_buckets.empty() ) 
	{
		m_buckets.erase( m_buckets.begin(), m_buckets.end() );
	}

	m_bucketRoadPos = m_startRoadPos;

	deque<TCurvature>::iterator i;
	for( i = m_curvature.begin(); i < m_curvature.end(); i++ ) 
	{
		TCurvature curvElem = *i;
		int code = FindCode( curvElem.curvature );
		if( code == -1 ) 
		{
			gout << "[" << m_cvedId << "] ";
			gout << "ResetBuckets: unknown curvature category...skipping";
			gout << endl;

			continue;
		}

		if( m_buckets.size() <= 0 ) 
		{
			//
			// Make new bucket.
			//
			TBucket bucket;
			bucket.code = code;
			bucket.lowCurv = curvElem.curvature;
			bucket.roadPos = curvElem.roadPos;
			bucket.distToLowCurv = curvElem.dist;
			bucket.curvVel = CalcVel( curvElem.curvature );
			bucket.curvAccel = 0.0;
			m_buckets.push_back( bucket );
		}
		else 
		{
			//
			// Check to see if this belongs to the most recent bucket.
			//
			vector<TBucket>::reference currBucket = m_buckets.back();
			if( code == currBucket.code ) 
			{
				//
				// For small curvatures, make sure to generate a bucket
				// every X feet.
				//
				const double cSMALL_CURVATURE = 20.0;

				if( curvElem.curvature < cSMALL_CURVATURE ) 
				{
					const double cMAX_DIST_IN_SMALL_CURVATURE = 3.3; // meters
					double distDelta = curvElem.dist - currBucket.distToLowCurv;

					if( distDelta > cMAX_DIST_IN_SMALL_CURVATURE ) 
					{
						//
						// Make new bucket.
						//
						TBucket bucket;
						bucket.code = code;
						bucket.lowCurv = curvElem.curvature;
						bucket.roadPos = curvElem.roadPos;
						bucket.distToLowCurv = curvElem.dist;
						bucket.curvVel = CalcVel( curvElem.curvature );
						bucket.curvAccel = 0.0;
						m_buckets.push_back( bucket );
					}
					//
					// Check to see if this curvature is lower than the
					// one that is already there.
					//
					else if( curvElem.curvature < currBucket.lowCurv ) 
					{
						// update curvature
						currBucket.lowCurv = curvElem.curvature;
						currBucket.roadPos = curvElem.roadPos;
						currBucket.distToLowCurv = curvElem.dist;
						currBucket.curvVel = CalcVel( curvElem.curvature );
						currBucket.curvAccel = 0.0;
					}
				}
				//
				// Check to see if this curvature is lower than the
				// one that is already there.
				//
				else if( curvElem.curvature < currBucket.lowCurv ) 
				{
					// update curvature
					currBucket.lowCurv = curvElem.curvature;
					currBucket.roadPos = curvElem.roadPos;
					currBucket.distToLowCurv = curvElem.dist;
					currBucket.curvVel = CalcVel( curvElem.curvature );
					currBucket.curvAccel = 0.0;
				}
			}
			else 
			{
				//
				// This new element should be really compared against the
				// previous element as opposed to the one with the lower
				// curvature.
				//
				if( 
					code > currBucket.code * 0.95 &&
					code < currBucket.code * 1.05 &&
					curvElem.curvature < currBucket.lowCurv
					) 
				{
					// update curvature
					currBucket.lowCurv = curvElem.curvature;
					currBucket.roadPos = curvElem.roadPos;
					currBucket.distToLowCurv = curvElem.dist;
					currBucket.curvVel = CalcVel( curvElem.curvature );
					currBucket.curvAccel = 0.0;
				}
				else 
				{
					//
					// Make new bucket.
					//
					TBucket bucket;
					bucket.code = code;
					bucket.lowCurv = curvElem.curvature;
					bucket.roadPos = curvElem.roadPos;
					bucket.distToLowCurv = curvElem.dist;
					bucket.curvVel = CalcVel( curvElem.curvature );
					bucket.curvAccel = 0.0;
					m_buckets.push_back( bucket );
				}
			}
		}
	}
}  // end of ResetBuckets


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Finds the distance and the velocity associated with
//   the curvature.
//
// Remarks:  This function iterates through the buckets and finds the
//   lowest acceleration below the given threshold.  It then returns
//   associated distance to the curvature and the velocity needed to
//   navigate it.
//
// Arguments:
//    cMinAccelThreshold - Only look at accelerations below this threshold.
//    curvVel - This output contains the velocity associated with the
//      lowest acceleration bucket.
//    distToCurv - The output contains the associated distance to the
//      velocity in the previous output.
//
// Returns:  A boolean indicating if it was able to find an acceleration
//    below the given threshold.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCurvature::GetCurvature( 
			const double& cMinAccelThreshold,
			double& curvVel, 
			double& distToCurv
			)
{

	bool foundCurvature = false;
	double minAccel = cMinAccelThreshold;

	//
	// Iterate through all the buckets to find the lowest
	// acceleration.
	//
	vector<TBucket>::iterator i;
	for( i = m_buckets.begin(); i != m_buckets.end(); i++ ) 
	{
		TBucket& bucket = *i;

		if( bucket.distToLowCurv > 0.0 ) 
		{
			if( bucket.curvAccel < minAccel ) 
			{
				//
				// This is the lowest acceleration so far.
				//
				curvVel = bucket.curvVel;
				distToCurv = bucket.distToLowCurv;

				minAccel = bucket.curvAccel;
				foundCurvature = true;

				//
				// No need to look beyond the first harsh turn.
				// Doing this for efficiency's and realism's sake.
				//
				if( bucket.code >= 6 )  break;
			}
		}
	}

	return foundCurvature;
}  // end of GetCurvature


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Updates the distances and accelerations for each bucket.
//
// Remarks:  This function updates each bucket based on the given
//   road position and velocity.
//
// Arguments:
//    cObjRoadPos       - The object's current road position.
//    cObjVel           - The object's current velocity in meters/second.
//    distToFrontBumper - The object distance from its CG to the front bumper
//                        in meters.
//
// Returns:  
//
//////////////////////////////////////////////////////////////////////////////
void
CCurvature::UpdateBuckets( 
			const CRoadPos& cObjRoadPos,
			const double& cObjVel, 
			double distToFrontBumper
			)
{

	//
	// Find the distance between the start of the path and the given
	// roadPos.
	//
	double distFromStart = m_path.GetLength( &m_bucketRoadPos, &cObjRoadPos );

	vector<TBucket>::iterator i;
	for ( i = m_buckets.begin(); i != m_buckets.end(); i++ ) {

		(*i).distToLowCurv = ( (*i).distToLowCurv - 
							   ( distFromStart * cFEET_TO_METER )
							   );
		double dist = (*i).distToLowCurv - distToFrontBumper;
		(*i).curvAccel = CalcAccel( cObjVel, (*i).curvVel, dist );
#if 0
		gout << "objVel = " << objVel * cMS_TO_MPH << "mph  curvVel = ";
		gout << (*i).curvVel * cMS_TO_MPH << "mph  d=";
		gout << dist * cMETER_TO_FEET;
		gout << "ft   a=" << (*i).curvAccel << endl;
#endif
	}

	//
	// Update the bucket road position that's used to compute the
	// updated distances.
	//
	m_bucketRoadPos = cObjRoadPos;

}  // end of UpdateBuckets


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Prints the contents of the curvature deque to the screen.
//
// Remarks: Prints the contents of the curvature deque to the screen. 
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCurvature::DebugCurvature()
{

	gout << "** Curvature list [size = " << m_curvature.size();
	gout << "] **" << endl;

	deque<TCurvature>::iterator i;
	for ( i = m_curvature.begin(); i != m_curvature.end(); i++ ) {

		TCurvature elem = *i;
		gout << "  curv = " << elem.curvature << "   dist = ";
		gout << elem.dist * cMETER_TO_FEET << "ft   roadPos = ";
		gout << elem.roadPos << endl;
		
	}

}  // end of DebugCurvature


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Prints the contents of the buckets to the screen.
//
// Remarks: Prints the contents of the buckets to the screen. 
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCurvature::DebugBuckets()
{

	gout << "** Buckets [size = " << m_buckets.size() << "] **" << endl;
	vector<TBucket>::iterator j;
	for ( j = m_buckets.begin(); j != m_buckets.end(); j++ ) {

		TBucket elem = *j;
		gout << "  code = " << elem.code << "   curv = " << elem.lowCurv;
		gout << "  dist = " << elem.distToLowCurv * cMETER_TO_FEET << "ft";
		gout << "  v = " << elem.curvVel * cMS_TO_MPH << "mph";
		gout << "  a = " << elem.curvAccel << "m/s2" << endl;

	}

}  // end of DebugBuckets
