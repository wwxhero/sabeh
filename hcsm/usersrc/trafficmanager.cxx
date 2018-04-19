/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: trafficmanager.cxx
 *
 * Author:  Omar Ahmad, Huidi Tang
 *
 * Date:    September, 2002
 *
 * Description:  Contains code for the traffic manager object
 *               (TrafficManager HCSM).
 *
 ****************************************************************************/

#include "genericinclude.h"
#include "genhcsm.h"
#include "hcsmcollection.h"
#include <cvedpub.h>
#include <ctype.h>
#include <strstream>

#undef DEBUG_USER_CREATION
#undef DEBUG_USER_ACTIVITY
#undef DEBUG_GET_OWNVEH_ROADPOS
#undef DEBUG_MAINTAIN_PATH
#undef DEBUG_FIND_CREATION_POINTS
#undef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
#undef DEBUG_CREATE_OBJECTS
#undef DEBUG_DELETE_OBJECTS
#undef DEBUG_LIMIT_OVERLOAD


// Constants
const char* pNamePrefix = "TM_Ado_";
#define cTM_TIME_SPACE	1.0
#define cTM_EPSILON	0.5

//////////////////////////////////////////////////////////////////////////////
//
// Description:  For variable initialization.
//
// Remarks: 
//
// Arguments:
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void 
CTrafficManager::UserCreation( const CTrafMngrParseBlock* cpBlock )
{
	// Initialization
	m_creationCounter = 0;
	m_timeAtObjCreation = 0.0;
	m_rndStreamId = m_pRootCollection->m_rng.GetStream();

	// Make sure there is input set..
	m_inputSets = cpBlock->GetInputSets();

	bool noInputSets = m_inputSets.size() == 0;
	if( noInputSets ) 
	{	
		cerr << MessagePrefix();
		cerr << "TM requires at least one input set...[SUICIDE]" << endl;

		Suicide();
		return;
	}
	m_cInputSetItr = m_inputSets.begin();

	// initialize path information
	m_pPath = NULL;
	bool haveOwnshipPath = (
				CHcsmCollection::m_sOwnshipPath.IsValid() && 
				CHcsmCollection::m_sOwnshipPath.Size() > 0
				);
	if( haveOwnshipPath )
	{
		// if the ownship already has a path from scenario then we assume that this what's to be used
		m_pPath = new CPath( *cved );
		*m_pPath = CHcsmCollection::m_sOwnshipPath;
		//double pathLength = CHcsmCollection::m_sOwnshipPath.GetLength( &m_roadPos );
	}
} // end of UserCreation


////////////////////////////////////////////////////////////////////////////////
//
// Description:	This function obtains the ownship roadpos.  
//
// Remarks: 
//
// Arguments:
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficManager::GetOwnVehRoadPos( )
{

#ifdef  DEBUG_GET_OWNVEH_ROADPOS
	gout << "=== " << GetFrame();
	gout << " === TM::GetOwnVehRoadPos =====================" << endl;
#endif

	const CDynObj* cpObj = cved->BindObjIdToClass( 0 );
	if( !cpObj || !cpObj->IsValid() ) 
	{
#ifdef  DEBUG_GET_OWNVEH_ROADPOS
		gout << "  unable to get pointer to ownVeh CVED object" << endl;
#endif
		return;
	}

	CHcsm* pHcsm = m_pRootCollection->GetHcsm( cpObj->GetHcsmId() );
	if( !pHcsm ) 
	{

#ifdef  DEBUG_GET_OWNVEH_ROADPOS
		gout << "  unable to get pointer to ownVeh HCSM" << endl;
#endif
		return;

	}
	

	CRoadPos ownVehRoadPos;
	bool haveValFromMonitor = pHcsm->GetMonitorByName( 
												"RoadPos", 
												&ownVehRoadPos
												);
	if( !haveValFromMonitor || !ownVehRoadPos.IsValid() ) 
	{

#ifdef  DEBUG_GET_OWNVEH_ROADPOS
		gout << "  unable to get ownVeh roadpos from monitor";
		gout << endl;
#endif
		return;

	}

	m_roadPos = ownVehRoadPos;
	return;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function builds a path for the ownship.
//
//
// Remarks:	
//
// Arguments:
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void 
CTrafficManager::MaintainPath()
{
#ifdef DEBUG_MAINTAIN_PATH
	gout << "=== " << GetFrame();
	gout << " === TM::MaintainPath =====================" << endl;
#endif

	//
	// Set the distances to look ahead and behind.
	//
	double pathLengthBehindOwnVehicle = cFEET_PER_MILE * 0.1; // 1/10 mile
	double pathLengthAheadOwnVehicle  = cFEET_PER_MILE * 0.3; // 3/10 mile

	bool onHighway = false;
	if( onHighway )
	{
		pathLengthBehindOwnVehicle = cFEET_PER_MILE * 0.33333; // 1/3 mile
		pathLengthAheadOwnVehicle =  cFEET_PER_MILE * 0.66666; // 2/3 mile
	}

	double distToPathEnd = 0.0;
	if( m_pPath && m_pPath->IsValid() )
	{
		distToPathEnd = m_pPath->GetLength( &m_roadPos );
	}

#ifdef DEBUG_MAINTAIN_PATH
	gout << "  distToPathEnd = " << distToPathEnd << endl;
#endif
	if( distToPathEnd > pathLengthAheadOwnVehicle * 0.9 )  return;

	// Check if path exists. It it doesn't, build a new path; otherwise
	// extending the path by simply appending some distance to it. Then 
	// it is necessary to regenerate the path every two seconds because
	// the path may be getting too long with all the simple appending 
	// and with no deletion at the same time.
	const int cPATH_REGENERATE_FRAME_COUNT = 60;
	bool needNewPath = !m_pPath || m_pPath->Size() <= 0;
//	bool regeneratePath = ( GetFrame() % cPATH_REGENERATE_FRAME_COUNT == 0 );

#ifdef DEBUG_MAINTAIN_PATH
	gout << "  needNewPath=" << needNewPath << endl;
#endif

	if( needNewPath )
	{
#ifdef DEBUG_MAINTAIN_PATH
		gout << "  generating new path" << endl;
#endif
		if( m_pPath )	delete m_pPath;
	
		m_pPath = new CPath( *cved );
		m_pPath->Initialize( m_roadPos );
		m_pPath->Prepend( pathLengthBehindOwnVehicle );
		m_pPath->Append( pathLengthAheadOwnVehicle );
		m_distAtAppending = m_roadPos.GetDistance();

#ifdef DEBUG_MAINTAIN_PATH
		gout << "  path produced (size = " << m_pPath->Size() << "):" << endl;
		gout << *m_pPath << endl;
#endif
	}
	else
	{
		// simple appending
		double distToAppend = pathLengthAheadOwnVehicle - distToPathEnd;
#ifdef DEBUG_MAINTAIN_PATH
		gout << "  appending " << distToAppend << "ft to path" << endl;
#endif
		m_pPath->Append( distToAppend );
	}
}	// end of MaintainPath


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function builds a path for the ownship.
//
//
// Remarks:	
//
// Arguments:
//   cPathPrependDist - The distance to prepend to the path
//   cPathAppendDist - The distance to append to the path
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void 
CTrafficManager::MaintainPathDist( const double cPathPrependDist, const double cPathAppendDist )
{
#ifdef DEBUG_MAINTAIN_PATH
	gout << "=== " << GetFrame();
	gout << " === TM::MaintainPath =====================" << endl;
#endif

	double distToPathEnd = 0.0;
	if( m_pPath && m_pPath->IsValid() )
	{
		distToPathEnd = m_pPath->GetLength( &m_roadPos );
	}

#ifdef DEBUG_MAINTAIN_PATH
	gout << "  distToPathEnd = " << distToPathEnd << endl;
#endif
	if( distToPathEnd > cPathAppendDist * 0.9 )  return;

	// Check if path exists. It it doesn't, build a new path; otherwise
	// extending the path by simply appending some distance to it. Then 
	// it is necessary to regenerate the path every two seconds because
	// the path may be getting too long with all the simple appending 
	// and with no deletion at the same time.
	const int cPATH_REGENERATE_FRAME_COUNT = 60;
	bool needNewPath = !m_pPath || m_pPath->Size() <= 0 || distToPathEnd < 1.0;
//	bool regeneratePath = ( GetFrame() % cPATH_REGENERATE_FRAME_COUNT == 0 );

#ifdef DEBUG_MAINTAIN_PATH
	gout << "  needNewPath=" << needNewPath << endl;
#endif

	if( needNewPath )
	{
#ifdef DEBUG_MAINTAIN_PATH
		gout << "  generating new path" << endl;
#endif
		if( m_pPath )	delete m_pPath;
	
		m_pPath = new CPath( *cved );
		m_pPath->Initialize( m_roadPos );
		m_pPath->Prepend( cPathPrependDist );
		m_pPath->Append( cPathAppendDist );
		m_distAtAppending = m_roadPos.GetDistance();

#ifdef DEBUG_MAINTAIN_PATH
		gout << "  path produced (size = " << m_pPath->Size() << "):" << endl;
		gout << *m_pPath << endl;
#endif
	}
	else
	{
		// simple appending
		double distToAppend = cPathAppendDist - distToPathEnd;
#ifdef DEBUG_MAINTAIN_PATH
		gout << "  appending " << distToAppend << "ft to path" << endl;
#endif
		m_pPath->Append( distToAppend );
	}
}	// end of MaintainPathDist


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function finds the road positions for possible 
//   object creations.
//
// Remarks:	
//
// Arguments:
//
//	creationPoints - (Output) The resulting list of creation points found.
//	maxDensity - The maximum vehicle density specified by the user.
//	ownVehvel - The velocity of the ownship.
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficManager::FindCreationPointsOnHighway ( 
			vector<TTmCreationPoint>& creationPoints,
			const double& cMaxDensity,
			double ownVehVel
			)
{

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
	gout << "=== " << GetFrame();
	gout << " === TM::FindCreationPointsOnHighway =====================" << endl;
#endif

	const double cMIN_CREATION_DIST = 21.0;	// ft

	// Get objects on path.
	vector<int> objs;
	int numObjsOnPath = m_pPath->GetObjectsOnPath( objs );

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
	gout << "  # objs on path = " << numObjsOnPath << endl;
#endif

	// Store number of objs by lane
	vector<TTmObjsOnLaneInfo> numObjsEachLane; 
	
	// Ignore intersections
	if( !m_roadPos.IsRoad() )	return;

	// Assuming the road has same number of lanes as ownship road for now.
	// Later may need to use cCV_MAX_LANES.
	int ownshipNumLanes = m_roadPos.GetLane().GetRoad().GetNumLanes();
	int itr;
	for( itr = 0; itr < ownshipNumLanes; itr++ )
	{
		TTmObjsOnLaneInfo node;
		node.numObjsFront = 0;
		node.numObjsBack = 0;
		numObjsEachLane.push_back( node );
	}
	
	// For each obj on path, get roadpos and lane id.
	int i;
	for( i = 0; i < objs.size(); i++ )
	{
		const CDynObj* cpObj = cved->BindObjIdToClass( objs[i] );
		if( !cpObj || !cpObj->IsValid() ) 
		{

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
			gout << "  unable to get pointer to obj " << objs[i] << endl;
#endif

			continue;

		}

		CHcsm* pHcsm = m_pRootCollection->GetHcsm( cpObj->GetHcsmId() );
		if( !pHcsm ) 
		{
		
#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
			gout << "  unable to get pointer to hcsm for obj " << objs[i];
			gout << endl;
#endif
			continue;
		}
		
		CRoadPos roadPos;
		bool haveValFromMonitor = pHcsm->GetMonitorByName( 
												"RoadPos", 
												&roadPos
												);
		if( !haveValFromMonitor || !roadPos.IsValid() ) 
		{
				
#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
			gout << "failed to get obj roadpos from monitor ... check next obj ";
			gout << endl;
#endif
			continue;
		}

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
		gout << "  looking at obj " << objs[i] << endl;
		gout << "    roadPos = " << roadPos << endl;
#endif

		// Ignore intersections
		if( !roadPos.IsRoad() )	continue;	// continue for now.

		CLane lane = roadPos.GetLane();

		if( !lane.IsValid() )	continue;

		int laneId = roadPos.GetLane().GetRelativeId();

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
		gout << "    lane id = " << laneId << endl;
#endif

		// If ownship is on same road as current obj, compare dist to 
		// determine whether the obj is in front of ownship or behind ownship.
		// Then increment the number of objs in front or at back.If they
		// are not on same road, then check if current obj road is the next
		// road on path, if yes, the current obj is in front; otherwise the
		// current obj is determined to be on the back for now( May use
		// a different algorithm to determine later.)---Now changed when not
		// on same road: when they are on different road, continue for now.
		CRoad ownVehRoad = m_roadPos.GetLane().GetRoad();
		CRoad currObjRoad = roadPos.GetLane().GetRoad();
		if( ownVehRoad.GetName() == currObjRoad.GetName() )
		{
			// Assuming the own vehicle lane has the same direction as 
			// curr obj lane from the path.
			if( (lane.GetDirection() == ePOS &&
					m_roadPos.GetDistance() < roadPos.GetDistance()
					) ||
					( lane.GetDirection() == eNEG &&
					m_roadPos.GetDistance() > roadPos.GetDistance() )
			)
			{
				int val = numObjsEachLane[laneId].numObjsFront;
				numObjsEachLane[laneId].numObjsFront = val + 1;
				
			}
			else 
			{
				int val = numObjsEachLane[laneId].numObjsBack;
				numObjsEachLane[laneId].numObjsBack = val + 1;
			}
		}
		else
		{

			continue;	// continue for now

#if 0
			bool isNextRoad = m_pPath->IsNextRoad( currObjRoad );
			if( isNextRoad )
			{
				int val = numObjsEachLane[laneId].numObjsFront;
				numObjsEachLane[laneId].numObjsFront = val + 1;
			}
			else
			{
				int val = numObjsEachLane[laneId].numObjsBack;
				numObjsEachLane[laneId].numObjsBack = val + 1;
			}
#endif
		}
					

	}	// for each obj


	// Compute traffic density for each lane. If it is less than desired, compute
	// the number of vehicles needed to meet the density requirement. 

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
	gout << " number of lanes on ownship road = " << numObjsEachLane.size() << endl;
#endif


	for( int j = 0; j < numObjsEachLane.size(); j++ )
	{
		int numObjsFront = numObjsEachLane[j].numObjsFront;
		int numObjsBack = numObjsEachLane[j].numObjsBack;
		int numObjs = numObjsFront + numObjsBack;

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
		gout << "  total objs in lane " << j << " = " << numObjs << endl;
		gout << "  objs in front = " << numObjsFront << endl;
		gout << "  objs at back  = " << numObjsBack << endl;
#endif
		double density = numObjs / 1.0f; // in miles

		if( density < cMaxDensity )
		{
			int numObjsNeededForRequiredDensity = static_cast<int>(cMaxDensity - numObjs);

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
			gout << "  lane density = " << density << endl;
			gout << "  cMaxDensity = " << cMaxDensity << endl;
			gout << "  need " << numObjsNeededForRequiredDensity << " more objs in this lane";
			gout << endl;
#endif

			// Assign a priority value to the roadpos that is equal to the number
			// of objs needed.
			int priority = numObjsNeededForRequiredDensity;

			// Make the creation roadpos behind ownship if the ownship is slow and ahead
			// of ownship if fast. For the situation when the ownship is near speed limit,
			// if there are more objs in front of ownship than back in the lane, then 
			// create roadpos behind, if there are more objs behind, then make the
			// creation point ahead. 
			double speedLimit = m_roadPos.GetSpeedLimit();
			const double cMIN_OWN_VEH_VEL = speedLimit - 10.0;
			const double cMAX_OWN_VEH_VEL = speedLimit + 10.0;

			bool ownVehSlow = ownVehVel < cMIN_OWN_VEH_VEL;
			bool ownVehFast = ownVehVel > cMAX_OWN_VEH_VEL;

			// When using the path start and end for the creation roadpos, sometimes
			// the roadpos has a distance that is close to 0, which means the obj to be
			// created is at the very beginning the road. This seems to cause a problem. So
			// need to check this distance to make sure it is a valid distance for creating
			// an obj.( the distance must be >= cMIN_CREATION_DIST ). For now, assuing all the lanes are
			// positive since that seems to the case for highway setting in the database.
			// Need to consider the negative lanes when checking the distance in the future
			// when the TM is extended to the town setting.
			CRoadPos creationRoadPos;
			CRoadPos roadPosFromPathStartOrEnd;
			if( ownVehSlow )
			{
		
#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
				gout << " ownship slow... trying to make creation roadpos behind " << endl;
#endif
				roadPosFromPathStartOrEnd = m_pPath->GetStart( j );		

				
			}
			else if( ownVehFast )
			{

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
				gout << endl;
				gout << " ownship fast... trying to make creation roadpos ahead " << endl;
#endif

				roadPosFromPathStartOrEnd = m_pPath->GetEnd( j );
			}
			else
			{
				if( numObjsFront > numObjsBack )
				{

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
					gout << endl;
					gout << " ownship near speed limit and less objs behind..." << endl;
					gout << " so trying to make creation roadPos behind " << endl;
#endif
				
					roadPosFromPathStartOrEnd = m_pPath->GetStart( j );
				}
				else
				{

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
					gout << endl;
					gout << "  ownship near speed limit and less objs ahead ..." << endl;
					gout << "  so trying to make creation roadPos in front " << endl;
					
#endif
					roadPosFromPathStartOrEnd = m_pPath->GetEnd( j );
					
				}

			}

			// Check the distance. Need to consider negative lanes for town setting
			// in the future. If the distance from the path start or end roadpos is
			// less than cMIN_CREATION_DIST, skip the roadpos for now.
			if( roadPosFromPathStartOrEnd.GetDistance() < cMIN_CREATION_DIST )
			{
				continue;
			}
			else
			{
				creationRoadPos = roadPosFromPathStartOrEnd;

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
				gout << "  ownVeh vel = " << ownVehVel << endl;
				gout << "  speed limit = " << speedLimit << endl;
				gout << "  creation roadpos = " << creationRoadPos << endl;
#endif
			}

			TTmCreationPoint node;
			node.roadpos = creationRoadPos;
			node.priority = priority;
			creationPoints.push_back( node );
		

		}	// if density is less than required.
		else
		{

#ifdef DEBUG_FIND_CREATION_POINTS_ON_HIGHWAY
			gout << "  reached desired density ... no more obj will be " <<
					"created on this lane. This happens even when the " <<
					"desired max objects may not be reached. So density " <<
					"determines creation ultimately, not max objects value" <<
					endl;
#endif

			continue;
		}

	}	// for each lane


}	// end of FindCreationPointsOnHighway


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function finds the road positions for possible 
//   object creations.
//
// Remarks:	
//
// Arguments:
//
//	creationPoints - (Output) The resulting list of creation points found.
//	maxDensity - The maximum vehicle density specified by the user.
//	ownVehvel - The velocity of the ownship.
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficManager::FindCreationPoints ( 
			vector<TTmCreationPoint>& creationPoints,
			const double& cMaxDensity,
			double ownVehVel
			)
{

#ifdef DEBUG_FIND_CREATION_POINTS
	gout << "=== " << GetFrame();
	gout << " === TM::FindCreationPoints =====================" << endl;
#endif

	// Get objects on path.
	vector<int> objs;
	int numObjsOnPath = m_pPath->GetObjectsOnPath( objs );

#ifdef DEBUG_FIND_CREATION_POINTS
	gout << "  # objs on path = " << numObjsOnPath << endl;
#endif

	if( !m_pPath->IsValid() )  return;

	CRoadPos endPos;
	CLane changeLane;
	CPath::ETravelCode code = m_pPath->Travel( m_cInputSetItr->createDist, m_roadPos, endPos, changeLane );
	if( code != CPath::eCV_TRAVEL_OK && code != CPath::eCV_TRAVEL_LANE_CHANGE )  return;

	if( endPos.IsValid() && endPos.IsRoad() )
	{
		CRoadPos creationRoadPos;
		bool success = endPos.GetRoadPosInOppositeDir( creationRoadPos );
		if( success )
		{
			creationRoadPos.SetOffset( 0.0 );

			TTmCreationPoint node;
			node.roadpos = creationRoadPos;
			node.priority = 1;
			creationPoints.push_back( node );
		}
	}
}	// end of FindCreationPoints


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function creates the objects.
//
//
// Remarks:	
//
// Arguments:
//	creationPoints - Road positions for objects to be created.
//	cSolName - The type of vehicle to be created.
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficManager::CreateObjects( 
			vector<TTmCreationPoint>& creationPoints,
			const string& cSolName
			)
{

#ifdef DEBUG_CREATE_OBJECTS
	gout << "=== " << GetFrame();
	gout << " === TM::CreateObjects =====================" << endl;
#endif

	if( creationPoints.size() == 0 )	return;

	// Sort the creation points by priority with descending order.
	TTmCreationPoint hold;
	int i;
	int loc;
	for( i = 1; i < creationPoints.size(); i++ )
	{
		hold = creationPoints[i];
		loc = i;
		while( 0 < loc && ( hold.priority > creationPoints[loc-1].priority ) )
		{
			creationPoints[loc] = creationPoints[loc-1];
			loc--;
		}
		creationPoints[loc] = hold;
	}


#ifdef DEBUG_CREATE_OBJECTS
	gout << "  creationPoints [size = " << creationPoints.size() << "]: " << endl;
#endif

	vector<TTmCreationPoint>::iterator j;
	for( j = creationPoints.begin(); j != creationPoints.end(); j++ )
	{

#ifdef DEBUG_CREATE_OBJECTS
		gout << " " << j->roadpos << " [" << j->priority << "]";
		gout << endl;
#endif
	}
//	gout << endl;


	bool isObjCreated = false;
	// Pick the roadpos with the highest priority
	for( int k = 0; k < creationPoints.size(); k++ )
	{
		CRoadPos roadpos = creationPoints[k].roadpos;
	
		if( !roadpos.IsValid() )	continue;

		// Don't create obj on intersections
		if( !roadpos.IsRoad() )		return;

		// Make sure the roadpos is not occupied by another obj and that 
		// there is no vehicle quickly approaching.
		bool isVacant = true;	// need a cved function to do this check later.


#ifdef DEBUG_CREATE_OBJECTS
		gout << "  isVacant = " << isVacant << endl;
#endif

		if( isVacant )
		{
			// Create the obj
			CAdoParseBlock blk;

			blk.SetName( "TM_Ado" );	// All TM created objs have this name
			blk.SetSolName( cSolName );	
			blk.SetAutoControlHeadLight( true );
			blk.SetAutoControlBrakeLight( true );

//			vector<string> pathStr;
//			(*m_pPath).GetString( pathStr );
//			blk.SetPath( pathStr );

			// calculate initial velocity based on curvature at the creation point
			double curvature = roadpos.GetCurvature();
			double curvVel = ( curvature / 15 ) + 5.0;
			double initVel = curvVel;
			if( initVel > roadpos.GetSpeedLimit() )
			{
				initVel = roadpos.GetSpeedLimit();
			}

			blk.SetRoadPos( roadpos.GetString() );
			blk.SetVelCtrlFollowSpeedLimit( true );
			blk.SetVelCtrlInitVel( initVel );

			// randomize the color of the object being created 
			const CSolObjVehicle* pSolObj = dynamic_cast<const CSolObjVehicle*>(CCved::GetSol().GetObj( cSolName ));
			if( pSolObj ) 
			{
				const vector<CSolColor> colors = pSolObj->GetColors();
				if( colors.size() > 0 )
				{
					long randomNumber = m_pRootCollection->m_rng.RandomLongRange( 0, colors.size() - 1, m_rndStreamId );
					blk.SetColorIndex( (int) randomNumber );
				}
			}

			string blockStr;
			blk.Store( blockStr );

			vector<string> adoPath = blk.GetPath();
#ifdef DEBUG_CREATE_OBJECTS
			gout << "  blk name = " << blk.GetName() << endl;
			gout << "  blk solName = " << blk.GetSolName() << endl;
	//		gout << "  blk path = " << blk.GetPath() << end;
			gout << "  blk path size = " << adoPath.size() << endl;
			gout << "  blk roadpos = " << blk.GetRoadPos() << endl;
			gout << "  blk initVel = " << blk.GetVelCtrlInitVel() << endl;
#endif

			CHcsm* pTM_Ado = m_pRootCollection->CreateHcsm( "Ado", blk);


			if( !pTM_Ado )
			{
				
#ifdef DEBUG_CREATE_OBJECTS
				gout << " failed to create an obj " << endl;
#endif
				continue;

			}
			else
			{
				m_creationCounter++;
				m_timeAtObjCreation = GetFrame() * GetTimeStepDuration();
				isObjCreated = true;
				
#ifdef DEBUG_CREATE_OBJECTS
				gout << "  obj created " << endl;
				gout << "  actual creation roadpos = " << roadpos << endl;
				gout << "  m_creationCounter = " << m_creationCounter << endl;
#endif

				break;
			}

		}	// if vacant
		else	
		{
			continue; // try the roadpos with next highest priority
		}
	}
	if( !isObjCreated )
	{
		// None of the roadposes in the list is vacant, try it the next frame
		return;	
	}

}	// end of CreateObjects


////////////////////////////////////////////////////////////////////////////////
//
// Description:  This function deletes the objects and the deletion is determined
//	by how far the object is from ownship.
//
//
// Remarks:	
//
// Arguments:
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficManager::DeleteObjects()
{

#ifdef DEBUG_DELETE_OBJECTS
	gout << "=== " << GetFrame();
	gout << " === TM::DeleteObjects =====================" << endl;
#endif

	const int cTM_pathBehindOwnship = 5280 / 3 ; // 1/3 mile
	const int cTM_pathAheadOwnship = ( 5280 * 2 ) / 3; // 2/3 mile
	const int cTM_deleteDistInFront = cTM_pathAheadOwnship + 100;
	const int cTM_deleteDistAtBack = cTM_pathBehindOwnship + 200;

	vector<int> objs;
	cved->GetObj( "TM_Ado", objs );

#ifdef DEBUG_DELETE_OBJECTS
	gout << "  num of TM objs = " << objs.size() << endl;
#endif

	//
	// Whether an obj needs to be deleted is determined by how far 
	// it is from ownship and whether it is in front of or at back 
	// of ownship. If it is ahead of ownship and its distance from 
	// ownship is greater than 2/3 mile plus 100ft, then delete it; 
	// or if it is behind ownship and its distance from ownship is 
	// greater than 1/3 mile plus 200ft, then delete it.
	//
	int i;
	for( i = 0; i < objs.size(); i++ )
	{
		// Get roadpos from monitor
		const CDynObj* cpObj = cved->BindObjIdToClass( objs[i] );
		bool invalidObj = !cpObj || !cpObj->IsValid();
		if( invalidObj ) 
		{
#ifdef DEBUG_DELETE_OBJECTS
			cerr << "  unable to get pointer to obj " << objs[i];
			cerr << "...check next obj" << endl;
#endif
			continue;
		}

		CHcsm* pHcsm = m_pRootCollection->GetHcsm( cpObj->GetHcsmId() );
		if( !pHcsm ) 
		{
				
#ifdef DEBUG_DELETE_OBJECTS
			cerr << "  cannot get pointer to hcsm " << cpObj->GetHcsmId();
			cerr << "...check next obj" << endl;
#endif
			continue;
		}
				
		CRoadPos roadPos;
		bool haveValFromMonitor = pHcsm->GetMonitorByName( 
												"RoadPos", 
												&roadPos
												);
		bool haveValidRoadPos = haveValFromMonitor && roadPos.IsValid(); 
		if( !haveValidRoadPos ) 
		{					
#ifdef DEBUG_DELETE_OBJECTS
			cerr << "  failed to get obj " << objs[i];
			cerr << " roadpos from monitor...check next obj" << endl;
#endif
			continue;
		}

#ifdef DEBUG_DELETE_OBJECTS
		gout << "  looking at obj " << objs[i] << endl;
		gout << "    roadPos = " << roadPos << endl;
#endif

		// No deletion on intersection
		if( !roadPos.IsRoad() )		continue;
		if( !m_roadPos.IsRoad() )	continue;

		// Get obj distance from ownship
		CLane lane = roadPos.GetLane();
		CLane ownshipLane = m_roadPos.GetLane();

		if( !lane.IsValid() )	continue;

		
		CRoad ownshipRoad = ownshipLane.GetRoad();
		CRoad objRoad = lane.GetRoad();
	
		// When on same road
		if( ownshipRoad.GetName() == objRoad.GetName() )
		{
			cvELnDir ownshipLaneDir = ownshipLane.GetDirection();
			cvELnDir objLaneDir = lane.GetDirection();

			// for positive lane
			if( ownshipLaneDir == ePOS && ownshipLaneDir == objLaneDir )
			{
				double ownshipDist = m_roadPos.GetDistance();
				double objDist = roadPos.GetDistance();
				double distFromOwnship = objDist - ownshipDist;

				if( distFromOwnship >= ( cTM_deleteDistInFront) || distFromOwnship <= -cTM_deleteDistAtBack )		
				{
					// delete the obj 
					if( m_pRootCollection->DeleteHcsm( pHcsm ) )
					{

#ifdef DEBUG_DELETE_OBJECTS
						gout << "    ** deleting" << endl;
#endif
					}

					else
					{

#ifdef DEBUG_DELETE_OBJECTS
						gout << "    failed to delete obj" << endl;
#endif
						continue; 
					}

				}
					
			}
			
			// for negative lane
			if( ownshipLaneDir == eNEG && ownshipLaneDir == objLaneDir )
			{
				double ownshipDist = m_roadPos.GetDistance();
				double objDist = roadPos.GetDistance();
				double distFromOwnship = objDist - ownshipDist;

				if( distFromOwnship < -( cTM_deleteDistInFront ) || distFromOwnship >= cTM_deleteDistAtBack )
				{
				
					// delete the obj 
					if( m_pRootCollection->DeleteHcsm( pHcsm ) )
					{
						
#ifdef DEBUG_DELETE_OBJECTS
						gout << "    ** deleting" << endl;
#endif
					}
					else
					{

#ifdef DEBUG_DELETE_OBJECTS
						gout << "    failed to delete obj" << endl;
#endif
						continue; 
					}
					
				
				}
			
			}

		}	// on same road as ownship	
		else
		{
			// When not on same road, first determine if obj is in front or at 
			// back of ownship, then compute the distance between them for 
			// possible deletion.
			CPoint3D ownshipPos = m_roadPos.GetXYZ();
			CPoint3D objPos = roadPos.GetXYZ();
			
			double x0 = ownshipPos.m_x;
			double y0 = ownshipPos.m_y;
			double x1 = objPos.m_x;
			double y1 = objPos.m_y;

			bool isFront = false;
			bool isBack = false;

			// Assuming ownship and obj have same lane direction
			bool isPositiveLane = ownshipLane.GetDirection() == ePOS;   
			if( x0 == x1 )
			{		
				if( isPositiveLane )
				{
					if( y0 > y1 )	isBack = true;
					else	isFront = true;		
				}
				else
				{
					if( y0 > y1 )	isFront = true;
					else	isBack = true;
				}
			}
			if( x0 < x1 )
			{
				if( isPositiveLane )	isFront = true;
				else	isBack = true;
			}
			if( x0 > x1 )
			{
				if( isPositiveLane )	isBack = true;
				else	isFront = true;
			}

			// Compute distance from obj to ownship
			double dx = x0 - x1;
			double dy = y0 - y1;
			double dist = sqrt( ( dx * dx ) + ( dy * dy ) );

			if( isFront && !isBack )
			{
				
#ifdef DEBUG_DELETE_OBJECTS
				gout << " obj in front: " << objs[i] << endl;
				gout << " dist to ownship = " << dist << endl;
				gout << " deletion dist in front = " << cTM_deleteDistInFront << endl;
#endif

				if( dist >= cTM_deleteDistInFront )
				{
					if( m_pRootCollection->DeleteHcsm( pHcsm ) )
					{
										
#ifdef DEBUG_DELETE_OBJECTS
						gout << "    ** deleting obj in front" << endl;
#endif
					}
					else
					{

#ifdef DEBUG_DELETE_OBJECTS
						gout << "    failed to delete obj in front" << endl;
#endif
						continue; 
					}
				}
			}
			if( isBack && !isFront )
			{

#ifdef DEBUG_DELETE_OBJECTS
				gout << "    obj at back" << endl;
				gout << "    dist to ownship = " << dist << endl;
				gout << "    deletion dist at back = " << cTM_deleteDistAtBack << endl;
#endif

				if( dist >= cTM_deleteDistAtBack )
				{
					if( m_pRootCollection->DeleteHcsm( pHcsm ) )
					{
										
#ifdef DEBUG_DELETE_OBJECTS
						gout << "    ** deleting obj in back" << endl;
#endif
					}
					else
					{

#ifdef DEBUG_DELETE_OBJECTS
						gout << "    failed to delete obj at back" << endl;
#endif
						continue; 
					}
				}
			}
		}	// when not on same road as ownship


	}	// for each obj
//	gout << endl;

}	// end of DeleteObjects


#if 0
static bool 
compare_dists( const CTmObjs& first, const CTmObjs& second )
{
	return first.distToOwnship < second.distToOwnship;
}
#endif

static void
InsertTmObjElem( vector<TTmObj>& tmObjs, TTmObj& node )
{
	if( tmObjs.size() == 0 )
	{
		tmObjs.push_back( node );
	}
	else
	{
		vector<TTmObj>::iterator itr;
		for( itr = tmObjs.begin(); itr != tmObjs.end(); itr++ )
		{
			if( node.distToOwnship < itr->distToOwnship )  break;
		}

		tmObjs.insert( itr, node );
	}
}


////////////////////////////////////////////////////////////////////////////////
//
// Description: Gets an accounting of the number of objects created by TM.
//
// Remarks:	Collect all objects in the database and compute their distance 
//   and relative position to the driver's eyesight. Also, count how many of 
//   them were created by me.
//
// Arguments:
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficManager::CountObjectsCreatedByTM( vector<TTmObj>& tmObjs )
{
	CPoint3D ownPos = m_roadPos.GetXYZ();
	CVector3D ownTan;
	bool success = cved->GetOwnVehicleTan( ownTan );
	if( !success )
	{
		gout << "TM: **error** unable to get own vehicle tangent vector" << endl;
		return;
	}

	vector<int> dynamicObjs;
	success = cved->GetObj( "TM_Ado", dynamicObjs );  // success is 0 if no objects were found

	int numberObjectsBehindDriverEye = 0;
	CCved::TIntVec::const_iterator cItr;
	for( cItr = dynamicObjs.begin(); cItr != dynamicObjs.end(); cItr++ )
	{
		CPoint3D objPos = cved->GetObjPos( *cItr );
		CVector3D diff = objPos - ownPos;
		diff.Normalize();
		double objDist = ownPos.Dist( objPos );
		double dotProduct = ownTan.DotP( diff );
		bool objAhead = ( dotProduct > 0.0 );
		if( !objAhead ) numberObjectsBehindDriverEye++;

		// insert objects into vector sorted by distance to ownship
		TTmObj node;
		node.cvedObjId = *cItr;
		node.distToOwnship = objDist;
		node.aheadOfOwnship = objAhead;
		node.markedForDeletion = false;
		InsertTmObjElem( tmObjs, node );
	}
}


////////////////////////////////////////////////////////////////////////////////
//
// Description: Kill objects created by the TM.
//
// Remarks:	This function kills objects created by the TM based on criteria
//   defined in the arguments.
//
// Arguments:
//   tmObjs - A vector of objects created by the TM.
//   numToKill - Number of vehicles to kill.
//   killBehindOnly - Kill behind the ownship only.
//   cDistToKill - Kill objects outside of this distance.
//   
// Returns:	Number of vehicles killed by this function.
//
//////////////////////////////////////////////////////////////////////////////
int
CTrafficManager::KillTmObjs( 
	vector<TTmObj>& tmObjs, 
	int numToKill, 
	bool killBehindOnly, 
	const double cDistToKill 
	)
{
#ifdef DEBUG_DELETE_OBJECTS
	gout << "[" << GetFrame() << "]: KillTmObjs  numToKill=" << numToKill;
	gout << "  killBehindOnly=" << killBehindOnly << "  distToKill=";
	gout << cDistToKill << endl;
#endif

	// first kill objects behind me
	int numKilled = 0;
	vector<TTmObj>::reverse_iterator ritr;
	for( ritr = tmObjs.rbegin(); ritr != tmObjs.rend(); ritr++ )
	{
		bool doneKilling = numKilled >= numToKill;
		if( doneKilling )  return numKilled;

		if( cDistToKill > 0.0 && ritr->distToOwnship < cDistToKill )  return numKilled;

		if( ritr->markedForDeletion )  continue;

		if( killBehindOnly && ritr->aheadOfOwnship )  continue;

		const CDynObj* cpObj = cved->BindObjIdToClass( ritr->cvedObjId );
		if( !cpObj || !cpObj->IsValid() ) 
		{
#ifdef DEBUG_DELETE_OBJECTS
			gout << "  unable to get ptr to cved obj " << ritr->cvedObjId << endl;
#endif
			continue;
		}

		CHcsm* pHcsm = m_pRootCollection->GetHcsm( cpObj->GetHcsmId() );
		if( !pHcsm ) 
		{
#ifdef DEBUG_DELETE_OBJECTS
			gout << "  cannot get pointer to hcsm " << cpObj->GetHcsmId();
			gout << " for cved obj " ritr->cvedObjId << endl;
#endif
			continue;
		}

		if( !m_pRootCollection->DeleteHcsm( pHcsm ) )
		{
#ifdef DEBUG_DELETE_OBJECTS
			gout << "    failed to delete obj" << endl;
#endif
			continue; 
		}

#ifdef DEBUG_DELETE_OBJECTS
		gout << "    ** deleting " << ritr->cvedObjId << endl;
#endif
		ritr->markedForDeletion = true;
		numKilled++;
	}

	return numKilled;
}


////////////////////////////////////////////////////////////////////////////////
//
// Description: Kill objects created by the TM.
//
// Remarks:	This function kills objects created by the TM based on criteria
//   defined in the arguments.
//
// Arguments:
//   tmObjs - A vector of objects created by the TM.
//   cAheadDist -
//   cBehindDist - 
//   
// Returns:	Number of vehicles killed by this function.
//
//////////////////////////////////////////////////////////////////////////////
int
CTrafficManager::KillTmObjsDist( 
	vector<TTmObj>& tmObjs, 
	const double cAheadDist,
	const double cBehindDist
	)
{
#ifdef DEBUG_DELETE_OBJECTS
	gout << "[" << GetFrame() << "]: KillTmObjs  aheadDist=" << cAheadDist;
	gout << "  behindDist=" << cBehindDist << endl;
#endif

	// first kill objects behind me
	int numKilled = 0;
	vector<TTmObj>::reverse_iterator ritr;
	for( ritr = tmObjs.rbegin(); ritr != tmObjs.rend(); ritr++ )
	{
		bool kill = (
				( ritr->aheadOfOwnship && ritr->distToOwnship > cAheadDist ) ||
				( !ritr->aheadOfOwnship && ritr->distToOwnship > cBehindDist )
				);

		if( !kill )  continue;

		const CDynObj* cpObj = cved->BindObjIdToClass( ritr->cvedObjId );
		if( !cpObj || !cpObj->IsValid() ) 
		{
#ifdef DEBUG_DELETE_OBJECTS
			gout << "  unable to get ptr to cved obj " << ritr->cvedObjId << endl;
#endif
			continue;
		}

		CHcsm* pHcsm = m_pRootCollection->GetHcsm( cpObj->GetHcsmId() );
		if( !pHcsm ) 
		{
#ifdef DEBUG_DELETE_OBJECTS
			gout << "  cannot get pointer to hcsm " << cpObj->GetHcsmId();
			gout << " for cved obj " ritr->cvedObjId << endl;
#endif
			continue;
		}

		if( !m_pRootCollection->DeleteHcsm( pHcsm ) )
		{
#ifdef DEBUG_DELETE_OBJECTS
			gout << "    failed to delete obj" << endl;
#endif
			continue; 
		}

#ifdef DEBUG_DELETE_OBJECTS
		gout << "    ** deleting " << ritr->cvedObjId << endl;
#endif
//		static int deleteCounter = 0;
//		deleteCounter++;
//		gout << MessagePrefix() << ":" << deleteCounter << " delete " << ritr->cvedObjId << endl;

		ritr->markedForDeletion = true;
		numKilled++;
	}

	return numKilled;
}


////////////////////////////////////////////////////////////////////////////////
//
// Description: Overload control.  This routine deletes 'DesKill' objects 
//   (or less) in order to limit the traffic produced by the TM.
//
// Remarks:	The routine classifies all objects based on their distance
//   and relative position to the driver's eyesight, it then
//   deletes first vehicles that are further behind from the driver,
//   then further ahead etc.
//
// Arguments:
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficManager::LimitOverload(	vector<TTmObj>& tmObjs,	int numToKill )
{
#ifdef DEBUG_LIMIT_OVERLOAD
	gout << "LimitOverload: kill " << numToKill << " out of " << tmObjs.size() << endl;
#endif

	int numKilled = 0;

	// first kill objects behind ownship
	bool needToKill = numKilled < numToKill;
	if( needToKill ) numKilled = KillTmObjs( tmObjs, numToKill, true, -1.0 );

	// then kill the farthest away objects
	needToKill = numKilled < numToKill;
	if( needToKill ) numKilled = KillTmObjs( tmObjs, numToKill, false, -1.0 );
}  // End LimitOverload

#if 0
/**************************************************************************
 * Finds the end point along the road network. Also collects the roads and 
 * intersection while doiing so. Returns only the ones present in the
 * inclusion list.
 *
 **************************************************************************/
void
CTrafficManager::FindEndRdLn(
			veRdLnDs *rld,
			veRdLnDs *endrld,
			int *index, 
			int *nEndRLD,
			double size, 
			IntrData *visit, 
			int *nIntr,
			RoadData *roads,
			int *nRds,
			TrfMngrInfo_t     *pInfo,
			SVEmergPathMode_t *SVmode
			)
{
	/*structure containing the data used by all the  
	 *search functions. passed by value because of the 
	 *recursivityof the function 
	 */
	GroupedData GD;
	/* TRIP */
	double 		keep_size;
	
	veRd       *rd;
	veIntrsxn  intr[2];
	double     dist[2], d, len;
	int        lane;
	int        i, j;
	double     dist_from_intr[MAX_RLD];
	veBool     touchIntr;   /* Used to see if we passed the current road or not */
	int			OnReverse=0;
	int		   currentGraphLevel=-1;
	*nEndRLD = 0;
	*nIntr   = 0;
	*nRds    = 0;

	rd    = (veRd *)veGetRdFromRdLnDs(rld);
	lane  = veGetLnFromRdLnDs(rld);
	d     = veGetDsFromRdLnDs(rld);
	len   = veQryRoadLength(rd);

	dist[0] = len - d;
	dist[1] = d;

	veQrySrcSnkRd(rd, intr+1, intr+0);

	touchIntr = veFALSE;



#ifdef DEBUG_TM_FindEndRdLn
	fprintf(stderr, 
		"-------------------------------------------------------------------------\n"); 
	fprintf(stderr, "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\\n"); 
	fprintf(stderr, "/\\/\\/\\/\\   		FindEndRdLn 		/\\/\\/\\/\\/\\\n"); 
	fprintf(stderr, 
			" rd = %s , lane = %d , Ds(d) = %g, dist[0]= %g , dist[1]= %g \n",
			veQryRoadName(rd),lane,d,dist[0],dist[1]);
#endif

#if (EMERG_EXTENSIONS == 1)
	keep_size=size;
	for (i = 0; i < 1; i++) 
	{	

		double togo = 0;

		GD.IsSearchingInDriverDirection = smTrue; 
		/* TRIP */
		if ( IsRoadHighway( rd)) {
			size=GetAheadHighwayRange(keep_size);
		} else {
			size=GetAheadRange(keep_size);
		}
			
		if( veQryLaneDirRdLnDs(rld ) == vePOS )
			togo = dist[0];
		else
			togo = dist[1];

		/* if (dist[i] >= size) */ 
		if (togo >= size) 
		{/* the EndRld is ON this road segment */
#ifdef DEBUG_TM_FindEndRdLn
			fprintf(stderr," \n");
			fprintf(stderr, "dist[i] >= size\n");
			fprintf(stderr, "Direction : %s // dist[%d]=%g // Range = %g\n",
				(i==TM_HEADING_BACKWARD?"TM_HEADING_BACKWARD":"TM_HEADING_FORWARD"),
				i, dist[i],size); 
#endif

			/*Determin which way the driver is going */
			if (vePOS==veQryLaneDirRdLnDs(rld )){
				d = dist[1] + size; /* dist[1] is curr driver pos */
			 	dist_from_intr[*nEndRLD] = dist[0] - size;
			} else {
				d = dist[1] - size; /* dist[1] is curr driver pos */
			 	dist_from_intr[*nEndRLD] = size + dist[1];
			}
	
			 
			veBuildRdLnDs(&endrld[*nEndRLD], rd, lane, d);
    	/* store the distane of this end point from th intr it is visited.
         This is helpful if we find final points that are on same road */
			 
			 
			AddRoad(	roads, 
						nRds, 
						rd, 
            (veQryLaneDirRdLnDs(rld)==vePOS)?dist[1]:d,
            (veQryLaneDirRdLnDs(rld)==vePOS)?d:dist[1],
						smTrue);
			 
			(*nEndRLD)++;
			 
			if (*nEndRLD >= MAX_RLD) break;
			 
#ifdef DEBUG_TM_FindEndRdLn						
	{ 
		int i;	
		veRd rd;				
		fprintf(stderr,"//-------------------------------------------------------------//\n");
		fprintf(stderr, "FindEndRld: case dist[i] >= size\n");
		for (i = 0; i < *nRds; i++) {
			rd.rid = roads[i].rd.rid;
			if (roads[i].rd.rid == -1) continue; 
			fprintf(stderr, "\tRoad = %s, d1 = %g, d2 = %g, isAeahDriver %s\n",
						   veQryRoadName(&rd), 
							roads[i].d1, roads[i].d2,
						(roads[i].isAheadDriver==veTRUE?"Yes":"NO"));
		}
		fprintf(stderr,"//-------------------------------------------------------------//\n");
	}
#endif						
											 
			 
		} else 
		{/* the EndRld is PAST this road segment */
		
#ifdef DEBUG_TM_FindEndRdLn
			fprintf(stderr," \n");
			fprintf(stderr, "dist[i] < size\n");
			fprintf(stderr, "Direction : %s // dist[%d]=%g // Range = %g\n",
				(i==TM_HEADING_BACKWARD?"TM_HEADING_BACKWARD":"TM_HEADING_FORWARD"),
				i, dist[i],size); 
			fprintf(stderr, "Add road and Calling SearchEndRdLnFromIntersection\n"); 
#endif
		
			AddRoad(	roads,
						nRds, 
						rd, 
            (veQryLaneDirRdLnDs(rld)==vePOS)?dist[1]:0.0,
            (veQryLaneDirRdLnDs(rld)==vePOS)?len:dist[1],
						smTrue);
						
			touchIntr = veTRUE;
			
			/*initialise GD structure */
			GD.pEndRLD 				= endrld;
			GD.pNbEndRLD 			= nEndRLD;
			GD.pEndRLDDistFromInterX = dist_from_intr;
			GD.pVisitedInterX 		= visit;/*Array of interX we keeping trakof*/
			GD.pNbVisitedInterX 	= nIntr;/*nb inter in the array */
			GD.pVisitedRoads 		= roads;/*Array of roads we keep trak of */
			GD.pNbVisitedRoads 		= nRds;/*nb roads in the array */

			SearchEndRdLnFromIntersection(
				&GD,
				*rld,
				(vePOS==veQryLaneDirRdLnDs(rld ))?intr[0]:intr[1],
				size,
				(vePOS==veQryLaneDirRdLnDs(rld ))?dist[0]:dist[1],/*A distance we are passed */	
				&OnReverse,
				currentGraphLevel		);
			
			if (*nEndRLD >= MAX_RLD) break;
			
#ifdef DEBUG_TM_FindEndRdLn						
				{ 
					int i;	
					veRd rd;				
					fprintf(stderr,"//-------------------------------------------------------------//\n");
					fprintf(stderr, "after SearchEndRdLnFromIntersection in FindEndRld ....  Road Pieces:\n");
					fprintf(stderr, "FROM GD struct  ....  Road Pieces:\n");
					for (i = 0; i < *GD.pNbVisitedRoads; i++) {
						rd.rid = GD.pVisitedRoads[i].rd.rid;
						if (GD.pVisitedRoads[i].rd.rid == -1) continue; 
						fprintf(stderr, "\tRoad = %s, d1 = %g, d2 = %g, isAeahDriver %s\n",
								       veQryRoadName(&rd), 
										GD.pVisitedRoads[i].d1, GD.pVisitedRoads[i].d2,
									(GD.pVisitedRoads[i].isAheadDriver==veTRUE?"Yes":"NO"));
					}
					fprintf(stderr,"//-------------------------------------------------------------//\n");
					fprintf(stderr, "FROM pointer function  struct  ....  Road Pieces:\n");
					for (i = 0; i < *nRds; i++) {
						rd.rid = roads[i].rd.rid;
						if (roads[i].rd.rid == -1) continue; 
						fprintf(stderr, "\tRoad = %s, d1 = %g, d2 = %g, isAeahDriver %s\n",
								       veQryRoadName(&rd), 
										roads[i].d1, roads[i].d2,
									(roads[i].isAheadDriver==veTRUE?"Yes":"NO"));
					}
					fprintf(stderr,"//-------------------------------------------------------------//\n");
				}
#endif						
											 
			
		}/*End else  (dist[i] !>= size) */
		
	}
#else
	/* Searches in the forward direction of road (i = 0) and backward 
     * direction of road (i = 1) to find rld at distance of size. 
	 */
	/* TRIP */
	keep_size=size;
	for (i = 0; i < 2; i++) 
	{	

		/*Determin which way the driver is going */
		if (vePOS==veQryLaneDirRdLnDs(rld )){
			GD.IsSearchingInDriverDirection = 
				(i==0?smTrue:smFalse);
#ifdef DEBUG_TM_FindEndRdLn
			fprintf(stderr, "\n____________________________________________________________________\n"
			  "FindEndRdLn : IsSearchingInDriverDirection: %s \n",i==0?"smTrue":"smFalse");
#endif

		} else {
			GD.IsSearchingInDriverDirection = 
				(i==0?smFalse:smTrue);
#ifdef DEBUG_TM_FindEndRdLn
			fprintf(stderr, "\n____________________________________________________________________\n"
			  "FindEndRdLn : IsSearchingInDriverDirection: %s \n",i==0?"smFalse":"smTrue");
#endif
		}
	
		/* TRIP */
		if (GD.IsSearchingInDriverDirection==smFalse) {
			
			if ( IsRoadHighway( rd)) {
				size=GetBehindHighwayRange(keep_size);
			} else {
				size=GetBehindRange(keep_size);
			}
		} else {
			if ( IsRoadHighway( rd)) {
				size=GetAheadHighwayRange(keep_size);
			} else {
				size=GetAheadRange(keep_size);
			}
		}
			

		if (dist[i] >= size) 
		{/* the EndRld is ON this road segment */

#ifdef DEBUG_TM_FindEndRdLn
			fprintf(stderr," \n");
			fprintf(stderr, "dist[i] >= size\n");
			fprintf(stderr, "Direction : %s // dist[%d]=%g // Range = %g\n",
				(i==TM_HEADING_BACKWARD?"TM_HEADING_BACKWARD":"TM_HEADING_FORWARD"),
				i, dist[i],size); 
#endif
		
			 d = dist[1] + ((i==0)?(size):(-size)); /* dist[1] is curr driver pos */
			 
			 veBuildRdLnDs(&endrld[*nEndRLD], rd, lane, d);
    		   /* store the distane of this end point from th intr it is visited.
        		  This is helpful if we find final points that are on same road */
			 
			 dist_from_intr[*nEndRLD] = size + dist[1-i];
			 
			 AddRoad(	roads, 
						nRds, 
						rd, 
						(i==0)?dist[1]:d,
						(i==0)?d:dist[1],
						GD.IsSearchingInDriverDirection);
			 
			 (*nEndRLD)++;
			 
			 if (*nEndRLD >= MAX_RLD) break;
			 
			 
#ifdef DEBUG_TM_ROADPIECETRACK						
	{ 
		int i;	
		veRd rd;				
		fprintf(stderr,"//-------------------------------------------------------------//\n");
		fprintf(stderr, "FindEndRld: case dist[i] >= size\n");
		for (i = 0; i < *nRds; i++) {
			rd.rid = roads[i].rd.rid;
			if (roads[i].rd.rid == -1) continue; 
			fprintf(stderr, "\tRoad = %s, d1 = %g, d2 = %g, isAeahDriver %s\n",
						   veQryRoadName(&rd), 
							roads[i].d1, roads[i].d2,
						(roads[i].isAheadDriver==veTRUE?"Yes":"NO"));
		}
		fprintf(stderr,"//-------------------------------------------------------------//\n");
	}
#endif						
											 

			 
			 
		} else 
		{/* the EndRld is PAST this road segment */
#ifdef DEBUG_TM_FindEndRdLn
			fprintf(stderr," \n");
			fprintf(stderr, "dist[i] < size\n");
			fprintf(stderr, "Direction : %s // dist[%d]=%g // Range = %g\n",
				(i==TM_HEADING_BACKWARD?"TM_HEADING_BACKWARD":"TM_HEADING_FORWARD"),
				i, dist[i],size); 
			fprintf(stderr, "Add road and Calling SearchEndRdLnFromIntersection\n"); 
#endif
		
			AddRoad(	roads,
						nRds, 
						rd, 
						(i==0)?dist[1]:0.0, 
						(i==0)?len:dist[1],
						GD.IsSearchingInDriverDirection);
						
			touchIntr = veTRUE;
			
			/*initialise GD structure */
			GD.pEndRLD 				= endrld;
			GD.pNbEndRLD 			= nEndRLD;
			GD.pEndRLDDistFromInterX = dist_from_intr;
			GD.pVisitedInterX 		= visit;/*Array of interX we keeping trakof*/
			GD.pNbVisitedInterX 	= nIntr;/*nb inter in the array */
			GD.pVisitedRoads 		= roads;/*Array of roads we keep trak of */
			GD.pNbVisitedRoads 		= nRds;/*nb roads in the array */

			if (GD.IsSearchingInDriverDirection==smFalse)
			{
				/*if inter is not dummy, we do not want to go further */
				if (veFALSE==veIsIntrsxnTwoRoad(&intr[i])) {
#ifdef DEBUG_TM_FindEndRdLn
					fprintf(stderr, 
					"Going backward and inter is not dummy, stopping search\n");
#endif
					continue;
				} else {
					/*currentGraphLevel = TM_HEADING_BACKWARD_LEVEL;*/
#ifdef DEBUG_TM_FindEndRdLn
					fprintf(stderr, 
						"Going backward....inter is dummy "
						"currentGraphLevel set to %d\n",currentGraphLevel);
#endif
				}
			}

			SearchEndRdLnFromIntersection(
				&GD,
				*rld,
				intr[i],		
				size,
				dist[i],	/*A distance we are passed */	
				&OnReverse,
				currentGraphLevel		);
			
#ifdef DEBUG_TM_ROADPIECETRACK						
				{ 
					int i;	
					veRd rd;				
					fprintf(stderr,"//-------------------------------------------------------------//\n");
					fprintf(stderr, "after SearchEndRdLnFromIntersection in FindEndRld ....  Road Pieces:\n");
					fprintf(stderr, "FROM GD struct  ....  Road Pieces:\n");
					for (i = 0; i < *GD.pNbVisitedRoads; i++) {
						rd.rid = GD.pVisitedRoads[i].rd.rid;
						if (GD.pVisitedRoads[i].rd.rid == -1) continue; 
						fprintf(stderr, "\tRoad = %s, d1 = %g, d2 = %g, isAeahDriver %s\n",
								       veQryRoadName(&rd), 
										GD.pVisitedRoads[i].d1, GD.pVisitedRoads[i].d2,
									(GD.pVisitedRoads[i].isAheadDriver==veTRUE?"Yes":"NO"));
					}
					fprintf(stderr,"//-------------------------------------------------------------//\n");
					fprintf(stderr, "FROM pointer function  struct  ....  Road Pieces:\n");
					for (i = 0; i < *nRds; i++) {
						rd.rid = roads[i].rd.rid;
						if (roads[i].rd.rid == -1) continue; 
						fprintf(stderr, "\tRoad = %s, d1 = %g, d2 = %g, isAeahDriver %s\n",
								       veQryRoadName(&rd), 
										roads[i].d1, roads[i].d2,
									(roads[i].isAheadDriver==veTRUE?"Yes":"NO"));
					}
					fprintf(stderr,"//-------------------------------------------------------------//\n");
				}
#endif						
											 
			
			
			
			if (*nEndRLD >= MAX_RLD) break;
			
		}/*End else  (dist[i] !>= size) */
		
	}/*Endfor (i = 0; i < 2; i++) */
#endif
	
	
	
/* If two end points are on same road, they should not cross each other.
 *
 *	----------------------------------------
 *	----------------------------------> * 
 *	----------------------------------------   over lapping points.
 *   	* <---------------------------------
 *	----------------------------------------
 *
 *
 *	----------------------------------------
 *	 -----------> * 
 *	----------------------------------------   non over lapping points.
 *                         * <---------            (acceptable)
 *	----------------------------------------
 *
 *
 * Therefore remove over lapping points. We use dist_from_intr for this.
 */

	if (touchIntr == veTRUE) /* test not required if not passed any intr */
	{
		for (i = 0; i < (*nEndRLD - 1); i++)
		{
			if (veIsValidRoad(endrld[i].rd) == veFALSE) continue;
			
			if (dist_from_intr[i] < -0.9) continue;
			
			for (j = (i+1); j < *nEndRLD; j++)
			{
    			if (veIsValidRoad(endrld[j].rd) == veFALSE) continue;
				
    			if (veIsEqualRds(&endrld[i].rd, &endrld[j].rd) == veTRUE)
    			{
    				d = dist_from_intr[i] + dist_from_intr[j];
    				len = veQryRoadLength(&endrld[i].rd);
    				if (d > len) 
    				{
    					endrld[i].rd.rid = -1;
    					endrld[j].rd.rid = -1;
    				}
    				else if ( fabs( d - len ) <= 0.001 )
    				{
    					endrld[j].rd.rid = -1;
    				}
    				else
    				{
    					dist_from_intr[j] = -1.0;
    				}
    				break;
    			}
			}
		}
	}

	 /* If roads have same d1 and d2 then  :
	 	we used to remove them, but this is inconsistent with Endrld search
		we actually need to extend this road to a minimumdistance */
	for (j = 0; j < *nRds; j++)
	{
		if ( fabs( roads[j].d1 - roads[j].d2 ) <= 0.001 ) {
		
			if 	(roads[j].d1 == 0) 
			{
				roads[j].d2 = MIN(	veQryRoadLength(&roads[j].rd),
									TM_MOVE_AWAY_FROM_INTER_DIST);
	
			} else {
				
				roads[j].d1 = MAX(	0,
									roads[j].d2-TM_MOVE_AWAY_FROM_INTER_DIST);
	
				
			}			 
			
		}
			
	}
	

	/* Remove invalid rlds */
	i = 0;
	for (j = 0; j < *nEndRLD; j++)
	{
		if (veIsValidRoad(endrld[j].rd) == veFALSE) continue;
		
		if (i != j)
		{
			endrld[i] = endrld[j];
			index[i] = index[j];
		}
		
		i++;
	}
	*nEndRLD = i;

#if (EMERG_EXTENSIONS == 1)
	/*
	 * If the kind of SV we're seeking to create is not BH_INFRONT
	 * but, we're on a highway then just change the mode and hope
	 * for the best.
	 * Else if the kind of SV we're seeking is BH_CROSSING but we're
	 * are unable to find such a position on the network then just
	 * flip a coin and choose either BH_ONCOMING or BH_INFRONT and
	 * hope for the best.
	 */ 
	if( IsRoadHighway( &rld->rd ) && (*SVmode != BH_INFRONT) ){ 
		*SVmode = BH_INFRONT;
	} else if( !IsRoadHighway( &rld->rd ) && 
							(*nEndRLD == 1) && (*SVmode == BH_CROSSING) ){
		if( rRandDouble() < 0.5 ){
			*SVmode = BH_INFRONT;
		} else {
			*SVmode = BH_ONCOMING;
		}
	} 
#endif

	/* Remove invalid roads */
	i = 0;
	for (j = 0; j < *nRds; j++)
	{
		if (veIsValidRoad(roads[j].rd) == veFALSE) {
#ifdef DEBUG_TM_FindEndRdLn
		fprintf(stderr, 
					"FindRLD - there are #%d roads... Removing Road [%d]= ""%s"" \n",
					*nRds, j, veQryRoadName(&roads[j].rd));
#endif
			continue;
		}
		
		if (i != j) roads[i] = roads[j];
		
		i++;
	}
	*nRds = i;

	/* Remove invalid intersections */
	i = 0;
	for (j = 0; j < *nIntr; j++)
	{
		if (veIsValidIntrsxn(visit[j].intr) == veFALSE) continue;
		
		if (i != j) visit[i] = visit[j];
		
		i++;
	}
	*nIntr = i;

#ifdef DEBUG_TM_FindEndRdLn
	fprintf(stderr, 
		"-------------------------------------------------------------------------\n"); 
	fprintf(stderr, "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\\n"); 
	fprintf(stderr, "/\\/\\/\\/\\  END OF FindEndRdLn 		/\\/\\/\\/\\/\\\n"); 
#endif

}

#endif


////////////////////////////////////////////////////////////////////////////////
//
// Description:  This function does most of TM's work.
//
//
// Remarks:	
//
// Arguments:
//   
// Returns:	void
//
//////////////////////////////////////////////////////////////////////////////
void 
CTrafficManager::UserActivity( const CTrafMngrParseBlock* cpBlock )
{	

#ifdef DEBUG_USER_ACTIVITY
	gout << "=== " << GetFrame();
	gout << " === TM::UserActivity =====================" << endl;
#endif

	// Check if there is an External Driver
	const CDynObj* pOwnVehObj = cved->BindObjIdToClass( 0 );
	bool noOwnVeh = !pOwnVehObj || !pOwnVehObj->IsValid();
	if( noOwnVeh ) 
	{
#ifdef  DEBUG_USER_ACTIVITY
		gout << "  no external driver... return " << endl;
#endif

		return;
	}

	// Build own vehicle roadpos.
	GetOwnVehRoadPos();
	if( !m_roadPos.IsValid() )	
	{
#ifdef  DEBUG_USER_ACTIVITY
		gout << "  failed to obtain own vehicle roadpos....return " << endl;
#endif
		return;
	}
	
	if( !m_roadPos.IsRoad() )
	{
#ifdef  DEBUG_USER_ACTIVITY
		 gout << "  ownship on intersection... no obj will be created";
		 gout << "...return " << endl;
#endif

		return;
	}

//	gout << MessagePrefix() << "start---------" << endl;

	// Get ownship velocity            
	double ownVehVel = pOwnVehObj->GetVelImm() * cMS_TO_MPH;  // mph

#ifdef DEBUG_USER_ACTIVITY
	gout << "  ownVeh roadPos = " << m_roadPos << endl;
	gout << "  ownVeh vel = " << ownVehVel << " mph" << endl;
#endif

	vector<TTmObj> tmObjs;
	CountObjectsCreatedByTM( tmObjs );

	//gout << "[" << GetFrame() << "]: " << tmObjs.size() << " objs" << endl;
	vector<TTmObj>::const_iterator cItr;
	for( cItr = tmObjs.begin(); cItr != tmObjs.end(); cItr++ )
	{
		//gout << " " << cItr->cvedObjId << ": ahead=" << cItr->aheadOfOwnship << "  delete=" << cItr->markedForDeletion << "  dist=" << cItr->distToOwnship << " ft" << endl;
	}

#ifdef DEBUG_MNGR
	fprintf( stderr, "Vehicles created by me = %d, ", nb_tm_vehicles );
	fprintf( stderr, "total = %d, ", n_AllObjs );
	fprintf( stderr, "total behind eye sight %d\n", n_obj_behind_driver_eye );
#endif

	//
	// Build and maintain a path around the own vehicle's current 
	// road position. ** This needs to be modified to use the ownvehicle path if one exists **
	//
	MaintainPath();

#ifdef DEBUG_USER_ACTIVITY
	gout <<	"  number of input sets = " << m_inputSets.size() << endl;
#endif

	//
	// Check if it is time for creating more objects. If time elapsed
	// is less than run frequency desired by user, don't create
	// more objs. 
	//
	vector<CTrafMngrParseBlock::TInputSet>::const_iterator setItr;
	int setNum = 0;
	for( setItr = m_inputSets.begin(); setItr != m_inputSets.end(); setItr++ )
	{
		int maxObjects = setItr->maxObjects;
		double maxDensity = setItr->maxDensity;
		double runFreq = setItr->runFreq;        // seconds

#ifdef DEBUG_USER_ACTIVITY	
	//	gout << "	input set name = " << setItr->name  << endl;
	//	gout << "	createDist     = "  << setItr->createDist << endl;
	//	gout << "	deleteDist     = "  << setItr->deleteDist << endl;
	//	gout << "	absDeleteDist  = "  << setItr->absDeleteDist << endl;
	//	gout << "	minDensity     = "  << setItr->minDensity << endl;
		gout << "   maxDensity     = "  << maxDensity << " per mile" << endl;
		gout << "   maxObjects     = "  << maxObjects << " total" << endl;
		gout << "   runFreq        = "  << runFreq << " s" << endl;
		gout << endl;
#endif
		
		double currTime = GetFrame() * GetTimeStepDuration();
		double timeElapsedSinceObjCreation = currTime - m_timeAtObjCreation;

#ifdef DEBUG_USER_ACTIVITY
		gout << "  m_timeAtObjCreation = " << m_timeAtObjCreation << endl;
		gout << "  time elapsed since last creation = "
			 << timeElapsedSinceObjCreation << " s" << endl;
#endif

		bool needToWait = timeElapsedSinceObjCreation < runFreq;
		if( needToWait )
		{
			continue;
		}

		vector<CTrafMngrParseBlock::TWeight> composition;
		composition = setItr->solWeights;

#ifdef DEBUG_USER_ACTIVITY
		gout << "  number of vehicle types = " << composition.size() << endl;
		gout << "  vehicle types and weights > 0: " << endl;
		gout << "  composition size = " << composition.size() << endl;
#endif

		if( composition.size() == 0 )
		{
			cerr << MessagePrefix();
			cerr << " need sol names and weights in input set ... return ";
			cerr << endl;

			return;
		}

		// Ramdomly picking a vehicle type with desired probability.
		// Assuming weights entered by user are in percentage and that
		// the total of all weights is 100 percent.
		double min = 0.0;
		double max = 1.0;
		
		double numRnd = m_pRootCollection->m_rng.RandomDoubleRange( min, max, m_rndStreamId );
		double sum = 0.0;
		string solName;
		
		vector<CTrafMngrParseBlock::TWeight>::iterator compItr;
		for( 
			compItr = composition.begin(); 
			compItr != composition.end(); 
			compItr++ 
			)
		{
			string currSol = compItr->name;
			double currWeight = static_cast<double>(compItr->weight);

#ifdef DEBUG_USER_ACTIVITY
			if( currWeight > 0 )
			{
				gout << "  weight = " << currWeight;
				gout << "  sol name = " << currSol << endl;
			}
#endif
			sum = sum + ( currWeight / 100 );
			if( numRnd <= sum || compItr + 1 == composition.end() )
			{

#ifdef DEBUG_USER_ACTIVITY
				gout << " numRnd = " << numRnd << endl;
				gout << " sum = " << sum << endl;
				gout << " picked sol = " << currSol << endl;
				gout << " picked weight = " << currWeight << endl;
#endif

				solName = currSol;
				break;
			}

		}

		//
		// Check if any objs need to be deleted.
		//
		int objsToKill = tmObjs.size() - maxObjects;
		LimitOverload( tmObjs, objsToKill );

		//
		// Recycle vehicles that were created by me but have travelled
		// too far away.  If we delete any of them, we quit.
		//
		const double cDELETION_DIST_AHEAD = setItr->createDist + 1000.0;
		const double cDELETION_DIST_BEHIND = 1000.0;
		//int numKilled = KillTmObjsDist( tmObjs, cDELETION_DIST_AHEAD, cDELETION_DIST_BEHIND );
//		gout << MessagePrefix() << "setNum = " << setNum << endl;
		int numKilled = KillTmObjsDist( tmObjs, cDELETION_DIST_AHEAD, setItr->deleteDist );

		MaintainPathDist( cDELETION_DIST_BEHIND, cDELETION_DIST_AHEAD );

		// Check if the current number of objs created by the TM is less than the number
		// of objs desired. If so, create more objs with one at a time.
		vector<int> objs;
		cved->GetObj( "TM_Ado", objs );

#ifdef DEBUG_USER_ACTIVITY
		gout << "  maxObjects = " << maxObjects << endl;
		gout << "  TM created " << objs.size() << " objects" << endl;
#endif		
		bool createMoreObjects = objs.size() < maxObjects;
		if( createMoreObjects )
		{
			vector<TTmCreationPoint> creationPoints;

			// Get the creation points list.
			if( m_roadPos.IsRoad() )
			{
				CRoad road = m_roadPos.GetRoad();
				if( road.IsHighway() )
				{
					FindCreationPointsOnHighway( 
								creationPoints, 
								maxDensity, 
								ownVehVel 
								);
				}
				else
				{
					FindCreationPoints( 
								creationPoints, 
								maxDensity, 
								ownVehVel 
								);
				}
			}

			CreateObjects( creationPoints, solName );
		}

		setNum++;
	}	// for each input set
} // end of UserActivity




//////////////////////////////////////////////////////////////////////////////
void 
CTrafficManager::UserDeletion( const CTrafMngrParseBlock* )
{

} // end of UserDeletion



