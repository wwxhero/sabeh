/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: ado_follow.cxx,v 1.63 2015/09/25 14:10:42 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    March, 2000
 *
 * Description:  Contains code for the Follow HCSM.
 *
 ****************************************************************************/
#include "hcsmpch.h"
#include "vehdyncommand.h"
#include "dynobj.h"
#include "controllers.h"
#include "support.h"

#include <pi_iostream>
using namespace std;
bool debugFollow = false;

//
// Debugging macros.
//
#undef DEBUG_FOLLOW             //2  // needs CVED id of object			
#undef DEBUG_GET_LEAD_OBJECT_ID //2  // needs CVED id of object
#undef DEBUG_NADS // "Follow"

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Finds an object to follow.
//
// Remarks:  Given a vector of objects sorted by distance to the owner
//   object, this function finds an object to follow.  This function
//   ignores all invalid and non-vehicle objects. 
//
// Arguments:
//   objs      - A vector of objects sorted by distance to the owner object.
//   leadObjId - The CVED id of the object to follow.
//
// Returns: A boolean indicating if an object to follow was found.
//
//////////////////////////////////////////////////////////////////////////////
bool
CFollow::GetLeadObjectId( 
			CAdoInfoPtr pI, 
			const vector<int>& objs, 
			int& leadObjId 
			)
{

#ifdef DEBUG_GET_LEAD_OBJECT_ID
//	bool debugThisObj = pI->m_objName == DEBUG_NADS;

	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_GET_LEAD_OBJECT_ID;
	if( debugThisObj ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== GetLeadObjectId ==============" << endl;
	}
#endif

	bool foundObj = false;
	vector<int>::const_iterator i;
	for( i = objs.begin(); i != objs.end(); i++ ) 
	{
		int objId = *i;

#ifdef DEBUG_GET_LEAD_OBJECT_ID
		if( debugThisObj ) 
		{
			gout << "  looking at object " << objId << endl;
		}
#endif

		// don't look at myself
		if( objId == pI->m_pObj->GetId() ) 
		{
			
#ifdef DEBUG_GET_LEAD_OBJECT_ID
			if( debugThisObj ) 
			{
				gout << "    same object as me...skip " << endl;
			}
#endif

			continue;
		}

		// get the CVED object
		const CDynObj* pLeadObj = cved->BindObjIdToClass( objId );

		// ignore invalid objects
		bool invalidObj = !pLeadObj || !pLeadObj->IsValid();
		if( invalidObj ) 
		{
#ifdef DEBUG_GET_LEAD_OBJECT_ID
			if( debugThisObj )  gout << "    not a valid object" << endl;
#endif

			continue;
		}

		//
		// Ignore all objects execpt for vehicles, ddo and exteral
		// driver.
		//
		cvEObjType leadObjType = pLeadObj->GetType();

		bool notInMask = ( 
			leadObjType != eCV_VEHICLE &&
			leadObjType != eCV_TRAJ_FOLLOWER &&
			leadObjType != eCV_EXTERNAL_DRIVER &&
			leadObjType != eCV_TRAILER &&
			leadObjType != eCV_EXTERNAL_TRAILER
			);
		if( notInMask )  continue;

		//
		// Lead object is not the own vehicle.
		//
		CHcsm* pLeadHcsm = m_pRootCollection->GetHcsm( pLeadObj->GetHcsmId() );
		if( !pLeadHcsm ) 
		{
			gout << MessagePrefix( pI->m_pObj->GetId() );
			gout << "cannot get pointer to lead hcsm " << pLeadObj->GetHcsmId() <<endl;

			continue;
		}

		CRoadPos leadRoadPos;
		bool haveValFromMonitor;

		if ( objId == 1 )  // external trailer 
		{
			haveValFromMonitor = pLeadHcsm->GetMonitorByName( 
															"TrailerPos", 
															&leadRoadPos
															);
		}
		else
		{
			haveValFromMonitor = pLeadHcsm->GetMonitorByName( 
															"RoadPos", 
															&leadRoadPos
															);
		}

		if( !haveValFromMonitor || !leadRoadPos.IsValid() ) 
		{
//			gout << MessagePrefix( pI->m_pObj->GetId() );
//			gout << "failed to get lead road position for object ";
//			gout << objId << endl;

			continue;
		}

		//
		// Look at offsets and stuff.
		//
		bool onSameRoad = (
				leadRoadPos.IsRoad() && 
				pI->m_roadPos.IsRoad() &&
				leadRoadPos.GetRoad() == pI->m_roadPos.GetRoad()
				);
		if( onSameRoad )
		{
			double leadWidth = cved->GetObjWidth( objId );
			double clearance = fabs( pI->m_roadPos.GetLateralDistance( leadRoadPos ) );
#ifdef DEBUG_GET_LEAD_OBJECT_ID
			if( debugThisObj )
			{
				gout << "    clearance1 = " << clearance << endl;
			}
#endif
			clearance -= ( leadWidth * 0.5 ) + ( pI->m_objWidth * 0.5 );

#ifdef DEBUG_GET_LEAD_OBJECT_ID
			if( debugThisObj )
			{
				gout << "    clearance2 = " << clearance << endl;
			}
#endif

			bool haveSideClearance = clearance > 0.5;
			if( haveSideClearance )
			{
				bool laneChange = (
							pI->m_pCurrLcCond != NULL &&
							!pI->m_pCurrLcCond->IsForcedLaneOffset() &&
							pI->m_pCurrLcCond->IsTargLane() &&
							pI->m_pCurrLcCond->GetTargLane().GetRelativeId() == 
							leadRoadPos.GetLane().GetRelativeId()
							);
				if( !laneChange )
				{
					//
					// Check to see if this vehicle is lane changing into my
					// lane.
					//

					//
					// To be more efficient...check to see if the lead
					// vehicle has its turn signals on or not.
					//
					bool leadChangeLeft;
					bool leadChangeRight;
					if( pLeadObj->GetType() != eCV_EXTERNAL_DRIVER )
					{
						const CVehicleObj* pLeadVehObj = dynamic_cast<const CVehicleObj*>( pLeadObj );
						if (pLeadVehObj){
							leadChangeLeft = pLeadVehObj->GetVisualState() & cCV_LEFT_TURN_SIGNAL;
							leadChangeRight = pLeadVehObj->GetVisualState() & cCV_RIGHT_TURN_SIGNAL;
						}else{
							leadChangeLeft  = false;
							leadChangeRight = false;						
						}
					}
					else
					{
						leadChangeLeft  = false;
						leadChangeRight = false;
					}

#ifdef DEBUG_GET_LEAD_OBJECT_ID
					gout << "    leadChangeLeft  = " << leadChangeLeft << endl;
					gout << "    leadChangeRight = " << leadChangeRight << endl;
#endif

					if( leadChangeLeft )
					{
						//
						// Lead object is changing lanes to the left...check
						// to my right.
						//
						bool checkRight = (
									pI->m_roadPos.IsRoad() &&
									pI->m_currLane.IsValid() &&
									!pI->m_currLane.IsRightMost()
									);
						if( checkRight )
						{
							CLane rightLane = pI->m_currLane.GetRight();
							bool leadLcIntoMyLane = (
								leadRoadPos.GetLane().GetRelativeId() == 
								rightLane.GetRelativeId()
										);
							if( !leadLcIntoMyLane )
							{							
#ifdef DEBUG_GET_LEAD_OBJECT_ID
								if( debugThisObj )
								{
									gout << "    skipping...not changing into my lane";
									gout << endl;
								}
#endif
								continue;
							}
						}
						else
						{
#ifdef DEBUG_GET_LEAD_OBJECT_ID
							if( debugThisObj )
							{
								gout << "    skipping...no lane to my right";
								gout << endl;
							}
#endif
							continue;
						}
					}
					else if( leadChangeRight )
					{
						//
						// Lead object is changing lanes to the right...check
						// to my left.
						//
						bool checkLeft = (
									pI->m_roadPos.IsRoad() &&
									pI->m_currLane.IsValid() &&
									!pI->m_currLane.IsLeftMost()
									);
						if( checkLeft )
						{
							CLane leftLane = pI->m_currLane.GetLeft();
							bool leadLcIntoMyLane = (
								leadRoadPos.GetLane().GetRelativeId() == 
								leftLane.GetRelativeId()
										);
							if( !leadLcIntoMyLane )
							{							
#ifdef DEBUG_GET_LEAD_OBJECT_ID
								if( debugThisObj )
								{
									gout << "    skipping...not changing into my lane";
									gout << endl;
								}
#endif
								continue;
							}
						}
						else
						{
#ifdef DEBUG_GET_LEAD_OBJECT_ID
							if( debugThisObj )
							{
								gout << "    skipping...no lane to my left";
								gout << endl;
							}
#endif
							continue;
						}
					}
					else
					{
						//
						// Lead Object is not changing lanes.
						//
#ifdef DEBUG_GET_LEAD_OBJECT_ID
						if( debugThisObj )
						{
							gout << "    skipping...have clearance" << endl;
						}
#endif
						continue;
					}
				}
			}
		}

		//
		// Make sure that the lead object is headed in the same direction
		// as the lane on my path.
		//
		CVector3D leadObjTan = pLeadObj->GetTan();
		leadObjTan.m_k = 0.0;
		if( leadRoadPos.IsRoad() )
		{
			int laneCrdrId = pI->m_pPath->GetLaneIdFromRoad(
													&pI->m_roadPos,
													leadRoadPos.GetRoad()
													);
			CLane origLane = leadRoadPos.GetLane();
			int origLaneId = leadRoadPos.GetLane().GetRelativeId();

			bool notOnSameLane = laneCrdrId != origLaneId;
			if( notOnSameLane )
			{
				CLane leadLaneOnPath( leadRoadPos.GetRoad(), laneCrdrId );
				leadRoadPos.SetLane( leadLaneOnPath );
				leadRoadPos.SetOffset( 0.0 );

				CVector3D leadRoadPosTan = leadRoadPos.GetTangent();
				leadRoadPosTan.m_k = 0.0;

				double dotP = leadRoadPosTan.DotP( leadObjTan );

				//
				// If the dotP is near negative 1 then the lead object is
				// oncoming on my lane...ignore it.
				//
				bool leadObjOncoming = dotP - ( -1.0 ) < 0.1;
#ifdef DEBUG_GET_LEAD_OBJECT_ID
				if( debugThisObj )
				{
					gout << "    lrp=" << leadRoadPos << endl;
					gout << "    lot=" << leadObjTan;
					gout << "   lrpt=" << leadRoadPosTan;
					gout << "   dotP=" << dotP << endl;
				}
#endif

				if( leadObjOncoming )
				{
#ifdef DEBUG_GET_LEAD_OBJECT_ID
					if( debugThisObj )
					{
						gout << "    this object is oncoming on my lane...skip";
						gout << endl;
					}
#endif
					continue;
				}
                continue;
			}
		}else{
            //its possible to have a vehicle in our path, but in a different lane, 
            //we need to find if thats the case

            
            bool sameIntr = !leadRoadPos.IsRoad() && !pI->m_roadPos.IsRoad() &&
                leadRoadPos.GetIntrsctn().GetId() == pI->m_roadPos.GetIntrsctn().GetId();
            if (sameIntr){
                vector< pair<int,double> > driver_crds,lead_crds;
                leadRoadPos.GetCorridors(lead_crds);
                pI->m_roadPos.GetCorridors(driver_crds);
                auto inter = pI->m_roadPos.GetIntrsctn();
                bool same_crdr = false;
                bool onOverlappingCrdr = false;
                int idx = inter.GetCrdrIdx();
                //see if we have any overlap between our crds and the lead veh crdrs;
                for (auto itr = driver_crds.begin(); itr != driver_crds.end(); itr++){
                    if (leadRoadPos.HasCorridor(itr->first - idx )){
                        same_crdr = true;
                        onOverlappingCrdr = true;
                        break;
                    }
                }
                if (!same_crdr){
                    //the vehicles are not on the same crdr, lets see if crdr over laps with the
                    //lead vehicles crdr
                    CPoint3D pos = pI->m_roadPos.GetBestXYZ();
                    CRoadPos reProjected(*cved, pos);
                    for (auto itr = driver_crds.begin(); itr != driver_crds.end(); itr++){
                        int idx = pI->m_roadPos.GetIntrsctn().GetCrdrIdx();
                        CVED::CCrdr crdr(pI->m_roadPos.GetIntrsctn(),itr->first - idx);
                        for (auto ditr = lead_crds.begin(); ditr != lead_crds.end(); ditr++){
                            if (crdr.GetFirstMrgDist(ditr->first - idx) >= 0 || 
                                crdr.GetLastMrgDist(ditr->first - idx) >= 0){
                                onOverlappingCrdr = true;
                                break;
                            }
                            
                        }
                    }
                    
                }
                if (!onOverlappingCrdr){
                    //the veh is not on a crdr that overlaps with the ADOs current crdr, is veh is not a threat
                    continue; //these two crds in no way overlap;
                }
                
            }else{
                //lets check if the 
                auto nextInter = pI->m_roadPos.GetNextIntrsctn();
                bool sameNextIntr = !leadRoadPos.IsRoad() && 
                                    pI->m_roadPos.IsRoad() &&
                                    nextInter.IsValid() &&
                                    leadRoadPos.GetIntrsctn().GetId() == nextInter.GetId();
                if (sameNextIntr){
                    vector< pair<int,double> > lead_crds;
                    leadRoadPos.GetCorridors(lead_crds);
                    bool driversLaneLeadsToLeadVeh = false;
                    for (auto ditr = lead_crds.begin(); ditr != lead_crds.end(); ditr++){
                        CVED::CCrdr crdr(*cved,ditr->first);
                        if (crdr.GetSrcLnIdx() == pI->m_roadPos.GetLane().GetIndex()){
                            driversLaneLeadsToLeadVeh = true;
                            break;
                        }
                    }
                    if (!driversLaneLeadsToLeadVeh){
                        //the lead veh is on a crdr our lane does not connect to
                        continue;
                    }

                }
            }

            
        }


#ifdef DEBUG_GET_LEAD_OBJECT_ID
		if( debugThisObj )
		{
			gout << "  ** following obj " << objId << endl;
		}
#endif

		leadObjId = objId;
		foundObj = true;
		break;

	}

	return foundObj;

}  // end of GetLeadObjectId


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the distance to the own vehicle.
//
// Remarks:  
//
// Arguments:
//   pI      - A pointer to the main ado info structure.
//   pDynObj - A pointer to the own vehicle.
//
// Returns: The distance to the own vehicle.
//
//////////////////////////////////////////////////////////////////////////////
double
CFollow::ComputeDistToOwnVehicle( CAdoInfoPtr pI, const CDynObj* pDynObj )
{

	//
	// Lead object is the own vehicle.
	//
	const CExternalDriverObj* pOwnVehicle = 
						dynamic_cast<const CExternalDriverObj*>( pDynObj );

	double distToOwnVehicle = 0.0;

	if( pOwnVehicle ) 
	{
		//
		// Got a valid pointer to own vehicle.
		//
		CPoint3D ownVehiclePos = pOwnVehicle->GetPosHighFreq();

		bool haveValidOwnVehRoadPos = (
					m_ownVehicleRefreshFrame + 1 == GetFrame()
					);
		if( haveValidOwnVehRoadPos ) 
		{
			//
			// We have a road position for the own vehicle from
			// the previous frame.  Use this to compute the new
			// road position.
			//
			m_ownVehicleRoadPos.SetXYZ( ownVehiclePos );

			if( m_ownVehicleRoadPos.IsValid() ) 
			{
				distToOwnVehicle = pI->m_pPath->GetLength( 
													&pI->m_roadPos,
													&m_ownVehicleRoadPos 
													);

				m_ownVehicleRefreshFrame = GetFrame();
			}
			else 
			{
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "unable to compute own vehicle road pos" << endl;
				gout << "  cartPos = " << ownVehiclePos << endl;
			}
		}
		else 
		{
			//
			// We don't a road position for the own vehicle from 
			// the previous frame.  Build a new road position.
			//
			CRoadPos roadPos( *cved, ownVehiclePos );
			
			if( roadPos.IsValid() ) 
			{
				distToOwnVehicle = pI->m_pPath->GetLength( 
													&pI->m_roadPos,
													&roadPos 
													);

				m_ownVehicleRoadPos      = roadPos;
				m_ownVehicleRefreshFrame = GetFrame();
			}
			else 
			{
				gout << MessagePrefix( pI->m_pObj->GetId() );
				gout << "unable to compute own vehicle road pos" << endl;
				gout << "  cartPos = " << ownVehiclePos << endl;
			}

		}

	}
	else 
	{
		//
		// Unable to get pointer to own vehicle.
		//
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "unable to get pointer to own vehicle" << endl;
	}

	return distToOwnVehicle;

}  // end of ComputeDistToOwnVehicle


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the distance at which to follow the lead vehicle.
//
// Remarks:  
//
// Arguments:
//   pI      - A pointer to the main ado info structure.
//
// Returns: The distance to follow (in meters).
//
//////////////////////////////////////////////////////////////////////////////
double
CFollow::ComputeFollowDist( CAdoInfoPtr pI, double leadVel )
{

	double refreshTime = (
			( GetFrame() - pI->m_followRefreshFrame ) *
			pI->m_timeStepDuration
			);
	const double cREFRESH_FOLLOW_TIME = 15.0;  // seconds
	bool refresh = (
			refreshTime > cREFRESH_FOLLOW_TIME || 
			pI->m_followRefreshFrame < 0
			);

	if ( refresh ) 
	{
		//
		// Compute a random value between min and max.
		//
		m_followRandomization = m_pRootCollection->m_rng.RandomDoubleRange(
										pI->m_followMin,
										pI->m_followMax,
										pI->m_rngStreamId
										);
		pI->m_followRefreshFrame = GetFrame();
#if 0
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "isTime = " << pI->m_isFollowTime << ":  ";
		gout << m_followRandomization << endl;
		gout << "  min = " << pI->m_followMin;
		gout << "   max = " << pI->m_followMax << endl;
#endif
	}

	if( m_followRandomization < 0 ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "invalid or uninitialize followRandomization value = ";
		gout << m_followRandomization << endl;

		if( pI->m_isFollowTime )
		{
			m_followRandomization = 1.0;   // second
		}
		else
		{
			m_followRandomization = 100.0; // feet
		}
	}

	double distToFollow;
	if( pI->m_isFollowTime ) 
	{
		distToFollow = leadVel * m_followRandomization * cFEET_TO_METER;
	}
	else 
	{
		distToFollow = m_followRandomization * cFEET_TO_METER;
	}
	
	const double cMIN_FOLLOW_DIST = 5.0;  // meters
	if( distToFollow < cMIN_FOLLOW_DIST && leadVel > 1.0 )  distToFollow = cMIN_FOLLOW_DIST;

#if 0
	gout << MessagePrefix( pI->m_pObj->GetId() );
	gout << "distToFollow = " << distToFollow * cMETER_TO_FEET << "ft";
	gout << endl;
#endif

	return distToFollow;

}  // end of ComputeFollowDist


void 
CFollow::Creation()
{

	m_ownVehicleRefreshFrame = -10;  // keep less than -1
	m_followRandomization = -1.0;
	m_currFollEngTime = -1;
	m_initFollDist = -1;
	m_prevFollowActualDist = -1.0;
	m_lastLeadId = -1;

}  // FollowCreation


void 
CFollow::PreActivity()
{

}  // FollowPreActivity


//////////////////////////////////////////////////////////////////////////////
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
CFollow::PostActivity()
{

	//
	// Set outputs to nothing.
	//
	SetOutputTargAccelNoValue();

	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_FOLLOW
  static counter = 1;
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_FOLLOW;
	if( debugThisObj ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== FOLLOW POST-ACTIVITY ==============" << endl;
    counter = 0;
	}
  counter++;
#endif

	//
	// Get the objects on my path.
	//
	vector<int> objs;
	int numObjs = pI->m_pPath->GetObjectsOnPath( pI->m_roadPos, objs );

#ifdef DEBUG_FOLLOW
	if( debugThisObj ) 
	{
		gout << "  found " << numObjs << " objects on path:";
		vector<int>::iterator i;
		for( i = objs.begin(); i != objs.end(); i++ )
		{
			gout << "  " << *i;
		}
		gout << endl;
	}
#endif

	if( numObjs <= 0 )  return;

	//
	// Get the vehicle closest to me.  Assuming that the
	// "objs" vector contains objects sorted by distance.
	//
	int leadObjId;
	bool foundObject = GetLeadObjectId( pI, objs, leadObjId );
	if( !foundObject )  return;




#ifdef DEBUG_FOLLOW
	if( debugThisObj ) 
	{
		gout << "  following object " << leadObjId << endl;
	}
#endif

	// 
	// Use the follow object's id to get the CVED object.
	//
	const CDynObj* pLeadObj = cved->BindObjIdToClass( leadObjId );
	if( !pLeadObj || !pLeadObj->IsValid() ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "invalid follow object (id = ";
		gout << leadObjId << ")" << endl;

		return;
	}

#ifdef DEBUG_FOLLOW
	if( debugThisObj ) 
	{
		gout << "    type = " << cvObjType2String( pLeadObj->GetType() );
		gout << "  name = " << pLeadObj->GetName();
		gout << "  hcsmid = " << pLeadObj->GetHcsmId() << endl;
		gout << "  pos = " << pLeadObj->GetPos() << endl;
	}
#endif

	//
	// Calculate the distance to follow.
	//
	double distToFollow = ComputeFollowDist( pI, pLeadObj->GetVelImm() );

	//
	// Add the distance from my CG to my front bumper and the distance
	// from the follow object's CG to it's rear bumper.
	//
	double bumperAdjust = (
				( ( pI->m_pObj->GetXSize() * 0.5 ) + ( pLeadObj->GetXSize() * 0.5 ) ) *
				cFEET_TO_METER
				);
	distToFollow += bumperAdjust;

	if( distToFollow < 0.0 )   distToFollow = 0.0;

#ifdef DEBUG_FOLLOW
	if( debugThisObj ) 
	{
		gout << "  length =" << pI->m_pObj->GetXSize();
		gout << "  width =" << pI->m_pObj->GetYSize() << endl;
		gout << "  following dist = " << distToFollow * cMETER_TO_FEET;
		gout << "ft" << endl;
	}
#endif

	//
	// Calculate actual distance between me and the lead vehicle.
	//
	double actualDist = 0.0;

	//
	// Lead object is not the own vehicle.
	//
	CRoadPos leadRoadPos;
	CHcsm* pLeadHcsm = m_pRootCollection->GetHcsm( pLeadObj->GetHcsmId() );
	if( !pLeadHcsm ) 
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "cannot get pointer to lead hcsm" << endl;
	}
	else 
	{
		bool haveValFromMonitor;

		if ( leadObjId == 1 )  // external trailer 
		{
			haveValFromMonitor = pLeadHcsm->GetMonitorByName( 
															"TrailerPos", 
															&leadRoadPos
															);
		}
		else
		{
			haveValFromMonitor = pLeadHcsm->GetMonitorByName( 
														"RoadPos", 
														&leadRoadPos
														);
		}
		if( !haveValFromMonitor || !leadRoadPos.IsValid() ) 
		{
//			gout << MessagePrefix( pI->m_pObj->GetId() );
//			gout << "failed to get lead road position";
//			gout << endl;
		}
		else 
		{
#ifdef DEBUG_FOLLOW
			if( debugThisObj ) 
			{
				gout << "  lead road pos = " << leadRoadPos << endl;
			}
#endif
			actualDist = pI->m_pPath->GetLength( 
											&pI->m_roadPos, 
											&leadRoadPos 
											);

			bool getLengthError = fabs( actualDist ) < cNEAR_ZERO;
			if( getLengthError )
			{
				//
				// CPath's GetLength probably returned a 0.0 because it wasn't
				// able to figure out the length.  So, estimate the length by
				// calculating each vehicle's distance to the end of their
				// respective corridors.
				//
				if( !leadRoadPos.IsRoad() )
				{

					int currObjIntrsctnId, leadObjIntrsctnId;

					CCrdr leadCrdr = leadRoadPos.GetCorridor();
					leadObjIntrsctnId = leadCrdr.GetIntrsctn().GetId();

					if( pI->m_roadPos.IsRoad() )
					{
						// If current obj is on the source road of the intersection
						// the lead obj is on.	
						currObjIntrsctnId = pI->m_roadPos.GetLane().GetNextIntrsctn().GetId();

						if( currObjIntrsctnId == leadObjIntrsctnId )
						{
				
							double leadDistFromCrdrEnd = leadCrdr.GetLength() - leadRoadPos.GetDistance();
							if( leadDistFromCrdrEnd < 0.0 ) leadDistFromCrdrEnd = 0.0;

							double distFromCrdrEnd;
						
							CLane currLane = pI->m_roadPos.GetLane();
							if( currLane.GetDirection() == ePOS )
							{
								distFromCrdrEnd = currLane.GetRoad().GetLinearLength() - pI->m_roadPos.GetDistance();
							}
							else
							{
								distFromCrdrEnd = pI->m_roadPos.GetDistance();
							}

							CCrdr crdr = pI->m_pPath->GetNextCrdr( pI->m_roadPos );

							if( crdr.IsValid() )  
							{
								if( crdr.GetRelativeId() == leadCrdr.GetRelativeId() )
								{
									distFromCrdrEnd += crdr.GetLength();
								}
								else
								{
									distFromCrdrEnd += leadCrdr.GetLength();
								}
							}		
							
							if( distFromCrdrEnd < 0.0 )  distFromCrdrEnd = 0.0;
							

							actualDist = distFromCrdrEnd - leadDistFromCrdrEnd;
							if( actualDist < 0.0 ) actualDist = 0.0;
						}
					}	
					else
					{	

						// If current obj and lead obj are on the same intersection.
						currObjIntrsctnId = pI->m_roadPos.GetCorridor().GetIntrsctn().GetId();

						if( currObjIntrsctnId == leadObjIntrsctnId )
						{
							double leadObjDist = leadRoadPos.GetDistance();
							double currObjDist = pI->m_roadPos.GetDistance();

							int leadCrdrId = leadRoadPos.GetCorridor().GetRelativeId();
							int currCrdrId = pI->m_roadPos.GetCorridor().GetRelativeId();

							double leadCrdrLength = leadRoadPos.GetCorridor().GetLength();
							double currCrdrLength = pI->m_roadPos.GetCorridor().GetLength();


							double leadObjDistFromCrdrEnd = -1;
							double currObjDistFromCrdrEnd = -1;

							if( leadCrdrId == currCrdrId )
							{

								actualDist = fabs( leadObjDist - currObjDist );
							}
							else
							{	
								CLane leadObjDstntnLn = leadRoadPos.GetCorridor().GetDstntnLn();
								CLane currObjDstntnLn = pI->m_roadPos.GetCorridor().GetDstntnLn();

								if( leadObjDstntnLn == currObjDstntnLn )
								{
									leadObjDistFromCrdrEnd = leadCrdrLength - leadObjDist;
									currObjDistFromCrdrEnd  = currCrdrLength - currObjDist;

									actualDist = fabs( leadObjDistFromCrdrEnd - currObjDistFromCrdrEnd );
							
								}
								else
								{
									actualDist = fabs( leadObjDist - currObjDist );
								}


							}

#ifdef DEBUG_FOLLOW
							if( debugThisObj ) 
							{
								gout << " *** both objs on same intersection " << endl;
								gout << " *** leadObjDist = " << leadObjDist << endl;
								gout << " *** currObjDist = " << currObjDist << endl;

								gout << " *** curr crdr id = " << currCrdrId << endl;
								gout << " *** curr crdr length = " << currCrdrLength << endl;
								gout << " *** lead crdr id = " << leadCrdrId << endl;
								gout << " *** lead crdr length = " << leadCrdrLength << endl;

								gout << " *** leadObjDistFromCrdrEnd = " << leadObjDistFromCrdrEnd << endl;
								gout << " *** currObjDistFromCrdrEnd = " << currObjDistFromCrdrEnd << endl;

								gout << " *** actualDist = " << actualDist << endl;
							
							}
#endif
							
						}	// on same intersection

					}	// curr obj on intersection
				}	// lead obj on intersection
			}

#ifdef DEBUG_FOLLOW
			if( debugThisObj ) 
			{
				gout << "  lead road pos = " << leadRoadPos << endl;
				gout << "  actual dist = " << actualDist << "ft" << endl;
			}
#endif

			actualDist = actualDist * cFEET_TO_METER;

			// adjust actual distance from "center to center" to "bumper to bumper"
			actualDist -= bumperAdjust;

		}
	}

	double accel = GetAccel( pLeadObj );

	//
	// Write calculated information to the output parameters.
	//
#ifdef DEBUG_FOLLOW
	gout << "leadObjId = " << leadObjId;
	gout << "   distToFollow = " << distToFollow * cMETER_TO_FEET << "ft   ";
	gout << "adjusted actualDist = " << actualDist * cMETER_TO_FEET << "ft" << endl;
	gout << "leadVel = " << pLeadObj->GetVelImm() * cMS_TO_MPH << "mph   ";
	gout << "myVel = " << pI->m_currVel * cMS_TO_MPH << "mph   ";
	gout << "aggressiveness = " << pI->m_aggressiveness << endl;
#endif

	// meaning of variables
	// m_currFollEngTime - how long we have been following the current leader
	// m_lastLeadId - the object we were following during the last frame
	//

	if ( m_lastLeadId == leadObjId ) {
		m_currFollEngTime++;
	}
	else {
		m_currFollEngTime = 0;
		m_initFollDist    = actualDist;
	}
	m_lastLeadId = leadObjId;

	double targAccel;
	if( pI->m_maintainGap.IsActive() )
	{
		if( pI->m_objName == "Ado" )  debugFollow = true;
	}
	bool gotFollow = FollowController( 
								GetFrame(),
								GetTimeStepDuration(),
								leadObjId, 
								pI->m_FollowInfo,
								pI->m_FollowParams,
								actualDist,
								m_prevFollowActualDist,
								m_currFollEngTime,
								m_initFollDist,
								pLeadObj->GetVelImm(),
								pI->m_currVel,
								pI->m_prevVel,
								accel,
								pI->m_aggressiveness,
								85 * cMPH_TO_MS,		// HACK, give 85mph max speed
								targAccel,
								pI->m_followTtc
								);
	pI->m_followDist = actualDist * cMETER_TO_FEET;   // stores in ft
    pI->m_followTarget = leadObjId;
#if 0
	if( pI->m_maintainGap.IsActive() )
	{
		if( pI->m_objName == "Ado" )
		{
			debugFollow = false;
			double targDist;
			if ( pI->m_FollowParams.folTimeMode ) {
				targDist = pI->m_currVel * pI->m_FollowParams.folValue;
			}
			else {
				targDist = pI->m_FollowParams.folValue;
			}
			gout << "***** " << pI->m_objName << " ******" << endl;
			gout << "** leadObjId  = " << leadObjId << endl;
			gout << "** actualDist = " << actualDist << " m" << endl;
			gout << "** targDist   = " << targDist << endl;
			gout << "** targAccel  = " << targAccel << " m/s^2" << endl;
		}
	}
#endif

	if( !gotFollow ) 
	{
		SetOutputTargAccelNoValue();
	}
	else 
	{
		if( pI->m_maxAccelDueToCurv < targAccel ) 
		{
			targAccel = pI->m_maxAccelDueToCurv;
		}
#if 0  // code for when stuck behind a stopped ADO...doesn't really work
		bool needToAccelForLaneChange = (
					pI->m_currVel < 10.0 * cMPH_TO_MS &&
					pI->m_pCurrLcCond != NULL &&
					pI->m_pCurrLcCond->IsTargLane() &&
					leadRoadPos.IsValid() &&
					leadRoadPos.IsRoad() &&
					pI->m_pCurrLcCond->GetTargLane().GetId() != 
					leadRoadPos.GetLane().GetId()
					);
		if( needToAccelForLaneChange )
		{
			bool haveEnoughDist = actualDist - bumperAdjust > 0.5;
#ifdef DEBUG_FOLLOW
			gout << "needToAccelForLaneChange = " << needToAccelForLaneChange << endl;
			gout << "haveEnoughDist = " << haveEnoughDist << endl;
#endif
			if( haveEnoughDist )
			{
				targAccel = 2.0;
			}
		}
#endif
		SetOutputTargAccel( targAccel );
	}

	m_prevFollowActualDist = actualDist;

}  // FollowPostActivity



