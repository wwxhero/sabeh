
/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: ado_merge.cxx, $Id: ado_merge.cxx,v 1.6 2015/09/03 15:20:27 IOWA\dheitbri Exp $
 *
 * Author:  Huidi Tang
 *
 * Date:    November, 2002
 *
 * Description:  Contains code for the Merge HCSM.
 *
 ****************************************************************************/

#include "hcsmpch.h"
#include "controllers.h"
#include "support.h"
#include <pi_iostream>
using namespace std;



//
// Debugging macros.
//
#undef			DEBUG_USER_POST_ACTIVITY
#undef			DEBUG_GRAPHIC	//2 // cved id
#undef			DEBUG_PROCESS_OFF_STATE	//2
#undef			DEBUG_TRANSITION_OFF_TO_GAP_SEARCH	//2
#undef			DEBUG_COMPUTE_TTA	//2
#undef			DEBUG_COMPUTE_ACTUAL_DIST	//2
#undef			DEBUG_PROCESS_TRACK_STATE	//2
#undef			DEBUG_GAP_SEARCH	//2
#undef			DEBUG_TRANSITION_GAP_SEARCH_TO_OFF
#undef			DEBUG_TRANSITION_TRACK_TO_OFF	//2
#undef			DEBUG_COMPUTE_APPRCH_OBJDIST_TOP1	//2
#undef			DEBUG_TRANSITION_GAP_SEARCH_TO_TRACK


//
// Constants.
//
const double		cMERGE_TTC = 1.5;
const double		cMERGE_GAP_THRESHOLD = 2.0;
const double	cMERGE_MAX_DECEL = - 0.3 * cGRAVITY;
const double		cMERGE_TIME_FOR_GAP_SEARCH = 5.0;
const int		cMERGE_MAX_NUM_OBJS_FOR_VEL_CONTROL = 4;
const double		cMERGE_INVALID_TARGACCEL = 999.0;

const double    cMetersToFeet  = 3.28084;


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function initializes local variables.
//
// Remarks:  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////

void 
CMerge::UserCreation()
{
	m_state = eMERGE_OFF;
	m_isMergeLaneChangeDone = false;
	m_isOnMergeRoad = false;
	m_firstFrame = true;
	m_distToP1Ado = -1;
	m_distToP1ApprchObj = -1;
	m_lastLeadId = -1;
	m_prevFollowActualDist = -1.0;
	m_currFollEngTime = -1;
	m_initFollDist = -1;
	m_distAtP1 = -1;
	
}  // end of Creation


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The Merge HCSM pre-activity function.
//
// Remarks:  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CMerge::UserPreActivity()
{

}  


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function computes the time to arrival at point p1 which is 
//	defined to be at some distance after the first merging point of the merge corridor
//  and target corridor.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//	objId - The object identifier.
//	dist - The object distance to point p1 ( in ft ).
//	tta	- ( output )The time to arrival at pont P1 ( in seconds ).
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CMerge::ComputeTimeToArrival( CAdoInfoPtr& pI, int objId,  double dist, double& tta )
{

#ifdef DEBUG_COMPUTE_TTA
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_COMPUTE_TTA;
	if( debugThisObj )
	{
		gout << endl;
		gout << " ===========CMerge::ComputeTta ========== " << endl;
		gout << " compute tta for obj id = " << objId << endl;
	}
#endif

	// Get obj vel
	const CDynObj* pObj = cved->BindObjIdToClass( objId );

	// ignore invalid objects
	bool invalidObj = !pObj || !pObj->IsValid();
	if( invalidObj )
	{
		tta = -1;
		return;
	}

	double objCurrVel = pObj->GetVelImm();  // ms
			
	bool notStopped = objCurrVel > cNEAR_ZERO;
	if( notStopped )
	{
		tta = dist * cFEET_TO_METER / objCurrVel;

#ifdef DEBUG_COMPUTE_TTA
		if( debugThisObj )
		{
			gout << " curr vel = " << objCurrVel * cMS_TO_MPH << " mph " << endl;
			gout << " curr vel = " << objCurrVel << " ms " << endl;
			gout << " curr dist to P1 = " << dist << " ft " << endl;
			gout << " curr dist to p1 = " << dist * cFEET_TO_METER << " meters " << endl;
			gout << " tta = " << tta << endl;
		}
#endif
		
	}
	else
	{
		tta = 1000.0;
	}

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function finds the vehicle id that the ado on the merge road is
//	going to follow or lead during the merge.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//	gapObjId - The obj identifier.
//	targAccel - The target acceleration.
//
// Returns:	it returns true when a vehicle id is found and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::GapSearch( CAdoInfoPtr& pI, int& gapObjId, double& targAccel )
{

#ifdef DEBUG_GAP_SEARCH
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_GAP_SEARCH;
	if( debugThisObj )
	{
		gout << endl;
		gout << "==========CMerge::GapSearch=============" << endl;
	}
#endif


#ifdef DEBUG_GRAPHIC
	bool debugThisObj2 = pI->m_pObj->GetId() == DEBUG_GRAPHIC;
	if( debugThisObj2 )
	{
		gout << " gaphic debug: curr obj id = " << pI->m_pObj->GetId() << endl;
		gout << " debugThisObj2 = " << debugThisObj2 << endl;
	}
#endif


	// Get Ado dist to P1
	double distToP1;
	double currDist = pI->m_roadPos.GetDistance();

	bool isOnRoad = pI->m_roadPos.IsRoad();

	if( isOnRoad )
	{
		CRoad mergeRd = pI->m_roadPos.GetLane().GetRoad();
		double mergeRdLength = mergeRd.GetLinearLength();
		
		distToP1 = mergeRdLength - currDist + m_distAtP1;
	}
	else
	{
		distToP1 = m_distAtP1 - currDist;
	}

	m_distToP1Ado = distToP1;

	// Compute time to arrival to P1 
	double ttaAdoAtP1;
	ComputeTimeToArrival( pI, pI->m_pObj->GetId(), distToP1, ttaAdoAtP1 );

#ifdef	DEBUG_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " merge ado id = " << pI->m_pObj->GetId() << endl;
		gout << " merge ado tta to p1 = " << ttaAdoAtP1 << endl;
		gout << " merge ado dist to p1 = " << distToP1 << endl;
		gout << " merge ado curr dist  = " << currDist << endl;
	}
#endif

	int numObjs = pI->m_objsApprch.size();

#ifdef	DEBUG_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " get approching objs for merge ado " << pI->m_pObj->GetId() << endl;
		gout << " number of approaching objs = " <<  numObjs << endl;
	}
#endif

	CLane targCrdrSrcLn = m_targCrdr.GetSrcLn();
	CRoad targCrdrSrcRd = targCrdrSrcLn.GetRoad();
	bool isGapFound = false;

	if( numObjs == 0 )
	{

#ifdef	DEBUG_GAP_SEARCH
		if( debugThisObj )
		{
			gout << " no approaching obj...gap is found ...merge ado is leader " << endl;
		}
#endif

		isGapFound = true;
		targAccel = -1;
		gapObjId = -1;
		return true;
	}

	int numApprchObjsSlowerThanMergeAdo = 0;
	bool prevObjPassedP1 = false;
	// For each approaching obj.
	for ( int j = 0; j < numObjs; j++ )
	{
		int objId = pI->m_objsApprch[j].objId;
		double objDist = pI->m_objsApprch[j].objDist;
		bool isObjOnRoad = pI->m_objsApprch[j].isOnRoad;
		double distToMergePoint = pI->m_objsApprch[j].distFromOwner;

#ifdef	DEBUG_GAP_SEARCH
		if( debugThisObj )
		{
			gout << endl;
			gout << " apprch obj id = " << objId << endl;
			gout << " apprch obj dist = " << objDist << endl;
			gout << " apprch obj on road = " << isObjOnRoad << endl;
		}
#endif


		// Get obj dist to point P1
		double objDistToP1;

		if( !isObjOnRoad )	// If obj is on intersection
		{
			// Ignore obj passed point P1.
			if( objDist > m_distAtP1 )	
			{
				if( j != numObjs - 1 )
				{

#ifdef	DEBUG_GAP_SEARCH
					if( debugThisObj )
					{
						gout << " apprch obj " << objId << " passed p1, check next obj " << endl;
					}
#endif
					prevObjPassedP1 = true;
					continue;
				}
				else
				{

#ifdef	DEBUG_GAP_SEARCH
					if( debugThisObj )
					{
						gout << " apprch obj " << objId << " passed p1 but no follower, so gap is found ";
						gout << endl;
					}
#endif
					isGapFound = true;
					gapObjId = -1;
					targAccel = -1;
					return true;
				}
			}

			objDistToP1 = m_distAtP1 - objDist;
		}
		else	// If obj on target corridor source road
		{
			if( targCrdrSrcLn.GetDirection() == ePOS )
			{
				objDistToP1 = targCrdrSrcRd.GetLinearLength() - objDist + m_distAtP1;
			}
			else
			{
				objDistToP1 = objDist + m_distAtP1;
			}
		}

		m_distToP1ApprchObj = objDistToP1;

		// Compute tta to P1
		double ttaObjAtP1;
		ComputeTimeToArrival( pI, objId, objDistToP1, ttaObjAtP1 ); 


		// If ado passed p1, go from GS to OFF.
		if( distToP1 < 0 )
		{
			targAccel = cMERGE_INVALID_TARGACCEL;
			return false;
		}
		else
		{
			// Ignore an obj passed p1.
			if( objDistToP1 < 0 )
			{
				continue;
			}
		}

		// Pick a gap
		double diff = ttaObjAtP1 - ttaAdoAtP1;
		double gap;

#ifdef	DEBUG_GAP_SEARCH
		if( debugThisObj )
		{
			gout << " numApprchObjsSlowerThanMergeAdo = ";
			gout << numApprchObjsSlowerThanMergeAdo << endl;
			gout << " diff = " << diff << endl;
		}
#endif

		// For case when no gap is found.
		if( numApprchObjsSlowerThanMergeAdo >= 4 && diff >= 1.5 )
		{

#ifdef	DEBUG_GAP_SEARCH
			if( debugThisObj )
			{
				gout << " no gap is found ... need to check stop dist " << endl;
			}
#endif
			break;
		}


		// If ado can reach P1 ahead of approaching obj
		if( diff > 0 )
		{
			numApprchObjsSlowerThanMergeAdo++;

#ifdef	DEBUG_GAP_SEARCH
			if( debugThisObj )
			{
				gout << " merge ado reaches P1 ahead of apprch obj " << objId << endl;
				gout << " merge ado tta  = " << ttaAdoAtP1 << endl;
				gout << " apprch obj tta  = " << ttaObjAtP1 << endl;
			}
#endif
			if( diff > cMERGE_TTC )
			{
				gap = diff;
				isGapFound = true;

#ifdef	DEBUG_GAP_SEARCH
				if( debugThisObj )
				{
					gout << endl;
					gout << " meet ttc ..." << endl;
					gout << " gap is found, lead obj or follow obj " << objId << endl;
				}
#endif


				if( j == 0 )	// Ado is in front of all approaching obj
				{
					
					const CDynObj* pFollowObj = cved->BindObjIdToClass( objId );
					if( !pFollowObj || !pFollowObj->IsValid() )		return	false; // ??

						
					CPoint3D followObjPos = pFollowObj->GetPosImm();
					CPoint3D ownerObjPos = pI->m_pObj->GetPosImm();
					double radiusFollower = pFollowObj->GetXSize() * 0.5;

#ifdef	DEBUG_GRAPHIC
					if( debugThisObj2 )
					{
						gout << " followObjPos = " << followObjPos.m_x << " " << followObjPos.m_y << " " ;
						gout << followObjPos.m_z << endl;
						gout << " ownerObjPos = " << ownerObjPos.m_x << " " << ownerObjPos.m_y << " " ;
						gout << ownerObjPos.m_z << endl;
						gout << " radiussFollower = " << radiusFollower << endl;
					}
#endif
						
					pI->m_mergeInfo.clear();
					TMergeInfo node;
					node.pos = followObjPos;
					node.radius = radiusFollower;
					node.leadObjId = -1;
					pI->m_mergeInfo.push_back( node );
					gapObjId = -1;
						
				}		
				else	// When there is at least one approaching obj in front of Ado
				{
						int leadObjId;
						if( prevObjPassedP1 )
						{
							leadObjId = pI->m_objsApprch[j].objId;
						}
						else
						{

#ifdef	DEBUG_GAP_SEARCH
							if( debugThisObj )
							{
								gout << " #### apprch obj in gap front = ";
								gout << pI->m_objsApprch[j-1].objId << endl;
								gout << " #### apprch obj in gap back = " << objId << endl;
							}

#endif
							leadObjId = pI->m_objsApprch[j-1].objId;
						}
					
						
						const CDynObj* pLeadObj = cved->BindObjIdToClass( leadObjId );
						if( !pLeadObj || !pLeadObj->IsValid() )		return	false; // ??

						
						CPoint3D leadObjPos = pLeadObj->GetPosImm();
						CPoint3D ownerObjPos = pI->m_pObj->GetPosImm();
						double radiusLeader = pLeadObj->GetXSize() * 0.5;


#ifdef	DEBUG_GRAPHIC
						if( debugThisObj2 )
						{
							gout << " leadObjPos = " << leadObjPos.m_x << " " << leadObjPos.m_y << " " ;
							gout << leadObjPos.m_z << endl;
							gout << " ownerObjPos = " << ownerObjPos.m_x << " " << ownerObjPos.m_y << " " ;
							gout << ownerObjPos.m_z << endl;
							gout << " radiussLeader = " << radiusLeader << endl;
						}
#endif
					
						pI->m_mergeInfo.clear();
						TMergeInfo node;
						node.pos = leadObjPos;
						node.radius = radiusLeader;
						node.leadObjId = leadObjId;
						pI->m_mergeInfo.push_back( node );
						gapObjId = leadObjId;		
					
					}
					targAccel = -1;
					return true;
			}	// meet ttc
			else
			{
				if( j == numObjs - 1 )	// If there is no follower behind the obj.
				{
					gap = fabs( diff );
					isGapFound = true;

#ifdef	DEBUG_GAP_SEARCH
					if( debugThisObj )
					{
						gout << endl;
						gout << " does not meet ttc, but there is no follower ... ";
						gout << " so a gap is found " << endl;
						gout << " follow obj " << objId << endl;
					}
#endif


					const CDynObj* pLeadObj = cved->BindObjIdToClass( objId );
					if( !pLeadObj || !pLeadObj->IsValid() )		return	false; // ??

					CPoint3D leadObjPos = pLeadObj->GetPosImm();
					CPoint3D ownerObjPos = pI->m_pObj->GetPosImm();
					double radiusLeader = pLeadObj->GetXSize() * 0.5;


#ifdef	DEBUG_GRAPHIC
					if( debugThisObj2 )
					{
						gout << " leadObjPos = " << leadObjPos.m_x << " " << leadObjPos.m_y << " " ;
						gout << leadObjPos.m_z << endl;
						gout << " ownerObjPos = " << ownerObjPos.m_x << " " << ownerObjPos.m_y << " " ;
						gout << ownerObjPos.m_z << endl;
						gout << " radiussLeader = " << radiusLeader << endl;
					}
#endif

					// insert lead approaching obj
					pI->m_mergeInfo.clear();
					TMergeInfo node;
					node.pos = leadObjPos;
					node.radius = radiusLeader;
					node.leadObjId = objId;
					pI->m_mergeInfo.push_back( node );
					gapObjId = objId;
					targAccel = -1;
					return true;
				}	// not meet ttc
			}
		}	// reacher sooner
		else
		{

#ifdef	DEBUG_GAP_SEARCH
			if( debugThisObj )
			{
				gout << " merge ado reaches P1 later than apprch obj " << objId << endl;
				gout << " merge ado tta  = " << ttaAdoAtP1 << endl;
				gout << " apprch obj tta  = " << ttaObjAtP1 << endl;
			}
#endif

			// If there is no follower behind the approaching obj
			if( j == numObjs - 1 )
			{
//				if( fabs( diff ) >= cMERGE_TTC )
//				{
					gap = fabs( diff );
					isGapFound = true;

#ifdef	DEBUG_GAP_SEARCH
					if( debugThisObj )
					{
						gout << endl;
						gout << " there is no follower " << endl;
						gout << " meet ttc ... " << endl;
						gout << " gap is found, follow obj " << objId << endl;
					}
#endif
						
					const CDynObj* pLeadObj = cved->BindObjIdToClass( objId );
					if( !pLeadObj || !pLeadObj->IsValid() )		return	false; // ??

					CPoint3D leadObjPos = pLeadObj->GetPosImm();
					CPoint3D ownerObjPos = pI->m_pObj->GetPosImm();
					double radiusLeader = pLeadObj->GetXSize() * 0.5;

						
#ifdef	DEBUG_GRAPHIC
					if( debugThisObj2 )
					{
						gout << " leadObjPos = " << leadObjPos.m_x << " " << leadObjPos.m_y << " " ;
						gout << leadObjPos.m_z << endl;
						gout << " ownerObjPos = " << ownerObjPos.m_x << " " << ownerObjPos.m_y << " " ;
						gout << ownerObjPos.m_z << endl;
						gout << " radiussLeader = " << radiusLeader << endl;
					}
#endif

					// insert lead approaching obj
					pI->m_mergeInfo.clear();
					TMergeInfo node;
					node.pos = leadObjPos;
					node.radius = radiusLeader;
					node.leadObjId = objId;
					pI->m_mergeInfo.push_back( node );
						

					gapObjId = objId;
					targAccel = -1;
					return true;
			//	}
			}	// if no follower
			else	// If there is a follower behind the approaching obj
			{

#ifdef	DEBUG_GAP_SEARCH
				if( debugThisObj )
				{
					gout << " There is a follower " << endl;
				}
#endif

				// Get the tta of the follower behind the approaching obj
				int followObjId = pI->m_objsApprch[j+1].objId;
				double followObjDist = pI->m_objsApprch[j+1].objDist;
				bool isFollowObjOnRoad = pI->m_objsApprch[j+1].isOnRoad;

				double followObjDistToP1;

				if( !isFollowObjOnRoad )	// If follow obj is on intersection
				{

					followObjDistToP1 = m_distAtP1 - followObjDist;
				}
				else	// If follow obj on target corridor source road
				{
					if( targCrdrSrcLn.GetDirection() == ePOS )
					{
						followObjDistToP1 = targCrdrSrcRd.GetLinearLength() - followObjDist
											+ m_distAtP1;
					}
					else
					{
						followObjDistToP1 = followObjDist + m_distAtP1;
					}
				}	
					
				// Compute follow obj tta to P1
				double ttaFollowObjAtP1;
				ComputeTimeToArrival( pI, followObjId, followObjDistToP1, ttaFollowObjAtP1 );

				double gapBetweenApprchObjs = fabs( ttaObjAtP1 - ttaFollowObjAtP1 );

				double gapBetweenAdoAndFollower = ttaAdoAtP1 - ttaFollowObjAtP1;

				if( gapBetweenAdoAndFollower < 0 )
				{

#ifdef	DEBUG_GAP_SEARCH
					if( debugThisObj )
					{
						gout << " merge ado reaches P1 ahead of follower ... ";
						gout << " check ttc and gap threshold " << endl;
						gout << " merge ado tta  = " << ttaAdoAtP1 << endl;
						gout << " tta of follower of curr apprch obj = " << ttaFollowObjAtP1;
						gout << endl;
					}
#endif
				}
				else
				{

#ifdef	DEBUG_GAP_SEARCH
					if( debugThisObj )
					{
						gout << " merge ado also reaches P1 later than follower ... ";
						gout << " check next apprch obj";
						gout << endl;
						gout << " merge ado tta  = " << ttaAdoAtP1 << endl;
						gout << " tta of follower of curr apprch obj = ";
						gout << ttaFollowObjAtP1 << endl;
					}
#endif

				}



#ifdef	DEBUG_GAP_SEARCH
				if( debugThisObj )
				{
					gout << " gapBetweenApprchObjs = " << gapBetweenApprchObjs << endl;
					gout << " gapBetweenAdoAndFollower = ";
					gout << fabs( gapBetweenAdoAndFollower ) << endl;
					gout << " gap thershold = " << cMERGE_GAP_THRESHOLD << endl;
				}
#endif

				if( gapBetweenAdoAndFollower < 0 && 
					fabs( gapBetweenAdoAndFollower ) >= cMERGE_TTC )
				{
					if( gapBetweenApprchObjs >= cMERGE_GAP_THRESHOLD )
					{
						gap = fabs( gapBetweenAdoAndFollower );
						isGapFound = true;

#ifdef	DEBUG_GAP_SEARCH
						if( debugThisObj )
						{
							gout << endl;
							gout << " meet ttc and gap threshold ... " << endl;
							gout << " gap is found, follow lead obj " << objId << endl;
						}
#endif


						// Show gap in graphic debug
						const CDynObj* pLeadObj = cved->BindObjIdToClass( objId );
						if( !pLeadObj || !pLeadObj->IsValid() )		return	false; // ??

						CPoint3D leadObjPos = pLeadObj->GetPosImm();
						CPoint3D ownerObjPos = pI->m_pObj->GetPosImm();
						double radiusLeader = pLeadObj->GetXSize() * 0.5;

#ifdef	DEBUG_GRAPHIC
						if( debugThisObj2 )
						{
							gout << " leadObjPos = " << leadObjPos.m_x << " " << leadObjPos.m_y << " " ;
							gout << leadObjPos.m_z << endl;
							gout << " ownerObjPos = " << ownerObjPos.m_x << " " << ownerObjPos.m_y << " " ;
							gout << ownerObjPos.m_z << endl;
							gout << " radiussLeader = " << radiusLeader << endl;
						}
#endif

						// insert lead approaching obj
						pI->m_mergeInfo.clear();
						TMergeInfo node;
						node.pos = leadObjPos;
						node.radius = radiusLeader;
						node.leadObjId = objId;
						pI->m_mergeInfo.push_back( node );
							
						
						gapObjId = objId;
						targAccel = -1;
						return true;

					}	// meet gap threshold
				}	// meet ttc threshold

			}	// has follower

		}	// If approaching obj reaches P1 ahead of Ado


	}	// for

	// If a gap is not found, check stop distance. If distance to P1 is less than 
	// stop distance, stop ado at P1.
	if( !isGapFound )
	{

		double currVel = pI->m_currVel;	// m/s
		double stopDist = ( currVel * currVel )/ ( 2 * fabs( cMERGE_MAX_DECEL ) );	// meters
		stopDist = stopDist * cMETER_TO_FEET;

#ifdef	DEBUG_GAP_SEARCH
		if( debugThisObj )
		{
			gout << " no gap is found ... check stop distance " << endl;
			gout << " stopDist = " << stopDist << " ft" << endl;
			gout << " distToP1 = " << distToP1 << " ft" << endl;
			gout << " merge ado tta  = " << ttaAdoAtP1 << endl;
		}
#endif


		if( distToP1 <= stopDist )
		{

		
#ifdef	DEBUG_GAP_SEARCH
			if( debugThisObj )
			{
				gout << " ----------------------------------------" << endl << endl << endl;
				gout << " distToP1 < stopDist .... need to stop at P1" << endl;
				gout << " ******* inputs to stopController " << endl;
				gout << " pI->m_currVel( mph ) = " << pI->m_currVel *cMS_TO_MPH << endl;
				gout << " distToStop ( ft ) = "<< m_distToP1Ado << endl;
				gout << " aggressiveness = " << pI->m_aggressiveness << endl;
				gout << " m_firstFrame = " << m_firstFrame << endl;
			}
		
#endif

			// This is to prevent Ado from accelerating after decelerating to stop
			// and will stop at its stop distance even if it has passes P1.
			if( m_distToP1Ado < 0 )
			{
				m_distToP1Ado = 0.1;
			}

			// Stop Ado at Point P1.
			bool haveTargAccel = StopController( 
								pI->m_currVel, 
								m_distToP1Ado * cFEET_TO_METER, 
								pI->m_aggressiveness, 
								m_firstFrame,
								m_firstDist,
								m_minDist,
								targAccel
								);

#ifdef	DEBUG_GAP_SEARCH
			if( debugThisObj )
			{
				gout << " ******* outputs from stopController " << endl;
				gout << " targAccel = " << targAccel << endl;
				gout << " stopController returned value = " << haveTargAccel << endl;
				gout << " m_firstFrame = " << m_firstFrame * cMETER_TO_FEET << " ft" << endl;
				gout << " m_firstDist = " << m_firstDist * cMETER_TO_FEET << " ft" << endl;
				gout << " m_minDist = " << m_minDist * cMETER_TO_FEET << " ft" << endl;
				gout << " ----------------------------------------" << endl << endl << endl;
			}

		
#endif

			gapObjId = -1;
		}
	}	// when no gap is found
		
	return false;
}	// end of GapSearch





//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function process the Gap Search state.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//	gapObjId - The object identifier.
//	targAccel - The target acceleration.
//
// Returns:	it returns true if a gap is found and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::ProcessGapSearchState( CAdoInfoPtr& pI, int& gapObjId, double& targAccel )
{
	bool hasGap = GapSearch( pI, gapObjId, targAccel );

	if( hasGap )	return true;
	else	return false;


}	// end of ProcessGapSearchState


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function process the Off state.
//
// Remarks:  
//
// Arguments:
//
//	targAccel - The target acceleration.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CMerge::ProcessOffState( CAdoInfoPtr& pI, double& targAccel )
{

	// Velocity control. Pick a velocity that is either the speed limit or the 
	// average velocity of the approaching traffic, whichever is higher.
	double speedLimit = pI->m_roadPos.GetSpeedLimit();

	double total = 0.0;
	double average = -1;
	int numApprchObjs = pI->m_objsApprch.size();

	if( numApprchObjs == 0 )
	{
		targAccel = 999;
		return;
	}

	int numObjsToLook = 0;
	if( numApprchObjs >= cMERGE_MAX_NUM_OBJS_FOR_VEL_CONTROL )
	{
		numObjsToLook = cMERGE_MAX_NUM_OBJS_FOR_VEL_CONTROL;
	}
	else
	{
		numObjsToLook = numApprchObjs;
	}


	for( int i = 0; i < numObjsToLook; i++ )
	{
		int objId = pI->m_objsApprch[i].objId;

		// Get obj vel
		const CDynObj* pObj = cved->BindObjIdToClass( objId );

		// ignore invalid objects
		bool invalidObj = !pObj || !pObj->IsValid();
		if( invalidObj )
		{
			targAccel = cMERGE_INVALID_TARGACCEL;
			return;
		}

		double objVel = pObj->GetVelImm();  // ms
		total += objVel;
		
	}


	if( numApprchObjs >= cMERGE_MAX_NUM_OBJS_FOR_VEL_CONTROL )
	{
		average = total / cMERGE_MAX_NUM_OBJS_FOR_VEL_CONTROL;
	}
	else
	{
		average = total / numApprchObjs;
	}
	
	double targVel;

	if( average > speedLimit * cMPH_TO_MS )
	{
		targVel = average;
	}
	else
	{
		targVel = speedLimit * cMPH_TO_MS;
	}


#ifdef DEBUG_PROCESS_OFF_STATE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_PROCESS_OFF_STATE;
	if( debugThisObj )
	{
		gout << endl;
		gout << "==========CMerge::ProcessOffState============" << endl;
		gout << " get new targAccel for vel control in OFF state " << endl;
		gout << " speed limit = " << speedLimit * cMPH_TO_MS << " ms " << endl;
		gout << " speed limit = " << speedLimit << " mph " << endl << endl;
		gout << " average vel = " << average << " ms " << endl;
		gout << " average vel = " << average * cMS_TO_MPH << " mph " << endl << endl;
		gout << " targVel = " << targVel << endl << endl;
		gout << " curr vel = " << pI->m_currVel << " ms " << endl;
		gout << " curr vel = " << pI->m_currVel * cMS_TO_MPH << " mph " << endl << endl;
		gout << " prev vel = " << pI->m_prevVel << " ms " << endl;
		gout << " curr vel = " << pI->m_prevVel * cMS_TO_MPH << " mph " << endl << endl;
		gout << " numObjsToLook = " << numObjsToLook << endl;

	}
#endif
	targAccel = CalcTargAccelForNormalCruising(
							pI->m_currVel,
							pI->m_prevVel,
							targVel,
							pI->m_aggressiveness,
							m_pRootCollection->m_rng,
							pI->m_rngStreamId
							);

#ifdef DEBUG_PROCESS_OFF_STATE
	if( debugThisObj )
	{
		gout << " targAccel returned from call = " << targAccel << endl << endl;
	}
#endif
	

}	// end of ProcessOffState

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function checks if a road is a merge road.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::CheckMergeRoad( CAdoInfoPtr& pI )
{

	const int MERGE_ONRAMP_ATTR = 41;

	// Check if the Ado is on a merge road. If the Ado is on road, check 
	// the current road; if on intersection, check the corridor source road.
	if( pI->m_roadPos.IsValid() )
	{
		CRoad road;
		if( pI->m_roadPos.IsRoad() )
		{
			road = pI->m_roadPos.GetRoad();
		}
		else
		{
			road = pI->m_roadPos.GetCorridor().GetSrcLn().GetRoad();
		}

		// check attributes
		vector<CAttr> attrs;
		road.QryAttr( attrs );

		vector<CAttr>::iterator i;
		for ( i = attrs.begin(); i != attrs.end(); i++ )
		{
			CAttr attr = *i;
			if( attr.GetId() == MERGE_ONRAMP_ATTR )
			{
				// Yes, on a merge road
				m_isOnMergeRoad = true;
				return true;
			}
		}
	}
	m_isOnMergeRoad = false;					
	return false;
}	// end of CheckMergeRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function checks the conditions from the Off state
//	to Gap Search state. The conditions are:  current road is a merge road and
//	the ado is only cMERGE_TIME_FOR_GAP_SEARCH minutes away from point p1.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//
// Returns:	It returns true if the conditions are met and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::TransitionOffToGapSearch( CAdoInfoPtr& pI )
{

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_TRANSITION_OFF_TO_GAP_SEARCH;
#endif

	bool isOnMergeRoad = CheckMergeRoad( pI );

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << endl;
		gout << " ===========CMerge::TransitionOffToGapSearch ==========" << endl;
		gout << " isOnMergeRoad = " << isOnMergeRoad << endl;
	}
#endif

	// Compute tta to p1.
	CCrdr targCrdr, mergeCrdr;
	CIntrsctn mergeIntrsctn;

	bool isOnRoad = pI->m_roadPos.IsRoad();

	if( isOnRoad )
	{
		mergeCrdr = pI->m_pPath->GetNextCrdr( pI->m_roadPos );
	}
	else
	{
		mergeCrdr = pI->m_roadPos.GetCorridor();
	}

	if( !mergeCrdr.IsValid() )
	{

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " mergeCrdr invalid ... return false " << endl;
	}
#endif
		return false;
	}

	mergeIntrsctn = mergeCrdr.GetIntrsctn();

	if( !mergeIntrsctn.IsValid() )
	{

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " mergeIntrsctn invalid ... return false " << endl;
	}
#endif
		return false;
	}

	vector<CCrdr> allCrdrs;
	mergeIntrsctn.GetAllCrdrs( allCrdrs );	
	CLane dtntnLnMergeCrdr = mergeCrdr.GetDstntnLn();

	if( !dtntnLnMergeCrdr.IsValid() )	return false;

	vector<CCrdr>::iterator i;
	for( i = allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		CCrdr aCrdr = *i;
		if( aCrdr.GetDstntnLn() == dtntnLnMergeCrdr &&
			aCrdr.GetRelativeId() != mergeCrdr.GetRelativeId() 
		)
		{
			targCrdr = aCrdr;
			break;
		}
	}

	if( !targCrdr.IsValid() )
	{

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " targCrdr invalid ... return false " << endl;
	}
#endif
		return false;
	}

	m_targCrdr = targCrdr;

	int mergeCrdrId = mergeCrdr.GetRelativeId();
	int targCrdrId = targCrdr.GetRelativeId();

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " mergeIntrsctn = " << mergeIntrsctn.GetName() << endl;
		gout << " mergeCrdr = " << mergeCrdrId << endl;
		gout << " targCrdr = " << targCrdrId << endl;
	}
#endif
	

	// Get the transition point which is the first intersecting point of the
	// merge corridor and target corridor.
	double mergeDist = mergeCrdr.GetFirstMrgDist( targCrdrId );

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " ***dist at transition point P for crdrs " << mergeCrdrId << " and ";
		gout << targCrdrId << " = " << mergeDist << endl;
	}
#endif


	// Get the dist at point P1 after the transition point.
	const	double cDIST_P_TO_P1 = 20; // ft
	double distAtP1 = mergeDist + cDIST_P_TO_P1;
	m_distAtP1 = distAtP1;


	double distToP1;
	double currDist = pI->m_roadPos.GetDistance();

	if( isOnRoad )
	{
		CRoad mergeRd = pI->m_roadPos.GetLane().GetRoad();
		if( !mergeRd.IsValid () )
		{

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
			if( debugThisObj )
			{
				gout << " merge road invalid ... return false " << endl;
			}
#endif
			return false;
		}

		double mergeRdLength = mergeRd.GetLinearLength();
		
		distToP1 = mergeRdLength - currDist + distAtP1;
	}
	else
	{
		distToP1 = distAtP1 - currDist;
	}

	m_distToP1Ado = distToP1;

	// Compute time to arrival to P1 
	double tta;

#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " inputs to compute tta " << endl;
		gout << " curr obj id = " << pI->m_pObj->GetId() << endl;
		gout << " dist to p1 = " << distToP1 << endl;
	}
#endif

	ComputeTimeToArrival( pI, pI->m_pObj->GetId(), distToP1, tta );


#ifdef DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
	if( debugThisObj )
	{
		gout << " tta before GS = " << tta << endl;
		gout << " cMERGE_TIME_FOR_GAP_SEARCH = " << cMERGE_TIME_FOR_GAP_SEARCH << endl;
		gout << " isOnMergeRoad = " << isOnMergeRoad << endl;
	}
#endif


	if( isOnMergeRoad && tta > 0 && tta <= cMERGE_TIME_FOR_GAP_SEARCH )
	{

#ifdef	DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
		if( debugThisObj )
		{

			gout << endl;
			gout << " Ado " << pI->m_pObj->GetId() << " from Off to GapSearch " << endl;
		
		}
#endif
		return true;
	}
	else
	{

#ifdef	DEBUG_TRANSITION_OFF_TO_GAP_SEARCH
		if( debugThisObj )
		{

			gout << " Ado " <<pI->m_pObj->GetId() << " :  not on a merge road or";
			gout << " too soon for Gap Search, stay in Off state ";
			gout << endl << endl << endl;
		}
#endif		
		return false;
		

	}

}	// end of TransitionOffToGapSearch

///////////////////////////////////////////////////////////////////////////////
///\brief
///This function process the Track state.
///
/// Remarks:  
///
/// Arguments:
///
///\param[in]	pI - The Ado info pointer.
///\param[in]	actualDist - The following distance ( in ft ).
///\param[out]	targAccel - The target acceleration returned.
///
///\return	It returns true if the track is successful and false otherwise.
///\todo  This functions needs to track if we still have a chance at making the
///       requested merge, the target veh might have slowed down
///////////////////////////////////////////////////////////////////////////////

bool 
CMerge::ProcessTrackState( CAdoInfoPtr& pI, double actualDist, double& targAccel )
{

#ifdef	DEBUG_PROCESS_TRACK_STATE
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_PROCESS_TRACK_STATE;
	if( debugThisObj )
	{
		gout << endl;
		gout << " ===========CMerge::ProcessTrackState========== " << endl;
	}
#endif

	// Ado is leader when there is no approaching obj.
	if( pI->m_mergeInfo.size() == 0 )
	{

#ifdef	DEBUG_PROCESS_TRACK_STATE
		if( debugThisObj )
		{
			gout << " no approaching obj before p1... set targAccel " << endl;
		}
#endif
		targAccel = 1;
		return true;
	}

	int leadObjId = (pI->m_mergeInfo)[0].leadObjId;

	// When Ado is in front of all approaching objs.
	if ( leadObjId == -1 )
	{

#ifdef	DEBUG_PROCESS_TRACK_STATE
		if( debugThisObj )
		{
			gout << " merge ado in front of all approaching objs ... set targAccel " << endl;
		}
#endif
		targAccel = 1;
		return true;
	}	
	else	// If Ado needs to follow an approaching obj.
	{

		// Show graphic bug.
		const CDynObj* pLeadObj = cved->BindObjIdToClass( leadObjId );
        
		if( !pLeadObj || !pLeadObj->IsValid() )		return	false; // ??
        //if we are ahead of our target make sure we can soon enough
        if(actualDist < 0 &&!pI->m_roadPos.IsRoad()){
            auto mergeCrdr = pI->m_roadPos.GetCorridor();
	 

	        //int mergeCrdrId = mergeCrdr.GetRelativeId();
	        //int targCrdrId = m_targCrdr.GetRelativeId();

	        //// Get the transition point which is the first intersecting point of the
	        //// merge corridor and target corridor.
         //   double mergeDist = mergeCrdr.GetFirstMrgDist( targCrdrId );
         //   double myCurDist = pI->m_roadPos.GetDistance(mergeCrdrId);
         //   double currSpeed = pI->m_currVel;
         //   double stopDist = (currSpeed * currSpeed)/(2*5);
         //   stopDist *=  cMetersToFeet;
         //   // we cannot stop fast enough to hit our target gap
         //   if ( mergeDist < 0 &&   actualDist < 0){
         //       m_isMergeLaneChangeDone = true;
         //       return false; 
         //   }

        }
		CPoint3D leadObjPos = pLeadObj->GetPosImm();
		CPoint3D ownerObjPos = pI->m_pObj->GetPosImm();
		double radiusLeader = pLeadObj->GetXSize() * 0.5;

						
//#ifdef	DEBUG_GRAPHIC
//		if( debugThisObj )
//		{
//			gout << " leadObjPos = " << leadObjPos.m_x << " " << leadObjPos.m_y << " " ;
//			gout << leadObjPos.m_z << endl;
//			gout << " ownerObjPos = " << ownerObjPos.m_x << " " << ownerObjPos.m_y << " " ;
//			gout << ownerObjPos.m_z << endl;
//			gout << " radiussLeader = " << radiusLeader << endl;
//		}
//#endif

		pI->m_mergeInfo.clear();
		TMergeInfo node;
		node.pos = leadObjPos;
		node.radius = radiusLeader;
		node.leadObjId = leadObjId;
		pI->m_mergeInfo.push_back( node );


		double leadVel = pLeadObj->GetVelImm();	// in m/s
		double leadAccel = GetAccel( pLeadObj );
	

		if ( m_lastLeadId == leadObjId ) {
			m_currFollEngTime++;
		}
		else {
			m_currFollEngTime = 0;
			m_initFollDist    = actualDist;
		}
		m_lastLeadId = leadObjId;

		

		CFollowInfo fi;
		m_mergeParams                       = pI->m_FollowParams;
		m_mergeParams.useReactDelay         = false;
		m_mergeParams.folTimeMode           = false;
		m_mergeParams.folValue              = 20 * cFEET_TO_METER;
		m_mergeParams.normal.distKp         = 1.0;
		m_mergeParams.normal.distKd			= 0.0;
		m_mergeParams.normal.velKp          = 2.0;
		m_mergeParams.normal.accelToCatchUp = true;
		m_mergeParams.normal.posAccClip     = 4.0;
		m_mergeParams.normal.negAccClip     = -15;	// has impact on slow down



#ifdef	DEBUG_PROCESS_TRACK_STATE
		if( debugThisObj )
		{
			gout << endl;
			gout << " FollowController inputs: " << endl << endl;
			gout << "	curr obj id = " << pI->m_pObj->GetId() << endl;
			gout << "	lead obj id = " << leadObjId << endl;
			gout << "	actualDist passed in = " << actualDist << " ft " << endl;
			gout << "	prevFollowAcutalDist = " << m_prevFollowActualDist << " ft " << endl;
			gout << "	current follow time = " << m_currFollEngTime << endl;
			gout << "	initFollDist = " << m_initFollDist << " ft " << endl;
			gout << "	lead obj vel = " << leadVel * cMS_TO_MPH << " mph " << endl;
			gout << "	ado vel = " << pI->m_currVel * cMS_TO_MPH << " mph " << endl;
			gout << "	ado prev vel = " << pI->m_prevVel * cMS_TO_MPH << " mph " << endl;
			gout << "	agressiveness = " << pI->m_aggressiveness << endl;
			gout << "	frame # = " << GetFrame() << endl << endl;
		}
#endif


		bool gotFollow = FollowController( 
										GetFrame(),
										GetTimeStepDuration(),
										leadObjId, 
										fi,
										m_mergeParams,
										actualDist * cFEET_TO_METER,
										m_prevFollowActualDist * cFEET_TO_METER,
										m_currFollEngTime,
										m_initFollDist * cFEET_TO_METER ,
										leadVel,
										pI->m_currVel,
										pI->m_prevVel,
										leadAccel,
										pI->m_aggressiveness,
										85 * cMPH_TO_MS,	
										targAccel,
										m_mergeTtc
										);

#ifdef	DEBUG_PROCESS_TRACK_STATE
		if( debugThisObj )
		{
			gout << " FollowController outputs: " << endl << endl;
			gout << "	targAccel returned = " << targAccel << endl;
			gout << "	followTTc returned = " << pI->m_followTtc << endl;
			gout << "	gotFollow = " << gotFollow << endl << endl;
		}
		
#endif

		
	
		m_prevFollowActualDist = actualDist;

		if( !gotFollow ) 
		{
			return false;
		}
		else
		{

			// No lane change for now.
#if 0
			// If passed P1, do a merge lane change.
			if( m_distToP1Ado < 0 )
			{
				pI->m_mergeLaneChangeIsNeeded = true;
				pI->m_mergeTargCrdr = m_targCrdr;
			}
#endif

			return true;
		}



	}	// Ado follows an approaching obj
	
	return false;

}	// end of ProcessTrackState



//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function checks the condition from Gap Search state to
//	Track state. Condition: a gap is found.
//
// Remarks:  
//
// Arguments:
//
//	hasGap - Whether there is a gap.
//
// Returns:	It returns true if the condition is met and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::TransitionGapSearchToTrack( bool hasGap )
{
	if( hasGap )
	{

#ifdef	DEBUG_TRANSITION_GAP_SEARCH_TO_TRACK
		gout << endl << " From GapSearch to Track " << endl;
#endif

		return true;
	}
	else	return false;


}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function checks the condition from Gap Search state to
//	Off state. Condition: when current road is not a merge road any more.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//
// Returns:	It returns true if the condition is met and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::TransitionGapSearchToOff( CAdoInfoPtr& pI )
{
	bool isOnMergeRoad = CheckMergeRoad( pI );
	if( !isOnMergeRoad || m_isMergeLaneChangeDone )
	{

#ifdef	DEBUG_TRANSITION_GAP_SEARCH_TO_OFF
		gout << " from GapSearch to Off ... because not on a merge road any more " << endl;
#endif			

		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function computes the distances to point p1 for the objs 
//	that are on the target lane, target corridor or its destination road.
//
// Remarks:  
//
// Arguments:
//
//	pI - ado info pointer.
//	gapObjId - the identifier of the object.
//	distToP1 - distance to point p1 ( in ft ).
//
// Returns:	It returns true if the distance is successfully calculated and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::ComputeApprchObjDistToP1( CAdoInfoPtr& pI, int gapObjId, double& distToP1 )
{

	int numObjs = pI->m_objsApprch.size();

#ifdef	DEBUG_COMPUTE_APPRCH_OBJDIST_TOP1
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_COMPUTE_APPRCH_OBJDIST_TOP1;
	if( debugThisObj )
	{
		gout << endl;
		gout << " ==========CMerge::ComputeApprchObjDistToP1==========" << endl;
		gout << " Ado " << pI->m_pObj->GetId() << " : " << " number of approaching objs = ";
		gout << numObjs << endl;
	}
#endif

	if( gapObjId == -1 )
	{
		distToP1 = 0.0;
		return false;
	}
		
	
	// For each approaching obj when it is on source road or intersection
	bool onSrcRdOrIntrsctn = false;

	for ( int i = 0; i < numObjs; i++ )
	{
		int objId = pI->m_objsApprch[i].objId;
		double objDist = pI->m_objsApprch[i].objDist;
		bool isObjOnRoad = pI->m_objsApprch[i].isOnRoad;

#ifdef	DEBUG_COMPUTE_APPRCH_OBJDIST_TOP1
		if( debugThisObj )
		{
			gout << endl;
			gout << " obj id = " << objId << endl;
			gout << " gapObjId passed in = " << gapObjId << endl;
		//	gout << " obj dist = " << objDist << endl;
		//	gout << " obj on road = " << isObjOnRoad << endl;	
		//	gout << " m_distAtP1 = " << m_distAtP1 << endl;
		}
#endif

		if( objId == gapObjId )
		{

			onSrcRdOrIntrsctn = true;

#ifdef	DEBUG_COMPUTE_APPRCH_OBJDIST_TOP1
			if( debugThisObj )
			{
				gout << " found gap obj id " << objId << endl;
				gout << " onSrcRdOrIntrsctn = " << onSrcRdOrIntrsctn << endl;
			}
#endif


			if( !isObjOnRoad )	// If obj is on intersection
			{
				distToP1 = m_distAtP1 - objDist;
			}
			else	
			{
				// Obj is on target corridor source road.
				CLane targCrdrSrcLn = m_targCrdr.GetSrcLn();
				CRoad targCrdrSrcRd = targCrdrSrcLn.GetRoad();

				if( targCrdrSrcLn.GetDirection() == ePOS )
				{
					distToP1 = targCrdrSrcRd.GetLinearLength() - objDist + m_distAtP1;
				}
				else
				{
					distToP1 = objDist + m_distAtP1;
				}
			}
			return true;
		}
	} // for

	
	if( !onSrcRdOrIntrsctn )	// Obj is on dstntn road
	{
			// The followed obj is on destntn road
		int numFwdObjs = pI->m_objsFwd.size();
		double followDist = -1;

		for( int j = 0; j < numFwdObjs; j++ )
		{
			if( pI->m_objsFwd[j].objId == gapObjId )
			{
				followDist = pI->m_objsFwd[j].distFromOwner;
				break;
			}
		}

		double currDist = pI->m_roadPos.GetDistance();

		if( !pI->m_roadPos.IsRoad ())		// when curr obj on intersection
		{
			if( currDist > m_distAtP1 )		// passed p1
			{
				distToP1 = followDist + currDist - m_distAtP1;
				distToP1 = - distToP1;
			}
			else	// before p1
			{
				distToP1 = followDist - ( m_distAtP1 - currDist );
				distToP1 = - distToP1;
			}
		}
		else		// when curr obj on merge road
		{
			// when curr obj on dstntn road
			CLane dstntnLn = m_targCrdr.GetDstntnLn();
			CRoad dstntnRd = dstntnLn.GetRoad();
			double dstntnRdLength = dstntnRd.GetLinearLength();
			CRoad currRoad = pI->m_roadPos.GetLane().GetRoad();

			bool onSameRd = currRoad.GetName() == dstntnRd.GetName();
			if ( onSameRd )
			{
				double distOnTargCrdr = m_targCrdr.GetLength() - m_distAtP1;

				if( dstntnLn.GetDirection() == ePOS )
				{
					distToP1 = distOnTargCrdr + currDist + followDist;
					distToP1 = - distToP1;
				}
				else
				{
					distToP1 = distOnTargCrdr + dstntnRdLength - currDist + followDist;
					distToP1 = - distToP1;
				}
			}
			else	// when curr obj on merge road
			{
				CLane mergeLn = pI->m_roadPos.GetLane();
				CRoad mergeRd = mergeLn.GetRoad();


				if( mergeLn.GetDirection() == ePOS )
				{
					distToP1 = mergeRd.GetLinearLength() - currDist + m_distAtP1;
					distToP1 = - distToP1;
				}
				else
				{
					distToP1 = currDist + m_distAtP1;
					distToP1 = - distToP1;
				}
			}
		}


#ifdef	DEBUG_COMPUTE_APPRCH_OBJDIST_TOP1
		if( debugThisObj )
		{
			gout << gapObjId << " on dstntn road " << endl;
			gout << " dist to p1 = " << distToP1 << endl;
		}
#endif
		return true;
		
	}	// lead obj on dstntn road
				
	return false;	// When gap obj is not found
}	// end of ComputeApprchObjDistToP1


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function computes the follow distance between the merge
//	ado and the lead obj.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//  apprchObjDistToP1 - The distance to point p1 of the approaching obj.( in ft )
//	actualDist - The following distance between the current obj and lead obj ( in ft ).
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CMerge::ComputeActualDist( CAdoInfoPtr& pI, double apprchObjDistToP1, double& actualDist )
{
#ifdef	DEBUG_COMPUTE_ACTUAL_DIST
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_COMPUTE_ACTUAL_DIST;
	if( debugThisObj )
	{
		gout << endl;
		gout << " =======CMerge::ComputeActualDist=========" << endl;
	}
#endif

	double distToP1;

	double currDist = pI->m_roadPos.GetDistance();

	bool isOnRoad = pI->m_roadPos.IsRoad();

	if( isOnRoad )
	{
		CLane currLn = pI->m_roadPos.GetLane();
		CRoad currRd = currLn.GetRoad();
		double currRdLength = currRd.GetLinearLength();

		CLane dstntnLn = m_targCrdr.GetDstntnLn();
		double targCrdrLength = m_targCrdr.GetLength();

		bool isOnDstntnRd = currRd.GetName() == dstntnLn.GetRoad().GetName();
		if( isOnDstntnRd )	// If on dstntn rd.
		{

#ifdef	DEBUG_COMPUTE_ACTUAL_DIST	
			if( debugThisObj )
			{
				gout << " targCrdrLength = " << targCrdrLength << endl;
				gout << " m_distAtP1 = " << m_distAtP1 << endl;
				gout << " currDist = " << currDist << endl;
				gout << " currRdLength = " << currRdLength << endl;
			}
#endif

			if( dstntnLn.GetDirection() == ePOS )
			{
				distToP1 = targCrdrLength - m_distAtP1 + currDist;
				distToP1 = - distToP1;
			}
			else
			{
				distToP1 = targCrdrLength - m_distAtP1 + currRdLength - currDist;
				distToP1 = - distToP1;
			}
		}
		else	// Assuming on the source road.
		{
			if( currLn.GetDirection() == ePOS )
			{
				distToP1 = currRdLength - currDist + m_distAtP1;
			}
			else
			{
				distToP1 = currDist + m_distAtP1;
			}
		}
		
	}
	else
	{
		distToP1 = m_distAtP1 - currDist;
	}

	m_distToP1Ado = distToP1;


	if( apprchObjDistToP1 > 0 && distToP1 > 0 )
	{
		double diff = apprchObjDistToP1 - distToP1;

		if( diff > 0 )
		{
			actualDist = - diff;
		}
		else
		{
			actualDist = distToP1 - apprchObjDistToP1;
		}
	}
	else if( apprchObjDistToP1 < 0 && distToP1 < 0 )
	{
		if( apprchObjDistToP1 < distToP1 )
		{
			actualDist = fabs( apprchObjDistToP1 ) - fabs( distToP1 );
		}
		else
		{
			actualDist = fabs( distToP1 ) - fabs( apprchObjDistToP1 );
			actualDist = - actualDist;
		}
	}
	else if( distToP1 > 0 && apprchObjDistToP1 < 0 )
	{
		actualDist = distToP1 + fabs( apprchObjDistToP1 );
	}
	else
	{
		actualDist = fabs( distToP1 ) + apprchObjDistToP1;
		actualDist = - actualDist;
	}


#ifdef	DEBUG_COMPUTE_ACTUAL_DIST
	if( debugThisObj )
	{
		gout << " Compute actual dist " << endl << endl;
		gout << "	merge ado dist to P1 = " << distToP1 << endl;
		gout << "	apprch obj dist to P1 = " << apprchObjDistToP1 << endl;
		gout << "	actual dist = " << actualDist << endl << endl;
	}
#endif


}	// end of ComputeActualDist


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function checks the condition from Track state to Gap Search
//	state.	Condition: when the previous Track is unsuccessful.
//
// Remarks:  
//
// Arguments:
// 
//	hasTrack - Whether track is successful.
//
// Returns:	It returns true if the condition is met and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::TransitionTrackToGapSearch( bool hasTrack )
{
	if( !hasTrack )		return true;
	else	return false;

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function checks the condition from Track state to Off
//	state.	Condition: when Track is successful and when the current road is
//	not a merge road any more.
//
// Remarks:  
//
// Arguments:
//
//	pI - The ado info pointer.
//
// Returns:	It returns true if the condition is met and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CMerge::TransitionTrackToOff( CAdoInfoPtr& pI )
{
	bool isOnMergeRd = CheckMergeRoad( pI );

#ifdef DEBUG_TRANSITION_TRACK_TO_OFF
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_TRANSITION_TRACK_TO_OFF;
	if( debugThisObj )
	{
		gout << " isOnMergeRd = " << isOnMergeRd << endl << endl;
		if( !isOnMergeRd )
		{
			gout << " Go from Track to Off because not on merge road any more. " << endl;
		}
	}
#endif

	if( m_isMergeLaneChangeDone  || !isOnMergeRd )	return true;
	else	return false;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function handles the states and transitions.
//
// Remarks:  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CMerge::UserPostActivity()
{

#ifdef DEBUG_USER_POST_ACTIVITY
	gout << endl;
	gout << "=================CMerge::UserPostActivity==========" << endl;
#endif

	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	bool haveAccel = false;
	double targAccel;

	if( m_state == eMERGE_OFF )
	{
		// Process Off state
		ProcessOffState( pI, targAccel );
		if( targAccel == cMERGE_INVALID_TARGACCEL )
		{
			haveAccel = false;
		}
		else
		{
			haveAccel = true;
		}
		// Check transitions to Gap Search state.
		bool hasTransition = TransitionOffToGapSearch( pI );
		if( hasTransition )
		{
			m_state = eMERGE_GAP_SEARCH;
		}
		else
		{
			SetOutputTargAccelNoValue();
			return;
		}

	}
	else if( m_state == eMERGE_GAP_SEARCH )
	{
		// Process Gap Search state
		bool hasGap = ProcessGapSearchState( pI, m_gapObjId, targAccel );

#ifdef	DEBUG_USER_POST_ACTIVITY
		gout << " hasGap = " << hasGap << endl;
		gout << " m_gapObjId = " << m_gapObjId << endl;
#endif

		if( hasGap )
		{
			haveAccel = false;
		}
		else if( !hasGap && targAccel == cMERGE_INVALID_TARGACCEL )
		{
			haveAccel = false;
			SetOutputTargAccelNoValue();
			return;
		}
		else
		{
			haveAccel = true;
		}

		// Check transitions to Track or Off state.
		bool hasTransition = TransitionGapSearchToOff( pI );
		bool hasTransition2 = TransitionGapSearchToTrack( hasGap );

		if( hasTransition )
		{
			m_state = eMERGE_OFF;
			SetOutputTargAccelNoValue();
			return;
		}
		if( hasTransition2 )
		{
			haveAccel = false;
			m_state = eMERGE_TRACK;
				
		}

	}
	else
	{

		// Process Track state
		double actualDist = 0.0;
		double gapObjDistToP1;
		bool hasDist = ComputeApprchObjDistToP1( pI, m_gapObjId, gapObjDistToP1 );

		if( hasDist )
		{
			ComputeActualDist( pI, gapObjDistToP1, actualDist );
		}

		bool hasTrack = ProcessTrackState( pI, actualDist, targAccel );

		// Check transtions to Gap Search or Off state.
		bool hasTransition1 = TransitionTrackToGapSearch( hasTrack );
		if( hasTransition1 )
		{
			haveAccel = false;
			m_state = eMERGE_GAP_SEARCH;
		}
		else
		{
			haveAccel = true;
		}
		
		bool hasTransition2 = TransitionTrackToOff( pI );
		if( hasTransition2 )
		{
			haveAccel = false;
			m_state = eMERGE_OFF;
		}

	}


#ifdef	DEBUG_USER_POST_ACTIVITY
	gout << "haveAccel = " << haveAccel << "  targAccel = " << targAccel << endl;
#endif

	if( haveAccel )
	{
		SetOutputTargAccel( targAccel );
	}
	else
	{
		SetOutputTargAccelNoValue();
	}


}  // end of UserPostActivity
