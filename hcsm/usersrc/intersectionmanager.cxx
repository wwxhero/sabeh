/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: intersectionmanager.cxx,v 1.38 2016/10/28 20:56:06 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    November, 1999
 *
 * Description:  Contains code for the intersection manager object
 *               (IntersectionManager HCSM).
 *
 ****************************************************************************/

#include "genericinclude.h"
#include "genhcsm.h"
#include "hcsmcollection.h"

#include <pi_iostream>
using namespace std;

//
// Debugging macros.
//
#undef DEBUG_PREACTIVITY
#undef DEBUG_REMOVEINACTIVE
#undef DEBUG_PROCESSREGISTER
#undef DEBUG_PRIORITIZE_VEHICLES
#undef DEBUG_SIGNAL_VEHICLES 


void 
CIntersectionManager::UserCreation( 
			const CIntersectionMngrParseBlock* cpSnoBlock
			)
{
	PrintCreationMessage();

	int numIntrsctns = cved->GetNumIntersections();
	bool notEnoughRoom = numIntrsctns > cMAX_IM_INTRSCTN;
	if( notEnoughRoom )
	{
		cerr << MessagePrefix();
		cerr << "IM: number of intersections, " << numIntrsctns;
		cerr << ", exceed internal storage limitations." << endl;
		cerr << "Please increase size of cMAX_IM_INTRSCTN and ";
		cerr << "recompile this software.  [SUICIDE]" << endl;

		Suicide();
		return;
	}

	//
	// Initialize private members.
	//
	int i;
	for( i = 0; i < numIntrsctns; i++ )
	{
		m_intrsctns[i].active = false;
		m_intrsctns[i].activeImIdx = -1;

		//
		// Disable intersetions that don't need an intersection manager.
		//
		CIntrsctn intrsctn( *cved, i );
		if( intrsctn.IsValid() )
		{
			vector<CRoad> roadsAtIntrsctn;
			intrsctn.GetRoads( roadsAtIntrsctn );
            m_intrsctns[i].disabled = roadsAtIntrsctn.size() < 3 && !intrsctn.IsControlled();
		}
		else
		{
			m_intrsctns[i].disabled = true;
		}
	}

	//
	// Insert the free elements of m_activeIntrsctns into the queue.
	//
	for( i = 0; i < cMAX_ACTIVE_IM_INTRSCTN; i++ )
	{
		m_freeActiveIntrsctnsIdx.push( i );
	}

	m_activeIntrsctnsLastElem = 0;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Removes intersetions from the active intersection list 
//   that are no longer active.
//	
// Remarks: 
//
// Arguments:
//	
// Returns:	
//
//////////////////////////////////////////////////////////////////////////////
void
CIntersectionManager::RemoveInactiveIntrsctns()
{
#ifdef DEBUG_REMOVEINACTIVE
	gout << MessagePrefix();
	gout << "=== RemoveInactiveIntrsctns ===========================" << endl;
	gout << "  m_activeIntrsctnsLastElem = " << m_activeIntrsctnsLastElem;
	gout << endl;
#endif

	int i;
	int lastActiveElem = 0;
	for( i = 0; i <= m_activeIntrsctnsLastElem; i++ )
	{
		bool elemInUse = m_activeIntrsctns[i].m_intrsctnIdx >= 0;
		if( !elemInUse )  continue;

#ifdef DEBUG_REMOVEINACTIVE
		CIntrsctn intrsctn( *cved, m_activeIntrsctns[i].m_intrsctnIdx );
		gout << "  checking to remove intersection ";
		gout << m_activeIntrsctns[i].m_intrsctnIdx;
		gout << "   " << intrsctn.GetName() << endl;
		gout << "    remove obj list size = ";
		gout << m_activeIntrsctns[i].m_removeObjs.size() << endl;
#endif

		//
		// Remove all the object's that are in the vehicle removal
		// list which was set by the PrioritizeVehicles function.
		//
		set<int>::iterator j;
		for( 
			j = m_activeIntrsctns[i].m_removeObjs.begin(); 
			j != m_activeIntrsctns[i].m_removeObjs.end();
			j++
			)
		{
			//
			// This is the object that has to be removed.
			//
			int removeObjId = *j;

#ifdef DEBUG_REMOVEINACTIVE
			gout << "    removing object " << removeObjId << endl;
#endif

			SetVehicleState( removeObjId, eGO, -1.0 );

			//
			// Locate and remove this object from the main object set.
			//
			set<int>::iterator k;
			for( 
				k = m_activeIntrsctns[i].m_objs.begin();
				k != m_activeIntrsctns[i].m_objs.end();
				k++
				)
			{
				if( *k == removeObjId )
				{
#ifdef DEBUG_REMOVEINACTIVE
					gout << "    found object in objs set...removing it";
					gout << endl;
#endif
					m_activeIntrsctns[i].m_objs.erase( k );
//					if( m_activeIntrsctns[i].m_objs.size() == 0 )
//					{
//						// no more objects at this intersection, make it inactive
//						m_activeIntrsctns[i].m_intrsctnIdx = -1;
//					}
					break;
				}
			}

			//
			// Locate and remove this object from the map.
			//
			map<int, TImActiveObjInfo>::iterator m = m_activeIntrsctns[i].m_objInfo.find( removeObjId );
			bool foundObjInMap = m != m_activeIntrsctns[i].m_objInfo.end();
			if( foundObjInMap ) 
			{
				m_activeIntrsctns[i].m_objInfo.erase( m );
#ifdef DEBUG_REMOVEINACTIVE
				gout << "    found object in map...removing it";
				gout << endl;
#endif
			}
		}

		m_activeIntrsctns[i].m_removeObjs.clear();
		
		bool freeElem = elemInUse && m_activeIntrsctns[i].m_objs.size() <= 0;

		if( freeElem )
		{
#ifdef DEBUG_REMOVEINACTIVE
			gout << "    freeing intersection" << endl;
#endif

			int intrsctnIdx = m_activeIntrsctns[i].m_intrsctnIdx;
			m_activeIntrsctns[i].m_intrsctnIdx = -1;
			m_freeActiveIntrsctnsIdx.push( i );
			m_intrsctns[intrsctnIdx].active = false;
			m_intrsctns[intrsctnIdx].activeImIdx = -1;
		}

		if( elemInUse && !freeElem )  lastActiveElem = i;
	}

	if( lastActiveElem < m_activeIntrsctnsLastElem )
	{
		m_activeIntrsctnsLastElem = lastActiveElem;
	}
}  // end of RemoveInactiveIntrsctns


//////////////////////////////////////////////////////////////////////////////
//
// Description: Process requests to register vehicles with intersection
//   managers.
//	
// Remarks: 
//
// Arguments:
//	
// Returns:	
//
//////////////////////////////////////////////////////////////////////////////
void
CIntersectionManager::ProcessRegisterRequests()
{
#ifdef DEBUG_PROCESSREGISTER
	gout << MessagePrefix();
	gout << "=== ProcessRegisterRequests ===========================" << endl;
#endif

	TImRegisterData elem;
	while( m_pRootCollection->GetImRegisterElem( elem ) )
	{
		int intrsctnToRegister = elem.intrsctnId;
		int objToRegister = elem.objId;

		if( m_intrsctns[intrsctnToRegister].disabled )
		{
#ifdef DEBUG_PROCESSREGISTER
			gout << "skipping obj " << objToRegister;
			gout << " on disabled intersection " << intrsctnToRegister << endl;
#endif
			return;
		}
		
		int activeIntrsctnIdx;
		bool intrsctnAlreadyRegistered = m_intrsctns[intrsctnToRegister].active;
		if( intrsctnAlreadyRegistered )
		{
			//
			// No need to register the intersection, just register the 
			// object.
			//
#ifdef DEBUG_PROCESSREGISTER
			gout << "registering obj " << objToRegister;
			gout << " on intersection " << intrsctnToRegister << endl;
#endif

			activeIntrsctnIdx = m_intrsctns[intrsctnToRegister].activeImIdx;
			m_activeIntrsctns[activeIntrsctnIdx].m_objs.insert( objToRegister );

		}
		else
		{
			//
			// Register the intersection and the object.
			//
			bool freeElementsAvail = m_freeActiveIntrsctnsIdx.size() > 0;
			if( !freeElementsAvail )
			{
				cerr << MessagePrefix();
				cerr << "IM: no more free active intersetions available.  ";
				cerr << "Increase the size of cMAX_ACTIVE_IM_INTRSCTNS";
				cerr << endl;

				return;
			}
			activeIntrsctnIdx = m_freeActiveIntrsctnsIdx.front();
			m_freeActiveIntrsctnsIdx.pop();

#ifdef DEBUG_PROCESSREGISTER
			gout << "starting new IM for intersection " << intrsctnToRegister;
			gout << endl;
#endif
			
			m_intrsctns[intrsctnToRegister].active = true;
			m_intrsctns[intrsctnToRegister].activeImIdx = activeIntrsctnIdx;

#ifdef DEBUG_PROCESSREGISTER
			gout << "registering obj " << objToRegister;
			gout << " on intersection " << intrsctnToRegister << endl;
			gout << "size of set = " << m_activeIntrsctns[activeIntrsctnIdx].m_objs.size() << endl;
#endif

			m_activeIntrsctns[activeIntrsctnIdx].m_intrsctnIdx = intrsctnToRegister;
			m_activeIntrsctns[activeIntrsctnIdx].m_objs.insert( objToRegister );
			if( activeIntrsctnIdx > m_activeIntrsctnsLastElem )
			{
				m_activeIntrsctnsLastElem = activeIntrsctnIdx;
			}
		}
	}
}  // end of ProcessRegisterRequests


//////////////////////////////////////////////////////////////////////////////
//
// Description: Displays debugging information about the active 
//   intersection list.
//	
// Remarks: 
//
// Arguments:
//	
// Returns:	
//
//////////////////////////////////////////////////////////////////////////////
void
CIntersectionManager::DebugActiveIntrsctns()
{
	int i;
	gout << "The active intersection table:" << endl;
	for( i = 0; i < cMAX_ACTIVE_IM_INTRSCTN; i++ )
	{
		if( m_activeIntrsctns[i].m_intrsctnIdx >= 0 )
		{
			gout << "  " << i << ":";

			set<int>::iterator j;
			for( 
				j = m_activeIntrsctns[i].m_objs.begin();
				j != m_activeIntrsctns[i].m_objs.end();
				j++
				)
			{
				gout << "  " << *j;
			}

			gout << endl;
		}
	}
}  // end of DebugActiveIntrsctns


void 
CIntersectionManager::UserPreActivity( 
			const CIntersectionMngrParseBlock* cpSnoBlock 
			)
{
#ifdef DEBUG_PREACTIVITY
	gout << endl;
	gout << MessagePrefix();
	gout << "======== IM PRE-ACTIVITY ==============" << endl;
#endif

	//
	// Remove inactive intersections.
	//
	RemoveInactiveIntrsctns();

	//
	// Process the latest register requests and update the 
	// active intersection arrays.
	//
	ProcessRegisterRequests();

//	DebugActiveIntrsctns();

	//
	// For each intersection, prioritize the vehicles.
	//
	CObjTypeMask objMask;
//	objMask.Set( eCV_TRAJ_FOLLOWER );
	objMask.Set( eCV_VEHICLE );
	objMask.Set( eCV_EXTERNAL_DRIVER );
//	objMask.Set( eCV_TRAILER );

	int i;
	for( i = 0; i < cMAX_ACTIVE_IM_INTRSCTN; i++ )
	{
		//
		// Skip inactive intersection managers.
		//
		bool intrsctnActive = m_activeIntrsctns[i].m_intrsctnIdx >= 0;
		if( !intrsctnActive )  continue;

		CIntrsctn intrsctn( *cved, m_activeIntrsctns[i].m_intrsctnIdx );
		
#ifdef DEBUG_PREACTIVITY
		gout << "prioritizing vehicles for intrsctn " << intrsctn.GetName();
		gout << endl;
		gout << "activeIntrsctnsIdx = " << i << "  intrsctn id = ";
		gout << m_activeIntrsctns[i].m_intrsctnIdx << endl;
#endif

		//
		// Get the prioritized corridors list for the current intersection.
		//
		static vector<TCrdrPriorityList> crdrList;
		crdrList.clear();
		intrsctn.PrioritizeCorridors( crdrList );

#if 0 //def DEBUG_PREACTIVITY
		gout << "  the crdr priority list:" << endl;
		int crdrId;
		vector<TCrdrPriorityList>::iterator crdrListNode;
		for( 
			crdrId = 0, crdrListNode = crdrList.begin(); 
			crdrListNode != crdrList.end();
			crdrId++, crdrListNode++
			)
		{
			TCrdrPriorityList& node = *crdrListNode;
			vector<int> cList = node.intrsctingCrdrs;
			gout << "    " << crdrId << ":";

			vector<int>::iterator elem;
			for( elem = cList.begin(); elem != cList.end(); elem++ )
			{
				gout << "  " << *elem;
			}

			gout << endl;
		}
#endif

		static vector<TVehiclePriorityList> vehPriorityList;
		vehPriorityList.clear();
		PrioritizeVehicles( 
					intrsctn,
					m_activeIntrsctns[i].m_objs,
					m_activeIntrsctns[i].m_objInfo,
					crdrList, 
					objMask,
					vehPriorityList,
					m_activeIntrsctns[i].m_removeObjs
					);

#ifdef DEBUG_PREACTIVITY
		{
			gout << "  vehicle priority list [size = " << vehPriorityList.size() << "]";
			gout << endl;
			vector<TVehiclePriorityList>::iterator i;
			for( i = vehPriorityList.begin(); i != vehPriorityList.end(); i++ )
			{
				TVehiclePriorityList node = *i;
				gout << "    " << node.objId << ":";

				vector<TAttachedList>::iterator j;
				for( j = node.vehicleList.begin(); j != node.vehicleList.end(); j++ )
				{
					TAttachedList objNode = *j;
					gout << "   " << objNode.obj << "|" << objNode.priority;
				}

				gout << endl;
			}

		}

		gout << "  vehicle remove object list [size = ";
		gout << m_activeIntrsctns[i].m_removeObjs.size() << "]: ";

		set<int>::iterator k;
		for( 
			k = m_activeIntrsctns[i].m_removeObjs.begin();
			k != m_activeIntrsctns[i].m_removeObjs.end();
			k++
			)
		{
			gout << " " << *k;
		}
		gout << endl;
#endif

		SignalVehicles( intrsctn, vehPriorityList );
	}

}  // end of UserPreActivity


void CIntersectionManager::UserPostActivity( 
			const CIntersectionMngrParseBlock* cpSnoBlock 
			)
{

}


void CIntersectionManager::UserDeletion( 
			const CIntersectionMngrParseBlock* cpSnoBlock
			)
{

	PrintDeletionMessage();

}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Computes the time that the object will take to reach the
//   merge point.
//	
// Remarks: Returns a large value for the time if the given velocity is
//   close to zero.
//
// Arguments:
//   mergeDist - The distance to the merge point (in feet).
//   objVel    - The object's velocity (in meters/second).
//	
// Returns:	A double that indicates the amount of time needed (in seconds).
//
//////////////////////////////////////////////////////////////////////////////
static double
ComputeTimeToMergeDist( double mergeDist, double objVel )
{

	bool zeroVel = fabs( objVel ) < cNEAR_ZERO;
	if( zeroVel )
	{
		const double cLONG_TIME = 1000.0;
		return cLONG_TIME;
	}
	else
	{
		return ( mergeDist * cFEET_TO_METER ) / objVel;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Computes the distance to a given point inside the 
//   intersection.
//	
// Remarks: 
//
// Arguments:
//   cObjRoadPos    - The object's current road position.
//   distToLocation - The distance to the point from the start of 
//                    the corridor (in feet).
//	
// Returns:	A double that indicates the distance to the point inside the
//   intersection (in feet).
//
//////////////////////////////////////////////////////////////////////////////
static double
ComputeDistToIntrsctnLocation( 
			const CRoadPos& cObjRoadPos,
			const CCrdr& cCrdr,
			double distToLocation
			)
{
	double dist;
	if( cObjRoadPos.IsRoad() )
	{
		bool objRoadConnectedToIntersection = (
			cCrdr.GetSrcRd() == cObjRoadPos.GetRoad() ||
			cCrdr.GetDstntnRd() == cObjRoadPos.GetRoad()
			);
		if( objRoadConnectedToIntersection )
		{
			if( cObjRoadPos.GetLane().GetDirection() == ePOS )
			{
				dist = (
					distToLocation + 
					cObjRoadPos.GetRoad().GetLinearLength() - 
					cObjRoadPos.GetDistance()
					);
			}
			else
			{
				dist = distToLocation + cObjRoadPos.GetDistance();
			}
		}
		else
		{
			CPath pathFromObjRoadToIntersection( cObjRoadPos );
			CRoadPos roadPosOnCrdr( cCrdr.GetIntrsctn(), cCrdr );
			bool success = pathFromObjRoadToIntersection.Append( roadPosOnCrdr );
			if( success )
			{
				dist = pathFromObjRoadToIntersection.GetLength() + distToLocation;
			}
			else
			{
				dist = (
					distToLocation + 
					cObjRoadPos.GetRoad().GetLinearLength() - 
					cObjRoadPos.GetDistance()
					);
				gout << "ComputeDistToIntrsctnLocation: error building path" << endl;
			}
		}
	}
	else
	{
		CIntrsctn crdrIntrsctn = cCrdr.GetIntrsctn();
		CIntrsctn objIntrsctn = cObjRoadPos.GetIntrsctn();

		bool sameIntrsctn = crdrIntrsctn.GetId() == objIntrsctn.GetId();
		if( sameIntrsctn )
		{
			dist = distToLocation - cObjRoadPos.GetDistance();
		}
		else
		{
			CCrdr objCrdr = cObjRoadPos.GetCorridor();
			dist = objCrdr.GetLength() - cObjRoadPos.GetDistance();
			CRoad nextRoad = objCrdr.GetDstntnRd();
			dist += nextRoad.GetLinearLength();
			dist += distToLocation;
		}
	}

	return dist;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Computes the distance to a given point inside the 
//   intersection.
//	
// Remarks: 
//
// Arguments:
//   cObjRoadPos    - The object's current road position.
//   crdr - The distance to the point from the start of 
//                    the corridor (in feet).
//	
// Returns:	A double that indicates the distance to the point inside the
//   intersection (in feet).
//
//////////////////////////////////////////////////////////////////////////////
static double
ComputeDistToIntrsctnLocation( 
			const CRoadPos& cObjRoadPos,
			double distToLocation
			)
{
	double dist;
	if( cObjRoadPos.IsRoad() )
	{
		if( cObjRoadPos.GetLane().GetDirection() == ePOS )
		{
			dist = (
				distToLocation + 
				cObjRoadPos.GetRoad().GetLinearLength() - 
				cObjRoadPos.GetDistance()
				);
		}
		else
		{
			dist = distToLocation + cObjRoadPos.GetDistance();
		}
	}
	else
	{
		dist = distToLocation - cObjRoadPos.GetDistance();
	}

	return dist;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Figures out if the vehicle can safely come to a stop
//   from its given velocity in the specified dist.
//	
// Remarks: 
//
// Arguments:
//	 objVel - The vehicle's velocity (meters/sec).
//   dist   - The distance in which to come to a stop (feet).
//
// Returns:	A boolean indicating if coming to a stop is possible.
//
//////////////////////////////////////////////////////////////////////////////
static bool
CanVehicleStop( double objVel, double dist )
{
	double distP = ( objVel * objVel )/ ( 0.6 * cGRAVITY );	// meters
	distP = distP * cMETER_TO_FEET;
	
	return (dist >= distP );
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Finds the ADO's target corridor at the given intersection.
//	
// Remarks: This function parses the string from the ADO's ImTargCrdrs 
//   monitor.  The monitor string has pairs of intersection id and corridor id.
//
// Arguments:
//   cIntrsctn - The intersection at which to find the targ corridor.
//   cStr - The string from the monitor.
//   crdrId - (output) The id of the ADO's target corridor at the given
//       intersection.
//   
//	
// Returns:	A boolean indicating if it found the target corridor associated
//  with the given intersection.
//
//////////////////////////////////////////////////////////////////////////////
static bool
FindAdoTargCrdr( const CIntrsctn cIntrsctn, const string& cStr, int& crdrId )
{
	//
	// Parse the string which has pairs of [intrsctnId crdrId].
	//
	const char* pToken = cStr.c_str();
	char token[256];
	strncpy( token, pToken,256 );
	char *pCurrPos = NULL;
	char* pIntrsctnId = strtok_s( token, " ",&pCurrPos );
	while ( pIntrsctnId )
	{
		int intrsctnId = atoi( pIntrsctnId );

		char* pCrdrId = strtok_s( NULL, " ",&pCurrPos );
		if( pCrdrId )
		{
			crdrId = atoi( pCrdrId );

			bool foundIntrsctn = intrsctnId == cIntrsctn.GetId();
			if( foundIntrsctn )
			{
				// found the intersection
				return true;
			}
			else
			{
				// look at the next pair
			    pIntrsctnId = strtok_s( NULL, " ",&pCurrPos );
			}
		}
		else
		{
			pIntrsctnId = NULL;
		}
	}

	//
	// Ran out of pairs and found no match.
	//
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Builds the corridor that represents the ADO's target 
//  corridor through the current intersection.
//	
// Remarks: 
//
// Arguments:
//   cpObj - A pointer to the ADO's CVED object.
//   cpObjHcsm - The pointer to the ADO's top-level hcsm.
//   cIntrsctn - The intersection at which to find the targ corridor.
//   crdr - (output) The target corridor.
//   
//	
// Returns:	A boolean indicating if it was able to build the target crdr.
//
//////////////////////////////////////////////////////////////////////////////
bool
CIntersectionManager::BuildAdoTargCrdr(
			const CDynObj* cpObj,
			CHcsm* pObjHcsm, 
			const CIntrsctn cIntrsctn, 
			CCrdr& crdr 
			)
{
	string targCrdrs;
	bool haveValFromMonitor = pObjHcsm->GetMonitorByName( 
													"ImTargCrdrs", 
													&targCrdrs
													);
	if( !haveValFromMonitor ) 
	{
		return false;
	}

	int crdrId;
	bool success = FindAdoTargCrdr( cIntrsctn, targCrdrs, crdrId );
	if( success )
	{
		CCrdr tempCrdr( cIntrsctn, crdrId );
		if( !tempCrdr.IsValid() )
		{
			cerr << MessagePrefix( cpObj->GetId() );
			cerr << "unable to build corridor from crdrId = ";
			cerr << crdrId << " at intersection named ";
			cerr << cIntrsctn.GetName() << endl;

			return false;
		}

		crdr = tempCrdr;
	}
	else
	{
		return false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Prioritizes the vehicles at an intersection.
//	
// Remarks: 
//
// Arguments:
//   cIntrsctn       - The current intersection.
//   cObjsAtIntrsctn - All the objects at the current intersection.
//   cObjInfo        - Information about the objects in this intersection.
//   cCrdrList       - The prioritized corridor list for this intersection.
//   cObjMask        - Look at only the objects specified by this mask.
//   vehPrioList     - (output) The output vector that contains the object 
//                     id, the vehicle ids of the first vehicles on the 
//                     object's prioritized corridors and their priority.
//   vehRemoveList   - The list of vehicles that no longer need to be
//                     looked at.
//	
// Returns:	
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntersectionManager::PrioritizeVehicles( 
			const CIntrsctn& cIntrsctn,
			const set<int>& cObjsAtIntrsctn,
			const map<int, TImActiveObjInfo>& cObjInfo,
			const vector<TCrdrPriorityList>& cCrdrList,
			const CObjTypeMask& cObjMask,
			vector<TVehiclePriorityList>& vehPrioList,
			set<int>& vehRemoveList
			) 
{

#ifdef DEBUG_PRIORITIZE_VEHICLES
	gout << MessagePrefix();
	gout << "=== PrioritizeVehicles =============================" << endl;
	gout << "  #objs on intersection = " << cObjsAtIntrsctn.size() << endl;
#endif

	//
	// For each vehicle in the intersection, examine the 
	// intersecting corridors based on the prioritized 
	// corridors list.
	//
	set<int>::const_iterator i;
	for( i = cObjsAtIntrsctn.begin(); i != cObjsAtIntrsctn.end(); i++ )
	{
		//
		// Get the vehicle's CVED object pointer.
		//
		int objId = *i;
		const CDynObj* pObj = cved->BindObjIdToClass( objId );
		if( !pObj || !pObj->IsValid() )  continue;
		const CVehicleObj * pVeh = dynamic_cast<const CVehicleObj*>(pObj);
		if (!pVeh) continue; //Do we have a vehicle
#ifdef DEBUG_PRIORITIZE_VEHICLES
	//	bool debugThisObj = objId == DEBUG_PRIORITIZE_VEHICLES;
	//	if(debugThisObj)
	//	{
			gout << MessagePrefix();
			gout << "=== PrioritizeVehicles =============================" << endl;
			gout << "  #objs on intersection = " << cObjsAtIntrsctn.size() << endl;

			gout << " curr Obj id = " << objId << " ~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	//	}
#endif

		//
		// Get the vehicle's RoadPos from the vehicle's monitor.
		//
		CRoadPos objRoadPos;
		CHcsm* pObjHcsm = m_pRootCollection->GetHcsm( pObj->GetHcsmId() );
		if( !pObjHcsm ) 
		{
			cerr << MessagePrefix( pObj->GetId() );
			cerr << "cannot get pointer to lead hcsm" << endl;
		}
		else 
		{
			bool haveValFromMonitor = pObjHcsm->GetMonitorByName( 
															"RoadPos", 
															&objRoadPos
															);
			if( !haveValFromMonitor || !objRoadPos.IsValid() ) 
			{
				gout << MessagePrefix( pObj->GetId() );
				gout << "failed to get object's RoadPos monitor";
				gout << endl;

				continue;
			}
		}

		if( !objRoadPos.IsValid() )
		{
#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{

				gout << "  no valid objRoadPos for current vehicle...";
				gout << "skip this vehicle" << endl;
		//	}
#endif

			continue;
		}

		//
		// Get the vehicle's (target) crdr through the intersection.
		//
		CCrdr crdr;
		bool success = BuildAdoTargCrdr( pObj, pObjHcsm, cIntrsctn, crdr );
		if( !success )  continue;

		//
		// Figure out if this vehicle should be removed from the
		// IM.  Basically when the vehicle is within 5ft of the
		// crdr's end or it is already on the crdr destination road,
		// it is eliminated from the IM.
		//
		// NOTE:
		// This should be replaced by more accurately checking when
		// the back end of the vehicle has crossed over the last
		// intersecting corridor.
		//

		bool removeFromIm = (
					!objRoadPos.IsRoad() && 
					fabs( objRoadPos.GetDistance() - crdr.GetLength() ) < 5.0 &&
					objRoadPos.GetCorridor().GetId() == crdr.GetId() 
					);
					
		if( removeFromIm )
		{
#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << "   reached end of crdr...removing" << endl; 
		//	}	
#endif
			vehRemoveList.insert( objId );
			continue;
		}
			
		int objCrdrId = crdr.GetRelativeId();
		double mrgVel1 = pObj->GetVelImm();

		//
		// If the crdr is a red light then introduce a mandatory stop.
		//
		bool stopDueToLight = false;
		bool stopDueToAvoidCollision = false;
        bool isOnRamp = objRoadPos.GetIsOnRamp();  
		double distToHldOfs = ComputeDistToIntrsctnLocation( 
											objRoadPos, 
											crdr,
											crdr.GetFirstHldOfsDist()
											);
		CCrdr::ECrdrPriority crdrPriority = crdr.GetCrdrPriority();
		bool haveTrafficLight = (
			crdrPriority == CCrdr::eLIGHT_RED ||
			crdrPriority == CCrdr::eLIGHT_RED_STRAIGHT ||
			crdrPriority == CCrdr::eLIGHT_RED_TURN_LEFT ||
			crdrPriority == CCrdr::eLIGHT_RED_TURN_RIGHT ||
			crdrPriority == CCrdr::eLIGHT_GREEN ||
			crdrPriority == CCrdr::eLIGHT_GREEN_STRAIGHT ||
			crdrPriority == CCrdr::eLIGHT_GREEN_TURN_LEFT ||
			crdrPriority == CCrdr::eLIGHT_GREEN_TURN_RIGHT ||
			crdrPriority == CCrdr::eLIGHT_YELLOW ||
			crdrPriority == CCrdr::eLIGHT_YELLOW_STRAIGHT ||
			crdrPriority == CCrdr::eLIGHT_YELLOW_TURN_LEFT ||
			crdrPriority == CCrdr::eLIGHT_YELLOW_TURN_RIGHT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_RED ||
			crdrPriority == CCrdr::eLIGHT_FLASH_RED_STRAIGHT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_RED_TURN_LEFT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_RED_TURN_RIGHT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_GREEN ||
			crdrPriority == CCrdr::eLIGHT_FLASH_GREEN_STRAIGHT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_GREEN_TURN_LEFT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_GREEN_TURN_RIGHT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_YELLOW ||
			crdrPriority == CCrdr::eLIGHT_FLASH_YELLOW_STRAIGHT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_YELLOW_TURN_LEFT ||
			crdrPriority == CCrdr::eLIGHT_FLASH_YELLOW_TURN_RIGHT
			);

#ifdef DEBUG_PRIORITIZE_VEHICLES
	//	if(debugThisObj)
	//	{
			gout << "  on crdr " << objCrdrId;
			gout << "  vel = " << mrgVel1 * cMS_TO_MPH << " mph" << endl;
			gout << "  distToHldOfs = " << distToHldOfs << " ft" << endl;
	//	}
#endif

		if( crdrPriority == CCrdr::eLIGHT_RED || crdrPriority == CCrdr::eLIGHT_RED_STRAIGHT || crdrPriority == CCrdr::eLIGHT_RED_TURN_LEFT || crdrPriority == CCrdr::eLIGHT_RED_TURN_RIGHT )
		{
			map<int, TImActiveObjInfo>::const_iterator j = cObjInfo.find( objId );
			bool foundObjInMap = j != cObjInfo.end();
			if( foundObjInMap && j->second.travelState == eSTOP )
			{
				//
				// I was already attempting to stop...no need to check
				// again.
				//
				stopDueToLight = true;
			}
			else
			{
				stopDueToLight = CanVehicleStop( mrgVel1, distToHldOfs );
			}

#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << "  stop due to red light = " << stopDueToLight << endl;
		//	}
#endif
		}
		else if( crdrPriority == CCrdr::eLIGHT_YELLOW || crdrPriority == CCrdr::eLIGHT_YELLOW_STRAIGHT || crdrPriority == CCrdr::eLIGHT_YELLOW_TURN_LEFT || crdrPriority == CCrdr::eLIGHT_YELLOW_TURN_RIGHT )
		{
			map<int, TImActiveObjInfo>::const_iterator j = cObjInfo.find( objId );
			bool foundObjInMap = j != cObjInfo.end();
			if( foundObjInMap && j->second.travelState == eSTOP )
			{
				//
				// I was already attempting to stop...no need to check
				// again.
				//
				stopDueToLight = true;
			}
			else
			{
				stopDueToLight = CanVehicleStop( mrgVel1, distToHldOfs );
			}

#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << "  stop due to yellow light = " << stopDueToLight << endl;
		//	}
#endif
		}

		//
		// Check to see if the vehicle is on a corridor which has a
		// lower priority than some other corridors.
		//
		bool hasPrioritizedCrdrList = (
			cCrdrList[objCrdrId].intrsctingCrdrs.size() > 0
			);
		if( !hasPrioritizedCrdrList && !haveTrafficLight )
		{

#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << "  no prioritized list...skip to next vehicle " << endl;
		//	}
#endif

			continue;
		}
		
#ifdef DEBUG_PRIORITIZE_VEHICLES
	//	if(debugThisObj)
	//	{
			gout << "  size of prioritized corridor list = ";
			gout << cCrdrList[objCrdrId].intrsctingCrdrs.size();
			gout << endl;
	//	}
#endif
		
		vector<int> interCrdrIds = cCrdrList[objCrdrId].intrsctingCrdrs;

		static vector<TAttachedList> vec;
		vec.clear();
		vector<int>::iterator j;
		for( j = interCrdrIds.begin(); j != interCrdrIds.end(); j++ )
		{

			int intersectingCrdrId = *j;
			
#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << endl;
				gout << "  looking at intersecting crdr = ";
				gout << intersectingCrdrId << endl;
		//	}
#endif

			CCrdr interCrdr( cIntrsctn, intersectingCrdrId  );
			int firstVehicle = cved->GetFirstObjOnIntrsctingCrdr( 
										objCrdrId, 
										interCrdr, 
										cObjMask 
										);

			CRoadPos firstObjRoadPos;
			const CDynObj* pFirstObj = NULL;
			bool foundIntersectingObj = false;
			do
			{

				if( firstVehicle == -1 )
				{
#ifdef DEBUG_PRIORITIZE_VEHICLES
				//	if(debugThisObj)
				//	{
						gout << "    crdr has no vehicles on it....continue";
						gout << endl;
				//	}
#endif
					break;
				}

				
#ifdef DEBUG_PRIORITIZE_VEHICLES
			//	if(debugThisObj)
			//	{
					gout << "    firstVehicle = " << firstVehicle << endl;
			//	}
#endif
				
				// Get the vehicle's CVED object pointer.
				pFirstObj = cved->BindObjIdToClass( firstVehicle );
				if( !pFirstObj->IsValid() )  break;

				// Get the vehicle's RoadPos from the vehicle's monitor.
				CHcsm* pFirstObjHcsm = m_pRootCollection->GetHcsm( 
													pFirstObj->GetHcsmId() 
													);
				if( !pFirstObjHcsm ) 
				{
					cerr << MessagePrefix( pObj->GetId() );
					cerr << "cannot get pointer to hcsm of vehicle ";
					cerr << firstVehicle << endl;
					break;
				}
				else 
				{
					static string roadPostStr = "RoadPos";
					bool haveValFromMonitor = pFirstObjHcsm->GetMonitorByName( 
																roadPostStr, 
																&firstObjRoadPos
																);
					if( !haveValFromMonitor || !firstObjRoadPos.IsValid() ) 
					{
						cerr << MessagePrefix( pObj->GetId() );
						cerr << "failed to get RoadPos monitor for vehicle ";
						cerr << firstVehicle << endl;

						break;
					}
				}

				if( !firstObjRoadPos.IsValid() )
				{

#ifdef DEBUG_PRIORITIZE_VEHICLES
				//	if(debugThisObj)
				//	{
						gout << "    no valid objRoadPos ...skip this vehicle.";
						gout << endl;
				//	}
#endif
					break;
				}


#ifdef DEBUG_PRIORITIZE_VEHICLES
			//	if(debugThisObj)
			//	{
					gout << "    firstObjRoadPos from monitor = " << firstObjRoadPos << endl;
			//	}
#endif

				//
				// If there is a vehicle that has passed an intersecting 
				// crdr and has its roadPos on the intersecting crdr's 
				// destination road, then remove that vehicle. ( Since 
				// when the obj center is on a road and has part still 
				// hanging on an intersection, it may be reported both 
				// on the road and intersection in the internal list.)
				//
				CIntrsctn dstntnIntrsctn;
				if( firstObjRoadPos.IsRoad() )
				{
					dstntnIntrsctn = firstObjRoadPos.GetLane().GetNextIntrsctn();

					if( dstntnIntrsctn.GetId() != cIntrsctn.GetId() )
					{
						vehRemoveList.insert( firstVehicle );
						break;
					}
				}



				//
				// Get the intersecting vehicle's target corridor.
				//
				CCrdr otherCrdr;
				int firstObjCrdrId;
				if( firstObjRoadPos.IsRoad() )
				{
#ifdef DEBUG_PRIORITIZE_VEHICLES
				//	if(debugThisObj)
				//	{
						gout << "  before call crdr id = " << crdr.GetRelativeId() << endl;
				//	}
#endif
					bool haveValFromMonitor = pFirstObjHcsm->GetMonitorByName( 
																	"TargCrdr", 
																	&otherCrdr
																	);
					if( !haveValFromMonitor || !otherCrdr.IsValid() ) 
					{
//						cerr << MessagePrefix( pObj->GetId() );
//						cerr << "failed to get TargCrdr monitor for vehicle ";
//						cerr << firstVehicle << endl;

						break;
					}
#ifdef DEBUG_PRIORITIZE_VEHICLES
				//	if(debugThisObj)
				//	{
						gout << "  after call crdr id = " << crdr.GetRelativeId() << endl;
				//	}
#endif

					firstObjCrdrId = otherCrdr.GetRelativeId();
				}
				else
				{
					firstObjCrdrId = firstObjRoadPos.GetCorridor().GetRelativeId();
                    double dist = -1;
                    vector<double> dists;
                    interCrdr.GetMrgDstFirst(dists);
                    if (dists.size() > objCrdrId){
                        dist = dists[objCrdrId];
                    }
                    double distOnCrdr = firstObjRoadPos.GetDistanceOnLane(firstObjCrdrId);
                    double distPastMergePnt = distOnCrdr - dist;
                    if (distPastMergePnt > 0){
                        double speed = max(pFirstObj->GetVel(),pObj->GetVel());
                        if ((speed * 2.0 * 3.28083) > distPastMergePnt){
                            firstObjCrdrId = -1;
                        }
                    }
                    
                }

#ifdef DEBUG_PRIORITIZE_VEHICLES
			//	if(debugThisObj)
			//	{
					gout << "    firstVehicle crdr Id = " << firstObjCrdrId << endl;
			//	}
#endif

				//
				// Check to make sure the first vehicle on this corridor is 
				// really on this corridor by checking it's corridor from its
				// Monitor.
				//
				foundIntersectingObj = intersectingCrdrId == firstObjCrdrId;
				if( !foundIntersectingObj )
				{
#ifdef DEBUG_PRIORITIZE_VEHICLES
				//	if(debugThisObj)
				//	{
						gout << "    failed same crdr test...look at the ";
						gout << "vehicle after this vehicle" << endl;
				//	}
#endif
					firstVehicle = cved->GetObjOnCrdrBehindObj( 
															firstVehicle,
															objCrdrId, 
															interCrdr, 
															cObjMask 
															);
				}
			} 
			while( !foundIntersectingObj );

			if( !foundIntersectingObj )  continue;

			double mrgDist1 = crdr.GetFirstMrgDist( intersectingCrdrId );
			double mrgDist2 = interCrdr.GetFirstMrgDist( objCrdrId );

			
			// This is for cases when the first merging dist occurs
			// at where the two corridors are not intersecting with
			// each other but are next to each other. Then use the last
			// merging dist.
			if( mrgDist1 < 0.0 )
			{
				mrgDist1 = crdr.GetLastMrgDist( *j );
				if( mrgDist1 < 0.0 )  mrgDist1 = 0.0;
			}

			//
			// There are times when CVED incorrectly reports 
			// corridor merging distances.  One way to catch
			// these erroneous results is when the merge dist
			// is smaller than the corridor hold offset 
			// distance.  In those cases, set the merge dist
			// to be the corridor's hold offset and some fudge
			// factor.
			//
			if( mrgDist1 < cCrdrList[objCrdrId].hldOfsDist )
			{
				mrgDist1 = cCrdrList[objCrdrId].hldOfsDist + 5.0;
#ifdef DEBUG_PRIORITIZE_VEHICLES
			//	if(debugThisObj)
			//	{
					gout << "    @caught CVED errorneous result for ";
					gout << "mrgDist1...adjusting" << endl;
			//	}
#endif
			}

			if( mrgDist2 < 0.0 )
			{
				mrgDist2 = interCrdr.GetLastMrgDist( objCrdrId );
				if( mrgDist2 < 0.0 )  mrgDist2 = 0.0;
			}

#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << "    mrgDist1 = " << mrgDist1 << " " ;
				gout << "    mrgDist2 = " << mrgDist2 << endl;
		//	}
#endif

			double dist1 = ComputeDistToIntrsctnLocation( objRoadPos, mrgDist1 );
			double dist2 = ComputeDistToIntrsctnLocation( firstObjRoadPos, mrgDist2 );
#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << "    dist1 = " << dist1 << " ";
				gout << "    dist2 = " << dist2 << endl;
		//	}
#endif
			
			double mrgVel2 = pFirstObj->GetVelImm();
			
			// Sometimes the obj velocity is negative.
			mrgVel1 = fabs( mrgVel1 );
			mrgVel2 = fabs( mrgVel2 );


			double t1 = ComputeTimeToMergeDist( dist1, mrgVel1);
			double t2 = ComputeTimeToMergeDist( dist2, mrgVel2);

			bool canIStopIfNeeded = CanVehicleStop( mrgVel1, dist1 );

#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj)
		//	{
				gout << "    mrgVel1 = " << mrgVel1 * cMS_TO_MPH << "mph  ";
				gout << "    mrgVel2 = " << mrgVel2 * cMS_TO_MPH << "mph" << endl;
				gout << "    t1 = " << t1 << " ";
				gout << "    t2 = " << t2 << endl;
				gout << "    canIStopIfNeeded = " << canIStopIfNeeded << endl;
		//	}
#endif

			if( !canIStopIfNeeded )
			{
#ifdef DEBUG_PRIORITIZE_VEHICLES
			//	if(debugThisObj)
			//	{
					gout << "    skipping since I can't stop anyway in time";
					gout << endl;
			//	}
#endif

				continue;
			}
		
			//
			// Calculate the adjust time.  This decides by how much
			// time the vehicle has to beat the other vehicle in
			// order for it to considered safe to traverse the 
			// intersection without stopping.
			//
			double adjustTime;
			bool leftTurnCrdr = crdr.GetCrdrDirection() == CCrdr::eLEFT;
			if( leftTurnCrdr && mrgVel1 * cMS_TO_MPH > 10.0 ) 
			{
				//
				// A vehicle that is turning left is going to have to
				// slow down to make the turn.  Thus, increase the 
				// time factor.
				//
				adjustTime = 4.0;
			}
			else
			{
				adjustTime = 1.0;
			}

			TAttachedList list;
			list.obj = firstVehicle;

			const double cMAX_TTC = 5.0;    // seconds
            bool otherVehicleIsOnRamp = false;
            if (firstObjRoadPos.IsRoad()){
                if (firstObjRoadPos.GetLane().IsOnRamp()){
                    otherVehicleIsOnRamp = true;
                }
            }else{
                vector< pair<int,double> > crds;
                firstObjRoadPos.GetCorridors(crds);
                if (crds.size() == 1){
                    auto ln = firstObjRoadPos.GetCorridor().GetSrcLn();
                    if (ln.IsValid() && ln.IsOnRamp()){
                        otherVehicleIsOnRamp = true;
                    }
                }
                
            }
            static string HasStopSignTargetStr = "HasStopSignTarget";
            bool vehHasStopTarget = false;
			bool haveValFromStop = pObjHcsm->GetMonitorByName( 
				    										HasStopSignTargetStr, 
				    										&vehHasStopTarget
				    										);

            bool otherVehicleWillArriveFirst = t2 < t1 + adjustTime;
			bool haveTimeBeforeOtherVehicleArrives = (t2 - t1) >= cMAX_TTC;
			bool otherVehicleTooClose = fabs( dist2 ) < 20.0;
			bool otherVehicleHasPriority = (
						otherVehicleWillArriveFirst ||
					//	haveTimeBeforeOtherVehicleArrives ||
						otherVehicleTooClose ||
                        vehHasStopTarget
						);
		    otherVehicleHasPriority = otherVehicleHasPriority & (!otherVehicleIsOnRamp);
			// Stop if not being able to pass the intersecting area
			// on time and if there is plenty of time for a stop.
			bool stopDueToLikelyCollision = false;
            bool otherVehicleAtStopSign = false;

            //both vehicles are stoped check to see if we are at a stop sign
            //If my vehicle has been at the stop sign longer, my vehicle
            //will get priority
            if (mrgVel1 < 1.0f && otherVehicleHasPriority){
                bool OtherHasStopSign = false;
                bool IHaveStopSign = false;
                //pObjHcsm 
				CHcsm* pFirstObjHcsm = m_pRootCollection->GetHcsm( 
													pFirstObj->GetHcsmId() 
													);

                if (pFirstObjHcsm){
                    static string HasStopSignTargetStr = "HasStopSignTarget";
                    bool hasStopTarget = false;
				    bool haveValFromMonitor3 = pFirstObjHcsm->GetMonitorByName( 
				    												HasStopSignTargetStr, 
				    												&hasStopTarget
				    												);
                    otherVehicleAtStopSign = hasStopTarget;
                    if (hasStopTarget || mrgVel1 < 1.0f){
                        int framesStoped = 0;
                        int otherFramsesStopped = 0;
                        static string HasStoppedAtStopSignFrameStr = "StoppedAtStopSignFrame";
                    
                    
				        bool haveValFromMonitor1 = pFirstObjHcsm->GetMonitorByName( 
				    												    HasStoppedAtStopSignFrameStr, 
				    												    &otherFramsesStopped
				    												    );
				        bool haveValFromMonitor2 = pObjHcsm->GetMonitorByName( 
				    												    HasStoppedAtStopSignFrameStr, 
				    												    &framesStoped
				    												    );
                        if (otherFramsesStopped > 0)
                            otherVehicleAtStopSign = true;
                    
                        if ((framesStoped < otherFramsesStopped || (framesStoped > 0 && hasStopTarget))  
                            && framesStoped > 0 ){
                            otherVehicleHasPriority = false;
                        }
                    }

                }
            }

#if 1

			if( canIStopIfNeeded && otherVehicleHasPriority)
			{ 
				if(!haveTimeBeforeOtherVehicleArrives ) 
				{
					if( !stopDueToLight && !otherVehicleIsOnRamp)
					{			

#ifdef DEBUG_PRIORITIZE_VEHICLES
				//		if(debugThisObj)
				//		{
							cout << " stop due to avoid collision " << endl;
				//		}
#endif
                        CAttr catt;
                        objRoadPos.GetAttribute(cCV_INTERSTATE_ATTR,catt);
                        bool isInterstate = catt.IsValid() && catt.GetId() == cCV_INTERSTATE_ATTR; 
                        if (isInterstate){
                            //For interstates, we are going to trust in merge + follow behaviors 
                            if (objRoadPos.GetIsOnRamp()){
 						        stopDueToLikelyCollision = false;
						        stopDueToAvoidCollision = false;
						        otherVehicleHasPriority = true;                       
                            }
                        }
						stopDueToLikelyCollision = false;
						stopDueToAvoidCollision = false;
						otherVehicleHasPriority = true;
					}
				}
			}		
#endif
	
			list.priority = otherVehicleHasPriority;
			vec.push_back( list );


#ifdef DEBUG_PRIORITIZE_VEHICLES
		//	if(debugThisObj) 
		//	{
				gout << "  leftTurnCrdr = " << leftTurnCrdr << endl;
				gout << "  otherVehicleWillArriveFirst = " << otherVehicleWillArriveFirst << endl;
				gout << "  haveTimeBeforeOtherVehicleArrives = " << haveTimeBeforeOtherVehicleArrives;
				gout << endl;
				gout << "  otherVehicleTooClose = " << otherVehicleTooClose << endl;
				gout << "  otherVehicleHasPriority = " << otherVehicleHasPriority << endl;
		//	}
#endif


		}	// for each prioritized corridor of the current vehicle.

 		TVehiclePriorityList node;
		node.objId = objId;
		node.vehicleList = vec;
		node.hldOfsDist = distToHldOfs;
		node.stopDueToLight = stopDueToLight; 
		node.stopDueToAvoidCollision = stopDueToAvoidCollision;
		node.stopCheckFrame = GetFrame();
        node.isOnRamp = isOnRamp;
		vehPrioList.push_back( node );

	}	//  for each vehicle in the intersection.
											
}	// end of PrioritizeVehicles

//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets a vehicle's ImStop dial to tell it whether to stop
//   or go.
//	
// Remarks: If an error occurs, this function prints an error message
//   and simply returns.
//
// Arguments:
//   objId      - The vehicle's CVED id.
//   state      - The vehicle's state.
//   hldOfsDist - The distance from the start of the corridor to the
//                hold offset.
//	
// Returns:	
//
//////////////////////////////////////////////////////////////////////////////
void
CIntersectionManager::SetVehicleState( 
			int objId, 
			EImTravelState state,
			double hldOfsDist
			)
{
	//
	// Get the vehicle's CVED object pointer.
	//
	const CDynObj* pObj = cved->BindObjIdToClass( objId );
	if( !pObj || !pObj->IsValid() )
	{
		cerr << MessagePrefix( objId ) << "IM:SetVehicleState: ";
		cerr << "unable to get a CVED object pointer" << endl;

		return;
	}

	//
	// Get a pointer to the vehicle's HCSM.
	//
	CHcsm* pObjHcsm = m_pRootCollection->GetHcsm( pObj->GetHcsmId() );
	if( !pObjHcsm ) 
	{
		cerr << MessagePrefix( objId ) << "IM:SetVehicleState: ";
		cerr << "cannot get pointer to hcsm" << endl;

		return;
	}

	CAdo* pAdoHcsm = dynamic_cast<CAdo*> ( pObjHcsm );
	if( !pAdoHcsm )
	{
		//
		// Expect this probem to happen for OwnVehicle since the HCSM
		// is a DriverMirror and not a CAdo.
		//
		if( objId != 0 )
		{
			cerr << MessagePrefix( objId ) << "IM:SetVehicleState: ";
			cerr << "dynamic cast to CAdo failed" << endl;
		}

		return;
	}

	if( state == eGO )
	{
		pAdoHcsm->SetDialImStopNoValue();
	}
	else if( state == eSTOP )
	{
		//CRoadPos roadPos;
		pAdoHcsm->SetDialImStop( hldOfsDist );
	}
}  // end of SetVehicleState


static void
UpdateActiveIntrsctnVehMapForGo( 
			int objId, 
			map<int, TImActiveObjInfo>& objInfo
			)
{

	map<int, TImActiveObjInfo>::iterator i = objInfo.find( objId );
	bool foundObjInMap = i != objInfo.end();
	if( foundObjInMap ) 
	{
		//
		// Update info about this object into the map.
		//
		if( i->second.travelState != eGO ) 
		{
			i->second.travelState = eGO;
			i->second.framesStopped = 0;
		}

#ifdef DEBUG_SIGNAL_VEHICLES
		gout << "    updating info in map" << endl;
#endif
	}
	else
	{
		//
		// Insert info about the object into this map.
		//
		TImActiveObjInfo elem;
		elem.travelState = eGO;
		elem.stoppedForObj = -1;
		elem.framesStopped = 0;
		objInfo[objId] = elem;

#ifdef DEBUG_SIGNAL_VEHICLES
		gout << "    inserting new elem into map" << endl;
#endif
	}

}


static void
UpdateActiveIntrsctnVehMapForStop( 
			int objId, 
			int stoppedForObj, 
			map<int, TImActiveObjInfo>& objInfo 
			)
{

	map<int, TImActiveObjInfo>::iterator i = objInfo.find( objId );
	bool foundObjInMap = i != objInfo.end();
	if( foundObjInMap ) 
	{
		//
		// Update info about this object into the map.
		//
		if( i->second.travelState != eSTOP ) 
		{
			i->second.travelState = eSTOP;
			i->second.framesStopped = 0;
		}
		else
		{
			i->second.framesStopped = i->second.framesStopped + 1;
		}

#ifdef DEBUG_SIGNAL_VEHICLES
		gout << "    updating info in map" << endl;
#endif
	}
	else
	{
		//
		// Insert info about the object into this map.
		//
		TImActiveObjInfo elem;
		elem.travelState = eSTOP;
		elem.stoppedForObj = stoppedForObj;
		elem.framesStopped = 0;
		objInfo[objId] = elem;

#ifdef DEBUG_SIGNAL_VEHICLES
		gout << "    inserting new elem into map" << endl;
#endif
	}

}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Signals the vehicles to stop/go given the vehicle priority
//   list for an intersection.
//	
// Remarks: Communicates to vehicles using their buttons and dials.
//
// Arguments:
//   cIntrsctn       - The current intersection.
//   cVehPrioList    - The vehicle priority list.
//	
// Returns:	
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntersectionManager::SignalVehicles( 
			const CIntrsctn& cIntrsctn, 
			const vector<TVehiclePriorityList>& cVehPrioList
			)
{

#ifdef DEBUG_SIGNAL_VEHICLES
	gout << MessagePrefix();
	gout << "=== SignalVehicles =================================" << endl;
#endif

	//
	// Iterate through the result vector. If the vehicle has no other 
	// vehicles in its way or if the vehicle can cross the intersecting
	// point before other vehicles reach there, let it go.
	//
	int numObjsInGoState = 0;
	int randomIndex;
	bool go = false;

#ifdef DEBUG_SIGNAL_VEHICLES
	gout << "vehicle priority list size = " << cVehPrioList.size() << endl;
#endif

	int activeIntrsctnIdx = m_intrsctns[ cIntrsctn.GetId() ].activeImIdx;

	int k;
	for( k = 0; k < cVehPrioList.size(); k++ )
	{
		int objId = cVehPrioList[k].objId;

#ifdef DEBUG_SIGNAL_VEHICLES
		gout << "  vehicle " << objId << "---->" << endl;
#endif

		vector<TAttachedList> prioList = cVehPrioList[k].vehicleList;
		if( prioList.size() == 0 && ( !cVehPrioList[k].stopDueToLight ) )
		{
			//
			// Let the vehicle go because there are no other vehicles
			// on its intersecting corridors.
			//
#ifdef DEBUG_SIGNAL_VEHICLES
			gout << "    go" << endl;
#endif
			go = true;
			numObjsInGoState++;

			UpdateActiveIntrsctnVehMapForGo( 
						objId, 
						m_activeIntrsctns[activeIntrsctnIdx].m_objInfo
						);
				
			SetVehicleState( objId, eGO, cVehPrioList[k].hldOfsDist );
		}
		else
		{
			//
			// This vehicle has other vehicles on its priority list.
			// Look at each vehicle to see if this vehicle has priority
			// over all of them.
			//
			bool hasPriority = true;
			int stoppedForObj = -1;
			int w;
			for( w = 0; w < prioList.size(); w++ )
			{
				bool otherVehHasPriority = prioList[w].priority;

				if( otherVehHasPriority ) {
					stoppedForObj = prioList[w].obj;
					hasPriority = false;

					break;
				}
			}
            //the Merge controller takes care of avoiding 
			if( (hasPriority || cVehPrioList[k].isOnRamp) && 
                ( !cVehPrioList[k].stopDueToLight && !cVehPrioList[k].stopDueToAvoidCollision ) )
			{
				// Let the vehicle go since it can cross the intersecting 
				// point before all the other vehicles on its intersecting 
				// corridors reach there.

				go = true;
				numObjsInGoState++;
				
				UpdateActiveIntrsctnVehMapForGo( 
							objId, 
							m_activeIntrsctns[activeIntrsctnIdx].m_objInfo
							);

				SetVehicleState( objId, eGO, cVehPrioList[k].hldOfsDist );

#ifdef DEBUG_SIGNAL_VEHICLES
				gout << "    go" << endl;
#endif
			}
			else
			{
				//
				// Stop this vehicle.
				//
				UpdateActiveIntrsctnVehMapForStop( 
							objId, 
							stoppedForObj, 
							m_activeIntrsctns[activeIntrsctnIdx].m_objInfo
							);

				SetVehicleState( objId, eSTOP, cVehPrioList[k].hldOfsDist );

#ifdef DEBUG_SIGNAL_VEHICLES
				gout << "    stop" << endl;
#endif
			}
		}
	}

	if( numObjsInGoState <= 0 )
	{
#ifdef DEBUG_SIGNAL_VEHICLES
			gout << "numObjsInGoState = " << numObjsInGoState << endl;
#endif
	}


#if 0
	// If number of objects in the above out vector is equal to 
	// the number of objects in the intersection and that there
	// is no vehicle has the priority to go, then have to randomly
	// select to go. Sometimes the number of objs in the out vector
	// is less than the number of objs in the intersection, it means
	// some objects don't have prioritized list and have the priority 
	// to go any time. Then it is not a deadlock.
	if( cObjsAtIntrsctn.size() == cVehPrioList.size() )
	{
		if( numObjsInGoState == 0 || !go )
		{
			// If there is 0 vehicle on the intersection that has priority,
			// select a vehicle at random to let it go. To make it easier, 
			// select any slot in the above result vector. Then let the 
			// vehicle at cVehPrioList[randomIndex] go.

#ifdef DEBUG_SIGNAL_VEHICLES
			gout << endl;
			gout << " Have to randomly select a vehicle to go because of a deadlock. ";
			gout << endl;
#endif

			randomIndex = rand() % cVehPrioList.size();
		}
	}
#endif

}  // end of SignalVehicles

