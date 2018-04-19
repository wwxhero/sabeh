/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: candidateset.cxx,v 1.33 2015/12/16 18:48:24 IOWA\dheitbri Exp $
 *
 * Author:       Jillian Vogel
 * Date:         October, 1999
 *
 * Description: Implementation of the CCandidateSet class, used to assemble
 *				the candidate HCSM and CVED ids from various object filters.
 *
 ****************************************************************************/
#include "candidateset.h"

#include <algorithm>
#include <roadpos.h>

#include <iterator>
#include <list>

#undef DEBUG_RELATIVE_CANDIDATES



void CRelativeParamInfo::ScanX(const string& str)
{
	char c;
	m_numLanes = sscanf(str.c_str(), "[%c %d:%d:%d:%d:%d:%d:%d:%d:%d:%d]", &c, &m_lanes[0], &m_lanes[1],  &m_lanes[2], &m_lanes[3], &m_lanes[4], &m_lanes[5], &m_lanes[6], &m_lanes[7], &m_lanes[8], &m_lanes[9]) - 1;
}

void CRelativeParamInfo::ScanY(const string& str)
{
	char c;
	double f[10];
	sscanf( str.c_str(), "[%c %lf:%lf:%lf]", &c, &f[0], &f[1], &f[2] );
	m_yMin = f[0];
	m_yMax = f[1];
	m_yIdeal = f[2];
}


CRelativeParamInfo::CRelativeParamInfo(const string& str)
{
	int firstStart = str.find( '[', 0 );
	if (firstStart == str.npos) return;
	int firstEnd = str.find( ']', firstStart );
	if (firstEnd == str.npos) return;
	int lastStart = str.find( '[', firstEnd );
	if (lastStart == str.npos) return;
	int lastEnd = str.find( ']', lastStart );
	if (lastEnd == str.npos) return;

	string first = str.substr(firstStart, firstEnd-firstStart+1);
	string last = str.substr(lastStart, lastEnd-lastStart+1);

	char c;
	sscanf( first.c_str(), "[%c", &c );
	if (c == 'Y') {
		m_xFirst = false;
		ScanY( first );
		ScanX( last );
	} else {
		m_xFirst = true;
		ScanX( first );
		ScanY( last );
	}

#ifdef DEBUG_RELATIVE_CANDIDATES
	
	//X - First from 0 to 1000, optimal 1.68809e-039
	//Y - Second lanes -1
	gout << "X - " << (m_xFirst?"First":"Second") << " from " << m_yMin 
		 << " to " << m_yMax << ", optimal " << m_yIdeal << endl;
	gout << "Y - " << (m_xFirst?"Second":"First") << " lanes ";
	for (int n = 0; n < m_numLanes; n++) gout << m_lanes[n] << ' ';
	gout << endl;

#endif

}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Default constructor
//
// Remarks: Initializes the local variables to default values.
//
//////////////////////////////////////////////////////////////////////////////
CCandidateSet::CCandidateSet() 
	: m_useNames(false),
	  m_useTypes(false),
	  m_usePstns(false),
	  m_useRoads(false),
	  m_useRel(false)
{ 
	// The default value for the type mask is
	//	every type.  That way, we can begin 
	//	with all objects and filter the list
	//	down from there.
	m_typeMask.SetAll();
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Overloaded assignment operator
//
// Remarks: Performs a deep copy from the parameter to the current instance
//
// Parameter: rhs - right-hand side of the assignment.
//
// Returns: pointer to the current instance
//
//////////////////////////////////////////////////////////////////////////////
CCandidateSet& 
CCandidateSet::operator=(const CCandidateSet& rhs)
{
	if (this != &rhs) {
		
		m_names		= rhs.m_names;
		m_useNames	= rhs.m_useNames;

		m_typeMask	= rhs.m_typeMask;
		m_useTypes	= rhs.m_useTypes;

		m_pstns		= rhs.m_pstns;
		m_usePstns	= rhs.m_usePstns;

		m_roads		= rhs.m_roads;
		m_useRoads	= rhs.m_useRoads;

		m_relToDriver	= rhs.m_relToDriver;
		m_relObjName	= rhs.m_relObjName;
		m_relNumObjs	= rhs.m_relNumObjs;
		m_relParams		= rhs.m_relParams;
		m_useRel		= rhs.m_useRel;
	}

	return *this;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Add a list of names to the local list 
//
// Parameter: names - names to add to the list
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::AddNames(const vector<string>& names) {

	vector<string>::const_iterator itr;
	for (itr = names.begin(); itr != names.end(); itr++)
		m_names.push_back(*itr);

	m_useNames = true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Add a list of types to the local type mask 
//
// Parameter: types - types to add to the mask
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::AddTypes(const vector<string>& types) {

	vector<string>::const_iterator itr;

	// Since the default value for type mask is all types, 
	//	we need to clear out the mask before adding the
	//	types the user wants.
	if (!m_useTypes)
		m_typeMask.Clear();

	for (itr = types.begin(); itr != types.end(); itr++)
		m_typeMask.Set(cvString2ObjType(itr->c_str()));
	
	m_useTypes = true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Add a list of positions to the local list 
//
// Parameter: pstns - positions to add to the list
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::AddPstns(const vector<pbTPstn>& pstns) {

	vector<pbTPstn>::const_iterator itr;
	for (itr = pstns.begin(); itr != pstns.end(); itr++)
		m_pstns.push_back(*itr);

	m_usePstns = true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Add a list of positions to the local list 
//
// Parameter: roads - positions to add to the list
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::AddRoads(const vector<pbTRoad>& roads) {

	vector<pbTRoad>::const_iterator itr;
	for (itr = roads.begin(); itr != roads.end(); itr++)
		m_roads.push_back(*itr);

	m_useRoads = true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets the relative set 
//
// Parameter: info - information describing which relative object(s) to get
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::SetRelative(const CActionParseBlock::TRelativeInfo& info) {
	m_relToDriver = info.m_isDriver;
	if (!m_relToDriver) m_relObjName = info.m_objName;
	for (vector<string>::const_iterator i = info.m_params.begin(); i != info.m_params.end(); i++) {
		// parse each string
		CRelativeParamInfo info(*i);
		m_relParams.push_back(info);
	}
	m_relNumObjs = info.m_numObjs;

	if (m_relParams.size() > 0) m_useRel = true;
}



//////////////////////////////////////////////////////////////////////////////
//
// Description: Make the parameter the only name in the local list
//
// Parameter: name - name to replace the list
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::SetName(const string& name) {

	m_names.clear();
	m_names.push_back(name);
	m_useNames = true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Make the parameter the only type in the local list
//
// Parameter: type - type to replace the list
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::SetType(const string& type) {

	m_typeMask.Clear();
	m_typeMask.Set(cvString2ObjType(type.c_str()));
	m_useTypes = true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Make the parameter the only position in the local list
//
// Parameter: pstn - position to replace the list
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::SetPstn(const pbTPstn& pstn) {

	m_pstns.clear();
	m_pstns.push_back(pstn);
	m_usePstns = true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Make the parameter the only road in the local list
//
// Parameter: road - road to replace the list
//
//////////////////////////////////////////////////////////////////////////////
void
CCandidateSet::SetRoad(const pbTRoad& road) {

	m_roads.clear();
	m_roads.push_back(road);
	m_useRoads = true;
}



//////////////////////////////////////////////////////////////////////////////
//
// Description: For a given trigger condition, if there are more than required 
//				objects	that satisfy this condition, this function eliminates 
//				objects belonging to relative object lane till reqd no. of objs
//				are remaining.
//
// Parameter: cved	 -  reference for the current CCved instance where the 
//						objects reside.
//			  objIds -  reference to the vector consisting of object Ids
//			  num    -  total objects required
//			  laneId -  lane Id of relative object
//
//  returns:  
//
//////////////////////////////////////////////////////////////////////////////
int
CCandidateSet::CountObjsInLaneAndRemIf( 
			const CVED::CCved& cCved, 
			vector<int>& objIds, 
			int num, 
			int laneId 
			) const
{
	int count = 0;
	vector<vector<int>::iterator> posToClear;
	posToClear.clear();

	// for each object
	vector<int>::iterator i;
	for( i = objIds.begin(); i != objIds.end(); i++ ) 
	{
		CPoint3D pos = cCved.GetObjPos( *i );
		CVED::CRoadPos rdPos( cCved, pos );
		//positions.push_back( rdPos );
        if (!rdPos.IsValid() || !rdPos.IsRoad())
            continue;
		bool onSameLane = rdPos.GetLane().GetRelativeId() == laneId;

#ifdef DEBUG_RELATIVE_CANDIDATES
		gout << "looking at obj " << *i << endl;
		gout << "  onSameLane = " << onSameLane << endl;
#endif
		// if objs on same lane, push it into vector
		if( onSameLane )
		{
			count++;
			posToClear.push_back( i );
		}
	}

	bool needToEliminateObjs = objIds.size() - count >= num;

#ifdef DEBUG_RELATIVE_CANDIDATES
	gout << "needToEliminateObjs = " << needToEliminateObjs << endl;
	if (needToEliminateObjs) gout << "Need to eliminate '" << count << "' objs from a total of '" 
		 << objIds.size() << "'\n";
#endif
	
	// Reverse iterate through posToClear and erase each obj Id from
	// vector objIds and re-evaluate condition to erase objs.
	if( needToEliminateObjs )
	{
		vector<vector<int>::iterator>::reverse_iterator rem;
		for( rem = posToClear.rbegin(); rem != posToClear.rend(); rem++ ) 
		{
			// erasing object Id
			objIds.erase( *rem );
			// popping out the iterator that got erased
			//posToClear.pop_back();
		}
        posToClear.clear();
		// re-evaluate the condition
		needToEliminateObjs = objIds.size() - count >= num;
	}

	bool needToRemoveMore = objIds.size() > num;
//gout << "tooManyObjects = " << tooManyObjects << endl;
	if( needToRemoveMore ) 
	{
		// there are still too many objects
		return 1;
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the HCSM IDs associated with the live, dynamic objects
//	in the simulation that match the given criteria.
//
// Remarks:  The candidates are assembled according to the following rules:
//	Multiple filter items of the same type are combined with an 'or' 
//	relationship, while filter items of different types are combined 
//	with an 'and' relationship.  For example, if 
//		m_names = { "Obj1", "Obj2" }
//		m_typesMask = TrajFollower
//	Then all objecs named either Obj1 or Obj2 and of type TrajFollower will
//	be returned.  
//
// Parameter: cved - reference for the current CCved instance where the 
//					objects reside.
//			  candidates - resulting set of CCandidates that contain the 
//							HCSM and CVED ids
//
// Returns: The number of candidates added to the set
//
int
CCandidateSet::GetCandidates(const CVED::CCved& cved, 
							 set<CCandidate>& candidates,
							 CHcsmCollection& collection) const 
{
	CCandidate tmpCand;
	set<CCandidate> nameObjs, typeObjs, pstnObjs, roadObjs;
	vector<int> tmpVec;
	vector<int>::const_iterator vecItr;

	int cntr = 0;

	bool setInit = false;

	if (m_useNames) {
		vector<string>::const_iterator itr;
		for (itr = m_names.begin(); itr != m_names.end(); itr++) {
			cved.GetObj(*itr, tmpVec);
			for (vecItr=tmpVec.begin(); vecItr!=tmpVec.end(); vecItr++) {
				tmpCand.m_cvedId = *vecItr;
				if ( *vecItr >= 0 ) {
					tmpCand.m_hcsmId = cved.GetObjHcsmId(*vecItr);
					candidates.insert(tmpCand);
				}
			}
			// need to check HCSMs that don't have CVED objects.
			if (*itr == "StaticObjManager") {
				CHcsm* pHcsm = collection.GetHcsm("StaticObjManager");
				if (pHcsm) {
					tmpCand.m_cvedId = 0;
					tmpCand.m_hcsmId = collection.GetHcsmId(pHcsm);
					candidates.insert(tmpCand);
				}
			}
			else if (*itr == "TrafficLightManager") {
				CHcsm* pHcsm = collection.GetHcsm("TrafficLightManager");
				if (pHcsm) {
					tmpCand.m_cvedId = 0;
					tmpCand.m_hcsmId = collection.GetHcsmId(pHcsm);
					candidates.insert(tmpCand);
				}
			}
		}
				
		setInit = true;
	}

	if ( m_usePstns ) {
		CPoint3D pos;
		if (setInit) {
			// we already have a set started, so go through the current set and remove any objects
			// not in the position set
			for (set<CCandidate>::iterator i = candidates.begin(); i != candidates.end(); i++) {
				CVED::CObj obj( cved, i->m_cvedId );
				bool inPosition = false;
				for (vector<pbTPstn>::const_iterator itr = m_pstns.begin(); itr != m_pstns.end(); itr++) {
					pos.m_x = itr->x;
					pos.m_y = itr->y;
					pos.m_z = itr->z;
					if ( pos.DistSq( obj.GetPos()) < itr->radius * itr->radius ) {
						inPosition = true;
						break;
					}
				}
				if (!inPosition) {
					set<CCandidate>::iterator tempIt = i;
					tempIt--;
					candidates.erase( i );
					if (candidates.size() == 0) break;
					i = tempIt;
				}
			}
		} else {
			vector<pbTPstn>::const_iterator itr;

			for (itr = m_pstns.begin();
				 itr != m_pstns.end(); itr++) {
			
				pos.m_x = itr->x;
				pos.m_y = itr->y;
				pos.m_z = itr->z;

				cved.GetObjsNear(pos, itr->radius, tmpVec, m_typeMask);

				for (vecItr = tmpVec.begin(); 
					 vecItr != tmpVec.end();
					 vecItr++) {
					tmpCand.m_cvedId = *vecItr;
					tmpCand.m_hcsmId = cved.GetObjHcsmId(tmpCand.m_cvedId);
					candidates.insert( tmpCand );
				}
			}
			setInit = true;
		}
	}

	if (m_useRoads) {
		vector<pbTRoad>::const_iterator itr;
		CVED::CRoad road;

		for (itr = m_roads.begin();
			 itr != m_roads.end(); itr++) {
		
#ifdef _PowerMAXOS
			bitset<cCV_MAX_LANES> lanes;
			for (int c = 0; c < itr->lanes.size(); c++) {
				if (itr->lanes[c] == '1')
					lanes[c] = 1;
			}
#else
			bitset<cCV_MAX_LANES> lanes(itr->lanes);
#endif		
			road = cved.GetRoad(itr->road);
			if (road.IsValid()) {
                tmpVec.clear();
				cved.GetAllDynObjsOnRoad(road.GetId(), lanes,
										 tmpVec, m_typeMask);

				for (vecItr = tmpVec.begin(); 
					 vecItr != tmpVec.end();
					 vecItr++) {
					tmpCand.m_cvedId = *vecItr;
					tmpCand.m_hcsmId = cved.GetObjHcsmId(tmpCand.m_cvedId);
					roadObjs.insert(tmpCand);
				 }
			}
		}
		if (setInit) {
			// the set is already initialized, so just intersect the roadset with the candidate set.
			set<CCandidate> intersect;
			insert_iterator<set<CCandidate> > resIns( intersect, intersect.begin() );
			set_intersection( roadObjs.begin(), roadObjs.end(), candidates.begin(), candidates.end(), resIns );
			candidates.swap( intersect );
		} else {
			candidates.swap( roadObjs );
			setInit = true;
		}
	}

	if ( m_useTypes ) {
		if (setInit) {
			// the set has been initialized, so just remove any objects in the set that are not of a type
			// in the type set.
			int num = candidates.size();
			for (set<CCandidate>::iterator i = candidates.begin(); i != candidates.end(); i++) {
				if ( !m_typeMask.Has( cved.GetObjType( i->m_cvedId ) ) ) {
					set<CCandidate>::iterator tempIt = i;
					tempIt--;
					candidates.erase( i );
					if (candidates.size() == 0) break;
					i = tempIt;
				}
			}
		} else {
			cved.GetAllDynamicObjs(tmpVec, m_typeMask);
			for (vector<int>::iterator i = tmpVec.begin(); i != tmpVec.end(); i++) {
				CCandidate cand;
				cand.m_cvedId = (*i);
				cand.m_hcsmId = cved.GetObjHcsmId(cand.m_cvedId);
				candidates.insert( cand );
			}
		}
	}

	if (m_useRel) 
	{
		static vector<CCandidate> curSetCand;
		curSetCand.clear();
		// Get the location of the object we're looking relative to.
		CVED::CRoadPos relObjPos(cved);
		CPoint3D relObjPt;
		if (m_relToDriver) 
		{
#ifdef DEBUG_RELATIVE_CANDIDATES
			gout << " ====== Relative Candidate testing  ========= " << endl;
#endif
			if( cved.GetOwnVehiclePos( relObjPt ) ) {
				relObjPos.SetXYZ( relObjPt );
			} else {
				gout << "Can't get driver's position" << endl;
			}
		} 
		else 
		{
			int id;
			if (cved.GetObj( m_relObjName, id )) 
			{
				// Found the object, get it's position
				relObjPos.SetXYZ( cved.GetObjPos( id ) );
			} 
			else 
			{
#ifdef DEBUG_RELATIVE_CANDIDATES
				gout << "Can't get object \"" << m_relObjName 
					 << "\"'s position" << endl;
#endif
			}
		}

		if (setInit) 
		{
			// the set has been initialized, so figure out which one(s) to keep
		}
		else 
		{
			// nothing in the set - start from scratch
			bool done = false;
			vector<CRelativeParamInfo>::const_iterator iter = m_relParams.begin();
			while (!done) {
				// 
				CVED::CPath path(cved);
				CVED::CRoadPos startPt( relObjPos );
				CVED::CRoadPos endPt( relObjPos );
				CVED::CRoadPos optPos( relObjPos );

				startPt.Travel( iter->m_yMin );
				endPt.Travel( iter->m_yMax );

#ifdef DEBUG_RELATIVE_CANDIDATES
				gout << "Start Travelling " << iter->m_yMin << endl;
				gout << "End Travelling " << iter->m_yMax << endl;
#endif

				optPos.Travel( iter->m_yIdeal );
				static bitset<cCV_MAX_CRDRS> laneMask;
				laneMask.reset();
				int targetObjLane = 0;
                if (relObjPos.IsRoad()){
                    targetObjLane = relObjPos.GetLane().GetRelativeId();
				    int x;
				    for ( x=0; x < iter->m_numLanes; x++ ) 
				    {
				    	laneMask.set( targetObjLane - 
				    		(relObjPos.GetLane().GetDirection()==ePOS?1:-1) * 
				    		iter->m_lanes[x] );

#ifdef DEBUG_RELATIVE_CANDIDATES
					gout << "Adding " << targetObjLane - iter->m_lanes[x] 
						 << " to lane mask" << endl;	
#endif
				    }

                }
				path.Initialize( startPt );
				path.Append( endPt );
                if (relObjPos.IsRoad())
                    path.SetLaneMask( laneMask );

#ifdef DEBUG_RELATIVE_CANDIDATES
				gout << "Path is " << path << endl;
#endif
				
				static vector<int> objIds;
				objIds.clear();
				path.GetObjectsOnPath( objIds );
				{
					// check if relative dial is set to
					// driver or existing ADO
					int relObjId;
					bool hasRelativeObj = 
						!m_relToDriver && cved.GetObj( m_relObjName, relObjId );

					// loop to erase 'external driver'
					vector<int>::iterator i;
					for( i = objIds.begin(); i != objIds.end(); i++ )
					{
						if( *i == 0 )
						{
							objIds.erase( i );

#ifdef DEBUG_RELATIVE_CANDIDATES
							gout << "\n*** eliminated driver from vector " 
            					 << "(size=" << objIds.size() << ") ***\n";
#endif

							break;
						}
					}
					// loop to erase 'relative obj'
					if( hasRelativeObj )
					{
						for( i = objIds.begin(); i != objIds.end(); i++ )
						{
							if( *i == relObjId ) 
							{
								objIds.erase( i );

#ifdef DEBUG_RELATIVE_CANDIDATES
								gout << "\n*** eliminated relative object from " 
									 << "vector (size=" << objIds.size() 
									 << ") ***\n" << endl;
#endif

								break;
							}					
						}
					}
				}

#ifdef DEBUG_RELATIVE_CANDIDATES
				gout << "Found " << objIds.size() << " + " 
					 << candidates.size() << " objects, need " 
					 << m_relNumObjs << endl;
#endif				
				// Calculate the total candidates found.
				if (objIds.size() + candidates.size() > m_relNumObjs) 
				{
					// need to sort them by priority
					int numNeeded = m_relNumObjs - candidates.size();

					// look at the first priority - by lane or by distance
					if (iter->m_xFirst)
					{
						// Get the numNeeded best by looking at X (lane)
						int curIndex = iter->m_numLanes-1;
						while (curIndex >= 0) 
						{
                            if (relObjPos.IsRoad()){
                                int curTargLane = targetObjLane - 
								    (relObjPos.GetLane().GetDirection()==ePOS?1:-1) * 
								    iter->m_lanes[curIndex];
    #ifdef DEBUG_RELATIVE_CANDIDATES
							    gout << "Checking lane " << curTargLane << endl;
    #endif
							    // NEED TO WORK ON THIS FUNCTION
							    if (!CountObjsInLaneAndRemIf( cved, objIds, numNeeded, curTargLane )) 
							    {
								    // skip the rest of the lanes, there are enough objects here
    #ifdef DEBUG_RELATIVE_CANDIDATES
								    gout << "Skipping the rest of the lanes" << endl;
    #endif
								    curIndex = 0;
							    }
							    curIndex--;
						    }else{
                                break;
                            }
                        }
					}
					
					// keep the N object in objIds that are closest to Y (distance along lane)
					while ((objIds.size() > 0) && (objIds.size() > numNeeded)) 
					{
						static vector<double> distAway;
						distAway.clear();
						for (vector<int>::iterator i = objIds.begin(); i != objIds.end(); i++) 
						{
							CPoint3D pos = cved.GetObjPos( *i );
							CVED::CRoadPos rdPos( cved, pos );
							CVED::CPath distPath( rdPos );
							bool before = true;
							if (!distPath.Append( optPos )) {
								distPath.Prepend( optPos );
								before = false;
							}

							// Find the distance btwn the rel obj Pos and curr
							// obj and push it into vector
							double dist = fabs( distPath.GetLength( before?&rdPos:&optPos, before?&optPos:&rdPos ));
							distAway.push_back( dist );
#ifdef DEBUG_RELATIVE_CANDIDATES
						gout << "objIds size = " << objIds.size() 
							 << ", cand size = " << candidates.size() 
							 << ", numNeed = " << numNeeded << endl;
							gout << "Object " << *i << " - off by " << dist << endl;
#endif
						}
						// remove the objects thats farthest from the relative 
						// object 
						vector<int>::iterator bigIndex = objIds.begin();
						vector<int>::iterator step = objIds.begin();
						double biggest = distAway[0];
						for (vector<double>::iterator big = distAway.begin(); 
							 big != distAway.end(); big++) 
						{
							if (*big > biggest)
							{
								biggest = *big;
								bigIndex = step;
							}
							step++;
						}
#ifdef DEBUG_RELATIVE_CANDIDATES
						gout << "Erasing object " << *bigIndex << endl;
#endif
						objIds.erase( bigIndex );
					}
				}
				
				/* WE DON'T NEED IT BUT KEEP IT FOR FUTURE
				// We need to remove duplicates caused due to 
				// two diff conditions
				//
				vector<int>::iterator remIter = objIds.begin();
				while( remIter != objIds.end() )
				{
					int countDuplicates = 0;
					for( int i = 0; i < objIds.size(); i++ )
					{
						if( *remIter == objIds[i] )
						{
							countDuplicates++;
							if( countDuplicates > 1 )
							{
								// so duplicate ids exist, erase them!
								// gout << "removing : " << *remIter << " " << objIds[i];
								objIds.erase( remIter );
								remIter = objIds.begin();
								countDuplicates = 0;
								break;
							}
						}
					}
					remIter++;
				}*/

				// by this point, we've removed any extras we don't need, so add them all.
				for ( int x=0; x < objIds.size(); x++ ) {
					CCandidate cand;
					cand.m_cvedId = (objIds[x]);
					cand.m_hcsmId = cved.GetObjHcsmId(cand.m_cvedId);
					candidates.insert( cand );
#ifdef DEBUG_RELATIVE_CANDIDATES
					gout << "Candidate cvedId=" << cand.cvedId << "  hcsmId=" << cand.hcsmId << endl;
#endif
				}
				iter++;
				if (iter == m_relParams.end()) done = true;
				if (candidates.size() >= m_relNumObjs) done = true;
			}
		}
	}

	return candidates.size();

}


/////////////////////////////////////////////////////////////////////////
//
//	CCandidate utility class member functions
//
/////////////////////////////////////////////////////////////////////////
CCandidate::CCandidate() : m_hcsmId(-1), m_cvedId(-1) {}
CCandidate::~CCandidate() {}
CCandidate::CCandidate(const CCandidate& rhs)
{
	*this = rhs;
}
/////////////////////////////////////////////////////////////////////////
//	
//	Operators: needed so that CCandidate can be a member of a STL set
//
CCandidate&
CCandidate::operator=(const CCandidate& rhs)
{
	if (this != &rhs) {
		m_hcsmId = rhs.m_hcsmId;
		m_cvedId = rhs.m_cvedId;
	}
	return *this;
}
bool
CCandidate::operator<(const CCandidate& rhs) const
{
	bool lessThan;
	if (rhs.m_hcsmId > -1 && m_hcsmId != rhs.m_hcsmId)
		lessThan = (m_hcsmId < rhs.m_hcsmId);
	else
		lessThan = (m_cvedId < rhs.m_cvedId);

	return lessThan;
}
bool
CCandidate::operator!=(const CCandidate& rhs) const
{
	bool notEqual;
	if (rhs.m_hcsmId > -1)
		notEqual = (m_hcsmId != rhs.m_hcsmId);
	else
		notEqual = (m_cvedId != rhs.m_cvedId);

	return notEqual;
}
