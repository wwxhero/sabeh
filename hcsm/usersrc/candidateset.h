/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: candidateset.h,v 1.10 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author:       Jillian Vogel
 * Date:         October, 1999
 *
 * Description: Definition of the CCandidateSet class used to assemble
 *				the candidate objects from various object filters.
 *				Also defines the CCandidate class, the basic product of
 *				the CCandidateSet class.
 *
 ****************************************************************************/
#ifndef __CANDIDATE_SET_H__
#define __CANDIDATE_SET_H__

#include "cvedpub.h"
#include "triggerparseblock.h"
#include "hcsmcollection.h"

////////////////////////////////
//                            //
//	CCandidate utility class  //
//                            //
////////////////////////////////
class CCandidate 
{
public:
	CCandidate();
	CCandidate( const CCandidate& );
	~CCandidate();
	CCandidate& operator=( const CCandidate& );
	bool operator<( const CCandidate& ) const;
	bool operator!=( const CCandidate& ) const;

	int m_hcsmId;
	int m_cvedId;
};
////////////////////////////////
//                            //
//	Relative utility class    //
//                            //
////////////////////////////////
class CRelativeParamInfo 
{
public:
	CRelativeParamInfo() {};
	CRelativeParamInfo( const string& cStr );
	~CRelativeParamInfo() {};

	bool	m_xFirst;
	int		m_lanes[10];
	int		m_numLanes;
	double	m_yMin;
	double	m_yMax;
	double	m_yIdeal;

private:
	void ScanX( const string& cStr );
	void ScanY( const string& cStr );
};

class CCandidateSet 
{
public:
	CCandidateSet(); 
	~CCandidateSet() {}
	CCandidateSet( const CCandidateSet& cS ) { *this = cS; }
	CCandidateSet& operator=( const CCandidateSet& );

	void AddNames( const vector<string>& );
	void AddTypes( const vector<string>& ); 
	void AddPstns( const vector<pbTPstn>& );
	void AddRoads( const vector<pbTRoad>& );

	void SetName( const string& );
	void SetType( const string& );
	void SetPstn( const pbTPstn& );
	void SetRoad( const pbTRoad& );
	void SetRelative( const CActionParseBlock::TRelativeInfo& );

	int GetCandidates( 
				const CVED::CCved&, 
				set<CCandidate>&,
				CHcsmCollection&
				) const;
	int CountObjsInLaneAndRemIf( 
				const CVED::CCved& cCved, 
				vector<int>& objIds, 
				int num, 
				int laneId 
				) const;

private:
	vector<string>		m_names;
	bool				m_useNames;
	
	CVED::CObjTypeMask	m_typeMask;
	bool				m_useTypes;
	
	vector<pbTPstn>		m_pstns;
	bool				m_usePstns;

	vector<pbTRoad>		m_roads;
	bool				m_useRoads;

	bool				m_relToDriver;
	string				m_relObjName;
	int					m_relNumObjs;
	vector<CRelativeParamInfo> m_relParams;
	bool				m_useRel;
};

#endif

