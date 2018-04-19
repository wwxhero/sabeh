/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: lanechangeconds.h,v 1.17 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:		 May, 2000
 *
 * Description:  Interface for the CLaneChangeConds class.
 *
 ****************************************************************************/

#ifndef __CLANECHANGECONDS_H
#define __CLANECHANGECONDS_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#endif

#include <pi_map>
using namespace std;

#include "lanechangecond.h"
#include "util_ado.h"
#include "adoinfo.h"
#include <cved.h>

typedef map<ELcCondition, CLaneChangeCond> TLcConditions;

class CHcsmCollection;

//////////////////////////////////////////////////////////////////////////////
//
// This class contains the functionality than an ADO needs to keep track
// of status of all lane change conditions.  The functions SetCved and 
// SetHcsmCollection must be called sometime after creation and before
// calling any other functions in this class.
//
//////////////////////////////////////////////////////////////////////////////
class CLaneChangeConds
{

public:
	CLaneChangeConds();
	CLaneChangeConds( const CLaneChangeConds& );
	CLaneChangeConds& operator=( const CLaneChangeConds& );
	~CLaneChangeConds();

	void SetCved( CCved* );
	void SetHcsmCollection( CHcsmCollection* );
	CLaneChangeCond& GetCondition( const ELcCondition );
	void SetCondition( const ELcCondition, const CLaneChangeCond& );

	bool AnyConditionsActive();
	ELcCondition GetCurrentCondition( const ELcCondition );
	void ResetCurrentCondition();

	void CheckAllConditions( const CAdoInfo&, long, double );
	bool AbortCondition( const CAdoInfo&, bool );
	bool IsLeftLaneChange( const CLane&, const CLane& );

private:
	bool GetLeadObjectId( const CAdoInfo&, vector<int>&, int& );
	void InsertCondition( const ELcCondition, const CLaneChangeCond& );
	void CheckExternalCommand( const CAdoInfo&, CLaneChangeCond& );
    void CheckPathGuidance( const CAdoInfo&, CLaneChangeCond& );
	void CheckLosingCorridor( const CAdoInfo&, CLaneChangeCond& );
	void CheckSlowVehicle( const CAdoInfo&, CLaneChangeCond&, long, double );
	void CheckVerySlowVehicle( const CAdoInfo&, CLaneChangeCond&, long, double );
	void CheckHighwayMerge( const CAdoInfo&, CLaneChangeCond& );
	void CheckToAvoidMergingVehicle( const CAdoInfo&, CLaneChangeCond& );
    void CheckNonPassingLane( const CAdoInfo&, CLaneChangeCond&, long );
	bool GapAccept(  
				const CAdoInfo& cInfo,
				const double& cTtcBack,
				const double& cTtcFront,
				const double& cDistBack,
				const double& cDistFront,
				const CLane& cTargLane
				);
	bool GapAccept(  
				const CAdoInfo& info,
				const double ttcThreshold,
				const double distThreshold,
				const CCrdr& targCorridor
				);
	bool GapAcceptBackObjsOnCrdr(
				const CAdoInfo& info,
				const double ttcThreshold,
				const double distThreshold,
				const CCrdr& targCorridor
				);
	bool GapAcceptFwdObjsOnCrdr(
				const CAdoInfo& info,
				const double ttcThreshold,
				const double distThreshold,
				const CCrdr& targCorridor
				);

	TLcConditions m_conditions;
	CCved* m_pCved;
	CHcsmCollection* m_pRootCollection;
	long m_slowLcWaitStartFrame;    // frame # in which vehicle started waiting

};

#endif // __CLANECHANGECONDS_H
