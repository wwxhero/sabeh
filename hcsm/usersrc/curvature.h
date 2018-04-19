/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: curvature.h,v 1.22 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:		 March, 2000
 *
 * Description:  Interface for the CCurvature class.
 *
 ****************************************************************************/

#ifndef __CCURVATURE_H
#define __CCURVATURE_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#endif

#include <pi_iostream>
#include <list>
#include <pi_string>
#include <pi_vector>
#include <deque>
using namespace std;

#include <cvedpub.h>
using namespace CVED;


typedef struct TCurvature 
{
	double    curvature;
	CRoadPos  roadPos;
	double    dist;
} TCurvature;


typedef struct TCurvLimit 
{
	int       code;
	int       lowerLimit;
} TCurvLimit;

typedef struct TBucket 
{
	int       code;
	double    lowCurv;
	CRoadPos  roadPos;
	double    distToLowCurv;
	double    curvVel;
	double    curvAccel;
} TBucket;

const double cSAMPLE_DIST = 6.0;  // feet
const double cDEFAULT_LAT_ACCEL     = 5.0;//3.92;  // 0.4G  m/s^2
const double cDEFAULT_LAT_ACCEL_BUS = 1.96;  // 0.2G  m/s^2

//////////////////////////////////////////////////////////////////////////////
//
// This class contains the functionality than an ADO needs to handle
// curvature on roads.  The user must call the following member functions
// in order to properly initialize this class:  InitializeCurvature, 
// ResetBuckets and UpdateBuckets.  Everytime the current road position
// changes, the user must call UpdateBuckets before calling GetCurvature
// to get the target velocity and target distance relevant to the upcoming
// curvature.
//
//////////////////////////////////////////////////////////////////////////////
class CCurvature
{

public:
	CCurvature();
	CCurvature( const cvEObjType );
	CCurvature( const CCurvature& );
	CCurvature& operator=( const CCurvature& );
	~CCurvature();

	void   SetCurvatureLatAccelLimit(const double&);
	void   SetCvedId( int );
	void   InitializeCurvature( const CRoadPos&, const CPath& );
	void   RefreshCurvature( const CRoadPos&, const CPath&, long );
	void   ResetBuckets();
	void   UpdateBuckets( const CRoadPos&, const double&, double );
	bool   GetCurvature( const double&, double&, double& );

	void   DebugCurvature();
	void   DebugBuckets();

private:
	void   InitializeLimits();
	int    FindCode( const double& );
	double CalcVel( const double& );
	double CalcAccel( const double&, const double&, const double& );
	int    FindElemFromDist( const double& );

	int m_cvedId;
	CRoadPos m_startRoadPos;
	CPath m_path;
	double m_latAccelms2; //meters per second squared
	deque<TCurvature> m_curvature;
	vector<TCurvLimit> m_limits;
	vector<TBucket> m_buckets;
	CRoadPos m_bucketRoadPos;
};

#endif // __CCURVATURE_H
