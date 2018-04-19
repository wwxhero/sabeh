/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: support.h,v 1.14 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:		 May, 2000
 *
 * Description:  This header file contains prototypes for functions in 
 *   support.cxx.
 *
 ****************************************************************************/

#ifndef __SUPPORT_H
#define __SUPPORT_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
#pragma warning(disable:4786) 
#endif

#include <pi_vector>
#include <randnumgen.h>
using namespace std;

#include "adoinfo.h"
#include "candidateset.h"

typedef struct TVelInfo 
{
	double vel;
	double dist;
} TVelInfo;

typedef struct TAccelInfo
{
	double accel;
	double vel;
} TAccelInfo;

double
CalcTargAccelForNormalCruising(
			const double currVel,
			const double prevVel,
			const double targVel,
			const double aggressiveness,
			CRandNumGen& rndNumGen,
			const int rndStream
			);

double
GetAccel( const CObj* cpObj );

void
ResolveAccelConservative( 
			const vector<TVelInfo>& pairs,
			const double initVel,
			double& vel,
			double& dist
			);
void
ResolveAccelConservative2( 
			const double cAccelList[],
			int     size,
			int&    which
			);

void
ResolveAccelInfoConservative(
			const vector<TAccelInfo>& cAccelList,
			double& accel,
			double& vel
			);

double CalculateRandomSignalTime(
			const double minSignalTime,
			const double maxSignalTime,
			CRandNumGen& rndNumGen,
			const int rndStream
			);

void NewPath( CAdoInfo* pI );
void ExtendPath( CAdoInfo* pI );

int GetInstigatorHcsmId( const set<CCandidate>& cInstigatorSet );
void PutCandidatesIntoArr( 
			const set<CCandidate>& cCandidateSet,
			int* pCandidateArr,
			int& candidateArrSize
			);

#endif // __SUPPORT_H
