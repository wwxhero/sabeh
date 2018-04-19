/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: lanechangecond.h,v 1.8 2015/11/20 15:19:57 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:		 May, 2000
 *
 * Description:  Interface for the CLaneChangeCond class.
 *
 ****************************************************************************/

#ifndef __CLANECHANGECOND_H
#define __CLANECHANGECOND_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#endif

#include "util_ado.h"
#include <cvedpub.h>
using namespace CVED;

//////////////////////////////////////////////////////////////////////////////
//
// This class contains the functionality than an ADO needs to keep track
// of status of individual lane change conditions.
//
//////////////////////////////////////////////////////////////////////////////
class CLaneChangeCond
{

public:
	CLaneChangeCond();
	CLaneChangeCond( 
		const bool, 
		const double, 
		const CLane&, 
		const ELcCondition,
		const bool 
		);
	CLaneChangeCond( 
		const bool, 
		const double, 
		const CCrdr&, 
		const ELcCondition,
		const bool 
		);
	CLaneChangeCond( const CLaneChangeCond& );
	CLaneChangeCond& operator=( const CLaneChangeCond& );
	~CLaneChangeCond();

	inline bool   GetActive() const;
	inline void   SetActive( const bool );
	inline double GetUrgency() const;
	inline void   SetUrgency( const double );
	inline bool   IsTargLane() const;
	inline const  CLane& GetTargLane() const;
	inline void          SetTargLane( const CLane& );
	inline const CCrdr&  GetTargCrdr() const;
	inline void          SetTargCrdr( const CCrdr& );
	inline ELcCondition  GetType() const;
	inline void          SetType( const ELcCondition );
	inline bool   IsLeftLaneChange() const;
	inline void   SetLeftLaneChange( bool );
	inline bool   IsForcedLaneOffset() const;
	inline void   SetIsForcedLaneOffset( bool );
	inline double GetForcedLaneOffset() const;
	inline void   SetForcedLaneOffset( double );
	inline bool   IsSkipSignal() const;
	inline void   SetSkipSignal( bool );
    inline bool   IsFinishing() const;
    inline void   SetFinishing(bool);
    inline void   SetScenarioInducedAbort(bool);
    inline bool   GetScenarioInducedAbort();
	
private:
	bool         m_active;
	double       m_urgency;
	bool         m_isTargLane;  // TRUE: targLane;  FALSE: targCrdr
	CLane        m_targLane;
	CCrdr        m_targCrdr;
	ELcCondition m_type;
	bool         m_leftLaneChange;
	bool         m_isForcedLaneOffset;
	double       m_forcedLaneOffset;
	bool         m_skipSignal;
    bool         m_isFinishing; //<if we have effectivly "finished" our lane change but are looking to converge to a base condition
    bool         m_scenarioInducedAbort; //< some action has asked us to abort the lane change
};

#include "lanechangecond.inl"

#endif // __CLANECHANGECOND_H
