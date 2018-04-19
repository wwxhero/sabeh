/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: lanechangecond.cxx,v 1.6 2015/11/20 15:19:57 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    May, 2000
 *
 * Description:  Contains the implementation for the CLaneChangeCond class.
 *
 ****************************************************************************/

#include "lanechangecond.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CLaneChangeCond::CLaneChangeCond():
	m_active( false ),
	m_urgency( 0.5 ),
	m_type( eLC_NONE ),
	m_leftLaneChange( false ),
    m_isFinishing(false),
    m_scenarioInducedAbort(false)
{

}

CLaneChangeCond::CLaneChangeCond(
	const bool active,
	const double urgency,
	const CLane& targLane,
	const ELcCondition type,
	const bool leftLaneChange
	):
	m_active( active ),
	m_urgency( urgency ),
	m_isTargLane( true ),
	m_targLane( targLane ),
	m_type( type ),
	m_leftLaneChange( leftLaneChange ),
    m_isFinishing(false),
    m_scenarioInducedAbort(false)
{

}

CLaneChangeCond::CLaneChangeCond(
	const bool active,
	const double urgency,
	const CCrdr& targCrdr,
	const ELcCondition type,
	const bool leftLaneChange
	):
	m_active( active ),
	m_urgency( urgency ),
	m_isTargLane( false ),
	m_targCrdr( targCrdr ),
	m_type( type ),
	m_leftLaneChange( leftLaneChange ),
    m_isFinishing(false),
    m_scenarioInducedAbort(false)
{

}

CLaneChangeCond::CLaneChangeCond( const CLaneChangeCond& objToCopy )
{

	// call the assignment operator
	*this = objToCopy;

}

CLaneChangeCond& 
CLaneChangeCond::operator=( const CLaneChangeCond& objToCopy )
{

	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) {

		m_active     = objToCopy.m_active;
		m_urgency    = objToCopy.m_urgency;
		m_isTargLane = objToCopy.m_isTargLane;
		m_targLane   = objToCopy.m_targLane;
		m_targCrdr   = objToCopy.m_targCrdr;
		m_type       = objToCopy.m_type;
		m_leftLaneChange = objToCopy.m_leftLaneChange;
        m_isFinishing = objToCopy.m_isFinishing;
        m_scenarioInducedAbort = objToCopy.m_scenarioInducedAbort;

	}

	return *this;

}

CLaneChangeCond::~CLaneChangeCond()
{


}
