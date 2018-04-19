/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: maintaingapInfo.cxx,v 1.11 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    September, 2002
 *
 * Description:  Contains the implementation for the CMaintainGapInfo class.
 *
 ****************************************************************************/

#include "maintaingapinfo.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CMaintainGapInfo::CMaintainGapInfo():
	m_objName( "" ),
	m_objId( -1 ),
	m_distMode( true ),
	m_value( -1.0 ),
	m_maxSpeed( -1.0 ),
	m_minSpeed( -1.0 ),
	m_distKp( 6.0 ),
	m_velKp( 5.0 ),
	m_maxAccel( 10.0 ),
	m_maxDecel( -10.0 ),
	m_percentAroundTarget( -1.0 ),
	m_distDuration( -1.0 ),
	m_duration( -1.0 ),
	m_disableSpeed( -1.0 ),
	m_duration2( -1.0 ),
	m_pMaintainGapDial( NULL )
{
	m_hasExpr = false;

}

CMaintainGapInfo::CMaintainGapInfo(
	const string& cObjName,
	const int cObjId,
	bool cDistMode,
	const double& cValue,
	const double& cMaxSpeed,
	const double& cMinSpeed,
	const double& cDistKp,
	const double& cVelKp,
	const double& cMaxAccel,
	const double& cMaxDecel,
	const double& cPercentAroundTarget,
	const double& cDistDuration,
	const double& cDuration,
	const double& cDisableSpeed,
	const double& cDuration2,
	CDial* pDial
	):
	m_objName( cObjName ),
	m_objId( cObjId ),
	m_distMode( cDistMode ),
	m_value( cValue ),
	m_maxSpeed( cMaxSpeed ),
	m_minSpeed( cMinSpeed ),
	m_distKp( cDistKp ),
	m_velKp( cVelKp ),
	m_maxAccel( cMaxAccel ),
	m_maxDecel( cMaxDecel ),
	m_percentAroundTarget( cPercentAroundTarget ),
	m_distDuration( cDistDuration ),
	m_duration( cDuration ),
	m_disableSpeed( cDisableSpeed ),
	m_duration2( cDuration2 ),
	m_pMaintainGapDial( pDial )
{

}

CMaintainGapInfo::CMaintainGapInfo( const CMaintainGapInfo& objToCopy )
{

	// call the assignment operator
	*this = objToCopy;

}

CMaintainGapInfo& 
CMaintainGapInfo::operator=( const CMaintainGapInfo& cRhs )
{

	// check to make sure that object passed in is not me
	if( this != &cRhs ) 
	{
		m_objName       = cRhs.m_objName;
		m_objId         = cRhs.m_objId;
		m_distMode      = cRhs.m_distMode;
		m_value         = cRhs.m_value;
		m_maxSpeed      = cRhs.m_maxSpeed;
		m_minSpeed      = cRhs.m_minSpeed;
		m_distKp        = cRhs.m_distKp;
		m_velKp         = cRhs.m_velKp;
		m_maxAccel      = cRhs.m_maxAccel;
		m_maxDecel      = cRhs.m_maxDecel;
		m_percentAroundTarget = cRhs.m_percentAroundTarget;
		m_distDuration  = cRhs.m_distDuration;
		m_duration      = cRhs.m_duration;
		m_disableSpeed  = cRhs.m_disableSpeed;
		m_duration2      = cRhs.m_duration2;
		m_pMaintainGapDial = cRhs.m_pMaintainGapDial;
		m_hasExpr		= cRhs.m_hasExpr;
	}

	return *this;

}

CMaintainGapInfo::~CMaintainGapInfo()
{


}

void
CMaintainGapInfo::SetMaintainGapDial( CDial* pDial )
{
	m_pMaintainGapDial = pDial;
}


bool
CMaintainGapInfo::IsActive()
{
	return m_objId >= 0;
}
bool
CMaintainGapInfo::UsingExpr(){
	return m_hasExpr;
}
void
CMaintainGapInfo::Reset()
{
	m_objId = -1;

	//
	// Set the MaintainGap Dial to no value.
	//
	if( m_pMaintainGapDial )
	{
		m_pMaintainGapDial->SetNoValue();
	}
	else
	{
		cerr << "Ado::CMaintainGap: m_pMaintainGapDial has not been set.";
		cerr << "  call SetMaintainGapDial member function" << endl;
	}
}
