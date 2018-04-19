/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: maintaingapInfo.h,v 1.11 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:		 September, 2002
 *
 * Description:  Interface for the CMaintainGapInfo class.
 *
 ****************************************************************************/

#ifndef __CMAINTAINGAPINFO_H
#define __CMAINTAINGAPINFO_H

#include "dial.h"
#include <string>
using namespace std;

//////////////////////////////////////////////////////////////////////////////
//
// This class contains the functionality than an ADO needs to keep track
// of status of individual lane change conditions.
//
//////////////////////////////////////////////////////////////////////////////
class CMaintainGapInfo
{

public:
	CMaintainGapInfo();
	CMaintainGapInfo( 
		const string&, 
		const int, 
		bool,
		const double&, 
		const double&,
		const double&,
		const double&,
		const double&,
		const double&, 
		const double&,
		const double&,
		const double&,
		const double&,
		const double&,
		const double&,
		CDial*
		);
	CMaintainGapInfo( const CMaintainGapInfo& );
	CMaintainGapInfo& operator=( const CMaintainGapInfo& );
	~CMaintainGapInfo();

	void SetMaintainGapDial( CDial* pDial );
	bool IsActive();
	bool UsingExpr();
	void Reset();

	string       m_objName;      // object's name to maintain gap from
	int          m_objId;        // object's cved id
	bool         m_distMode;     // dist or time mode
	double       m_value;        // distance (ft) to maintain gap from object
								 // or time (sec) to maintain gap from object
	double       m_maxSpeed;     // max speed (m/s) to achieve
	double       m_minSpeed;     // min speed (m/s) to no drop below
	double       m_distKp;       // distance gain
	double       m_velKp;        // velocity gain
	double       m_maxAccel;     // m/s^2
	double       m_maxDecel;     // m/s^2
	double       m_percentAroundTarget; // can sway this much from specified dist
	double       m_distDuration; // spend x seconds [inside range if specified] and then reset
	double       m_duration;     // max duration
	double       m_disableSpeed; // disable maintain gap above this speed (m/s)
	double       m_duration2;     // max duration2 for Veridian equation
	bool		 m_hasExpr;		 //is it using an expression for distance?

	CDial*       m_pMaintainGapDial; // a pointer to the maintaingap dial
};

#endif // __CMAINTAINGAPINFO_H
