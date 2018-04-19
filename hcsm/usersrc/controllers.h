/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: controllers.h,v 1.15 2004/04/27 19:14:55 schikore Exp $
 *
 * Author(s):    Horatio German, Omar Ahmad
 *
 * Date:		 March, 2002
 *
 * Description:  Interface for the controllers.cxx source file.
 *
 ****************************************************************************/

#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

#include <randnumgen.h>


double SignificantAccelController( 
			const double currVel, 
			const double targVel,
			const double aggressiveness,
			CRandNumGen& randNumGen,
			const int rndStream
			);
double SlowDownGraduallyController(
			const double currVel, 
			const double targVel,
			const double aggressiveness
			);
double SpeedMaintenanceController(
			const double& cCurrVel, 
			const double& cPrevVel,
			const double& cTargVel,
			const double& cAggressiveness
			);
double LinearDecelController(
			const double& cCurrVel, 
			const double& cTargVel,
			const double& cTargDist
			);
bool StopController(
			const double& cCurrVel,
			const double& cDistToStop,
			const double& cAggressiveness,
			bool& firstFrame,
			double& firstDist,
			double& minDist,
			double& accel
			);
double ForcedVelocityController(
			const double& cCurrVel,
			const double& cTargVel,
			const double& cTargAccel
			);

class CFollowInfo;
struct TFollowParams;

bool FollowController(
			int           frame,
			double         deltaT,
			int           leadObjCvedId,
			CFollowInfo&  fi,
			const TFollowParams& fp,
			const double& cActualDist,
			const double& cPrevActualDist,
			int           followThisLeadAge,	// how long we're following him
			double         initFollowDist,
			const double& cLeadVel,
			const double& cCurrVel,
			const double& cPrevVel,
			const double& cLeadAccel,
			const double& cAggressiveness,
			const double& cMaxSpeed,
			double&       accelOutput,
			double&        ttgout,
			bool          gapMode = false
			);

#endif // __CONTROLLERS_H
