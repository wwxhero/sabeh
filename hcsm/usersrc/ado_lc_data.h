/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 *
 * Version: $Id: ado_lc_data.h,v 1.9 2013/05/08 15:31:02 IOWA\vhorosewski Exp $

 * Author(s): Sunil Bulusu
 *
 * Date: April, 2004
 * Updated: May, 2005
 *
 * Description:  Interface for the ado_lc_data.cxx source file.
 *
 ****************************************************************************/
#ifndef __ADO_LC_DATA_H
#define __ADO_LC_DATA_H

bool LcLookupDistTime(
			double velocity,
			double urgency,
			int solId, 
			double &longDist,
			double &time
			);

bool LcLookupVelocityfromDist(
			double dist,
			double urgency,
			int solId, 
			double& velocity
			);

bool LcLookupVelocityfromDist(
			double dist,
			int solId, 
			vector<vector<double> >&
			);

bool LcLookupVelocityfromTime(
			double time,
			double urgency,
			int solId, 
			double& velocity	
			);

bool LcLookupVelocityfromTime(
			double dist,
			int solId,
			vector<vector<double> >&
			);

bool LcLookupUrgencyfromDist(
			double velocity,
			double Dist,	
			int solId, 
			double &urgency
			);

bool LcLookupUrgencyfromTime( 
			double velocity,
			double time,	
			int solId, 
			double &urgency
			);


// End of 'ado_lc_data.h' Header file

#endif  // __ADO_LC_DATA_H
