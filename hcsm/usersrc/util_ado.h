/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: util_ado.h,v 1.10 2003/01/16 19:32:41 oahmad Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:		 May, 2000
 *
 * Description:  This header file includes common utility constants, 
 *   macros and enumerations for ADOs.
 *
 ****************************************************************************/

#ifndef __UTIL_ADO_H
#define __UTIL_ADO_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#endif

enum ELcCondition { 
			eLC_NONE, 
			eLC_EXTERNAL_COMMAND, 
			eLC_PATH_GUIDANCE,
			eLC_LOSING_CORRIDOR, 
			eLC_NONPASSING_LANE,
			eLC_SLOW_VEHICLE,
			eLC_VERY_SLOW_VEHICLE, 
			eLC_HIGHWAY_MERGE, 
			eLC_AVOID_MERGING_VEHICLE
			};

const int cLC_STATUS_DO_LC = -1;
const int cLC_STATUS_NO_LC = 0;
const int cLC_STATUS_NO_LC_UNTIL_INTRSCTN = 1;

#endif // __UTIL_ADO_H
