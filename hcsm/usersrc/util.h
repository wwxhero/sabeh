/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: util.h,v 1.8 2016/01/12 17:19:17 iowa\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:		 May, 2000
 *
 * Description:  This header file includes common utility constants and
 *   macros.
 *
 ****************************************************************************/

#ifndef __UTIL_H
#define __UTIL_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#endif

//
// Constants.
//
const double cNEAR_ZERO = 0.0001;
const double cMPH_TO_MS = 0.44703;    // meters/sec
const double cMPH_TO_FTPS = 1.46667;  // feet/sec
const double cMS_TO_MPH = 2.23700;    // mph
const double cMS_TO_KPH = 3.6;        // kph
const double cGRAVITY = 9.80665;      // meters/sec^2
const double cMETER_TO_FEET = 3.2808; // feet 
const double cFEET_TO_METER = 0.3048; // meters
const double cRAD_TO_DEG = 57.2957795;
const double cDEG_TO_RAD = 0.0174533;
const double cFEET_PER_MILE = 5280.0; // feet per mile
const double cPI = 3.141592653589793238;
//
// Macros.
//
#ifndef MAX
#define MAX(arg1, arg2) ( (arg1) > (arg2) ? (arg1) : (arg2) )
#endif

#ifndef MIN
#define MIN(arg1, arg2) ( (arg1) < (arg2) ? (arg1) : (arg2) )
#endif

#endif // __UTIL_H
