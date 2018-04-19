/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: hcsminterface.h,v 1.5 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
 *
 * Author:  Jillian Vogel
 *
 * Date:    October, 1999
 *
 * Description:  Contains global functions prototypes for the HCSM system
 *
 ****************************************************************************/
#ifndef _HCSM_INTERFACE_H_
#define _HCSM_INTERFACE_H_

#ifdef _WIN32
#include <ostream>
#include <iostream>
#elif __sgi
#include <iostream.h>
#elif _PowerMAXOS
#include <iostream>
#endif

#include <string>
using namespace std;

void LogData( int streamNum, float val );
void MotionBasePreposition( void );
void MotionBaseTune( void );
void TerminateSimulation( void );

#endif // _HCSM_INTERFACE_H_

