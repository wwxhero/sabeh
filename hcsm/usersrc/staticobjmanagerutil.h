/***************************************************************************** *
 * (C) Copyright 2001 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: staticobjmanagerutil.h,v 1.1 2001/01/18 21:03:23 schikore Exp $
 *
 * Author:  Matt Schikore
 *
 * Date:    January, 2001
 *
 * Description:  Contains code for the static object manager HCSM.
 *
 ****************************************************************************/


#ifndef __STATICOBJMANAGERUTIL_H_
#define __STATICOBJMANAGERUTIL_H_

void
StaticObjManInitialSetup( const CSobjMngrParseBlock* pSnoBlock,
                        CCved& cved,
                        CHcsmCollection& rootCollection,
                        CHcsm* pStaticObjMan );

#endif   //   __STATICOBJMANAGERUTIL_H_
