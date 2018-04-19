/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: ownvehiclemirror.cxx,v 1.3 2000/03/08 21:57:08 jvogel Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    November, 1999
 *
 * Description:  Contains code for the own vehicle mirror object
 *               (OwnVehicleMirror HCSM).
 *
 ****************************************************************************/

#include "genericinclude.h"
#include "genhcsm.h"
#include "hcsmcollection.h"

#ifdef __sgi
#include <strstream.h>
#define ostringstream ostrstream
#elif _PowerMAXOS
#include <ctype.h>
#include <strstream>
#define ostringstream ostrstream
#elif _WIN32
#include <sstream>
#endif

//
// Debugging macros.
//

void COwnVehicleMirror::UserCreation( 
			const COwnVehicleMirrorParseBlock* pSnoBlock
			)
{

	// print creation message with template name and hcsm id
	PrintCreationMessage();

}


void COwnVehicleMirror::UserPreActivity( 
			const COwnVehicleMirrorParseBlock* pSnoBlock 
			)
{	


}


void COwnVehicleMirror::UserPostActivity( 
			const COwnVehicleMirrorParseBlock* pSnoBlock 
			)
{

}


void COwnVehicleMirror::UserDeletion( 
			const COwnVehicleMirrorParseBlock* pSnoBlock
			)
{

	PrintDeletionMessage();

}
