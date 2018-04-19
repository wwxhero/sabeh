/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: enviroinfo.cxx,v 1.5 2004/04/27 19:15:00 schikore Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    November, 1999
 *
 * Description:  Contains code for the environment controller object
 *               (EnvironmentController HCSM).
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

#define DEBUG_LEVEL_ENVIRONMENTAL_CONTROLLER 1 

void CEnviroInfo::UserCreation( 
			const CEnviroInfoParseBlock* pSnoBlock
			)
{

	// print creation message with template name and hcsm id
	PrintCreationMessage();

	eCVEnviroType enviroType;	
	cvTEnviroInfo cvInfo;

	CEnviroInfoParseBlock::TEnviroInfo eI =
		pSnoBlock->GetEnviroInfo();

	enviroType = cvStringToEnviroType(eI.type);
	if (enviroType == eLIGHTNING){
		cvInfo.info.Lightning.degree = cvStringToLMHScale(eI.info.Lightning.degree);
	} else if (enviroType == eVISIBILITY){
		cvInfo.info.Visibility.dist = eI.info.Visibility.dist;
	} else if (enviroType == eHAZE){
		cvInfo.info.Haze.dist = eI.info.Haze.dist;
	} else if (enviroType == eFOG){
		cvInfo.info.Fog.dist = eI.info.Fog.dist;
	} else if (enviroType == eSMOKE){
		cvInfo.info.Smoke.degree = eI.info.Smoke.degree;
	} else if (enviroType == eCLOUDS){
		cvInfo.info.Clouds.altitude = eI.info.Clouds.altitude;
		cvInfo.info.Clouds.type = eI.info.Clouds.type;
	} else if (enviroType == eGLARE){
		cvInfo.info.Glare.degree = eI.info.Glare.degree;
	} else if (enviroType == eSNOW){
		cvInfo.info.Snow.degree = cvStringToLMHScale(eI.info.Snow.degree);
	} else if (enviroType == eRAIN){
		cvInfo.info.Rain.degree = cvStringToLMHScale(eI.info.Rain.degree);
	} else if (enviroType == eWIND){
		cvInfo.info.Wind.vel = eI.info.Wind.vel;
		cvInfo.info.Wind.gust = eI.info.Wind.gust;
#if 0
		cvInfo.info.Wind.dir.x = eI.info.Wind.dirX;
		cvInfo.info.Wind.dir.y = eI.info.Wind.dirY;
#endif
	}
	gout<<"\n envirotype : "<<eI.type<<endl;
#if 0
	cved->SetEnviroCondition(enviroType, cvInfo);
	cved->DumpEnvArea();
#endif
}


void CEnviroInfo::UserPreActivity( 
			const CEnviroInfoParseBlock* pSnoBlock 
			)
{	


}


void CEnviroInfo::UserPostActivity( 
			const CEnviroInfoParseBlock* pSnoBlock 
			)
{

}


void CEnviroInfo::UserDeletion( 
			const CEnviroInfoParseBlock* pSnoBlock
			)
{

	PrintDeletionMessage();

}
