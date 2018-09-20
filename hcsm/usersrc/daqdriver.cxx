/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: daqdriver.cxx,v 1.15 2018/03/27 14:48:17 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad, Sunil Bulusu
 *
 * Date:    May, 2001
 *
 * Description:  Contains code for the DaqDriver HCSM.
 *
 ****************************************************************************/

#include "hcsmpch.h"
#include <fstream>
#include "fastdeque.h"

using namespace CVED;

// Debugging macros
#undef DEBUG_DAQ_DRIVER

void 
CDaqDriver::UserCreation( const CDaqDriverParseBlock* cpSnoBlock )
{
	m_dataFreq = cpSnoBlock->GetFreq();
	double behavFreq = 1 / GetTimeStepDuration();
	m_executionMultiplier = m_dataFreq / behavFreq;

	PrintCreationMessage();
	gout << "--- DaqDriver is running at " << m_dataFreq;
	gout << "Hz  multipler = " << m_executionMultiplier << " ---" << endl;

	if( m_executionMultiplier - ((int) m_executionMultiplier) > 0.01 )
	{
		gout << MessagePrefix();
		gout << "** DaqDriver doesn't handle non-whole multiplers **" << endl;
	}

	//
	// Reading the DaqData file name and frequency.
	//
	const string drvrPosFileName = cpSnoBlock->GetDriverPosFileName();
	const string drvrOriFileName = cpSnoBlock->GetDriverOriFileName();
	
	//
	// Get the path to the data directory.
	//
	char pDaqPosFilePath[1024];
	char pDaqOriFilePath[1024];
	if( getenv( "NADSSDC_SCN" ) ) 
	{
		strncpy( pDaqPosFilePath, getenv( "NADSSDC_SCN" ),1024);
		strncpy( pDaqOriFilePath, getenv( "NADSSDC_SCN" ),1024);
	}
	else
	{
		cerr << MessagePrefix() << "undefined system ";
		cerr << "variable 'NADSSDC_SCN'" << endl;
	}
	
	strcat( pDaqPosFilePath, drvrPosFileName.c_str() );
	strcat( pDaqOriFilePath, drvrOriFileName.c_str() );

	//
	// Opening the Daq data files.
	//
	ifstream daqPosFile, daqOriFile;
	daqPosFile.open( pDaqPosFilePath );
	daqOriFile.open( pDaqOriFilePath );

	//
	// Make sure that the files opened properly.
	//
	bool fileOpenError = !daqPosFile || !daqOriFile;
	if( fileOpenError )
	{
		cerr << MessagePrefix() << "unable to open '";

		if( !daqPosFile ) 
		{
			cerr << pDaqPosFilePath << "'" << endl;
		}
		else if( !daqOriFile) {
			cerr << pDaqOriFilePath << "'" << endl;
		}

		Suicide();
		return;
	}

	// DaqData structure
	TDaqData daqData;
	CPoint3D drvObjPos;
	CVector3D drvObjOri; // roll.i + pitch.j + yaw.k
	double drvFrame;
	char ignore;
	char ignorePosLine[1024];
	char ignoreOriLine[1024];
	
		
	// 
	// Reading values from the Daq Data files into daqData structure
	// and storing them into the fastdeque container.
	//
	while( daqPosFile && daqOriFile )
	{
		if( daqPosFile.get() == '#' && daqOriFile.get() == '#' )
		{
			// comment...ignore
			daqPosFile.getline( ignorePosLine, 1024 );
			daqOriFile.getline( ignoreOriLine, 1024 );

			continue;
		}
		else
		{
			daqPosFile >> drvFrame >> drvObjPos.m_y >> drvObjPos.m_x >> drvObjPos.m_z;
			daqOriFile >> drvFrame >> drvObjOri.m_j >> drvObjOri.m_i >> drvObjOri.m_k;

			// set the values in the daqData structure
			daqData.objPosition = drvObjPos; 
			daqData.objOrientation = drvObjOri;
			daqData.signal = 0; 
			daqData.brake = 0;
			
			// push the value into Deque
			m_DaqDataContainer.push_back(daqData);
		}
	}

	cved->SetFakeExternalDriver( false );
	
	//
	// Create the DriverMirror HCSM.
	//
	int hcsmId = -1;
	CHcsm* pDmHcsm = m_pRootCollection->GetHcsm( "DriverMirror" );
	if(pDmHcsm != 0 )
	{
		hcsmId = m_pRootCollection->GetHcsmId( pDmHcsm );
	}
	
	// Initial cart position
	TDaqData startData = m_DaqDataContainer[m_DaqDataContainer.begin()];
	CPoint3D cartPos = startData.objPosition;
	CVector3D cartOri = startData.objOrientation;

	// Calculating the Tangent and Lateral vectors for the object
	// using values from the DaqDataC Container
	double roll  = cartOri.m_i;
	double pitch = cartOri.m_j;
	double yaw   = cartOri.m_k;

	CVector3D initTan; 
	initTan.m_i = cos(yaw) * cos(pitch);
    initTan.m_j = -sin(yaw) * cos(roll) - cos(yaw) * sin(pitch) * sin(roll);
    initTan.m_k = sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll);

	CVector3D initLat;
	initLat.m_i= sin(yaw) * cos(pitch);
    initLat.m_j = cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll);
    initLat.m_k = -cos(yaw) * sin(roll) - sin(yaw) * sin(pitch) * cos(roll);

	// 
	// Set attributes and crate a CVED object to represent the driver.
	//
	const string solName = "ChevyBlazerRed";
	const CSolObj* cpSolObj = cved->GetSol().GetObj( solName );
	cvTObjAttr attr = { 0 };
	attr.solId = cpSolObj->GetId();
	attr.xSize = cpSolObj->GetLength();
	attr.ySize = cpSolObj->GetWidth();
	attr.zSize = cpSolObj->GetHeight();

	cvEObjType externalObjType = eCV_EXTERNAL_DRIVER;
	m_pObj = cved->CreateDynObj(
							"ExternalDriver",
							externalObjType,
							hcsmId,
							attr,
							&cartPos,
							&initTan,
							&initLat
							);
	bool invalidObj = !m_pObj || !m_pObj->IsValid();
	if( invalidObj )
	{
		// unable to create CVED object...suicide
		cerr << MessagePrefix();
		cerr << "object (name=" << "ExternalDriver";
		cerr << ") unable to create self in CVED  [SUICIDE]" << endl;
		
		Suicide();
	}

#ifdef DEBUG_DAQ_DRIVER
	gout << "=== DAQ DRIVER - CREATION =================" << endl;
	gout << "PosFile: " << drvrPosFileName << endl;
	gout << "OriFile: " << drvrOriFileName << endl;
	gout << "==OBJECT DATA==" << endl;
	gout << "initPos: " << cartPos << endl;
	gout << "initOri: " << cartOri << endl;
	gout << "tangential: " << initTan << endl;
	gout << "lateral: " << initLat << endl << endl << endl;
#endif

}  // end of UserCreation


void 
CDaqDriver::UserPreActivity( const CDaqDriverParseBlock* cpSnoBlock )
{
		
#ifdef DEBUG_DAQ_DRIVER
	gout << MessagePrefix();
	gout << "=== DAQ DRIVER - PRE_ACTIVITY =================" << endl;
#endif
}  // end of UserPreActivity


void
CDaqDriver::ProcessOneDaqFrame()
{
	if( m_DaqDataContainer.size() > 1 )
	{
		m_DaqDataContainer.pop_front();
		TDaqData nextData = m_DaqDataContainer[ m_DaqDataContainer.begin() ];
		CPoint3D currPos = nextData.objPosition;
		CVector3D currOri = nextData.objOrientation;
		
		// Calculating the Tangent and Lateral vectors for the object
		// using values from the DaqData Container
		double roll = currOri.m_i;
		double pitch =  currOri.m_j;
		double yaw =  currOri.m_k;

		CVector3D currTan; 
		currTan.m_i = cos(yaw) * cos(pitch);
		currTan.m_j = -sin(yaw) * cos(roll) - cos(yaw) * sin(pitch) * sin(roll);
		currTan.m_k = sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll);

		CVector3D currLat;
		currLat.m_i= sin(yaw) * cos(pitch);
		currLat.m_j = cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll);
		currLat.m_k = -cos(yaw) * sin(roll) - sin(yaw) * sin(pitch) * cos(roll);


		CExternalDriverObj* pExternalDriverObj = dynamic_cast<CExternalDriverObj *>(m_pObj);
		// Setting the position, tan and lat vectors
		pExternalDriverObj->SetPos( currPos );
		pExternalDriverObj->SetTan( currTan );
		pExternalDriverObj->SetLat( currLat );
	}
	else
	{
		Suicide();
	}
}


void 
CDaqDriver::UserPostActivity( const CDaqDriverParseBlock* cpSnoBlock )
{
	//
	// Execute n frames of daq data.
	//
	bool executeMoreThanOncePerFrame = m_executionMultiplier > 1;
	if( executeMoreThanOncePerFrame )
	{
		int executeCount = (int) m_executionMultiplier;
		int i;
		for( i = 0; i < executeCount; i++ )
		{
			ProcessOneDaqFrame();
		}
	}
	else
	{
		m_framesSinceLastExecution++;
		if( m_framesSinceLastExecution >= 1 / m_executionMultiplier )
		{
			ProcessOneDaqFrame();
			m_framesSinceLastExecution = 0;
		}
	}

#ifdef DEBUG_DAQ_DRIVER
	gout << MessagePrefix();
	gout << "=== DAQ DRIVER - POST_ACTIVITY =================" << endl;
	gout << "Drv Pos: " << m_pObj->GetPos() << endl;
	gout << "Drv Vel: " << m_pObj->GetVel() << endl;
#endif
	
}  // end of UserPostActivity


void CDaqDriver::UserDeletion( const CDaqDriverParseBlock* cpSnoBlock )
{
	//
	// Print deletion message to output.
	//
	PrintDeletionMessage();

	//
	// Check to make sure that the CVED object is still valid
	// before deleting it.
	//
	bool externalObjValid = m_pObj && m_pObj->IsValid();
	if( externalObjValid ) 
	{
		cved->DeleteDynObj( m_pObj );
	}
	cved->SetFakeExternalDriver( true );

}  // end of UserDeletion
