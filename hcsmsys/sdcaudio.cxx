//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: sdcaudio.cxx,v 1.6 2004/04/01 22:11:53 oahmad Exp $
//
// Author(s):    
//
// Date:         October, 2002
// Description:  Audio generation for sdc virtual environment
//
//////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "sdcaudio.h"

// Audio specific global variables
static const CSol*        m_pSol;
static const CCved*       m_pCved;

static long               m_Tick;
static vector<int>        m_Objs;			// objects available at this run



/////////////////////////////////////////////////////////////////////////////
//
// Description: Initiliaze audio component.
//
// Remarks:
// This function implements the frame-by-frame audio processing in realtime.
//
static void
InternalAudioProc(
	int   tick,						// simulation tick counter
	const vector<int>& objs,		// objects that existed in prior frames
	const vector<int>& newObjs,		// newly created objects
	const vector<int>& delObjs)		// deleted objects
{
	unsigned int i;

	// Object information
	CPoint3D     pos;
	CVector3D    tang;
	int          solid;
	const char*  pName;
	cvEObjType   tp;
	const CSolObj* pSolObj;
	string       solCat;

	int          hornId;
	int          soundId;

	printf("\nFrame %d\n", tick);

	for( i = 0; i < objs.size(); i++ ) 
	{
		int obj = objs[i];

		pName = m_pCved->GetObjName( obj );
		pos   = m_pCved->GetObjPos( obj );
		tang  = m_pCved->GetObjTan( obj );

		printf(
			"   OLD Obj %d,%-20s: (%6.1f, %6.1f, %5.1f), (%4.2f, %4.2f) \n",
			obj, 
			pName, 
			pos.m_x, pos.m_y, pos.m_z, 
			tang.m_i, tang.m_j
			);
	}

	for( i = 0; i < newObjs.size(); i++ ) 
	{
		int obj = newObjs[i];
		const char* pTypeStr;
		string      solName;

		pName    = m_pCved->GetObjName(obj);
		solid    = m_pCved->GetObjSolId(obj);
		tp       = m_pCved->GetObjType(obj);
		pTypeStr = cvObjType2String(tp);
		pSolObj  = m_pSol->GetObj(solid);
		solName  = pSolObj->GetName();
		solCat   = pSolObj->GetCategoryName();

		hornId  = -1;
		soundId = -1;
//		if( !strcmp(solCat.c_str(), "Vehicle") ) {
		if( tp == eCV_VEHICLE )
		{
			const CSolObjVehicle* pO = dynamic_cast<const CSolObjVehicle*>(pSolObj);

			hornId  = pO->GetHornSoundId();
			soundId = pO->GetPassingVehSoundId();
		}

		printf(
			"   NEW Obj %d,%-20s: type %d(%s), sol=%d(%s)\n", 
			obj, 
			pName, 
			tp, 
			pTypeStr,
			solid, 
			solName.c_str()
			);
		printf("                                   SolCat=%s\n", solCat.c_str());
		printf("                                   hornId=%d, soundId=%d\n", hornId, soundId);

	}

	for( i = 0; i < delObjs.size(); i++ ) 
	{
		int obj = delObjs[i];
		pName = m_pCved->GetObjName( obj );
		printf("   DEL Obj %d,%-20s\n", obj, pName);
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Initiliaze audio component.
//
// Remarks:
// This function performs one-time initialiation of any necessary audio 
// components.  It should query the hardware, setup any necessary
// handles, verify that any required libraries and installed and then 
// return;  It should not load sounds or anything else that is 
// run specific.
//
// This function does not have to perform in 'real-time'.
//
// Returns:
// The function returns true to indicate successful initialization, otherwise
// it should return false.  If it returns false, it prints a short message
// to indicate the error
//
bool
InitAudio()
{
	m_Objs.reserve(300);

	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Perform run specific initialization.
//
// Remarks:
// This function performs any necessary initialization before starting a
// "run".  It can interrogate CVED if necessary, and load any sounds in
// anticipation of run-time use.
//
// In case of problems (missing sounds etc.) the function prints a one-time
// error message.
//
// This function does not have to perform in 'real-time'.
//
// Args:
// cved - the instance of the correlated virtual environment database; this
// handle will remain valid until TermAudioForRun is called.
//
void
InitAudioForRun(const CCved*  pC)
{
	m_pCved = pC;
	m_pSol  = &m_pCved->GetSol();

	m_Objs.clear();
	m_Tick = 0;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Implement real-time audio generation.
//
// Remarks:
// This function is called once per simulation iteration.  It is responsible
// for generating scenario specific sounds in real-time.
//
//
void
AudioExec()
{
	CObjTypeMask mask;
	CPoint3D     driverPos;
	double       radius = 1000.0;	// how far around the driver to look

	static vector<int>  temp;		// temporary buffer space
	static vector<int>  newObjs;	// list of newly created objects
	static vector<int>  delObjs;	// deleted objects
	static vector<int>  objs;		// objects remaining active

	if ( !m_pCved->GetOwnVehiclePos( driverPos) ) {
		driverPos.m_x = -11914;
		driverPos.m_y = 2349;
		driverPos.m_z = 0.0;
		radius = 200.0;
//		printf("No driver\n");
//		return;
	}

	m_Tick++;
	mask.SetAll();
	mask.Clear(eCV_TRAFFIC_LIGHT);
	mask.Clear(eCV_TRAFFIC_SIGN);
	mask.Clear(eCV_COORDINATOR);

//	m_pCved->GetObjsNear(driverPos, radius, temp, mask);
	cerr << "@@@@ AudioExec: CVED doesn't support GetObjsNear" << endl;
	assert( 0 );

	unsigned int i, j;

	newObjs.clear();
	delObjs.clear();
	objs.clear();

	for (i=0; i<temp.size(); i++) {
		bool found = false;
		for (j=0; j<m_Objs.size(); j++) {
			if ( temp[i] == m_Objs[j] ) {
				found = true;
				break;
			}
		}
		if ( found ) 
			objs.push_back(temp[i]);
		else
			newObjs.push_back(temp[i]);
	
	}

	for (i=0; i<m_Objs.size(); i++) {
		bool found = false;
		for (j=0; j<temp.size(); j++) {
			if ( m_Objs[i] == temp[j] ) {
				found = true;
				break;
			}
		}
		if ( !found ) delObjs.push_back(m_Objs[i]);
	}

	InternalAudioProc(m_Tick, objs, newObjs, delObjs);

	// debug printing
#if 0
	printf("\n--- Frame %d\n", m_Tick);

	if ( newObjs.size() > 0 ) {
		printf("New     : ");
		for (i=0; i<newObjs.size(); i++) printf("%3d ", newObjs[i]);
		printf("\n");
	}

	if ( delObjs.size() > 0 ) {
		printf("Deleted : ");
		for (i=0; i<delObjs.size(); i++) printf("%3d ", delObjs[i]);
		printf("\n");
	}

	if ( objs.size() > 0 ) {
		printf("Existing: ");
		for (i=0; i<objs.size(); i++) printf("%3d ", objs[i]);
		printf("\n");
	}
#endif

	// preparing for next time
	m_Objs = temp;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Perform run specific close-out activities.
//
// Remarks:
// This function performs any necessary release of resources of handles
// for after termination of a run.  If nothing else, the function ensures
// that the hardware ceases to produce any sounds.  The CVED instance
// provided in InitAudioForRun() will now become invalid.
//
// This function does not have to perform in 'real-time'.
//
//
void 
TermAudioForRun()
{
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Perform process specific close-out activities.
//
// Remarks:
// This function performs any necessary release of resources of handles
// before termination of the process
//
// This function does not have to perform in 'real-time'.
//
//
void
TermAudio()
{
}