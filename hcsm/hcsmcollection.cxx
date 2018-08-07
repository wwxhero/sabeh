//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmcollection.cxx,v 1.166 2016/10/28 20:50:44 IOWA\dheitbri Exp $
// Author(s):    Omar Ahmad
// Date:         July, 1998
//
// Description:  Implemention of the CHcsmCollection class.
//
//////////////////////////////////////////////////////////////////////////////

#include <direct.h>
#include "EnvVar.h"


#include <genericinclude.h>
#include <string>
#include<boost/tokenizer.hpp>
#include <random>

using namespace std;

#include <cvedpub.h>

#include <snoparse.h>
#include "hcsmcollection.h"

const int cMAP_ERASE_ERROR = 0;

// MemLog constants

#define HLOG_CREATE_START    10300
#define HLOG_CREATE_2        10301
#define HLOG_CREATE_3        10302
#define HLOG_CREATE_4        10303
#define HLOG_CREATE_5        10304
#define HLOG_CREATE_6        10305
#define HLOG_CREATE_7        10306
#define HLOG_CREATE_END      10307

#define HLOG_PROCCREAT_START 10320
#define HLOG_PROCCREAT_DOONE 10321
#define HLOG_PROCCREAT_END   10322

#define HLOG_DELHCSM_START   10330
#define HLOG_DELHCSM_END     10331

#define HLOG_DELALLHCSM_START 10350
#define HLOG_DELALLHCSM_1     10351
#define HLOG_DELALLHCSM_END   10352

#define HLOG_PROCDEL_START   10360
#define HLOG_PROCDEL_END     10361

#define HLOG_EXECALL_START   10370
#define HLOG_EXECALL_DOONE_1 10371
#define HLOG_EXECALL_DOONE_2 10372
#define HLOG_EXECALL_END     10373

#define HLOG_DTOR_1          10500
#define HLOG_DTOR_2          10501

bool  CHcsmCollection::m_verbose;
char  CHcsmCollection::m_sObjBtnDialValToHcsm[cOBJ_BTNDIAL_SIZE];
char  CHcsmCollection::m_sExperiment[128];
char  CHcsmCollection::m_sSubject[128];
char  CHcsmCollection::m_sRun[128];
char  CHcsmCollection::m_sRunInst[128];
bool  CHcsmCollection::m_logActivities = false;;
bool  CHcsmCollection::m_printActvLog;
float CHcsmCollection::m_sSCC_Scen_Pos_X_Crossbeam;
float CHcsmCollection::m_sSCC_Scen_Pos_Y_Carriage;
float CHcsmCollection::m_sSCC_Scen_Pos_Hex_X;
float CHcsmCollection::m_sSCC_Scen_Pos_Hex_Y;
float CHcsmCollection::m_sSCC_Scen_Pos_Hex_Z;
float CHcsmCollection::m_sSCC_Scen_Pos_Hex_Roll;
float CHcsmCollection::m_sSCC_Scen_Pos_Hex_Pitch;
float CHcsmCollection::m_sSCC_Scen_Pos_Hex_Yaw;
float CHcsmCollection::m_sSCC_Scen_Pos_TT;
float CHcsmCollection::m_sAccelPedalPos;
float CHcsmCollection::m_sBrakePedalForce;
float CHcsmCollection::m_sSteeringWheelAngle;
short CHcsmCollection::m_sCruiseControlIncoming;
short CHcsmCollection::m_sCruiseControl;
int   CHcsmCollection::m_sCisTurnSignal;
float CHcsmCollection::m_sSpeedometerBackdrive;
int   CHcsmCollection::m_sHorn;
int   CHcsmCollection::m_sHornFiltered;

int   CHcsmCollection::m_sRecasButton;
float CHcsmCollection::m_sAuxiliaryButtons[20];
int   CHcsmCollection::m_sBrakeCond[cBRAKE_COND_SIZE];
int   CHcsmCollection::m_sTireCond[cTIRE_COND_SIZE];
int   CHcsmCollection::m_sSteeringCond;
int   CHcsmCollection::m_sAlertCond;
int   CHcsmCollection::m_sInfoCond;
int   CHcsmCollection::m_sCabComponentCond;
int   CHcsmCollection::m_sSCC_Scenario_Stop_Ind;
int   CHcsmCollection::m_sSCC_PlacePhoneCall;	// cell value
int   CHcsmCollection::m_sPlacePhoneCallAge;	// frame on which cell value was set
float CHcsmCollection::m_sLogStreams[cNUM_LOG_STREAMS];
float CHcsmCollection::m_sLogStreamsExt[cNUM_LOG_STREAMS];
char  CHcsmCollection::m_sSCC_DataRed_Params[cNUM_DATARED_PARAMS_SIZE];
int   CHcsmCollection::m_sSCC_DataRed_Segments[cNUM_DATARED_SEGMENTS_SIZE];
float CHcsmCollection::m_sFollowInfo[cFOLLOW_INFO_SIZE];
float CHcsmCollection::m_sSensorInfo[cSENSOR_INFO_SIZE];

TDynObjData  CHcsmCollection::m_sDynObjData;
int          CHcsmCollection::m_sDynObjDataSize;
TStatObjData CHcsmCollection::m_sStatObjData;
int          CHcsmCollection::m_sStatObjDataSize = 0;

int CHcsmCollection::m_sHeadlightVisualState = 0; //<are the headlights on or off? matches SCC_HeadlightVisualState
int CHcsmCollection::m_sHeadlightScenarioControl = 0; //<has the scenario asked to set the headlight state

int   CHcsmCollection::m_sChangedStatObjOption[cMAX_CHANGED_STAT_OBJ];
int   CHcsmCollection::m_sChangedStatObjId[cMAX_CHANGED_STAT_OBJ];
short CHcsmCollection::m_sChangedStatObjDataSize = 0;
const char cDefault[] = "Default";

map<string, double>                          CHcsmCollection::m_exprVariables;
map<string, shared_ptr<mt19937>>             CHcsmCollection::m_randomGenerators;
map<string, CPoint3D>                        CHcsmCollection::m_exprPosVariables;
map<string, vector< pair<string,double> > >  CHcsmCollection::m_varQueues;

map<int, vector<float> >    CHcsmCollection::m_sDiGuyPathInfo; //<path for the DiGuys
map<int, vector<float> >    CHcsmCollection::m_sDiGuyPathTimesInfo; //<optional time at node for DiGuys
map<int, vector<char> >     CHcsmCollection::m_sDiGuyPathActionInfo; //<optional action at node for DiGuys

vector<CDiGuyUpdateCommand> CHcsmCollection::m_sDiGuyCommandQueue;

CActvLog         CHcsmCollection::m_sActvLog;
int              CHcsmCollection::m_frame;
map<CHcsm*, int> CHcsmCollection::m_hcsmMap;
TWriteCellData   CHcsmCollection::m_sWriteCellData[cMAX_WRITE_CELL_DATA_SIZE];
map<string,int>  CHcsmCollection::m_sAdditionalCellNamesToIds;
int              CHcsmCollection::m_sWriteCellDataSize;

TScenarioWriteCellData  CHcsmCollection::m_sScenarioWriteCellData[cMAX_SCENARIO_WRITE_CELL_EVENTS];
int                     CHcsmCollection::m_sScenarioWriteCellDataSize = 0;
TScenarioWriteCellData  CHcsmCollection::m_sScenarioWriteUniformData[cMAX_SET_UNIFORM_CELL_EVENTS];
int                     CHcsmCollection::m_sScenarioWriteUniformDataSize = 0;
TCabVisualOperations    CHcsmCollection::m_sCabOperations[cMAX_VISUAL_OPERATIONS];
int                     CHcsmCollection::m_CabOperationsSize = 0;
TAttachShaderCmd        CHcsmCollection::m_sAttachShaderCmds[cMAX_SET_UNIFORM_CELL_EVENTS];
int                     CHcsmCollection::m_sAttachShaderCmdsDataSize;
TSetSwitchCmd           CHcsmCollection::m_sSetSwitchCmds[cMAX_SCENARIO_SET_SWITCH];
int                     CHcsmCollection::m_sSetSwitchCmdsDataSize =0;
CHcsmStaticLock         CHcsmCollection::m_sLockVisualOptions;


float           CHcsmCollection::m_sFalseAlarmInfo[cFALSE_ALARM_SIZE];
map<int,string> CHcsmCollection::m_sVisualDisplayText;
string          CHcsmCollection::m_sPlayAudioText;
string          CHcsmCollection::m_sVisualSettings;
float           CHcsmCollection::m_sLaneDevInfo[cLANE_DEV_INFO_SIZE];
float           CHcsmCollection::m_sSplineDevInfo[cLANE_DEV_INFO_SIZE];
float           CHcsmCollection::m_sLaneDepartWarn[cLANE_DEPART_WARN_SIZE];
short           CHcsmCollection::m_sLaneMarkingInfo[cLANE_MARKING_INFO_SIZE];
float           CHcsmCollection::m_sOwnVehCurvature;

int  CHcsmCollection::m_sHour = 12;
int  CHcsmCollection::m_sMinute = 0;
bool CHcsmCollection::m_sChangeTimeOfDay = false;

bool CHcsmCollection::m_sDisableCurvature = false;

#ifdef TTA_DIST_FOR_ODSS //HACK for demonstration
double CHcsmCollection::m_sDistanceToInt = 0;
#endif

int   CHcsmCollection::m_sWarningLights;
int   CHcsmCollection::m_sCollWarnAlg;
float CHcsmCollection::m_sFcwInfo[4];
short CHcsmCollection::m_sTimeToWarn;


bool CHcsmCollection::m_sBUD_IsOn = false;
bool CHcsmCollection::m_sLCW_IsOn = false;
bool CHcsmCollection::m_sBLS_IsOn = false;
bool CHcsmCollection::m_sFCW_IsOn = false;

bool CHcsmCollection::m_sIsOnPath = false;
CVED::CPath CHcsmCollection::m_sOwnshipPath;

ELdwStatus CHcsmCollection::m_sLdwStatus = eLDW_NONE;
EBswStatus CHcsmCollection::m_sBswStatus = eBSW_NONE;
EFcwStatus CHcsmCollection::m_sFcwStatus = eFCW_NONE;
float CHcsmCollection::m_sBackUpDistance = 35.0; //< current back up distance in feet

bool                                   CHcsmCollection::m_sHapticSeat_IsEnabled = 0; //< whether haptic seat is enabled
CHcsmCollection::eFlashingLightMode    CHcsmCollection::m_sFlashingLightMode = CHcsmCollection::eNO_LIGHTS; //< flashing lights
CHcsmCollection::eBUDCameraDisplayMode CHcsmCollection::m_sBUDCameraDisplayMode = CHcsmCollection::eNO_DISPLAY; //< backup camera display and distance bar

float CHcsmCollection::m_sBUDBarWarnLowDist = 30.0;  //< the backup distance in feet at which the distance bar starts to grow
float CHcsmCollection::m_sBUDBarWarnHighDist = 1.0; //< the backup distance at which the distance bar reaches maximum warning level
bool  CHcsmCollection::m_sBSWCamera_IsEnabled  = 0; //< whether blind spot warning camera is enabled
bool  CHcsmCollection::m_sIPAlert_IsEnabled	 = 0; //< whether alert icons in instrument panel is enabled

HANDLE          CHcsmCollection::m_sLastGoodLocationMutex = NULL; //< Mutex Loc for last good location
float           CHcsmCollection::m_sLastGoodLocation[3]; //< last location the Ext Driver was on the Path
CHcsmStaticLock CHcsmCollection::m_cabSettingsCriticalSection; //< Critical Section for Cab Settings
CHcsmStaticLock CHcsmCollection::m_DiGuyPathCriticalSection; //< Critical Section for Cab Settings

#ifdef AUDIO_TRIGGER_BYPASS
int CHcsmCollection::m_sAudio_Trigger = 0;
short CHcsmCollection::m_sACC_On = 0;
#endif
short CHcsmCollection::m_sSensor_Config[cSENSOR_CONFIG_SIZE] = {400,16,1};

#if USING_RTEXPSLITE == 0
RTEX::SubsysBase* CHcsmCollection::m_spRTEX = NULL;
#endif

float CHcsmCollection::m_sSirenEffect = 0;
float CHcsmCollection::m_sSirenSpeed = -1;

short CHcsmCollection::m_sACC_Warning = 0;
short CHcsmCollection::m_sCruise_State= 0;
float CHcsmCollection::m_sCruise_SetSpeed = 0;
float CHcsmCollection::m_sACC_Gap = 0;
short CHcsmCollection::m_sALF_State = 0;
int   CHcsmCollection::m_sLDW_Severity = 0;
int   CHcsmCollection::m_sFCW_Severity = 0;

COnScreenGraph CHcsmCollection::m_sDisplayGraph;
bool CHcsmCollection::m_sGraphIsOn = false;

float CHcsmCollection::m_sOwnVehDistOnPath = 0;
double CHcsmCollection::m_sSmoothForwardRoadVector[3] = {1.0,0,0};
TReadCellFuncptr CHcsmCollection::ReadCellNumeric = NULL;


bool  CHcsmCollection::m_ExternalDriverRehearsalControl = false;
bool  CHcsmCollection::m_sMakeOwnVehicleChangeLanesLeft = false;
bool  CHcsmCollection::m_sMakeOwnVehicleChangeLanesRight = false;
float CHcsmCollection::m_sOwnVehicleSpeed = 55;

//bool (CHcsmCollection::*ReadCellNumeric)(const string&, int, float&) = NULL;
//////////////////////////////////////////////////////////////////////////////
/// Construction/Destruction
///\todo replace sscanf
//////////////////////////////////////////////////////////////////////////////

CHcsmCollection::CHcsmCollection(
			const double timeStepDuration,
			CCved* pCved
			)
{
	m_numHcsm = 0;
	m_ownDriverSurrogate = nullptr;
	//
	// Initialize the root HCSM instance array and create linked
	// list of free/blank entries in the root HCSM array.
	//
	int i;
	for( i = 0; i < cMAX_ROOT_HCSM; i++ )
	{
		m_hcsmInstances[i] = NULL;
		m_freeList.push_front(i);
	}
	m_hcsmMap.clear();
	//
	// The starting frame is 1 because the frame counter gets
	// incremented at the end of every call to ExecuteAllHcsm.
	//
	m_frame = 1;
	m_timeStepDuration = timeStepDuration;
	m_pCved = pCved;
//	int slots = m_memLog.CreateMem( 64 * 1024 );
//	gout << "** Creating memory log with " << slots << " slots" << endl;
	bool result = m_memLog.Init();
	if( !result )
	{
		cerr << "CHcsmCollection::memory log is currently disabled" << endl;
	}
	m_memLog.ClearLog();

	string strEx;
    NADS::GetEnvVar(strEx,"MEMLOGEXCLUDE" );
	if( strEx.size() > 0 )
	{
		int n1, n2;
		stringstream ss;
        ss<<strEx;
        ss>>n1>>n2;
        if( ss.fail() )
		{
			m_memLog.Exclude( n1, n2 );
			fprintf(stderr, "** Note: memlog excluding range %d-%d\n", n1, n2);
		}
		else
		{
			fprintf(stderr, " ** Warning: MEMLOGEXCLUDE set but incorrectly\n");
		}
	}

	//
	// Activity event logs.
	//
	m_sActvLog.Init();

	//
	// Hcsms can use this instance of the rng for any purpose they
	// deem necessary.
	//
	m_rng.SetAllSeeds( 2, 1 );

	// Log streams.
	for( i = 0; i < cNUM_LOG_STREAMS; i++ )
	{
		CHcsmCollection::m_sLogStreams[i] = 0.0;
	}
	for( i = 0; i < cNUM_LOG_STREAMS; i++ )
	{
		CHcsmCollection::m_sLogStreamsExt[i] = 0.0;
	}

	// Initialize data reduction segments and parameters.
	for( i = 0; i < cNUM_DATARED_SEGMENTS_SIZE; i++ )
	{
		CHcsmCollection::m_sSCC_DataRed_Segments[i] = 0;
	}
	memset(
		CHcsmCollection::m_sSCC_DataRed_Params,
		0,
		cNUM_DATARED_PARAMS_SIZE
		);

	// Initialize brake and tire conditions.
	int cnt;
	for( cnt = 0; cnt < cBRAKE_COND_SIZE; cnt++ )
	{
		CHcsmCollection::m_sBrakeCond[cnt] = 0;
	}
	for( cnt = 0; cnt < cTIRE_COND_SIZE; cnt++ )
	{
		CHcsmCollection::m_sTireCond[cnt] = 0;
	}

	//
	// Dynamic object data.
	//
	CHcsmCollection::m_sDynObjDataSize = 0;

	//
	// Follow info.
	//
	m_sFollowInfo[0] = -1.0f; // no object in front
	for( i = 1; i < cFOLLOW_INFO_SIZE; i++ )
	{
		m_sFollowInfo[i] = 0.0f;
	}

	//
	// Sensor info.
	//
	m_sSensorInfo[0] = -1.0f; // no object in front
	for( i = 1; i < cSENSOR_INFO_SIZE; i++ )
	{
		m_sSensorInfo[i] = 0.0f;
	}

	m_sWriteCellDataSize = 0;
	for( i = 0; i < cMAX_WRITE_CELL_DATA_SIZE; i++ )
	{
		m_sWriteCellData[i].elemId = -1;
	}

	for( i = 0; i < cFALSE_ALARM_SIZE; i++ )
	{
		m_sFalseAlarmInfo[i] = 0.0f;
	}

	//
	// Preload files.
	//
	m_preloadHandleCounter = 0;

	//
	// Clear out expression variables.
	//
	m_exprVariables.clear();
    m_randomGenerators.clear();
    m_randomGenerators[cDefault] = shared_ptr<mt19937>(new std::mt19937(std::random_device()));
	m_exprPosVariables.clear();

	m_sVisualDisplayText.clear();
	m_sPlayAudioText.clear();
	m_sVisualSettings.clear();

	//
	// Initialize the static member variables.
	//
	for( i = 0; i < cLANE_DEV_INFO_SIZE; i++ )
	{
		m_sLaneDevInfo[i] = 0.0f;
	}

	for( i = 0; i < cLANE_DEPART_WARN_SIZE; i++ )
	{
		m_sLaneDepartWarn[i] = 0.0f;
	}

	m_sOwnVehCurvature = 0.0f;
	m_sSCC_Scenario_Stop_Ind = 0;

	if (m_sLastGoodLocationMutex == NULL){ //this is a static
		m_sLastGoodLocationMutex  = CreateMutex(NULL,FALSE,"LastLocationMUTEX");
	}
	m_sOwnshipPath.SetCved( m_pCved );
}

CHcsmCollection::~CHcsmCollection()
{

	MemLog( HLOG_DTOR_1, 0, "collection dtor 1" );

	// cleanup all Hcsms
	DeleteAllHcsm();

	MemLog( HLOG_DTOR_2, 0, "collection dtor 2" );

	m_memLog.DumpToFile( "memlog.txt" );
	FILE* pF;
#if (_MSC_VER > 1500)
    #pragma warning( push )
    #pragma warning(disable:4996)
#endif
	pF = fopen( "memlog.formatted.txt", "w" );
#if (_MSC_VER > 1500)
    #pragma warning( pop )
#endif
	if( pF == 0 )
	{
		cerr << "** unable to open memory log file" << endl;
	}
	else
	{
		m_memLog.FormatedFileDump( pF, HLOG_EXECALL_START, false );
		fclose( pF );
	}


	if( m_numHcsm != 0 )
	{
		cerr << MyName() << "::~CHcsmCollection: Hcsms not cleaned up!!";
		cerr << endl;
	}

	//
	// Write activity log to a file.
	//
	if( m_logActivities )
	{
		//
		// Build the activity log file name.  It should be written to
		// bin directory(where other logs exist) and the name should be
		// composed of the run instance name and a ".txt" suffix.
		//
		string actvLogFileName = GetActvLogFileName();
#if 0
		if( getenv( "NADSSDC_BIN" ) )
		{
			actvLogFileName = getenv( "NADSSDC_BIN" );
			actvLogFileName += "\\";
		}

		if( strlen( m_sRunInst ) > 0 )
		{
			actvLogFileName += "actvlog_";
			actvLogFileName += m_sRunInst;
		}
		else
		{
			actvLogFileName += "actvlog";
		}
		actvLogFileName += ".txt";
#endif
		gout << "*** Writing behaviors activity log to " << actvLogFileName;
		gout << endl;
		m_sActvLog.Store( actvLogFileName );
	}
}

//////////////////////////////////////////////////////////////////////////////
// Query Functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Create, Delete and Execute
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Creates an HCSM instance given its template's name.
//
// Remarks:  This function takes as input an HCSM template's name.  It checks
//   to make sure that a template exists with the given name and, if so, then
//   creates an instance of the HCSM template.  A Hcsm will not be active in
//   the frame that it is created.  It becomes active at end of the current
//   frame and is executed for the first time in the following frame.
//
// Arguments:
//   cHcsmName - The HCSM template's name.
//   snoBlock - The HCSM initialization information from the parser.
//
// Returns:  This function returns a pointer to the root HCSM.  If returns a
//   NULL pointer if there is an error.
//
//////////////////////////////////////////////////////////////////////////////
CHcsm* CHcsmCollection::CreateHcsm(
			const string cHcsmName,
			const CSnoBlock& cSnoBlock
			)
{

	CHcsm* pHcsm = NULL;

	MemLog( 0, HLOG_CREATE_START, 0 );

	//
	// Try to create an instance of the class that represents
	// the HCSM template.
	//
	try
	{
		// find the type that matches the hcsm name string
#if defined (EDO_CONTROLLER)
		if (cHcsmName != "Ado"
			|| cSnoBlock.GetName() == "ExternalDriver")
			pHcsm = GetClassFromTemplateName( cHcsmName, cSnoBlock );
#else
		pHcsm = GetClassFromTemplateName( cHcsmName, cSnoBlock );
#endif
		MemLog( 0, HLOG_CREATE_2, 0 );
	}
	catch ( cvCInternalError s )
	{
		// caught CVED exception
		s.Notify();
		gout << MyName() << ": caught CVED cvCInternalError while creating ";
		gout << cHcsmName << " HCSM" << endl;
		return NULL;
	}
	catch( cvCError s )
	{
		// caught CVED exception
		MemLog( 0, HLOG_CREATE_3, 0 );
		s.Notify();
		gout << MyName() << ": caught CVED exception while creating ";
		gout << cHcsmName << " HCSM.  Creation aborted." << endl;
		return NULL;
	}
	catch( CSnoBlock::TCountError e )
	{
		// caught snoparser exception
		MemLog( 0, HLOG_CREATE_4, 0 );
		gout << MyName() << ": caught CSnoBlock field count exception while creating ";
		gout << cHcsmName << " HCSM.  Creation aborted." << endl;
		gout << "  " << e.msg << endl;
		return NULL;
	}
	catch( ... )
	{
		// caught unknown exception
		MemLog( 0, HLOG_CREATE_5, 0 );
		gout << MyName() << ": caught unknown exception while creating ";
		gout << cHcsmName << " HCSM.  Creation aborted." << endl;
		return NULL;
	}

	if( !pHcsm )
	{
		MemLog( 0, HLOG_CREATE_6, 0 );
		// invalid Hcsm name
		gout << MyName() << "::CreateHcsm: Unknown Hcsm Name!" << endl;
		return NULL;
	}

	// first check to see if there are any free elements left
	if( m_freeList.size() <= 0 )
	{
		// root array is full!
		MemLog( 0, HLOG_CREATE_7, 0 );
		gout << MyName() << "::CreateHcsm: Out of room in root array!";
		gout << endl;
		return NULL;
	}

	// get an index to a free element from the back of the list
	int newIndex = m_freeList.back();

	m_hcsmInstances[newIndex] = pHcsm;
	m_numHcsm++;

	// insert hcsm root array index into map
	m_hcsmMap[pHcsm] = newIndex;

	// remove element from free list
	m_freeList.pop_back();

	// insert Hcsm into creation set
	m_hcsmToCreate.insert( pHcsm );

	MemLog( 0, HLOG_CREATE_END, 0 );

	return pHcsm;
}  // CreateHcsm

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Processes all pending requests to activate newly created
//   Hcsms.
//
// Remarks:  This function processes all pending requests to activate newly
//   created Hcsms.  The collection class calls this function at the end of
//   each frame to ensure consistency.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::ProcessHcsmCreate()
{
	MemLog( 0, HLOG_PROCCREAT_START, 0 );

	// THIS CHECK IS HERE BECAUSE THE CONCURRENT WILL CHOKE WITHOUT
	// IT.  IT SHOULD NOT BE PERMANENT.
	if( m_hcsmToCreate.size() > 0 )
	{
		set<CHcsm*>::iterator i;
		for( i = m_hcsmToCreate.begin(); i != m_hcsmToCreate.end(); ++i )
		{
			//
			// Activate the root HCSM and all its dependents.
			//
			CHcsm* pHcsm = *i;
			pHcsm->SetStateTree( eACTIVE );

			//
			// Call the creation function.
			//
			MemLog( 0, HLOG_PROCCREAT_DOONE, (char*) pHcsm->GetName().c_str() );
			pHcsm->CallCreation();
		}

		m_hcsmToCreate.clear();
	}

	MemLog( 0, HLOG_PROCCREAT_END, 0 );
}  // ProcessHcsmCreate

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Deletes a top-level or root HCSM instance given a pointer
//   to the instance.
//
// Remarks:  This function takes as input a pointer to a top-level or root
//   HCSM instance.  If the pointer is invalid or points to an invalid Hcsm,
//   it immediately returns false.  Otherwise, it queue's the Hcsm for
//   deletion at the end of the current frame and updates its data structures
//   to reflect that there is an extra open slot.
//
// Arguments:
//   pHcsm - A pointer to the HCSM instance.
//
// Returns:  This function returns true if it is able to successfully
//   queue the root HCSM, pointed to by the input pointer, for deletion.
//   It returns false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool CHcsmCollection::DeleteHcsm( CHcsm* pHcsm )
{
	MemLog( 0, HLOG_DELHCSM_START, (char *)pHcsm->GetName().c_str() );

	// is this a root Hcsm?
	if( !pHcsm->IsRoot() )
	{
		// input is not a root Hcsm
		gout << MyName() << "::DeleteHcsm: Hcsm is not a root Hcsm!" << endl;
		MemLog( -1, HLOG_DELHCSM_END, 0 );
		return false;
	}

	//
	// Find the entry in the map.
	//
	map<CHcsm*, int>::iterator mapIterator;
	mapIterator = m_hcsmMap.find( pHcsm );
	if( mapIterator == m_hcsmMap.end() )
	{
		// cannot find Hcsm in mapvc
		gout << MyName() << "::DeleteHcsm: Hcsm not found!" << endl;
		MemLog( -2, HLOG_DELHCSM_END, 0 );
		return false;
	}

	//
	// Add delete request to delete list.
	//

	m_hcsmToDelete.insert( pHcsm );

	//
	// Set hcsm's state to DYING....does this really need to be done?
	//
	pHcsm->SetStateTree( eDYING );

	MemLog( 0, HLOG_DELHCSM_END, 0 );

	return true;
}  // DeleteHcsm


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Deletes all top-level HCSMs.
//
// Remarks:  This function deletes all top-level HCSMs and their
//   descendents from the system.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::DeleteAllHcsm()
{

	MemLog( 0, HLOG_DELALLHCSM_START, 0 );
	//
	// Cleanup all HCSMs.
	//
	int i;
	for( i = 0; i < cMAX_ROOT_HCSM; i++ )
	{
		if( m_hcsmInstances[i] )  DeleteHcsm( m_hcsmInstances[i] );
	}
	//m_hcsmMap.clear();
	MemLog( 0, HLOG_DELALLHCSM_1, 0 );

	ProcessHcsmDelete();

	if( m_numHcsm != 0 )
	{
		gout << MyName() << "::DeleteAllHcsm: Hcsms not cleaned up!!";
		gout << endl;
	}

	MemLog( 0, HLOG_DELALLHCSM_END, 0 );
}  // DeleteAllHcsm


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Processes all pending requests to delete Hcsms.
//
// Remarks:  This function processes all pending requests to delete Hcsms.
//   The collection class calls this function at the end of each frame
//   to ensure consistency.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::ProcessHcsmDelete()
{
	MemLog( 0, HLOG_PROCDEL_START, 0 );

	//
	// Look the set that keeps track of pending Hcsm delete requests.
	//
	// THIS CHECK IS HERE BECAUSE THE CONCURRENT WILL CHOKE WITHOUT
	// IT.  IT SHOULD NOT BE PERMANENT.
	//
	if( m_hcsmToDelete.size() > 0 )
	{

		set<CHcsm*>::iterator i;
		for( i = m_hcsmToDelete.begin(); i != m_hcsmToDelete.end(); ++i )
		{
			// get the pointer to the Hcsm to delete
			CHcsm* pHcsm = *i;

			// find the entry in the map
			map<CHcsm*, int>::iterator mapIterator;
			mapIterator = m_hcsmMap.find( pHcsm );
			if( mapIterator == m_hcsmMap.end() )
			{
				// cannot find Hcsm in map
				gout << MyName() << "::ProcessHcsmDelete: Hcsm not found!";
				gout << endl;
				continue;
			}

			// add element to the start of the free list
			m_freeList.push_front( mapIterator->second );

			// update instance list
			m_hcsmInstances[mapIterator->second] = NULL;
			m_numHcsm--;

			// remove the entry from the map
			// return value: 0 --> error, 1 --> ok
			if( m_hcsmMap.erase( mapIterator->first ) == cMAP_ERASE_ERROR )
			{
				// cannot erase Hcsm from map
				gout << MyName() << "::ProcessHcsmDelete: Hcsm erase failed!";
				gout << endl;
				continue;
			}

			string hcsmName = pHcsm->GetName();
			try
			{
				delete pHcsm;
			}
			catch ( cvCInternalError s )
			{
				// caught CVED exception
				s.Notify();
				gout << MyName();
				gout << ": caught CVED cvCInternalError while deleting ";
				gout << hcsmName << " HCSM" << endl;
			}
			catch( cvCError s )
			{
				// print error message
				s.Notify();
				gout << MyName() << ": caught CVED exception while ";
				gout << "deleting " << hcsmName << " HCSM" << endl;
			}
			catch( ... )
			{
				// print error message
				gout << MyName() << ": caught unknown exception while ";
				gout << "deleting " << hcsmName << " HCSM" << endl;
			}
		}

		m_hcsmToDelete.clear();
	}

	MemLog( 0, HLOG_PROCDEL_END, 0 );

}  // ProcessHcsmDelete

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the specifed dial of the given object.
//
// Remarks:
//
// Arguments:
//   cCvedId - The CVED object whose dial should be set.
//   cDialName - The name of the dial.
//   cDialValue - The dial's value.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::SetHcsmDial(
			const int cCvedId,
			const string& cDialName,
			const string& cDialValue
			)
{
	//
	// Get a pointer to the CVED by the given id.
	//
	const CDynObj* pObj = m_pCved->BindObjIdToClass( cCvedId );
	if( !pObj || !pObj->IsValid() )
	{
		cerr << "SetHcsmDial: invalid cved id = " << cCvedId << endl;
		return;
	}

	//
	// Ask CVED for a pointer to the root HCSM of this CVED object.
	//
	CHcsm* pHcsm = GetHcsm( pObj->GetHcsmId() );
	if( !pHcsm )
	{
		cerr << "SetHcsmDial: invalid hcsm id = " << pObj->GetHcsmId();
		cerr << " for cved id = " << cCvedId << endl;
		return;
	}

	//
	// Set the specified dial of the root HCSM.
	//
	bool success = pHcsm->SetDialByNameStr( cDialName, cDialValue );
	if( !success )
	{
		cerr << "SetHcsmDial: unable to set dial for cved id = " << cCvedId;
		cerr << endl;
		return;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the specifed button of the given object.
//
// Remarks:
//
// Arguments:
//   cCvedId - The CVED object whose dial should be set.
//   cButtonName - The name of the dial.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::SetHcsmButton(
			const int cCvedId,
			const string& cButtonName
			)
{
	//
	// Get a pointer to the CVED by the given id.
	//
	const CDynObj* pObj = m_pCved->BindObjIdToClass( cCvedId );
	if( !pObj || !pObj->IsValid() )
	{
		cerr << "SetHcsmButton: invalid cved id = " << cCvedId << endl;
		return;
	}

	//
	// Ask CVED for a pointer to the root HCSM of this CVED object.
	//
	CHcsm* pHcsm = GetHcsm( pObj->GetHcsmId() );
	if( !pHcsm )
	{
		cerr << "SetHcsmButton: invalid hcsm id = " << pObj->GetHcsmId();
		cerr << " for cved id = " << cCvedId << endl;
		return;
	}

	//
	// Set the specified dial of the root HCSM.
	//
	bool success = pHcsm->SetButtonByName( cButtonName );
	if( !success )
	{
		cerr << "SetHcsmButton: unable to set button for cved id = ";
		cerr << cCvedId << endl;
		return;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Reads the settings to buttons and dials of various objects
//   from external sources.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::ReadExternalBtnDialSettings()
{
	string btnDialValStr = m_sObjBtnDialValToHcsm;
    const size_t tokSize = 256;
	const char* pToken = btnDialValStr.c_str();
	char bigToken[tokSize];
	char cmd1[tokSize];
	char cmd2[tokSize];
	char *pCurrPos = NULL;

	cmd2[0] = 0;
#if (_MSC_VER > 1500)
    #pragma warning( push )
    #pragma warning(disable:4996)
#endif
	strncpy( bigToken, pToken,tokSize );
	char* pSemi;
	pSemi = strtok_s(bigToken, "|",&pCurrPos);
	if( pSemi )
	{
		strncpy( cmd1, pSemi,tokSize );
		pSemi = strtok_s( 0, "|" ,&pCurrPos);
		if( pSemi )
		{
			strncpy( cmd2, pSemi,tokSize );
		}
		else
		{
			cmd2[0] = 0;
		}
	}
	else
	{
		cmd1[0] = 0;
	}

	int hack;
	for( hack = 0; hack < 2; hack++ )
	{
		char token[256];

		if( hack == 0 )
		{
			strncpy( token, cmd1,tokSize );
		}
		else
		{
			strncpy( token, cmd2,tokSize );
		}

		char* pCvedId = strtok_s( token, ":",&pCurrPos );
		if( pCvedId )
		{
			//
			// Found the CVED id.
			//
			string cvedIdStr = pCvedId;
			int cvedId = atoi( cvedIdStr.c_str() );

			char* pBtnDialName = strtok_s( NULL, ":",&pCurrPos );
			if( pBtnDialName )
			{
				//
				// Found the button/dial name.
				//
				string btnDialNameStr = pBtnDialName;

				char* pBtnDialVal = strtok_s( NULL, ":",&pCurrPos );
				if( pBtnDialVal )
				{
					//
					// Found the dial and its value.  Set the dial.
					//
					string btnDialValStr = pBtnDialVal;

					SetHcsmDial( cvedId, btnDialNameStr, btnDialValStr );
					m_sObjBtnDialValToHcsm[0] = 0;
				}
				else
				{
					//
					// Found a button only.  Set the button.
					//
					SetHcsmButton( cvedId, btnDialNameStr );
					m_sObjBtnDialValToHcsm[0] = 0;
				}
			}
			else
			{
				cerr << "ReadExternalBtnDialSettings: invalid setting = ";
				cerr << m_sObjBtnDialValToHcsm << endl;
			}
		}
	} // for
#if (_MSC_VER > 1500)
    #pragma warning( pop )
#endif
}  // ReadExperimenterObjSettings

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes all HCSMs in the hcsm array.
//
// Remarks:  This function executes each active HCSM in the hcsm array
//   by calling the HCSM's execute function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::ExecuteAllHcsm()
{

	//do timer we need to maintain
	// we need to maintian (m_currRate * .8) > m_currUpdateTime, if not we are dropping frames
#ifdef CALC_HEADROOM
	LARGE_INTEGER tempF, tempC;
    __int64 currTicks,tickDiff,tickPerS;

	QueryPerformanceFrequency(&tempF);
	QueryPerformanceCounter(&tempC);

	tickPerS = tempF.QuadPart;
	currTicks = tempC.QuadPart;

	tickDiff = currTicks - m_lastTicks;
	m_currRate = double(tickDiff)/double(tickPerS) * 1000.0;

	m_lastTicks = currTicks;
#endif
	MemLog( -1, HLOG_EXECALL_START, 0 );
	ReadExternalBtnDialSettings();



    multimap<int,CHcsm*> elemsByPriorityToExecute;


	//
	// Execute all void hcsms in the hcsm array.
	//
	// COULD THIS BE MORE EFFICIENT BY USING THE MAP TO FIND HCSMS ??
	//
	int i;
	for( i = 0; i < cMAX_ROOT_HCSM; i++ )
	{
		//
		// Check to make sure that this slot contains a valid pointer to
		// an hcsm instance.
		//
		if( m_hcsmInstances[i] )
		{
			//
			// Now check to see if the root hcsm is active.
			//
			if ( m_hcsmInstances[i]->GetState() == eACTIVE )
			{
				//MemLog( i, HLOG_EXECALL_DOONE_1, m_hcsmInstances[i]->MyName() );
				//m_hcsmInstances[i]->Execute();
				//MemLog( i, HLOG_EXECALL_DOONE_2, 0 );
                elemsByPriorityToExecute.insert( pair<int,CHcsm*>( m_hcsmInstances[i]->GetPriorityLevel(),m_hcsmInstances[i]) );
			}
		}
	}

    multimap<int,CHcsm*>::const_iterator itr;
    for (itr = elemsByPriorityToExecute.begin();itr != elemsByPriorityToExecute.end(); itr++){
        MemLog( i, HLOG_EXECALL_DOONE_1, itr->second->MyName() );
        try {
		    itr->second->Execute();
        }catch(cvCInternalError e){
            gout<<"Got Internal Error: "<<e.m_msg<<endl<<"When running"<<itr->second->GetName()<<endl;
        }
		MemLog( i, HLOG_EXECALL_DOONE_2, 0 );
    }

	//
	// Process all pending delete Hcsm requests.
	//
	ProcessHcsmDelete();

	//
	// Process all pending activate newly created Hcsm requests.
	//
	ProcessHcsmCreate();

	//
	// Increment the HCSM system frame number.
	//
	m_frame++;

	MemLog( -1, HLOG_EXECALL_END, 0 );
#ifdef CALC_HEADROOM
	QueryPerformanceCounter(&tempC);

	currTicks = tempC.QuadPart;

	tickDiff = currTicks -  m_lastTicks;
	m_currUpdateTime = double(tickDiff)/double(tickPerS) * 1000.0;

	if (rand() % 30 == 0 || m_currRate < 8 )
		gout<<"rate =	"<<m_currRate<<"	update time	"<<m_currUpdateTime<<endl;
#endif

}  // ExecuteAllHcsm


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Retrieves the Hcsm ID for the given HCSM.
//
// Remarks:  This function uses the information stored in the m_hcsmMap.
//
// Arguments:
//   pHcsm - Pointer to the HCSM instance.
//
// Returns:  Integer ID of given HCSM.  A value of "-1" if it isn't able
//   to find the HCSM.
//
//////////////////////////////////////////////////////////////////////////////
int CHcsmCollection::GetHcsmId( CHcsm* pHcsm )
{
	map<CHcsm*, int>::iterator mapIterator;
	mapIterator = m_hcsmMap.find( pHcsm );
	if( mapIterator != m_hcsmMap.end() )
	{
		return m_hcsmMap[pHcsm];
	}

	return -1;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Retrieves the pointer to the Hcsm for the given ID.
//
// Remarks:  This function uses the information stored in the m_hcsmInstances.
//
// Arguments:
//   id - Integer ID of desired HCSM.
//
// Returns:  A pointer to the CHcsm instance, if it exists, NULL otherwise.
//
//////////////////////////////////////////////////////////////////////////////
CHcsm* CHcsmCollection::GetHcsm( int id ) const
{
	return m_hcsmInstances[id];
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Retrieves a pointer to the Hcsm for the first Hcsm found
// 	with the given name.
//
// Remarks:  This function uses the information stored in the m_hcsmInstances.
//
// Arguments:
//   name - Hcsm name of desired HCSM.
//
// Returns:  A pointer to the CHcsm instance, if it exists, NULL otherwise.
//
//////////////////////////////////////////////////////////////////////////////
CHcsm* CHcsmCollection::GetHcsm( const string& cName ) const
{
	map<CHcsm*, int>::const_iterator itr;
	for( itr = m_hcsmMap.begin(); itr != m_hcsmMap.end(); itr++ )
	{
		if( itr->first && itr->first->GetName() == cName )  return itr->first;
	}
	return 0;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets an HCSM's debugging level.
//
// Remarks:  This function sets an HCSM's debugging level.  The collection
//   sets the debugging level of the HCSM identified by the given id.
//
// The function with no HCSM specifier, sets the debug level to all root HCSMs.
//
// Arguments:
//   id - Integer ID of desired HCSM.
//   debugLevel - The debugging level.
//
// Returns:  A pointer to the CHcsm instance, if it exists, NULL otherwise.
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::SetHcsmDebugLevel(
			const int id,
			const CHcsmDebugItem::ELevel debugLevel
			)
{
	if (id < cMAX_ROOT_HCSM && m_hcsmInstances[id])
		m_hcsmInstances[id]->SetDebugLevel( debugLevel );
}


void CHcsmCollection::SetHcsmDebugLevel(
			CHcsm* pHcsm,
			const CHcsmDebugItem::ELevel debugLevel
			)
{
	pHcsm->SetDebugLevel( debugLevel );
}

void CHcsmCollection::SetHcsmDebugLevel(
			const CHcsmDebugItem::ELevel debugLevel
			)
{
	int i;

	for( i = 0; i < cMAX_ROOT_HCSM; i++ )
	{
		if( m_hcsmInstances[i] )
		{
			m_hcsmInstances[i]->SetDebugLevel( debugLevel );
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets an HCSM's debugging level.
//
// Remarks:  This function sets an HCSM's debugging mode.  The collection
//   sets the debugging mode of the HCSM identified by the given id.
//
// The function with no HCSM specifier, sets the debug mode to all root HCSMs.
//
// Arguments:
//   id - Integer ID of desired HCSM.
//   mode - The debugging level.
//   pH  - pointer to the HCSM to set the mode for
//
// Returns:  A pointer to the CHcsm instance, if it exists, NULL otherwise.
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetHcsmDebugMode( int id, EDebugMode mode )
{
	m_hcsmInstances[id]->SetDebugMode( mode );
}

void
CHcsmCollection::SetHcsmDebugMode( CHcsm*  pH, EDebugMode mode )
{
	pH->SetDebugMode( mode );
}


void
CHcsmCollection::SetHcsmDebugMode( EDebugMode mode )
{
	int i;
	for( i = 0; i < cMAX_ROOT_HCSM; i++ )
	{
		if( m_hcsmInstances[i] && m_hcsmInstances[i]->GetState() == eACTIVE )
		{
			m_hcsmInstances[i]->SetDebugMode( mode );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Register a vehicle with the intersection manager.
//
// Remarks:  Adds the register request to a pool of requests.  The IM
//   reads the request from here and processes them in order.  This
//   probably shouldn't be located in the HCSM collection but, for now,
//   this is the best place to put it.
//
// Arguments:
//   objId - The CVED id of the HCSM making the request.
//   intrsctnId - The insersection id of the intersection with which to
//                register.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::ImRegister( int objId, int intrsctnId )
{

	//
	// The the register register request to the vector.
	//
	TImRegisterData elem;
	elem.objId = objId;
	elem.intrsctnId = intrsctnId;

	m_imRegisterData.push( elem );

}  // ImRegister


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the oldest IM register request.
//
// Remarks:
//
// Arguments:
//   elem - (output) Contains the request data.
//
// Returns:  A boolean indicating if there is a pending request.
//
//////////////////////////////////////////////////////////////////////////////
bool
CHcsmCollection::GetImRegisterElem( TImRegisterData& elem )
{
	if( m_imRegisterData.size() > 0 )
	{
		elem = m_imRegisterData.front();
		m_imRegisterData.pop();
		return true;
	}

	return false;
}  // GetImRegisterElem

void
CHcsmCollection::MemLog( int hcsmId, int tag, const char* pMsg )
{
	m_memLog.Log( tag, m_frame, (float) hcsmId, pMsg );
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Given a vector of filenames, loads the contents of the
//  files into memory.
//
// Remarks: This function needs to be called before starting execution
//  of the behaviors.  It assumes that the files are located in the
//  same directory pointed to by NADSSDC_SCN or the current directory.
//
// Arguments:
//   cFileNames - A vector of file names.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::PreloadFiles( const vector<string>& cFileNames )
{
	vector<string>::const_iterator itr;
	for( itr = cFileNames.begin(); itr != cFileNames.end(); itr++ )
	{
//		gout << "Preloading '" << *itr << "' ..." << endl;

		//
		// Build the file name and path.
		//
		string fileName = *itr;
        string scnPath;
        NADS::GetEnvVar(scnPath,"NADSSDC_SCN" );
		if( scnPath == "" )
		{
			gout << "PreloadFiles: undefined system ";
			gout << "variable 'NADSSDC_SCN'" << endl;
		}
		scnPath +=fileName;
		//
		// Opening the data file.
		//
		ifstream preloadFile;
		preloadFile.open( scnPath );

		//
		// Make sure that the files opened properly.
		//
		if( !preloadFile )
		{
			gout << "PreloadFiles: unable to open '" << scnPath << "'";
			gout << endl;

			continue;
		}

		//
		// Read the contents of the file into a vector of strings..one for
		// each line.
		//
		TPreloadData preloadData;
		preloadData.fileName = fileName;
		char buf[1024];

		while( preloadFile )
		{
			preloadFile.getline( buf, 1024 );
			string data = buf;
			preloadData.fileData.push_back( buf );
		}

		m_preloadFiles.push_back( preloadData );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets a new handle to the specified preload file.
//
// Remarks:
//
// Arguments:
//   cFileName - The name of the preload file to get a handle for.
//
// Returns: An integer which represents the handle to a file.  -1 if no
//  file was found corresponding to the input filename.
//
//////////////////////////////////////////////////////////////////////////////
int
CHcsmCollection::GetPreloadFileHandle( const string& cFileName )
{
	//
	// Locate the filename.
	//
	bool foundFile = false;
	vector<TPreloadData>::const_iterator cItr;
	for(
		cItr = m_preloadFiles.begin();
		cItr != m_preloadFiles.end();
		cItr++
		)
	{
		if( cFileName == cItr->fileName )
		{
			foundFile = true;
			break;
		}
	}

	if( !foundFile )  return -1;

	//
	// Instantiate a new iterator for the file and return an associated
	// handle to it.
	//
	TPreloadMapData mapData;
	mapData.cCurr = cItr->fileData.begin();
	mapData.cEnd  = cItr->fileData.end();

	m_preloadHandleCounter++;
	m_preloadHandleMap[m_preloadHandleCounter] = mapData;
	return m_preloadHandleCounter;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Given a file handle, gets the sample of data from the file
//  pointed to by the file handle.
//
// Remarks:
//
// Arguments:
//   fileHandle - The file handle.
//   data - (output) The next sample from the file pointed to by the handle.
//
// Returns: A boolean that indicates whether the file has another sample
//  to return.
//
//////////////////////////////////////////////////////////////////////////////
bool
CHcsmCollection::GetNextPreloadFileSample( int fileHandle, string& data )
{
	map<int, TPreloadMapData>::iterator mapIterator;
	mapIterator = m_preloadHandleMap.find( fileHandle );
	if( mapIterator == m_preloadHandleMap.end() )
	{
		gout << "GetNextPreloadFileSample: fileHandle ";
		gout << fileHandle << " not found" << endl;

		return false;
	}

	mapIterator->second.cCurr++;
	if( mapIterator->second.cCurr != mapIterator->second.cEnd )
	{
		data = *(mapIterator->second.cCurr);
		return true;
	}
	else
	{
		// remove the entry from the map
		// return value: 0 --> error, 1 --> ok
		if( m_preloadHandleMap.erase( mapIterator->first ) == cMAP_ERASE_ERROR )
		{
			gout << "GetNextPreloadFileSample: map erase failed" << endl;
		}
		return false;
	}
}



//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// The following function are related to the activity logs.
//
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the HCSM creation log.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM being created.
//   hcsmType - An integer representing the HCSM type.
//   cHcsmName - The HCSM's name.
//   cPos - The HCSM's position at creation.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetHcsmCreateLog(
	CHcsm* pHcsm,
	int hcsmType,
	const string cHcsmName,
	const CPoint3D& cPos
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventHcsmCreate event;
		event.SetData( GetHcsmId( pHcsm ), hcsmType, cHcsmName, cPos );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the Start DataRed log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   segment - An integer representing the segment.
//   pParams - Param string
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionStartDataRedLog(
	int hcsmId,
	int segment,
	const char* pColumn,
	const char* pParams
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionStartDataRed event;
		event.SetData( hcsmId, segment, pColumn, pParams );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the Start DataRed log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   segment - An integer representing the segment.
//   pParams - Param string
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionStopDataRedLog(
	int hcsmId,
	int segment,
	const char* pParams
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionStopDataRed event;
		event.SetData( hcsmId, segment, pParams );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the UseTrafManSet log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   cSetName - The name of the set to change to.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionUseTrafManSetLog(
	int hcsmId,
	const string& cSetName
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionUseTrafManSet event;
		event.SetData( hcsmId, cSetName );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the PlayAudio log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionPlayAudioLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionPlayAudio event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the VehicleFailure log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionVehicleFailureLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionVehicleFailure event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the TrafficLight log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   trafLightId - the CVED id of the traffic light
//   trafLightState - the state the traffic light will change to
//   time - the amount of time that the traffic light will take to change
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionTrafficLightLog(
	int hcsmId,
	int trafLightId,
	int trafLightState,
	double time
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionTrafficLight event;
		event.SetData( hcsmId, trafLightId, trafLightState, time );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the LogData log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   logId - The index into the log array
//   value - The value stored at that index
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionLogDataLog(
	int hcsmId,
	int logId,
	double value
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionLogData event;
		event.SetData( hcsmId, logId, value );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the TerminateSimulation log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionTerminateSimulationLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionTerminateSimulation event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the PreposMotion log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionPreposMotionLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionPreposMotion event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the TuneMotion log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionTuneMotionLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionTuneMotion event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the PhoneCall log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionPhoneCallLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionPhoneCall event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the ResetDial log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionResetDialLog(
	int hcsmId,
	const string& cDialName
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionResetDial event;
		event.SetData( hcsmId, cDialName );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the SetVariable log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   varName - the name of the variable being set
//   value - the value to assign the variable
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionSetVariableLog(
	int hcsmId,
	const string& varName,
	double value
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionSetVariable event;
		event.SetData( hcsmId, varName, value );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the SetDial Action log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   cDialName - the name of the dial being set
//   cValue - the value of the dial
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionSetDialLog(
	int hcsmId,
	const string& cDialName,
	const string& cValue
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionSetDial event;
		event.SetData( hcsmId, cDialName, cValue );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the SetButton Action log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//   cButtonName - the name of the variable being set
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionSetButtonLog(
	int hcsmId,
	const string& cButtonName
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionSetButton event;
		event.SetData( hcsmId, cButtonName );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the Create Action log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionCreateLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionCreate event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the Create Action log.
//
// Remarks:
//
// Arguments:
//   hcsmId - The HCSM id of the trigger
//
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetActionDeleteLog(
	int hcsmId
	)
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventActionDelete event;
		event.SetData( hcsmId );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the HCSM deletion log.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM being created.
//   cPos - The HCSM's position at creation.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetHcsmDeleteLog( CHcsm* pHcsm, const CPoint3D& cPos )
{
	//
	// Instance a HCSM create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventHcsmDelete event;
		event.SetData( GetHcsmId( pHcsm ), cPos );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the HCSM activation log.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM being activated.
//   cPos - The HCSM's position at activation.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetHcsmActivateLog( CHcsm* pHcsm, const CPoint3D& cPos )
{
	//
	// Instance a HCSM activate event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventHcsmActivate event;
		event.SetData( GetHcsmId( pHcsm ), cPos );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the CVED creation log.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM representing the CVED object being created.
//   cvedId - The CVED object's unique id.
//   cvedType - The CVED object's type.
//   cPos - The object's position at creation.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetCvedCreateLog(
	CHcsm* pHcsm,
	int cvedId,
	cvEObjType cvedType,
	const CPoint3D& cPos
	)
{
	//
	// Instance a CVED create event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventCvedCreate event;
		event.SetData( GetHcsmId( pHcsm ), cvedId, cvedType, cPos );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets a trigger's fire event.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM being created.
//   hcsmType - An integer representing the HCSM type.
//   cHcsmName - The HCSM's name.
//   cPos - The HCSM's position at creation.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetTriggerFireLog(
			CHcsm* pHcsm,
			int instigatorHcsmId,
			int* pCandidateSet,
			int candidateSetSize
			)
{
	//
	// Instance a trigger file event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventTriggerFire event;
		event.SetData(
					GetHcsmId( pHcsm ),
					instigatorHcsmId,
					pCandidateSet,
					candidateSetSize
					);
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the dial set event.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM being created.
//   cDialName - A string representing the dial's name.
//   cSetting - The dial's setting value.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetDialSettingLog(
			CHcsm* pHcsm,
			const string& cDialName,
			const string& cSetting
			)
{
	//
	// Instance a HCSM dial/button set event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventDialButSet event;
		event.SetData( GetHcsmId( pHcsm ), cDialName, cSetting );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the dial set event.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM being created.
//   cDialName - A string representing the dial's name.
//   value - The dial's setting value.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetDialSettingLog(
			CHcsm* pHcsm,
			const string& cDialName,
			double value
			)
{
	//
	// Instance a HCSM dial/button set event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventDialButSet event;
		char buf[128];
		sprintf_s( buf,128, "%g", value );
		string valueStr = buf;
		event.SetData( GetHcsmId( pHcsm ), cDialName, valueStr );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the button set event.
//
// Remarks:
//
// Arguments:
//   pHcsm - A pointer to the HCSM being created.
//   cButtonName - The button's name.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetButtonSettingLog( CHcsm* pHcsm, const string& cButtonName )
{
	//
	// Instance a HCSM dial/button set event and add it to the activity log.
	//
	if( m_logActivities )
	{
		CEventDialButSet event;
		event.SetData( GetHcsmId( pHcsm ), cButtonName, "" );
		m_sActvLog.Add( m_frame, &event );

		if( m_printActvLog )  event.Print( m_frame );
	}
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///    Gets a random number generator
///\remark
///    if one is not found with given name returns a random number generator
///    that is inited with a seed set from random device (which is a HW lvl)
///
////////////////////////////////////////////////////////////////////////////
void
   CHcsmCollection::CreateRandomNumberGenerator( const string& cName, const vector<long> &seed ){
    if (seed.size() == 0){
        m_randomGenerators[cName] = shared_ptr<mt19937>(new std::mt19937(std::random_device()));
    }else{
		std::seed_seq seq(seed.begin(),seed.end());
        m_randomGenerators[cName] = shared_ptr<mt19937>(new std::mt19937(seq));
    }
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///    Gets a random number generator
///\remark
///    if one is not found with given name returns a random number generator
///    that is inited with a seed set from random device (which is a HW lvl)
///
////////////////////////////////////////////////////////////////////////////
shared_ptr<mt19937>
CHcsmCollection::GetRandomNumberGenerator(const string& cName){
    auto itr = m_randomGenerators.find(cName);
    if (itr != m_randomGenerators.end()){
        return itr->second;
    }else{
       return m_randomGenerators[cDefault];
    }
}
bool
CHcsmCollection::DoesRandomNumberGeneratorExist(const string& cName)
{
	return ( m_randomGenerators.find( cName ) != m_randomGenerators.end() );
}

void
CHcsmCollection::SetExprVariable( const string& cName, double value )
{

	m_exprVariables[cName] = value;
}

bool
CHcsmCollection::ExprVariableExists( const string& cName )
{
	return ( m_exprVariables.find( cName ) != m_exprVariables.end() );
}

double
CHcsmCollection::GetExprVariable( const string& cName )
{
	if( !ExprVariableExists( cName ) )
	{
		return 0.0;
	}
	else
	{
		return m_exprVariables[cName];
	}
}

void
CHcsmCollection::SetExprPosVariable( const string& cName, const CPoint3D &value )
{
	m_exprPosVariables[cName] = value;
}

bool
CHcsmCollection::ExprPosVariableExists( const string& cName )
{
	return ( m_exprPosVariables.find( cName ) != m_exprPosVariables.end() );
}

CPoint3D
CHcsmCollection::GetExprPosVariable( const string& cName )
{
	if( !ExprPosVariableExists( cName ) )
	{
		return CPoint3D(0,0,0);
	}
	else
	{
		return m_exprPosVariables[cName];
	}
}

void CHcsmCollection::ClearVariables(){
	m_exprPosVariables.clear();
    m_varQueues.clear();
    m_randomGenerators.clear();
    m_randomGenerators[cDefault] = shared_ptr<mt19937>(new std::mt19937(std::random_device()));
}
////////////////////////////////////////////////////////////////////////////////////////////
///\brief
///     This function does var queue operations
///
////////////////////////////////////////////////////////////////////////////////////////////
bool
CHcsmCollection::DoVarQueueOperation(const string& opp){
  typedef boost::tokenizer<boost::char_separator<char> >
    tokenizer;
  boost::char_separator<char> sep(":;,()");
  tokenizer tokens(opp, sep);
  tokenizer::iterator itr = tokens.begin();
  if (itr == tokens.end()){
    return false;
  }
  if (*itr == "CreateQueue"){
     if (itr != tokens.end()){
         itr++;
         string queName = *itr;
         if (m_varQueues.find(queName) == m_varQueues.end()){
            //create and empty que
             m_varQueues[queName];
             return true;
         }else{
            return false;
         }
     }else{
         return false;
     }
  }
  auto targetQueueItr = m_varQueues.find(*itr);
  if (targetQueueItr != m_varQueues.end()){
    string queueName = *itr;
    itr++;
    if (itr!=tokens.end()){
      if (*itr == "SaveToFile"){
          if (itr != tokens.end()){
              itr++;
              string fileName = *itr;
              ofstream ofile(fileName);
              if (!ofile.is_open()){
                return false;
              }
              //save each item to a file
              for (auto qItr = targetQueueItr->second.begin(); qItr != targetQueueItr->second.end(); qItr++){
                  ofile<<qItr->first<<"\t"<<qItr->second<<endl;
              }
              ofile.close();
          }else{
            return false;
          }
      }
      else if (*itr == "LoadFile"){
          if (itr != tokens.end()){
              itr++;
              string fileName = *itr;
              ifstream ifile(fileName);
              if (!ifile.is_open()){
                return false;
              }
              pair<string,double> valuePair;
              while (!ifile.eof() && !ifile.bad()){
                  ifile>>valuePair.first>>valuePair.second;
                  if (!ifile.fail())
                      targetQueueItr->second.push_back(valuePair);
              }
              ifile.close();
          }else{
            return false;
          }
      }
      else if (*itr == "PopFront"){
          if (targetQueueItr->second.size()  == 0)
              return false;
          pair<string,double> valuePair = targetQueueItr->second.front();
          CHcsmCollection::SetExprVariable(valuePair.first, valuePair.second);
          targetQueueItr->second.erase(targetQueueItr->second.begin());
      }
      else if (*itr == "PopBack"){
          if (targetQueueItr->second.size()  == 0)
              return false;
          pair<string,double> valuePair = targetQueueItr->second.back();
          CHcsmCollection::SetExprVariable(valuePair.first, valuePair.second);
          targetQueueItr->second.pop_back();
      }
      else if (*itr == "PushBackVar"){
          if (itr == tokens.end()){
            return false;
          }
          itr++;
          pair<string,double> valuePair;
          valuePair.first = *itr;
          valuePair.second = GetExprVariable(*itr);
          targetQueueItr->second.push_back(valuePair);
      }
      else if (*itr == "PushBackList"){
          if (itr == tokens.end()){
            return false;
          }
          itr++;
          if (itr == tokens.end()) return false;
          pair<string,double> valuePair;
          valuePair.first = *itr;
          itr++;
          if (itr == tokens.end()) return false;
          stringstream converter;
          string temp;
          temp = *itr;
          converter<<temp<<"\t";
          itr++;
          if (itr == tokens.end()) return false;
          temp = *itr;
          converter<<temp<<"\t";
          itr++;
          if (itr == tokens.end())
              temp = "1";
          else
            temp = *itr;
          converter<<temp<<"\t";

          float min = 0; float max = 0; float stride = 1.0f;
          converter>>min>>max>>stride;
          max+=stride;
          for (float i = min; i < max; i+=stride){
            valuePair.second = i;
            targetQueueItr->second.push_back(valuePair);
          }
      }
      else if (*itr == "PushBackValue"){
          if (itr == tokens.end()){
            return false;
          }
          itr++;
          pair<string,double> valuePair;
          valuePair.first = *itr;
          if (itr == tokens.end()){
            return false;
          }
          itr++;
          stringstream converter;
          converter<<*itr;
          converter>>valuePair.second;
          if (!converter.fail()){
            targetQueueItr->second.push_back(valuePair);
          }else{
            valuePair.second = GetExprVariable(*itr);
            targetQueueItr->second.push_back(valuePair);
          }
      }
      else if (*itr == "Clear"){
          targetQueueItr->second.clear();
      }
      else if (*itr == "RandomShuffle"){
          std::random_device rd;
          //randonmly shuffle our vector
          std::random_shuffle(
              targetQueueItr->second.begin(),
              targetQueueItr->second.end(),
              [&rd] (int i){
                  return rd()%i;
              }
          );
      }
    }else{
        return false;
    }
  }
  return true;

}
//////////////////////////////////////////////////////////////////////////////
///\brief
///     Get Size of a specified Q, if it does not yet exist, return false size = 0
//////////////////////////////////////////////////////////////////////////////
bool
CHcsmCollection::GetVarQueueSize(const string& varQue, int& size){
  auto targetQueueItr = m_varQueues.find(varQue);
  if (targetQueueItr != m_varQueues.end()){
      size = (int)targetQueueItr->second.size();
      return true;
  }else{
      size = 0;
      return false;
  }
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets the visual display text.
//
// Remarks:  This text is supposed to get displayed on the visual display.
//
// Arguments:
//
// Returns: A const string reference.
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::GetVisualDisplayText( string& displayText, int location )
{
    map<int,string>::iterator itr = m_sVisualDisplayText.find(location);
    if (itr != m_sVisualDisplayText.end()){
        displayText = itr->second;
    }else{
        displayText.clear();
    }
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Set the visual display text.
//
// Remarks:  This text is supposed to get displayed on the visual display.
//
// Arguments:
//  cDisplayText - The text to be displayed.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetVisualDisplayText( const string& cDisplayText, int location )
{
	m_sVisualDisplayText[location] = cDisplayText;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets the visual settings text.
//
// Remarks:  This text is used to determine specific traits of the visuals.
//           It is usually read from the header block of the scenario file.
//
// Arguments:
//
// Returns: A const string reference.
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::GetVisualSettings( string& visualSettings )
{
	visualSettings = m_sVisualSettings;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Set the visual settings text.
//
// Remarks:  This text is used to determine specific traits of the visuals.
//           It is usually read from the header block of the scenario file.
//
// Arguments:
//  cDisplayText - The text to be displayed.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetVisualSettings( const string& cVisualSettings )
{
	m_sVisualSettings = cVisualSettings;
}

bool
CHcsmCollection::IsSimulationTerminated( void )
{
	return CHcsmCollection::m_sSCC_Scenario_Stop_Ind == 1;
}

string
CHcsmCollection::GetActvLogFileName( void ) const
{
	//
	// Build the activity log file name.  It should be written to
	// bin directory(where other logs exist) and the name should be
	// composed of the run instance name and a ".txt" suffix.
	//
	string logFileName;
    NADS::GetEnvVar(logFileName,"NADSSDC_BIN" );
	if( logFileName != "" )
	{
		logFileName += "\\";
	}

	if( m_sExperiment && m_sSubject && m_sRun )
	{
		logFileName += m_sExperiment;
		_mkdir( logFileName.c_str() );
		logFileName += "\\";

		logFileName += m_sSubject;
		_mkdir( logFileName.c_str() );
		logFileName += "\\";

		logFileName += m_sRun;
		_mkdir( logFileName.c_str() );
		logFileName += "\\";
	}

#if 0
	ULARGE_INTEGER lFreeBytesAvailable,lTotalNumberOfBytes,	lTotalNumberOfFreeBytes;
	if( 0 == GetDiskFreeSpaceEx( logFileName.c_str(), &lFreeBytesAvailable, &lTotalNumberOfBytes, &lTotalNumberOfFreeBytes) )
	{
		cerr << "ERROR CHcsmCollection: not enough disk space: freeBytesAvail = ";
		cerr << lFreeBytesAvailable << endl;
		return "";
	}
#endif

	string actvLogFileName = logFileName;
	if( strlen( m_sRunInst ) > 0 )
	{
		actvLogFileName += "actvlog_";
		actvLogFileName += m_sRunInst;
	}
	else
	{
		actvLogFileName += "actvlog";
	}
	actvLogFileName += ".log";

	return actvLogFileName;
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///		Sets the path for diGuy Object
///\remark
///		This provides buffering for the graphics system, and HCSM
///		the control of the DiGuy is done through graphics. action and times
///		should be 1/4 the size of path, or empty
///
///\note This is a blocking call
///
///\param[in] id ID of object
///\param[in] path array of positions x,y,z,yaw
///\param[in] actions array of actions (0-255)
///\param[in] times time to perform blocking actions at each node
/////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::SetDiGuyPathInfo(int id,
	                                   const vector<float> &path,
									   const vector<char> &actions,
									   const vector<float> &times){
	m_DiGuyPathCriticalSection.Lock();
	auto itr = CHcsmCollection::m_sDiGuyPathInfo.find(id);
	m_sDiGuyPathInfo[id] = path;
	if (actions.size() > 0){
		m_sDiGuyPathActionInfo[id] = actions;
		m_sDiGuyPathTimesInfo[id] = times;
	}
	m_DiGuyPathCriticalSection.UnLock();
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///		Sets the path for diGuy Object
///\remark
///		This provides buffering for the graphics system, and HCSM
///		the control of the DiGuy is done through graphics. action and times
///		should be 1/4 the size of path, or empty
///
///\note This is a blocking call
///
///\param[in] id ID of object
///\param[out] path array of positions x,y,z,yaw
///\param[out] actions array of actions (0-255)
///\param[out] times time to perform blocking actions at each node
/////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::GetDiGuyPathInfo(int id,
	                                   vector<float> &path,
									   vector<char> &actions,
									   vector<float> &times){
	m_DiGuyPathCriticalSection.Lock();
	actions.clear();
	times.clear();
	auto itr = CHcsmCollection::m_sDiGuyPathInfo.find(id);
	if (itr != m_sDiGuyPathInfo.end()){
		path = itr->second;
	}
	auto itr2 = CHcsmCollection::m_sDiGuyPathActionInfo.find(id);
	if (itr2 != m_sDiGuyPathActionInfo.end()){
		actions = itr2->second;
	}
	itr = CHcsmCollection::m_sDiGuyPathTimesInfo.find(id);
	if (itr != m_sDiGuyPathTimesInfo.end()){
		times = itr->second;
	}
	m_DiGuyPathCriticalSection.UnLock();
}
void CHcsmCollection::AddDiGuyCommand(const CDiGuyUpdateCommand & command){
	m_DiGuyPathCriticalSection.Lock();
	m_sDiGuyCommandQueue.push_back(command);
	m_DiGuyPathCriticalSection.UnLock();
}
void CHcsmCollection::GetDiGuyCommandQueue(vector<CDiGuyUpdateCommand>& commands) {
	m_DiGuyPathCriticalSection.Lock();
	commands = m_sDiGuyCommandQueue;
	m_DiGuyPathCriticalSection.UnLock();
}
void CHcsmCollection::clearDiGuyCommandQueue(){
	m_DiGuyPathCriticalSection.Lock();
	m_sDiGuyCommandQueue.clear();
	m_DiGuyPathCriticalSection.UnLock();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///\brief
///		Set the last known good location of the ownship
///\remark
///		This function set the last good location of the ownship, the last good location
///		is defined as the last position the user was on the road, and in pre-defined
///		ownship path. This function provides a mutex lock as the read and writer of this
///		data is most likely going to be two different threads, so we want to make sure we
///		do not get somekind of dirty read on the user end
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::SetLastGoodPosition(float x, float y, float z){
	DWORD waitCode = WaitForSingleObject(CHcsmCollection::m_sLastGoodLocationMutex,1);
	//we should probully handle the wait code here.....
	m_sLastGoodLocation[0] = x;
	m_sLastGoodLocation[1] = y;
	m_sLastGoodLocation[2] = z;
	ReleaseMutex(CHcsmCollection::m_sLastGoodLocationMutex);

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///\brief
///		Set the last known good location of the ownship
///\remark
///		This function set the last good location of the ownship, the last good location
///		is defined as the last position the user was on the road, and in pre-defined
///		ownship path. This function provides a mutex lock as the read and writer of this
///		data is most likely going to be two different threads, so we want to make sure we
///		do not get somekind of dirty read on the user end
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CHcsmCollection::GetLastGoodPosition(float &x, float &y, float &z){
	DWORD waitCode = WaitForSingleObject(CHcsmCollection::m_sLastGoodLocationMutex,1);
	//we should probully handle the wait code here.....
	x = m_sLastGoodLocation[0];
	y = m_sLastGoodLocation[1];
	z = m_sLastGoodLocation[2];
	ReleaseMutex(CHcsmCollection::m_sLastGoodLocationMutex);
}

CHcsmStaticLock::CHcsmStaticLock(){
    InitializeCriticalSection(&m_critSection);
}
void CHcsmStaticLock::Lock(){
    EnterCriticalSection(&m_critSection);
    return;
}
bool CHcsmStaticLock::TryLock(){
    return TryEnterCriticalSection(&m_critSection) == TRUE;
}
void CHcsmStaticLock::UnLock(){
    LeaveCriticalSection(&m_critSection);
    return;
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///    Gets the ADO driving the Ownship
///\remark
///    This function may return null
/////////////////////////////////////////////////////////////////////////////
CHcsm*
CHcsmCollection::GetExtDriverSurrogate( ) const{
  return m_ownDriverSurrogate;

}
/////////////////////////////////////////////////////////////////////////////
///\brief
///    Sets the reference of the ADO Surrogate
///\remark
///    This function does not actually set a ADO to be the Surrogate,
///    it just set the reference so it can be found be other systems.
/////////////////////////////////////////////////////////////////////////////
void
CHcsmCollection::SetExtDriverSurrogate(CHcsm* pObj){
	m_ownDriverSurrogate = pObj;
}