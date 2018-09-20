/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: hcsmcollection.h,v 1.143 2018/09/07 14:30:20 IOWA\dheitbri Exp $
 * Author(s):    Omar Ahmad
 * Date:         July, 1998
 *
 * Description:  Interface for the CHcsmCollection class.
 *
 ****************************************************************************/

#ifndef __CHCSMCOLLECTION_H
#define __CHCSMCOLLECTION_H
#define USING_RTEXPSLITE 1
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include <list>
#include <map>
#include <set>
#include <queue>
#include <string>
#include <memory>
using namespace std;

#include <snoblock.h>
#include <randnumgen.h>
#include <random>
#include <cvedpub.h>
#include <memlog.h>
//#include <subsysbase.h>


#include "hcsm.h"
#include "debugitem.h"
#include "actvlog.h"
#include "actionparseblock.h"
#include "point3d.h"
#include "path.h"
#include "EnvVar.h"
#include "RendererInterface.h"

#define AUDIO_TRIGGER_BYPASS

using namespace CVED;

typedef struct
{
	int	objId;
	int intrsctnId;
} TImRegisterData;

const int cMAX_DYN_OBJ = 20;
const int cMAX_DYN_OBJ_NAME_SIZE = 32;
const int cMAX_VARNAME_SIZE  = 256;
const int cMAX_CHANGED_STAT_OBJ = 20;
typedef struct
{
	int            cvedId[cMAX_DYN_OBJ];
	int            solId[cMAX_DYN_OBJ];
	int            hcsmTypeId[cMAX_DYN_OBJ];
	short          colorIndex[cMAX_DYN_OBJ];
	char           name[cMAX_DYN_OBJ * cMAX_DYN_OBJ_NAME_SIZE];
	float          pos[cMAX_DYN_OBJ * 3];
	float          rollPitch[cMAX_DYN_OBJ * 2];
	float          heading[cMAX_DYN_OBJ];
	unsigned int   audioVisualState[cMAX_DYN_OBJ];
	float          vel[cMAX_DYN_OBJ];
    float          tarPos[cMAX_DYN_OBJ * 3];
    float          tarVelMs[cMAX_DYN_OBJ]; //<target velocity in ms 
    float          tarAccMs2[cMAX_DYN_OBJ];//<target accell in ms^2 
} TDynObjData;

const int cMAX_STAT_OBJ = 20;
const int cMAX_STAT_OBJ_NAME_SIZE = 32;
typedef struct
{
	int            cvedId[cMAX_STAT_OBJ];
	int            solId[cMAX_STAT_OBJ];
	int            hcsmTypeId[cMAX_STAT_OBJ];
	char           name[cMAX_STAT_OBJ * cMAX_STAT_OBJ_NAME_SIZE];
	float          pos[cMAX_STAT_OBJ * 3];
	float          rollPitch[cMAX_STAT_OBJ * 2];
	float          heading[cMAX_STAT_OBJ];
	unsigned int   audioVisualState[cMAX_STAT_OBJ];
} TStatObjData;

typedef struct
{
	string         cellName;
	int            elemId;
} TWriteCellData;
const int cMAX_WRITE_CELL_DATA_SIZE = 10;

typedef struct
{
	string                           cellName;
	CActionParseBlock::ECellDataType cellType;
	vector<int>                      cvedIDs;
	vector<float>                    floatData;
	vector<int>                      intData;
	vector<short>                    shortData;
	string                           cellData;
	long                             frame;
} TScenarioWriteCellData;
const int cMAX_SCENARIO_WRITE_CELL_EVENTS = 20;

typedef struct
{
	string                           uniformName;
	unsigned int                     targetCvedId; //<CVED ID
	CActionParseBlock::ECellDataType cellType;
	vector<float>                    floatData;
	vector<int>                      intData;
	vector<short>                    shortData;
} TScenarioSetUniform;
const int cMAX_SET_UNIFORM_CELL_EVENTS = 20;

typedef struct
{
	string                           cellName;
	string                           pragmas;
	vector<int>                      cvedIDs;
} TAttachShaderCmd;
const int cMAX_SCENARIO_ATTACH_SHADER = 20;

typedef struct
{
	string                           switchName;
	int                              id;
	vector<int>                      cvedIDs;
} TSetSwitchCmd;
const int cMAX_SCENARIO_SET_SWITCH = 80;

typedef enum {
	eNone = 0,
	eSwitch  =1
} EVisaulOperationType;

typedef struct
{
	EVisaulOperationType    eOptionType;
	string                  element;
	int                     option1;
	int                     option2;
} TCabVisualOperations;
const int cMAX_VISUAL_OPERATIONS = 10;




typedef struct
{
	string         fileName;
	vector<string> fileData;
} TPreloadData;

typedef struct
{
	vector<string>::const_iterator cCurr;
	vector<string>::const_iterator cEnd;
} TPreloadMapData;

typedef struct
{
	char	varName[cMAX_VARNAME_SIZE];
	char	label[cMAX_VARNAME_SIZE];
	char	group[cMAX_VARNAME_SIZE];
} TGraphVar;
typedef struct
{
    string setting;
    string value;
} TSetEnvConditionCommand;

class COnScreenGraph 
{
public:
	enum EGraphTypes{
		eBARGRAPH,
		eNOGRAPH
	};
	vector<TGraphVar> m_vars;
	float	m_transparency;
	CPoint2D m_position;
	EGraphTypes	m_type;
	float	m_lblFontSize;
	float	m_groupFontSize;
	float	m_width;
	float	m_height;
	COnScreenGraph(){
		m_type = eNOGRAPH;
		m_lblFontSize = 0.0;
		m_groupFontSize = 0.0;
		m_width = 0.0;
		m_height = 0.0;
		m_transparency = 0.0;
	}

};
class CDiguyRotationOverride
{
public:
	CDiguyRotationOverride();
	CDiguyRotationOverride(int, const std::string&, const COrientation&);
	CDiguyRotationOverride(const CDiguyRotationOverride&);
	CDiguyRotationOverride& operator= (const CDiguyRotationOverride&);
	CDiguyRotationOverride& operator= (CDiguyRotationOverride&&);
	CDiguyRotationOverride(const CDiguyRotationOverride&&);
	int                     HCSMid;
	string                  joint;
	COrientation            angles;
};

class CDiGuyUpdateCommand{
public:
	unsigned	m_Id;					///< User specified unique identifier
	int         m_size;                 ///< Size of Extra Command Info
	int         m_command;              ///<command to set
	float       m_params[5];                
	char        m_dataLoad[128];        ///< Data we have yet to send
	CDiGuyUpdateCommand(){
		m_Id = -1;			
		m_size = 0;         
		m_command = -1;      
		memset(m_params,0,sizeof(m_params));    
		memset(m_dataLoad,0,sizeof(m_dataLoad));
	}
};
static_assert(std::is_trivially_copy_constructible<CDiGuyUpdateCommand>::value == true,"CDiGuyCommand must support shallow copy");
typedef bool (*TReadCellFuncptr)(const string&, int, float&);

const int cMAX_ROOT_HCSM = 1200;
const int cMAX_DEBUG_ITEMS = 5000;
const int cOBJ_BTNDIAL_SIZE = 128;
const int cNUM_LOG_STREAMS = 5;
const int cNUM_DATARED_SEGMENTS = 5;
const int cNUM_DATARED_PARAMS = (cDR_COLSIZE + cDR_DATASIZE) * 32 * cNUM_DATARED_SEGMENTS;
const int cNUM_DATARED_SEGMENTS_SIZE = cNUM_DATARED_SEGMENTS + 1;
const int cNUM_DATARED_PARAMS_SIZE = cNUM_DATARED_PARAMS + sizeof( int );
const int cBRAKE_COND_SIZE = 11;
const int cTIRE_COND_SIZE = 10;
const int cFOLLOW_INFO_SIZE = 9;
const int cFOLLOW_INFO_HEADWAY_IDX = 2;
const int cFOLLOW_INFO_TTC_IDX = 3;
const int cSENSOR_INFO_SIZE = 10;
const int cSENSOR_INFO_HEADWAY_IDX = 2;
const int cSENSOR_INFO_TTC_IDX = 3;
const int cFALSE_ALARM_SIZE = 4;
const int cFCW_INFO_SIZE = 4;
const int cLANE_DEV_INFO_SIZE = 4;
const int cLANE_MARKING_INFO_SIZE = 2;
const int cLANE_DEPART_WARN_SIZE = 4;
const int cSENSOR_CONFIG_SIZE = 3;

const int cNO_ABORT = 0;
const int cCOLLISION_ABORT = 1;
const int cOFF_ROAD_ABORT = 2;
const int cNAVIGATION_ABORT = 3;
const int cOPP_ABORT = 4;
const int cOPP_ABORT_NO_RESTART = 5;

typedef enum { 
	eSENSOR_INDEX_MAX_RANGE = 0, 
	eSENSOR_INDEX_CONE_ANGLE, 
	eSENSOR_INDEX_LANE_CHECK 
} ESensorIndex;

typedef enum {
	eTURN_SIGNAL_OFF = 1,
	eTURN_SIGNAL_LEFT,
	eTURN_SIGNAL_RIGHT,
	eTURN_SIGNAL_BOTH
} ETurnSignalState;
typedef enum {
	eLDW_NONE = 0,
	eLDW_MONITORING,
	eLDW_LEFT,
	eLDW_RIGHT
} ELdwStatus;
typedef enum {
	eBSW_NONE = 0,
	eBSW_LOW_LEFT,
	eBSW_LOW_RIGHT,
	eBSW_LOW_BOTH,
	eBSW_HIGH_LEFT,
	eBSW_HIGH_RIGHT,
	eBSW_HIGH_BOTH
} EBswStatus;
typedef enum { eFCW_NONE = 0, eFCW_LOW, eFCW_HIGH } EFcwStatus;

/////////////////////////////////////////////////////////////////////////////////
///\brief
///     This is a wrapper for critical sections used by HCSMCollection
///
//////////////////////////////////////////////////////////////////////////////////
class CHcsmStaticLock{
public:
    CHcsmStaticLock();
    bool TryLock();
    void Lock();
    void UnLock();
private:
    CRITICAL_SECTION m_critSection;
};

//////////////////////////////////////////////////////////////////////////////
///
/// This class maintains a collection of all root HCSM instances.  It provides
/// functions to create, delete and execute root HCSMs.  It also provides
/// functions that can provide information about the dynamic state of the HCSM
/// system such as how many root HCSMs are active.  Finally, it maintains
/// the HCSM system frame counter.
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsmCollection
{
public:
	CHcsmCollection( const double = 0.1, CCved* = NULL );   // default is 10 Hz
//	CHcsmCollection( const CHcsmCollection& );
//	CHcsmCollection& operator=( const CHcsmCollection& );
	virtual ~CHcsmCollection();


    typedef struct THeadlightSetting{
        THeadlightSetting();
	    bool   On;
	    double Azimuth;
	    double Elevation;
	    double BeamWidth;
	    double BeamHeight;
	    double ConstCoeff;
	    double LinearCoeff;
	    double QuadCoeff;
	    double HeightFade;
	    double Intensity;
	    double CutOffDist;
	    double LampSeparation;
	    double LampForwardOffset;
    } THeadlightSetting;

	inline int NumHcsm() const;
	inline int GetFrame() const;
	inline double GetTimeStepDuration() const;
	CHcsm* CreateHcsm( const string, const CSnoBlock& );
	bool DeleteHcsm( CHcsm* );
	void DeleteAllHcsm();
	void ExecuteAllHcsm();
	inline CCved* GetCved() { return m_pCved; }

	static int GetHcsmId( CHcsm* );
	CHcsm* GetHcsm( int ) const;
	CHcsm* GetHcsm( const string& ) const;

	CHcsm* GetExtDriverSurrogate( ) const;
	void  SetExtDriverSurrogate(CHcsm*);
	void SetHcsmDebugLevel( const int, const CHcsmDebugItem::ELevel );
	void SetHcsmDebugLevel( CHcsm*, const CHcsmDebugItem::ELevel );
	void SetHcsmDebugLevel( const CHcsmDebugItem::ELevel );

	void SetHcsmDebugMode( int, EDebugMode );
	void SetHcsmDebugMode( CHcsm*, EDebugMode );
	void SetHcsmDebugMode( EDebugMode );

	void LogDebugItem( const CHcsmDebugItem& );
	bool GetDebugItem( CHcsmDebugItem& );

	void ImRegister( int, int );
	bool GetImRegisterElem( TImRegisterData& elem );

	void MemLog( int hcsmId, int tag, const char* msg );
	void SetHcsmCreateLog(
				CHcsm* pHcsm,
				int hcsmType,
				const string cHcsmName,
				const CPoint3D& cPos
				);
	void SetHcsmDeleteLog( CHcsm* pHcsm, const CPoint3D& cPos );
	void SetHcsmActivateLog( CHcsm* pHcsm, const CPoint3D& cPos );
    const THeadlightSetting& GetHeadLightSettings() const;
    void SetHeadLightSettings(const THeadlightSetting&);
	static void SetCvedCreateLog(
				CHcsm* pHcsm,
				int cvedId,
				cvEObjType cvedType,
				const CPoint3D& cPos
				);
	static void SetTriggerFireLog(
				CHcsm* pHcsm,
				int instigatorHcsmId,
				int* pCandidateSet,
				int candidateSetSize
				);
	static void SetDialSettingLog(
				CHcsm* pHcsm,
				const string& cDialButtonName,
				const string& cSetting
				);
	static void SetDialSettingLog(
				CHcsm* pHcsm,
				const string& cDialName,
				double value
				);
	static void SetActionStartDataRedLog(
				int hcsmId,
				int segment,
				const char* pColumn,
				const char* pParams
				);
	static void SetActionStopDataRedLog(
				int hcsmId,
				int segment,
				const char* params
				);
	static void SetActionUseTrafManSetLog(
				int hcsmId,
				const string& cSetName
				);
	static void SetActionPlayAudioLog(
				int hcsmId
				// TBD
				);
	static void SetActionVehicleFailureLog(
				int hcsmId
				// TBD
				);
	static void SetActionTrafficLightLog(
				int hcsmId,
				int trafLightId,
				int trafLightState,
				double time
				);
	static void SetActionLogDataLog(
				int hcsmId,
				int logId,
				double value
				);
	static void SetActionTerminateSimulationLog(
				int hcsmId
				);
	static void SetActionPreposMotionLog(
				int hcsmId
				// TBD
				);
	static void SetActionTuneMotionLog(
				int hcsmId
				// TBD
				);
	static void SetActionPhoneCallLog(
				int hcsmId
				);
	static void SetActionResetDialLog(
				int hcsmId,
				const string& cDialName
				);
	static void SetActionSetDialLog(
				int hcsmId,
				const string& cDialName,
				const string& cValue
				);
	static void SetActionSetButtonLog(
				int hcsmId,
				const string& cDialName
				);
	static void SetActionSetVariableLog(
				int hcsmId,
				const string& varName,
				double value
				);
	static void SetActionCreateLog(
				int hcsmId
				);
	static void SetActionDeleteLog(
				int hcsmId
				);
	static void SetDiGuyPathInfo(int id, 
		                         const vector<float>&, 
		                         const vector<char>&, 
								 const vector<float>&);
	static void GetDiGuyPathInfo(int id, 
		                         vector<float>&, 
		                         vector<char>&, 
								 vector<float>&);
	static void AddDiGuyCommand(const CDiGuyUpdateCommand &);
	static void GetDiGuyCommandQueue(vector<CDiGuyUpdateCommand>& );
	static void AddDiGuyJointOverride(const CDiguyRotationOverride&);
	static void GetDiGuyJointOverrides(vector<CDiguyRotationOverride>&);
	static void clearDiGuyCommandQueue();
	static void clearDiGuyJointOverides();
    static void AddSpotlightCommand(const CSpotlightCommand&);
    static void GetSpotlightQueue(vector<CSpotlightCommand>& commands) ;
    static void ClearSpotlightQueue() ;
	void SetButtonSettingLog( CHcsm* pHcsm, const string& cDialButtonName );

	static void SetExprVariable( const string& cName, double value );
	static bool DoesRandomNumberGeneratorExist( const string& cName);
    static void CreateRandomNumberGenerator( const string& cName, const vector<long> &seed);
    static std::shared_ptr<std::mt19937> GetRandomNumberGenerator(const string& cName); 
	static bool ExprVariableExists( const string& cName );
	static double GetExprVariable( const string& cName );
	static void SetExprPosVariable( const string& cName, const CPoint3D &value );
	static bool ExprPosVariableExists( const string& cName );
	static CPoint3D GetExprPosVariable( const string& cName );
	static void ClearVariables();

	static void SetVisualDisplayText( const string& cDisplayText, int location = 1 );
	static void GetVisualDisplayText( string& displayText, int location = 1 );
	static void SetVisualSettings( const string& cVisualSettings );
	static void GetVisualSettings( string& visualSettings );
	static bool IsSimulationTerminated( void );
    static bool DoVarQueueOperation(const string& varQue);
    static bool GetVarQueueSize(const string& varQue, int &size);

	void PreloadFiles( const vector<string>& cFileNames );
	int GetPreloadFileHandle( const string& cFileName );
	bool GetNextPreloadFileSample( int fileHandle, string& data );

	// random nubmer generator
	CRandNumGen m_rng;

	// vector that includes all sol ids to be excluded
	vector<int> m_excludeSolIds;

	static void SetLastGoodPosition(float x, float y, float z);
	static void GetLastGoodPosition(float &x, float &y, float &z);

private:
	void ProcessHcsmCreate();
	void ProcessHcsmDelete();
	void ReadExternalBtnDialSettings();
	void SetHcsmButton(
				const int cvedId,
				const string& buttonName
				);
	void SetHcsmDial(
				const int cvedId,
				const string& dialName,
				const string& dialValue
				);
	inline const char* MyName();
	string GetActvLogFileName( void ) const;


	int m_numHcsm;                  // number of HCSMs
	double m_timeStepDuration;
	CHcsm* m_hcsmInstances[cMAX_ROOT_HCSM];
	list<int> m_freeList;
	CHcsm* m_ownDriverSurrogate; //< The ADO that simulates the ownship, null when not simulated
	static map<CHcsm*, int> m_hcsmMap;
	set<CHcsm*> m_hcsmToCreate;     // contains hcsms created in current frame
	set<CHcsm*> m_hcsmToDelete;     // holds deletion requests from crnt frame
	CCved* m_pCved;                 // pointer to CVED
	queue<TImRegisterData> m_imRegisterData;
	CMemoryLog m_memLog;

	// debugging queue
	queue<CHcsmDebugItem> m_DebugItems;

	// the HCSM parser generates the following function
	CHcsm* GetClassFromTemplateName( string, const CSnoBlock& );

	// vector that includes all sol object names
	// to be excluded
	vector<string> m_excludeSolNames;
	static map<string, double> m_exprVariables;
    static map<string, shared_ptr<mt19937>> m_randomGenerators; //<mersen twister based generators
	static map<string, CPoint3D> m_exprPosVariables;
    static map<string, vector< pair<string,double> > > m_varQueues;
	static map<int, vector<float> > m_sDiGuyPathInfo; //<path for the DiGuys
	static map<int, vector<float> > m_sDiGuyPathTimesInfo; //<optional time at node for DiGuys
	static map<int, vector<char> > m_sDiGuyPathActionInfo; //<optional action at node for DiGuys
	static vector<CDiGuyUpdateCommand> m_sDiGuyCommandQueue;
	static vector<CDiguyRotationOverride> m_sDiGuyJointOverrides;
	static vector<CSpotlightCommand> m_spotLightCommands;
	
    vector<TPreloadData> m_preloadFiles;
	map<int, TPreloadMapData> m_preloadHandleMap;
	int m_preloadHandleCounter;

	double m_currRate;           //< Current frame rate in MS;
	double m_currUpdateTime;    //< Current frame update time

	__int64 m_lastTicks;        //< CPU ticks since last update.

    THeadlightSetting m_ownshipHeadlights;

	static HANDLE m_sLastGoodLocationMutex; //< Mutex Loc for last good location
	static float  m_sLastGoodLocation[3]; //< last location the Ext Driver was on the Path

public:
	static int m_frame;                    // current frame number
	static bool  m_verbose;
	static char  m_sObjBtnDialValToHcsm[cOBJ_BTNDIAL_SIZE];
	static char  m_sExperiment[128];
	static char  m_sSubject[128];
	static char  m_sRun[128];
	static char  m_sRunInst[128];
	static CActvLog m_sActvLog;
	static bool  m_logActivities;
	static bool  m_printActvLog;
	static float m_sSCC_Scen_Pos_X_Crossbeam;
	static float m_sSCC_Scen_Pos_Y_Carriage;
	static float m_sSCC_Scen_Pos_Hex_X;
	static float m_sSCC_Scen_Pos_Hex_Y;
	static float m_sSCC_Scen_Pos_Hex_Z;
	static float m_sSCC_Scen_Pos_Hex_Roll;
	static float m_sSCC_Scen_Pos_Hex_Pitch;
	static float m_sSCC_Scen_Pos_Hex_Yaw;
	static float m_sSCC_Scen_Pos_TT;
	static float m_sAccelPedalPos;
	static float m_sBrakePedalForce;
	static float m_sSteeringWheelAngle;
	static short m_sCruiseControlIncoming;
	static short m_sCruiseControl;
	static int m_sCisTurnSignal;
	static float m_sSpeedometerBackdrive;
	static int m_sHorn;
	static int m_sHornFiltered;
	static int m_sRecasButton;
	static float m_sAuxiliaryButtons[20];
	static int m_sBrakeCond[cBRAKE_COND_SIZE];
	static int m_sTireCond[cTIRE_COND_SIZE];
	static int m_sSteeringCond;
	static int m_sAlertCond;
	static int m_sInfoCond;
	static int m_sCabComponentCond;
	static int m_sSCC_Scenario_Stop_Ind;
	static int m_sSCC_PlacePhoneCall;	// cell value
	static int m_sPlacePhoneCallAge;	// frame on which cell value was set
	static float m_sLogStreams[cNUM_LOG_STREAMS];
	static float m_sLogStreamsExt[cNUM_LOG_STREAMS]; //extened logstream
	static char m_sSCC_DataRed_Params[cNUM_DATARED_PARAMS_SIZE];
	static int m_sSCC_DataRed_Segments[cNUM_DATARED_SEGMENTS_SIZE];
	static float m_sFollowInfo[cFOLLOW_INFO_SIZE];
	static float m_sSensorInfo[cSENSOR_INFO_SIZE];
	static float m_sLaneDevInfo[cLANE_DEV_INFO_SIZE];
	static float m_sLaneDepartWarn[cLANE_DEPART_WARN_SIZE];
	static float m_sSplineDevInfo[cLANE_DEV_INFO_SIZE];
	static short m_sLaneMarkingInfo[cLANE_MARKING_INFO_SIZE];
	static float m_sOwnVehCurvature;
	static float m_sOwnVehDistOnPath;

    static double m_sSmoothForwardRoadVector[3];
	
    static bool m_sDisableCurvature;

	static TDynObjData m_sDynObjData;
	static int m_sDynObjDataSize;

	static int m_sHeadlightVisualState; //<are the headlights on or off? matches SCC_HeadlightVisualState
	static int m_sHeadlightScenarioControl; //<has the scenario asked to set the headlight state

	static TStatObjData m_sStatObjData;
	static int m_sStatObjDataSize;

	static int m_sChangedStatObjOption[cMAX_CHANGED_STAT_OBJ];
	static int m_sChangedStatObjId[cMAX_CHANGED_STAT_OBJ];
	static short m_sChangedStatObjDataSize;

	static TCabVisualOperations m_sCabOperations[cMAX_VISUAL_OPERATIONS];
	static int m_CabOperationsSize;
    static CHcsmStaticLock m_cabSettingsCriticalSection; //< Critical Section for Cab Settings
	static CHcsmStaticLock m_DiGuyPathCriticalSection; //< Critical adding pathNodes

	static TWriteCellData m_sWriteCellData[cMAX_WRITE_CELL_DATA_SIZE];
	static map<string,int> m_sAdditionalCellNamesToIds; //<cell names to ids that are not in m_sWriteCellData
	static int m_sWriteCellDataSize;

	static TScenarioWriteCellData m_sScenarioWriteCellData[cMAX_SCENARIO_WRITE_CELL_EVENTS];
	static int m_sScenarioWriteCellDataSize;

	static TScenarioWriteCellData m_sScenarioWriteUniformData[cMAX_SET_UNIFORM_CELL_EVENTS];
	static int m_sScenarioWriteUniformDataSize;

	static TAttachShaderCmd m_sAttachShaderCmds[cMAX_SET_UNIFORM_CELL_EVENTS];
	static int m_sAttachShaderCmdsDataSize;

	static TSetSwitchCmd m_sSetSwitchCmds[cMAX_SCENARIO_SET_SWITCH];
	static int m_sSetSwitchCmdsDataSize;

    static TSetEnvConditionCommand m_TSetEnvConditionCommand[cMAX_SET_UNIFORM_CELL_EVENTS];
    static int m_sSetEnvConditionCommands;

    static CHcsmStaticLock m_sLockVisualOptions;

	static float m_sFalseAlarmInfo[cFALSE_ALARM_SIZE];

	static map<int,string> m_sVisualDisplayText;

	
	static string m_sPlayAudioText;

	static string m_sVisualSettings;

	static int m_sWarningLights;
	static int m_sCollWarnAlg;
	static float m_sFcwInfo[4];
	static short m_sTimeToWarn;

	static bool m_sBUD_IsOn;//<Checks to see what warning systems are on
	static bool m_sLCW_IsOn;
	static bool m_sBLS_IsOn;
	static bool m_sFCW_IsOn;

	static bool m_sIsOnPath; //<checks to see if the ownveh is on the path
	static CVED::CPath m_sOwnshipPath;

	static ELdwStatus m_sLdwStatus;
	static EBswStatus m_sBswStatus;
	static EFcwStatus m_sFcwStatus;
	static float m_sBackUpDistance; //< current back up distance in feet

	static short m_sACC_Warning;		// is 1 short.  (-1 if cruise not set.  0 normally, 1 if warning)
	static short m_sCruise_State;		//<  (0 if off, 1 if normal cruise on, 2 if normal cruise set, 3 if acc on, 4 if acc set)
	static float m_sCruise_SetSpeed;	//< (set speed in mph)
	static float m_sACC_Gap;			//<gap in seconds
	static short m_sALF_State;			//< automatic lane following (0 if off, 1 if on)
	static int   m_sLDW_Severity;		//size of gap to lane line that LDW warning is issued (1=early or 2=late)
	static int   m_sFCW_Severity;		//size of TTC at which FCW warning is issued (1=early or 2=late)

	static float m_sSirenEffect;       //< Tells the Audio engine to play a Siren, 
	static float m_sSirenSpeed;		   //< The speed value for SirenEffect to reference (and be settable by trigger)


	static int m_sHour;
	static int m_sMinute;
	static bool m_sChangeTimeOfDay;
	static short m_sSensor_Config[cSENSOR_CONFIG_SIZE];
#ifdef AUDIO_TRIGGER_BYPASS
	static short m_sACC_On;
    static int m_sAudio_Trigger;
#endif
#ifdef TTA_DIST_FOR_ODSS
	static double m_sDistanceToInt;
#endif
	enum eFlashingLightMode {
		eNO_LIGHTS = 0,
		eMIRROR_LIGHTS,
		eA_PILLAR_LIGHTS,
		eBOTH
	};

	enum eBUDCameraDisplayMode {
		eNO_DISPLAY = 0,
		eINSTRUMENT_PANEL,
		eINTERIOR_MIRROR
	};
	enum eHeadlightControlMode{
		eDISABLE          = 0,
		eUSER_ENABLE      = 1, //allow the driver to turn on the headlights
		eHIGH_BEAM_ENABLE = 2 //allow the driver to turn on the highbeams
	};
	//static bool (*ReadCellNumeric)(const string&, int, float&); //<Function pointer to a "readcell function"
	static TReadCellFuncptr ReadCellNumeric;
	/// warning cue configuration
	static bool m_sHapticSeat_IsEnabled; //< whether haptic seat is enabled
	static eFlashingLightMode m_sFlashingLightMode; //< flashing lights mode
	static eBUDCameraDisplayMode m_sBUDCameraDisplayMode; //< backup camera and distance bar display mode
	static float m_sBUDBarWarnLowDist;  //< the backup distance at which the distance bar starts to grow
	static float m_sBUDBarWarnHighDist; //< the backup distance at which the distance bar reaches maximum warning level
	static bool m_sBSWCamera_IsEnabled; //< whether blind spot warning camera is enabled
	static bool m_sIPAlert_IsEnabled; //< whether alert icons in instrument panel is enabled

	static COnScreenGraph m_sDisplayGraph;
	static bool m_sGraphIsOn;

	static bool m_ExternalDriverRehearsalControl;
	static bool m_sMakeOwnVehicleChangeLanesLeft;
	static bool m_sMakeOwnVehicleChangeLanesRight;
	static float m_sOwnVehicleSpeed;
#ifndef USING_RTEXPSLITE
	static RTEX::SubsysBase *m_spRTEX;
#endif
};

#include "hcsmcollection.inl"

#endif // __CHCSMCOLLECTION_H
