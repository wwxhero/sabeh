//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2003-2004 by National Advanced Driving Simulator and
// Simulation Center, The University of Iowa and the University of Iowa.
// All rights reserved.
//
// Version:		$Id: ScenarioControl.h,v 1.43 2015/08/28 21:21:47 IOWA\dheitbri Exp $
// Author(s):   Yiannis Papelis
// Date:        December, 2003
//
// Description: Header file for the CScenarioControl class.
//
/////////////////////////////////////////////////////////////////////////////
#pragma once
#include <cvedpub.h>
#include "hcsmcollection.h"
#include <hcsmspec.h>
#include <snoparse.h>
#include <filename.h>
#include <ctype.h>
#include <staticobjmanagerutil.h>
#include <sol2.h>
#include <winhrt.h>
#include <pi_fstream>
#include <pi_iostream>
#include <pi_map>
#include <pi_string>
#include <pi_vector>

#define COLLISION_WITH_TERRAIN

using namespace std;

const int cTRAF_LIGHT_MAX_SIZE = 40;
const int cMAX_COLLISION_LIST_SIZE = 10;

/////////////////////////////////////////////////////////////////////////////
///\brief
///		This class is responsible for running scenario files
///\todo Class header needs to be filled out
///\ingroup HCSM
///
////////////////////////////////////////////////////////////////////////////
class CScenarioControl
{
public:
	CScenarioControl();
	~CScenarioControl();

	void SetDefaults( float behavDelta, int dynaMult, int verbose );
	const char* GetLastErrorString( void ) const;

	bool InitDistriEDOCtrlSim( bool simulateOwnVeh );
	bool InitDistriEDOCtrlSim(const char*, bool simulateOwnVeh);
	bool InitSimulation( const char*, bool simulateOwnVeh, bool isFile = true );
	void TerminateSimulation( void );
	bool GetDriverInitState( double ipos[3], double& heading );

	const char*       GetLriName( void ) const;
	const char*       GetScnFileName( bool expand=false ) const;
	const CSol&       GetSol( void ) const;
	CCved&            GetCved( void );
	CHcsmCollection&  GetHcsm( void );
	long              GetFrame( void );
	string            GetCabSolObjName( void ) const;
	const CSolObj*    GetCabSolObj( void ) const;
	string            GetCabTrailerSolObjName( void ) const;
	const CSolObj*    GetCabTrailerSolObj( void ) const;

	bool GetCabInformation(
				string& cabSolObjName,
				int& cabModelType,
				bool& showCab,
				CPoint3D& offset,
				string& cabTrailerSolObjName,
				int& cabTrailerModelType
				);

    bool IsCabShakeCrashEffectEnabled();
    bool IsBrokenWindshieldCrashEffectEnabled();

	bool SetCabType( const char* cabSOLName, const char* trailerSOLName = NULL );

	bool GetOwnshipLights(
				double& azimuth,
				double& elevation,
				double& beamWidth,
				double& beamHeight,
				double& constCoeff,
				double& linearCoeff,
				double& quadCoeff,
				double& heightFade,
				double& intensity,
				double& cutOffDist,
				double& lampSeparation,
				double& lampForwardOffset
				);

	void GetDateAndTime(
				int& year,
				int& month,
				int& day,
				int& hour,
				int& minute
				);

	void SetDateAndTime(
				int year,
				int month,
				int day,
				int hour,
				int minute
				);

	void GetTerrainCoordinates(
				float& longitude,
				float& latitude,
				float& altitude
				);

	void GetSkyModelValues(
				bool& sunEntityEnabled,
				bool& sunlightEnabled,
				bool& moonEntityEnabled,
				bool& moonlightEnabled,
				float& sunlightIntensity,
				float& moonlightIntensity
				);

	inline bool ScenarioHasBlanking( void ) const { return m_scenarioHasBlanking; };
	void GetBlankColor(float &r, float &g, float &b);
	void ExecMaintainer( void );
	void ExecBehaviors( void );
	void ExecDynamics( void );

	int GetScenarioHeadlightSetting( void );
	int GetNewAndDeletedDynamicObjects(
				const CObjTypeMask&	mask,		// mask of objects to look for
				int&				currCount,	// count of current objects
				int					currList[],	// list of current objects
				int&				newCount,	// count of new objects
				int					newList[],	// list of new objects
				int&				delCount,	// count of deleted objects
				int					delList[],	// list of deleted objects
				int					bufSize
				);

	int GetNewAndDeletedVirtualObjects(
				int&				currCount,	// count of current objects
				int					currList[],	// list of current objects
				int&				newCount,	// count of new objects
				int					newList[],	// list of new objects
				int&				delCount,	// count of deleted objects
				int					delList[],	// list of deleted objects
				int					bufSize
				);

	int GetObjectState( const int [], cvTObjState [], int );

	bool GetDebugItem( CHcsmDebugItem& item );
	void SetDebuggingState( int objId, EDebugMode state );

	// collision detection
//	static int   m_sCollisionPrevCvedObjId[cMAX_COLLISION_LIST_SIZE];
	static int   m_sCollisionCount;
	static short m_sCollisionCvedObjId[cMAX_COLLISION_LIST_SIZE];
	static short m_sCollisionCvedObjType[cMAX_COLLISION_LIST_SIZE];
	static int   m_sCollisionSolObjId[cMAX_COLLISION_LIST_SIZE];
	static int   m_sCollisionListSize;
//	static int   m_sTrailerCollisionPrevCvedObjId[cMAX_COLLISION_LIST_SIZE];
	static int   m_sTrailerCollisionCount;
	static short m_sTrailerCollisionCvedObjId[cMAX_COLLISION_LIST_SIZE];
	static short m_sTrailerCollisionCvedObjType[cMAX_COLLISION_LIST_SIZE];
	static int   m_sTrailerCollisionSolObjId[cMAX_COLLISION_LIST_SIZE];
	static int   m_sTrailerCollisionListSize;
	void  DetectCollisionsWithOwnship( void );

	// environement conditions and other VRED related data collection
	static float m_sOwnVehToLeadObjDist;
	static float m_sSpeedLimit;
	static float m_sVisibility;
	static float m_sWindSpeed;
	static float m_sWindDirection[2];

	void SetDefaultEnvValues( void );
	void ComputeEnvironmentalConditions(
				const CPoint3D& cOwnVehCartPos
				);
	void ComputeDynObjData( const CPoint3D& cOwnVehCartPos );
	void ComputeStatObjData( const CPoint3D& cOwnVehCartPos, bool onlyObjWithNonZeroAVState = true );
	void ComputeLeadVehInfo( const CPoint3D& cOwnVehCartPos );
	void ComputeLaneDepartureWarning( const CPoint3D& cOwnVehCartPos );
	void ComputeBackupDetectionWarning( const CPoint3D& cOwnVehCartPos );
	void ComputeOwnVehInfo( const CPoint3D& cOwnVehCartPos );
	void ComputeCollisionAvoidanceWarnings(
				const CPoint3D& cOwnVehCartPos,
				int& _rWarningLights,
				int _rSelectAlgorithm,
				float* pFcwInfo,
				short& timeToWarn,
				float frequencyOfExecution
				);
	static int TRWCAS(
				const CDynaParams& dynaParams,
				const CPoint3D rotObjPos,
				const float relHeading,
				const float range,
				const double ownVehVel,
				const float vel_closing,
				const bool useFAZ
				);
	static int LPWS(
				const CDynaParams& dynaParams,
				const CPoint3D rotObjPos,
				const float relHeading,
				const double ownVehVel
				);
	static bool ObjInFrontCone(
				const CDynaParams& dynaParams,
				const CPoint3D rotObjPos,
				const float relHeading,
				const float curvature,
				const double cMaxRange,
				double& bumperToBumperDist,
				double& angleInCone
				);
	static bool ObjInFrontCone(
				const CSolObjVehicle* cpSolObjVeh,
				const CPoint3D rotObjPos,
				const float relHeading,
				const float curvature,
				const double cMaxRange,
				double& bumperToBumperDist,
				double& angleInCone
				);
	static int FCW(
				const double ownVehVel,
				float* pFcwInfo,
				float frequencyOfExecution
				);
	// traffic light data
	static short m_sTrafLightSize;
	static short m_sTrafLightId[cTRAF_LIGHT_MAX_SIZE];
	static short m_sTrafLightState[cTRAF_LIGHT_MAX_SIZE];
	void ComputeTrafLightData( const CPoint3D& cOwnVehCartPos );


	void GetVirtObjectsBlocks(const std::vector<CSnoBlock>* &pBlocks);
	void GetSolColorNames(const std::map<string, set<int> >* &pSets) ;
	void EnableRehearsalControlOfExternalDriver(bool);
	void SetRehearsalSpeedOfExternalDriver(float TargetVelocity,string TypeOfVelocity,bool isStopping);
	void SetRehearsalChangeLaneRightExternalDriver();
	void SetRehearsalChangeLaneLeftExternalDriver();
	CCved*            m_pCved;
	CHcsmCollection*  m_pRootColl;
    const string& GetLriName();
private:
	bool InitSimulation( CSnoParser& parser, bool simulateOwnVeh );
	bool    m_haveError;
	char    m_lastError[512];

	CHeaderParseBlock* m_pHdrBlk;	// ptr to header block

	vector<int>  m_objFromLastTime;	// helps keep track of creations/deletions
	vector<int>  m_virtualObjFromLastTime;	// helps keep track of creations/deletions

	vector<CSnoBlock> m_virtualObjects; //<Copy of all Sno Blocks of Virt Objects
	map<string, set<int> > m_modelOptionPairs; //<We only need to precreate 1 object 1 color combo

	float   m_behavDeltaT;			// behaviors execution period
	int     m_dynaMult;				// how many times dynamics runs per behavior
	int     m_verbose;				// verbosity level
	bool    m_scenarioHasBlanking;
	float   m_blankColor[3];

	float	m_sirenSpeed;			//<M/S when Driver is over this speed, we set HCSM::SirenEffect

	int m_prevLdwFrame;
	ELdwStatus m_prevLdwStatus;

	bool ProcessFile(
			const string& cFileName,
			const char* cEnvVarName,
			string& fileNameFull,
			bool testFileExistence = true
			);
	string m_lriFileNamePrefix;
	string m_scnFileName;
	string m_scnFileNameFull;
	string m_cabSolObjName;
	string m_cabTrailerSolObjName;
	bool m_showCab;

//	CVED::CPath m_ownshipPath;

	bool m_ownshipHeadlightsOn;
	double m_ownshipHeadlightsAzimuth;
	double m_ownshipHeadlightsElevation;
	double m_ownshipHeadlightsBeamWidth;
	double m_ownshipHeadlightsBeamHeight;
	double m_ownshipHeadlightsConstCoeff;
	double m_ownshipHeadlightsLinearCoeff;
	double m_ownshipHeadlightsQuadCoeff;
	double m_ownshipHeadlightsHeightFade;
	double m_ownshipHeadlightsIntensity;
	double m_ownshipHeadlightsCutOffDist;
	double m_ownshipHeadlightsLampSeparation;
	double m_ownshipHeadlightsLampForwardOffset;

	short m_year;
	short m_month;
	short m_day;
	short m_hour;
	short m_minute;

	float m_longitude;
	float m_latitude;
	float m_altitude;

	bool m_sunEntityEnabled;
	bool m_sunlightEnabled;
	bool m_moonEntityEnabled;
	bool m_moonlightEnabled;
	float m_sunlightIntensity;
	float m_moonlightIntensity;

	const CSolObj* m_cpCabSolObj;
	const CSolObj* m_cpCabTrailerSolObj;

	CRoadPos m_ownVehRoadPos;
	CRoadPos m_intVehRoadPos; //<location does not change durring

	void  ResetCollisionDetectionVars( void );

	struct SCollisionInfo {
		SCollisionInfo()
		{
			StartCounting = false;
			CollisionCount = 0;
			NonCollisionCount = 0;
			NonCollisionStreak = 0;
		}

		bool StartCounting;
		int CollisionCount;
		int NonCollisionCount;
		int NonCollisionStreak;
	};

	map<int, SCollisionInfo*> m_CollisionInfoMap;
	map<int, SCollisionInfo*> m_TrailerCollisionInfoMap;

    std::string m_lriName;

#ifdef COLLISION_WITH_TERRAIN
	map<int, short> m_TerrainObjIdToTypeMap;
	// terrain obj type
	// 500: non-collidable terrain (should not appear in the collision object list)
	// 501: wall
	// 502: bumper
	// 503: struct
#endif

	void FindPreCreateObjects(CSnoParser::TIterator pBlock,CSnoParser::TIterator pEnd);

	int UpdateCollisionObjList(
			int	&collisionObjListSize,
			short collisionObjList[cMAX_COLLISION_LIST_SIZE],
			map<int, SCollisionInfo*> &collisionInfo,
			vector<int> &newCollisionObjs
			);

	IExternalObjectControl* m_pExternalObjCtrl;
};
