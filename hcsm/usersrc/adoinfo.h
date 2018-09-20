/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: adoinfo.h,v 1.84 2018/05/17 16:27:08 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         December, 1999
 *
 * Description:  Interface for the CAdoInfo class.
 *
 ****************************************************************************/

#ifndef __CADOINFO_H
#define __CADOINFO_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#endif

#include <pi_iostream>
#include <pi_string>
using namespace std;

#include <cvedpub.h>
using namespace CVED;

#include <adoparseblock.h>
#include "util_ado.h"
#include "curvature.h"
#include "maintaingapinfo.h"
#include "objectinitcond.h"
#include "vehdyncommand.h"

const double cPATH_REFRESH_TIME = 5.0;  // seconds
const double cDefaultMaxSteerRateRadspS =  2.256; //<The defualt max steering rate used by the dynamics
class CLaneChangeCond;

// Data structures to facilitate the randomized lane deviation

enum TLaneDevModel {
	ELaneDevNoModel = -1,
	ELaneDevRamps,
	ELaneDevSin,
	ELaneDevRampsSin,
	ELaneDevExpression
};

enum EForcedVelMode {
	eFV_NONE = -1,
	eFV_OV_VEL,
	eFV_OV_TRACK,
	eFV_SIN,
	eFV_NORMAL,
	eFV_STAY
};

//
// This class holds the information about the model used by the vehicle
// for randomized lane deviation behavior.
//
class CRandLaneDevInfo {
public:
	CRandLaneDevInfo();
	~CRandLaneDevInfo();

	TLaneDevModel    m_Model;					// which model is in use

	// all parameters are uniform random numbers between the XXX1 and XXX2 
	// variables.  Each vehicle randomizes in isolation from others.
	// The variable with no 1 or 2 at the end is the actual sample

	bool             m_Enable;					// if false, no lane deviation

	CRandNumGen      m_Rnd;						// for random numbers

	// ramp model parametesr
	double      m_Rise, m_Rise1, m_Rise2;			// rise time, in seconds
	double      m_Fall, m_Fall1, m_Fall2;			// fall time, in seconds
	double      m_Idle, m_Idle1, m_Idle2;			// idle time, in seconds
	double      m_RampAmp,  m_RampAmp1, m_RampAmp2;	// amplitute, in feet (?)
	double      m_Pos;								// deviation sign (+-1)
	double      m_RampStartTime;	// time variable for ramp calculations

	// sin model parameters
	double      m_Bias, m_Bias1, m_Bias2;			// baseline offset, in feet
	double      m_SinAmp, m_SinAmp1, m_SinAmp2;		// sin wave amplitute, in feet
	double      m_Frequ, m_Frequ1, m_Frequ2;			// sin wave frequency
	double      m_Phase, m_Phase1, m_Phase2;			// sin wave phase

	// how often to re-randomize the model and the parameters
	double      m_ReevalTime, m_ReevalTime1, m_ReevalTime2;

	// down timer/counter for re-randomizing the model & parameters
	long       m_Timer;

	double      m_Last;								// last value produced by model
};

// Data structures that facilitate the following algorithms

enum TFollowState { 
	EFolStNone= -1, 
	EFolStStop, 
	EFolStAccel, 
	EFolStDeccel, 
	EFolStSteady, 
	EFolStEmerg 
};

//////////////////////////////////////////////////////////////////////////////
//
// Facilitate following algorithm; specifically delayed reaction
//
//////////////////////////////////////////////////////////////////////////////
class CFollowInfo {
public:
	CFollowInfo() : m_OldState(-1), m_OldAccel(0.0), 
		m_Timer(0.0), m_Counting(false) {};
	~CFollowInfo() {};

	int       m_OldState;
	double    m_OldAccel;
	double    m_Timer;
	bool      m_Counting;
};


//////
//
// Normal follow controller parameters
//
/////
struct TNormFollowParams {
	double distKp;			// proportional gain for distance tracking
	double distKi;			// integral gain for distance tracking
	double distKd;			// derivative gain for distance tracking
	double velKp;			// proportional gain for velocity tracking

	double vel2Kp;			// proportional gain for tracking app velocity

	double ovspeedClip;		// overspeed clipping, follow speed will not exceed
							// leader's speed times this
	double clipVelRange;	// overspeed clipping only when within this range

	double posAccClip;		// maximum acceleration produced
	double negAccClip;		// minimum deceleration produced, except for final stop
	double bumpStopDist;	// bumper to bumper stopping distance
	double appDecRate;		// approach deceleration rate; used when approaching
	double maxAppSpeed;		// max speed (mi/hr) when approaching a stopped leader
							// a stopped vehicle
	bool   accelToCatchUp;	// if set, controller will accelerate as well as dec
};

//////
//
// Emergency follow controller parameters
//
/////
struct TEmergFollowParams {
	double distKp;			// proportional gain for distance tracking
	double distKi;			// integral gain for distance tracking
	double distKd;			// derivative gain for distance tracking
	double velKp;			// proportional gain for velocity tracking
	double velKi;			// integral gain for velocity tracking
	double velKd;			// derivative gain for velocity tracking
	double aclKp;			// proportional gain for acceleration tracking
	double posAccClip;		// output no bigger than that
	double negAccClip;		// output no less than that
};

//////
//
// Overall follow controller parameters
//
//////
struct TFollowParams {
	bool                folTimeMode;	// if rue, follow time mode, else fol dist
	double              folValue;		// follow time or distance
	double               ttcThres1;		// ttc less than this, 100% emerg brk
	double               ttcThres2;		// ttc less than this, blend 
	double               ttcThres3;		// ttc more than this, no follow
	double               engageThres;	// when dist less, follow is engaged

	bool                useReactDelay;		 // unless set, delays have no effect
	double              stopToStartDelay;	 // reaction delay 
	double              steadyToDeccelDelay; // reaction delay before braking
	double              deccelToAccelDelay;	 // more reaction delay
	double              steadyToAccelDelay;

//	double percThres;		// percent change before doing anything (NOT USED)
	TNormFollowParams   normal;			// normal controller parameters
	TEmergFollowParams  emerg;			// emergency controller parameters
	TNormFollowParams   maintainGap;    // maintain gap normal controller pars
};



typedef struct{
	CPoint3D pos;
	double   radius;
	int      leadObjId;
} TMergeInfo;



typedef struct TVelCntrl
{
	double initVel;
	bool   initOvVel;
	double refreshTime;
	CAdoParseBlock::CDelay distribution;
	double targetVel;               // m/s
	int    targetVelDurationCount;  // frames
	bool   followSpeedLimit;
} TVelCntrl;

typedef struct TLaneChange
{
	double initDelay;
	double turnSignalTime;
	double urgency;
	double steeringForce;
    double maxLatOffset;
    double minLookAhead; //0.5 default
	bool   enableLosingCorridor;
	bool   enableHighwayMerge;
	bool   enableAvoidMergingObject;
	bool   enableSlowMovingObject;
	bool   enableVerySlowMovingObject;
	bool   enableNonPassingLane;
} TLaneChange;

//////////////////////////////////////////////////////////////////////////////
//
// This class contains the information structure for the ADO object.
//
//////////////////////////////////////////////////////////////////////////////
class CAdoInfo
{

public:
	CAdoInfo( const CRoadPos&, const double, const cvEObjType );
	CAdoInfo( const CAdoInfo& );
	CAdoInfo& operator=( const CAdoInfo& );
	~CAdoInfo();
	enum ERunMode { eAUTONOMOUS, eREMOTE_CONTROL };

	bool IsExpiredPathRefreshCounter();
	void ResetPathRefreshCounter();
	void SetCurrRoadPos( const CRoadPos& );
	void SyncRoadPosVars();

	string      m_objName;
	CDynObj*    m_pObj;
	cvEObjType  m_objType;
	long        m_ageFrame;
	double      m_timeStepDuration;
	int         m_pathRefreshCounter;
	CRoadPos    m_roadPos;
	CLane       m_currLane;
	CRoad       m_currRoad;
	CIntrsctn   m_currIntrsctn;
	CCrdr       m_currCrdr;
	CRoadPos    m_prevRoadPos;
	bool        m_offroad;
	CPoint3D    m_offroadCartPos;
	CRoadPos    m_offroadGuideRoadPos;
	CLane       m_trackLane;
	EInitCond   m_initCondState;
	EDynaFidelity m_dynModel;
	double      m_currVel;
	double      m_prevVel;
	double      m_aggressiveness;
	ERunMode    m_runMode;
	ofstream*   m_pLogFile;
	bool        m_cmdMode;
	CVehDynCommand m_commands;
	int         m_rngStreamId;
	CPath*      m_pPath;
	bool        m_avoidTurnsAtIntersection;

	TVelCntrl   m_velCntrl;

	CCurvature  m_curvature;
	double      m_maxAccelDueToCurv;  // maximum accel due to curvature

	CLaneChangeCond* m_pCurrLcCond;
	ELcCondition m_pastLcCond;
	long        m_prevLcCompleteFrame;
	bool        m_targLaneSplitsFromCurrLane;
	double      m_lcSinAngle;
	double      m_lcLatDistTraveled;
	double      m_lcTotalLatDist;
	double      m_lcTargPointLatDistToTarget;
	double      m_lcTargPointTotalLatDist;
	double      m_lcMaxSinAngle;
	TU16b       m_lightState;
	TLaneChange m_lcInfo;
	bool        m_nextTurnSignalComputed;
	TU16b       m_nextTurnSignal;
	double      m_signalTimeFromDial;
	bool        m_hasSignalTimeFromDial;
	bool        m_leftLaneChangeButton;
	bool        m_rightLaneChangeButton;
	bool        m_reporjectAndResetLaneOffsetButton; //<tells the ADO to reset the lane offset dial + track what ever lane its in
	double      m_laneChangeUrgency;
	int         m_laneChangeWaitCount;
	int         m_laneChangeDurationCount;
	int         m_lcInhibitCount;				// count down for inhibit
	int         m_lcStatus;
	int         m_lcStatusCount;
	bool	    m_haveForcedLaneOffset;
	double	    m_forcedLaneOffset;
	double	    m_prevForcedLaneOffset;    //<our old offset
	double		m_forcedLaneOffsetUrgency;
    double      m_forcedLaneOffsetTurnRate;
	int		    m_forcedOffsetFrameCounter;
	int			m_forcedOffsetDelay; //<how many frames to delay the offset before being applied
	double		m_forcedOffsetMaxSteerDistance; //<how far we can move the target point while marking a LC op
	double		m_forcedOffsetMaxSteerForce; //<scaler used to control speed the target point moves
	double		m_forcedOffsetTolerance; //<how close do we have to be to the target point before our operation is done
    double      m_forcedOffsetMinLookAhead; //<min lookahead distance;
	int         m_visualStateFromDial;
	int         m_visualStateFromDialCount;
	int         m_audioStateFromDial;
	int         m_audioStateFromDialCount;
	bool        m_hasMoved; //<if the ADO has never moved, assume the ADO is "parked", and leave the brake lights off...

	double      m_objLength;
	double      m_objWidth;
	vector<CCved::TObjListInfo> m_objsFwd;      // objects lists
	vector<CCved::TObjListInfo> m_objsBack;
	vector<CCved::TObjListInfo> m_objsBack2;
	vector<CCved::TObjListInfo> m_objsApprch;
	vector<CCved::TObjListInfo> m_objsOncom;

	CMaintainGapInfo m_maintainGap;

	bool        m_isFollowTime;             // indicates if follow min/max contain
	double      m_followMin;                // time or dist values
	double      m_followMax;
	long        m_followRefreshFrame;
	bool        m_imStopHasValue;
	double      m_imStopHldOfsDist;
   

	char        m_acgoutHcsmText[5];		// indicates whose hcsm accel is used
	vector<TMergeInfo> m_mergeInfo;
	bool	    m_mergeLaneChangeIsNeeded;
	CCrdr	    m_mergeTargCrdr;

	TFollowParams m_FollowParams;			// follow algorithm parameters
	CFollowInfo   m_FollowInfo;				// follow algorithm needs this 

	double      m_followTtc;                // follow computes; others read
											// 999 is sentinel for no value
	double      m_followDist;               //in feet
    int         m_followTarget;

	double		m_lastOvel;					// Last Own Vehicle Velocity
	double      m_forcedVel;				// if > 0, forced velocity
	double      m_forcedVelAccel;           // accel to reach forced vel, if fvel>0
	int         m_forcedVelStart;           // frame when forced vel was applied
	int         m_forcedVelEnd;             // frame when forced vel should end (if > 0)
	EForcedVelMode m_forcedVelMode;         // type of forced velocity setting
	double	    m_forcedVelChange;          // for ovtrack increment/decrement field
	int         m_forcedVelFileHandle;      // for file settings
	double		m_forcedVelFileMultiplier;  // multiplier for file settings

	bool        m_DisableLaneDevWhenForcedOffsetDial;
	CRandLaneDevInfo m_randLaneDev;			// random lane deviation model

	CRandLaneDevInfo m_lcNeutralizeOffset;	// used for calculation of offset 
											// during LcNeutral state in lane 
											// change
	string		m_forcedVelExpression;		///< Used for the expression forced Vel
	double		m_signalFadeIn;				///< Used to fade in signal components for forced velocity
	bool        m_autoControlBrakeLightState;
	bool        m_autoControlHeadLightsState;

	int         m_gapStartFrame; //< start of Maint Gap, needed for expressions

	float       m_fwdDistThreshold;
	float		m_backDistThreshold;

	float       m_fwdTimeThreshold;
	float		m_backTimeThreshold;

    float       m_forcedPercent;				//< How Much we are mixing in external steering override with  
    float       m_forcedSteerAngle;				//< steering angle
    float       m_forcedSteerDist;				//< how far to project out the target point for forced steering override
};

#endif // __CADOINFO_H
