/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: adoinfo.cxx,v 1.77 2018/05/17 16:27:08 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    December, 1999
 *
 * Description:  Contains the implementation for the CAdoInfo class.
 *
 ****************************************************************************/

#include "adoinfo.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CAdoInfo::CAdoInfo(	
			const CRoadPos& roadPos, 
			const double timeStepDur,
			const cvEObjType objType 
			):
	m_objName( "" ), 
	m_objType( objType ),
	m_ageFrame( 0 ),
	m_prevRoadPos( roadPos ),
	m_offroad( false ),
	m_offroadGuideRoadPos( roadPos ),
	m_trackLane(),
	m_pObj( NULL ),
	m_initCondState( eWAIT ),
	m_dynModel( eNON_LINEAR ),
	m_currVel( -1.0 ),
	m_prevVel( -1.0 ),
	m_aggressiveness( -1.0 ),
	m_runMode( eAUTONOMOUS ),
	m_pLogFile( NULL ),
	m_cmdMode( false ),
	m_rngStreamId( -1 ),
	m_pPath( NULL ),
	m_avoidTurnsAtIntersection( false ),
	m_curvature( objType ),
	m_maxAccelDueToCurv( 100 ),		// large value; acts as sentinel
	m_pCurrLcCond( NULL ),
	m_pastLcCond( eLC_NONE ),
	m_prevLcCompleteFrame( -1 ),
	m_targLaneSplitsFromCurrLane( false ),
	m_lcSinAngle( 0.0 ),
	m_lcLatDistTraveled( 0.0 ),
	m_lcTotalLatDist( 0.0 ),
	m_lcMaxSinAngle( 0.0 ),
	m_lightState( 0 ),
	m_nextTurnSignalComputed( false ),
	m_nextTurnSignal( 0 ),
	m_signalTimeFromDial( -1 ),
	m_hasSignalTimeFromDial( false ),
	m_leftLaneChangeButton( false ),
	m_rightLaneChangeButton( false ),
    m_reporjectAndResetLaneOffsetButton(false),
	m_laneChangeUrgency( -1.0 ),
	m_laneChangeWaitCount( -1 ),
	m_laneChangeDurationCount( -1 ),
	m_lcInhibitCount( -1 ),
	m_objLength( 0.0 ),
	m_objWidth( 0.0 ),
	m_lcStatus( -1 ),
	m_lcStatusCount( -1 ),
	m_isFollowTime( true ),
	m_followMin( 1.0 ),
	m_followMax( 1.5 ),
	m_followRefreshFrame( -1 ),
	m_imStopHasValue( false ),
	m_imStopHldOfsDist( 0.0 ),
	m_forcedVel( -1.0 ),
	m_forcedVelAccel( 0.0 ),
	m_forcedVelStart( -1 ),
	m_forcedVelEnd( -1 ),
	m_forcedVelMode( eFV_NONE ),
	m_forcedVelChange( 0.0 ),
	m_forcedVelFileHandle( -1 ),
	m_forcedLaneOffset( 0.0 ),
	m_haveForcedLaneOffset( false ),
	m_forcedOffsetFrameCounter( 0 ),
	m_followTtc( 9999.0 ),
	m_followDist( 9999.0 ),
    m_visualStateFromDial( -1 ),
	m_visualStateFromDialCount( -1 ),
    m_audioStateFromDial( -1 ),
	m_audioStateFromDialCount( -1 ),
	m_lastOvel(-1),
	m_hasMoved(false),
    m_gapStartFrame(-1),
    m_forcedPercent(0),				 
    m_forcedSteerAngle(0),				
    m_forcedSteerDist(1.0)				
{

	SetCurrRoadPos( roadPos );
	m_timeStepDuration = timeStepDur;
	ResetPathRefreshCounter();

	// velocity control
	m_velCntrl.initVel                = -1;
	m_velCntrl.initOvVel			  = false;
	m_velCntrl.refreshTime            = 45;  // seconds
	m_velCntrl.distribution.m_type    = CAdoParseBlock::eFIXED;
	m_velCntrl.distribution.m_param1  = -1;
	m_velCntrl.distribution.m_param2  = -1;
	m_velCntrl.targetVel              = -1;  // m/s
	m_velCntrl.targetVelDurationCount = -1;  // sec
	m_velCntrl.followSpeedLimit       = false;

	// lane change
	m_lcInfo.initDelay = 1.0;                // sec
	m_lcInfo.turnSignalTime = 2.0;           // sec
	m_lcInfo.urgency = 0.5;
	m_lcInfo.steeringForce = 1.0;
	m_lcInfo.enableLosingCorridor = true;
	m_lcInfo.enableHighwayMerge = true;
	m_lcInfo.enableAvoidMergingObject = true;
	m_lcInfo.enableSlowMovingObject = true;
	m_lcInfo.enableVerySlowMovingObject = true;
	m_lcInfo.enableNonPassingLane = true;

	// give different random seeds for lane deviation
	if (roadPos.IsValid()){
		long distSeed = abs(static_cast<long>(roadPos.GetDistance()) + 1); //seed needs to be non 0
		m_randLaneDev.m_Rnd.SetAllSeeds(distSeed, distSeed);
		m_randLaneDev.m_Rnd.SetSeed(distSeed, distSeed+1, 0);
		m_randLaneDev.m_Timer = -1;  // this will trigger immediate re-eval
	}else{
		long someVal = rand()%500+1;
		m_randLaneDev.m_Rnd.SetAllSeeds(someVal, someVal);
		m_randLaneDev.m_Rnd.SetSeed(someVal, someVal+1, 0);
		m_randLaneDev.m_Timer = -1;  // this will trigger immediate re-eval
	}

	// setup default follow parameters
	m_FollowParams.folTimeMode  = true;		// follow time
	m_FollowParams.folValue     = 1.5;
	m_FollowParams.ttcThres1    = 4.0;
	m_FollowParams.ttcThres2    = 6.5;
	m_FollowParams.ttcThres3    = 40.0;
	m_FollowParams.engageThres  = 100.0;
	m_FollowParams.useReactDelay       = true;
	m_FollowParams.steadyToAccelDelay  = 0.5;
	m_FollowParams.steadyToDeccelDelay = 0.8;
	m_FollowParams.deccelToAccelDelay  = 1.0;
	m_FollowParams.stopToStartDelay    = 1.5;

	m_FollowParams.normal.distKp       = 0.6;
	m_FollowParams.normal.distKi       = 0.0;
	m_FollowParams.normal.distKd       = -0.08;
	m_FollowParams.normal.velKp        = 3.1;
	m_FollowParams.normal.vel2Kp       = 0.7;
	m_FollowParams.normal.ovspeedClip  = 1.2;
	m_FollowParams.normal.clipVelRange = 100.0;
	m_FollowParams.normal.posAccClip   = 2.3;
	m_FollowParams.normal.negAccClip   = -4.0;
	m_FollowParams.normal.bumpStopDist = 7.0;
	m_FollowParams.normal.appDecRate   = -4.0;
	m_FollowParams.normal.maxAppSpeed  = 15;		// In miles/hr
	m_FollowParams.normal.accelToCatchUp = true;

	m_FollowParams.emerg.distKp     = 0.95;
	m_FollowParams.emerg.distKi     = 0.0;
	m_FollowParams.emerg.distKd     = 0.0;
	m_FollowParams.emerg.velKp      = 1.5;
	m_FollowParams.emerg.velKi      = -1.1;
	m_FollowParams.emerg.velKd      = 0.0;	
	m_FollowParams.emerg.aclKp      = 0.0;
	m_FollowParams.emerg.posAccClip = 0.0;
	m_FollowParams.emerg.negAccClip = -11.0;

	// setup default maintain gap parameters
	m_FollowParams.maintainGap = m_FollowParams.normal;
	m_FollowParams.maintainGap.distKp       = 6.0;
	m_FollowParams.maintainGap.distKi       = 0.0;
	m_FollowParams.maintainGap.distKd       = -0.08;
	m_FollowParams.maintainGap.velKp        = 5;
	m_FollowParams.maintainGap.vel2Kp       = 0.0;
	m_FollowParams.maintainGap.ovspeedClip  = 1.5;
	m_FollowParams.maintainGap.clipVelRange = 50.0;
	m_FollowParams.maintainGap.posAccClip   = 10.0;
	m_FollowParams.maintainGap.negAccClip   = -10.0;
	m_FollowParams.maintainGap.bumpStopDist = 7.0;
	m_FollowParams.maintainGap.appDecRate   = -2.5;
	m_FollowParams.maintainGap.maxAppSpeed  = 15;		// In miles/hr
	m_FollowParams.maintainGap.accelToCatchUp = true;
}

CAdoInfo::CAdoInfo( const CAdoInfo& cRhs )
{

	// call the assignment operator
	*this = cRhs;

}

CAdoInfo& 
CAdoInfo::operator=( const CAdoInfo& cRhs )
{

	// check to make sure that object passed in is not me
	if( this != &cRhs ) 
	{
		m_objName                  = cRhs.m_objName;
		m_objType                  = cRhs.m_objType;
		SetCurrRoadPos( m_roadPos );
		m_prevRoadPos              = cRhs.m_prevRoadPos;
		m_offroad                  = cRhs.m_offroad;
		m_offroadCartPos           = cRhs.m_offroadCartPos;
		m_offroadGuideRoadPos      = cRhs.m_offroadGuideRoadPos;
		m_trackLane                = cRhs.m_trackLane;
		m_pObj                     = cRhs.m_pObj;
		m_initCondState            = cRhs.m_initCondState;
		m_dynModel                 = cRhs.m_dynModel;
		m_currVel                  = cRhs.m_currVel;
		m_prevVel                  = cRhs.m_prevVel;
		m_velCntrl                 = cRhs.m_velCntrl;
		m_aggressiveness           = cRhs.m_aggressiveness;
		m_runMode                  = cRhs.m_runMode;
		m_pLogFile                 = cRhs.m_pLogFile;
		m_cmdMode                  = cRhs.m_cmdMode;
		m_commands                 = cRhs.m_commands;
		m_rngStreamId              = cRhs.m_rngStreamId;
		m_pPath                    = cRhs.m_pPath;
		m_avoidTurnsAtIntersection = cRhs.m_avoidTurnsAtIntersection;
		m_curvature                = cRhs.m_curvature;
		m_maxAccelDueToCurv        = cRhs.m_maxAccelDueToCurv;
		m_pCurrLcCond              = cRhs.m_pCurrLcCond;
		m_pastLcCond               = cRhs.m_pastLcCond;
		m_prevLcCompleteFrame      = cRhs.m_prevLcCompleteFrame;
		m_targLaneSplitsFromCurrLane = cRhs.m_targLaneSplitsFromCurrLane;
		m_lcSinAngle               = cRhs.m_lcSinAngle;
		m_lcLatDistTraveled        = cRhs.m_lcLatDistTraveled;
		m_lcTotalLatDist           = cRhs.m_lcTotalLatDist;
		m_lcMaxSinAngle            = cRhs.m_lcMaxSinAngle;
		m_lightState               = cRhs.m_lightState;
		m_lcInfo                   = cRhs.m_lcInfo;
		m_nextTurnSignalComputed   = cRhs.m_nextTurnSignalComputed;
		m_nextTurnSignal           = cRhs.m_nextTurnSignal;
	    m_signalTimeFromDial       = cRhs.m_signalTimeFromDial;
		m_hasSignalTimeFromDial    = cRhs.m_hasSignalTimeFromDial;
		m_leftLaneChangeButton     = cRhs.m_leftLaneChangeButton;
		m_rightLaneChangeButton    = cRhs.m_rightLaneChangeButton;
        m_reporjectAndResetLaneOffsetButton = cRhs.m_reporjectAndResetLaneOffsetButton;
		m_laneChangeUrgency        = cRhs.m_laneChangeUrgency;
		m_laneChangeWaitCount      = cRhs.m_laneChangeWaitCount;
		m_laneChangeDurationCount  = cRhs.m_laneChangeDurationCount;
		m_lcInhibitCount           = cRhs.m_lcInhibitCount;
		m_objLength                = cRhs.m_objLength;
		m_objWidth                 = cRhs.m_objWidth;
		m_objsFwd                  = cRhs.m_objsFwd;
		m_objsBack                 = cRhs.m_objsBack;
		m_objsBack2                = cRhs.m_objsBack2;
		m_objsApprch               = cRhs.m_objsApprch;
		m_objsOncom                = cRhs.m_objsOncom;
		m_maintainGap              = cRhs.m_maintainGap;
		m_lcStatus                 = cRhs.m_lcStatus;
		m_lcStatusCount            = cRhs.m_lcStatusCount;
		m_isFollowTime             = cRhs.m_isFollowTime;
		m_followMin                = cRhs.m_followMin;
		m_followMax                = cRhs.m_followMax;
		m_followRefreshFrame       = cRhs.m_followRefreshFrame;
		m_imStopHasValue           = cRhs.m_imStopHasValue;
		m_imStopHldOfsDist         = cRhs.m_imStopHldOfsDist;
		m_FollowParams             = cRhs.m_FollowParams;
		m_FollowInfo               = cRhs.m_FollowInfo;
		m_forcedVel                = cRhs.m_forcedVel;
		m_forcedVelAccel           = cRhs.m_forcedVelAccel;
		m_forcedVelStart           = cRhs.m_forcedVelStart;
		m_forcedVelEnd             = cRhs.m_forcedVelEnd;
		m_forcedVelMode            = cRhs.m_forcedVelMode;
		m_forcedVelChange          = cRhs.m_forcedVelChange;
		m_forcedVelFileHandle      = cRhs.m_forcedVelFileHandle;
		m_forcedLaneOffset         = cRhs.m_forcedLaneOffset;
		m_haveForcedLaneOffset     = cRhs.m_haveForcedLaneOffset;
		m_forcedOffsetFrameCounter = cRhs.m_forcedOffsetFrameCounter;
		m_randLaneDev              = cRhs.m_randLaneDev;
		m_followTtc                = cRhs.m_followTtc;
		m_followDist               = cRhs.m_followDist;
        m_followTarget             = cRhs.m_followTarget;
		m_visualStateFromDial      = cRhs.m_visualStateFromDial;
		m_visualStateFromDialCount = cRhs.m_visualStateFromDialCount;
		m_audioStateFromDial       = cRhs.m_audioStateFromDial;
		m_audioStateFromDialCount  = cRhs.m_audioStateFromDialCount;
        m_gapStartFrame            = cRhs.m_gapStartFrame;
        m_forcedPercent            = cRhs.m_forcedPercent   ;				//< How Much we are mixing in external steering override with  
        m_forcedSteerAngle         = cRhs.m_forcedSteerAngle;				//< steering angle
        m_forcedSteerDist          = cRhs.m_forcedSteerDist;				//< how far to project out the target point for forced steering override
	}

	return *this;
}

CAdoInfo::~CAdoInfo()
{


}


bool
CAdoInfo::IsExpiredPathRefreshCounter()
{
	return ( m_pathRefreshCounter <= 0 );
}


void
CAdoInfo::ResetPathRefreshCounter()
{
	int spreadRefresh = 0;
	if( m_pObj != 0 )
	{
		spreadRefresh = m_pObj->GetId() % 45;
	}
	m_pathRefreshCounter = 
		static_cast<int>(( cPATH_REFRESH_TIME / m_timeStepDuration ) + 1 + spreadRefresh );
}


void
CAdoInfo::SetCurrRoadPos( const CRoadPos& roadPos )
{
	m_roadPos = roadPos;
	SyncRoadPosVars();
}


void
CAdoInfo::SyncRoadPosVars()
{
	if ( m_roadPos.IsValid() )
	{
		if( m_roadPos.IsRoad() )
		{
			m_currLane = m_roadPos.GetLane();
			m_currRoad = m_roadPos.GetRoad();
		}
	
		else
		{
			m_currIntrsctn = m_roadPos.GetIntrsctn();
			m_currCrdr     = m_roadPos.GetCorridor();
		}
	}
}


CRandLaneDevInfo::CRandLaneDevInfo()
{

	m_Model  = ELaneDevRamps;
	m_Enable = true;

	m_RampStartTime = 0;

	m_Rise1    = 3.0f;
	m_Rise2    = 7.0f;
	m_Fall1    = 0.8f;
	m_Fall2    = 1.5f;
	m_Idle1    = 1.5f;
	m_Idle2    = 3.0f;
	m_RampAmp1 = 0.6f;
	m_RampAmp2 = 1.5f;
	m_Pos      = 1.0f;

	m_Bias1   = -0.5;
	m_Bias2   = 0.5f;
	m_SinAmp1 = 0.1f;
	m_SinAmp2 = 0.4f;
	m_Frequ1  = 0.02f;
	m_Frequ2  = 0.08f;
	m_Phase1  = 0.0f;
	m_Phase2  = 0.0f;

	m_ReevalTime1 = 30.0f;
	m_ReevalTime2 = 50.0f;

	m_Last = 0.0f;
}


CRandLaneDevInfo::~CRandLaneDevInfo()
{
}
