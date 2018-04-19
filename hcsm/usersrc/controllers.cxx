/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: controllers.cxx,v 1.30 2015/08/28 15:31:31 IOWA\dheitbri Exp $
 *
 * Author:  Horatio German, Omar Ahmad
 *
 * Date:    March, 2002
 *
 * Description:  Contains code for the ADO acceleration controllers.
 *
 ****************************************************************************/
#include <math.h>
#include "controllers.h"
#include "adoinfo.h"
#include "util.h"
#include <direct.h>

// To enable debugging, define DEBUG_FOLLOW_CONTROLLER and set the 
// value of DEBUG_LEAD_ID to -1 to show messages for all cars, or 
// to a specific id to show messages from the vehicle following that id
#undef DEBUG_FOLLOW_CONTROLLER
#define DEBUG_LEAD_ID             -1

// a range of speeds for the lead vehicle during which the
// commanded following distance transitions to the bumper
// to bumper distance
const double FDIST_GOES_TO_BUMPDIST_V1 = 10 * cMPH_TO_MS;		// 10mph
const double FDIST_GOES_TO_BUMPDIST_V2 = 15 * cMPH_TO_MS;		// 15mph

// a range of TTC during which the controller will
// decelerate more aggressively to remain below the
// maximum approach speed
const double ACCEL_LIMITED_BY_APP_DEC_TTC1 = 10;			// sec
const double ACCEL_LIMITED_BY_APP_DEC_TTC2 = 35;			// sec

// when the lead vehicle is going slower than the first value and
// we are going slower than the max approach speed times
// the second value, we ease up on the
// brake so we get there sooner; without this, the
// approach behavior is asymptotic and takes for ever
// to settle
const double MAX_LEAD_VEL_TO_EASE_BRAKE = 7.0 * cMPH_TO_MS; 
const double EASE_BRAKE_APP_SPEED_THRES_FUDGE = 0.85;

// when lead veh goes below this speed, and we reach the bumper
// to bumper distance our deceleration transitions linearly from
// whatever it was to the BRAKE_EFFORT VALUE
const double LOW_SPEED_BRAKE_VEL_THRES = 2 * cMPH_TO_MS;		// 2 mph
const double LOW_SPEED_BRAKE_BRAKE_EFFORT = -4.0;			// -0.4 G

//////////////////////////////////////////////////////////////////////////////
//
// Description:  The controller for situations where the given target 
//   velocity is near the current velocity and thus this controller 
//   produces an acceleration that maintains a velocity range.
//
//   The controller uses the aggressiveness to pick parameters that
//   control how quickly the target speed is attained.  Generally,
//   a value of 1 will give the fastest tracking and a value of 0
//   the slowest tracking
//
// Remarks: 
//
// Arguments:
//   cCurrVel - The current velocity (meters/second).
//   cPrevVel - The velocity from the previous frame (meters/second).
//   cTargVel - The desired target velocity (meters/second).
//   cUrgency - The agressiveness (between 0.0 and 1.0).
//
// Returns:  A double that represents the acceleration.
//
//////////////////////////////////////////////////////////////////////////////
double
SpeedMaintenanceController(
			const double& cCurrVel,
			const double& cPrevVel,
			const double& cTargVel,
			const double& cUrgency
			)
{
	// PID parameters; kP is modulated by aggressiveness
	const double cKP0 = 2.4f;		// Kp for aggressiveness 0.0
	const double cKP1 = 5.5f;		// Kp for aggressiveness 1.0

	double velMph = cTargVel / cMPH_TO_MS;	// convert to mi/hr

	double kp = cKP0 + cUrgency * (cKP1 - cKP0);
	double ki = 10.0f;
	double kd = 0.0f;

	// speed dependent acceleration clipping; helps stabilize
	// the controller and clips too high accelerations when
	// the controller first takes over.
	// Modulated by cUrgency
	const  double cABS_MAX_ACCEL_10MPH_0 = 1.5f;	// aggr = 0.0
	const  double cABS_MAX_ACCEL_80MPH_0 = 2.1f;	// aggr = 0.0
	const  double cABS_MAX_ACCEL_10MPH_1 = 2.5f;	// aggr = 1.0
	const  double cABS_MAX_ACCEL_80MPH_1 = 3.3f;	// aggr = 1.0

	double maxAccel;
	double maxAt10, maxAt80;
	
	maxAt10 = cABS_MAX_ACCEL_10MPH_0 + 
		cUrgency * (cABS_MAX_ACCEL_10MPH_1-cABS_MAX_ACCEL_10MPH_0);
	maxAt80 = cABS_MAX_ACCEL_80MPH_0 + 
		cUrgency * (cABS_MAX_ACCEL_80MPH_1-cABS_MAX_ACCEL_80MPH_0);

	if( velMph <= 10.0f ) 
	{
		maxAccel = maxAt10;
	}
	else if( velMph >= 80.0f ) 
	{
		maxAccel = maxAt80;
	}
	else 
	{
		maxAccel = maxAt10 + ((velMph-10.0f)/70.0f) * (maxAt80 - maxAt10);
	}

	double velDiff = cTargVel - cCurrVel;
	double time    = 1.0 / 30.0;
	double errDist = (cCurrVel - cPrevVel) * time;
	double errAcc  = (cCurrVel - cPrevVel) / time;
	double accel   = kp * velDiff + ki * errDist + kd * errAcc;

	// clip acceleration; when slowing down, do it slower
	if( accel > maxAccel )  accel = maxAccel;
	if( accel < -maxAccel ) accel = -maxAccel * 0.5;

	return accel;
}  // end of SpeedMaintenanceController


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The controller for situations where the given target 
//   velocity is lower than the current velocity and it has be achieved
//   in a limited amount of time.
//
// Remarks: 
//
// Arguments:
//   cCurrVel - The current velocity (meters/second).
//   cTargVel - The desired target velocity (meters/second).
//   cTargDist - The distance in which to achieve the target velocity (meters).
//
// Returns:  A double that represents the acceleration.  This number will
//   be negative since this controller always returns a deceleration.
//
//////////////////////////////////////////////////////////////////////////////
double
LinearDecelController(
			const double& cCurrVel,
			const double& cTargVel,
			const double& cTargDist
			)
{
	double numer = ( cTargVel * cTargVel ) - ( cCurrVel * cCurrVel );
	double denom = 2 * cTargDist;
	
	double accel;
	if( fabs( denom ) > cNEAR_ZERO )
	{
		accel = numer / denom;
	}
	else
	{
		accel = -100.0;
	}

	return accel;
}  // end of LinearDecelController

//////////////////////////////////////////////////////////////////////////////
//
// Description:  The controller for coming to full stop. Based on a stopping
//		distance (larger than a minimum threshold).
//
// Remarks: 
//
// Arguments:
//   cCurrVel - The current velocity (meters/second).
//   cDistToStop - The distance in which to achieve zero velocity (meters).
//   cAggressiveness - The agressiveness (between 0.0 and 1.0).
//   firstFrame - Bool used to set the initial distance. 
//   accel - (output) A accel/decel needed to stop at the stop sign.
//
// Returns:  A boolean indicating if an acceleration is being returned.
//
//////////////////////////////////////////////////////////////////////////////
bool
StopController(
			const double& cCurrVel,
			const double& cDistToStop,
			const double& cAggressiveness,
			bool& firstFrame,
			double& firstDist,
			double& minDist,
			double& accel
			)
{
	const double cMAX_DECEL = - ( 0.6 * cGRAVITY );

	if( firstFrame ) 
	{
		firstDist = cDistToStop;
		minDist = ( cCurrVel * cCurrVel ) / ( 2 * fabs( cMAX_DECEL ) );
		firstFrame = false;
	}

	double interval = firstDist - minDist;
	if ( interval < 0 )  interval = 0;

	double threshold = minDist + (1 - cAggressiveness) * interval;

	bool tooFarAway = cDistToStop > threshold;

#if 0
	gout << "** currVel    = " << cCurrVel << "m/s" << endl;
	gout << "** firstDist  = " << firstDist << "m" << endl;
	gout << "** minDist    = " << minDist << "m" << endl;
	gout << "** interval   = " << interval << "m" << endl;
	gout << "** distToStop = " << cDistToStop << "m" << endl;
	gout << "** threshold  = " << threshold << "m" << endl;
	gout << "** tooFarAway = " << tooFarAway << endl;
#endif

	if( tooFarAway )
	{
		return false;
	}
	else
	{
		bool divideByZero = fabs( cDistToStop ) < cNEAR_ZERO ;
		if( !divideByZero )
		{
			accel = -1.0 * ( cCurrVel * cCurrVel ) / ( 2 * cDistToStop );		
			if( accel < -11.0 )  accel = -11.0;
		}
		else
		{
			bool stopped = cCurrVel < cNEAR_ZERO;
			if( stopped )
			{
				accel = -0.5;
			}
			else
			{
				accel = -11.0;
			}
		}
		return true;
	}
}	// end of StopController



//////////////////////////////////////////////////////////////////////////////
//
// Description:  The controller for achieving a target velocity at the given
//   target acceleration.
//
// Remarks: 
//
// Arguments:
//   cCurrVel - The current velocity (m/s).
//	 cTargVel - The target velocity = 0.0 (m/s).
//   cTargAccel - Target deceleration to be imposed to the vehicle (m/s^2).
//
// Returns:  A double that represents the acceleration.  This number will
//   be negative since this controller always returns a deceleration as long
//	 as the vehicle has not stoped.
//
//////////////////////////////////////////////////////////////////////////////
double
ForcedVelocityController(
			const double& cCurrVel,
			const double& cTargVel,
			const double& cTargAccel
			)
{
	double out;

	bool askedToStop = cTargVel < 0.0001;
	if( askedToStop ) 
	{
		bool almostStopped = cCurrVel < 0.4;
		if( almostStopped ) 
		{
			out = -1.0;		// this will 'lock' the brakes
		}
		else 
		{
			out = cTargAccel;
		}
	}
	else 
	{
		const double cEASE_VEL = 2.5 * 0.4407;          // mph
		double velDiff = fabs( cCurrVel - cTargVel );
		bool reachedTarg = velDiff < 0.05 * 0.4407;		// mph
		bool withinEase  = velDiff < cEASE_VEL;

		if( reachedTarg ) 
		{
			out = 0.0;
		}
		else if( withinEase ) 
		{
			out = fabs( cTargAccel ) * ( cTargVel-cCurrVel ) / cEASE_VEL;
		}
		else 
		{
			out = cTargAccel;
		}
	}

//	printf("ForcedVelCtrl: currV=%6.3f, targV=%6.3f, targA=%6.4f, out=%6.4f\n",
//		cCurrVel, cTargVel, cTargAccel, out);
	return out;
}	// end of ForcedVelocityController



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Simulates reaction time
//
//////////////////////////////////////////////////////////////////////////////
static double
ReactionDelay(
	int                    debugThisFrame,	// for debuging
	const TFollowParams&   cParams,
	CFollowInfo&           fi,
	double                 currVel,
	double                 accel,
	double                 deltaT,
	int                    frame
	)
{
	TFollowState curState;
	double       accelOutput;

	// compute current state
	if ( fabs(currVel) < 0.5 && accel <= 0.0 ) {
		curState = EFolStStop;
	}
	else if ( fabs(accel) < 0.08 ) {
		curState = EFolStSteady;
	}
	else if ( accel > 0.5 ) {
		curState = EFolStAccel;
	}
	else if ( accel < -4.5 ) {
		curState = EFolStEmerg;
	}
	else if ( accel < -0.08 ) {
		curState = EFolStDeccel;
	}
	else {
		curState = EFolStNone;
	}

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("   State for delay is %d\n", curState);
#endif

	if ( curState == fi.m_OldState ) {
		accelOutput   = accel;
		fi.m_OldAccel = accel;
		fi.m_Counting = false;
	}
	else {
		// Delay between stopped and accelerate
		if ( fi.m_OldState == EFolStStop && curState == EFolStAccel ) {

#ifdef DEBUG_FOLLOW_CONTROLLER
if( debugThisFrame )  printf("    Delay: old=Stp, cur=Acc, timer=%3.1f, oldAccel=%.2f, counting=%s\n", 
	   fi.m_Timer, fi.m_OldAccel, fi.m_Counting ? "y" : "n");
#endif
			if ( fi.m_Counting ) {
				fi.m_Timer -= deltaT;

				if ( fi.m_Timer <= 0.0 ) {	// done with delay
					accelOutput    = accel;
					fi.m_OldState  = curState;
					fi.m_OldAccel  = accel;
					fi.m_Timer     = 0.0;
					fi.m_Counting  = false;
				}
				else {
					accelOutput = fi.m_OldAccel;
				}
			}
			else {
				fi.m_Timer    = cParams.stopToStartDelay;
				fi.m_Counting = true;
				accelOutput   = accel;
				fi.m_OldAccel = accel;
			}
		}

		// Delay between steady and acceleration
		else if ( fi.m_OldState == EFolStSteady && curState == EFolStAccel ) {

#ifdef DEBUG_FOLLOW_CONTROLLER
if( debugThisFrame )  printf("    Delay: old=Std, cur=Acc, timer=%3.1f, oldAccel=%.2f, counting=%s\n", 
	   fi.m_Timer, fi.m_OldAccel, fi.m_Counting ? "y" : "n");
#endif
			if ( fi.m_Counting ) {
				fi.m_Timer -= deltaT;

				if ( fi.m_Timer <= 0.0 ) {	// done with delay
					accelOutput    = accel;
					fi.m_OldState  = curState;
					fi.m_OldAccel  = accel;
					fi.m_Timer     = 0.0;
					fi.m_Counting  = false;
				}
				else {
					accelOutput = fi.m_OldAccel;
				}
			}
			else {
				fi.m_Timer    = cParams.steadyToAccelDelay;
				fi.m_Counting = true;
				accelOutput   = accel;
				fi.m_OldAccel = accel;
			}
		}

		// Delay between steady/accel and deceleration
		else if ( fi.m_OldState == EFolStSteady || fi.m_OldState == EFolStAccel
			&& curState == EFolStDeccel ) {

#ifdef DEBUG_FOLLOW_CONTROLLER
if( debugThisFrame )  printf("    Delay: old=Std, cur=Dec, timer=%3.1f, oldAccel=%.2f, counting=%s\n", 
	   fi.m_Timer, fi.m_OldAccel, fi.m_Counting ? "y" : "n");
#endif
			if ( fi.m_Counting ) {
				fi.m_Timer -= deltaT;

				if ( fi.m_Timer <= 0.0 ) {	// done with delay
					accelOutput    = accel;
					fi.m_OldState  = curState;
					fi.m_OldAccel  = accel;
					fi.m_Timer     = 0.0;
					fi.m_Counting  = false;
				}
				else {
					accelOutput = fi.m_OldAccel;
				}
			}
			else {
				fi.m_Timer    = cParams.steadyToDeccelDelay;
				fi.m_Counting = true;
				accelOutput   = accel;
				fi.m_OldAccel = accel;
			}
		}

		// Delay between deceleration and acceleration
		else if ( fi.m_OldState == EFolStDeccel && curState == EFolStAccel ) {

#ifdef DEBUG_FOLLOW_CONTROLLER
if( debugThisFrame )  printf("    Delay: old=Dec, cur=Acc, timer=%3.1f, oldAccel=%.2f, counting=%s\n", 
	   fi.m_Timer, fi.m_OldAccel, fi.m_Counting ? "y" : "n");
#endif
			if ( fi.m_Counting ) {
				fi.m_Timer -= deltaT;

				if ( fi.m_Timer <= 0.0 ) {	// done with delay
					accelOutput    = accel;
					fi.m_OldState  = curState;
					fi.m_OldAccel  = accel;
					fi.m_Timer     = 0.0;
					fi.m_Counting  = false;
				}
				else {
					accelOutput = fi.m_OldAccel;
				}
			}
			else {
				fi.m_Timer    = cParams.deccelToAccelDelay;
				fi.m_Counting = true;
				accelOutput   = accel;
				fi.m_OldAccel = accel;
			}
		}

		else {
			fi.m_OldState  = curState;
			fi.m_OldAccel  = accel;
			accelOutput    = accel;
			fi.m_Counting  = false;
		}
	}

	return accelOutput;
}


/////////////////////////////////////////////////////////////////////////////
//
// Normal follow controller.  It does not deal with emergency braking;
// Some of the 'features'
//   If told to catch up, will not exceed the lead veh speed over a margin
//   Given const lead vehicle speed, will match the distance pretty quickly
//   As long as the lead vehicle doesn't accel/decel beyond a threshold, the
//      controller will track the lead vehicle in fixed distance or fixed time
//      mode.  (NOTE: fixed time mode works much better than fixed distance)
//   No oscillations
//   If behind a stopped vehicle, it will approach up to a preset distance
//     then come to a stop.
//   Maximum and minimum accelerations are clipped
//
// As long as the lead deceleration does not exceed a threshold, there should
// be no collisions.  Of course, wrong parameter can cause it to collide.
//
static bool 
NormalFollowController(
		int   debugThisFrame,						// for debugging
		const TFollowParams& cFollowParams,
		const double& cTargDist,
		const double& actualDist,
		const double& prevActualDist,
		const double& leadVel,
		const double& currVel,
		const double& prevVel,
		const double& ttc,					// calculated time to collision
		const double& maxSpeed,			// don't exceed this speed
		double& output,
		int    frame,
		bool   gapMode
)
{
	double deltaT = 1.0 / 30.0;
	const TNormFollowParams* pParams;
	if( gapMode )
	{
		pParams = &(cFollowParams.maintainGap);
	}
	else
	{
		pParams = &(cFollowParams.normal);
	}

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("  NormalFollowCtrl trgDst=%5.1f actDst=%5.1f, prvDst=%5.1f, vel=%5.1f\n",
		cTargDist, actualDist, prevActualDist, currVel);
#endif
	
	// Adjust target following distance based on our own speed;  at lower
	// speeds, the following distance approaches the 'bumper-to-bumper' distance
	// whereas at higher speeds it is as defined.
	double v1 = FDIST_GOES_TO_BUMPDIST_V1;	// below this speed, fdist = bumper
	double v2 = FDIST_GOES_TO_BUMPDIST_V2;	// above this speed, fidst = as passed
	double vblend;

	vblend = (currVel - v1) / (v2 - v1);
	if ( vblend < 0.0 ) vblend = 0.0; else if ( vblend > 1.0 ) vblend = 1.0;

	double targDist = cTargDist;
	if ( !gapMode ) {
		targDist = (1.0 - vblend) * pParams->bumpStopDist + vblend * targDist;
	}

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("    Modifed following dist: %6.1f (blend=%5.3f)\n", targDist, vblend);
#endif

	// The variable maxVel is the maximum velocity we can have given the current 
	// distance that would not require exceeding the approach deceleration rate
	// in order to stop at the proper distance.  The concept with this 'maxVel'
	// is that if we are under that, we don't really worry about having to slow down.
	// The equation of the velocity is v = sqrt(vtag*vtarg-2*accel*dist)
	double maxVel;
	double sqTerm;
	double velPid, velErr;
	double distPid, distErr;
	
	sqTerm = leadVel * leadVel - 2.0 * pParams->appDecRate * (actualDist-pParams->bumpStopDist);
	if ( sqTerm <= 0.0 ) maxVel = -1.0; else maxVel = sqrt(sqTerm);

	if ( ttc > 40.0 ) {
		maxVel = -1.0;
	}
	else {
		maxVel = -pParams->appDecRate * ttc;
	}

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("    MaxApproachSpeed = %.2f, curSpeed = %.2f\n", // , low speed mode = %s\n",
		maxVel, currVel);//, lowSpeedSit ? "YES" : "NO");
#endif

	// correction factor for approach speed; only decelerates 
	// We also multiply the correction with a scaling factor that
	// reduces its effectiveness as the ttc goes up.  When the TTC is over
	// a threshold, the effectiveness is 0 and when the ttc is 0, the effectiveness
	// is 1.0
	double  vel2p;		// modified proportional gain for approach vel conrol
	double l1 = ACCEL_LIMITED_BY_APP_DEC_TTC1;	// scale at that or lower ttc is 1.0
	double l2 = ACCEL_LIMITED_BY_APP_DEC_TTC2;	// scale at that or higher ttc is 0.0
	if ( maxVel > 0.0 ) {
		vel2p  = (0.9 * maxVel - currVel) * pParams->vel2Kp;	// 0.9 is fudge
		double scale;

		scale = (ttc - l2) / (l1 - l2);
		if ( scale < 0.0 ) scale = 0.0;
		if ( scale > 1.0 ) scale = 1.0;

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("    SCALE=%.3lf, vel2p before/after: %lf %lf\n", scale, vel2p, vel2p * scale);
#endif
		vel2p *= scale;
		if ( vel2p > 0.0 ) vel2p = 0.0;
	}
	else {
		vel2p = 0.0;
	}

	// compute "normal" following controller
	velErr  = leadVel - currVel;
	velPid  = pParams->velKp * velErr;
	distErr = actualDist - targDist;

	// the controller is non-symmetric on the distance error to avoid
	// excessive accelerations when following a vehicle from far away
	if( distErr > 0.0 ) 
	{
		if( distErr > 100.0 )  distErr = 100.0;
		distPid = (
			pParams->distKp * 0.5 * distErr + 
			pParams->distKi * (prevActualDist - actualDist) * deltaT + 
			pParams->distKd * (prevActualDist - actualDist) / deltaT
			);
	}
	else {
		distPid = (
			pParams->distKp * distErr + 
			pParams->distKi * (prevActualDist - actualDist) * deltaT + 
			pParams->distKd * (prevActualDist - actualDist) / deltaT
			);
	}

	double NormOutput = velPid + distPid;

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )
	{
		printf(
			"    velPid=%.2f, distPid=%.2f, vel2=%.2f out (bef corrections)=%.2f\n", 
			velPid, 
			distPid, 
			vel2p, 
			NormOutput
			);
	}
#endif

	// when the lead vehicle is stopped or slow and we are going slower than the
	// approach speed we add a small factor to the acceleration so it doesn't
	// take for ever to come to a stop.  This may eventually need to be
	// replaced with a different formulation controller designed specifically
	// for approaching a slow moving vehicle.
	double scale1 = 0.0;
	double scale2 = 0.0;
	if ( leadVel >= 0.0 && leadVel <= MAX_LEAD_VEL_TO_EASE_BRAKE ) {
		scale1 = 1.0 - leadVel / MAX_LEAD_VEL_TO_EASE_BRAKE;
	}
	if ( currVel < EASE_BRAKE_APP_SPEED_THRES_FUDGE * maxVel ) {
		scale2 = 1.0 - currVel / (EASE_BRAKE_APP_SPEED_THRES_FUDGE * maxVel);
	}

	if ( distErr > 0.0 ) 
		output = NormOutput + 2.0 * scale1 * scale2;	// 2.0 is a gain
	else
		output = NormOutput;

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("    slow speed boost: scale1,2 = %5.3f %5.3f, out = %.2f\n",
		scale1, scale2, output);
#endif


	// clipping and corrections to output
	// boolean flags are there to help with debugging messages but also
	// have function; DO NOT REMOVE
	bool   b1 = false, b2 = false, b3 = false, b4 = false, b5 = false;

	// if our current speed is more than the approach speed and the time
	// to collision is low, we limit acceleration
	if ( currVel > 0.9 * maxVel && maxVel > 0.0 && ttc < l2 ) {
		b4 = true;
#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )
	{
		gout << "ttc = " << ttc << endl;
		gout << "velPid = " << velPid << "   distPid = " << distPid << "   vel2p = " << vel2p << endl;
	}
#endif
		if ( velPid + distPid < vel2p ) 
			output = velPid + distPid;
		else
			output = vel2p;
	}

	if ( output < pParams->negAccClip ) {
		output = pParams->negAccClip;
        //is the target moving faster than me, and can I accelerate?
        if (distErr < 0 && velErr > 0 && pParams->posAccClip > 0){
            //calculate the required accel rate so that we hit our target gap 
            // a = v^2/(2s)
            double accelRate = (velErr * velErr) / (abs(2*distErr)); //the target is moving 
            if (accelRate > pParams->posAccClip){
                output = pParams->posAccClip;
            }
            else if (accelRate > pParams->posAccClip*0.5 ){
                output = accelRate *0.9;
            }

        }
    }

	if ( output < pParams->negAccClip ) {

		b1 = true;
	}

	if ( output >= 0.0 ) {
		if ( pParams->accelToCatchUp == false && actualDist > pParams->clipVelRange ) {
			b2 = true;
			output = 9999.9;		// for debugging print only
		}
		else {
			if ( currVel > maxSpeed ) {
				b3 = true;
				output = 0.0;
			}
			if ( currVel > pParams->ovspeedClip * leadVel && leadVel > 5.0 
						&& actualDist < pParams->clipVelRange ) {
				b3 = true;
				output = 0.0;
			}

			if ( output > pParams->posAccClip ) {
				output = pParams->posAccClip;
				b5 = true;
			}
		}
	}


#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  
	{
		if ( b1 ) printf("   clip by negAccClip to %6.2f\n", pParams->negAccClip);
		if ( b2 ) printf("   no output because no accel to catchup\n");
		if ( b3 ) printf("   clip to 0 due to overspeed (lim=%.2f)\n", maxSpeed);
		if ( b4 ) printf("   clip to stay within maxVel(%.1f)\n", maxVel);
		if ( b5 ) printf("   clip by posAccClip to %6.2f\n", pParams->posAccClip);
		printf("   FINAL output = %.3f\n", output);
	}
#endif

	if ( b2 ) return false; else return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// The emergency follow controller.  Its purpose is to stop a vehicle
// when the time to collision falls below a threshold.  The controller
// is not meant to be used for following when the TTC is large.
//
// Some of its features
//   Will generally not accelerate
//   Will decelerate only as much as needed to stop at a certain
//           distance behind the lead vehicle
//   Smooth - no oscillations
//   Maximum deceleration specified in the parameters
//   Built in reaction delay (TBD)
//
// Testing procedure: 
//   Ensure it's the only controller called;  
//
//   Follow vehicle reaches desired speed, stopped lead vehicles appears 
//   at a certain TTC => ensure that uniform deceleration is applied so 
//   follower stops behind vehicle.
//   In the above, vary the TTC within the whole range.
//
//   Two vehicles with same speed follow each other at a distance D
//   Lead vehicle slams on brakes with maximum deceleration => ensure that
//   follower stops no harder than it needs and stops behind lead vehicle.
//
static double 
EmergencyFollowController(
			bool          debugThisFrame,
			const TFollowParams& cParams,
			int           leadObjCvedId,
			const double& cTargDist,
			const double& cActualDist,
			const double& cPrevActualDist,
			bool          firstTime,
			const double& cLeadVel,
			const double& cCurrVel,
			const double& cPrevVel,
			const double& cLeadAccel,
			const double& cAggressiveness,
			int           frame
			)
{
	const TEmergFollowParams& par = cParams.emerg;

	double deltaT = 1.0 / 30.0;

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("  Emergency brake controller **\n");
#endif

	double output;
	double accError  = cLeadAccel - (cCurrVel - cPrevVel)/deltaT;
	double distError = cActualDist - cTargDist;

	double emergVelTerm  = par.velKp * (cLeadVel - cCurrVel);
	double emergDistTerm = par.distKp * (cActualDist - cTargDist) + 
		par.distKi * (cPrevActualDist - cActualDist) * deltaT + 
		par.distKd * (cPrevActualDist - cActualDist) / deltaT;

	if ( emergDistTerm > 0  ) {
#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("   ====> Clip dist term rel to vel from %.2f to %.2f\n",
			emergDistTerm, 0.0);
#endif
		emergDistTerm = 0.0;
	}
	output = par.aclKp * accError + emergVelTerm + emergDistTerm;
	if ( output > par.posAccClip ) output = par.posAccClip;
	if ( output < par.negAccClip ) output = par.negAccClip;
	if ( cCurrVel > 1.25 * cLeadVel && output > 0.0 ) output = 0.0;

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf("   AccTerm=%.2f,   VelTerm=%.2f, DistTerm=%.2f Out=%.2f\n",
			par.aclKp * accError, 
			emergVelTerm, 
			emergDistTerm,
			output);
#endif

	return output;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The controller for following another vehicle.
//
// Remarks: 
//
// A new addition to the controller is meant to deal with the problem of
// a new vehicle cutting in front of us causing a sharp and sudden brake
// due to the discontinuity on the following distance.
// The solution concept is as follows:  we monitor the time for which we 
// have been following a 
// vehicle, and as long as there is no immediate danger of collision, we
// transition the following distance between the initial following distance
// and the commanded following distance over a period of N seconds.  
// 
// Arguments:
//   frame          - The current frame number (used for debugging).
//   leadObjCvedId  - The lead object's CVED id.
//   cActualDist    - The actual distance between the leader and the 
//                    follower (in meters).
//	 cLeadVel       - The leader's speed (m/s).
//	 cCurrVel       - The follower's speed (m/s).
//   cPrevVel       - The follower's previous speed (m/s).
//   cLeadAccel     - The leader's acceleration (m/s^2).
//   cAggressiveness - The agressiveness (between 0.0 and 1.0).
//   gapMode        - When true, controller used to maintain gap, not follow
//   accelOutput    - (output) The acceleration for the follower.
//
// Returns:  A boolean that indicates if a valid acceleration is being 
//  returned.
//
//////////////////////////////////////////////////////////////////////////////
bool 
FollowController(
			int           frame,
			double         deltaT,
			int           leadObjCvedId,
			CFollowInfo&  fi,
			const TFollowParams& cFollowParams,
			const double& cActualDist,
			const double& cPrevActualDist,
			int           followThisLeadAge,
			double         initFollowDist,
			const double& cLeadVel,
			const double& cCurrVel,
			const double& cPrevVel,
			const double& cLeadAccel,
			const double& cAggressiveness,
			const double& cMaxSpeed,
			double&       accelOutput,
			double&        ttgout,
			bool          gapMode
			)
{
	double ttc;							// time to collision
	double accel;			            // output
//	double deltaT = 1.0/30.0;

#ifdef DEBUG_FOLLOW_CONTROLLER
	bool debugThisFrame = ( frame % 5 == 0 ) && 
			(DEBUG_LEAD_ID < 0 || DEBUG_LEAD_ID == leadObjCvedId);
	if( debugThisFrame )
	{
		printf( "\n" );
		printf( "--- FollowController  gapMode = %d\n", gapMode );
	}
//	static TFollowParams  cParams;
#else
	bool debugThisFrame = false;
#endif
	const TFollowParams& cParams = cFollowParams;

#if 0 //def DEBUG_FOLLOW_CONTROLLER
	static bool firstTime = true;
	if( firstTime ) 
	{
		//
		// Read the parameters from a file if it exists.
		//
		cParams = cFollowParams;	// in case there is no file
		FILE *p = fopen( "followIn.txt", "r" );
		if( p ) 
		{
			char line[300];
			int ival;

			fgets(line, 299, p);
			sscanf(line, "%d%lf%lf%lf%lf%lf%d%lf%lf%lf%lf", 
				&cParams.folTimeMode, 
				&cParams.folValue, 
				&cParams.ttcThres1, 
				&cParams.ttcThres2, 
				&cParams.ttcThres3,
				&cParams.engageThres,
				&ival,
				&cParams.steadyToAccelDelay,
				&cParams.steadyToDeccelDelay,
				&cParams.deccelToAccelDelay,
				&cParams.stopToStartDelay);
			cParams.useReactDelay = ival ? true : false;

			fgets(line, 299, p);
			if( gapMode )
			{
				sscanf(line, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d",
					&cParams.maintainGap.distKp, &cParams.maintainGap.distKi, 
					&cParams.maintainGap.distKd, 
					&cParams.maintainGap.velKp, &cParams.maintainGap.vel2Kp,
					&cParams.maintainGap.ovspeedClip, &cParams.maintainGap.clipVelRange,
					&cParams.maintainGap.posAccClip, &cParams.maintainGap.negAccClip,
					&cParams.maintainGap.bumpStopDist, &cParams.maintainGap.appDecRate, 
					&cParams.maintainGap.maxAppSpeed, &ival);
				cParams.maintainGap.accelToCatchUp = ival ? true : false;
			}
			else
			{
				sscanf(line, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d",
					&cParams.normal.distKp, &cParams.normal.distKi, 
					&cParams.normal.distKd, 
					&cParams.normal.velKp, &cParams.normal.vel2Kp,
					&cParams.normal.ovspeedClip, &cParams.normal.clipVelRange,
					&cParams.normal.posAccClip, &cParams.normal.negAccClip,
					&cParams.normal.bumpStopDist, &cParams.normal.appDecRate, 
					&cParams.normal.maxAppSpeed, &ival);
				cParams.normal.accelToCatchUp = ival ? true : false;
			}

			fgets(line, 299, p);
			sscanf(line, "%lf%lf%lf%lf%lf%lf%lf%lf%lf",
				&cParams.emerg.distKp, &cParams.emerg.distKi, &cParams.emerg.distKd, 
				&cParams.emerg.velKp, &cParams.emerg.velKi, &cParams.emerg.velKd,
				&cParams.emerg.aclKp, 
				&cParams.emerg.posAccClip, &cParams.emerg.negAccClip );

			fclose(p);

			fprintf(stderr, "FollowController: loaded values from file:\n");
			fprintf(stderr, "Reg:   %d %lf %lf %lf %lf %lf %d %lf %lf %lf %lf\n", 
				cParams.folTimeMode,
				cParams.folValue, cParams.ttcThres1, cParams.ttcThres2, cParams.ttcThres3,
				cParams.engageThres,
				cParams.useReactDelay?1:0, cParams.steadyToAccelDelay,
				cParams.steadyToDeccelDelay, cParams.deccelToAccelDelay, 
				cParams.stopToStartDelay);

			fprintf(stderr, "Norm:  %lf %lf %lf %lf %lf %lf %lf\n", cParams.normal.distKp,
				cParams.normal.distKi, cParams.normal.distKd,
				cParams.normal.velKp, cParams.normal.vel2Kp,
				cParams.normal.ovspeedClip, cParams.normal.clipVelRange);

			fprintf(stderr, "Norm:  %lf %lf %lf %lf %lf %s\n",
				cParams.normal.posAccClip, cParams.normal.negAccClip, 
				cParams.normal.bumpStopDist,
				cParams.normal.appDecRate, cParams.normal.maxAppSpeed,
				cParams.normal.accelToCatchUp ? "y" : "n");

			fprintf(stderr, "Emerg: %lf %lf %lf %lf %lf ...\n", cParams.emerg.distKp,
				cParams.emerg.distKi, cParams.emerg.distKd, cParams.emerg.velKp,
				cParams.emerg.velKi);

			fprintf(stderr, "MGap:  %lf %lf %lf %lf %lf %lf %lf\n", cParams.maintainGap.distKp,
				cParams.maintainGap.distKi, cParams.maintainGap.distKd,
				cParams.maintainGap.velKp, cParams.maintainGap.vel2Kp,
				cParams.maintainGap.ovspeedClip, cParams.maintainGap.clipVelRange);

			fprintf(stderr, "MGap:  %lf %lf %lf %lf %lf %s\n",
				cParams.maintainGap.posAccClip, cParams.maintainGap.negAccClip, 
				cParams.maintainGap.bumpStopDist,
				cParams.maintainGap.appDecRate, cParams.maintainGap.maxAppSpeed,
				cParams.maintainGap.accelToCatchUp ? "y" : "n");
		}
		FILE *p0 = fopen("followctrl.txt", "w");	// create the file empty
		fclose(p0);

		firstTime = false;
	}
#endif

	const TNormFollowParams* cntrllrParams;
	if( gapMode )
	{
		cntrllrParams = &(cParams.maintainGap);
	}
	else
	{
		cntrllrParams = &(cParams.normal);
	}

	//
	// Compute ttc.
	//
	const double cTTC_INFINITE = 9999.0;
	bool leaderFasterThanFollower = cLeadVel >= cCurrVel - 0.2;
	if( leaderFasterThanFollower ) 
	{
		ttc = cTTC_INFINITE;
	}
	else 
	{
		ttc = cActualDist / ( cCurrVel - cLeadVel );
		// to help plotting the output
		if( ttc > 99 ) ttc = cTTC_INFINITE;
	}

	// don't use ttc for maintain gap mode
	if( gapMode ) ttc = cTTC_INFINITE;

	ttgout = ttc;

	//
	// Compute target distance.
	//
	double targDist;
	if( cParams.folTimeMode ) 
	{
		targDist = cCurrVel * cParams.folValue;

#ifdef DEBUG_FOLLOW_CONTROLLER
		if( debugThisFrame )  
		{
			gout << "targDist thru time = " << targDist << "m" << endl;
		}
#endif
	}
	else 
	{
		targDist = cParams.folValue;

#ifdef DEBUG_FOLLOW_CONTROLLER
		if( debugThisFrame )  
		{
			gout << "targDist thru dist = " << targDist << "m" << endl;
		}
#endif
	}

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )
	{
		gout << "bumpStopDist = " << cntrllrParams->bumpStopDist << "m";
		gout << endl;
	}
#endif

	if( fabs( targDist ) < cntrllrParams->bumpStopDist ) 
	{
		if( targDist < 0.0 )
		{
			targDist = -1.0 * cntrllrParams->bumpStopDist;
		}
		else
		{
			targDist = cntrllrParams->bumpStopDist;
		}
	}

#ifdef DEBUG_FOLLOW_CONTROLLER
	// print initial debug message
	if( debugThisFrame )
	{
		printf( 
			"  leadId=%2d, ft=%4d, ifd=%5.1f TTC=%5.2f, FOLDIST=%7.3f\n", 
			leadObjCvedId, 
			followThisLeadAge, 
			initFollowDist, 
			ttc, 
			cActualDist
			);
		printf( "   leadVel=%5.1f,  leadAccel=%5.2f\n", cLeadVel, cLeadAccel );
	}
#endif

	// Determine if the controller should be engaged
	bool tooFarToWorryAbout = (
				ttc > cParams.ttcThres3 && 
				cActualDist >= cParams.engageThres
				);
	bool disableController = !gapMode && tooFarToWorryAbout;
	if( disableController  ) 
	{
#ifdef DEBUG_FOLLOW_CONTROLLER
		if( debugThisFrame )  
		{
			printf(
				"ttc > thres3(%.2lf) && cActualDist > limit(%lf) => NO CONTROL\n",
				cParams.ttcThres3, 
				cParams.engageThres
				);
		}
#endif
		return false;
	}

	// Modify the target following distance based on how long we have been
	// following a vehicle
	// Params: 
	//   N    - number of seconds to transition between the actual initial
	//          follow dist and the one specified in the parameters
	//   TCFC - too close for comfort - a ttc threshold used to nullify the
	//          'shortening' of the following distance;  this ensures that
	//          if we are fast approaching the newly followed vehicle, we
	//          won't run into it.
#define N     4.5	// seconds
#define TCFC  4.0	// seconds

	// linear interpolation of the target following disnt
	//		int           followThisLeadAge,
	//		double         initFollowDist,
	// HACK -> 30 is frames per sec
	
#if 0  // ---- disable for now --------
	double dilutedFollowDist = targDist;
	if( followThisLeadAge < N * 30.0 && !gapMode ) 
	{
		double factor  = 0.0f;
		double factor2 = 0.0f;
		if ( initFollowDist < targDist ) {

			factor = followThisLeadAge / (N * 30.0);

			if( ttc < 0.5 * TCFC ) 
			{
				factor = 0.0;
			}
			else if( ttc < 1.5 * TCFC ) 
			{
				factor2 = (ttc - TCFC) / (0.5 * TCFC);
				factor *= factor2;
			}
			dilutedFollowDist = initFollowDist + factor * (targDist - initFollowDist );
			if( dilutedFollowDist < normalCntrllrParams->bumpStopDist )
			{
				dilutedFollowDist = normalCntrllrParams->bumpStopDist;
			}
		}
#ifdef DEBUG_FOLLOW_CONTROLLER
		if( debugThisFrame )
		{
			printf(
				"  f1/f2=%.2f, %.2f, diluted targDist = %5.2f\n",
				factor, 
				factor2,
				dilutedFollowDist
				)
		};
#endif
	}
	targDist = dilutedFollowDist;
#endif  // ------- end disable ------------

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )  printf( "Target distance is %.2f\n", targDist );
#endif

	// Depending on the time to collision, we calculate the bleding function
	// for combining the effect of the normaland emergency controller.
	// Linear blending is used.
	double blend1 = 0, blend2 = 0;
	double ebrakeaccel, followaccel = 0.0;

	double t2mint1 = cParams.ttcThres2 - cParams.ttcThres1;
	blend1  = -ttc / t2mint1 + cParams.ttcThres2 / t2mint1;
	blend2  = ttc / t2mint1 - cParams.ttcThres1 / t2mint1;

	if( blend1 < 0.0 ) blend1 = 0.0;
	if( blend1 > 1.0 ) blend1 = 1.0;

	if( blend2 < 0.0 ) blend2 = 0.0;
	if( blend2 > 1.0 ) blend2 = 1.0;

	if( !gapMode ) 
	{
		ebrakeaccel = EmergencyFollowController(
							debugThisFrame,
							cParams, 
							leadObjCvedId,
							targDist,
							cActualDist, 
							cPrevActualDist, 
							false, 
							cLeadVel,
							cCurrVel, 
							cPrevVel, 
							cLeadAccel, 
							cAggressiveness, 
							frame
							);
	}
	else 
	{
		ebrakeaccel = 0.0;
	}

	bool haveNorm = NormalFollowController(
							debugThisFrame, 
							cParams, 
							targDist, 
							cActualDist, 
							cPrevActualDist, 
							cLeadVel, 
							cCurrVel, 
							cPrevVel,
							ttc, 
							cMaxSpeed, 
							followaccel, 
							frame, 
							gapMode 
							);
	if( !haveNorm ) 
	{
		return false;
	}

	accel = blend1 * ebrakeaccel + blend2 * followaccel;

	// Emergency tap on the brake; once we get beyond the bumper to
	// bumper distance, if the lead vehicle is stopped, we apply the
	// brakes very sharply
	bool emergTapOnBrake = (
				cActualDist <= cntrllrParams->bumpStopDist && 
				cCurrVel <= cLeadVel &&
				cLeadVel < LOW_SPEED_BRAKE_VEL_THRES
				);
	if( emergTapOnBrake ) 
	{
		double newAccel;

		newAccel = 0.2 * LOW_SPEED_BRAKE_BRAKE_EFFORT 
			- 0.8 * LOW_SPEED_BRAKE_BRAKE_EFFORT 
			* (cCurrVel - LOW_SPEED_BRAKE_VEL_THRES)/LOW_SPEED_BRAKE_VEL_THRES;
		if ( newAccel > 0.2 * LOW_SPEED_BRAKE_BRAKE_EFFORT )
			newAccel = 0.2 * LOW_SPEED_BRAKE_BRAKE_EFFORT;

#ifdef DEBUG_FOLLOW_CONTROLLER
		if( debugThisFrame )  
		{
			printf(
				" >>>>> Stop condition accel was %6.2f, now %6.2f\n", 
				accel, 
				newAccel
				);
		}
#endif
		accel = newAccel;
	}

	if( cParams.useReactDelay )
	{
		accelOutput = ReactionDelay(
							debugThisFrame, 
							cParams, 
							fi, 
							cCurrVel, 
							accel, 
							deltaT, 
							frame
							);
	}
	else
	{
		accelOutput = accel;
	}

#ifdef DEBUG_FOLLOW_CONTROLLER
	if( debugThisFrame )
	{
		printf(
			"LeadAccel=%6.2f, Blend (Emerg, Norm):(%4.2f, %4.2f), Out=%6.3f, "
			"DelayReagout=%6.3f\n",
			cLeadAccel, 
			blend1, 
			blend2, 
			accel, 
			accelOutput
			);
	}
#endif

#ifdef DEBUG_FOLLOW_CONTROLLER
	FILE* pO;
	pO = fopen( "followctrl.txt", "a+" );
	fprintf(
		pO, 
		"%.2f %.2f %.2f %.2f %.1f %.1f %.5f %.3f %.3f %.5f %.2f\n", 
		targDist, cActualDist, 
		cLeadVel, cCurrVel, 
		blend1*10.0, blend2*10.0, 
		cLeadAccel,
		ebrakeaccel, followaccel,
		accelOutput,
		ttc
		);
	fclose( pO );
#endif

	return true;
}	// end of FollowController
