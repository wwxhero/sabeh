//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Author(s):   Chris Schwarz
// Date:        July, 2004
//
// Description: The implementation of several collision warning algorithms 
//
//////////////////////////////////////////////////////////////////////////////

#include "math.h"
#include "util.h"
#include "ScenarioControl.h"
#include <transmat.h>

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616
#define BUFLEN     6
#define	SIGN(x)	(((x) < 0)? -1 : 1)
#define AVGLEN     200

int cvedIdPrev, cvedIdMemory, accRegion;
float rangeMemory, rangerateMemory, objVelMemory, angleMemory;
int        debug_ct1 = 999,       debug_ct2 = 999;
int    leftWarnOn_ct = 999,  leftWarnOff_ct = 999;
int   rightWarnOn_ct = 999, rightWarnOff_ct = 999;
int         accOn_ct = 0,         accOff_ct = 0;
bool   leftWarn_prev = 0,    rightWarn_prev = 0;

float krvBuf[AVGLEN], krvAvg;
int   krv_idx = 0;
float dl_buf[BUFLEN];
int   buf_idx = 0;
float leadVehVel_prev = 0.0f;
float decel_lead_prev = 0.0f;
float leadObjId_prev = 0.0f;
int   false_alarm_trigger = 0, false_alarm_reset = 0;
float false_alarm_cond_prev = 0.0f, range_prev = 0.0f;
float false_time = 0.0f;

//////////////////////////////////////////////////////////////////////////////
///
///\brief Computes the collision avoidance system warnings.
///
///\remark 
///		These systems ONLY take VEHICLES into acount, we will need to expand this 
///		for other object classes
///
///\param	cOwnVehCartPos - The own vehicle's cartesian position in feet.
///
///\param   cAlgorithm - The algorithm to execute (0=none, 1=TRW, 2=TRWFAZ, 3=LPWS, 4=FCW, 5=ACC, 6=FCW Simple).
///
///\param	frequencyOfExeuction - Number of frames per second that this function 
///			is called.
///
///\return		void
///
//////////////////////////////////////////////////////////////////////////////
void 
CScenarioControl::ComputeCollisionAvoidanceWarnings( 
			const CPoint3D& cOwnVehCartPos,
			int& _rWarningLights, 
			const int cAlgorithm, 
			float* pFcwInfo,
			short& timeToWarn,
			float frequencyOfExecution
			)
{
	//
	// Reset the sensor info cell.
	//
	CHcsmCollection::m_sSensorInfo[0] = -1.0f;
	// these are being set since the titler is not able to look at 
	// element 0 to see if the values in elements 1 and 4 are valid.
	CHcsmCollection::m_sSensorInfo[1] = 0.0f;
	CHcsmCollection::m_sSensorInfo[3] = 0.0f;

	const double cAccLeadVehThresholdVel = 8.0 * cMPH_TO_MS;
	const float cMaxRange = CHcsmCollection::m_sSensor_Config[eSENSOR_INDEX_MAX_RANGE];
	const float cMaxSqrdRng = cMaxRange * cMaxRange; // (ft)
	bool leftWarn = 0, rightWarn = 0;
	timeToWarn = 0;

	static int sccCounter = 0;
	sccCounter++;
	bool printStuff = 0; //( sccCounter % 30 ) == 0;
	if( printStuff )
	{
		fprintf( stdout, "cAlgorithm = %d sccCounter: ",cAlgorithm );
	}

	// calculate count thresholds for warning lags
	const float cLagOn  = 68.0f;  // msec
	const float cLagOff = 768.0f; // msec
	const float cAccLagOn = 1500.0f; // msec
	const float cAccLagOff = 1500.0f; // msec
	const int cLagOnCount  = (int) ( cLagOn  / 1000.0f * frequencyOfExecution );
	const int cLagOffCount = (int) ( cLagOff / 1000.0f * frequencyOfExecution );
	const int cAccLagOnCount = (int) ( cAccLagOn / 1000.0f * frequencyOfExecution );
	const int cAccLagOffCount = (int) ( cAccLagOff / 1000.0f * frequencyOfExecution );

	// get own vehicle data
	CVector3D ownVehTang, ownVehLat, ownVehAngVel;
	double ownVehVel;
	m_pCved->GetOwnVehicleVel( ownVehVel  );
	m_pCved->GetOwnVehicleTan( ownVehTang );
	m_pCved->GetOwnVehicleLat( ownVehLat  );
	m_pCved->GetOwnVehicleAngularVel( ownVehAngVel );
	float ownVehHeading = (float) atan2( ownVehTang.m_j, ownVehTang.m_i );
	float ownVehYawRate = (float) ownVehAngVel.m_k;
	float curvature;
	if( ownVehVel > 1.0 )
	{
		curvature = (float)(ownVehYawRate * cDEG_TO_RAD / ( ownVehVel * cMETER_TO_FEET ));
	}
	else 
	{
		curvature = 0.0;
	}
	krvAvg -= krvBuf[krv_idx] / ( (float) AVGLEN );
	krvBuf[krv_idx] = curvature;
	krvAvg += krvBuf[krv_idx] / ( (float) AVGLEN );
	krv_idx++;
	if ( krv_idx>=AVGLEN ) krv_idx = 0;

	int i;
	if( printStuff )
	{
		fprintf( stdout, "numObj=%d  ", CHcsmCollection::m_sDynObjDataSize );
	}

	bool sensorObjectWritten = false;
	bool foundObj = false;

	// increment lag counters
	leftWarnOff_ct  += 1;
	leftWarnOn_ct   += 1;
	rightWarnOff_ct += 1;
	rightWarnOn_ct  += 1;
	accOn_ct        += 1;
	accOff_ct       += 1;

	// avoid counter overflows
	if ( leftWarnOff_ct  >= 30000 ) leftWarnOff_ct  = 30000;
	if ( leftWarnOn_ct   >= 30000 ) leftWarnOn_ct   = 30000;
	if ( rightWarnOff_ct >= 30000 ) rightWarnOff_ct = 30000;
	if ( rightWarnOn_ct  >= 30000 ) rightWarnOn_ct  = 30000;


	// There are three ways to prematurely exit the loop
	//   1.  the cved ID from the array is not a valid object
    //   x#2 is not true, it is sorted by type then distance, 
	//   2.  the range is beyond a specified threshold (array is sorted by range)
	//   3.  the cAlgorithm is 0, indicating no algorithm is used
	for( i = 0; i < CHcsmCollection::m_sDynObjDataSize; i++ )
	{
		// check for valid cved object
		int cvedId = CHcsmCollection::m_sDynObjData.cvedId[i];
		if( !m_pCved->IsObjValid( cvedId ) ) continue;
		if( printStuff )
		{
			fprintf( stdout, "--%d ", cvedId);
		}
		
		// check the range between ADO's and driver obj.
		CPoint3D objPos;
		objPos.m_x = CHcsmCollection::m_sDynObjData.pos[3*i];
		objPos.m_y = CHcsmCollection::m_sDynObjData.pos[3*i + 1];
		objPos.m_z = CHcsmCollection::m_sDynObjData.pos[3*i + 2];
		float squaredDist = (float)cOwnVehCartPos.DistSq( objPos );
		//->this is not tr
        if( squaredDist > cMaxSqrdRng ) continue;

		// check the algorithm select switch
		if( cAlgorithm == 0 ) break;

		//
		// All checks passed.  Now do work
		//

		// get object data
		float     objVel = CHcsmCollection::m_sDynObjData.vel[i];
		float objHeading = CHcsmCollection::m_sDynObjData.heading[i];
		int        solId = CHcsmCollection::m_sDynObjData.solId[i];

		// calculate heading of obj relative to ownVeh
		float relHeading = objHeading - ownVehHeading;
		if ( relHeading > M_PI  ) relHeading -= 2.0f * (float) M_PI;
 		if ( relHeading < -M_PI ) relHeading += 2.0f * (float) M_PI;

		// if relative heading is greater than pi/4, no warning is given
		if( printStuff )
		{
			fprintf( stdout, "%.1f", fabs(relHeading));
		}
		if ( fabs(relHeading) > M_PI_4 ) continue;
		if( printStuff )
		{
			fprintf( stdout, "+" );
		}

		// closing velocity > 0 if obj is faster than ownVeh
		float closingVelocity = ( objVel - (float) ownVehVel ) * (float) cMETER_TO_FEET;

		// subtract ownVehCartPos and rotate system to a heading of zero
		CTransMat rotMatrix, transMatrix, fwdShift, bakShift;
		transMatrix.Trans( -cOwnVehCartPos.m_x, -cOwnVehCartPos.m_y, -cOwnVehCartPos.m_z );
		fwdShift.Trans( closingVelocity, 0.0f, 0.0f );
		//bakShift.Trans( 0.0f, -closingVelocity, 0.0f );
		CPoint3D transObjPos = transMatrix.Apply( objPos );
        rotMatrix.RotZ( -ownVehHeading );
		CPoint3D rotObjPos = rotMatrix.Apply( transObjPos );

		// get dynamics parameters of obj from the SOL.
		const CSolObj* cpSolObj = m_pCved->GetSol().GetObj( solId );
		if (!cpSolObj) //if we can't get a sol obj, we can't consider the obj
			continue;
		const CSolObjVehicle* cpSolObjVeh = dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
		if (!cpSolObjVeh) //our algorithm only takes vehicles into account
			continue;
		const CDynaParams& dynaParams = cpSolObjVeh->GetDynaParams();
		
		// call one collision avoidance warning algorithm
		float distToObj = sqrt( squaredDist );
		int warning = 0;
		int ttw = 0;
		switch ( cAlgorithm )
		{
			case 0:
				warning = 0; // no collision avoidance warning used
				ttw = 0; // no time to warn since no collision avoidance warning used
				break;
			case 1:
				ttw = TRWCAS( dynaParams, fwdShift.Apply(rotObjPos), relHeading, distToObj, ownVehVel, closingVelocity, 0 );
				warning = TRWCAS( dynaParams, rotObjPos, relHeading, distToObj, ownVehVel, closingVelocity, 0 );
				break;
			case 2:
				ttw = TRWCAS( dynaParams, fwdShift.Apply(rotObjPos), relHeading, distToObj, ownVehVel, closingVelocity, 1 );
				warning = TRWCAS( dynaParams, rotObjPos, relHeading, distToObj, ownVehVel, closingVelocity, 1 );
				break;
			case 3:
				ttw = LPWS( dynaParams, fwdShift.Apply(rotObjPos), relHeading, ownVehVel );
				warning = LPWS( dynaParams, rotObjPos, relHeading, ownVehVel );
				break;
			case 5:
				if( objVel > cAccLeadVehThresholdVel ) 
				{
					double bumperToBumperDist;
					double angleInCone;

					ttw = 0;
					warning = 0;
					if( printStuff )
					{
						fprintf( stdout, "=  " );
					}
					foundObj = ObjInFrontCone( 
									dynaParams, 
									rotObjPos, 
									relHeading, 
									krvAvg, 
									cMaxRange, 
									bumperToBumperDist, 
									angleInCone 
									);

					if( printStuff )
					{
						fprintf( 
							stdout, 
							"relPos(%.4f %.4f %.4f) relHeading %.4f %d  ", 
							rotObjPos.m_x, rotObjPos.m_y, rotObjPos.m_z, relHeading, foundObj
							);
					}
					if( foundObj )
					{
						CPoint3D leadVehPos = m_pCved->GetObjPos( cvedId  );
						double leadObjLength = m_pCved->GetObjLength( cvedId );
						double cabLength = m_cpCabSolObj->GetLength();
						double adjustDist = ( leadObjLength * 0.5f ) + ( cabLength * 0.5f );

						accRegion = 1;
						if( cvedId == cvedIdMemory ) 
						{
							rangeMemory = (float)bumperToBumperDist;
							rangerateMemory = closingVelocity;
							objVelMemory = objVel;
							angleMemory = (float)angleInCone;
						}
						CHcsmCollection::m_sSensorInfo[0] = (float) cvedId;
						CHcsmCollection::m_sSensorInfo[1] = (float) distToObj;
						CHcsmCollection::m_sSensorInfo[2] = (float) ( bumperToBumperDist / ownVehVel );
						CHcsmCollection::m_sSensorInfo[3] = (float) bumperToBumperDist;
						bool calcTtc = objVel < ownVehVel;
						if( calcTtc )
						{
							CHcsmCollection::m_sSensorInfo[4] = (float) ( bumperToBumperDist / -closingVelocity );
						}
						else
						{
							CHcsmCollection::m_sSensorInfo[4] = 10000.0f;
						}
						CHcsmCollection::m_sSensorInfo[5] = (float) objVel;
						CHcsmCollection::m_sSensorInfo[6] = (float) objPos.m_x;
						CHcsmCollection::m_sSensorInfo[7] = (float) objPos.m_y;
						CHcsmCollection::m_sSensorInfo[8] = (float) objPos.m_z;
						CHcsmCollection::m_sSensorInfo[9] = (float) angleInCone;

						if( printStuff )
						{
							fprintf( stdout, "written  " );
						}

						sensorObjectWritten = true;
					}
				}
				else
				{
					ttw = 0;
					warning = 0;
				}
				break;
			case 6 :
				{
					ttw = 0;
					warning = 0;

					double bumperToBumperDist;
					double angleInCone;

					ttw = 0;
					warning = 0;
					if( printStuff )
					{
						fprintf( stdout, "=  " );
					}
					foundObj = ObjInFrontCone( 
										cpSolObjVeh, 
										rotObjPos, 
										relHeading, 
										krvAvg, 
										cMaxRange, 
										bumperToBumperDist, 
										angleInCone 
										);

					if( printStuff )
					{
						fprintf( 
							stdout, 
							"relPos(%.4f %.4f %.4f) objHeading %.4f ownVehHeading %.4f %d  ", 
							rotObjPos.m_x, rotObjPos.m_y, rotObjPos.m_z, objHeading, ownVehHeading, foundObj
							);
					}
					pFcwInfo[0] = (float) 0.0;
					if( foundObj )
					{
						CPoint3D leadVehPos = m_pCved->GetObjPos( cvedId  );
						double leadObjLength = m_pCved->GetObjLength( cvedId );
						double cabLength = m_cpCabSolObj->GetLength();
						double adjustDist = ( leadObjLength * 0.5f ) + ( cabLength * 0.5f );

//						accRegion = 1;
						if (cvedId == cvedIdMemory)
						{
							rangeMemory = (float)bumperToBumperDist;
							rangerateMemory = closingVelocity;
							objVelMemory = objVel;
							angleMemory = (float)angleInCone;
						}

						float ttc = 10000.0f;
						bool calcTtc = objVel < ownVehVel;
						if( calcTtc )
						{
							ttc = (float) ( bumperToBumperDist / -closingVelocity );
						}

						pFcwInfo[0] = (float) 1.0;
						pFcwInfo[1] = (float) cvedId;
						pFcwInfo[2] = (float) bumperToBumperDist;

						if( ttc < 0.0 ) ttc = 0.0;
						double ttcMax = 2.2;
						if( CHcsmCollection::m_sFCW_Severity == 1 )   ttcMax = 3.6;
						if( ttc >= 0.0 && ttc < ttcMax )
						{
							if( printStuff )
							{
								fprintf( stdout, "written  " );
							}

							sensorObjectWritten = true;
							warning = 3;
							pFcwInfo[0] = (float) 2.0;
						}
						pFcwInfo[3] = (float) ttc;
					}
				}
				break;
			default:
				warning = 0; // no collision.  FCW called later (if used)
				ttw = 0;
		}

		// break out warning into left and right warnings
		bool leftWarnInst, rightWarnInst;
		switch ( warning )
		{
			case 0:
				leftWarnInst  = 0;
				rightWarnInst = 0;
				break;
			case 1:
				leftWarnInst  = 1;
				rightWarnInst = 0;
				break;
			case 2:
				leftWarnInst  = 0;
				rightWarnInst = 1;
				break;
			case 3:
				leftWarnInst  = 1;
				rightWarnInst = 1;
				break;
			default:
				leftWarnInst  = 0;
				rightWarnInst = 0;
		}

		// keep track of time-to-warn flag
		bool timeToWarnInst=0;
		if ( ttw > 0 && warning == 0 ) timeToWarnInst = 1;

		// accumulate warnings about other vehicles
		leftWarn  = leftWarn  || leftWarnInst;
		rightWarn = rightWarn || rightWarnInst;
		timeToWarn = timeToWarn || timeToWarnInst;

		if( ( cAlgorithm == 5 || cAlgorithm == 6 )  && sensorObjectWritten )
		{
			// In this mode, we only care about the closest object.  No
			// need to look at other objects.
			break;
		}
	}

	if (!foundObj) {
		if( printStuff ) 
			fprintf(stdout,"No Object");
		accRegion = 2;
		CHcsmCollection::m_sSensorInfo[0] = -1.0f;
		CHcsmCollection::m_sSensorInfo[1] = 0.0f;
		CHcsmCollection::m_sSensorInfo[2] = 0.0f;
		CHcsmCollection::m_sSensorInfo[3] = 0.0f;
		CHcsmCollection::m_sSensorInfo[4] = 0.0f;
		CHcsmCollection::m_sSensorInfo[5] = 0.0f;
		CHcsmCollection::m_sSensorInfo[6] = 0.0f;
		CHcsmCollection::m_sSensorInfo[7] = 0.0f;
		CHcsmCollection::m_sSensorInfo[8] = 0.0f;
		CHcsmCollection::m_sSensorInfo[9] = 0.0f;
	}

	if (cvedIdPrev != CHcsmCollection::m_sSensorInfo[0]) {
		if (cvedIdMemory == CHcsmCollection::m_sSensorInfo[0]) {
			//picked up a known vehicle.  Pick it up immediately
			accOn_ct = cAccLagOnCount;
			accOff_ct = accOn_ct;
		}
		else if (CHcsmCollection::m_sSensorInfo[0] <= 0) {
			//lost vehicle. start lag to lose vehicle
			accOff_ct = 0;
		}
		else {
			//picked up a different, unknown vehicle.  start lag to pick it up and lose the old one.
			accOff_ct = 0;
			accOn_ct = 0;
		}
	}

	if (accOff_ct == cAccLagOffCount && accOff_ct <=  accOn_ct) 
	{
		if (CHcsmCollection::m_sSensorInfo[0]>0) cvedIdMemory = (int)CHcsmCollection::m_sSensorInfo[0];
	}
	if (accOff_ct == 6*cAccLagOffCount && accOff_ct <=  accOn_ct) 
	{
		cvedIdMemory = (int)CHcsmCollection::m_sSensorInfo[0];
	}
	if ( accOn_ct == cAccLagOnCount && accOn_ct <= accOff_ct) 
	{
		cvedIdMemory = (int)CHcsmCollection::m_sSensorInfo[0];
	}

	cvedIdPrev = (int)CHcsmCollection::m_sSensorInfo[0];

	if (cvedIdMemory != CHcsmCollection::m_sSensorInfo[0]) 
	{
		if (cvedIdMemory > 0 && accOff_ct < cAccLagOffCount) 
		{
			accRegion = 3;
			rangeMemory += rangerateMemory/frequencyOfExecution;
			CHcsmCollection::m_sSensorInfo[0] = (float) cvedIdMemory;
			CHcsmCollection::m_sSensorInfo[1] = (float) 0.0;
			CHcsmCollection::m_sSensorInfo[2] = (float) ( rangeMemory / ownVehVel );
			CHcsmCollection::m_sSensorInfo[3] = (float) rangeMemory;
			bool calcTtc = rangerateMemory < 0.0;
			if( calcTtc )
			{
				CHcsmCollection::m_sSensorInfo[4] = (float) ( rangeMemory / -rangerateMemory );
			}
			else
			{
				CHcsmCollection::m_sSensorInfo[4] = 10000.0f;
			}
			CHcsmCollection::m_sSensorInfo[5] = (float) objVelMemory;
			CHcsmCollection::m_sSensorInfo[6] = (float) 0.0;
			CHcsmCollection::m_sSensorInfo[7] = (float) 0.0;
			CHcsmCollection::m_sSensorInfo[8] = (float) 0.0;
			CHcsmCollection::m_sSensorInfo[9] = (float) angleMemory;
		}
		else 
		{
			accRegion = 4;
			CHcsmCollection::m_sSensorInfo[0] = -1.0f;
			CHcsmCollection::m_sSensorInfo[1] = 0.0f;
			CHcsmCollection::m_sSensorInfo[2] = 0.0f;
			CHcsmCollection::m_sSensorInfo[3] = 0.0f;
			CHcsmCollection::m_sSensorInfo[4] = 0.0f;
			CHcsmCollection::m_sSensorInfo[5] = 0.0f;
			CHcsmCollection::m_sSensorInfo[6] = 0.0f;
			CHcsmCollection::m_sSensorInfo[7] = 0.0f;
			CHcsmCollection::m_sSensorInfo[8] = 0.0f;
			CHcsmCollection::m_sSensorInfo[9] = 0.0f;
		}
	}

#if 0
	debug_ct2 += 1;
	if ( debug_ct2 >= 25 )
	{
		cout << " cvedIdMem="	<< cvedIdMemory
			 << " cvedIdOut="	<< CHcsmCollection::m_sSensorInfo[0] 
			 << " cvedIdPrev="	<< cvedIdPrev
			 << " region="	<< accRegion 
			 << " OnCt="	<< accOn_ct 
			 << " OffCt="	<< accOff_ct 
			 << endl;
	    debug_ct2 = 0;
	}
#endif

	//
	// Left Warning Light
	//

	// detect leading and trailing edges of left warning signal
	bool leftWarnLeadEdge = 0, leftWarnTrailEdge = 0;
	if ( ( leftWarn > leftWarn_prev ) )
	{
		leftWarnLeadEdge  = 1;
		leftWarnTrailEdge = 0;
		leftWarnOn_ct     = 0;
#if 0
		cout << " LLeadEdge:  LW="   << leftWarn
			 << " LWprev="   << leftWarn_prev
			 << endl;
#endif
	}
	if ( ( leftWarn < leftWarn_prev ) )
	{
		leftWarnLeadEdge  = 0;
		leftWarnTrailEdge = 1;
		leftWarnOff_ct    = 0;
#if 0
		cout << " LTrailEdge:  LW="   << leftWarn
			 << " LWprev="   << leftWarn_prev
			 << endl;
#endif
	}
	leftWarn_prev  = leftWarn;

	// combinatorial logic to set left warning lights
	bool leftA = leftWarnOn_ct  <  leftWarnOff_ct;
	bool leftB = leftWarnOn_ct  >= cLagOnCount;
	bool leftC = leftWarnOff_ct >= cLagOffCount;
	bool leftWarnLight = ( leftB && !leftC ) || ( leftA && leftB ) || ( leftA && !leftC );

	//
	// Right Warning Light
	//

	// detect leading and trailing edges of right warning signal
	bool rightWarnLeadEdge = 0, rightWarnTrailEdge = 0;
	if ( ( rightWarn > rightWarn_prev ) )
	{
		rightWarnLeadEdge  = 1;
		rightWarnTrailEdge = 0;
		rightWarnOn_ct     = 0;
#if 0
		cout << " RLeadEdge:  RW"   << rightWarn
			 << " RWprev="   << rightWarn_prev
			 << endl;
#endif
	}
	if ( ( rightWarn < rightWarn_prev ) )
	{
		rightWarnLeadEdge  = 0;
		rightWarnTrailEdge = 1;
		rightWarnOff_ct    = 0;
#if 0
		cout << " RTrailEdge:  RW"   << rightWarn
			 << " RWprev="   << rightWarn_prev
			 << endl;
#endif
	}
	rightWarn_prev = rightWarn;

	// combinatorial logic to set right warning lights
	bool rightA = rightWarnOn_ct  <  rightWarnOff_ct;
	bool rightB = rightWarnOn_ct  >= cLagOnCount;
	bool rightC = rightWarnOff_ct >= cLagOffCount;
	bool rightWarnLight = ( rightB && !rightC ) || ( rightA && rightB ) || ( rightA && !rightC );

	// combine left and right warning lights
	if      (  leftWarnLight && !rightWarnLight ) _rWarningLights = 1;
	else if ( !leftWarnLight &&  rightWarnLight ) _rWarningLights = 2;
	else if (  leftWarnLight &&  rightWarnLight ) _rWarningLights = 3;
	else                                          _rWarningLights = 0;

	// call FCW system if being used
	if( cAlgorithm == 4 )
	{
		_rWarningLights = FCW( ownVehVel, pFcwInfo, frequencyOfExecution );
		if( printStuff )
		{
			fprintf( stdout, "  w=%d", _rWarningLights );
		}
	}
	else if( cAlgorithm == 6 )
	{
		if( _rWarningLights > 0 ) 
		{
			_rWarningLights += 64;
		}
	}

	// zero out warning if no algorithm selected
	if( cAlgorithm == 0 ) _rWarningLights = 0;

	if( printStuff )
	{
		fprintf( stdout, "\n" );
	}

	//
	// Debugging
	//

#if 0
	// debugging print statements
	if ( leftWarn==0 && leftWarnLight==1)
	{
		debug_ct1 += 1;
		if ( debug_ct1 >= 100 )
		{
			cout << " _rWarningLights ="   << _rWarningLights
				<< " LW="   << leftWarn
				<< " LA="   << leftA
				<< " LB="   << leftB
				<< " LC="   << leftC
				<< " LOnCt"   << leftWarnOn_ct
				<< " LOffCt"   << leftWarnOff_ct
				<< " Lwl="   << leftWarnLight
				<< endl;
			debug_ct1 = 0;
		}
	}
	if ( rightWarn==0 && rightWarnLight==1)
	{
		debug_ct1 += 1;
		if ( debug_ct1 >= 100 )
		{
			cout << " _rWarningLights ="   << _rWarningLights
				<< " RW="   << rightWarn
				<< " RA="   << rightA
				<< " RB="   << rightB
				<< " RC="   << rightC
				<< " ROnCt"   << rightWarnOn_ct
				<< " ROffCt"   << rightWarnOff_ct
				<< " Rwl="   << rightWarnLight
				<< endl;
			debug_ct1 = 0;
		}
	}
#endif

#if 0
	// debugging print statements
	if ( leftWarn || leftWarnLight )
	{
		debug_ct1 += 1;
		if ( debug_ct1 >= 20 )
		{
			cout << " leftWarn="      << leftWarn 
				 << " leftWarnLight=" << leftWarnLight  
				 << " LWLE="          << leftWarnLeadEdge 
				 << " LWTE="          << leftWarnTrailEdge
				 << endl;
		    debug_ct1 = 0;
		}
	}

	if ( rightWarn || rightWarnLight )
	{
		debug_ct1 += 1;
		if ( debug_ct1 >= 20 )
		{
			cout << " rightWarn="      << rightWarn 
				 << " rightWarnLight=" << rightWarnLight  
				 << " RWLE="           << rightWarnLeadEdge 
				 << " RWTE="           << rightWarnTrailEdge
				 << endl;
				 debug_ct1 = 0;
		}
	}
#endif
}


//////////////////////////////////////////////////////////////////////////////
///\brief TRWCAS
///\remark 
///		Provide functionality of TRW Collision Avoidance System (CAS)
///		algorithms
///
///
//////////////////////////////////////////////////////////////////////////////
int
CScenarioControl::TRWCAS(
		const CDynaParams& dynaParams,
		const CPoint3D rotObjPos,
		const float relHeading,
		const float range, 
		const double ownVehVel, 
		const float vel_closing, 
		const bool useFAZ
		)
{
	const float cProx_front      = 4.0f; // feet
	const float cProx_rear       = 30.0f; // feet
	const float cProx_side       = 11.0f; // feet
	const float cProx_faz        = 162.0f; // feet
	const float cWheelbase_half  = 7.5f; // feet
	const float cTrackwidth_half = 3.75f; // feet
	const float cTprox           = 3.0f; // seconds
	const float cVelMin          = 4.47f; // m/sec
	const float cTrack_margin    = 1.0f; // feet

	// calculate wheel positions and rotate to relative heading
	CTransMat rotMatrix;
	CPoint2D posLF, posLR, posRF, posRR;
	const double wheelBaseForw = dynaParams.m_WheelBaseForw;
	const double wheelBaseRear = dynaParams.m_WheelBaseRear;
	const double wheelTrack    = dynaParams.m_WheelTrack;
	posLF.m_y = rotObjPos.m_y - wheelTrack/2.0f;
	posLF.m_x = rotObjPos.m_x + wheelBaseForw;
	posLR.m_y = rotObjPos.m_y - wheelTrack/2.0f;
	posLR.m_x = rotObjPos.m_x - wheelBaseRear;
	posRF.m_y = rotObjPos.m_y + wheelTrack/2.0f;
	posRF.m_x = rotObjPos.m_x + wheelBaseForw;
	posRR.m_y = rotObjPos.m_y + wheelTrack/2.0f;
	posRR.m_x = rotObjPos.m_x - wheelBaseRear;
	rotMatrix.RotZ( relHeading );
	CPoint2D rotPosLF = rotMatrix.Apply( posLF );
	CPoint2D rotPosLR = rotMatrix.Apply( posLR );
	CPoint2D rotPosRF = rotMatrix.Apply( posRF );
	CPoint2D rotPosRR = rotMatrix.Apply( posRR );

#if 0
	cout << "rotObjPos=" << rotObjPos << endl;
	cout << "rotPosLF="  << rotPosLF  << endl;
	cout << "rotPosLR="  << rotPosLR  << endl;
	cout << "rotPosRF="  << rotPosRF  << endl;
	cout << "rotPosRR="  << rotPosRR  << endl << endl;
#endif

	// define proximity zones and fast approach zones
	CBoundingBox rightProxZone(
			-cProx_rear       - cWheelbase_half, 
			-cProx_side       - cTrackwidth_half, 
			 cProx_front      + cWheelbase_half,
			-cTrackwidth_half - cTrack_margin
			); // right proximity zone
	CBoundingBox rightFAZone(
			-cProx_faz        - cWheelbase_half, 
			-cProx_side       - cTrackwidth_half, 
			-cProx_rear       - cWheelbase_half,
			-cTrackwidth_half - cTrack_margin
			); // right fast approach zone
	CBoundingBox leftProxZone(
			-cProx_rear       - cWheelbase_half, 
			 cTrackwidth_half + cTrack_margin, 
			 cProx_front      + cWheelbase_half,
			 cProx_side       + cTrackwidth_half 
			); // left proximity zone
	CBoundingBox leftFAZone(
			-cProx_faz        - cWheelbase_half, 
			 cTrackwidth_half + cTrack_margin, 
			-cProx_rear       - cWheelbase_half,
			 cProx_side       + cTrackwidth_half 
			); // left fast approach zone

	// test point to see if in proximity zones or fast approach zones
	bool inLeftProxZone  =  leftProxZone.Encloses ( rotPosLF ) 
						 || leftProxZone.Encloses ( rotPosLR ) 
						 || leftProxZone.Encloses ( rotPosRF ) 
						 || leftProxZone.Encloses ( rotPosRR );
	bool inRightProxZone =  rightProxZone.Encloses( rotPosLF ) 
						 || rightProxZone.Encloses( rotPosLR ) 
						 || rightProxZone.Encloses( rotPosRF ) 
						 || rightProxZone.Encloses( rotPosRR );
	bool inLeftFAZone    =  leftFAZone.Encloses   ( rotPosLF ) 
						 || leftFAZone.Encloses   ( rotPosLR ) 
						 || leftFAZone.Encloses   ( rotPosRF ) 
						 || leftFAZone.Encloses   ( rotPosRR );
	bool inRightFAZone   =  rightFAZone.Encloses  ( rotPosLF ) 
						 || rightFAZone.Encloses  ( rotPosLR ) 
						 || rightFAZone.Encloses  ( rotPosRF ) 
						 || rightFAZone.Encloses  ( rotPosRR );

	// determine if obj is closing in fast on ownVeh
	bool closingFast = 0;
	double prox_warn = cProx_rear + vel_closing * cTprox;
	if ( range < prox_warn ) closingFast = 1;

	// activate warnings as appropriate
	bool leftWarn = 0, rightWarn = 0;
	if ( inLeftProxZone                         ) leftWarn  = 1; // test if in proximity zone
	if ( inRightProxZone                        ) rightWarn = 1; // test if in proximity zone
	if ( inLeftFAZone  && closingFast && useFAZ ) leftWarn  = 1; // test if in FAZ and closing
	if ( inRightFAZone && closingFast && useFAZ ) rightWarn = 1; // test if in FAZ and closing

	// deactivate warnings if speed is below threshold
	if ( ownVehVel < cVelMin ) // min speed for warnings not met
	{
		leftWarn = 0;
		rightWarn = 0;
	}

	// combine left and right warnings and return
	int warning;
	if      (  leftWarn && !rightWarn ) warning = 1;
	else if ( !leftWarn &&  rightWarn ) warning = 2;
	else if (  leftWarn &&  rightWarn ) warning = 3;
	else                                warning = 0;

	//
	// Debugging
	//

#if 0
	if ( leftWarn )
	{
		debug_ct2 += 1;
		if ( debug_ct2 >= 20 )
		{
			cout << " rotObjPos=" << rotObjPos 
				 << " CF="        << closingFast  
				 << " LW="        << leftWarn 
				 << " warn="      << warning 
				 << endl;
			cout << "rotPosLF="  << rotPosLF  << endl;
			cout << "rotPosLR="  << rotPosLR  << endl;
			cout << "rotPosRF="  << rotPosRF  << endl;
			cout << "rotPosRR="  << rotPosRR  << endl;
			cout << "rightProxZone=" << rightProxZone << endl;
			cout << "leftProxZone="  << leftProxZone << endl;
			cout << inLeftProxZone << inRightProxZone << inLeftFAZone << inRightFAZone << endl;
		    debug_ct2 = 0;
		}
	}

	if ( rightWarn )
	{
		debug_ct2 += 1;
		if ( debug_ct2 >= 20 )
		{
			cout << " rotObjPos=" << rotObjPos 
				 << " CF="        << closingFast 
				 << " RW="        << rightWarn 
				 << " warn="      << warning 
				 << endl;
			cout << "rotPosLF="  << rotPosLF  << endl;
			cout << "rotPosLR="  << rotPosLR  << endl;
			cout << "rotPosRF="  << rotPosRF  << endl;
			cout << "rotPosRR="  << rotPosRR  << endl;
			cout << "rightProxZone=" << rightProxZone << endl;
			cout << "leftProxZone="  << leftProxZone << endl;
			cout << inLeftProxZone << inRightProxZone << inLeftFAZone << inRightFAZone << endl;
			debug_ct2 = 0;
		}
	}
#endif

	return warning;
}


//////////////////////////////////////////////////////////////////////////////
///\brief ALIRT
///\remark  
///			Provide functionality of ALIRT Collision Avoidance System (CAS)
///			algorithm
///
/// Remarks:  
///
/// Arguments:
///
/// Returns: 
///
//////////////////////////////////////////////////////////////////////////////
int
CScenarioControl::LPWS( 
		const CDynaParams& dynaParams,
		const CPoint3D rotObjPos,
		const float relHeading,
		const double ownVehVel
		)
{
	const float cWheelbase_half  = 7.5f; // feet
	const float cTrackwidth_half = 3.75f; // feet
	const float cMirror_loc_x    = 4.0f; // feet
	const float cMirror_loc_y    = cTrackwidth_half; // feet
	const float cLeftMirrorHFOV  = 12.9f; // inner angle parallel to side of vehicle
	const float cRightMirrorHFOV = 22.5f; // inner angle parallel to side of vehicle
	const float cLeftDetectHFOV  = 90.0f - cLeftMirrorHFOV; // overlap 5deg with mirrorHFOV
	const float cRightDetectHFOV = 90.0f - cRightMirrorHFOV; // overlap 5deg with mirrorHFOV
	const float cOverlap         = 5.0f; // overlap between mirror and detect HFOVs
	const float cProx_side       = 14.0f; // side detection limit
	const float cProx_rear       = 50.0f; // rear detection limit
	const float cVelMin          = 8.94f; // m/sec

	CPoint2D leftMirrorLoc, rightMirrorLoc;
	CPoint2D leftFrontLineExt, rightFrontLineExt;
	CPoint2D leftRearLineExt, rightRearLineExt;

	// calculate wheel positions and rotate to relative heading
	CTransMat rotMatrix;
	CPoint2D posLF, posLR, posRF, posRR;
	const double wheelBaseForw = dynaParams.m_WheelBaseForw;
	const double wheelBaseRear = dynaParams.m_WheelBaseRear;
	const double wheelTrack    = dynaParams.m_WheelTrack;
	posLF.m_y = rotObjPos.m_y - wheelTrack/2.0f;
	posLF.m_x = rotObjPos.m_x + wheelBaseForw;
	posLR.m_y = rotObjPos.m_y - wheelTrack/2.0f;
	posLR.m_x = rotObjPos.m_x - wheelBaseRear;
	posRF.m_y = rotObjPos.m_y + wheelTrack/2.0f;
	posRF.m_x = rotObjPos.m_x + wheelBaseForw;
	posRR.m_y = rotObjPos.m_y + wheelTrack/2.0f;
	posRR.m_x = rotObjPos.m_x - wheelBaseRear;
	rotMatrix.RotZ( relHeading );
	CPoint2D rotPosLF = rotMatrix.Apply( posLF );
	CPoint2D rotPosLR = rotMatrix.Apply( posLR );
	CPoint2D rotPosRF = rotMatrix.Apply( posRF );
	CPoint2D rotPosRR = rotMatrix.Apply( posRR );

	//calculate angles for the four boundary lines
	double radiansLF =   cOverlap                      * M_PI / 180.0;
	double radiansLR = ( cOverlap + cLeftDetectHFOV  ) * M_PI / 180.0;
	double radiansRF =   cOverlap                      * M_PI / 180.0;
	double radiansRR = ( cOverlap + cRightDetectHFOV ) * M_PI / 180.0;

	// Define two points for each of four boundary lines.
	leftMirrorLoc.m_x     =  cMirror_loc_x;
	leftMirrorLoc.m_y     =  cMirror_loc_y;
	leftFrontLineExt.m_x  =  leftMirrorLoc.m_x - tan(radiansLF);
	leftFrontLineExt.m_y  =  leftMirrorLoc.m_y + 1.0;
	leftRearLineExt.m_x   =  leftMirrorLoc.m_x - tan(radiansLR);
	leftRearLineExt.m_y   =  leftMirrorLoc.m_y + 1.0;
	rightMirrorLoc.m_x    =  cMirror_loc_x;
	rightMirrorLoc.m_y    = -cMirror_loc_y;
	rightFrontLineExt.m_x =  rightMirrorLoc.m_x - tan(radiansRF);
	rightFrontLineExt.m_y =  rightMirrorLoc.m_y - 1.0;
	rightRearLineExt.m_x  =  rightMirrorLoc.m_x - tan(radiansRR);
	rightRearLineExt.m_y  =  rightMirrorLoc.m_y - 1.0;

	// Define the boundary lines by their points
	CLineSeg2D  leftFrontLine( leftMirrorLoc,  leftFrontLineExt  );
	CLineSeg2D   leftRearLine( leftMirrorLoc,  leftRearLineExt   );
	CLineSeg2D rightFrontLine( rightMirrorLoc, rightFrontLineExt );
	CLineSeg2D  rightRearLine( rightMirrorLoc, rightRearLineExt  );

	// Define two proximity zones that bound the outside of the detection area
	CBoundingBox rightProxZone(
			-cProx_rear - cWheelbase_half, 
			-cProx_side - cMirror_loc_x, 
			 cMirror_loc_y,
			-cMirror_loc_x 
			); // right proximity zone
	CBoundingBox leftProxZone(
			-cProx_rear - cWheelbase_half, 
			 cMirror_loc_x, 
			 cMirror_loc_y,
			 cProx_side + cMirror_loc_x 
			); // left proximity zone

	// test points to see if in detection zones
	bool inLeftProxZone  =   leftProxZone.Encloses          ( rotPosLF ) 
					      || leftProxZone.Encloses          ( rotPosLR ) 
					      || leftProxZone.Encloses          ( rotPosRF ) 
					      || leftProxZone.Encloses          ( rotPosRR );
	bool inRightProxZone =   rightProxZone.Encloses         ( rotPosLF ) 
					      || rightProxZone.Encloses         ( rotPosLR ) 
					      || rightProxZone.Encloses         ( rotPosRF ) 
					      || rightProxZone.Encloses         ( rotPosRR );
	bool inLeftDetect    = ( leftFrontLine.IsPtOnLeftSide   ( rotPosLF ) 
					      && leftRearLine.IsPtOnRightSide   ( rotPosLF ) )
					    || ( leftFrontLine.IsPtOnLeftSide   ( rotPosLR ) 
					      && leftRearLine.IsPtOnRightSide   ( rotPosLR ) )
					    || ( leftFrontLine.IsPtOnLeftSide   ( rotPosRF ) 
					      && leftRearLine.IsPtOnRightSide   ( rotPosRF ) )
					    || ( leftFrontLine.IsPtOnLeftSide   ( rotPosRR ) 
					      && leftRearLine.IsPtOnRightSide   ( rotPosRR ) );
	bool inRightDetect   = ( rightFrontLine.IsPtOnRightSide ( rotPosLF ) 
					      && rightRearLine.IsPtOnLeftSide   ( rotPosLF ) ) 
					    || ( rightFrontLine.IsPtOnRightSide ( rotPosLR ) 
					      && rightRearLine.IsPtOnLeftSide   ( rotPosLR ) ) 
					    || ( rightFrontLine.IsPtOnRightSide ( rotPosRF ) 
					      && rightRearLine.IsPtOnLeftSide   ( rotPosRF ) ) 
					    || ( rightFrontLine.IsPtOnRightSide ( rotPosRR ) 
					      && rightRearLine.IsPtOnLeftSide   ( rotPosRR ) );

	// activate warnings as appropriate
	bool leftWarn=0, rightWarn=0;
	if ( inLeftDetect  && inLeftProxZone  ) leftWarn  = 1;
	if ( inRightDetect && inRightProxZone ) rightWarn = 1;

	// deactivate warnings if speed is below threshold
	if ( ownVehVel < cVelMin ) // min speed for warnings is 10mph
	{
		leftWarn  = 0;
		rightWarn = 0;
	}

	// combine left and right warnings and return
	int warning;
	if      (  leftWarn && !rightWarn ) warning = 1;
	else if ( !leftWarn &&  rightWarn ) warning = 2;
	else if (  leftWarn &&  rightWarn ) warning = 3;
	else                                warning = 0;

	//
	// Debugging
	//
#if 0
	if ( leftWarn )
	{
		debug_ct2 += 1;
		if ( debug_ct2 >= 100 )
		{
			cout << " LW:  rotObjPos=" << rotObjPos 
				 << " warn="           << warning 
				 << " relH="           << relHeading 
				 << " posLR="          << rotPosLR 
				 << " posRR="          << rotPosRR 
				 << endl;
		    debug_ct2 = 0;
		}
	}

	if ( rightWarn )
	{
		debug_ct2 += 1;
		if ( debug_ct2 >= 100 )
		{
			cout << " RW:  rotObjPos=" << rotObjPos 
				 << " warn="           << warning 
				 << " relH="           << relHeading 
				 << " posLR="          << rotPosLR 
				 << " posRR="          << rotPosRR 
				 << endl;
		    debug_ct2 = 0;
		}
	}
#endif

	return warning;
}


//////////////////////////////////////////////////////////////////////////////
///\brief ACC
///\remark  
///			Provide functionality of cone emanating from the vehicle's front
///  similar to an ACC cone
///
/// Remarks:  
///
/// Arguments:
///
/// Returns: 
///
//////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::ObjInFrontCone( 
		const CDynaParams& dynaParams,
		const CPoint3D rotObjPos,
		const float relHeading,
		const float curvature,
		const double cMaxRange,
		double& bumperToBumperDist,
		double& angleInCone
		)
{
	const float cVehicleLength = 15.74f;
	CPoint2D sensorLocation( cVehicleLength * 0.5, 0.0 );

	// calculate wheel positions and rotate to relative heading
	CTransMat rotMatrix;
	CPoint2D posLF, posLR, posRF, posRR, posMLR, posMRR;
	const double wheelBaseForw = dynaParams.m_WheelBaseForw;
	const double wheelBaseRear = dynaParams.m_WheelBaseRear;
	const double wheelTrack    = dynaParams.m_WheelTrack;
	posLF.m_y = wheelTrack/2.0f;
	posLF.m_x = wheelBaseForw;
	posLR.m_y = wheelTrack/2.0f;
	posLR.m_x = -wheelBaseRear;
	posMLR.m_y = wheelTrack/4.0f;
	posMLR.m_x = -wheelBaseRear;
	posRF.m_y = -wheelTrack/2.0f;
	posRF.m_x = wheelBaseForw;
	posRR.m_y = -wheelTrack/2.0f;
	posRR.m_x = -wheelBaseRear;
	posMRR.m_y = -wheelTrack/4.0f;
	posMRR.m_x = -wheelBaseRear;
	rotMatrix.RotZ( relHeading );
	CPoint2D rotPosLF = rotMatrix.Apply( posLF );
	CPoint2D rotPosLR = rotMatrix.Apply( posLR );
	CPoint2D rotPosMLR = rotMatrix.Apply( posMLR );
	CPoint2D rotPosRF = rotMatrix.Apply( posRF );
	CPoint2D rotPosRR = rotMatrix.Apply( posRR );
	CPoint2D rotPosMRR = rotMatrix.Apply( posMRR );
	rotPosLF.m_y += rotObjPos.m_y;
	rotPosLF.m_x += rotObjPos.m_x;
	rotPosLR.m_y += rotObjPos.m_y;
	rotPosLR.m_x += rotObjPos.m_x;
	rotPosMLR.m_y += rotObjPos.m_y;
	rotPosMLR.m_x += rotObjPos.m_x;
	rotPosRF.m_y += rotObjPos.m_y;
	rotPosRF.m_x += rotObjPos.m_x;
	rotPosRR.m_y += rotObjPos.m_y;
	rotPosRR.m_x += rotObjPos.m_x;
	rotPosMRR.m_y += rotObjPos.m_y;
	rotPosMRR.m_x += rotObjPos.m_x;

	//calculate angles for the four boundary lines
	const double cConeSize = CHcsmCollection::m_sSensor_Config[eSENSOR_INDEX_CONE_ANGLE]; // degrees
	double coneLeftAngle = ( cConeSize / 2 ) * cDEG_TO_RAD;
	double coneRightAngle = ( cConeSize / 2 ) * cDEG_TO_RAD;
	CPoint2D coneLeftPoint( tan( (M_PI / 2) - coneLeftAngle ), 1.0 );
	CPoint2D coneRightPoint( tan( (M_PI / 2) - coneRightAngle ), -1.0 );
	CLineSeg2D coneLeftLine( sensorLocation, coneLeftPoint+sensorLocation );
	CLineSeg2D coneRightLine( sensorLocation, coneRightPoint+sensorLocation );

	// test points to see if in detection zone
	bool inConeDetect = (
		( coneRightLine.IsPtOnLeftSide( rotPosLR  ) && coneLeftLine.IsPtOnRightSide( rotPosLR  ) ) || 
		( coneRightLine.IsPtOnLeftSide( rotPosMLR ) && coneLeftLine.IsPtOnRightSide( rotPosMLR ) ) || 
		( coneRightLine.IsPtOnLeftSide( rotPosRR  ) && coneLeftLine.IsPtOnRightSide( rotPosRR  ) ) || 
		( coneRightLine.IsPtOnLeftSide( rotPosMRR ) && coneLeftLine.IsPtOnRightSide( rotPosMRR ) )
		//( coneRightLine.IsPtOnLeftSide( rotObjPos ) && coneLeftLine.IsPtOnRightSide( rotObjPos ) )
		);

	// activate warnings as appropriate
	bool inConeWarning = inConeDetect;
	float x_ado, y_ado, x_0, y_0, R2_ado, R2_min, R2_max, radius;
	if( inConeWarning )
	{
		// find the distance to the closest corner
		double minDist = sensorLocation.DistSq( rotPosLF );
		double tempDist = sensorLocation.DistSq( rotPosLR );
		if( tempDist < minDist )  minDist = tempDist;
		tempDist = sensorLocation.DistSq( rotPosRF );
		if( tempDist < minDist )  minDist = tempDist;
		tempDist = sensorLocation.DistSq( rotPosRR );
		if( tempDist < minDist )  minDist = tempDist;
		bumperToBumperDist = sqrt( minDist );

		x_ado = float(rotObjPos.m_x-sensorLocation.m_x);
		y_ado = float(rotObjPos.m_y-sensorLocation.m_y);
		angleInCone = atan2(x_ado,y_ado) * cRAD_TO_DEG - 90.0;

		if (abs(curvature) > 0.0001f) {
			radius = 1.0f/abs(curvature);
			y_ado = float(bumperToBumperDist * sin(angleInCone*cDEG_TO_RAD));
			x_0 = 0.0;
			y_0 = radius * SIGN(curvature);
			R2_ado = float(rotObjPos.m_x*rotObjPos.m_x + (y_ado-y_0)*(y_ado-y_0));
			R2_min = (radius - 7.0f) * (radius - 7.0f);
			R2_max = (radius + 7.0f) * (radius + 7.0f);
			if ( R2_ado < R2_min || R2_ado > R2_max ) inConeWarning = false;
		}
		else {
			R2_ado = 0.0;
			R2_min = 0.0;
			R2_max = 0.0;
			y_0 = 0.0;
			if ( y_ado < -7.0 || y_ado > 7.0 ) inConeWarning = false;
		}


#if 0
		debug_ct2 += 1;
		if ( debug_ct2 >= 30 )
		{
			cout << " rotObjPos.m_x="	<< rotObjPos.m_x
				 << " rotObjPos.m_y="	<< rotObjPos.m_y
				 << " rotPosLR.m_x="	<< rotPosLR.m_x 
				 << " rotPosLR.m_y="	<< rotPosLR.m_y
				 << " rotPosRR.m_x="	<< rotPosRR.m_x
				 << " rotPosRR.m_y="	<< rotPosRR.m_y
				 << endl;
		    debug_ct2 = 0;
		}
#endif

#if 0
		debug_ct2 += 1;
		if ( debug_ct2 >= 30 )
		{
			cout << " R2_min="	<< sqrt(R2_min)
				 << " R2_ado="	<< sqrt(R2_ado)
				 << " R2_max="	<< sqrt(R2_max) 
				 << " y_0="		<< y_0 
				 << " y_ado="	<< y_ado
				 << " Krv="		<< curvature
				 << endl;
		    debug_ct2 = 0;
		}
#endif


#if 0
		debug_ct2 += 1;
		if ( debug_ct2 >= 20 )
		{
			cout << " angle="	<< angleInCone
				 << " onleft="	<< coneRightLine.IsPtOnLeftSide( rotObjPos ) 
				 << " onright="	<< coneLeftLine.IsPtOnRightSide( rotObjPos ) 
				 << " m_x="		<< rotObjPos.m_x 
				 << " m_y="		<< rotObjPos.m_y 
				 << endl;
		    debug_ct2 = 0;
		}
#endif
	}

	return inConeWarning;
}


//////////////////////////////////////////////////////////////////////////////
///\brief ACC
///\remark  
///			Provide functionality of cone emanating from the vehicle's front
///  similar to an ACC cone
///
/// Remarks:  
///
/// Arguments:
///
/// Returns: 
///
//////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::ObjInFrontCone( 
		const CSolObjVehicle* cpSolObjVeh,
		const CPoint3D rotObjPos,
		const float relHeading,
		const float curvature,
		const double cMaxRange,
		double& bumperToBumperDist,
		double& angleInCone
		)
{
	const float cVehicleLength = 15.74f;
	CPoint2D sensorLocation( cVehicleLength * 0.5, 0.0 );

	// calculate wheel positions and rotate to relative heading
	CTransMat rotMatrix;
	CPoint2D posLF, posLR, posRF, posRR, posMLR, posMRR;
	const double length = cpSolObjVeh->GetLength();
	const double width = cpSolObjVeh->GetWidth();
	const double height = cpSolObjVeh->GetHeight();
	posLF.m_y = width/2.0f;
	posLF.m_x = length/2.0f;
	posLR.m_y = width/2.0f;
	posLR.m_x = -length/2.0f;
	posRF.m_y = -width/2.0f;
	posRF.m_x = length/2.0f;
	posRR.m_y = -width/2.0f;
	posRR.m_x = -length/2.0f;
	rotMatrix.RotZ( relHeading );
	CPoint2D rotPosLF = rotMatrix.Apply( posLF );
	CPoint2D rotPosLR = rotMatrix.Apply( posLR );
	CPoint2D rotPosRF = rotMatrix.Apply( posRF );
	CPoint2D rotPosRR = rotMatrix.Apply( posRR );
	rotPosLF.m_y += rotObjPos.m_y;
	rotPosLF.m_x += rotObjPos.m_x;
	rotPosLR.m_y += rotObjPos.m_y;
	rotPosLR.m_x += rotObjPos.m_x;
	rotPosRF.m_y += rotObjPos.m_y;
	rotPosRF.m_x += rotObjPos.m_x;
	rotPosRR.m_y += rotObjPos.m_y;
	rotPosRR.m_x += rotObjPos.m_x;

	//calculate angles for the four boundary lines
	const double cConeSize = CHcsmCollection::m_sSensor_Config[eSENSOR_INDEX_CONE_ANGLE]; // degrees
	double coneLeftAngle = ( cConeSize / 2 ) * cDEG_TO_RAD;
	double coneRightAngle = ( cConeSize / 2 ) * cDEG_TO_RAD;
	CPoint2D coneLeftPoint( tan( (M_PI / 2) - coneLeftAngle ), 1.0 );
	CPoint2D coneRightPoint( tan( (M_PI / 2) - coneRightAngle ), -1.0 );
	CLineSeg2D coneLeftLine( sensorLocation, coneLeftPoint+sensorLocation );
	CLineSeg2D coneRightLine( sensorLocation, coneRightPoint+sensorLocation );

	// test points to see if in detection zone
	bool inConeDetect = (
		( coneRightLine.IsPtOnLeftSide( rotPosLR  ) && coneLeftLine.IsPtOnRightSide( rotPosLR  ) ) || 
		( coneRightLine.IsPtOnLeftSide( rotPosRR  ) && coneLeftLine.IsPtOnRightSide( rotPosRR  ) )
		//( coneRightLine.IsPtOnLeftSide( rotObjPos ) && coneLeftLine.IsPtOnRightSide( rotObjPos ) )
		);

	// activate warnings as appropriate
	bool inConeWarning = inConeDetect;
	float x_ado, y_ado, x_0, y_0, R2_ado, R2_min, R2_max, radius;
	if( inConeWarning )
	{
		// find the distance to the closest corner
		double minDist = sensorLocation.DistSq( rotPosLF );
		double tempDist = sensorLocation.DistSq( rotPosLR );
		if( tempDist < minDist )  minDist = tempDist;
		tempDist = sensorLocation.DistSq( rotPosRF );
		if( tempDist < minDist )  minDist = tempDist;
		tempDist = sensorLocation.DistSq( rotPosRR );;
		if( tempDist < minDist )  minDist = tempDist;
		bumperToBumperDist = sqrt( minDist );

		x_ado = float(rotObjPos.m_x-sensorLocation.m_x);
		y_ado = float(rotObjPos.m_y-sensorLocation.m_y);
		angleInCone = atan2(x_ado,y_ado) * cRAD_TO_DEG - 90.0;

		if (abs(curvature) > 0.0001) {
			radius = float(1.0/abs(curvature));
			y_ado = float(bumperToBumperDist * sin(angleInCone*cDEG_TO_RAD));
			x_0 = 0.0f;
			y_0 = radius * SIGN(curvature);
			R2_ado = (float)(rotObjPos.m_x*rotObjPos.m_x) + (y_ado-y_0)*(y_ado-y_0);
			R2_min = (radius - 7.0f) * (radius - 7.0f);
			R2_max = (radius + 7.0f) * (radius + 7.0f);
			if ( R2_ado < R2_min || R2_ado > R2_max ) inConeWarning = false;
		}
		else {
			R2_ado = 0.0;
			R2_min = 0.0;
			R2_max = 0.0;
			y_0 = 0.0;
			if ( y_ado < -7.0 || y_ado > 7.0 ) inConeWarning = false;
		}


#if 0
		debug_ct2 += 1;
		if ( debug_ct2 >= 30 )
		{
			cout << " rotObjPos.m_x="	<< rotObjPos.m_x
				 << " rotObjPos.m_y="	<< rotObjPos.m_y
				 << " rotPosLR.m_x="	<< rotPosLR.m_x 
				 << " rotPosLR.m_y="	<< rotPosLR.m_y
				 << " rotPosRR.m_x="	<< rotPosRR.m_x
				 << " rotPosRR.m_y="	<< rotPosRR.m_y
				 << endl;
		    debug_ct2 = 0;
		}
#endif

#if 0
		debug_ct2 += 1;
		if ( debug_ct2 >= 30 )
		{
			cout << " R2_min="	<< sqrt(R2_min)
				 << " R2_ado="	<< sqrt(R2_ado)
				 << " R2_max="	<< sqrt(R2_max) 
				 << " y_0="		<< y_0 
				 << " y_ado="	<< y_ado
				 << " Krv="		<< curvature
				 << endl;
		    debug_ct2 = 0;
		}
#endif


#if 0
		debug_ct2 += 1;
		if ( debug_ct2 >= 20 )
		{
			cout << " angle="	<< angleInCone
				 << " onleft="	<< coneRightLine.IsPtOnLeftSide( rotObjPos ) 
				 << " onright="	<< coneLeftLine.IsPtOnRightSide( rotObjPos ) 
				 << " m_x="		<< rotObjPos.m_x 
				 << " m_y="		<< rotObjPos.m_y 
				 << endl;
		    debug_ct2 = 0;
		}
#endif
	}

	return inConeWarning;
}


//////////////////////////////////////////////////////////////////////////////
///
///\brief FCW Provide functionality of RECAS Forward Collision Warning (FCW) System
///
///
///\arg ownVehVel own vehicle velocity
///\arg pFcwInfo[4]
///		float[0]	FCW Zone	(0,1,2,3)
///		float[1]	Time-to-warn	(sec)
///		float[2]	Warning range	(feet)
///		float[3]	Lead vehicle decal (ft/sec^2)
///
///\return	warning
///
//////////////////////////////////////////////////////////////////////////////
int
CScenarioControl::FCW(
		const double ownVehVel, 
		float* pFcwInfo,
		float frequencyOfExecution
		)
{
	//vel_lead = velocity of lead vehicle (ft/sec)
	//decel_lead = deceleration of the lead vehicle (ft/sec^2) (decel_lead = -al)
	//range = bumper-to-bumper dist (ft)
	const float    cDecel_follow = 0.75f*32.2f; // assumed following vehicle deceleration (ft/sec^2)
	const float          cBuffer = 6.67f; // buffer distance (ft)
	const float   cReaction_time = 1.5f; // reaction time (sec)
	const float         cEpsilon = 0.0000001f; // epsilon.  prevents division by zero
	const float       vel_follow = (float) ownVehVel * (float) cMETER_TO_FEET; // velocity of following vehicle (ft/sec)
	const float        leadObjId = CHcsmCollection::m_sFollowInfo[0];
	const float false_alarm_cond = CHcsmCollection::m_sFalseAlarmInfo[0];

	// declare local variables
	float range, ranger, vel_lead, decel_lead, headway, ttc;
	float range_init = 0, vel_lead_init = 0;
	int i;

	 headway = CHcsmCollection::m_sFollowInfo[2];
	   range = CHcsmCollection::m_sFollowInfo[3];
	vel_lead = CHcsmCollection::m_sFollowInfo[5];
	  ranger = vel_lead - vel_follow;
	  
	if ( abs(ranger)>0.01 ) 
	{
		ttc = -range / ranger;
	} 
	else 
	{
		ttc = -range / 0.01f * SIGN(ranger);
	}

	// test for false alarm trigger or reset
	if ( false_alarm_cond==3 && false_alarm_cond_prev<3 ) 
	{
		false_alarm_trigger = 1;
		  false_alarm_reset = 0;
	}
	if ( false_alarm_cond_prev==3 && false_alarm_cond<3 ) 
	{
		false_alarm_trigger = 0;
		  false_alarm_reset = 1;
	}
	false_alarm_cond_prev = false_alarm_cond;

	// increment false alarm time
	if ( false_alarm_trigger ) false_time = false_time + (1.0f/frequencyOfExecution);
	if ( false_alarm_reset   ) false_time = 0.0f;

	// calculate vel_lead, range, headway, and decel_lead
	// initially no warning
	int warning = 0;
	if ( ( false_alarm_cond == 3 ) && ( false_time < 2.0 ) ) 
	{
	         range_init = CHcsmCollection::m_sFalseAlarmInfo[1];
	      vel_lead_init = CHcsmCollection::m_sFalseAlarmInfo[2];
		     decel_lead = -CHcsmCollection::m_sFalseAlarmInfo[3];
			   vel_lead = vel_lead_init - decel_lead * false_time; // remember decel_lead is positive!
		  float vel_rel = vel_lead - vel_follow;
	              range = range_init + range_prev + vel_rel * false_time;
		        headway = range / vel_follow;
			 range_prev = range - range_init;
	}
	else if ( leadObjId <= 0.0 ) 
	{
		dl_buf[buf_idx] = 0.0f;
		buf_idx++;
		if ( buf_idx > BUFLEN-1 ) buf_idx = 0;

		// output FCW variable to SCC_FCW_Info
		pFcwInfo[0] = (float) 0.0f;
		pFcwInfo[1] = (float) 999.0f;
		pFcwInfo[2] = (float) 0.0f;
		pFcwInfo[3] = (float) 0.0f;

		return warning;
	}
	else 
	{
		 warning = 4;
		if (leadObjId == leadObjId_prev) 
		{
			if ( leadVehVel_prev == vel_lead ) 
			{
				decel_lead = decel_lead_prev;
			}
			else 
			{
				decel_lead = (leadVehVel_prev - vel_lead) * frequencyOfExecution / 2.0f;
			}
		}
		else 
		{
			decel_lead = 0.0f;
		}
	}

	// save information for next frame
	leadObjId_prev = leadObjId;
	leadVehVel_prev = vel_lead;
	decel_lead_prev = decel_lead;

	// update buffer and average
	// increment buffer index, with wrap-around
	dl_buf[buf_idx] = decel_lead;
	float dl_avg = 0.0f, h_avg = 0.0f;
	for ( i=0; i<BUFLEN; i++ ) dl_avg += dl_buf[i] / (float) BUFLEN;
	buf_idx++;
	if ( buf_idx > BUFLEN-1 ) buf_idx = 0;

	// output FCW variable to SCC_FCW_Info
	pFcwInfo[3] = (float) dl_avg;

	// limit decel_lead to be strictly positive
	if ( dl_avg < cEpsilon ) dl_avg = cEpsilon; 

	// FCW algorithm
	double tc, tw, rthresh;
	int zone = 0;
	double   Th = range               /   vel_follow;
	double Th12 = vel_lead            /   dl_avg
				+ vel_follow          / ( 2.0 * cDecel_follow )
				- vel_lead * vel_lead / ( 2.0 * dl_avg * vel_follow )
				+ cBuffer             /   vel_follow
				+ cReaction_time;
	double Th23 = vel_lead            /   dl_avg
				- vel_follow          / ( 2.0 * cDecel_follow )
				- vel_lead * vel_lead / ( 2.0 * dl_avg * vel_follow )
				+ cBuffer             /   vel_follow;
	double  tw1 = range               /   vel_follow
				- vel_follow          / ( 2.0 * cDecel_follow )
				- cBuffer             /   vel_follow
				- cReaction_time;
	double  tw2 = range               /   vel_follow
				- vel_follow          / ( 2.0 * cDecel_follow )
				+ vel_lead * vel_lead / ( 2.0 * dl_avg * vel_follow )
				- cBuffer             /   vel_follow
				- cReaction_time;

	double   k1 = ( vel_lead - vel_follow )  / cDecel_follow;
	double   k2 = ( cDecel_follow - dl_avg ) / cDecel_follow;

	double    a = dl_avg                  / 2.0
				- cDecel_follow           / 2.0
				+ cDecel_follow * k2
				- k2 * k2 * cDecel_follow / 2.0;
	double    b = vel_follow
				+ cDecel_follow * k1
				- cDecel_follow * k1 * k2
				- vel_lead
				+ cEpsilon;
	double    c = -range
				- cDecel_follow * k1 * k1 / 2.0
				+ cBuffer;

	if ( a <= 0.0 )
	{
		tc = -c / ( b + cEpsilon);
	}
	else
	{
		double val = b * b - 4.0 * a * c;
		if (val<0) val = 0;
		tc = -( b / ( 2.0 * a ) ) + ( 1.0 / ( 2.0 * a ) ) * sqrt( val );
	}

	double tw3 = k1 + k2 * tc - cReaction_time;

	if ( Th < Th23 )
	{
		zone = 3;
		tw = tw3;
	}

	if ( Th > Th23 )
	{
		zone = 2;
		tw = tw2;
	}
	
	if ( ( Th > Th12 ) || ( vel_lead <= cEpsilon ) )
	{
		zone = 1;
		tw = tw1;
	}

	if ( tw < 0.0f )
	{
		tw = 0.0f;
	}
	else if ( tw > 999.0f )
	{
		tw = 999.0f;
	}

	if ( vel_follow > 0.0f ) 
	{
		rthresh = range + ( vel_lead - vel_follow ) * tw - dl_avg * tw * tw / 2.0;
	}
	else {
		rthresh = 0;
		zone = 0;
	}

	// calculate warning based on headway and time-to-warn from FCW
	if ( headway < 1.0 || tw < 2.0 ) warning = 8;
	if ( headway < 0.5 || tw < 1.0 ) warning = 16;
	if ( vel_lead <= vel_follow ) 
	{
		if ( range <= rthresh ) 
		{
			warning = 32;
		}
	}
	if ( ttc > 0 && ttc <= 2.1 ) warning = warning + 64;

	// output FCW variable to SCC_FCW_Info
	pFcwInfo[0] = (float) zone;
	pFcwInfo[1] = (float) tw;
	pFcwInfo[2] = (float) rthresh;

	//
	// Debugging
	//
#if 0
	// debugging print statements
	debug_ct1 += 1;
	if ( debug_ct1 >= 100 )
	{
		cout << " dl ="  << dl_avg
		     << " hw ="  << headway
		     << " tw ="  << tw
		     << " vf ="  << vel_follow
		     << " vl ="  << vel_lead
		     << " zn ="  << zone
		     << " r ="   << range
		     << " rth =" << rthresh
		     << " w ="   << warning
			 << endl;
		debug_ct1 = 0;
	}
#endif
#if 0
	// debugging print statements
	debug_ct1 += 1;
	if ( debug_ct1 >= 100 )
	{
		cout << " falsealarmcond =" << false_alarm_cond
		     << " leadObjId ="      << leadObjId
		     << " decel_lead ="     << dl_avg
			 << endl;
		debug_ct1 = 0;
	}

	if ( warning > 0 )
	{
		debug_ct2 += 1;
		if ( debug_ct2 >= 20 )
		{
			cout << " headway="           << headway 
				 << " warning="           << warning 
				 << " falsealarmcond="    << false_alarm_cond 
				 << " falsealarmtrigger=" << false_alarm_trigger 
				 << " falsealarmreset="   << false_alarm_reset 
				 << endl;
		    debug_ct2 = 0;
		}
	}
#endif

	return warning;
}
