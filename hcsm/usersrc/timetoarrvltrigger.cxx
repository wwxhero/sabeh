/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: timetoarrvltrigger.cxx,v 1.55 2013/11/27 00:00:40 iowa\dheitbri Exp $
 *
 * Author:       Jillian Vogel
 * Date:         October, 1999
 *
 * Description: Provides code for the 3 main functions of the 
 * 	TimeToArrvlTrigger HCSM.
 *
 ****************************************************************************/

#include <math.h>
#include "genhcsm.h"
#include "hcsmcollection.h"
#include "hcsminterface.h"
#include "expevalTTA.h"
#ifdef _WIN32
#include <strstream>
#elif _PowerMAXOS
#include <strstream>
#define ostringstream ostrstream
#elif __sgi
#include <strstream.h>
#endif
CCved* g_pCved2 = NULL;




extern void 
StringFlip( const volatile char from[], char to[], int size );
extern void
StrncpyFlip( char* pDst, const char* cpSrc, int size );

static void
PrintInvalidArgs( 
			int argC, 
			const CExprParser::CStrNum args[], 
			const char* pFuncName 
			)
{
	string funcName = pFuncName;
	gout << funcName << " has invalid arguments...throwing exception" << endl;
	gout << "  numArguments = " << argC << endl;

	int i;
	for( i = 0; i < argC; i++ )
	{
		gout << "  " << i << " = ";
		if( args[i].m_IsNum )
		{
			gout << args[i].m_Num << "  [NUMBER]";
		}
		else
		{
			gout << args[i].m_Str << "  [STRING]";
		}
		gout << endl;
	}

	throw InvalidArgs;
}

double
CTimeToArrvlTrigger::MySin( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if( invalidArgs )  throw InvalidArgs;
	return sin( args[0].m_Num );
}


double
CTimeToArrvlTrigger::MyCos( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if( invalidArgs )  throw InvalidArgs;
	return cos( args[0].m_Num );
}
double
CTimeToArrvlTrigger::MyAbs( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if( invalidArgs )  throw InvalidArgs;
	return abs( args[0].m_Num );
}

double
CTimeToArrvlTrigger::CellEquals( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 3 || args[0].m_IsNum || !args[1].m_IsNum || !args[2].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "CellEquals" );

	string cellName = args[0].m_Str;
	int index = (int) args[1].m_Num;
	double doubleVal = args[2].m_Num;

	if( cellName == "LogStreams" )
	{
		if( index < 0 || index >= cNUM_LOG_STREAMS *2 )  throw InvalidArgs;
		else if (index < 0 || index < cNUM_LOG_STREAMS){
			return fabs( CHcsmCollection::m_sLogStreams[index] - doubleVal ) < cNEAR_ZERO;
		}else{
			return fabs( CHcsmCollection::m_sLogStreamsExt[index -cNUM_LOG_STREAMS] - doubleVal ) < cNEAR_ZERO;
		}
	}
	else if( cellName == "AccelPedalPos" )
	{
		return fabs( CHcsmCollection::m_sAccelPedalPos - doubleVal ) < cNEAR_ZERO;
	}
	else if( cellName == "CruiseControl" )
	{
		return fabs( CHcsmCollection::m_sCruiseControl - doubleVal ) < cNEAR_ZERO;
	}
	else if ( cellName == "CruiseControlUF"){
		return fabs( CHcsmCollection::m_sCruiseControlIncoming - doubleVal ) < cNEAR_ZERO;
	}
	else if( cellName == "TurnSignal" )
	{
		return fabs( CHcsmCollection::m_sCisTurnSignal - doubleVal ) < cNEAR_ZERO;
	}

	return 0.0;
}

double
CTimeToArrvlTrigger::ReadVar( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = (argC != 1) || (args[0].m_IsNum);
	if( invalidArgs )  PrintInvalidArgs( argC, args, "ReadVar" );

	string varName = args[0].m_Str;
	if( !CHcsmCollection::ExprVariableExists( varName ) )
	{
		return 0.0;
	}
	
	return CHcsmCollection::GetExprVariable( varName );
}


double
CTimeToArrvlTrigger::ReadCell( int argC, const CExprParser::CStrNum args[] )
{
 	
	bool invalidArgs = argC != 2 || args[0].m_IsNum || !args[1].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "ReadCell" );

	string cellName = args[0].m_Str;
	int index = (int) args[1].m_Num;

	if( cellName == "PlacePhoneCall" )
	{
		return CHcsmCollection::m_sSCC_PlacePhoneCall;
	}
	else if( cellName == "PhoneCallAge" )
	{
		return CHcsmCollection::m_sPlacePhoneCallAge;
	}
	else if( cellName == "LogStreams" )
	{
		if( index < 0 || index >= cNUM_LOG_STREAMS *2 )  
			throw InvalidArgs;
		else if (index < 0 || index < cNUM_LOG_STREAMS){
			return CHcsmCollection::m_sLogStreams[index];
		}else{
			return CHcsmCollection::m_sLogStreamsExt[index - cNUM_LOG_STREAMS];
		}
	}
	else if( cellName == "AccelPedalPos" )
	{
		return CHcsmCollection::m_sAccelPedalPos;
	}
	else if( cellName == "BrakePedalForce" )
	{
		return CHcsmCollection::m_sBrakePedalForce;
	}
	else if( cellName == "SteeringWheelAngle" )
	{
		return CHcsmCollection::m_sSteeringWheelAngle;
	}
	else if( cellName == "CruiseControl" )
	{
		return CHcsmCollection::m_sCruiseControl;
	}
	else if ( cellName == "CruiseControlUF"){
		return CHcsmCollection::m_sCruiseControlIncoming;
	}
	else if( cellName == "TurnSignal" )
	{
		return CHcsmCollection::m_sCisTurnSignal;
	}
	else if( cellName == "OvVel" )
	{
		double ownVehVel = 0.0;
		g_pCved2->GetOwnVehicleVel( ownVehVel );
		ownVehVel *= cMS_TO_MPH;
#ifdef TTA_DIST_FOR_ODSS	//quick hack to grab Distance to Intersection for a specific scenario	
		CHcsmCollection::m_sDistanceToInt = m_currTimeToArrival;//*(ownVehVel * 1.46667f);
#endif
		return ownVehVel;
	}
	else if( cellName == "OvVelLocal" )
	{
		double ownVehVel = 0.0;
		g_pCved2->GetOwnVehicleVel( ownVehVel );
		return ownVehVel * cMS_TO_MPH;
	}
	else if( cellName == "OvLaneDev" )
	{
		double laneDev = 0.0;
		if( CHcsmCollection::m_sLaneDevInfo[0] != 0 )
		{
			laneDev = CHcsmCollection::m_sLaneDevInfo[1];
		}
		return laneDev;
	}
	else if( cellName == "OvHeadwayToLeadVeh" )
	{
		return CHcsmCollection::m_sFollowInfo[cFOLLOW_INFO_HEADWAY_IDX];
	}
	else if( cellName == "OvTtcToLeadVeh" )
	{
		return CHcsmCollection::m_sFollowInfo[cFOLLOW_INFO_TTC_IDX];
	}
	else if( cellName == "Horn" )
	{
		return CHcsmCollection::m_sHorn;
	}
	else if (cellName == "HornFiltered"){
		return CHcsmCollection::m_sHornFiltered;	
	}
	else if( cellName == "RECAS_Button" )
	{
		return CHcsmCollection::m_sRecasButton;
	}
	else if( cellName == "Auxiliary_Buttons" )
	{
		return CHcsmCollection::m_sAuxiliaryButtons[index];
	}
	else if ( cellName == "DynObj_Vel" )
	{
		return CHcsmCollection::m_sDynObjData.vel[index];
	}
	else if ( cellName == "My_Lifetime" )
	{
		double deltaT = GetFrame() - m_activationTime;

		double ownVehVel = 0.0;
		g_pCved2->GetOwnVehicleVel( ownVehVel );
		ownVehVel *= cMS_TO_MPH;

		gout<<"Speed "<<ownVehVel<<" time to red "<<(4.0 - (deltaT * GetTimeStepDuration()))<<" My TTA "<<m_currTimeToArrival<<endl;
		return deltaT * GetTimeStepDuration();
	}
	else if (cellName == "TimeToArrival"){
		return m_currTimeToArrival;
	}
	else if (cellName == "WarningLights"){
		return CHcsmCollection::m_sWarningLights;
	}
	else if (cellName == "ACC_Warning"){
		return CHcsmCollection::m_sACC_Warning;
	}
	else if (cellName == "Cruise_State"){
		return CHcsmCollection::m_sCruise_State;
	}
	else if (cellName == "Cruise_Speed"){
		return CHcsmCollection::m_sCruise_SetSpeed;
	}
	else if (cellName == "ACC_Gap"){
		return CHcsmCollection::m_sACC_Gap;
	}
	else if (cellName == "Sensor_Info"){
		return CHcsmCollection::m_sSensorInfo[index];
	}
	else if (cellName == "OnPath"){
		return CHcsmCollection::m_sIsOnPath;
	}
	else if (CHcsmCollection::ReadCellNumeric != NULL){
		float result = 0;
		if (CHcsmCollection::ReadCellNumeric(cellName,index -1,result)){
			return result;
		}
		else return 0.0;
	}
/*	else if (cellName == "OwnVehAccel"){
		int ownVehObjId;
		g_pCved2->GetObj( "ExternalDriver", ownVehObjId );
		const CDynObj* pOwnVehObj = g_pCved2->BindObjIdToClass( ownVehObjId );
		return (pOwnVehObj->GetVel()  - pOwnVehObj->GetVelImm()) / CHcsmCollection::GetTimeStepDuration();	
	}*/
	else
	{
		gout << "ExpressionTrigger: unknown cell name '" << cellName;
		gout << "'" << endl;
	}
	return 0.0;
}


double
CTimeToArrvlTrigger::GetObjVel( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetObjVel" );

	string objName = args[0].m_Str;

	int i;
	for( i = 0; i < CHcsmCollection::m_sDynObjDataSize; i++ )
	{
		char temp[cMAX_DYN_OBJ_NAME_SIZE] = { 0 };
		StrncpyFlip(
			temp,
			&CHcsmCollection::m_sDynObjData.name[i*cMAX_DYN_OBJ_NAME_SIZE],
			cMAX_DYN_OBJ_NAME_SIZE 
			);
		if( !strcmp( temp, objName.c_str() ) )
		{
			return CHcsmCollection::m_sDynObjData.vel[i];
		}
	}

	return 0.0;
}
//////////////////////////////////////////////////////////////////////////////
///\brief 
///		Returns the Euclidian Distance to the given object
///
///\remark  
///		This function Does not calcuate the road distance just the Euclidian
///		Distance to the ownship to the given vehicle
/// 
///\par   argC - The number of arguments.
///\par   args - The arguments.
///          arg #1: string - The object's name in the scenario.
///
///\return  The distance to the object if the object is found
///\return	 If the object is not found the Distance is set to 900000000000000000000.0
///
//////////////////////////////////////////////////////////////////////////////
double
CTimeToArrvlTrigger::GetObjDistPow( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetObjDistPow2" );

	string objName = args[0].m_Str;

	int i;
	for( i = 0; i < CHcsmCollection::m_sDynObjDataSize; i++ )
	{
		char temp[cMAX_DYN_OBJ_NAME_SIZE] = { 0 };
		StrncpyFlip(
			temp,
			&CHcsmCollection::m_sDynObjData.name[i*cMAX_DYN_OBJ_NAME_SIZE],
			cMAX_DYN_OBJ_NAME_SIZE 
			);
		if( !strcmp( temp, objName.c_str() ) )
		{
			CPoint3D ovPos;
			g_pCved2->GetOwnVehiclePos( ovPos );
			float deltaX = ovPos.m_x - CHcsmCollection::m_sDynObjData.pos[i*3];
			float deltaY = ovPos.m_y - CHcsmCollection::m_sDynObjData.pos[i*3+1];
			float deltaZ = ovPos.m_z - CHcsmCollection::m_sDynObjData.pos[i*3+2];
			return deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ;
		}
	}
	return 900000000000000000000.0;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns an object's TTC to the OwnVehicle given the 
//  object's name.
//
// Remarks:  Finds the first object in the DynObjData structures with the 
//  given name.  Uses the CVED id this object to query it's position and
//  velocity from CVED.  This function assumes that the OwnVehicle is
//  in front of the given object.
//
// Arguments:
//   argC - The number of arguments.
//   args - The arguments.
//          arg #1: string - The object's name in the scenario.
//
// Returns:  The TTC to the OwnVehicle for the given object.
//
//////////////////////////////////////////////////////////////////////////////
double 
CTimeToArrvlTrigger::GetObjTtcToOv( int argC, const CExprParser::CStrNum args[] )
{
	// make sure CVED is valid
	if( !g_pCved2 )  return 0;

	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetObjTtcToOv" );

	//
	// Given the object's name, find its CVED id.
	//
	string objName = args[0].m_Str;
	int objId;
	bool foundObj = g_pCved2->GetObj( objName, objId );
	if( !foundObj )
	{
		gout << "GetObjTtcToOv: unable to find object named '";
		gout << objName << "'" << endl;

		return 0.0;
	}

	//
	// Calculate the distance from the object to the OV.
	//
	CPoint3D objPos = g_pCved2->GetObjPos( objId );
	CPoint3D ovPos;
	bool gotOvInfo = g_pCved2->GetOwnVehiclePos( ovPos );
	if( !gotOvInfo )  return 0.0;
	double dist = fabs( (ovPos - objPos).Length() );

	//
	// Calculate the difference in velocities between the object and
	// the OV.
	//

	double objVel = g_pCved2->GetObjVel( objId );
	
	double ovVel;
	gotOvInfo = g_pCved2->GetOwnVehicleVel( ovVel );
	if( !gotOvInfo )  return 0.0;
	double velDiff = ( objVel - ovVel ) * cMETER_TO_FEET;  // convert to ft/s

	double ttc = dist / velDiff;
//	gout << objName << ":  obj ttc to ov = " << ttc << endl;
	return ttc;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the current candiates distance to the target point
//
// Remarks:  
//			m_currCandidate must be set first
// Arguments:
//   argC - The number of arguments.
//   args - The arguments.
//          arg #1: string - The object's name in the scenario.
//
// Returns:  The TTC to the OwnVehicle for the given object.
//
//////////////////////////////////////////////////////////////////////////////
double 
CTimeToArrvlTrigger::GetDistToTarg( int argC, const CExprParser::CStrNum args[] )
{
	const CObj* pObj = cved->BindObjIdToClass( m_currCandidate.m_cvedId );
	if( !pObj || !pObj->IsValid() ) 
	{
		return 1000000000.0;
	}

	CPoint3D objPos = pObj->GetPos();
	CVector3D objTan = pObj->GetTan();
	CVector3D objLat = pObj->GetLat();
	double objX = pObj->GetXSize();
	double objY = pObj->GetYSize();
	double objVel = pObj->GetVel();  // m/s

	CPoint3D  corners[4];
	corners[0] = objPos + objTan + objLat;
	corners[1] = objPos + objTan - objLat;
	corners[2] = objPos - objTan - objLat;
	corners[3] = objPos - objTan + objLat;

	double distancesSq[4];
	int closest = 0;
	int i;
	for( i = 0; i < 4; i++ )
	{
		double xDiff = corners[i].m_x - m_firePosition.m_x;
		double yDiff = corners[i].m_y - m_firePosition.m_y;
		distancesSq[i] = ( xDiff * xDiff ) + ( yDiff * yDiff );

		if( distancesSq[i] < distancesSq[closest] )  closest = i;
	}

	//
	// Protect from divide by zero.
	//
	double timeToArrival;
	bool objMoving = objVel > cNEAR_ZERO;

	return sqrt( distancesSq[closest] );
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the OwnVehicle's TTC to another object given the 
//  object's name.
//
// Remarks:  Finds the first object in the DynObjData structures with the 
//  given name.  Uses the CVED id this object to query it's position and
//  velocity from CVED.  This function assumes that the OwnVehicle is
//  behind the given object.
//
// Arguments:
//   argC - The number of arguments.
//   args - The arguments.
//          arg #1: string - The object's name in the scenario.
//
// Returns:  The OwnVehicle's TTC to the given object.
//
//////////////////////////////////////////////////////////////////////////////
double 
CTimeToArrvlTrigger::GetOvTtcToObj( int argC, const CExprParser::CStrNum args[] )
{
	// make sure CVED is valid
	if( !g_pCved2 )  return 0;

	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetOvTtcToObj" );

	//
	// Given the object's name, find its CVED id.
	//
	string objName = args[0].m_Str;
	int objId;
	bool foundObj = g_pCved2->GetObj( objName, objId );
	if( !foundObj )
	{
		gout << "GetObjTtcToOv: unable to find object named '";
		gout << objName << "'" << endl;

		return 0.0;
	}

	//
	// Calculate the distance from the object to the OV.
	//
	CPoint3D objPos = g_pCved2->GetObjPos( objId );
	CPoint3D ovPos;
	bool gotOvInfo = g_pCved2->GetOwnVehiclePos( ovPos );
	if( !gotOvInfo )  return 0.0;
	double dist = fabs( (ovPos - objPos).Length() );

	//
	// Calculate the difference in velocities between the object and
	// the OV.
	//
	double objVel = g_pCved2->GetObjVel( objId );
	double ovVel;
	gotOvInfo = g_pCved2->GetOwnVehicleVel( ovVel );
	if( !gotOvInfo )  return 0.0;
	double velDiff = ( ovVel - objVel ) * cMETER_TO_FEET;  // convert to ft/s

	double ttc = dist / velDiff;
//	gout << objName << ":  ov ttc to obj = " << ttc << endl;
	return ttc;
}

void 
CTimeToArrvlTrigger::UserCreation( const CTriggerParseBlock* cpBlock )
{
	PrintCreationMessage();
    m_priorityLevel = cpBlock->GetPriority();
	string prefix = MessagePrefix();
	strstream message;

	try 
	{
		// Initialize CObjectInitCond object
		m_initCondition.SetObjectInitCond(
								*cved, 
								GetFrame(), 
								GetTimeStepDuration(),  
								cpBlock->GetCrRad(),  
								cpBlock->GetActvDel(), 
								cpBlock->GetLifetime()
								);

		// Initialize CTriggerFireCond object
		m_fireCondition.InitFireCondition(
								GetTimeStepDuration(),
								cpBlock->GetFireDelFrames(),
								cpBlock->GetDebounce(),
								cpBlock->GetOneShot()
								);

		// Initialize other local variables
		m_firePosition = cpBlock->GetFirePosition();
		m_time = cpBlock->GetTime();
	    m_expression = cpBlock->GetExpression();

	
		//m_expEval.
		m_expEval.SetParent(this);
		m_expEval.m_functions["sin"]			= &CTimeToArrvlTrigger::MySin;
		m_expEval.m_functions["cos"]			= &CTimeToArrvlTrigger::MyCos;
		m_expEval.m_functions["ReadCell"]		= &CTimeToArrvlTrigger::ReadCell;
		m_expEval.m_functions["CellEquals"]	    = &CTimeToArrvlTrigger::CellEquals;
		m_expEval.m_functions["ReadVar"]		= &CTimeToArrvlTrigger::ReadVar;
		m_expEval.m_functions["GetObjVel"]		= &CTimeToArrvlTrigger::GetObjVel;
		m_expEval.m_functions["GetObjTtcToOv"]  = &CTimeToArrvlTrigger::GetObjTtcToOv;
		m_expEval.m_functions["GetOvTtcToObj"]  = &CTimeToArrvlTrigger::GetOvTtcToObj;
		m_expEval.m_functions["GetObjDistPow2"] = &CTimeToArrvlTrigger::GetObjDistPow;
		m_expEval.m_functions["Abs"]			= &CTimeToArrvlTrigger::MyAbs;
		m_expEval.m_functions["GetDistToTarg"]	= &CTimeToArrvlTrigger::GetDistToTarg;
		g_pCved2 = cved;

	
		if( m_expression.size() > 0 ){
			if( !m_expEval.Parse( m_expression.c_str() ) )
			{
				gout << MessagePrefix() << "failed to parse expression [SUICIDE]" << endl;
				gout << m_expression << endl;
				m_pTrigger = 0;
				Suicide();
				return;
			}
		}


		m_prevPosition = cpBlock->GetPosition();
		m_prevState = eWAIT;
		m_pTrigger = 0;
		m_path = CPath( *cved );
		m_path.SetString( cpBlock->GetPath() );

		// Initialize the action vector
		GetActions( cpBlock, m_pRootCollection, m_pActionVector );
		m_sequentialActions = cpBlock->GetSequentialActions();
		m_pCurrentAction = m_pActionVector.end();
		m_delayFrameCount = 0;
		m_waitingForAction = false;
        m_secondOrder = cpBlock->GetUseSecondOrder();
		m_boundSpeed = cpBlock->GetUpperBoundOnSpeed();
		m_logTTA = cpBlock->GetLogTTA();
		m_logTTANum = cpBlock->GetTTALogNum();
		InitCandidates( cpBlock );

		//
		// Make an entry into the activity log for HCSM creation.
		//
		m_pRootCollection->SetHcsmCreateLog( 
					this, 
					m_typeId, 
					m_name, 
					cpBlock->GetPosition() 
					);
	}
	catch( CSnoBlock::TCountError e )
	{
		cerr << MessagePrefix() << "caught TCountError exception...[SUICIDE]";
		cerr << endl;

		Suicide();
		return;
	}
	catch( ... )
	{
		cerr << MessagePrefix() << "caught unknown exception...[SUICIDE]";
		cerr << endl;

		Suicide();
		return;
	}
}

void 
CTimeToArrvlTrigger::UserActivity( const CTriggerParseBlock* cpBlock )
{
	// If the MakeTraffic button is pressed, then fire the action
	if( GetButtonFireTrigger() )
	{
		ExecuteConcurrentActions();
		return;
	}

	// Get current gmtrcpstn and position
	int actualTime = GetFrame();

	CPoint3D actualPosition;
	if( m_prevState == eUNDER_CONTROL )
	{
		actualPosition = m_pTrigger->GetPosImm();
	}
	else
	{
		actualPosition = m_prevPosition;
	}

	// Get the current state from the InitCondition object
	EInitCond state = m_initCondition.Execute( actualPosition, actualTime );
			
	// For use in eACTIVATE state
	cvTObjAttr triggerAttr = { 0 };

	switch( state )
	{
		//////////////////////////////////////////////////////////////////////
		// eWAIT
		//
		case eWAIT:

			// Don't do anything yet.
			break;	// case eWAIT


		//////////////////////////////////////////////////////////////////////
		// eACTIVATE
		//
		case eACTIVATE:
			{
				// Initialize the CVED representation of the Trigger.
				// Because we need to be able to fire it at any time 
				// using the FireTrigger button, it should exist in 
				// CVED for the duration of the HCSM.
				triggerAttr.hcsmId = m_pRootCollection->GetHcsmId(this);
				triggerAttr.xSize = 2.0;
				triggerAttr.ySize = 2.0;

				m_pTrigger = dynamic_cast<CCoordinatorObjectObj *> 
								(
									cved->CreateDynObj(
												cpBlock->GetName(),
												eCV_COORDINATOR, 
												m_typeId,
												triggerAttr,
												&actualPosition,
												0, 
												0
												)
			 						);
				if( !m_pTrigger )
				{
					cerr << MessagePrefix() << "unable to create self in CVED";
					cerr << "...[SUICIDE]" << endl;

					Suicide();
					return;
				}
				
				m_activationTime = actualTime;
				// Set the actions' parents to be this trigger's HCSM id
				for (CAction::TActionIterator i = m_pActionVector.begin(); i != m_pActionVector.end(); i++) {
					(*i)->SetTriggerId( m_pTrigger->GetHcsmId() );
				}

				// Evaluate the predicate if the Fire Condition object 
				// says to.
				bool evalPredicate = m_fireCondition.GetState() == eEVAL_PRED;
				if( evalPredicate )
				{
					bool result = Evaluate();
					m_fireCondition.SetPredicate( result );
				}

				// Fire the action if the Fire Condition object says so.
				bool fire = m_fireCondition.Execute( actualTime ) == eFIRE;
				if( fire )
				{
					CHcsmDebugItem item;
					item.SetTriggerFire(this->m_pTrigger->GetId());
					m_pRootCollection->LogDebugItem(item);

					if( m_sequentialActions )
					{
						InitializeSequentialActions();
					}
					else
					{
						ExecuteConcurrentActions();
					}
				}

				if( m_sequentialActions )
				{
					ExecuteSequentialActions();
				}
			}

			break;	// case eACTIVATE

		//////////////////////////////////////////////////////////////////////
		// eUNDER_CONTROL
		//
		case eUNDER_CONTROL:
			{
				// Evaluate the predicate if the Fire Condition object says
				// to.
				bool evalPredicate = m_fireCondition.GetState() == eEVAL_PRED;
				if( evalPredicate )
				{
					bool result = Evaluate();
					m_fireCondition.SetPredicate( result );
				}

				bool fire = m_fireCondition.Execute( actualTime ) == eFIRE;
				if( fire )
				{
					CHcsmDebugItem item;
					item.SetTriggerFire(this->m_pTrigger->GetId());
					m_pRootCollection->LogDebugItem(item);

					if( m_sequentialActions )
					{
						InitializeSequentialActions();
					}
					else
					{
						ExecuteConcurrentActions();
					}
				}

				if( m_sequentialActions )
				{
					ExecuteSequentialActions();
				}
			}

			break;	// case eUNDER_CONTROL


		//////////////////////////////////////////////////////////////////////
		// eDELETE
		//
		case eDELETE:
			DeleteHcsm( this );

			break;	// case eDELETE


		//////////////////////////////////////////////////////////////////////
		// eEXIT
		//
		case eEXIT:

			break;	// case eEXIT
	}

	// Set m_prev* local variables for next time step.
	m_prevState = state;
	m_prevPosition = actualPosition;
}

void 
CTimeToArrvlTrigger::UserDeletion( const CTriggerParseBlock* cpBlock )
{
	PrintDeletionMessage();

	if( m_pTrigger != 0 )  cved->DeleteDynObj( m_pTrigger );

	//
	// Add an entry to the activity log for HCSM deletion.
	//
	m_pRootCollection->SetHcsmDeleteLog( 
				this, 
				cpBlock->GetPosition() 
				);
}

void
CTimeToArrvlTrigger::InitCandidates( const CTriggerParseBlock* cpBlock )
{
	// Initialize the candidate sets associated with this trigger
	vector<string> byNameSet = cpBlock->GetByNameSet();
	if( !byNameSet.empty() )  m_candidateSet.AddNames( byNameSet );

	vector<string> byTypeSet = cpBlock->GetByTypeSet();
	if( !byTypeSet.empty() )  m_candidateSet.AddTypes( byTypeSet );
}

bool
CTimeToArrvlTrigger::Evaluate( void )
{

	static ofstream fout;
	static bool firstTime = true;
	CCandidate tmpCand;
	if (firstTime){
		fout.open("TTAtest.txt",ios_base::out);
		firstTime = false;
		fout<<"Root1	Root2	Accel	Speed	Distance	Time	Target"<<endl;
	}

	// Assemble the set of candidates
	set<CCandidate> candidateSet;
	m_candidateSet.GetCandidates( *cved, candidateSet, *m_pRootCollection );

	// 
	// Find the objects that lie on the pad and are within the given 
	// time-to-arrival of the fire position.
	//
	vector<int> objsOnPath;
	m_path.GetObjectsOnPath( objsOnPath );

	set<CCandidate> padObjsSet;
	vector<int>::const_iterator vecItr;
	for( vecItr = objsOnPath.begin(); vecItr != objsOnPath.end(); vecItr++ )
	{
		tmpCand.m_cvedId = *vecItr;
		tmpCand.m_hcsmId = cved->GetObjHcsmId( tmpCand.m_cvedId );
		padObjsSet.insert( tmpCand );
	}

	// 
	// If one or more of the candidates is within Radius of the 
	// FirePosition, the Trigger should fire.  Place the HCSM IDs 
	// of all the candidates that meet that criteria into the 
	// instigator set.
	//
	m_instigatorSet.clear();

	// For each candidate object
	set<CCandidate>::const_iterator cItr;
	float lowestTTA = 100000.0f;
	for( cItr = candidateSet.begin(); cItr != candidateSet.end(); cItr++ )
	{
		// If the candidate is on the pad, then check its estimated 
		// time to arrival.
		bool candidateOnPad = padObjsSet.find( *cItr ) != padObjsSet.end();
		if( candidateOnPad )
		{
			m_currTimeToArrival = GetTimeToArrival( *cItr, m_firePosition );
			//if we have an expression, re-calc m_time
			if (m_expression.size() > 0){
				m_currCandidate = *cItr; 
				m_time = m_expEval.Evaluate();
			}
			bool withinTtc = m_currTimeToArrival < m_time;
			if( withinTtc )
			{
				// Insert object into the instigator set.
				m_instigatorSet.insert( *cItr );
			}
			if (lowestTTA > m_currTimeToArrival){
				lowestTTA  =m_currTimeToArrival;
				
			}

		}
	}
	if (m_logTTA){
		LogData(m_logTTANum,lowestTTA);
	}

	//
	// If there's object(s) in the instigator set, then something 
	// caused the trigger to fire, so return true.  Otherwise, 
	// return false.
	//
	return !( m_instigatorSet.empty() );
}


double
CTimeToArrvlTrigger::GetTimeToArrival(
			const CCandidate& cCandObj, 
			const CPoint3D& cFirePos
			) 
{
	const CObj* pObj = cved->BindObjIdToClass( cCandObj.m_cvedId );
	if( !pObj->IsValid() ) 
	{
		return 1000.0;
	}

	CPoint3D objPos = pObj->GetPos();
	CVector3D objTan = pObj->GetTan();
	CVector3D objLat = pObj->GetLat();
	double objX = pObj->GetXSize();
	double objY = pObj->GetYSize();
	double objVel = pObj->GetVel();  // m/s

	CPoint3D  corners[4];
	corners[0] = objPos + objTan + objLat;
	corners[1] = objPos + objTan - objLat;
	corners[2] = objPos - objTan - objLat;
	corners[3] = objPos - objTan + objLat;

	double distancesSq[4];
	int closest = 0;
	int i;
	for( i = 0; i < 4; i++ )
	{
		double xDiff = corners[i].m_x - cFirePos.m_x;
		double yDiff = corners[i].m_y - cFirePos.m_y;
		distancesSq[i] = ( xDiff * xDiff ) + ( yDiff * yDiff );

		if( distancesSq[i] < distancesSq[closest] )  closest = i;
	}

	//
	// Protect from divide by zero.
	//
	double timeToArrival;
	bool objMoving = objVel > cNEAR_ZERO;
	if( objMoving )
	{
		double closestDist = sqrt( distancesSq[closest] );
		if (m_secondOrder){ //aka use accelaration

				double ownvel = objVel * cMETER_TO_FEET; 
				const CVehicleObj* cpVehObj = 
						dynamic_cast<const CVehicleObj *>( pObj );
				double accel = cpVehObj->GetAccel()* cMETER_TO_FEET;

				const CExternalDriverObj* cpExtDrObj = 
						dynamic_cast<const CExternalDriverObj *>( pObj );
				if (cpExtDrObj){
					accel = cpExtDrObj->GetAccel()* cMETER_TO_FEET;
				}

				//ownvel += accel*GetTimeStepDuration();
				//float accel = deltaSpeed/GetTimeStepDuration();
				double timetotarget;

				if (fabs(accel) > 0.1){ //once accel falls out our error function is just to large to deal with
					//we need to first figure out if our final velocity is going to be above our upper bound
					double vFinalPow2 = ownvel * ownvel + 2 * accel * closestDist;
					if (vFinalPow2 > 0 && (m_boundSpeed == 0 || vFinalPow2 < (m_boundSpeed* m_boundSpeed ) ) ){
						//our current accel is not going to bring us up over our upper bound
						//Solve quadratic equation for motion
						//a = -1/2 (accel in fps^2)
						//b = velocity in fps
					    //c = dist to target in feet
						double quadratic = -0.5 * (ownvel + sqrt(vFinalPow2));
						float root1 = quadratic/( 0.5 * accel);
						float root2 =(0 - closestDist)/quadratic;
						//vFinalPow2 = sqrt(vFinalPow2);
						//float root1 = (vFinalPow2 - ownvel)/accel;
						//float root2 = (-ownvel - vFinalPow2)/accel;

						if (root1 < root2 && root1 > 0)
							timeToArrival = root1;
						else if (root1 < 0 && root2 < 0)
							timeToArrival = 100000000.0f; //we are not going to make it
						else
							timeToArrival = root2;
					}
					else if (vFinalPow2 < 0){
						timeToArrival = 100000000.0f; //we are not going to make it
					}
					else if (ownvel < m_boundSpeed){
						//since our current rate of accelaration is going to be higher than our upper bound
						//in terms of speed we must calculate the TTA in two parts, first how long it takes to
						//reach the upper bound, plus how long at the upper speed will it take to reach the target
						//after we have reached speed
						double TimeToAccel = (m_boundSpeed * cMPH_TO_FTPS  - ownvel)/(accel);
						double TravelDistToAccel = .5 * accel * TimeToAccel * TimeToAccel + ownvel * TimeToAccel;
						double TimeAtConstVel = (closestDist - TravelDistToAccel) / (m_boundSpeed * cMPH_TO_FTPS);
						timeToArrival = TimeToAccel + TimeAtConstVel;
					}else{
						timeToArrival = closestDist/ownvel;
					}
				}else{
					timeToArrival = closestDist/ownvel;
					//check for a upper bound on the speed
				}
		}else{
			timeToArrival = ( closestDist * cFEET_TO_METER ) / objVel;
		}
//		gout << closestDist << " ft    " << timeToArrival << " secs" << endl;
	}
	else 
	{
		timeToArrival = 1000.0;  // a large value
	}
	if (fabs(timeToArrival) > 2 && timeToArrival < 0)
		return 100000000.0f;

	return timeToArrival;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes the actions concurrently.  That is, all actions
//  associated with this trigger are executed at the same time.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CTimeToArrvlTrigger::ExecuteConcurrentActions()
{
	//
	// Exeucte actions concurrently.
	//
	CAction::TActionIterator aI;
	for( aI = m_pActionVector.begin(); aI != m_pActionVector.end(); aI++ )
	{
		(*aI)->Execute( &m_instigatorSet );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the sequential actions if needed.  
//
// Remarks:  Sets the m_pCurrentAction member variable to point to the
//  first action on the action vector.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CTimeToArrvlTrigger::InitializeSequentialActions()
{
	//
	// Execute actions sequentially.
	//
	bool needToInitialize = m_pCurrentAction == m_pActionVector.end();
	if( needToInitialize )
	{
		m_pCurrentAction = m_pActionVector.begin();
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes the actions sequentially.  That is, each action
//  is executed in sequence from the first action to the last.  
//
// Remarks:  In order for the next action to be executed, the first action
//  has to complete itself.  This is most applicable when setting dials on
//  an HCSM.  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CTimeToArrvlTrigger::ExecuteSequentialActions()
{
	
	bool actionsRemaining = m_pCurrentAction != m_pActionVector.end();
	if( actionsRemaining )
	{
		if( m_waitingForAction )
		{
			bool isFinished = (*m_pCurrentAction)->IsFinished();
			if( isFinished )
			{
				m_waitingForAction = false;
				m_pCurrentAction++;
				actionsRemaining = m_pCurrentAction != m_pActionVector.end();
			}
		}
		if (actionsRemaining && !m_waitingForAction)
		{
			//
			// Wait to make sure that the delay counter has expired before
			// executing the current action.
			//
			bool readyToExecute = m_delayFrameCount <= 0;
			if( readyToExecute )
			{
				(*m_pCurrentAction)->Execute(&m_instigatorSet);
				m_waitingForAction = true;
				m_delayFrameCount = 
					static_cast<int>((*m_pCurrentAction)->GetDelay() / GetTimeStepDuration());
			}
			else
			{
				m_delayFrameCount--;
			}
		}
	}
}
