/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: expressiontrigger.cxx,v 1.53 2014/09/15 23:36:50 IOWA\vhorosewski Exp $
 *
 * Author:  Jillian Vogel
 * Date:    October, 1999
 *
 * Description: Provides code for the 3 main functions of the 
 *   ExpressionTrigger HCSM.
 *
 ****************************************************************************/

#include "expeval.h"
#include "genhcsm.h"
#include "hcsmcollection.h"
#include "support.h"
#include <strstream>
#undef DEBUG

CCved* g_pCved = NULL;

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

static double
MySin( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if( invalidArgs )  throw InvalidArgs;
	return sin( args[0].m_Num );
}


static double
MyCos( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if( invalidArgs )  throw InvalidArgs;
	return cos( args[0].m_Num );
}


static double
CellEquals( int argC, const CExprParser::CStrNum args[] )
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
			return fabs( CHcsmCollection::m_sLogStreamsExt[index - cNUM_LOG_STREAMS] - doubleVal ) < cNEAR_ZERO;
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

static double
SquareRoot( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if( invalidArgs )  throw InvalidArgs;
	return sqrt( args[0].m_Num );
}

static double
MyAbs( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if( invalidArgs )  throw InvalidArgs;
	return abs( args[0].m_Num );
}
static double
ReadVar( int argC, const CExprParser::CStrNum args[] )
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

static double
ReadCell( int argC, const CExprParser::CStrNum args[] )
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
		if( index < 0 || index >= cNUM_LOG_STREAMS *2 )  throw InvalidArgs;
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
		return CHcsmCollection::m_sSpeedometerBackdrive;
	}
	else if( cellName == "Hour" )
	{
		return CHcsmCollection::m_sHour;
	}
	else if( cellName == "Minute" )
	{
		return CHcsmCollection::m_sMinute;
	}
	else if( cellName == "OvVelLocal" )
	{
		double ownVehVel = 0.0;
		g_pCved->GetOwnVehicleVel( ownVehVel );
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
	else if ( cellName == "FcwInfo" )
	{
		if( index < 0 || index >= 4)  throw InvalidArgs;
		return CHcsmCollection::m_sFcwInfo[index];
	}
	else if (cellName == "LdwStatus"){
		return CHcsmCollection::m_sLdwStatus;
	}
	else if (cellName == "BackUpDistance"){
		return CHcsmCollection::m_sBackUpDistance;
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
		g_pCved->GetObj( "ExternalDriver", ownVehObjId );
		const CDynObj* pOwnVehObj = g_pCved->BindObjIdToClass( ownVehObjId );
		return (pOwnVehObj->GetVel()  - pOwnVehObj->GetVelImm()) / CHcsmCollection::GetTimeStepDuration();	
	}*/
	else
	{
		//gout << "ExpressionTrigger: unknown cell name '" << cellName;
		//gout << "'" << endl;
	}

	return 0.0;
}


static double
GetObjVel( int argC, const CExprParser::CStrNum args[] )
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
			double vel = CHcsmCollection::m_sDynObjData.vel[i];
			return CHcsmCollection::m_sDynObjData.vel[i];
		}
	}

	return 0.0;
}
static double
GetObjAccel( int argC, const CExprParser::CStrNum args[] ){

	int objId;
	if (argC != 1 || args[0].m_IsNum){
		gout<<"Invalid Arg(s) to GetObjAccel"<<endl;
		return 0;
	}
	int id;
	bool foundObj = g_pCved->GetObj(args[0].m_Str,id);
	if( !foundObj )
	{
		gout << "GetObjAccel: unable to find object named '";
		gout << args[0].m_Str << "'" << endl;

		return 0.0;
	}

	const CObj* pObj = g_pCved->BindObjIdToClass( id);

	const CVehicleObj* cpVehObj = 
				dynamic_cast<const CVehicleObj *>( pObj );
	if (!cpVehObj){
		gout << "GetObjAccel: Error object not vehicle"<<endl;
		return 0;
	}
	return cpVehObj->GetAccel();
}
//////////////////////////////////////////////////////////////////////////////
///\brief 
///		Returns the Euclidian Distance Squared to the given object
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
///\return	 If the object is not found the Distance is set to 2^31 - 8 feet
///
//////////////////////////////////////////////////////////////////////////////
static double
GetObjDistPow2( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || args[0].m_IsNum;
//	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetObjDistPow2" );

	string objName = args[0].m_Str;

	CPoint3D ovPos;
	g_pCved->GetOwnVehiclePos( ovPos );
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
			if (argC == 2)
			{ //if we are looking for another object
				string objName2 = args[1].m_Str;
				for(int j = 0; j < CHcsmCollection::m_sDynObjDataSize; j++ )
				{
					char temp[cMAX_DYN_OBJ_NAME_SIZE] = { 0 };
					StrncpyFlip(
						temp,
						&CHcsmCollection::m_sDynObjData.name[j*cMAX_DYN_OBJ_NAME_SIZE],
						cMAX_DYN_OBJ_NAME_SIZE 
					);
					if( !strcmp( temp, objName2.c_str() ) )
					{
						ovPos.m_x = CHcsmCollection::m_sDynObjData.pos[j*3];
						ovPos.m_y = CHcsmCollection::m_sDynObjData.pos[j*3+1];
						ovPos.m_z = CHcsmCollection::m_sDynObjData.pos[j*3+2];
					}
				}
			}
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
static double 
GetObjTtcToOv( int argC, const CExprParser::CStrNum args[] )
{
	// make sure CVED is valid
	if( !g_pCved )  return 0;

	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetObjTtcToOv" );

	//
	// Given the object's name, find its CVED id.
	//
	string objName = args[0].m_Str;
	int objId;
	bool foundObj = g_pCved->GetObj( objName, objId );
	if( !foundObj )
	{
		gout << "GetObjTtcToOv: unable to find object named '";
		gout << objName << "'" << endl;

		return 0.0;
	}

	//
	// Calculate the distance from the object to the OV.
	//
	CPoint3D objPos = g_pCved->GetObjPos( objId );
	CPoint3D ovPos;
	bool gotOvInfo = g_pCved->GetOwnVehiclePos( ovPos );
	if( !gotOvInfo )  return 0.0;
	double dist = fabs( (ovPos - objPos).Length() );

	//
	// Calculate the difference in velocities between the object and
	// the OV.
	//
	double objVel = g_pCved->GetObjVel( objId );
	double ovVel;
	gotOvInfo = g_pCved->GetOwnVehicleVel( ovVel );
	if( !gotOvInfo )  return 0.0;
	double velDiff = ( objVel - ovVel ) * cMETER_TO_FEET;  // convert to ft/s

	double ttc = dist / velDiff;
//	gout << objName << ":  obj ttc to ov = " << ttc << endl;
	return ttc;
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
static double 
GetOvTtcToObj( int argC, const CExprParser::CStrNum args[] )
{
	// make sure CVED is valid
	if( !g_pCved )  return 0;

	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetOvTtcToObj" );

	//
	// Given the object's name, find its CVED id.
	//
	string objName = args[0].m_Str;
	int objId;
	bool foundObj = g_pCved->GetObj( objName, objId );
	if( !foundObj )
	{
		gout << "GetObjTtcToOv: unable to find object named '";
		gout << objName << "'" << endl;

		return 0.0;
	}

	//
	// Calculate the distance from the object to the OV.
	//
	CPoint3D objPos = g_pCved->GetObjPos( objId );
	CPoint3D ovPos;
	bool gotOvInfo = g_pCved->GetOwnVehiclePos( ovPos );
	if( !gotOvInfo )  return 0.0;
	double dist = fabs( (ovPos - objPos).Length() );

	//
	// Calculate the difference in velocities between the object and
	// the OV.
	//
	double objVel = g_pCved->GetObjVel( objId );
	double ovVel;
	gotOvInfo = g_pCved->GetOwnVehicleVel( ovVel );
	if( !gotOvInfo )  return 0.0;
	double velDiff = ( ovVel - objVel ) * cMETER_TO_FEET;  // convert to ft/s

	double ttc = dist / velDiff;
//	gout << objName << ":  ov ttc to obj = " << ttc << endl;
	return ttc;
}


void 
CExpressionTrigger::UserCreation( const CTriggerParseBlock* cpBlock )
{
	PrintCreationMessage();
    m_priorityLevel = cpBlock->GetPriority();
	try 
	{
		// Initialize CObjectInitCond object.
		m_initCondition.SetObjectInitCond(
								*cved, 
								GetFrame(), 
								GetTimeStepDuration(),  
								cpBlock->GetCrRad(),  
								cpBlock->GetActvDel(), 
								cpBlock->GetLifetime()
								);

		// Initialize CTriggerFireCond object.
		m_fireCondition.InitFireCondition(
								GetTimeStepDuration(),
								cpBlock->GetFireDelFrames(),
								cpBlock->GetDebounce(),
								cpBlock->GetOneShot()
								);

		// Initialize other local variables.
		m_prevPosition = cpBlock->GetPosition();
		m_prevState = eWAIT;
		m_pTrigger = 0;
		m_expression = cpBlock->GetExpression();

		//
		// Initialize variables related to the expression being evaluated.
		//
#ifdef DEBUG
		gout << MessagePrefix() << "expression:" << endl;
		gout << m_expression << endl;
#endif

		m_expEval.m_functions["sin"] = MySin;
		m_expEval.m_functions["cos"] = MyCos;
		m_expEval.m_functions["ReadCell"] = ReadCell;
		m_expEval.m_functions["CellEquals"] = CellEquals;
		m_expEval.m_functions["ReadVar"] = ReadVar;
		m_expEval.m_functions["GetObjVel"] = GetObjVel;
		m_expEval.m_functions["GetObjAccel"]= GetObjAccel;
		m_expEval.m_functions["GetObjTtcToOv"] = GetObjTtcToOv;
		m_expEval.m_functions["GetOvTtcToObj"] = GetOvTtcToObj;
		m_expEval.m_functions["GetObjDistPow2"] = GetObjDistPow2;
		m_expEval.m_functions["Abs"] = MyAbs;
		m_expEval.m_functions["sqrt"] = SquareRoot;

		bool parseFailed = !m_expEval.Parse( m_expression.c_str() );
		if( parseFailed )
		{
			gout << MessagePrefix() << "failed to parse expression [SUICIDE]" << endl;
			gout << m_expression << endl;

			Suicide();
			return;
		}

		m_nthFromStart = cpBlock->GetNthFromStart();
		m_nthFromEnd = cpBlock->GetNthFromEnd();
		m_nAhead = cpBlock->GetVehicleAhead();
		m_nBehind = cpBlock->GetVehicleBehind();

		// Initialize the action vector.
		GetActions( cpBlock, m_pRootCollection, m_pActionVector );
		m_sequentialActions = cpBlock->GetSequentialActions();
		m_pCurrentAction = m_pActionVector.end();
		m_delayFrameCount = 0;
		m_waitingForAction = false;

		InitCandidates( cpBlock );

		g_pCved = cved;

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
		strstream message;
		message << 
			MessagePrefix()<< 
			"Suicide: error during CExpressionTrigger::UserCreation: " <<
			e.msg;
		Suicide();
		return;
	}
	catch(...) 
	{
		strstream message;
		message << 
			MessagePrefix() << 
			"Suicide: Unknown error during CExpressionTrigger::UserCreation: ";
		Suicide();
		return;
	}
}

void 
CExpressionTrigger::UserActivity( const CTriggerParseBlock* cpBlock )
{
	// If the MakeTraffic button is pressed, then fire the action.
	if( GetButtonFireTrigger() ) 
	{
		ExecuteConcurrentActions();
		return;
	}

	// Get current gmtrcpstn and position.
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

	// Get the current state from the InitCondition object.
	EInitCond state = m_initCondition.Execute( actualPosition, actualTime );
			
	// For use in eACTIVATE state.
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
				// 
				// Initialize the CVED representation of the Trigger.  
				// Because we need to be able to fire it at any time 
				// using the FireTrigger button, it should exist in 
				// CVED for the duration of the HCSM.
				//
				triggerAttr.hcsmId = m_pRootCollection->GetHcsmId( this );
				triggerAttr.xSize = 2.0;
				triggerAttr.ySize = 2.0;

				m_pTrigger = dynamic_cast<CCoordinatorObjectObj *> (
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
					gout << MessagePrefix();
					gout << cpBlock->GetName();
					gout << " unable to create self in CVED... [SUICIDE]";
					gout << endl;

					Suicide();
					return;
				}

				// Set the actions' parents to be this trigger's HCSM id
				CAction::TActionIterator i;
				for( i = m_pActionVector.begin(); i != m_pActionVector.end(); i++ ) 
				{
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
				//
				// Evaluate the predicate if the Fire Condition object 
				// says to.
				//
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
CExpressionTrigger::UserDeletion( const CTriggerParseBlock* cpBlock )
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
CExpressionTrigger::InitCandidates( const CTriggerParseBlock* cpBlock )
{
	// Initialize the candidate sets associated with this trigger.
	vector<string> byNameSet = cpBlock->GetByNameSet();
	if( !byNameSet.empty() )  m_candidateSet.AddNames( byNameSet );

	vector<string> byTypeSet = cpBlock->GetByTypeSet();
	if( !byTypeSet.empty() )  m_candidateSet.AddTypes( byTypeSet );

	vector<pbTRoad> byRoadSet = cpBlock->GetByRoadSet();
	if( !byRoadSet.empty() )  m_candidateSet.AddRoads( byRoadSet );
}

bool
CExpressionTrigger::Evaluate( void )
{
	//
	// Non-zero value makes the predicate true.
	//
	double result = m_expEval.Evaluate();
	bool predicateResult = fabs( result ) > cNEAR_ZERO;

#ifdef DEBUG
	gout << MessagePrefix() << "result = " << result;
	gout << "  return = " << predicateResult << endl;
#endif

	return predicateResult;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the trigger fire activty log.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CExpressionTrigger::SetTriggerFireLog( CAction::TActionIterator& aI )
{
	int instigatorHcsmId = GetInstigatorHcsmId( m_instigatorSet );

	set<CCandidate> candidateSet;
	(*aI)->m_candidateSet.GetCandidates( *cved, candidateSet, *m_pRootCollection);

	int candidateArr[16];
	int candidateArrSize = 0;
	if( (*aI)->m_useInstigators  )
	{
		PutCandidatesIntoArr( m_instigatorSet, candidateArr, candidateArrSize );
	}
	else
	{
		PutCandidatesIntoArr( candidateSet, candidateArr, candidateArrSize );
	}

	m_pRootCollection->SetTriggerFireLog(
				this,
				instigatorHcsmId,
				candidateArr,
				candidateArrSize
				);
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
CExpressionTrigger::ExecuteConcurrentActions()
{
	//
	// Exeucte actions concurrently.
	//
	CAction::TActionIterator aI;
	for( aI = m_pActionVector.begin(); aI != m_pActionVector.end(); aI++ )
	{
		SetTriggerFireLog( aI );
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
CExpressionTrigger::InitializeSequentialActions()
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
CExpressionTrigger::ExecuteSequentialActions()
{
	bool actionsRemaining = m_pCurrentAction != m_pActionVector.end();
	if( actionsRemaining )
	{
		if( m_waitingForAction )
		{
#ifdef DEBUG
			gout << MessagePrefix() << "waiting for action to complete";
			gout << endl;
#endif

			bool isFinished = (*m_pCurrentAction)->IsFinished();
			if( isFinished )
			{
#ifdef DEBUG
				gout << MessagePrefix() << "action completed" << endl;
#endif

				m_waitingForAction = false;
				m_pCurrentAction++;
				actionsRemaining = m_pCurrentAction != m_pActionVector.end();
			}
		}
		if (actionsRemaining && !m_waitingForAction)
		{
#ifdef DEBUG
			gout << MessagePrefix() << "delay = " << m_delayFrameCount << endl;
#endif

			//
			// Wait to make sure that the delay counter has expired before
			// executing the current action.
			//
			bool readyToExecute = m_delayFrameCount <= 0;
			if( readyToExecute )
			{
#ifdef DEBUG
				gout << MessagePrefix() << "executing action" << endl;
#endif
				SetTriggerFireLog( m_pCurrentAction );

				(*m_pCurrentAction)->Execute( &m_instigatorSet );
				m_waitingForAction = true;
				m_delayFrameCount = 
					(int) ( (*m_pCurrentAction)->GetDelay() / GetTimeStepDuration() );
			}
			else
			{
				m_delayFrameCount--;
			}
		}
	}
}