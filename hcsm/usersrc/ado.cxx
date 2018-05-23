/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: ado.cxx,v 1.286 2016/10/28 20:56:06 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    September, 1999
 *
 * Description:  Contains code for the autonomous dynamic object (Ado) HCSM.
 *
 ****************************************************************************/

#include "hcsmpch.h"
#include "controllers.h"
#include "curvature.h"
#include "support.h"
#include "util.h"

#include <pi_fstream>
#include <pi_iostream>
#include <sstream>
using namespace std;
#include <pi_string>

#include <exception>
#include "expevalAdo.h"
#include <tchar.h>

#include "CvedADOctrl.h"

//
// Debugging macros.
//

#undef DEBUG_ADO_INIT
#undef DEBUG_CURVATURE  //1        // needs frame #
#undef SCEN_EXTERNAL_DRIVER
#undef DEBUG_GRAPHIC_DEBUG //4	   // needs cved id

#define HLOG_ADO_USRCREATE_IN   12000		// user creation starts
#define HLOG_ADO_USRCREATE_1    12001		// user creation checkpoint
#define HLOG_ADO_USRCREATE_2    12002		// user creation checkpoint
#define HLOG_ADO_USRCREATE_3    12003		// user creation checkpoint
#define HLOG_ADO_USRCREATE_4    12004		// user creation checkpoint
#define HLOG_ADO_USRCREATE_OUT  12009		// user creation ends
#define HLOG_ADO_ACTIVATED_IN   12010		// 'activated' function in
#define HLOG_ADO_ACTIVATED_1    12011		// 'activated' checkpoint
#define HLOG_ADO_ACTIVATED_2    12012		// 'activated' checkpoint
#define HLOG_ADO_ACTIVATED_3    12013		// 'activated' checkpoint
#define HLOG_ADO_ACT_LISTS_1    12014
#define HLOG_ADO_ACT_LISTS_2    12015
#define HLOG_ADO_ACT_LISTS_3    12016
#define HLOG_ADO_ACT_LISTS_4    12017
#define HLOG_ADO_ACTIVATED_OUT  12018
#define HLOG_ADO_CURV1          12019
#define HLOG_ADO_CURV2          12020
#define HLOG_ADO_CURV3          12021
#define HLOG_ADO_CURV4          12022
#define HLOG_ADO_CURV5          12023


#undef DEBUG_EXTERNAL_DRIVER_SPEED

#ifdef DEBUG_EXTERNAL_DRIVER_SPEED
	double gEXTERNAL_DRIVER_SPEED = 15.9;
#endif

const size_t cMaxTokSize = 256;
extern void
StringFlip( const volatile char from[], char to[], int size );
extern void
StrncpyFlip( char* pDst, const char* cpSrc, int size );
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Determine reaction delay based on user's statistical spec.
//
// Remarks: This function returns a single value that is consistent with
//   the statistical model specified by the user.
//
// Arguments:
//   rgen       - The random number generator o use.
//   stream     - The stream to use in the generator.
//   cDelaySpec - User's specification of delay.
//
// Returns: The recation delay as a double.
//
//////////////////////////////////////////////////////////////////////////////
static double
PickDelValue(
	CRandNumGen& rgen,
	int stream,
	const CAdoParseBlock::CDelay& cDelaySpec
	)
{
	if( cDelaySpec.m_type == CAdoParseBlock::eFIXED )
	{
		return cDelaySpec.m_param1;
	}
	else if( cDelaySpec.m_type == CAdoParseBlock::eUNIFORM )
	{
		return rgen.RandomDoubleRange(
							cDelaySpec.m_param1,
							cDelaySpec.m_param2,
							stream
							);
	}
	else if( cDelaySpec.m_type == CAdoParseBlock::eNORMAL )
	{
		return rgen.NormalDouble(
							cDelaySpec.m_param1,
							cDelaySpec.m_param2,
							stream
							);
	}
	else
	{
		assert( 0 );
	}

	return 0.0;
}
 double
CAdo::GetObjVel( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || args[0].m_IsNum;
//	if( invalidArgs )  PrintInvalidArgs( argC, args, "GetObjVel" );

	string objName = args[0].m_Str;
	//this->cved->geto
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
double
CAdo::ReadVar( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = (argC != 1) || (args[0].m_IsNum);
	if( invalidArgs ) {
		gout << "Invalid Argument count'";
		return 0;
	}

	string varName = args[0].m_Str;
	if( !CHcsmCollection::ExprVariableExists( varName ) )
	{
		return 0.0;
	}

	return CHcsmCollection::GetExprVariable( varName );
}

double
CAdo::GetObjAccel( int argC, const CExprParser::CStrNum args[] ){

	int objId;
	if (argC != 1 || args[0].m_IsNum){
		gout<<"Invalid Arg(s) to GetObjAccel"<<endl;
		return 0;
	}
	int id;
	bool foundObj = cved->GetObj(args[0].m_Str,id);
	if( !foundObj )
	{
		gout << "GetObjAccel: unable to find object named '";
		gout << args[0].m_Str << "'" << endl;

		return 0.0;
	}

	const CObj* pObj = cved->BindObjIdToClass( id);

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
double
CAdo::GetObjDistPow2( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = (argC > 2 || argC < 1 || (args[0].m_IsNum && (argC != 2 || args[0].m_IsNum)) );
	if( invalidArgs )
		return 900000000000000000000.0;

	string objName = args[0].m_Str;

	CPoint3D ovPos;

	int i;
	int objId;
	CPoint3D pos1;

	if (cved->GetObj(objName,objId)){
		pos1 = cved->GetObjPos(objId);
	}
	else
		return 900000000000000000000.0;
	if (argC == 1)
		cved->GetOwnVehiclePos( ovPos );
	else if (argC == 2){
		objName = args[1].m_Str;
		if (cved->GetObj(objName,objId)){
			ovPos = cved->GetObjPos(objId);
		}
		else
			return 900000000000000000000.0;
	}
	double deltaX = ovPos.m_x - pos1.m_x;
	double deltaY = ovPos.m_y - pos1.m_y;
	double deltaZ = ovPos.m_z - pos1.m_z;
	return deltaX * deltaX + deltaY * deltaY + deltaZ + deltaZ;
}

//////////////////////////////////////////////////////////////////////////
///\brief
///		calculates a sign value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T)
////////////////////////////////////////////////////////////////////////
double CAdo::MySin( int argC, const CExprParser::CStrNum args[] ){
	double f = 0.1;
	double offset = 0;
	if (argC == 1){
		if (args[0].m_Num > 0 )
			f = 1/(args[0].m_Num);
	}
	else if (argC == 2){
		if (args[0].m_Num > 0 )
			f = 1/(args[0].m_Num);
		if (args[1].m_IsNum)
			offset = args[0].m_Num;
	}
	else{
		return 0; //we have an invalid number of arguments
	}
	double timeSinceStart = (
		(m_pI->m_ageFrame - m_pI->m_forcedVelStart) *
			m_pI->m_timeStepDuration + offset
		);
	return sin( timeSinceStart*  3.141592654 * f);

}
//////////////////////////////////////////////////////////////////////////
///\brief
///		reads a cell value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T)
////////////////////////////////////////////////////////////////////////
double CAdo::ReadCell( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 2 || args[0].m_IsNum || !args[1].m_IsNum;

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
		if( index < 0 || index >= cNUM_LOG_STREAMS )  throw InvalidArgs;
		return CHcsmCollection::m_sLogStreams[index];
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
		cved->GetOwnVehicleVel( ownVehVel );
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
	else if (cellName == "ACC_SMV"){ //Total Hack, need to fix expression parser
		double ownVehVel = 0.0;
		cved->GetOwnVehicleVel( ownVehVel );
		ownVehVel = ownVehVel * cMS_TO_MPH;
		if (index == 0)
			return ((0-3726.5f-(4000.0f*ownVehVel)/( ownVehVel-25.1f)-400.0f)-(0-15364.1f))/((4000.0f)/(1.46666f*( ownVehVel-25.1f)))/1.466666f;
		if (index == 1)
			return ((0-5507.7-(2218.8* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1f))/((2218.8)/(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 2)
			return ((0-5062.2-(2664.3* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1f))/(( 2664.3)/(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 3)
			return ((0-4616.6-(3109.9* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1f))/(( 3109.9)/(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 4)
			return ((0-4171.1-(3555.4* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1f))/(( 3555.4)/(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 5)
			return ((0-3725.5-(4001  * ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1f))/(( 4001)  /(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 6)
			return ((0-5507.7-(2218.8* ownVehVel)/( ownVehVel-25.1)-600)-(0-15364.1))/((2218.8)/(1.46666*( ownVehVel-25.1)))/1.466666;
            // return ((0-5212.0f-(2514.5f*ownVehVel)/( ownVehVel-25.1f)-700)-(0-15564.1f))/((2514.5f)/(1.46666f*( ownVehVel-25.1f)))/1.466666f;
        if (index == 7)
            return ((0-5062.2-(2664.3* ownVehVel)/( ownVehVel-25.1)-600)-(0-15364.1))/(( 2664.3)/(1.46666*( ownVehVel-25.1)))/1.466666;
			//return ((0-4620-(3106.5f* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1f))/((3106.5)/(1.46666f*( ownVehVel-25.1)))/1.466666f;
        if (index == 8)
            return ((0-4616.6-(3109.9* ownVehVel)/( ownVehVel-25.1)-600)-(0-15364.1))/(( 3109.9)/(1.46666*( ownVehVel-25.1)))/1.466666;
			//return ((0-4030-(3696.5f* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1f))/(( 3696.5)/(1.46666f*( ownVehVel-25.1)))/1.466666f;
        if (index == 9)
            return ((0-4171.1-(3555.4* ownVehVel)/(ownVehVel-25.1)-600)-(0-15364.1))/(( 3555.4)/(1.46666*( ownVehVel-25.1)))/1.466666;
			//return ((0-4150.7-(3575.8f* ownVehVel)/( ownVehVel-20)-700)-(0-15564.1f))/(( 3575.8)/(1.46666f*( ownVehVel-20)))/1.466666f;
        if (index == 10)
			return ((0-3725.5-(4001* ownVehVel)/( ownVehVel-25.1)-600)-(0-15364.1))/((4001)/(1.46666*( ownVehVel-25.1)))/1.466666;
		else
			return 0;

	}
	else if (cellName == "ACC_CHC"){ //Total Hack, need to fix expression parser
		double ownVehVel = 0.0;
		cved->GetOwnVehicleVel( ownVehVel );
		ownVehVel = ownVehVel * cMS_TO_MPH;
		if (index == 1){
			if (CHcsmCollection::m_sCruise_SetSpeed > 49.9 && CHcsmCollection::m_sCruise_SetSpeed < 59.9){
				return CHcsmCollection::m_sCruise_SetSpeed - 5;
			}
			else if (CHcsmCollection::m_sCruise_SetSpeed < 49.5){
				return 45.0;
			}else{
				return 55.0;
			}
		}else if (index == 2){
			if (ownVehVel > 49.9 && ownVehVel < 59.9){
				return ownVehVel - 5.0;
			}
			else if (ownVehVel< 49.5){
				return 45.0;
			}else{
				return 55.0;
			}
		}
	}
	else if (cellName == "Sensor_Info"){
		return CHcsmCollection::m_sSensorInfo[index];
	}
	else if (cellName == "OnPath"){
		return CHcsmCollection::m_sIsOnPath;
	}
/*	else if (cellName == "OwnVehAccel"){
		int ownVehObjId;
		g_pCved->GetObj( "ExternalDriver", ownVehObjId );
		const CDynObj* pOwnVehObj = g_pCved->BindObjIdToClass( ownVehObjId );
		return (pOwnVehObj->GetVel()  - pOwnVehObj->GetVelImm()) / CHcsmCollection::GetTimeStepDuration();
	}*/
	else
	{
		gout << "ExpressionTrigger: unknown cell name '" << cellName;
		gout << "'" << endl;
	}

	return 0.0;
}
//////////////////////////////////////////////////////////////////////////
///\brief
///		Returns the Own Veh. Velocity
///\remark
///		This function is a delegate used by Ado Expression Parser,
///		It also ignores all arguments (the argruments are only needed
///		to maintian compatability with the callback process
///
///\returns double -the Own Veh. Velocity
////////////////////////////////////////////////////////////////////////
double CAdo::GetOvVel( int argC, const CExprParser::CStrNum args[] ){
	double targVel = 0;
	cved->GetOwnVehicleVel( targVel );
	return targVel * cMS_TO_MPH;
}
//////////////////////////////////////////////////////////////////////////
///\brief
///		calculates a sign value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T)
////////////////////////////////////////////////////////////////////////
double CAdo::FadeIn( int argC, const CExprParser::CStrNum args[] ){
	double timeSinceStart = (
		(m_pI->m_ageFrame - m_pI->m_forcedVelStart) *
			m_pI->m_timeStepDuration
		);
	if (argC == 1){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = timeSinceStart/args[0].m_Num;
		if (fade > 1 )
			return 1.0;
		else if (fade < 0 )
			return 0;
		else
			return fade;
	}else if (argC == 2){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = (timeSinceStart- args[1].m_Num)/args[0].m_Num;
		if (fade > 1 )
			return 1.0;
		else if (fade < 0 )
			return 0;
		else
			return fade;
	}
	else{
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////
///\brief
///		calculates a sign value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T)
////////////////////////////////////////////////////////////////////////
double CAdo::FadeOut( int argC, const CExprParser::CStrNum args[] ){
	double timeSinceStart = (
		(m_pI->m_ageFrame - m_pI->m_forcedVelStart) *
			m_pI->m_timeStepDuration
		);
	if (argC == 1){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = timeSinceStart/args[0].m_Num;
		if (fade > 1 )
			return 0.0;
		else if (fade < 0 )
			return 1;
		else
			return (1.0-fade);
	}else if (argC == 2){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = (timeSinceStart- args[1].m_Num)/args[0].m_Num;
		if (fade > 1 )
			return 1.0;
		else if (fade < 0 )
			return 0;
		else
			return (1.0-fade);
	}
	else{
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////
///\brief
///		calculates a sign value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T)
////////////////////////////////////////////////////////////////////////
double CAdo::FadeInGap( int argC, const CExprParser::CStrNum args[] ){
	double timeSinceStart = (
		(GetFrame() - m_pI->m_gapStartFrame) *
			m_pI->m_timeStepDuration
		);
	if (argC == 1){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = timeSinceStart/args[0].m_Num;
		if (fade > 1 )
			return 1.0;
		else if (fade < 0 )
			return 0;
		else
			return fade;
	}else if (argC == 2){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = (timeSinceStart- args[1].m_Num)/args[0].m_Num;
		if (fade > 1 )
			return 1.0;
		else if (fade < 0 )
			return 0;
		else
			return fade;
	}
	else{
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////
///\brief
///		calculates a sign value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T)
////////////////////////////////////////////////////////////////////////
double CAdo::FadeOutGap( int argC, const CExprParser::CStrNum args[] ){
	double timeSinceStart = (
		(GetFrame() - m_pI->m_gapStartFrame) *
			m_pI->m_timeStepDuration
		);
	if (argC == 1){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = timeSinceStart/args[0].m_Num;
		if (fade > 1 )
			return 0.0;
		else if (fade < 0 )
			return 1;
		else
			return (1.0-fade);
	}else if (argC == 2){
		if (timeSinceStart < 0.01)
			return 0;
		double fade = (timeSinceStart- args[1].m_Num)/args[0].m_Num;
		if (fade > 1 )
			return 1.0;
		else if (fade < 0 )
			return 0;
		else
			return (1.0-fade);
	}
	else{
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////
///\brief
///		calculates a sign value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T)
////////////////////////////////////////////////////////////////////////
double CAdo::MySinGap( int argC, const CExprParser::CStrNum args[] ){
	double f = 0.1;
	double offset = 0;
	if (argC == 1){
		if (args[0].m_Num > 0 )
			f = 1/(args[0].m_Num);
	}
	else if (argC == 2){
		if (args[0].m_Num > 0 )
			f = 1/(args[0].m_Num);
		if (args[1].m_IsNum)
			offset = args[0].m_Num;
	}
	else{
		return 0; //we have an invalid number of arguments
	}
	double timeSinceStart = (
		(GetFrame()- m_pI->m_gapStartFrame) *
			m_pI->m_timeStepDuration + offset
		);
	return sin( timeSinceStart*  3.141592654 * f);

}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets the object's run mode.
//
// Remarks:  This function accepts a string that contains the object's
//   run mode and converts it into an enumeration.  It then returns
//   the enumeration.  It prints an error to the display if the input
//   string is not valid.
//
// Arguments:
//   cRunModeStr - A string that represents the object's run mode.
//
// Returns:  An enumeration that represents the object's run mode.
//
//////////////////////////////////////////////////////////////////////////////
CAdoInfo::ERunMode
CAdo::GetRunMode( const string& cRunModeStr )
{
	CAdoInfo::ERunMode runMode;
	if( cRunModeStr == "eAUTONOMOUS" )
	{
		runMode = CAdoInfo::eAUTONOMOUS;
	}
	else if( cRunModeStr == "eREMOTE_CONTROL" )
	{
		runMode = CAdoInfo::eREMOTE_CONTROL;
	}
	else
	{
		gout << MessagePrefix() << "unknown run mode '";
		gout << cRunModeStr << "'....defaulting ";
		gout << "to eREMOTE_CONTROL" << endl;

		runMode = CAdoInfo::eREMOTE_CONTROL;
	}

	return runMode;
}  // end of GetRunMode


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns a code for a dynamic model given a string.
//
// Remarks:  Use this function to translate a string representation of a
//   dynamic model into a code.  The string comes from the SCN file.
//
// Arguments:
//   cDynModelStr - A string that represents the dynamic model.
//
// Returns:  An enumeration that represents the dynamic model.
//
//////////////////////////////////////////////////////////////////////////////
EDynaFidelity
CAdo::GetDynModel( const string& cDynModelStr )
{
	EDynaFidelity dynModel;
	if( cDynModelStr == "Non Linear" )
	{
		dynModel = eNON_LINEAR;
	}
	else if( cDynModelStr == "Full Linear" )
	{
		dynModel = eFULL_LINEAR;
	}
	else if( cDynModelStr == "Reduced Linear" )
	{
		dynModel = eREDUCED_LINEAR;
	}
	else
	{
		gout << MessagePrefix() << "unknown dynamics model '";
		gout << cDynModelStr << "'....defaulting ";
		gout << "to eNON_LINEAR" << endl;

		dynModel = eNON_LINEAR;
	}

	return dynModel;
}  // end of GetDynModel


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Builds a path for an ADO.
//
// Remarks:  This function removes the old path and builds a path
//   starting from the ADO's current road position to the lane provided
//   as the argument.
//
// Arguments:
//   nextLane - The lane to include in the path.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::BuildPath( const CLane& cNextLane )
{
	CRoad nextRoad = cNextLane.GetRoad();
	double dist;
	if ( cNextLane.GetDirection() == ePOS )
	{
		dist = 1.0;
	}
	else
	{
		dist = nextRoad.GetLinearLength() - 1.0;
	}

	CRoadPos nextLaneStart( nextRoad, cNextLane, dist );
	if( nextLaneStart.IsValid() )
	{
		//
		// Clear the path and build a new one with the current road
		// position and the required right turn in the next intersection.
		//
		m_pI->m_pPath->Clear();
		m_pI->m_pPath->Append( m_pI->m_roadPos );
		m_pI->m_pPath->Append( nextLaneStart );
	}
}  // end of BuildPath


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Prints the contents of the SNO block to the screen.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::DumpSnoBlock( const CAdoParseBlock* cpSnoBlock )
{
	gout << "  hcsm id = " << m_pRootCollection->GetHcsmId( this );
	gout << endl;
	if( cpSnoBlock->GetName() != "Ado" )
	{
		gout << "  name = " << cpSnoBlock->GetName() << endl;
	}

	//
	// Initial conditions.
	//
	if( cpSnoBlock->GetCrRad() > 0 )
	{
		gout << "  creation radius = " << cpSnoBlock->GetCrRad();
		gout << " ft" << endl;
	}
	if( cpSnoBlock->GetActvDel() > 0 )
	{
		gout << "  activation delay = " << cpSnoBlock->GetActvDel();
		gout << " s" << endl;
	}
	if( cpSnoBlock->GetLifetime() > 0 )
	{
		gout << "  lifetime = " << cpSnoBlock->GetLifetime() << " s";
		gout << endl;
	}

	//
	// Velocity control.
	//
	if( cpSnoBlock->GetVelCtrlInitVel() > 0 )
	{
		gout << "  init vel = " << cpSnoBlock->GetVelCtrlInitVel();
		gout << " mph" << endl;
	}
	gout << "  velocity control distribution:" <<  endl;
	gout << "    refresh time = ";
	gout << cpSnoBlock->GetVelCtrlRefreshTime() << " s" << endl;
	gout << "    type = ";
	if( cpSnoBlock->GetVelCtrlDistribution().m_type == CAdoParseBlock::eUNIFORM )
	{
		gout << "UNIFORM" << endl;
	}
	else if( cpSnoBlock->GetVelCtrlDistribution().m_type == CAdoParseBlock::eNORMAL )
	{
		gout << "GAUSSIAN" << endl;
	}
	else if( cpSnoBlock->GetVelCtrlDistribution().m_type == CAdoParseBlock::eFIXED )
	{
		gout << "FIXED" << endl;
	}
	else
	{
		gout << "UNKNOWN" << endl;
	}
	gout << "    param1 = " << cpSnoBlock->GetVelCtrlDistribution().m_param1 << endl;
	gout << "    param2  = " << cpSnoBlock->GetVelCtrlDistribution().m_param2 << endl;

	//
	// Lane changes.
	//
	gout << "  lane changes:" << endl;
	if( cpSnoBlock->GetRandomize() )
	{
		gout << "    init delay = " << cpSnoBlock->GetLaneChangeInitDelay();
		gout << " s" << endl;
		gout << "    turn signal randomize = ";
		gout << cpSnoBlock->GetRandomize() << " :";
		vector<double> signalTimes = cpSnoBlock->GetRandomizeSignalTime();
		vector<double>::iterator i;
		for( i = signalTimes.begin(); i != signalTimes.end(); i++ )
		{
			gout << "  " << *i;
		}
		gout << endl;
	}
	else
	{
		gout << "    turn signal time = " << cpSnoBlock->GetSignalTime();
		gout << " s" << endl;
	}
	gout << "    urgency = " << cpSnoBlock->GetUrgency() << endl;
	gout << "    steering force = " << cpSnoBlock->GetSteeringForce();
	gout << endl;
	gout << "    disabled lane change conditions:" << endl;
	if( !cpSnoBlock->GetLosingCorridorEnable() )
	{
		gout << "      losing corridor" << endl;
	}
	if( !cpSnoBlock->GetHighwayMergeEnable() )
	{
		gout << "      highway merge" << endl;
	}
	if( !cpSnoBlock->GetAvoidMergingObjEnable() )
	{
		gout << "      avoid merging objects" << endl;
	}
	if( !cpSnoBlock->GetSlowMovingObjEnable() )
	{
		gout << "      slow moving object" << endl;
	}
	if( !cpSnoBlock->GetVerySlowMovingObjEnable() )
	{
		gout << "      very slow moving object" << endl;
	}
	if( !cpSnoBlock->GetNonPassingLaneEnable() )
	{
		gout << "      non passing lane" << endl;
	}

	if( cpSnoBlock->GetVisualState() > 0 )
	{
		gout << "  lights = " << cpSnoBlock->GetVisualState() << endl;
	}

	gout << "  sol name = " << cpSnoBlock->GetSolName() << endl;
	gout << "  dynamics model = " << cpSnoBlock->GetDynModel() << endl;
	gout << "  runMode = " << cpSnoBlock->GetRunMode() << endl;

	if( cpSnoBlock->GetCmdFile() != "" )
	{
		gout << "  cmd file name = " << cpSnoBlock->GetCmdFile();
		gout << endl;
	}
	if( cpSnoBlock->GetLogFile() != "" )
	{
		gout << "  log file name = " << cpSnoBlock->GetLogFile();
		gout << endl;
	}

	//
	// Follow time and distance.
	//
	gout << "  isFollowTime " << cpSnoBlock->IsFollowTime() << endl;
	if( cpSnoBlock->IsFollowTime() )
	{
		gout << "     min = " << cpSnoBlock->GetFollowTimeMin();
		gout << "  max = " << cpSnoBlock->GetFollowTimeMax() << endl;
	}
	else
	{
		gout << "     min = " << cpSnoBlock->GetFollowDistMin();
		gout << "  max = " << cpSnoBlock->GetFollowDistMax() << endl;
	}

	//
	// Path.
	//
	gout << "  avoidTurnsAtIntersection = " << cpSnoBlock->GetAvoidTurning();
	gout << endl;
	vector<string> tempPath = cpSnoBlock->GetPath();
	gout << "  path [ size = " << tempPath.size() << " ]" << endl;
#if 0
	vector<string>::iterator i;
	for ( i = tempPath.begin(); i != tempPath.end(); i++ ) {
		gout << "    " << *i << endl;
	}
#endif

	//
	// Initial offroad point.
	//
	gout << "  initial offroad path point = " << !cpSnoBlock->IsAdoOnRoad() << endl;
	if( !cpSnoBlock->IsAdoOnRoad() )
	{
		gout << "  offroad point = " << cpSnoBlock->GetOffroadPoint() << endl;
	}

	//
	// Relative creation.
	//
	gout << "  relative creation = " << cpSnoBlock->GetIsCreateRelative() << endl;
	if( cpSnoBlock->GetIsCreateRelative() )
	{
		gout << "    relative obj name = " << cpSnoBlock->GetCreateRelativeObjName() << endl;
		gout << "    long offset = " << cpSnoBlock->GetCreateRelativeOffsetLongitudinal() << " ft" << endl;

		// create by absolute lane or by lateral distance
		gout << "    create lateral feet = " << cpSnoBlock->GetIsCreateLatFeet() << endl;
		bool absolute = !cpSnoBlock->GetIsCreateLatFeet();
		if( absolute )
		{
			gout << "      absolute lane = " << cpSnoBlock->GetCreateAbsoluteLane() << endl;
		}
		else
		{
			gout << "      offs = " << cpSnoBlock->GetCreateRelativeOffsetLateral() << " ft";
		}
	}
}


void
CAdo::UserCreation( const CAdoParseBlock* cpSnoBlock )
{
	m_pI = NULL;
	//
	// Copy the name from SCN file if it has one.
	//
	if( cpSnoBlock->GetName().size() > 0 )  m_name = cpSnoBlock->GetName();

	m_pRootCollection->MemLog( 0, HLOG_ADO_USRCREATE_IN, m_name.c_str());

	//
	// Print creation message with template name and hcsm id.
	//
	PrintCreationMessage();

#ifdef DEBUG_ADO_INIT
	DumpSnoBlock( cpSnoBlock );
#endif

	//
	// Initialize InitCondtions class.
	//
	double activationDelay = cpSnoBlock->GetActvDel();
#ifdef SCEN_EXTERNAL_DRIVER
	if( cpSnoBlock->GetName() == "ExternalDriver" )
	{
		activationDelay = 40.0;
	}
#endif

	m_initConditions.SetObjectInitCond(
				*cved,
				GetFrame(),
				GetTimeStepDuration(),
				cpSnoBlock->GetCrRad(),
//				cpSnoBlock->GetActvDel(),
				activationDelay,
				cpSnoBlock->GetLifetime()
				);

	//
	// Get the SOL object.
	//
	const string cSolName = cpSnoBlock->GetSolName();
	const CSolObj* cpSolObj = cved->GetSol().GetObj( cSolName );
	if( !cpSolObj )
	{
		// invalid SOL name...suicide
		gout << "BH[" << GetFrame() << "]: object (name=";
		gout << cpSnoBlock->GetName();
		gout << ") has an invalid SOL name (" << cSolName;
		gout << ") [SUICIDE]" << endl;

		Suicide();
		return;
	}
	m_exprEval.SetParent(this);
	m_exprEval.m_functions["sin"] = &CAdo::MySin;
	m_exprEval.m_functions["FadeIn"] = &CAdo::FadeIn;
	m_exprEval.m_functions["FadeOut"] = &CAdo::FadeOut;
	m_exprEval.m_functions["OvVel"] = &CAdo::GetOvVel;
	m_exprEval.m_functions["ReadCell"] = &CAdo::ReadCell;
	m_exprEval.m_functions["ReadVar"] = &CAdo::ReadVar;
	m_exprEval.m_functions["GetObjDistPow2"] = &CAdo::GetObjDistPow2;
	m_exprEval.m_functions["GetObjVel"]	= &CAdo::GetObjVel;
	m_exprEval.m_functions["GetObjAccel"]= &CAdo::GetObjAccel;

	m_exprGapDisEvtal.SetParent(this);
	m_exprGapDisEvtal.m_functions["sin"] = &CAdo::MySinGap;
	m_exprGapDisEvtal.m_functions["FadeIn"] = &CAdo::FadeInGap;
	m_exprGapDisEvtal.m_functions["FadeOut"] = &CAdo::FadeOutGap;
	m_exprGapDisEvtal.m_functions["OvVel"] = &CAdo::GetOvVel;
	m_exprGapDisEvtal.m_functions["ReadCell"] = &CAdo::ReadCell;
	m_exprGapDisEvtal.m_functions["ReadVar"] = &CAdo::ReadVar;
	m_exprGapDisEvtal.m_functions["GetObjDistPow2"] = &CAdo::GetObjDistPow2;
	m_exprGapDisEvtal.m_functions["GetObjVel"]	= &CAdo::GetObjVel;
	m_exprGapDisEvtal.m_functions["GetObjAccel"]= &CAdo::GetObjAccel;

	m_exprMinSpeedEvtal.SetParent(this);
	m_exprMinSpeedEvtal.m_functions["sin"] = &CAdo::MySinGap;
	m_exprMinSpeedEvtal.m_functions["FadeIn"] = &CAdo::FadeInGap;
	m_exprMinSpeedEvtal.m_functions["FadeOut"] = &CAdo::FadeOutGap;
	m_exprMinSpeedEvtal.m_functions["OvVel"] = &CAdo::GetOvVel;
	m_exprMinSpeedEvtal.m_functions["ReadCell"] = &CAdo::ReadCell;
	m_exprMinSpeedEvtal.m_functions["ReadVar"] = &CAdo::ReadVar;
	m_exprMinSpeedEvtal.m_functions["GetObjDistPow2"] = &CAdo::GetObjDistPow2;
	m_exprMinSpeedEvtal.m_functions["GetObjVel"]	= &CAdo::GetObjVel;
	m_exprMinSpeedEvtal.m_functions["GetObjAccel"]= &CAdo::GetObjAccel;

	m_exprMaxSpeedEvtal.SetParent(this);
	m_exprMaxSpeedEvtal.m_functions["sin"] = &CAdo::MySinGap;
	m_exprMaxSpeedEvtal.m_functions["FadeIn"] = &CAdo::FadeInGap;
	m_exprMaxSpeedEvtal.m_functions["FadeOut"] = &CAdo::FadeOutGap;
	m_exprMaxSpeedEvtal.m_functions["OvVel"] = &CAdo::GetOvVel;
	m_exprMaxSpeedEvtal.m_functions["ReadCell"] = &CAdo::ReadCell;
	m_exprMaxSpeedEvtal.m_functions["ReadVar"] = &CAdo::ReadVar;
	m_exprMaxSpeedEvtal.m_functions["GetObjDistPow2"] = &CAdo::GetObjDistPow2;
	m_exprMaxSpeedEvtal.m_functions["GetObjVel"]	= &CAdo::GetObjVel;
	m_exprMaxSpeedEvtal.m_functions["GetObjAccel"]= &CAdo::GetObjAccel;

	m_exprMaxAccelEvtal.SetParent(this);
	m_exprMaxAccelEvtal.m_functions["sin"] = &CAdo::MySinGap;
	m_exprMaxAccelEvtal.m_functions["FadeIn"] = &CAdo::FadeInGap;
	m_exprMaxAccelEvtal.m_functions["FadeOut"] = &CAdo::FadeOutGap;
	m_exprMaxAccelEvtal.m_functions["OvVel"] = &CAdo::GetOvVel;
	m_exprMaxAccelEvtal.m_functions["ReadCell"] = &CAdo::ReadCell;
	m_exprMaxAccelEvtal.m_functions["ReadVar"] = &CAdo::ReadVar;
	m_exprMaxAccelEvtal.m_functions["GetObjDistPow2"] = &CAdo::GetObjDistPow2;
	m_exprMaxAccelEvtal.m_functions["GetObjVel"]	= &CAdo::GetObjVel;
	m_exprMaxAccelEvtal.m_functions["GetObjAccel"]= &CAdo::GetObjAccel;

	m_exprMinAccelEvtal.SetParent(this);
	m_exprMinAccelEvtal.m_functions["sin"] = &CAdo::MySinGap;
	m_exprMinAccelEvtal.m_functions["FadeIn"] = &CAdo::FadeInGap;
	m_exprMinAccelEvtal.m_functions["FadeOut"] = &CAdo::FadeOutGap;
	m_exprMinAccelEvtal.m_functions["OvVel"] = &CAdo::GetOvVel;
	m_exprMinAccelEvtal.m_functions["ReadCell"] = &CAdo::ReadCell;
	m_exprMinAccelEvtal.m_functions["ReadVar"] = &CAdo::ReadVar;
	m_exprMinAccelEvtal.m_functions["GetObjDistPow2"] = &CAdo::GetObjDistPow2;
	m_exprMinAccelEvtal.m_functions["GetObjVel"]	= &CAdo::GetObjVel;
	m_exprMinAccelEvtal.m_functions["GetObjAccel"]= &CAdo::GetObjAccel;


	//
	// Get the CVED object type and check to make sure it's valid.
	//
	const string cCategoryName = cpSolObj->GetCategoryName();
	cvEObjType objType = cvString2ObjType( cCategoryName.c_str() );
	if( objType == eCV_INVALID )
	{
		gout << MessagePrefix();
		gout << "UserCreation: invalid CVED object with SOL ";
		gout << "category name = " << cCategoryName << endl;

		Suicide();
		return;
	}

	//
	// Initialize private members.
	//
	if( cpSnoBlock->GetIsCreateRelative() )
	{
        CRoadPos relObjRoadPos = GetRoadposForRelCreate(cpSnoBlock);
        if (!relObjRoadPos.IsValid())
            return;
		//gout << "## final pos = " << createRoadPos << endl;
		m_pI = new CAdoInfo( relObjRoadPos, GetTimeStepDuration(), objType );
	}
	else
	{
		CRoadPos roadPos( *cved, cpSnoBlock->GetRoadPos() );
		if( cpSnoBlock->GetName() == "ExternalDriver" )
		{
			if( !roadPos.IsRoad() )
			{
				// if the roadPos is on a corridor, then we need to build it again
				// as it will correct set the m_cdo
				CRoadPos tempRoadPos( *cved,roadPos.GetBestXYZ() );
				//gout << "  ##road pos = " << roadPos << endl;
				roadPos = tempRoadPos;
			}
			bool updateCrdr = !roadPos.IsRoad() && CHcsmCollection::m_sOwnshipPath.IsValid();
			if( updateCrdr )
			{
				int crdrId = -1;
				bool success = CHcsmCollection::m_sOwnshipPath.GetCrdrFromIntrscn(
							roadPos.GetIntrsctn().GetId(),
							crdrId,
							NULL
							);
				if( success )
				{
					// if the path says that we should be on a different corridor than what the
					// current roadPos has, then we need to reprioritize the current roadPos so
					// that the correct corridor has the priority
					CCrdr currCrdr( roadPos.GetIntrsctn(), crdrId );
					success = roadPos.SetCrdPriority( &currCrdr );
					//gout << "  #road pos = " << roadPos.GetString() << "   success = " << success << endl;
				}
			}
		}
		m_pI = new CAdoInfo( roadPos, GetTimeStepDuration(), objType );
	}
	m_pI->m_objName = cpSnoBlock->GetName();
	m_pI->m_autoControlBrakeLightState = cpSnoBlock->GetAutoControlBrakeLight();
	m_pI->m_autoControlHeadLightsState = cpSnoBlock->GetAutoControlHeadLight();

	static int randGapInit = false;
	static CRandNumGen rgenGap(5, 5);		// 5, 5 are the seeds; just picked them!
	static int   r1Gap;	// all the streas needed by follow

	if( randGapInit == false )
	{
		randGapInit = true;
		r1Gap = rgenGap.GetStream();	// get a stream for each item to randomize
	}

	if (cpSnoBlock->GetFwdDistThresRand())
	{
		vector<double> fwdThreshDists = cpSnoBlock->GetFwdDistThresRandVal();
		vector<double>::iterator i = fwdThreshDists.begin();
		double minDist = *i;
		i++;
		double maxDist = *i;
		m_pI->m_fwdDistThreshold = rgenGap.RandomDoubleRange(minDist, maxDist, r1Gap);
	}
	else{
		m_pI->m_fwdDistThreshold = cpSnoBlock->GetFwdDistThresVal();
	}

	if (cpSnoBlock->GetBackDistThresRand())
	{
		vector<double> backThreshDists = cpSnoBlock->GetBackDistThresRandVal();
		vector<double>::iterator i = backThreshDists.begin();
		double minDist = *i;
		i++;
		double maxDist = *i;
		m_pI->m_backDistThreshold = rgenGap.RandomDoubleRange(minDist, maxDist, r1Gap);
	}
	else{
		m_pI->m_backDistThreshold = cpSnoBlock->GetBackDistThresVal();
	}

	if (cpSnoBlock->GetFwdTimeThresRand())
	{
		vector<double> fwdThreshTimes = cpSnoBlock->GetFwdTimeThresRandVal();
		vector<double>::iterator i = fwdThreshTimes.begin();
		double minTime = *i;
		i++;
		double maxTime = *i;
		m_pI->m_fwdTimeThreshold = rgenGap.RandomDoubleRange(minTime, maxTime, r1Gap);
	}
	else{
		m_pI->m_fwdTimeThreshold = cpSnoBlock->GetFwdTimeThresVal();
	}

	if (cpSnoBlock->GetBackTimeThresRand())
	{
		vector<double> backThreshTimes = cpSnoBlock->GetBackTimeThresRandVal();
		vector<double>::iterator j = backThreshTimes.begin();
		double minTime = *j;
		j++;
		double maxTime = *j;
		m_pI->m_backTimeThreshold = rgenGap.RandomDoubleRange(minTime, maxTime, r1Gap);
	}
	else
	{
		m_pI->m_backTimeThreshold = cpSnoBlock->GetBackTimeThresVal();
	}

	m_pI->m_backDistThreshold;
	m_pI->m_fwdDistThreshold;
	m_pI->m_backTimeThreshold;
	m_pI->m_fwdTimeThreshold;
	int i = 0;

#ifdef DEBUG_ADO_INIT
	gout << "  road pos = " << m_pI->m_roadPos.GetString() << endl;
#endif

	//
	// Read dynamics model information.
	//
	m_pI->m_dynModel = GetDynModel( cpSnoBlock->GetDynModel() );

	//
	// Read run mode information.
	//
	m_pI->m_runMode = GetRunMode( cpSnoBlock->GetRunMode() );

	//
	// Velocity control / Speed Limit.
	//
	if (cpSnoBlock->GetVelCtrlInitMatchOvVel()){
		m_pI->m_velCntrl.initOvVel = true;
		if (!cved->GetOwnVehicleVel(m_pI->m_velCntrl.initVel ) )
			m_pI->m_velCntrl.initVel = 0; //when we do a create this will get set to OvVel
	}else{
		m_pI->m_velCntrl.initOvVel = false;
		m_pI->m_velCntrl.initVel = cpSnoBlock->GetVelCtrlInitVel() * cMPH_TO_MS;
	}
	m_pI->m_velCntrl.followSpeedLimit = cpSnoBlock->GetVelCtrlFollowSpeedLimit();
	m_pI->m_velCntrl.refreshTime = cpSnoBlock->GetVelCtrlRefreshTime();
	m_pI->m_velCntrl.distribution = cpSnoBlock->GetVelCtrlDistribution();
	m_pI->m_velCntrl.distribution.m_param1 *= cMPH_TO_MS;
	m_pI->m_velCntrl.distribution.m_param2 *= cMPH_TO_MS;

	//
	// Agressiveness...faking it for now.
	//
	m_pI->m_aggressiveness = cpSnoBlock->GetAggressiveness();

	//
	// Is there a command file?
	//
	bool haveCmdFile = cpSnoBlock->GetCmdFile().size() > 0;
	if( haveCmdFile )
	{
		m_pI->m_cmdMode = true;
		m_pI->m_commands.ParseCommandFile( cpSnoBlock->GetCmdFile() );
	}

	//
	// Debugging.
	//
	string logFileName = cpSnoBlock->GetLogFile();
	bool haveLogFile = logFileName.size() > 0;
	if( haveLogFile )
	{
		//
		// Open a data file for logging ADO information.
		//
		gout << MessagePrefix();
		gout << "Log file name is '" << logFileName << "'" << endl;

		m_pI->m_pLogFile = new ofstream( logFileName.c_str() );

		if( !m_pI->m_pLogFile )
		{
			gout << MessagePrefix() << "Cannot log HCSM to file ";
			gout << "named '" << logFileName << "'" << endl;

			exit( -1 );
		}
	}  // end if haveLogFile

	//
	// Get stream for random number generator.
	//
	m_pI->m_rngStreamId = m_pRootCollection->m_rng.GetStream();
	bool noMoreStreams = m_pI->m_rngStreamId < 0;
	if( noMoreStreams )
	{
		gout << MessagePrefix();
		gout << "unable to get stream id from random number generator.  ";
		gout << "Might have to increase max number." << endl;
	}

	//
	// Copy path from SnoBlock to the local path.
	//
	m_pI->m_pPath = new CPath( *cved );
	m_pI->m_pPath->SetString( cpSnoBlock->GetPath() );
	bool invalidPath = m_pI->m_pPath->Size() > 0 && !m_pI->m_pPath->IsValid();
	if( invalidPath )
	{
		gout << MessagePrefix() << "invalid path....[SUICIDE]" << endl;
		Suicide();
	}
	m_pI->m_pPath->SetRandNumGen(
						&m_pRootCollection->m_rng,
						m_pI->m_rngStreamId
						);
	m_pI->m_avoidTurnsAtIntersection = cpSnoBlock->GetAvoidTurning();

	//
	// Check path and build curvature.
	//
	bool haveInitialPath = m_pI->m_pPath->Size() > 0;
//	gout << MessagePrefix() << "1 [" << m_pI->m_pPath->Size() << "]:" << endl;
//	gout << *m_pI->m_pPath << endl;
	if( !haveInitialPath )
	{
		NewPath( m_pI );
	}
//	gout << MessagePrefix() << "2 [" << m_pI->m_pPath->Size() << "]:" << endl;
//	gout << *m_pI->m_pPath << endl;
	ExtendPath( m_pI );
//	gout << MessagePrefix() << "3 [" << m_pI->m_pPath->Size() << "]:" << endl;
//	gout << *m_pI->m_pPath << endl;
	m_pI->m_pPath->Prepend(	1.0 );

	if( !m_pRootCollection->m_sDisableCurvature )
	{
		const CSolObjVehicle* cpSolVeh = dynamic_cast<const CSolObjVehicle*>(cpSolObj);
		if (cpSolVeh){
			double limit = cpSolVeh->GetLatAccelLimit();
			m_pI->m_curvature.SetCurvatureLatAccelLimit(limit);
		}
		m_pI->m_curvature.InitializeCurvature( m_pI->m_roadPos, *m_pI->m_pPath );
		m_pI->m_curvature.ResetBuckets();
	}
	ComputeIntersectionTurnSignal();

#ifdef DEBUG_CURVATURE
	gout << "===" << MessagePrefix() << "  roadPos = " << m_pI->m_roadPos;
	gout << endl;
	m_pI->m_curvature.DebugCurvature();
#endif

	//
	// Parse offroad stuff.
	//
	m_pI->m_offroad = !cpSnoBlock->IsAdoOnRoad();
	m_pI->m_offroadCartPos = cpSnoBlock->GetOffroadPoint();

	//
	// Follow time or distance.
	//
	m_pI->m_isFollowTime = cpSnoBlock->IsFollowTime();
	if( m_pI->m_isFollowTime )
	{
		m_pI->m_followMin = cpSnoBlock->GetFollowTimeMin();
		m_pI->m_followMax = cpSnoBlock->GetFollowTimeMax();
	}
	else
	{
		m_pI->m_followMin = cpSnoBlock->GetFollowDistMin();
		m_pI->m_followMax = cpSnoBlock->GetFollowDistMax();
	}

	//
	// Follow algorithm parameters
	//

	// declare the generator static so all ADOs will use the same
	// generator for producing their random numbers for the follow
	// algorithm.  That way the statistical distribution will be
	// across all ADOs which is the intention
	static int randInit = false;
	static CRandNumGen rgen(5, 5);		// 5, 5 are the seeds; just picked them!
	static int   r1, r2, r3, r4, r5;	// all the streas needed by follow

	if( randInit == false )
	{
		randInit = true;
		r1 = rgen.GetStream();	// get a stream for each item to randomize
		r2 = rgen.GetStream();
		r3 = rgen.GetStream();
		r4 = rgen.GetStream();
		r5 = rgen.GetStream();
	}

	double zeroToOne = rgen.RandomDouble(r1);

	m_pI->m_FollowParams.folTimeMode = cpSnoBlock->IsFollowTime();
	if( m_pI->m_FollowParams.folTimeMode )
	{
		m_pI->m_FollowParams.folValue = cpSnoBlock->GetFollowTimeMin() +
				zeroToOne * (cpSnoBlock->GetFollowTimeMax() -
				cpSnoBlock->GetFollowTimeMin());
	}
	else
	{
		m_pI->m_FollowParams.folValue = cpSnoBlock->GetFollowDistMin() +
				zeroToOne * (cpSnoBlock->GetFollowTimeMax() -
				cpSnoBlock->GetFollowTimeMin());
	}
	m_pI->m_FollowParams.ttcThres1     = cpSnoBlock->GetTtcThres1();
	m_pI->m_FollowParams.ttcThres2     = cpSnoBlock->GetTtcThres2();
	m_pI->m_FollowParams.ttcThres3     = cpSnoBlock->GetTtcThres3();
	m_pI->m_FollowParams.engageThres   = cpSnoBlock->GetEngThres();
	m_pI->m_FollowParams.useReactDelay = cpSnoBlock->GetUseReaDel();

	m_pI->m_FollowParams.stopToStartDelay   =
				PickDelValue(rgen, r2, cpSnoBlock->GetStpToAccDel());
	m_pI->m_FollowParams.steadyToDeccelDelay=
				PickDelValue(rgen, r3, cpSnoBlock->GetStdToAccDel());
	m_pI->m_FollowParams.deccelToAccelDelay =
				PickDelValue(rgen, r4, cpSnoBlock->GetDecToAccDel());
	m_pI->m_FollowParams.steadyToAccelDelay =
				PickDelValue(rgen, r5, cpSnoBlock->GetStdToAccDel());

	m_pI->m_FollowParams.normal.distKp       = cpSnoBlock->GetNormDistKp();
	m_pI->m_FollowParams.normal.distKi       = cpSnoBlock->GetNormDistKi();
	m_pI->m_FollowParams.normal.distKd       = cpSnoBlock->GetNormDistKd();
	m_pI->m_FollowParams.normal.velKp        = cpSnoBlock->GetNormVelKp();
	m_pI->m_FollowParams.normal.vel2Kp       = cpSnoBlock->GetNormVel2Kp();
	m_pI->m_FollowParams.normal.ovspeedClip  = cpSnoBlock->GetOvSpeedClip();
	m_pI->m_FollowParams.normal.clipVelRange = cpSnoBlock->GetOvSpeedRng();
	m_pI->m_FollowParams.normal.posAccClip   = cpSnoBlock->GetNormAccelClip();
	m_pI->m_FollowParams.normal.negAccClip   = cpSnoBlock->GetNormDecelClip();
	m_pI->m_FollowParams.normal.bumpStopDist = cpSnoBlock->GetBumpStopDist();
	m_pI->m_FollowParams.normal.appDecRate   = cpSnoBlock->GetAppDecRate();
	m_pI->m_FollowParams.normal.maxAppSpeed  = 0.0;		// NOT USED
	m_pI->m_FollowParams.normal.accelToCatchUp = cpSnoBlock->GetAccel2Catch();

	m_pI->m_FollowParams.emerg.distKp     = cpSnoBlock->GetEmergDistKp();
	m_pI->m_FollowParams.emerg.distKi     = cpSnoBlock->GetEmergDistKi();
	m_pI->m_FollowParams.emerg.distKd     = cpSnoBlock->GetEmergDistKd();
	m_pI->m_FollowParams.emerg.velKp      = cpSnoBlock->GetEmergVelKp();
	m_pI->m_FollowParams.emerg.velKi      = cpSnoBlock->GetEmergVelKi();
	m_pI->m_FollowParams.emerg.velKd      = cpSnoBlock->GetEmergVelKd();
	m_pI->m_FollowParams.emerg.aclKp      = cpSnoBlock->GetEmergAclKp();
	m_pI->m_FollowParams.emerg.negAccClip = cpSnoBlock->GetEmergDecClip();
	m_pI->m_FollowParams.emerg.posAccClip = 0.0;		// RARELY USED

	m_pI->m_FollowParams.maintainGap.distKp     = cpSnoBlock->GetMainGapDistKp();
	m_pI->m_FollowParams.maintainGap.distKi     = cpSnoBlock->GetMainGapDistKi();
	m_pI->m_FollowParams.maintainGap.distKd     = cpSnoBlock->GetMainGapDistKd();
	m_pI->m_FollowParams.maintainGap.velKp      = cpSnoBlock->GetMainGapVelKp();
	m_pI->m_FollowParams.maintainGap.posAccClip = cpSnoBlock->GetMainGapMaxAcc();
	m_pI->m_FollowParams.maintainGap.negAccClip = cpSnoBlock->GetMainGapMaxDec();

	// lane deviation parameters
	switch( cpSnoBlock->GetLcvModel() )
	{
		case CAdoParseBlock::eRAMPS       :
			m_pI->m_randLaneDev.m_Model = ELaneDevRamps;
			break;

		case CAdoParseBlock::eSIN         :
			m_pI->m_randLaneDev.m_Model = ELaneDevSin;
			break;

		case CAdoParseBlock::eRAMPSANDSIN :
			m_pI->m_randLaneDev.m_Model = ELaneDevRampsSin;
			break;

		case CAdoParseBlock::eEXPRESSION  :
			m_pI->m_randLaneDev.m_Model = ELaneDevExpression;
			break;

		default :
			m_pI->m_randLaneDev.m_Model = ELaneDevRampsSin;
			break;
	}

	m_pI->m_randLaneDev.m_Rise1    = cpSnoBlock->GetLcvRiseTime()[0];
	m_pI->m_randLaneDev.m_Rise2    = cpSnoBlock->GetLcvRiseTime()[1];
	m_pI->m_randLaneDev.m_Fall1    = cpSnoBlock->GetLcvFallTime()[0];
	m_pI->m_randLaneDev.m_Fall2    = cpSnoBlock->GetLcvFallTime()[1];
	m_pI->m_randLaneDev.m_Idle1    = cpSnoBlock->GetLcvStableTime()[0];
	m_pI->m_randLaneDev.m_Idle2    = cpSnoBlock->GetLcvStableTime()[1];
	m_pI->m_randLaneDev.m_RampAmp1 = cpSnoBlock->GetLcvRampsAmplitude()[0];
	m_pI->m_randLaneDev.m_RampAmp2 = cpSnoBlock->GetLcvRampsAmplitude()[1];

	m_pI->m_randLaneDev.m_Bias1   = cpSnoBlock->GetLcvBias()[0];
	m_pI->m_randLaneDev.m_Bias2   = cpSnoBlock->GetLcvBias()[1];
	m_pI->m_randLaneDev.m_SinAmp1 = cpSnoBlock->GetLcvSinAmplitude()[0];
	m_pI->m_randLaneDev.m_SinAmp2 = cpSnoBlock->GetLcvSinAmplitude()[1];
	m_pI->m_randLaneDev.m_Frequ1  = cpSnoBlock->GetLcvFrequency()[0];
	m_pI->m_randLaneDev.m_Frequ2  = cpSnoBlock->GetLcvFrequency()[1];
	m_pI->m_randLaneDev.m_Phase1  = cpSnoBlock->GetLcvPhase()[0];
	m_pI->m_randLaneDev.m_Phase2  = cpSnoBlock->GetLcvPhase()[1];

	m_pI->m_randLaneDev.m_ReevalTime1 = cpSnoBlock->GetLcvReevalInterv()[0];
	m_pI->m_randLaneDev.m_ReevalTime2 = cpSnoBlock->GetLcvReevalInterv()[1];

	m_pI->m_randLaneDev.m_Enable      = cpSnoBlock->GetLcvEnable();

	m_pI->m_DisableLaneDevWhenForcedOffsetDial = cpSnoBlock->GetLcvAutoDisable();

	//
	// Set all my descendents to inactive for now.
	//
	SetStateDescendents( eINACTIVE );

	//
	// ForcedLaneOffset dial stuff.
	//
	m_currForcedLaneOffset = 0.0;
	m_prevForcedLaneDialVal = 0.0;

	//
	// Calculating the randomized turn signals.
	//
	bool randomizeTurnSignal = cpSnoBlock->GetRandomize();
	if( randomizeTurnSignal )
	{
		//
		// Calculate uniform randomized turn signal.
		//
		vector<double> signalTimes = cpSnoBlock->GetRandomizeSignalTime();
		vector<double>::iterator i = signalTimes.begin();
		double minTime = *i;
		i++;
		double maxTime = *i;
		m_pI->m_lcInfo.turnSignalTime = CalculateRandomSignalTime(
											minTime,
											maxTime,
											m_pRootCollection->m_rng,
											m_pI->m_rngStreamId
											);
	}
	else
	{
		m_pI->m_lcInfo.turnSignalTime = cpSnoBlock->GetSignalTime();
	}
	m_pI->m_lcInfo.initDelay = cpSnoBlock->GetLaneChangeInitDelay();
	m_pI->m_lcInfo.urgency = cpSnoBlock->GetUrgency();
	m_pI->m_lcInfo.steeringForce = cpSnoBlock->GetSteeringForce();
	m_pI->m_lcInfo.enableLosingCorridor = cpSnoBlock->GetLosingCorridorEnable();
	m_pI->m_lcInfo.enableHighwayMerge = cpSnoBlock->GetHighwayMergeEnable();
	m_pI->m_lcInfo.enableAvoidMergingObject = cpSnoBlock->GetAvoidMergingObjEnable();
	m_pI->m_lcInfo.enableSlowMovingObject = cpSnoBlock->GetSlowMovingObjEnable();
	m_pI->m_lcInfo.enableVerySlowMovingObject = cpSnoBlock->GetVerySlowMovingObjEnable();
	m_pI->m_lcInfo.enableNonPassingLane = cpSnoBlock->GetNonPassingLaneEnable();
    m_pI->m_lcInfo.maxLatOffset = cpSnoBlock->GetMaxLaneChangeLatOffset();
    m_pI->m_lcInfo.minLookAhead = 0.5f;

	m_pRootCollection->MemLog( 0, HLOG_ADO_USRCREATE_OUT, 0 );

	m_pI->m_maintainGap.SetMaintainGapDial( &m_dialMaintainGap );

	//
	// Make an entry into the activity log for HCSM creation.
	//
	m_pRootCollection->SetHcsmCreateLog(
				this,
				m_typeId,
				m_name,
				m_pI->m_roadPos.GetXYZ()
				);
}  // UserCreation
/////////////////////////////////////////////////////////////////////////////
///\brief
///     Get the roadpos for relative create roadposition
///\remark
///     This
///
///
//////////////////////////////////////////////////////////////////////////////
CRoadPos
CAdo::GetRoadposForRelCreate( const CAdoParseBlock* cpSnoBlock){
    CRoadPos emptyRoadpos;
	// get the relative obect's CVED id
	string relObjName = cpSnoBlock->GetCreateRelativeObjName();
	CRoadPos relObjRoadPos(*cved);

    int relObjId;

	bool haveId = cved->GetObj( relObjName, relObjId );
	if( !haveId )
	{
		gout << MessagePrefix();
		gout << "UserCreation: unable to relative object's CVED id" << endl;
		gout << "  objName = " << relObjName << endl;

		Suicide();
		return emptyRoadpos;
	}

	 const CDynObj* pRelObj = cved->BindObjIdToClass( relObjId );
	 if( !pRelObj || !pRelObj->IsValid() )
	 {
	 	gout << MessagePrefix();
	 	gout << "UserCreation: unable to get pointer to relative object";
	 	gout << endl;
	 	gout << "  objName = " << relObjName << "   id = " << relObjId << endl;

	 	Suicide();
	 	return emptyRoadpos;
	 }

	 CHcsm* pRelObjHcsm = m_pRootCollection->GetHcsm( pRelObj->GetHcsmId() );
	 if( !pRelObjHcsm )
	 {
	 	gout << MessagePrefix();
	 	gout << "UserCreation: unable get HCSM pointer to relative object";
	 	gout << endl;
	 	gout << "  objName = " << relObjName << "  id = " << relObjId << endl;

	 	Suicide();
	 	return emptyRoadpos;
	 }


	 bool haveValFromMonitor = pRelObjHcsm->GetMonitorByName(
	 												"RoadPos",
	 												&relObjRoadPos
	 												);
	 if( !haveValFromMonitor || !relObjRoadPos.IsValid() )
	 {
	 	gout << "UserCreation: unable get relative object's RoadPos monitor";
	 	gout << endl;
	 	gout << "  objName = " << relObjName << "  id = " << relObjId << endl;

	 	Suicide();
	 	return emptyRoadpos;
	 }
	//
	// First apply the lateral offset.
	//
	CRoadPos createRoadPos = relObjRoadPos;

	if( cpSnoBlock->GetIsCreateLatFeet() )
	{
		// relative creation by distance, 0 implies same lane
		double offset = cpSnoBlock->GetCreateRelativeOffsetLateral();
		if( fabs( offset ) > 0.0 )
		{
			offset += createRoadPos.GetOffset();
			createRoadPos.SetOffset( offset );
			CPoint3D pos = createRoadPos.GetXYZ();
			createRoadPos.SetXYZ( pos );
		}
	}
	else if( cpSnoBlock->GetIsCreateRelative() )
	{
		int relativeLane = cpSnoBlock->GetCreateRelativeLane();
        if (!createRoadPos.IsRoad()){
            // relative creation by lane relative to current lane
			if( cpSnoBlock->GetCreateRelativeLane() == -1 )
			{
				CCrdr leftCrdr;
				bool result = createRoadPos.GetCorridor().GetLeftCrdrAlongDir(leftCrdr);
				if( result )
				{
					//gout << "before: " << createRoadPos.GetString() << endl;
					bool result = createRoadPos.SetCorridor( leftCrdr, createRoadPos.GetDistance() );
					if( !result )
					{
						gout << "ERROR: relative create failed at " << createRoadPos.GetString() << endl;
					}
					//gout << "after: " << createRoadPos.GetString() << endl;
				}
			}
			else
			{
				gout << MessagePrefix() << "relative lane placement on intersections supported only for left lanes ";
				gout << cpSnoBlock->GetCreateRelativeLane() << endl;
			}
        }
		else
		{
            CLane curlane = createRoadPos.GetLane();
            try{
                //int id = 0;//createRoadPos.GetLane().GetRelativeId();
                if (relativeLane < 0){
                    while (relativeLane < 0){
                        if (!curlane.IsLeftMost()){
                            curlane = curlane.GetLeft();
                        }else{
                            break;
                        }
                        relativeLane++;
                    }
                }else{
                    while (relativeLane > 0){
                        if (!curlane.IsRightMost()){
                            curlane = curlane.GetRight();
                        }else{
                            break;
                        }
                        relativeLane--;
                    }
                }
            }catch(cvCInternalError	e){
                gout<<"Failed to Change Lanes for relative creation with error"<<e.m_msg<<endl;
            }

			int tempLaneId = curlane.GetRelativeId();
            CRoadPos tempPos(createRoadPos.GetRoad(),curlane.GetRelativeId(),createRoadPos.GetDistance());
            if (tempPos.IsValid())
                createRoadPos = tempPos;
        }
	}
	else
	{
		// relative creation by absolute lane
		gout << MessagePrefix() << "absolute lane placement not yet supported";
		gout << endl;

#if 0
		double offset = 0.0;
		CLane lane = createRoadPos.GetLane();
		CLane absoluteLane = CLane( createRoadPos.GetRoad(), cpSnoBlock->GetCreateAbsoluteLane() );
		createRoadPos.SetOffset( 0.0 );
		createRoadPos.SetLane( absoluteLane );
#endif
	}

	if( createRoadPos.IsValid() )
	{
		relObjRoadPos = createRoadPos;
	}

	//
	// Now build a roadPos with the longitudinal offset.
	//
	double distToTravel = 0;
	if( !cpSnoBlock->GetUseExpressionForRelativeLongintidinalOffset() )
	{
		distToTravel = cpSnoBlock->GetCreateRelativeOffsetLongitudinal();
	}
	else
	{
		string expr = cpSnoBlock->GetCreateRelativeOffsetLongitudinalExpression();
		if( expr.size() == 0 )
		{
			gout << "Error " << relObjName;
			gout << " Uses expression for relative offset with an empty expression";
			gout << endl;
		}
		else
		{
			if( m_exprEval.Parse( expr.c_str() ) )
			{
				distToTravel = m_exprEval.Evaluate();
			}
			else
			{
				gout << "Error " << relObjName;
				gout << " Uses expression for relative offset failed to parse with expr:";
				gout << expr << endl;
			}
		}
	}

	// Check to see if the user has provided a path.  A path is very useful for cases where
	// the longitudinal offset carries the creation position across lane/intersection
	// boundaries.  Without a path, the potential for the creation point being in an
	// unexpected lane/corridor exists.
	CPath path( *cved );
	//gout << "startRoadPos = " << relObjRoadPos << endl;
	path.SetString( cpSnoBlock->GetPath() );
	bool invalidPath = path.Size() <= 0 || (!path.IsValid());

    if( invalidPath )
	{

        if (!path.Append(relObjRoadPos)){
            gout << MessagePrefix() << "invalid path....[SUICIDE]" << endl;
		    Suicide();
        }
	}

	// In version 1.277, "bool travelForward = true" was checked in. This is
	// incorrect for cases where distToTravel > 0.0.  It was unknown why it was
	// checked in like this and it is now being set back to
	// "bool travelForward = distToTravel >= 0.0".
	//bool travelForward = true;//distToTravel >= 0.0;
	bool travelForward = distToTravel >= 0.0;
	//gout << "travelForward = " << travelForward << " ft" << endl;
	if( travelForward )
	{
		if( path.Size() > 0 )
		{
#if 0
			bool success = path.Initialize( relObjRoadPos );
			if( !success )
			{
				gout << MessagePrefix() << "unable to initialize path with " << relObjRoadPos << endl;
				Suicide();
			}
#endif

			CRoadPos newRoadPos;
			CLane newLane;
			//gout << "path size is " << path.Size() << "   length = " << path.GetLength() << endl;

			CPath::ETravelCode code = path.Travel( distToTravel, relObjRoadPos, newRoadPos, newLane );
			if( code >= CPath::eCV_TRAVEL_OK )
			{
				createRoadPos = newRoadPos;
			}
			else
			{
				gout << MessagePrefix() << "error: relative creation path travel code = " << code << endl;
				Suicide();
			}
		}
		else
		{
			if( cpSnoBlock->GetAvoidTurning() )
			{
				double distAppended = path.Append(
											cCV_STRAIGHT_TURN,
											cCV_EPSILON,
											distToTravel
											);
				if( distAppended + 1.0 < distToTravel )
				{
					distToTravel -= distAppended;
					//gout << "distToTravel = " << distToTravel << " ft" << endl;
					path.Append( distToTravel );
				}
			}
			else
			{
				bool success = path.Append( distToTravel );
				if( !success )
				{
					gout << "UserCreation: unable get obtain relative creation point";
					gout << endl;
					gout << "  objName = " << relObjName;
					gout << "  relative dist = " << distToTravel << endl;

					Suicide();
					return emptyRoadpos;
				}
			}

			//gout << "path hint = " << relObjRoadPos.GetLane().GetId() << endl;
			createRoadPos = path.GetEnd();
		}
	}
	else
	{
		bool success = path.Prepend( fabs( distToTravel ) + 100.0 );
		if( !success )
		{
			gout << "UserCreation: unable get obtain relative creation point";
			gout << endl;
			gout << "  objName = " << relObjName;
			gout << "  relative dist = " << distToTravel << endl;

			Suicide();
			return emptyRoadpos;
		}
		else
		{
			CPath::ETravelCode code = path.TravelBack( fabs( distToTravel ), createRoadPos, createRoadPos, true );
			if( code != CPath::eCV_TRAVEL_OK && code != CPath::eCV_TRAVEL_END_OF_PATH )
			{
				gout << "UserCreation: unable to travel backwards for relative creation point";
				gout << endl;
				gout << "  objName = " << relObjName;
				gout << "  relative dist = " << distToTravel << endl;

				Suicide();
				return emptyRoadpos;
			}
		}

		//createRoadPos = path.GetStart();
	}
    return createRoadPos;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the variables associated withe the
//   vehicle dynamics.
//
// Remarks:
//
// Arguments:
//   slVehicleObj - A SOL object indicating the vehicle type, model, etc.
//   pVehicleObj - A pointer to the vehicle object.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::InitializeDynamicsVars(
			const CSolObj* cpSolObj,
			CVehicleObj* pVehicleObj
			)
{
	const CSolObjVehicle* cpSolVeh =
				dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
	if( !cpSolVeh )
	{
		gout << MessagePrefix();
		gout << "InitializeDynamicVars: unable to get reference ";
		gout << "to SOL object of category = ";
		gout << cpSolObj->GetCategoryName() << "...[SUICIDE]" << endl;

		Suicide();
		return;
	}

	const CDynaParams& dynaParams = cpSolVeh->GetDynaParams();

	double suspStif =
			( dynaParams.m_SuspStifMin + dynaParams.m_SuspStifMax ) / 2;
	pVehicleObj->SetSuspStif( suspStif );
	pVehicleObj->SetSuspStifImm( suspStif );
	double suspDamp =
			( dynaParams.m_SuspDampMin + dynaParams.m_SuspDampMax ) / 2;
	pVehicleObj->SetSuspDamp( suspDamp );
	pVehicleObj->SetSuspDampImm( suspDamp );

	double tireStif =
			( dynaParams.m_TireStifMin + dynaParams.m_TireStifMax ) / 2;
	pVehicleObj->SetTireStif( tireStif );
	pVehicleObj->SetTireStifImm( tireStif );
	double tireDamp =
			( dynaParams.m_TireDampMin + dynaParams.m_TireDampMax ) / 2;
	pVehicleObj->SetTireDamp( tireDamp );
	pVehicleObj->SetTireDampImm( tireDamp );
	pVehicleObj->SetDynaFidelity( m_pI->m_dynModel );
	pVehicleObj->SetDynaFidelityImm( m_pI->m_dynModel );
	pVehicleObj->SetQryTerrainErrCount( 0 );
	pVehicleObj->SetQryTerrainErrCountImm( 0 );
	pVehicleObj->SetDynaInitComplete( 0 );
	pVehicleObj->SetDynaInitCompleteImm( 0 );
}  // end of InitializeDynamicsVars


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Creates the CVED object for the Ado HCSM.
//
// Remarks:  This function creates the CVED object for the ado.
//   It first gets information about the object to be created from the SOL
//   then creates the CVED object.
//
// Arguments:
//   cpSnoBlock - The Ado's SNO block.
//
// Returns:  A boolean indicating if creating the CVED was successful.
//
//////////////////////////////////////////////////////////////////////////////
bool
CAdo::CreateCvedObject( const CAdoParseBlock* cpSnoBlock )
{
	// variable that gets modified if random generation
	// flag is 'on'.
	string tempSolName = cpSnoBlock->GetSolName();

	//
	// Check for Random Sol requirement
	//
	if( cpSnoBlock->GetRandomSol() )
	{
		// create a temp sol object to query object's category.
		const CSolObj* pTempSolObj = cved->GetSol().GetObj( tempSolName );
		if( !pTempSolObj )
		{
			// invalid SOL name...suicide
			gout << "BH[" << GetFrame() << "]: object (name=" << m_pI->m_objName;
			gout << ") has an invalid SOL name (" << tempSolName;
			gout << ") [SUICIDE]" << endl;

			Suicide();
			return false;
		}
		// get object category name
		const string categoryName = pTempSolObj->GetCategoryName();
		// Query sol for random sol name
		tempSolName =
			cved->GetSol().PickRandomSolObjectFromCategory(
									categoryName,
									m_pRootCollection->m_excludeSolIds
									);
	}

	//
	// Get the SOL object.
	//
	const string cSolName = tempSolName;
	const CSolObj* cpSolObj = cved->GetSol().GetObj( cSolName );
	if( !cpSolObj )
	{
		// invalid SOL name...suicide
		gout << "BH[" << GetFrame() << "]: object (name=" << m_pI->m_objName;
		gout << ") has an invalid SOL name (" << cSolName;
		gout << ") [SUICIDE]" << endl;

		Suicide();
		return false;
	}

	//
	// Get the CVED object type and check to make sure it's valid.
	//
	const string cCategoryName = cpSolObj->GetCategoryName();
	cvEObjType objType;
#ifndef _PowerMAXOS
	if( cpSnoBlock->GetName() == "ExternalDriver" )
	{
		// fake driver
		objType = eCV_EXTERNAL_DRIVER;
	}
	else
	{
		objType = cvString2ObjType( cCategoryName.c_str() );
	}
#else
	objType = cvString2ObjType( cCategoryName.c_str() );
#endif

	if( objType == eCV_INVALID )
	{
		gout << MessagePrefix();
		gout << "CreateCvedObject: invalid CVED object with SOL ";
		gout << "category name = " << cCategoryName << endl;

		Suicide();
		return false;
	}

	//
	// Initialize the attributes.
	//
	cvTObjAttr attr = { 0 };
	attr.solId = cpSolObj->GetId();
	attr.xSize = cpSolObj->GetLength();
	attr.ySize = cpSolObj->GetWidth();
	attr.zSize = cpSolObj->GetHeight();
	attr.colorIndex = cpSnoBlock->GetColorIndex();
	if( objType == eCV_EXTERNAL_DRIVER )
	{
		CHcsm* dmHcsm = m_pRootCollection->GetHcsm( "DriverMirror" );
		attr.hcsmId = m_pRootCollection->GetHcsmId( dmHcsm );
		m_pRootCollection->SetExtDriverSurrogate(this);
	}
	else
	{
		attr.hcsmId = m_pRootCollection->GetHcsmId( this );
	}
	m_pI->m_objLength = cpSolObj->GetLength();
	m_pI->m_objWidth  = cpSolObj->GetWidth();

	double initVel = 0;
	if (m_pI->m_velCntrl.initOvVel){
		cved->GetOwnVehicleVel(initVel);
		m_pI->m_velCntrl.initVel = initVel;
	}else{
		initVel =  m_pI->m_velCntrl.initVel;
	}

	//
	// Get the starting location and add the vehicle's CG in the z-axis.
	//
	CPoint3D cartPos;
	if( m_pI->m_offroad )
	{
		//
		// The vehicle is currently off-road...use the offroad point.
		//
		cartPos = m_pI->m_offroadCartPos;

		CPoint3D targPos = m_pI->m_roadPos.GetXYZ();
		CVector3D tan = targPos - cartPos;
		CVector3D lat( 0.0, 0.0, 0.0 );

		//
		// Create the CVED object.
		//
#if defined ADO_CONTROLLER
		m_pI->m_pObj = static_cast<CCvedADOCtrl*>(cved)->DistriCreateDynObj(
							cpSnoBlock->GetName(),
							attr,
							&cartPos,
							&tan
							);
#else
		m_pI->m_pObj = cved->CreateDynObj(
							cpSnoBlock->GetName(),
							objType,
							m_typeId,
							attr,
							&cartPos,
							&tan
							);
#endif
	}  // end if vehicle offroad
	else
	{
		//
		// The vehicle is on the road.
		//


		cartPos = m_pI->m_roadPos.GetXYZ();
        double vel;
        CRoadPos initTarget = m_pI->m_roadPos;
        if (initVel > 0 ){
            float distLookAhead = initVel * cMETER_TO_FEET;
            if (initTarget.Travel(distLookAhead) == CVED::CRoadPos::eERROR){
                initTarget = m_pI->m_roadPos;
            }
        }
        CVector3D rotVec;
        if (initVel > 0){
            CVector3D vec1 = m_pI->m_roadPos.GetTangentInterpolated();
            CVector3D vec2 = initTarget.GetTangentInterpolated();
            CVector3D vec3 = vec1-vec2;

            auto targPosRp = initTarget.GetBestXYZ();
            auto currPosRp = m_pI->m_roadPos.GetBestXYZ();
            CVector3D targPos(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            CVector3D currPos(currPosRp.m_x,currPosRp.m_y,currPosRp.m_z);
	        CVector3D targetSeg(targPos - currPos);
            float laneWidth = 0;

            float currentMaxOffset = 6.0f;

	        //Scaling this by 1/3 the segment length
	        //was found through experimentations
	        //to work well, this should be parmaiterized
	        vec3.Scale(targetSeg.Length()/3);


            if (vec3.Length() > currentMaxOffset){
                vec3.Scale(currentMaxOffset/vec3.Length());
            }
            CPoint3D projectedXYZ  = initTarget.GetBestXYZ();
	        CRoadPos tempRoadPos(initTarget);
	        tempRoadPos.SetXYZ(projectedXYZ + vec3);
            targPosRp = tempRoadPos.GetBestXYZ();
            CVector3D vec4(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            rotVec = vec4 - currPos;
            rotVec.Normalize();
        }
		//
		// Get the staring tangent and lateral vectors.
		//
		CVector3D tan = m_pI->m_roadPos.GetTangent();
		CVector3D lat = m_pI->m_roadPos.GetRightVec();
        if (initVel > 0){
            //CVector3D diff = tan - rotVec;
            //double yaw = atan2(diff.m_k,diff.m_i);
            //tan.DotP(rotVec)
            double yaw = acos(rotVec.DotP(tan) );
            if (/*diff.Length()*/ yaw > 0.01){
                auto axis = tan.CrossP(rotVec);
                if (axis.m_k<0) yaw*=-1;
                tan.RotZ(yaw);
                lat.RotZ(yaw);
            }
        }
		//
		// Create the CVED object.
		//
#if defined ADO_CONTROLLER
		m_pI->m_pObj = static_cast<CCvedADOCtrl*>(cved)->DistriCreateDynObj(
							m_pI->m_objName,
							attr,
							&cartPos,
							&tan,
							&lat);
#else
		m_pI->m_pObj = cved->CreateDynObj(
//							cpSnoBlock->GetName(),
							m_pI->m_objName,
							objType,
							m_typeId,
							attr,
							&cartPos,
							&tan,
							&lat
							);
#endif
	}  // end else vehicle on the road

	//
	// Make sure that the CVED object got created properly.
	//
	bool invalidObj = !m_pI->m_pObj || !m_pI->m_pObj->IsValid();
	if( invalidObj )
	{
		// unable to create CVED object...suicide
		gout << MessagePrefix();
		gout << "unable to create self in CVED  [SUICIDE]" << endl;

		Suicide();
		return false;
	}

	//
	// Get a pointer to the vehicle object.
	//
	CVehicleObj* pVehicleObj = dynamic_cast<CVehicleObj *>( m_pI->m_pObj );

	//
	// Set the initial velocity.
	//


	bool haveInitVel = initVel >= 0.0;
	if( haveInitVel )
	{
		// set velocity to both buffers
		m_pI->m_pObj->SetVel( initVel, false );
	}

	//
	// Initialize the vehicle state and dynamics.  The SOL contains vehicle
	// dynamics settings particular to each vehicle.
	//
	InitializeDynamicsVars( cpSolObj, pVehicleObj );

	//
	// Set the initial audio and visual state.
	//
	pVehicleObj->SetAudioState( cpSnoBlock->GetAudioState() );
	pVehicleObj->SetVisualState( cpSnoBlock->GetVisualState() );

	//
	// If the audio or visual has been set then assign these values
	// to the dials.  This way the ADO will play those sounds and
	// display those lights until they are explicity turned off.
	//
	if( cpSnoBlock->GetVisualState() > 0 )
	{
		char buf[128];
		sprintf( buf, "%d", cpSnoBlock->GetVisualState() );
		string str( buf );
		m_dialVisualState.SetValue( str );
	}

	if( cpSnoBlock->GetAudioState() > 0 )
	{
		char buf[128];
		sprintf( buf, "%d", cpSnoBlock->GetAudioState() );
		string str( buf );
		m_dialAudioState.SetValue( str );
	}

	m_pI->m_curvature.SetCvedId( m_pI->m_pObj->GetId() );

	//
	// Make an entry into the activity log for CVED creation.
	//
	m_pRootCollection->SetCvedCreateLog(
				this,
				m_pI->m_pObj->GetId(),
				objType,
				cartPos
				);
    //init control
    pVehicleObj->SetTargPos(cartPos);
    pVehicleObj->StoreOldTargPos();

	return true;
}  // end of CreateCvedObject


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses the MaintainGap dial settings.
//
// Remarks:  This function should be called once an ADO has detected that
//   its MaintainGap dial has a setting.  The proper setting for this dial
//   has the format:
//   <object name>;<dist>;<max speed to achieve>;<min speed to stay above>;
//   <urgency>;<disable speed>
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ParseMaintainGapDial()
{

	string dialStr = GetDialMaintainGap();


	//
	// Add an entry to the activity log for dial setting.
	//
	m_pRootCollection->SetDialSettingLog( this, "MaintainGap", dialStr );
    m_pI->m_gapStartFrame = GetFrame();
	const char* pToken = dialStr.c_str();
	char token[cMaxTokSize];
	strncpy( token, pToken, 256 );
	char* pCurPos;
	char* pObjName = strtok_s( token, ";", &pCurPos );
	bool usingXpr =  false;
	if( pObjName )
	{
		string objName = pObjName;

		int objId;
		bool haveCvedObj = cved->GetObj( objName, objId );
		if( haveCvedObj )
		{
			char* pMgType = strtok_s( NULL, "; " , &pCurPos);
			string mgType;
			if (pMgType != NULL)
				mgType = pMgType;
			if( pMgType && ( mgType == "d" || mgType == "t" ) )
			{
				bool distMode = mgType == "d";
				double mgValue = 0;
				char* pValue = strtok_s( NULL, ";", &pCurPos );
				if( pValue )
				{
					string valueStr = pValue;
					m_exprGapDisEvtal.Clear();
					if (valueStr.find("%") != string::npos){
						size_t first = valueStr.find_first_of('%');
						size_t last = valueStr.find_last_of('%');
						if (first != string::npos && last != string::npos){
							string val = valueStr.substr(first+1,last-first-1);
							m_exprGapDisEvtal.Parse(val.c_str());
							mgValue = m_exprGapDisEvtal.Evaluate();
							usingXpr = true;
						}else{
							gout << MessagePrefix();
							gout<<"Error Failed to parse expression "<<valueStr<<" for Maintian Gap"<<endl;
						}
					}else{
						mgValue = atof( valueStr.c_str() );
					}

					if( !distMode )
					{
						// convert from time to dist..the time is specified
						// taking into the account the mg obj's velocity at
						// the time the dial is initially set
						const CDynObj* pObj = cved->BindObjIdToClass( objId );
						if( !pObj || !pObj->IsValid() )
						{
							gout << MessagePrefix();
							gout << "MaintainGap unable to get pointer to obj = ";
							gout << objId << endl;

							m_pI->m_maintainGap.Reset();
							return;
						}

						double objVel = pObj->GetVelImm() * cMETER_TO_FEET;
						mgValue = mgValue * objVel;
						const double cMIN_MG_DIST = 20.0;
						if( fabs( mgValue ) < cMIN_MG_DIST )
						{
							if( mgValue < 0.0 )
							{
								mgValue = -1.0 * cMIN_MG_DIST;
							}
							else
							{
								mgValue = cMIN_MG_DIST;
							}
						}
					}
					/////////////////////////////////////////////////////////////////////////
					////Max Speed
					/////////////////////////////////////////////////////////////////////////
					char* pMaxSpeed = strtok_s( NULL, ";" , &pCurPos );
					double maxSpeed = 70;
					if( pMaxSpeed )
					{
						string maxSpeedStr = pMaxSpeed;
						m_exprMaxSpeedEvtal.Clear();
						//string valueStr = pValue;
						if (maxSpeedStr.find("%") != string::npos){
							size_t first = maxSpeedStr.find_first_of('%');
							size_t last = maxSpeedStr.find_last_of('%');
							if (first != string::npos && last != string::npos){
								string val = maxSpeedStr.substr(first+1,last-first-1);
								m_exprMaxSpeedEvtal.Parse(val.c_str());
								maxSpeed = m_exprMaxSpeedEvtal.Evaluate();
								usingXpr = true;
							}else{
								gout << MessagePrefix();
								gout<<"Error Failed to parse expression "<<maxSpeedStr<<" for Maintian Gap"<<endl;

							}
						}else{
							maxSpeed = atof( maxSpeedStr.c_str() );
						}
					    /////////////////////////////////////////////////////////////////////////
					    ////Min Speed
					    /////////////////////////////////////////////////////////////////////////
						char* pMinSpeed = strtok_s( NULL, ";", &pCurPos  );
						if( pMinSpeed )
						{
							string minSpeedStr = pMinSpeed;
							double minSpeed =0;
							m_exprMinSpeedEvtal.Clear();
							if (minSpeedStr.find("%") != string::npos){
								size_t first = minSpeedStr.find_first_of('%');
								size_t last = minSpeedStr.find_last_of('%');
								if (first != string::npos && last != string::npos){
									string val = minSpeedStr.substr(first+1,last-first-1);
									m_exprMinSpeedEvtal.Parse(val.c_str());
									minSpeed = m_exprMinSpeedEvtal.Evaluate();
									usingXpr = true;
								}else{
									gout << MessagePrefix();
									gout<<"Error Failed to parse expression "<<minSpeedStr<<" for Maintian Gap"<<endl;
								}
							}else{
								minSpeed= atof( minSpeedStr.c_str() );
							}

							char* pDistKp = strtok_s( NULL, ";", &pCurPos  );
							if( pDistKp )
							{
								string distKpStr = pDistKp;
								double distKp = atof( distKpStr.c_str() );

								char* pVelKp = strtok_s( NULL, ";", &pCurPos  );
								if( pVelKp )
								{
									string velKpStr = pVelKp;
									double velKp = atof( velKpStr.c_str() );

									char* pMaxAccel = strtok_s( NULL, ";", &pCurPos  );
									//////////////////////////////////////////////////////////////////////
									///\Max Accel
									/////////////////////////////////////////////////////////////////////
									if( pMaxAccel )
									{
										string maxAccelStr = pMaxAccel;
										double maxAccel = -5;
										m_exprMaxAccelEvtal.Clear();

										if (maxAccelStr.find("%") != string::npos){
											size_t first = maxAccelStr.find_first_of('%');
											size_t last = maxAccelStr.find_last_of('%');
											if (first != string::npos && last != string::npos){
												string val = maxAccelStr.substr(first+1,last-first-1);
												m_exprMaxAccelEvtal.Parse(val.c_str());
												maxAccel = m_exprMaxAccelEvtal.Evaluate();
												usingXpr = true;
											}else{
												gout << MessagePrefix();
												gout<<"Error Failed to parse expression "<<maxAccelStr<<" for Maintian Gap"<<endl;
											}
										}else{
											maxAccel = atof( maxAccelStr.c_str() );
										}
										////////////////////////////////////////////////////////////
										///\Max Deccel
										//////////////////////////////////////////////////////////////
										char* pMaxDecel = strtok_s( NULL, ";" , &pCurPos );
										if( pMaxDecel )
										{
											string maxDecelStr = pMaxDecel;
											m_exprMinAccelEvtal.Clear();
											double maxDecel = -5 ;//atof( maxDecelStr.c_str() );
											if (maxDecelStr.find("%") != string::npos){
												size_t first = maxDecelStr.find_first_of('%');
												size_t last = maxDecelStr.find_last_of('%');
												if (first != string::npos && last != string::npos){
													string val = maxDecelStr.substr(first+1,last-first-1);
													m_exprMinAccelEvtal.Parse(val.c_str());
													maxDecel = m_exprMinAccelEvtal.Evaluate();
													usingXpr = true;
												}else{
													gout << MessagePrefix();
													gout<<"Error Failed to parse expression "<<maxDecelStr<<" for Maintian Gap"<<endl;
												}
											}else{
												maxDecel = atof( maxDecelStr.c_str() );
											}

											char* pPercentAroundTarget = strtok_s( NULL, ";", &pCurPos  );
											if( pPercentAroundTarget )
											{
												string percentAroundTargetStr = pPercentAroundTarget;
												double percentAroundTarget = atof( percentAroundTargetStr.c_str() );


												char* pDistDuration = strtok_s( NULL, ";" , &pCurPos );
												if( pDistDuration )
												{
													string distDurationStr = pDistDuration;
													double distDuration = atof( distDurationStr.c_str() );

													char* pDuration = strtok_s( NULL, ";" , &pCurPos );
													if( pDuration )
													{
														string durationStr = pDuration;
														double duration = atof( durationStr.c_str() );

														// optional parameter
														char* pDisableSpeed = strtok_s( NULL, ";" , &pCurPos );
														if( pDisableSpeed )
														{
															string disableSpeedStr = pDisableSpeed;
															double disableSpeed = atof( disableSpeedStr.c_str() );

															m_pI->m_maintainGap.m_disableSpeed = disableSpeed * cMPH_TO_MS;


														}
														else
														{
															m_pI->m_maintainGap.m_disableSpeed = -1.0;
														}

														// optional parameter
														char* pDuration2 = strtok_s( NULL, ";" , &pCurPos );
														if( pDuration2 )
														{
															string durationStr2 = pDuration2;
															double duration2 = atof( durationStr2.c_str() );

#if 0
															gout << " duration2 = " << duration2 << endl;
#endif
															m_pI->m_maintainGap.m_duration2 = duration2;
														}
														else
														{
#if 0
															gout << " failed to read in duration2 " << endl;
#endif
															m_pI->m_maintainGap.m_duration2 = -2.0;
														}


														m_pI->m_maintainGap.m_objId    = objId;
														m_pI->m_maintainGap.m_objName  = objName;
														m_pI->m_maintainGap.m_distMode = true;
														m_pI->m_maintainGap.m_value    = mgValue;
														m_pI->m_maintainGap.m_maxSpeed = maxSpeed * cMPH_TO_MS;
														m_pI->m_maintainGap.m_minSpeed = minSpeed * cMPH_TO_MS;
														m_pI->m_maintainGap.m_distKp   = distKp;
														m_pI->m_maintainGap.m_velKp    = velKp;
														m_pI->m_maintainGap.m_maxAccel = maxAccel;
														m_pI->m_maintainGap.m_maxDecel = maxDecel;
														m_pI->m_maintainGap.m_percentAroundTarget = percentAroundTarget / 100;
														m_pI->m_maintainGap.m_distDuration = distDuration;
														m_pI->m_maintainGap.m_duration = duration;
														m_pI->m_maintainGap.m_hasExpr = usingXpr;

#if 1
														if( m_pRootCollection->m_verbose )
														{
															gout << MessagePrefix();
															gout << "MaintainGap dial setting:" << endl;
															gout << "  objName = " << m_pI->m_maintainGap.m_objName << endl;
															gout << "  objId = " << m_pI->m_maintainGap.m_objId << endl;
															if( m_pI->m_maintainGap.m_distMode )
															{
																gout << "  distMode = true" << endl;
																gout << "  value = " << m_pI->m_maintainGap.m_value << " ft" << endl;
															}
															else
															{
																gout << "  distMode = false" << endl;
																gout << "  value = " << m_pI->m_maintainGap.m_value << " sec" << endl;
															}
															gout << "  maxSpeed = " << m_pI->m_maintainGap.m_maxSpeed * cMS_TO_MPH << " mph";
															gout << "  minSpeed = " << m_pI->m_maintainGap.m_minSpeed * cMS_TO_MPH << " mph" << endl;
															gout << "  distKp = " << m_pI->m_maintainGap.m_distKp;
															gout << "  velKp = " << m_pI->m_maintainGap.m_velKp << endl;
															gout << "  maxAccel = " << m_pI->m_maintainGap.m_maxAccel << " m/s^2";
															gout << "  maxDecel = " << m_pI->m_maintainGap.m_maxDecel << " m/s^2" << endl;
															gout << "  percentAroundTarget = " << m_pI->m_maintainGap.m_percentAroundTarget * 100 << " %" << endl;
															gout << "  distDuration = " << m_pI->m_maintainGap.m_distDuration << " s" << endl;
															gout << "  duration = " << m_pI->m_maintainGap.m_duration << " s" << endl;
															gout << "  disableSpeed = " << m_pI->m_maintainGap.m_disableSpeed * cMS_TO_MPH << " mph" << endl;
															gout << "  duration2 = " << m_pI->m_maintainGap.m_duration2 << " s" << endl;
															gout << endl;
														}
#endif
													}
													else
													{
														//
														// Invalid duration.
														//
														gout << MessagePrefix();
														gout << "MaintainGap dial has invalid duration = ";
														gout << token << endl;

														m_pI->m_maintainGap.Reset();
													}
												}
												else
												{
													//
													// Invalid time in percent.
													//
													gout << MessagePrefix();
													gout << "MaintainGap dial has invalid timeInPercent = ";
													gout << token << endl;

													m_pI->m_maintainGap.Reset();
												}
											}
											else
											{
												//
												// Invalid percent around target.
												//
												gout << MessagePrefix();
												gout << "MaintainGap dial has invalid percentAroundTarget = ";
												gout << token << endl;

												m_pI->m_maintainGap.Reset();
											}
										}
										else
										{
											//
											// Invalid max decel.
											//
											gout << MessagePrefix();
											gout << "MaintainGap dial has invalid maxDecel = ";
											gout << token << endl;

											m_pI->m_maintainGap.Reset();
										}
									}
									else
									{
										//
										// Invalid max accel.
										//
										gout << MessagePrefix();
										gout << "MaintainGap dial has invalid maxAccel = ";
										gout << token << endl;

										m_pI->m_maintainGap.Reset();
									}
								}
								else
								{
									//
									// Invalid vel Kp.
									//
									gout << MessagePrefix();
									gout << "MaintainGap dial has invalid velKp = ";
									gout << token << endl;

									m_pI->m_maintainGap.Reset();
								}
							}
							else
							{
								//
								// Invalid dist Kp.
								//
								gout << MessagePrefix();
								gout << "MaintainGap dial has invalid distKp = ";
								gout << token << endl;

								m_pI->m_maintainGap.Reset();
							}
						}
						else
						{
							//
							// Invalid min speed.
							//
							gout << MessagePrefix();
							gout << "MaintainGap dial has invalid min speed = ";
							gout << token << endl;

							m_pI->m_maintainGap.Reset();
						}
					}
					else
					{
						//
						// Invalid max speed.
						//
						gout << MessagePrefix();
						gout << "MaintainGap dial has invalid max speed = ";
						gout << token << endl;

						m_pI->m_maintainGap.Reset();
					}
				}
				else
				{
					//
					// Invalid distance.
					//
					gout << MessagePrefix();
					gout << "MaintainGap dial has invalid distance = ";
					gout << token << endl;

					m_pI->m_maintainGap.Reset();
				}
			}
			else
			{
					//
					// Invalid maintain gap type.
					//
					gout << MessagePrefix();
					gout << "MaintainGap dial has invalid type = ";
					gout << token << endl;

					m_pI->m_maintainGap.Reset();
			}
		}
		else
		{
			//
			// Unable to extract object name.
			//
			gout << MessagePrefix();
			gout << "unable to obtain object (name = " << objName;
			gout << ") to maintain gap from...ignoring command" << endl;

			m_pI->m_maintainGap.Reset();
		}
	}
	else
	{
		//
		// Check for reset.
		//
		if( !strcmp( token, "reset" ) )
		{
			m_pI->m_maintainGap.Reset();
		}
		else
		{
			//
			// Invalid object name.
			//
			gout << MessagePrefix();
			gout << "MaintainGap dial has invalid object name = " << token;
			gout << endl;

			m_pI->m_maintainGap.Reset();
			m_dialMaintainGap.Reset();
		}
	}
}  // end of ParseMaintainGapDial


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses the TargetVelocity dial settings.
//
// Remarks:  This function should be called once an ADO has detected that
//   its TargetVelocity dial has a setting.  The proper setting for this dial
//   has the format:  <speed in mph>[;<duration in secs>].
//
//   <speed in mph> = positive values only
//   <duration in secs> = positive values only (optional)
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ParseTargetVelocityDial()
{
	string dialStr = GetDialTargetVelocity();

	//
	// Add an entry to the activity log for dial setting.
	//
	m_pRootCollection->SetDialSettingLog( this, "TargetVelocity", dialStr );

	const char* pToken = dialStr.c_str();
	char token[cMaxTokSize];
	char* pCurPos = NULL;
	strncpy( token, pToken,cMaxTokSize);
	char* pVel = strtok_s( token, ";", &pCurPos  );
	if( pVel )
	{
		string velStr = pVel;
		double vel = atof( velStr.c_str() );

		// duration is optional
		char* pDuration = strtok_s( NULL, ";" , &pCurPos );
		if( pDuration )
		{
			string durationStr = pDuration;
			double duration = atof( durationStr.c_str() );

			if( duration < 0.0 )
			{
				m_pI->m_velCntrl.targetVelDurationCount = static_cast<int>(duration);
			}
			else
			{
				m_pI->m_velCntrl.targetVelDurationCount =
										static_cast<int>(duration / GetTimeStepDuration());
			}
		}
		else
		{
			m_pI->m_velCntrl.targetVelDurationCount = -1;
		}

		m_pI->m_velCntrl.targetVel = vel * cMPH_TO_MS;

#if 1
		if( m_pRootCollection->m_verbose )
		{
			gout << MessagePrefix();
			gout << "TargetVelocity dial setting:" << endl;
			gout << "  velocity = " << m_pI->m_velCntrl.targetVel * cMS_TO_MPH;
			gout << " mph" << endl;
			gout << "  duration = ";
			if( m_pI->m_velCntrl.targetVelDurationCount == -2 )
			{
				gout << "forever...immediate reset" << endl;
			}
			else
			{
				gout << m_pI->m_velCntrl.targetVelDurationCount * GetTimeStepDuration();
				gout << " s" << endl;
			}
			gout << endl;
		}
#endif
	}
	else
	{
		//
		// Invalid direction.
		//
		gout << MessagePrefix();
		gout << "TargetVelocity dial has invalid velocity = ";
		gout << token << endl;
	}
}  // end of ParseTargetVelocityDial

//////////////////////////////////////////////////////////////////////////////
///\brief Parse the ForcedLaneOffset dail string
///
///\remark
///   This function should be called once an ADO has detected that
///   its ForcedLaneOffset dial has a setting.  The proper setting for this dial
///   has the format:  <offset in feet>[;<urgency>].
///\par
///   <offset in feet> = offset x feet to the right < 0 < offset x feet to the left
///   <duration in secs> = between 0 and 1
///
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ParseForcedLaneOffset()
{
	string dialStr = GetDialForcedLaneOffset();
	istringstream dailss(dialStr);
	stringstream errorStr;
	errorStr.clear();
	char tempc;
	bool error = false;
	double tempd;
	bool test;
	m_pRootCollection->SetDialSettingLog( this, "ForcedLaneOffset", dialStr );
	if (!dailss.eof() && dailss.good()){
		dailss>>m_pI->m_forcedLaneOffset;
		if (dailss.fail()){
			m_pI->m_forcedLaneOffset = 0.0;
			errorStr<<"Error Invalid ForcedLaneOffset Dailstring"<<endl;
		}
		if (!dailss.eof() && dailss.good()){
			dailss>>tempc;
			if (tempc != ';' || dailss.fail()){
				errorStr<<"Invalid or Missing Delimiter Used :"<<tempc<<endl;
				error = true;
			}
			dailss.peek(); //make sure the next char is not eof.
			if(!dailss.eof() && dailss.good()){
				dailss>>tempd;
				if (!dailss.fail()){
					if (tempd > 0.0 && tempd <= 2.0){
						m_pI->m_forcedLaneOffsetUrgency = tempd;
					}else {
						errorStr<<"Invalid Urgency"<<endl;
						error = true;
						m_pI->m_forcedLaneOffsetUrgency = 0.5;
					}
					if(!dailss.eof() && dailss.good()){
						dailss>>tempc;
						if (tempc != ';' || dailss.fail()){
							errorStr<<"Invalid or Missing Delimiter Used :"<<tempc<<endl;
							error = true;
						}
						dailss>>tempd;
						if (!dailss.fail()){
							m_pI->m_forcedOffsetDelay = (int)tempd;
					        if(!dailss.eof() && dailss.good()){
						        dailss>>tempc;
						        if (tempc != ';' || dailss.fail()){
							        errorStr<<"Invalid or Missing Delimiter Used :"<<tempc<<endl;
							        error = true;
						        }
						        dailss>>tempd;
						        if (!dailss.fail()){
							        m_pI->m_forcedLaneOffsetTurnRate = tempd;
                                    if(!dailss.eof() && dailss.good()){
						                dailss>>tempc;
						                if (tempc != ';' || dailss.fail()){
							                errorStr<<"Invalid or Missing Delimiter Used :"<<tempc<<endl;
							                error = true;
						                }
                                        dailss>>tempd;
                                        if (!dailss.fail()){
                                            m_pI->m_forcedOffsetMaxSteerDistance = tempd;
                                            if(!dailss.eof() && dailss.good()){
						                        dailss>>tempc;
						                        if (tempc != ';' || dailss.fail()){
							                        errorStr<<"Invalid or Missing Delimiter Used :"<<tempc<<endl;
							                        error = true;
						                        }
                                                dailss>>tempd;
                                                if (!dailss.fail()){
                                                    m_pI->m_forcedOffsetMaxSteerForce = tempd;
                                                    if(!dailss.eof() && dailss.good()){
						                                dailss>>tempc;
						                                if (tempc != ';' || dailss.fail()){
							                                errorStr<<"Invalid or Missing Delimiter Used :"<<tempc<<endl;
							                                error = true;
						                                }
                                                        dailss>>tempd;
                                                        if (!dailss.fail()){
                                                            m_pI->m_forcedOffsetTolerance = tempd/12.0f;
                                                           if(!dailss.eof() && dailss.good()){
						                                        dailss>>tempc;
						                                        if (tempc != ';' || dailss.fail()){
                                                                    errorStr<<"Invalid or Missing Delimiter Used :"<<tempc<<endl;
                                                                    error = true;
                                                                }
                                                                dailss>>tempd;
                                                                if (!dailss.fail()){
                                                                    m_pI->m_forcedOffsetMinLookAhead = tempd;
                                                                }else{
                                                                    m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                                                }
                                                           }else{
                                                               m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                                           }
                                                        }
                                                        else{
                                                            m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                                            m_pI->m_forcedOffsetTolerance = 1/12.0f;
                                                        }
                                                    }else{
                                                        m_pI->m_forcedOffsetTolerance = 0.05;
                                                        m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                                    }
                                                }else{
                                                    m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                                                    m_pI->m_forcedOffsetTolerance = 0.05;
                                                    m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                                }
                                            }else{
                                                m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                                                m_pI->m_forcedOffsetTolerance = 0.05;
                                                m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                            }
                                        }else{
                                            m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                                            m_pI->m_forcedOffsetTolerance = 0.05;
                                            m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                                            m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                        }
                                    }else{
                                        m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                                        m_pI->m_forcedOffsetTolerance = 0.05;
                                        m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                                        m_pI->m_forcedOffsetMinLookAhead = 0.5;
                                    }
						        }else{
                                    m_pI->m_forcedLaneOffsetTurnRate = 129.25;
                                    m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                                    m_pI->m_forcedOffsetTolerance = 0.05;
                                    m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                                    m_pI->m_forcedOffsetMinLookAhead = 0.5;
						        }
					        }else{
						        m_pI->m_forcedOffsetDelay = 10;
                                m_pI->m_forcedLaneOffsetTurnRate = 129.25;
                                m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                                m_pI->m_forcedOffsetTolerance = 0.05;
                                m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                                m_pI->m_forcedOffsetMinLookAhead = 0.5;
					        }
						}else{
							m_pI->m_forcedOffsetDelay = 10;
                            m_pI->m_forcedLaneOffsetTurnRate = 129.25;
                            m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                            m_pI->m_forcedOffsetTolerance = 0.05;
                            m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                            m_pI->m_forcedOffsetMinLookAhead = 0.5;
						}
					}else{
						m_pI->m_forcedOffsetDelay = 10;
                        m_pI->m_forcedLaneOffsetTurnRate = 129.25;
                        m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                        m_pI->m_forcedOffsetTolerance = 0.05;
                        m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                        m_pI->m_forcedOffsetMinLookAhead = 0.5;
					}
				}else{
					m_pI->m_forcedLaneOffsetUrgency = 0.5;
					m_pI->m_forcedOffsetDelay = 10;
                    m_pI->m_forcedLaneOffsetTurnRate = 129.25;
                    m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                    m_pI->m_forcedOffsetTolerance = 0.05;
                    m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                    m_pI->m_forcedOffsetMinLookAhead = 0.5;
				}
			}else{
				m_pI->m_forcedLaneOffsetUrgency = 0.5;
				m_pI->m_forcedOffsetDelay = 10;
                m_pI->m_forcedLaneOffsetTurnRate = 129.25;
                m_pI->m_forcedOffsetMaxSteerForce =1.0f;
                m_pI->m_forcedOffsetTolerance = 0.05;
                m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
                m_pI->m_forcedOffsetMinLookAhead = 0.5;
			}
		}else{
			m_pI->m_forcedLaneOffsetUrgency = 0.5;
			m_pI->m_forcedOffsetDelay = 10;
            m_pI->m_forcedLaneOffsetTurnRate = 129.25;
            m_pI->m_forcedOffsetMaxSteerForce =1.0f;
            m_pI->m_forcedOffsetTolerance = 0.05;
            m_pI->m_forcedOffsetMaxSteerDistance = 9.0f;
            m_pI->m_forcedOffsetMinLookAhead = 0.5;
		}
	}else{
		errorStr<<"Unknown Error"<<endl;
		error = true;
	}
	//
	// Add an entry to the activity log for dial setting.
	//


	if (error)
	{
		gout << MessagePrefix();
		gout << "ForcedLaneOffset dial has an Error "<<endl;
		gout<<errorStr.str();
	}
}  // end of ParseTargetVelocityDial

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses the ForcedVelocity dial settings.
//
// Remarks:  This function should be called once an ADO has detected that
//   its ForcedVelocity dial has a setting.  The proper setting for this dial
//   has the formats:
//      <target vel> [ ;<target accel> ]
//   OR any of the following forms:
//      stay
//      change  num%
//      change  num
//      reset
//      <target vel> [; <target accel> ]
//
//   The target velocity is required whereas the target acceleration is
//   optional.
//
// Arguments:
//   targVel   - (output) The target velocity read from the dial (in m/s).
//   haveAccel - (output) Is there a target acceleration.
//   targAccel - (output) The target acceleration read from the dial (in m/s^2).
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
bool
CAdo::ParseForcedVelocityDial(
			const double& cCurrVel,
			double& targVel,
			bool& haveAccel,
			double& targAccel
			)
{
	if( m_dialForcedVelocity.HasValue() )
	{
		string dialStr( GetDialForcedVelocity() ); // avoid string copying

		const char* pToken = dialStr.c_str();
		char keywd[64];
		sscanf( pToken, "%s", keywd );

		//
		// The 'stay' keyword simply means to remain at current speed
		// whatever it is.
		//
		if( !strcmp( keywd, "stay" ) )
		{
			// if there is an established forced velocity, we simply
			// return it otherwise we establish one
			if( m_pI->m_forcedVel < 0.0 )
			{
				m_pI->m_forcedVel      = cCurrVel;
				m_pI->m_forcedVelAccel = 0.0;
				m_pI->m_forcedVelStart = m_pI->m_ageFrame;
				targVel                = cCurrVel;
			}
			else
			{
				targVel = m_pI->m_forcedVel;
				//it seems that we have some slight losses in dynamics, we need to have some accel
				if (m_pI->m_forcedVel < cCurrVel){
					m_pI->m_forcedVelAccel = 1.0;
				}else if (m_pI->m_forcedVel > cCurrVel){
					m_pI->m_forcedVelAccel = -1.0;
				}else{
					m_pI->m_forcedVelAccel = 0;
				}
			}
			targAccel = m_pI->m_forcedVelAccel;		// remain at same speed
			haveAccel = true;
		}
		else if( !strcmp( keywd, "change" ) )
		{
			// changes current value
			double val   = -1;
			double accel = -1;
			char  perc  = 'x';
			int   numConv;
			if( m_pI->m_forcedVel < 0.0 )
			{
				// no established value
				numConv = sscanf( pToken, "%*s%lf%c%lf", &val, &perc, &accel );

				if( numConv == 1 )
				{
					// change, no %, no accel
					m_pI->m_forcedVel      = val * cMPH_TO_MS;
					targVel                = m_pI->m_forcedVel;
					m_pI->m_forcedVelAccel = 200.0;		// sentinel for 'no accel'
					haveAccel              = false;
				}
				else if( numConv == 2 )
				{
					// change, maybe %
					if( perc == '%' )
					{
						m_pI->m_forcedVel  = cCurrVel * ( 1.0 + val / 100.0 );
					}
					else
					{
						m_pI->m_forcedVel  = cCurrVel + ( val * cMPH_TO_MS );
					}
					targVel                = m_pI->m_forcedVel;
					m_pI->m_forcedVelAccel = 200.0;		// sentinel for 'no accel'
					haveAccel              = false;
				}
				else if( numConv == 3 )
				{
					// change, maybe %, accel
					if( perc == '%' )
					{
						m_pI->m_forcedVel  = cCurrVel * (1.0 + val / 100.0);
					}
					else
					{
						m_pI->m_forcedVel  = cCurrVel + val*cMPH_TO_MS;
					}
					targVel   = m_pI->m_forcedVel;

					// adjust acceleration to catch situations when the user
					// asks to decelerate bug gives positive accel
					if( val < 0.0 && accel > 0.0 || val > 0.0 && accel < 0.0 )
					{
						accel = -accel;
					}

					targAccel = m_pI->m_forcedVelAccel = accel;
					haveAccel = true;
				}
				else
				{
					assert( 0 );
				}
				m_pI->m_forcedVelStart = m_pI->m_ageFrame;
			}
			else
			{
				// have established value
				targVel = m_pI->m_forcedVel;
				if( m_pI->m_forcedVelAccel > 199 )
				{
					// check for sentinel
					haveAccel = false;
				}
				else
				{
					targAccel = m_pI->m_forcedVelAccel;
					haveAccel = true;
				}
			}
		}

		// format: sin a b c d
		// Velocity is: vel = a + b * sin(time+d / c * 2pi)
		// a = base velocity (in mph)
		// b = amplitude of sine wave (in mph)
		// c = period of sin wave (in seconds)
		// d = phase of sin wave (in seconds)
		// Unlinke a permanent change, when set to sin, we calculate at every
		// time step but we still keep track of when it was first applied so
		// we can use the proper phase for the sin wave.  We also compute
		// the acceleration ourselves, since it is important to have the
		// right acceleration and it varies
		else if( !strcmp( keywd, "sin" ) )
		{
			double a, b, c, d;
			int   numConv;

			if( m_pI->m_forcedVel < 0 )
			{
				m_pI->m_forcedVelStart = m_pI->m_ageFrame;
			}

			numConv = sscanf( pToken, "%*s%lf%lf%lf%lf", &a, &b, &c, &d );
			if( numConv == 4 )
			{
				bool divideByZero = c < cNEAR_ZERO;
				if( divideByZero )
				{
					gout << MessagePrefix();
					gout << "ERROR: ForcedVelocity dial has negative or zero value for c";
					gout << endl;

					return false;
				}

				double timeSinceStart = (
						(m_pI->m_ageFrame - m_pI->m_forcedVelStart) *
						m_pI->m_timeStepDuration
						);

//				targVel = cMPH_TO_MS * a + cMPH_TO_MS * b * sin(c * now + d);
				targVel =  a + b * sin(((timeSinceStart + d) / c) * (2 * 3.14159));
				targVel *= cMPH_TO_MS;
				// compute required acceleration
				haveAccel = true;
				targAccel = (targVel - cCurrVel) / m_pI->m_timeStepDuration;
				// accel is capped at 10, this seems to be the highest val accel is stable at.
				if (targAccel > 6 )
					targAccel = 6;
				else if (targAccel < -6)
					targAccel = -6;
				m_pI->m_forcedVel = targVel;


#if 0
				gout << MessagePrefix();
				gout << "ForcedVelocity dial setting: sinwave" << endl;
				gout << "  a = " << a << " mph" << endl;
				gout << "  b = " << b << " mph" << endl;
				gout << "  c = " << c << endl;
				gout << "  d = " << d << endl;
				gout << "  targVel = " << m_pI->m_forcedVel * cMS_TO_MPH << " mph" << endl;
#endif
			}
			else
			{
				gout << MessagePrefix();
				gout << "ForcedVelocity dial has invalid format = ";
				gout << pToken << endl;

				return false;
			}
		}

		//
		// Force my velocity to be the same as the OwnVehicle's every frame.
		//
		else if( !strcmp( keywd, "ovvel" ) )
		{
			haveAccel = true;
			m_pI->m_forcedVelAccel = 6;
			targAccel = m_pI->m_forcedVelAccel;
			bool inactive = m_pI->m_forcedVel < 0.0;
			if( inactive )
			{
				m_pI->m_forcedVelStart = m_pI->m_ageFrame;

				double duration = -1;
				int numConv = sscanf( pToken, "%*s%lf", &duration );

				if( numConv == 1 && duration >= 0 )
				{
					m_pI->m_forcedVelEnd =
						static_cast<int>(m_pI->m_forcedVelStart + ( duration / GetTimeStepDuration() ));
				}
				else
				{
					// no duration specified
					m_pI->m_forcedVelEnd = -1;
				}

#if 1
				if( m_pRootCollection->m_verbose )
				{
					gout << MessagePrefix();
					gout << "ForcedVelocity dial setting: ovvel" << endl;
					gout << "  duration = ";
					if( duration == -2 )
					{
						gout << "forever...immediate reset" << endl;
					}
					else if ( duration < 0 )
					{
						gout << "forever" << endl;
					}
					else
					{
						gout << duration << " s" << endl;
					}
				}
#endif

				//
				// Add an entry to the activity log for dial setting.
				//
				m_pRootCollection->SetDialSettingLog(
							this,
							"ForcedVelocity",
							dialStr
							);

				bool targVelValid = cved->GetOwnVehicleVel( targVel );
				if( targVelValid )
				{
					m_pI->m_forcedVel = targVel;
				}
				else
				{
					gout << MessagePrefix();
					gout << "ForcedVelocity dial cannot find ownveh";
					gout << endl;

					return false;
				}

				m_pI->m_forcedVelMode = eFV_OV_VEL;

				if( duration == -2 )
				{
					SetDialForcedVelocityNoValue();
				}
			}
			else
			{
				bool durationExpired = (
							m_pI->m_forcedVelEnd > 0 &&
							GetFrame() >= m_pI->m_forcedVelEnd
							);
				if( durationExpired )
				{
					// expired duration...reset dial
					SetDialForcedVelocityNoValue();
					m_pI->m_forcedVel      = -1.0;
					m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
					m_pI->m_forcedVelStart = -1;
					m_pI->m_forcedVelEnd   = -1;
					m_pI->m_forcedVelMode  = eFV_NONE;
					m_pI->m_forcedVelFileHandle = -1;
					m_pI->m_lastOvel = -1;
					return false;
				}
				else
				{

					//From Observation 2.5 time the first order differential of ovvel
					//seems to work well, a better system for taking into account the
					//change in accerlation of ov-vel should be used
					bool targVelValid = cved->GetOwnVehicleVel( targVel );
					if( targVelValid )
					{
						if (m_pI->m_lastOvel != -1){
							m_pI->m_forcedVel = targVel + 2.5*(targVel - m_pI->m_lastOvel);
							m_pI->m_lastOvel = targVel ;

						}else{
							m_pI->m_lastOvel = targVel ;
							m_pI->m_forcedVel = targVel;
							//m_pI->m_forcedVelAccel = 2*(cCurrVel - targVel);
						}
						targVel = m_pI->m_forcedVel;
						if (targVel - cCurrVel > 0)
							targAccel = 10;
						else
							targAccel = -10;

						m_pI->m_forcedVelAccel = targAccel;

					}
					else
					{
						gout << MessagePrefix();
						gout << "ForcedVelocity dial cannot find ownveh";
						gout << endl;

						return false;
					}
				}
			}
		}

		//
		// Track Own Vehicle's velocity every frame and apply that to ADO
		//
		else if( !strcmp( keywd, "ovtrack" ) )
		{
			haveAccel = false;

			bool inactive = m_pI->m_forcedVel < 0.0;
			if( inactive )
			{
				m_pI->m_forcedVelStart = m_pI->m_ageFrame;

				double duration = -1.0;
				double change = 0.0;
				int numConv = sscanf( pToken, "%*s%lf%lf", &change, &duration );

				m_pI->m_forcedVelChange = change; // setting change param
				if( duration > 0 )
				{
					m_pI->m_forcedVelEnd =
							static_cast<int>(m_pI->m_forcedVelStart + ( duration / GetTimeStepDuration() ));
				}
				else
				{
					m_pI->m_forcedVelEnd = -1;
				}

#if 1
				if( m_pRootCollection->m_verbose )
				{
					gout << MessagePrefix();
					gout << "ForcedVelocity dial setting: ovtrack" << endl;
					if ( numConv == 2 && duration > 0.0 )
					{
						gout << "  change = " << change << " mph" << endl;
						gout << "  duration = " << duration << " s" << endl;
					}
					else if ( numConv == 2 )
					{
						gout << "  change = " << change << " mph" << endl;
						if( duration == -2 )
						{
							gout << "  duration = forever...immediate reset" << endl;
						}
						else if ( duration < 0 )
						{
							gout << "  duration = forever" << endl;
						}
					}
			}
#endif

				//
				// Add an entry to the activity log for dial setting.
				//
				m_pRootCollection->SetDialSettingLog(
							this,
							"ForcedVelocity",
							dialStr
							);

				double ownVel = 0.0;
				bool targVelValid = cved->GetOwnVehicleVel( ownVel );
				if( targVelValid )
				{
					targVel = ownVel + m_pI->m_forcedVelChange * cMPH_TO_MS;
					m_pI->m_forcedVel = targVel;
					//gout << "Forced Vel: " << ownVel * cMS_TO_MPH << " + "
					//	 << m_pI->m_forcedVelChange << " = "
					//	 << m_pI->m_forcedVel * cMS_TO_MPH << endl;
				}
				else
				{
					gout << MessagePrefix();
					gout << "ForcedVelocity dial cannot find ownveh";
					gout << endl;

					return false;
				}

				m_pI->m_forcedVelMode = eFV_OV_TRACK;

				if( duration == -2 )
				{
					SetDialForcedVelocityNoValue();
				}
			}
			else
			{
				bool durationExpired = (
							m_pI->m_forcedVelEnd > 0 &&
							GetFrame() >= m_pI->m_forcedVelEnd
							);
				if( durationExpired )
				{
					// expired duration...reset dial
					SetDialForcedVelocityNoValue();
					m_pI->m_forcedVel      = -1.0;
					m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
					m_pI->m_forcedVelStart = -1;
					m_pI->m_forcedVelEnd   = -1;
					m_pI->m_forcedVelMode  = eFV_NONE;
					m_pI->m_forcedVelChange  = 0.0;
					m_pI->m_forcedVelFileHandle = -1;
					m_pI->m_lastOvel = -1;
					return false;
				}
				else
				{
					double ownVel;
					bool targVelValid = cved->GetOwnVehicleVel( ownVel );
					if( targVelValid )
					{
						targVel = ownVel + m_pI->m_forcedVelChange * cMPH_TO_MS;
						m_pI->m_forcedVel = targVel;
						//gout << "Forced Vel: " << ownVel * cMS_TO_MPH << " + "
						//	 << m_pI->m_forcedVelChange << " = "
						//	 << m_pI->m_forcedVel * cMS_TO_MPH << endl;
					}
					else
					{
						gout << MessagePrefix();
						gout << "ForcedVelocity dial cannot find ownveh";
						gout << endl;

						return false;
					}
				}
			}

		}
		else if( !strcmp( keywd, "reset" ) )
		{
			// reset dial
			SetDialForcedVelocityNoValue();
			m_pI->m_forcedVel      = -1.0;
			m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
			m_pI->m_forcedVelStart = -1;
			m_pI->m_forcedVelEnd   = -1;
			m_pI->m_forcedVelMode  = eFV_NONE;
			m_pI->m_forcedVelFileHandle = -1;
			m_pI->m_lastOvel = -1;

			//
			// Add an entry to the activity log for dial setting.
			//
			m_pRootCollection->SetDialSettingLog(
						this,
						"ForcedVelocity",
						dialStr
						);

			return false;
		}
		else if (!strcmp(keywd, "expr")){
            haveAccel = true;
			bool inactive = m_pI->m_forcedVel < 0.0 || m_pI->m_forcedVelExpression.size() == 0;
			if( inactive )
			{
				m_pI->m_forcedVelStart = m_pI->m_ageFrame;
				double duration = -1;
				double accel = 10;
				int numConv = sscanf( pToken, "%*s%lf%lf", &duration, &accel );
				haveAccel = true;
				if( numConv > 0)
				{
					if (duration > 0) {
						m_pI->m_forcedVelEnd =
							static_cast<int>(GetFrame() + ( duration / GetTimeStepDuration() ));
					}else if ( numConv > 1 ){
						accel = fabs(accel);
					}else{
						accel = 10;
					}
				}
				else
				{
					// no duration specified
					m_pI->m_forcedVelEnd = -1;
					accel = 10;
				}
			    m_pI->m_forcedVelAccel = accel;

				size_t first = dialStr.find_first_of('%');
				size_t last = dialStr.find_last_of('%');
				if (first != string::npos && last != string::npos){
					m_pI->m_forcedVelExpression = dialStr.substr(first+1,last-first-1);
					if (!m_exprEval.Parse(m_pI->m_forcedVelExpression.c_str())){
						gout << MessagePrefix();
						gout << "ForcedVelocity dial cannot parse expression::";
						gout << m_pI->m_forcedVelExpression;
						gout << endl;
						return false;
					}
				}

			}
			bool durationExpired = (
							m_pI->m_forcedVelEnd > 0 &&
							GetFrame() >= m_pI->m_forcedVelEnd
							);
			if( durationExpired )
				{
					// expired duration...reset dial
					SetDialForcedVelocityNoValue();
					m_pI->m_forcedVel      = -1.0;
					m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
					m_pI->m_forcedVelStart = -1;
					m_pI->m_forcedVelEnd   = -1;
					m_pI->m_forcedVelMode  = eFV_NONE;
					m_pI->m_forcedVelChange  = 0.0;
					m_pI->m_forcedVelFileHandle = -1;
					m_pI->m_lastOvel = -1;
					return false;
				}
			targVel = m_exprEval.Evaluate();
			targVel *= cMPH_TO_MS;
			if (targVel - cCurrVel > 0)
				targAccel = fabs(m_pI->m_forcedVelAccel);
			else
				targAccel = fabs(m_pI->m_forcedVelAccel) * -1;
			//m_pI->m_forcedVelAccel = targAccel;
			m_pI->m_forcedVel = targVel;
			//cout<<"ado forced vel: "<<targVel<<endl;

		}
		else if( !strcmp( keywd, "file:" ) )
		{
			//
			// The format of each line in the file should be as follows:
			// <targvel in mph> <targaccel in m/s^2>
			//
			bool inactive = m_pI->m_forcedVel < 0.0;
			if( inactive )
			{
				char buf[64];
				int numConv = sscanf( pToken, "%*s%s%lf", buf, &m_pI->m_forcedVelFileMultiplier );
				string fileName = buf;

				if( numConv == 1 )
				{
					//
					// Find the file in the list of preloaded files.
					//
//					gout << "Need to look for '" << fileName << "'" << endl;
					m_pI->m_forcedVelFileHandle =
							m_pRootCollection->GetPreloadFileHandle( fileName );
					if( m_pI->m_forcedVelFileHandle < 0 )
					{
						gout << "ForcedVelocity dial unable to get handle to file '";
						gout << fileName << "'" << endl;

						return false;
					}
					m_pI->m_forcedVelFileMultiplier = 1.0;

//					gout << "FV: FileHandle = " << m_pI->m_forcedVelFileHandle << endl;
				}
				else if ( numConv == 2 )
				{
					//
					// Find the file in the list of preloaded files.
					//
					//					gout << "Need to look for '" << fileName << "'" << endl;
					m_pI->m_forcedVelFileHandle =
						m_pRootCollection->GetPreloadFileHandle( fileName );
					if( m_pI->m_forcedVelFileHandle < 0 )
					{
						gout << "ForcedVelocity dial unable to get handle to file '";
						gout << fileName << "'" << endl;

						return false;
					}
//					gout << "FV: FileHandle = " << m_pI->m_forcedVelFileHandle << endl;
				}
				else
				{
					gout << "ForcedVelocity dial has inavlid 'file' setting" << endl;
					return false;
				}

#if 1
				if( m_pRootCollection->m_verbose )
				{
					gout << MessagePrefix();
					gout << "ForcedVelocity dial setting: file '" << fileName;
					gout << "'" << endl;
				}
#endif

				//
				// Add an entry to the activity log for dial setting.
				//
				m_pRootCollection->SetDialSettingLog(
							this,
							"ForcedVelocity",
							dialStr
							);
			}

			string data;
			bool gotData = m_pRootCollection->GetNextPreloadFileSample(
							m_pI->m_forcedVelFileHandle,
							data
							);
			if( gotData )
			{
				sscanf( data.c_str(), "%lf %lf", &targVel, &targAccel );
				targVel *= cMPH_TO_MS;
				haveAccel = true;
				targAccel *= m_pI->m_forcedVelFileMultiplier;
				m_pI->m_forcedVel = targVel;

//				gout << "tv = " << targVel << "  ta = " << targAccel << endl;
//				gout << "tv = " << m_pI->m_forcedVel * cMS_TO_MPH;
//				gout << "  cv = " << m_pI->m_currVel * cMS_TO_MPH << endl;
			}
			else
			{
				// reached end of data
				SetDialForcedVelocityNoValue();
				m_pI->m_forcedVel      = -1.0;
				m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
				m_pI->m_forcedVelStart = -1;
				m_pI->m_forcedVelEnd   = -1;
				m_pI->m_forcedVelMode  = eFV_NONE;
				m_pI->m_forcedVelChange  = 0.0;
				m_pI->m_forcedVelFileHandle = -1;
				m_pI->m_lastOvel = -1;
				return false;
			}
		}
		else
		{
			if( m_pI->m_forcedVel < 0.0 )
			{
				// no established value
				char token[cMaxTokSize];
				char* pCurrPos;
				strncpy( token, pToken,cMaxTokSize);
				char* pTargVel = strtok_s( token, "; ",&pCurrPos );
				m_pI->m_ageFrame = GetFrame();	// added

				if( pTargVel )
				{
					m_pI->m_forcedVelStart = m_pI->m_ageFrame;
					m_pI->m_forcedVel = targVel = atof( pTargVel ) * cMPH_TO_MS;

					char* pTargAccel = strtok_s( NULL, "; ",&pCurrPos  );
					if( pTargAccel )
					{
						targAccel = atof( pTargAccel );
						if ( targVel > cCurrVel )
							targAccel = fabs(targAccel);
						else
							targAccel = -fabs(targAccel);
						m_pI->m_forcedVelAccel = targAccel;
						haveAccel = true;

						char* pDuration = strtok_s( NULL, "; ",&pCurrPos  );
						if( pDuration )
						{
							double duration = atof( pDuration );
							m_pI->m_forcedVelEnd =
								static_cast<int>(m_pI->m_forcedVelStart + ( duration / GetTimeStepDuration() ));
						}
						else
						{
							m_pI->m_forcedVelEnd = -1;
						}
					}
					else
					{
						m_pI->m_forcedVelAccel = 200.0;
						haveAccel = false;
					}

#if 1
					if( m_pRootCollection->m_verbose )
					{
						gout << MessagePrefix();
						gout << "ForcedVelocity dial setting:" << endl;
						gout << "  targVel = " << m_pI->m_forcedVel * cMS_TO_MPH << " mph" << endl;
						gout << "  targAccel = ";
						if( haveAccel )
						{
							gout << targAccel << " m/s*s" << endl;
						}
						else
						{
							gout << "none" << endl;
						}
						gout << "  duration = ";
						if( m_pI->m_forcedVelEnd > 0 )
						{
							double duration =
									(m_pI->m_forcedVelEnd - m_pI->m_forcedVelStart) * GetTimeStepDuration();
							gout << duration << " s" << endl;
						}
						else
						{
							gout << "none" << endl;
						}
					}
#endif

					//
					// Add an entry to the activity log for dial setting.
					//
					m_pRootCollection->SetDialSettingLog(
								this,
								"ForcedVelocity",
								dialStr
								);

				}
				else
				{
					gout << MessagePrefix();
					gout << "ForcedVelocity dial has invalid target velocity = ";
					gout << token << endl;

					return false;
				}
			}
			else
			{
				bool durationExpired = (
							m_pI->m_forcedVelEnd > 0 &&
							GetFrame() >= m_pI->m_forcedVelEnd
							);
				if( durationExpired )
				{
					// expired duration...reset dial
					SetDialForcedVelocityNoValue();
					m_pI->m_forcedVel      = -1.0;
					m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
					m_pI->m_forcedVelStart = -1;
					m_pI->m_forcedVelEnd   = -1;
					m_pI->m_forcedVelFileHandle = -1;
					m_pI->m_lastOvel = -1;
					return false;
				}
				else
				{
					targVel = m_pI->m_forcedVel;
					if( m_pI->m_forcedVelAccel > 199 )
					{
						// check for sentinel
						haveAccel = false;
					}
					else
					{
						targAccel = m_pI->m_forcedVelAccel;
						haveAccel = true;
					}
				}
			}

		}
	}
	else if( m_dialForcedVelocity.HasBeenReset() )
	{
		// reset dial
		SetDialForcedVelocityNoValue();
		m_pI->m_forcedVel      = -1.0;
		m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
		m_pI->m_forcedVelStart = -1;
		m_pI->m_forcedVelEnd   = -1;
		m_pI->m_forcedVelMode  = eFV_NONE;
		m_pI->m_forcedVelFileHandle = -1;
		m_pI->m_lastOvel = -1;
		return false;
	}
	else if( m_pI->m_forcedVelMode == eFV_OV_VEL )
	{
		//
		// The dial has already been reset.
		//
		haveAccel = false;
		bool durationExpired = (
					m_pI->m_forcedVelEnd > 0 &&
					GetFrame() >= m_pI->m_forcedVelEnd
					);
		if( durationExpired )
		{
			// expired duration...reset internal settings
			m_pI->m_forcedVel      = -1.0;
			m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
			m_pI->m_forcedVelStart = -1;
			m_pI->m_forcedVelEnd   = -1;
			m_pI->m_forcedVelMode  = eFV_NONE;
			m_pI->m_forcedVelFileHandle = -1;
			m_pI->m_lastOvel = -1;
			return false;
		}
		else
		{
			bool targVelValid = cved->GetOwnVehicleVel( targVel );
			if( targVelValid )
			{
				m_pI->m_forcedVel = targVel;
			}
			else
			{
				gout << MessagePrefix();
				gout << "ForcedVelocity dial cannot find ownveh";
				gout << endl;

				return false;
			}
		}
	}
	else if( m_pI->m_forcedVelMode == eFV_OV_TRACK )
	{
		//
		// The dial has already been reset.
		//
		haveAccel = false;
		bool durationExpired = (
					m_pI->m_forcedVelEnd > 0 &&
					GetFrame() >= m_pI->m_forcedVelEnd
					);
		if( durationExpired )
		{
			// expired duration...reset internal settings
			m_pI->m_forcedVel      = -1.0;
			m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
			m_pI->m_forcedVelStart = -1;
			m_pI->m_forcedVelEnd   = -1;
			m_pI->m_forcedVelMode  = eFV_NONE;
			m_pI->m_forcedVelChange  = 0.0;
			m_pI->m_forcedVelFileHandle = -1;
			m_pI->m_lastOvel = -1;

			return false;
		}
		else
		{
			double ownVel;
			bool targVelValid = cved->GetOwnVehicleVel( ownVel );
			if( targVelValid )
			{
				targVel = ownVel + m_pI->m_forcedVelChange * cMPH_TO_MS;
				m_pI->m_forcedVel = targVel;

				//gout << "Forced Vel: " << ownVel * cMS_TO_MPH << " + "
				//	 << m_pI->m_forcedVelChange << " = "
				//	 << m_pI->m_forcedVel * cMS_TO_MPH << endl;
			}
			else
			{
				gout << MessagePrefix();
				gout << "ForcedVelocity dial cannot find ownveh";
				gout << endl;

				return false;
			}
		}
	}
	else
	{
		m_pI->m_forcedVel      = -1.0;
		m_pI->m_forcedVelAccel = 200.0;	// no need to set but do anyway
		m_pI->m_forcedVelStart = -1;
		m_pI->m_forcedVelEnd   = -1;
		m_pI->m_forcedVelMode  = eFV_NONE;
		m_pI->m_forcedVelFileHandle = -1;
		m_pI->m_lastOvel = -1;
		return false;
	}

	return true;
}  // end of ParseForcedVelocity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses the LaneChangeStatus dial settings.
//
// Remarks:  This function should be called once an ADO has detected that
//   its LaneChangeStatus dial has a setting.  The proper setting for this
//   dial has the format:  <status code>[;<duration in secs>].
//
//   <status code>      = cLC_STATUS_DO_LC (-1), cLC_STATUS_NO_LC (0),
//                        cLC_STATUS_NO_LC_UNTIL_INTRSCTN (1)
//   <duration in secs> = positive values only (optional)
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ParseLaneChangeStatusDial()
{
	string dialStr = GetDialLaneChangeStatus();

	//
	// Add an entry to the activity log for dial setting.
	//
	m_pRootCollection->SetDialSettingLog( this, "LaneChangeStatus", dialStr );

	const char* pToken = dialStr.c_str();
	char token[cMaxTokSize];
	char* pCurPos = NULL;
	strncpy( token, pToken,cMaxTokSize);
	char* pCode = strtok_s( token, ";" , &pCurPos );
	if( pCode )
	{
		string codeStr = pCode;
		int code = atoi( codeStr.c_str() );

		// duration is optional
		char* pDuration = strtok_s( NULL, ";" , &pCurPos );
		if( pDuration )
		{
			string durationStr = pDuration;
			double duration = atof( durationStr.c_str() );

			m_pI->m_lcStatusCount = static_cast<int>(duration / GetTimeStepDuration());
		}
		else
		{
			m_pI->m_lcStatusCount = -1;
		}

		m_pI->m_lcStatus = code;

#if 1
		if( m_pRootCollection->m_verbose )
		{
			gout << MessagePrefix();
			gout << "LaneChangeStatus dial setting:" << endl;
			gout << "  status = " << m_pI->m_lcStatus << endl;
			gout << "  duration = ";
			gout << m_pI->m_lcStatusCount * GetTimeStepDuration();
			gout << " s" << endl;
			gout << endl;
		}
#endif
	}
	else
	{
		//
		// Invalid direction.
		//
		gout << MessagePrefix();
		gout << "LaneChangeStatus dial has invalid status code = ";
		gout << token << endl;
	}
}  // end of ParseLaneChangeStatusDial


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses the AudioState dial settings.
//
// Remarks:  This function should be called once an ADO has detected that
//   its AudioState dial has a setting.  The proper setting for this dial
//   has the format:  <audio state>[;<duration in secs>].
//
//   <audio state>      = positive values only
//   <duration in secs> = positive values only (optional)
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ParseAudioStateDial()
{
	string dialStr = GetDialAudioState();

	//
	// Add an entry to the activity log for dial setting.
	//
	m_pRootCollection->SetDialSettingLog( this, "AudioState", dialStr );

	const char* pToken = dialStr.c_str();
	char* pCurPos = NULL;
	char token[cMaxTokSize];
	strncpy( token, pToken,cMaxTokSize);
	char* pAudioState = strtok_s( token, ";", &pCurPos  );
	if( pAudioState )
	{
		string audioStateStr = pAudioState;
		int audioState = atoi( audioStateStr.c_str() );
		bool invalidAudioState = audioState < 0;
		if( invalidAudioState )
		{
			gout << MessagePrefix();
			gout << "invalid setting for AudioState dial" << endl;
			gout << "  audio state = '" << audioState << "'" << endl;

			return;
		}

		// duration is optional
		char* pDuration = strtok_s( NULL, ";" , &pCurPos );
		if( pDuration )
		{
			string durationStr = pDuration;
			double duration = atof( durationStr.c_str() );
			m_pI->m_audioStateFromDialCount =
								static_cast<int>(duration / GetTimeStepDuration());
		}
		else
		{
			m_pI->m_audioStateFromDialCount = -1;
		}

		m_pI->m_audioStateFromDial = audioState;

#if 1
		if( m_pRootCollection->m_verbose )
		{
			gout << MessagePrefix();
			gout << "AudioState dial setting:" << endl;
			gout << "  audio state = " << m_pI->m_audioStateFromDial << endl;
			gout << "  duration = ";
			if( m_pI->m_audioStateFromDialCount < 0 )
			{
				gout << "forever until reset" << endl;
			}
			else
			{
				gout << m_pI->m_audioStateFromDialCount * GetTimeStepDuration();
				gout << " s" << endl;
			}
			gout << endl;
		}
#endif
	}
	else
	{
		//
		// Invalid direction.
		//
		gout << MessagePrefix();
		gout << "AudioState dial has invalid light state = ";
		gout << token << endl;
	}
}  // end of ParseAudioStateDial


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses the VisualState dial settings.
//
// Remarks:  This function should be called once an ADO has detected that
//   its VisualState dial has a setting.  The proper setting for this dial
//   has the format:  <light state>[;<duration in secs>].
//
//   <light state>      = positive values only
//   <duration in secs> = positive values only (optional)
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ParseVisualStateDial()
{
	string dialStr = GetDialVisualState();

	//
	// Add an entry to the activity log for dial setting.
	//
	m_pRootCollection->SetDialSettingLog( this, "VisualState", dialStr );

	const char* pToken = dialStr.c_str();
	char token[cMaxTokSize];
	strncpy( token, pToken,cMaxTokSize);
	char* pCurPos = NULL;
	char* pVisState = strtok_s( token, ";" , &pCurPos );
	if( pVisState )
	{
		string visStateStr = pVisState;
		int visState = atoi( visStateStr.c_str() );
		bool invalidVisState = visState < 0;
		if( invalidVisState )
		{
			gout << MessagePrefix();
			gout << "invalid setting for VisualState dial" << endl;
			gout << "  visual state = '" << visState << "'" << endl;

			return;
		}

		// duration is optional
		char* pDuration = strtok_s( NULL, ";", &pCurPos  );
		if( pDuration )
		{
			string durationStr = pDuration;
			double duration = atof( durationStr.c_str() );
			if( duration < 0.0 )
			{
				m_pI->m_visualStateFromDialCount = static_cast<int>(duration);
			}
			else
			{
				m_pI->m_visualStateFromDialCount =
									static_cast<int>(duration / GetTimeStepDuration());
			}
		}
		else
		{
			m_pI->m_visualStateFromDialCount = -1;
		}

		m_pI->m_visualStateFromDial = visState;

#if 1
		if( m_pRootCollection->m_verbose )
		{
			gout << MessagePrefix();
			gout << "VisualState dial setting:" << endl;
			gout << "  light state = " << m_pI->m_visualStateFromDial << endl;
			gout << "  duration = ";
			if( m_pI->m_visualStateFromDialCount == -2 )
			{
				gout << "forever...reseting now" << endl;
			}
			else if( m_pI->m_visualStateFromDialCount < 0 )
			{
				gout << "forever until reset" << endl;
			}
			else
			{
				gout << m_pI->m_visualStateFromDialCount * GetTimeStepDuration();
				gout << " s" << endl;
			}
			gout << endl;
		}
#endif
	}
	else
	{
		//
		// Invalid direction.
		//
		gout << MessagePrefix();
		gout << "VisualState dial has invalid light state = ";
		gout << token << endl;
	}
}  // end of ParseVisualStateDial


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses the LaneChange dial settings.
//
// Remarks:  This function should be called once an ADO has detected that
//   its LaneChange dial has a setting.  The proper setting for this dial
//   has the format:  <lane change direction>;<urgency>.
//
//   <lane change direction> = 0 for left lane changes
//   <lane chnage direction> = 1 for right lane changes
//   <urgency> >= 0.0  and <urgency> <= 1.0
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ParseLaneChangeDial()
{

	string dialStr = GetDialLaneChange();

	//
	// Add an entry to the activity log for dial setting.
	//
	m_pRootCollection->SetDialSettingLog( this, "LaneChange", dialStr );

	const char* pToken = dialStr.c_str();
	char* pCurPos = NULL;
	char token[cMaxTokSize];
	strncpy( token, pToken,cMaxTokSize);
	char* pLcDirection = strtok_s( token, ";" , &pCurPos );
	if( pLcDirection )
	{
		string lcDirectionStr = pLcDirection;
		int lcDirection;
		if( lcDirectionStr == "right" )
		{
			lcDirection = 1;  // go right
		}
		else
		{
			lcDirection = 0;  // go left
		}

		char* pUrgency = strtok_s( NULL, ";", &pCurPos  );
		if( pUrgency )
		{
			string urgencyStr = pUrgency;
			double urgency = atof( urgencyStr.c_str() );

			char* pSignalTime = strtok_s( NULL, ";", &pCurPos  );
			if( pSignalTime )
			{
				string signalTimeStr = pSignalTime;
				double signalTime = atof( signalTimeStr.c_str() );

				// duration is optional
				char* pDuration = strtok_s( NULL, ";", &pCurPos  );
				if( pDuration )
				{
					string durationStr = pDuration;
					double duration = atof( durationStr.c_str() );

					m_pI->m_laneChangeDurationCount =
										static_cast<int>(duration / GetTimeStepDuration());
				}
				else
				{
					m_pI->m_laneChangeDurationCount = -1;
				}

				//
				// Setting the Lane Change Direction from the Dial
				//
				if( lcDirection == 0 )
				{
					m_pI->m_leftLaneChangeButton = true;
				}
				else if( lcDirection == 1 )
				{
					m_pI->m_rightLaneChangeButton = true;
				}

				//
				// Setting the Urgency and turn Signal Time read from the dial
				//
				m_pI->m_laneChangeUrgency = urgency;
				m_pI->m_signalTimeFromDial = signalTime;
				m_pI->m_hasSignalTimeFromDial = true;

#if 1
				if( m_pRootCollection->m_verbose )
				{
					gout << MessagePrefix();
					gout << "LaneChange dial setting:" << endl;
					gout << "  direction   = ";
					if( lcDirection == 0 )
					{
						gout << "left" << endl;
					}
					else
					{
						gout << "right" << endl;
					}
					gout << "  urgency     = " << m_pI->m_laneChangeUrgency << endl;
					gout << "  signal time = " << m_pI->m_signalTimeFromDial << endl;
					gout << "  duration    = ";
					gout << m_pI->m_laneChangeDurationCount * GetTimeStepDuration();
					gout << " s" << endl;
					gout << endl;
				}
#endif
			}
			else
			{
				//
				// Invalid Signal Time
				//
				gout << MessagePrefix();
				gout << "LaneChange dial has invalid signal time = ";
				gout << token << endl;
			}
		}
		else
		{
			//
			// Invalid urgency.
			//
			gout << MessagePrefix();
			gout << "LaneChange dial has invalid urgency = ";
			gout << token << endl;
		}
	}
	else
	{
		//
		// Invalid direction.
		//
		gout << MessagePrefix();
		gout << "LaneChange dial has invalid direction = ";
		gout << token << endl;
	}


}  // end of ParseLaneChangeDial


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Reads the ADO's buttons and dials inside its
//   PreActivity function.
//
// Remarks:  This function should be called once an ADO has been activated.
//
// Arguments:
//   initVel = If > 0, the initial velocity assigned to me in m/s.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::PreActivityProcessButtonsAndDials( const double& cInitVel )
{
	//
	// Process lane change buttons.
	//
    if (m_pI->m_reporjectAndResetLaneOffsetButton){
        auto pos = m_pI->m_roadPos.GetBestXYZ();
        m_pI->m_roadPos.SetXYZ(pos);
        m_pI->SyncRoadPosVars();
        m_dialForcedLaneOffset.Reset();
        m_dialForcedLaneOffset.SetNoValue();
        m_dialLaneChange.SetNoValue();
    }
	m_pI->m_leftLaneChangeButton = false;
	m_pI->m_rightLaneChangeButton = false;
    m_pI->m_reporjectAndResetLaneOffsetButton = false;
	if( GetButtonChangeLaneLeft() )
	{
		//
		// Add an entry to the activity log for button setting.
		//
		m_pRootCollection->SetButtonSettingLog( this, "ChangeLaneLeft" );

		CLane curLane = m_pI->m_roadPos.GetLane();
		bool haveLane = !curLane.IsLeftMostAlongDir();
		if( haveLane )  m_pI->m_leftLaneChangeButton = true;
	}

	if( GetButtonChangeLaneRight() )
	{
		//
		// Add an entry to the activity log for button setting.
		//
		m_pRootCollection->SetButtonSettingLog( this, "ChangeLaneRight" );
		CLane curLane = m_pI->m_roadPos.GetLane();
		bool haveLane = !( curLane.IsRightMost() );
		if( haveLane )  m_pI->m_rightLaneChangeButton = true;
	}

	if( GetButtonProjectAndResetLaneOffset() )
	{
		//
		// Add an entry to the activity log for button setting.
		//
        auto pos = m_pI->m_roadPos.GetBestXYZ();
        m_pI->m_roadPos.SetXYZ(pos);
        m_pI->SyncRoadPosVars();
        m_dialLaneChange.SetNoValue();
        m_dialForcedLaneOffset.Reset();
        m_pI->m_reporjectAndResetLaneOffsetButton = true;
        m_pI->m_haveForcedLaneOffset = false;
        m_pI->m_prevForcedLaneOffset = 0;
        m_pI->m_forcedLaneOffset = 0;
	}

	//
	// Process LaneChange dial.
	//
	if( m_dialLaneChange.HasValue() )
	{
		bool noLaneChangeInProgress = m_pI->m_laneChangeUrgency < 0;
		if( noLaneChangeInProgress )
		{
			//
			// Parse dial for new lane change settings.
			//
			ParseLaneChangeDial();
			m_pI->m_laneChangeWaitCount = 3;
		}
		else if( m_pI->m_laneChangeWaitCount > 0 )
		{
			//
			// After requesting a lane change, wait for some frames before
			// checking to see if it's over.  We do this because it takes
			// the lane change algorithm a couple of frames to get the
			// lane change started.  If we start checking right away, it
			// will report itself as being done because it hasn't had a
			// chance to start it yet.
			//
			m_pI->m_laneChangeWaitCount--;
		}
		else if( m_pI->m_laneChangeWaitCount == 0 )
		{
			if( m_pI->m_laneChangeDurationCount == 0 )
			{
				//
				// The duration has expired.  Set the dial to no value.
				//
				m_dialLaneChange.SetNoValue();
				m_pI->m_laneChangeWaitCount = -1;
				m_pI->m_laneChangeUrgency = -1;
				m_pI->m_hasSignalTimeFromDial = false;
				m_pI->m_laneChangeDurationCount = -1;
			}
			else if( m_pI->m_laneChangeDurationCount > 0 )
			{
				m_pI->m_laneChangeDurationCount--;
			}

			//
			// Lane change currently in progress.  Check to see if it's
			// finished.
			//
			if( m_pI->m_pCurrLcCond == NULL )
			{
				// lane change complete
				m_dialLaneChange.SetNoValue();
				m_pI->m_laneChangeWaitCount = -1;
				m_pI->m_laneChangeUrgency = -1;
				m_pI->m_hasSignalTimeFromDial = false;
				m_pI->m_laneChangeDurationCount = -1;
			}
		}
	}

	//
	// Process turning buttons.
	//
	if( GetButtonTurnLeft() )
	{
		//
		// Add an entry to the activity log for button setting.
		//
		m_pRootCollection->SetButtonSettingLog( this, "TurnLeft" );

		//
		// Look at my path and see if needs to be changed.
		//
		CLane nextLane;
		bool haveLaneToLeft = m_pI->m_pPath->GetNextLaneClockwise( nextLane );
		if( haveLaneToLeft )
		{
			//
			// Figure out if we need to change the path.
			//
			bool needToChangePath = (
						!m_pI->m_pPath->IsNextRoad( nextLane.GetRoad() )
						);
			if( needToChangePath )
			{
				//
				// Build a new path with this lane.
				//
				BuildPath( nextLane );

				//
				// Refresh the path.
				//
				ExtendPath( m_pI );
				m_pI->ResetPathRefreshCounter();

				//
				// Refresh the curvature.
				//
				if( !m_pRootCollection->m_sDisableCurvature )
				{
					m_pI->m_curvature.InitializeCurvature(
										m_pI->m_roadPos,
										*m_pI->m_pPath
										);
					m_pI->m_curvature.ResetBuckets();
					m_pI->m_curvature.UpdateBuckets(
										m_pI->m_roadPos,
										m_pI->m_pObj->GetVelImm(),
										m_pI->m_objLength * 0.5 * cFEET_TO_METER
										);
				}

				ComputeIntersectionTurnSignal();

			}  // end if( needToChangePath )
		}  // end if( haveLaneToLeft )
	}  // if button for left turns

	if( GetButtonTurnRight() )
	{
		//
		// Add an entry to the activity log for button setting.
		//
		m_pRootCollection->SetButtonSettingLog( this, "TurnRight" );

		//
		// Get the next lane to the right after the upcoming intersection.
		//
		CLane nextLane;
		bool haveLaneToRight = (
					m_pI->m_pPath->GetNextLaneCounterClockwise( nextLane )
					);
		if( haveLaneToRight )
		{
			//
			// Figure out if we need to change the path.
			//
			bool needToChangePath = (
						!m_pI->m_pPath->IsNextRoad( nextLane.GetRoad() )
						);
			if( needToChangePath )
			{
				//
				// Build a new path with this lane.
				//
				BuildPath( nextLane );

				//
				// Refresh the path.
				//
				ExtendPath( m_pI );
				m_pI->ResetPathRefreshCounter();

				//
				// Refresh the curvature.
				//
				if( !m_pRootCollection->m_sDisableCurvature )
				{
					m_pI->m_curvature.InitializeCurvature(
										m_pI->m_roadPos,
										*m_pI->m_pPath
										);
					m_pI->m_curvature.ResetBuckets();
					m_pI->m_curvature.UpdateBuckets(
										m_pI->m_roadPos,
										m_pI->m_pObj->GetVelImm(),
										m_pI->m_objLength * 0.5 * cFEET_TO_METER
										);
				}

				ComputeIntersectionTurnSignal();
			}  // end if( needToChangePath )
		}  // end if( haveLaneToRight )
	}  // if button for right turns

	//
	// Read TargetVelocity dial.
	//
	if( m_pI->m_velCntrl.targetVel < 0.0 && m_dialTargetVelocity.HasValue() )
	{
		ParseTargetVelocityDial();
	}
	else if( m_dialTargetVelocity.HasBeenReset() )
	{
		m_dialTargetVelocity.SetNoValue();
		m_pI->m_velCntrl.targetVel              = -1.0;
		m_pI->m_velCntrl.targetVelDurationCount = -1;
	}
	else if( m_pI->m_velCntrl.targetVelDurationCount == -2 )
	{
		m_dialTargetVelocity.SetNoValue();
		m_pI->m_velCntrl.targetVelDurationCount = -1;
	}
	else if( m_pI->m_velCntrl.targetVelDurationCount > 0 )
	{
		//
		// Waiting for duration for expire.
		//
		m_pI->m_velCntrl.targetVelDurationCount--;
	}
	else if( m_pI->m_velCntrl.targetVelDurationCount == 0 )
	{
		//
		// Duration expired..reset target velocity setting.
		//
		m_dialTargetVelocity.SetNoValue();
		m_pI->m_velCntrl.targetVel              = -1.0;
		m_pI->m_velCntrl.targetVelDurationCount = -1;
	}

	//
	// Process MaintainGap dial.
	//
	if( !m_pI->m_maintainGap.IsActive() && m_dialMaintainGap.HasValue() )
	{
			ParseMaintainGapDial();
	}
	// See if we have any expression for maintain gap that need updated
	else if( m_pI->m_maintainGap.UsingExpr() && m_dialMaintainGap.HasValue() ){
		//if we have an expression for the distance value -evaluate it
		if ( !m_exprGapDisEvtal.IsEmpty()){
			double mgValue;
			mgValue = m_exprGapDisEvtal.Evaluate();
			if( !m_pI->m_maintainGap.m_distMode ){
						// convert from time to dist..the time is specified
						// taking into the account the mg obj's velocity at
						// the time the dial is initially set
				const CDynObj* pObj = cved->BindObjIdToClass( m_pI->m_maintainGap.m_objId  );
				if( !pObj || !pObj->IsValid() )
				{
					gout << MessagePrefix();
					gout << "MaintainGap unable to get pointer to obj = ";
					gout << m_pI->m_maintainGap.m_objId  << endl;

					m_pI->m_maintainGap.Reset();
					return;
				}

				double objVel = pObj->GetVelImm() * cMETER_TO_FEET;
				mgValue = mgValue * objVel;
				const double cMIN_MG_DIST = 20.0;
				if( fabs( mgValue ) < cMIN_MG_DIST )
				{
					if( mgValue < 0.0 )
					{
						mgValue = -1.0 * cMIN_MG_DIST;
					}
					else
					{
						mgValue = cMIN_MG_DIST;
					}
				}
			}
			m_pI->m_maintainGap.m_value = mgValue;
		}
		if( !m_exprMinSpeedEvtal.IsEmpty()){
			m_pI->m_maintainGap.m_minSpeed = m_exprMinSpeedEvtal.Evaluate() *cMPH_TO_MS;
		}
		if( !m_exprMaxSpeedEvtal.IsEmpty()){
			m_pI->m_maintainGap.m_maxSpeed = m_exprMaxSpeedEvtal.Evaluate() *cMPH_TO_MS;
		}
		if( !m_exprMinAccelEvtal.IsEmpty()){
			m_pI->m_maintainGap.m_maxDecel = m_exprMinAccelEvtal.Evaluate();
		}
		if( !m_exprMaxAccelEvtal.IsEmpty()){
			m_pI->m_maintainGap.m_maxAccel = m_exprMaxAccelEvtal.Evaluate();
		}
	}
	else if( m_dialMaintainGap.HasBeenReset() )
	{
		m_pI->m_maintainGap.Reset();
#if 0
		gout << MessagePrefix();
		gout << "Reset MaintainGap" << endl;
		gout << endl;
#endif
	}

	//
	// Process LaneChangeStatus dial.
	//
	if( m_pI->m_lcStatus == cLC_STATUS_DO_LC && m_dialLaneChangeStatus.HasValue() )
	{
		ParseLaneChangeStatusDial();
	}
	else if( m_dialLaneChangeStatus.HasBeenReset() )
	{
		// externally reset....set dial to novalue
		m_dialLaneChangeStatus.SetNoValue();
		m_pI->m_lcStatus = cLC_STATUS_DO_LC;
	}
	else if( m_pI->m_lcStatusCount > 0 )
	{
		//
		// Waiting for duration for expire.
		//
		m_pI->m_lcStatusCount--;
	}
	else if( m_pI->m_lcStatusCount == 0 )
	{
		//
		// Duration expired..reset lane change status setting.
		//
		m_dialLaneChangeStatus.SetNoValue();
		m_pI->m_lcStatus = cLC_STATUS_DO_LC;
	}

#ifdef SCEN_EXTERNAL_DRIVER
	if( m_pI->m_pObj->GetId() == 0 )
	{
		m_pI->m_lcStatus = cLC_STATUS_NO_LC;
	}
#endif

	//
	// Process LaneChangeInhibit dial.  Before doing that, decrement the
	// counter if it already exists.
	//
	if( m_pI->m_lcInhibitCount < 0 && m_dialInhibitLaneChange.HasValue() )
	{
		m_pI->m_lcInhibitCount =
						static_cast<int>(GetDialInhibitLaneChange() / GetTimeStepDuration());

#if 1
		if( m_pRootCollection->m_verbose )
		{
			gout << MessagePrefix();
			gout << "InhibitLaneChange dial setting = ";
			gout << GetDialInhibitLaneChange() << " s" << endl;
			gout << endl;
		}
#endif
	}
	else if( m_dialInhibitLaneChange.HasBeenReset() )
	{
		// externally reset....set dial to novalue
		m_dialInhibitLaneChange.SetNoValue();
		m_pI->m_lcInhibitCount = -1;
	}
	else if( m_pI->m_lcInhibitCount > 0 )
	{
		// waiting for inhibit counter to expire
		m_pI->m_lcInhibitCount--;
	}
	else if( m_pI->m_lcInhibitCount == 0 )
	{
		// counter expired....set myself to novalue
		m_dialInhibitLaneChange.SetNoValue();
		m_pI->m_lcInhibitCount = -1;
	}

	//
	// Process ImStop dial.
	//
	m_pI->m_imStopHasValue = m_dialImStop.HasValue();
	if( m_dialImStop.HasValue() )
	{
		m_pI->m_imStopHldOfsDist = GetDialImStop();
	}

	//
	// Process the ForcedLaneOffset dial
	//
	if( m_dialForcedLaneOffset.HasValue() )
	{
		ParseForcedLaneOffset();
		if( m_pI->m_haveForcedLaneOffset )
		{
			// Have a value from before.

			double diff = m_pI->m_forcedLaneOffset - m_prevForcedLaneDialVal;
			bool haveNewValue = fabs( diff ) > 0.1;
			if( haveNewValue )
			{
				m_prevForcedLaneDialVal = m_pI->m_forcedLaneOffset;
				m_pI->m_forcedOffsetFrameCounter = 0;
			}
		}
		else
		{
			// No value from before.

			m_pI->m_haveForcedLaneOffset = true;
			m_prevForcedLaneDialVal = 0.0;
			m_pI->m_forcedOffsetFrameCounter = 0;
            m_pI->m_prevForcedLaneOffset = 0;
		}
	}
	else
	{
		m_pI->m_haveForcedLaneOffset = false;
	}
    if ( m_dialForcedLaneOffset.HasBeenReset()){

    }
}  // end of PreActivityProcessButtonsAndDials



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Reads the ADO's buttons and dials inside its
//   PostActivity function.
//
// Remarks:  This function should be called once an ADO has been activated.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::PostActivityProcessButtonsAndDials()
{
	//
	// Setting the visual or audio dials overrides all internal
	// requests.
	//
	bool haveValidObj = m_pI->m_pObj && m_pI->m_pObj->IsValid();
	if( haveValidObj )
	{
		CVehicleObj* pVehObj = dynamic_cast<CVehicleObj*>( m_pI->m_pObj );

		//
		// Processing the Visual State dial.
		//
		if( m_pI->m_visualStateFromDial < 0 && m_dialVisualState.HasValue() )
		{
			ParseVisualStateDial();
			pVehObj->SetVisualState( m_pI->m_visualStateFromDial );

		}
		else if( m_dialVisualState.HasBeenReset() )
		{
			TU16b visState = pVehObj->GetVisualState();
			if( visState & m_pI->m_visualStateFromDial )
			{
				visState &= ~m_pI->m_visualStateFromDial;
				m_pI->m_visualStateFromDial &= ~m_pI->m_visualStateFromDial;
			}
			pVehObj->SetVisualState( visState );

			m_dialVisualState.SetNoValue();
			m_pI->m_visualStateFromDial = -1;
			m_pI->m_visualStateFromDialCount = -1;
		}
		else if( m_pI->m_visualStateFromDialCount == -2 )
		{
			pVehObj->SetVisualState( m_pI->m_visualStateFromDial );
			m_dialVisualState.SetNoValue();
			m_pI->m_visualStateFromDialCount = -1;
		}
		else if( m_pI->m_visualStateFromDialCount > 0 )
		{
			m_pI->m_visualStateFromDialCount--;
			pVehObj->SetVisualState( m_pI->m_visualStateFromDial );
		}
		else if( m_pI->m_visualStateFromDialCount == 0 )
		{
			TU16b visState = pVehObj->GetVisualState();
			if( visState & m_pI->m_visualStateFromDial )
			{
				visState &= ~m_pI->m_visualStateFromDial;
				m_pI->m_visualStateFromDial &= ~m_pI->m_visualStateFromDial;
			}
			pVehObj->SetVisualState( visState );

			m_dialVisualState.SetNoValue();
			m_pI->m_visualStateFromDial = -1;
			m_pI->m_visualStateFromDialCount = -1;
		}
		else if( m_pI->m_visualStateFromDial >= 0 )
		{
			pVehObj->SetVisualState( m_pI->m_visualStateFromDial );
		}

		//
		// Processing the Audio State dial.
		//
		if( m_pI->m_audioStateFromDial < 0 && m_dialAudioState.HasValue() )
		{
			ParseAudioStateDial();
			pVehObj->SetAudioState( m_pI->m_audioStateFromDial );
		}
		else if( m_dialAudioState.HasBeenReset() )
		{
			TU16b audioState = pVehObj->GetAudioState();
			if( audioState & m_pI->m_audioStateFromDial )
			{
				audioState &= ~m_pI->m_audioStateFromDial;
			}
			pVehObj->SetAudioState( audioState );

			m_dialAudioState.SetNoValue();
			m_pI->m_audioStateFromDial = -1;
			m_pI->m_audioStateFromDialCount = -1;
		}
		else if( m_pI->m_audioStateFromDialCount > 0 )
		{
			m_pI->m_audioStateFromDialCount--;
			pVehObj->SetAudioState( m_pI->m_audioStateFromDial );
		}
		else if( m_pI->m_audioStateFromDialCount == 0 )
		{
			TU16b audioState = pVehObj->GetAudioState();
			if( audioState & m_pI->m_audioStateFromDial )
			{
				audioState &= ~m_pI->m_audioStateFromDial;
			}
			pVehObj->SetAudioState( audioState );

			m_dialAudioState.SetNoValue();
			m_pI->m_audioStateFromDial = -1;
			m_pI->m_audioStateFromDialCount = -1;
		}
		else if( m_pI->m_audioStateFromDial >= 0 )
		{
			pVehObj->SetAudioState( m_pI->m_audioStateFromDial );
		}
	}
}  // end of PostActivityProcessButtonsAndDials



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Clears a vehicle's turn signals.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ClearTurnSignals()
{
	m_pI->m_lightState &= ~cCV_LEFT_TURN_SIGNAL;
	m_pI->m_lightState &= ~cCV_RIGHT_TURN_SIGNAL;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the turn signal based on the turn
//   through the next intersection.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::ComputeIntersectionTurnSignal()
{
	//
	// Don't compute turn signals inside intersections.  They should
	// already be set properly before entering an intersection.
	//
	if( !m_pI->m_roadPos.IsRoad() )  return;

	const double cLEFT_TURN_ANGLE_MIN = 45.0 * cDEG_TO_RAD;
	const double cLEFT_TURN_ANGLE_MAX = 135.0 * cDEG_TO_RAD;
	const double cRIGHT_TURN_ANGLE_MIN = 235.0 * cDEG_TO_RAD;
	const double cRIGHT_TURN_ANGLE_MAX = 315.0 * cDEG_TO_RAD;

	m_pI->m_nextTurnSignal = 0;

	//
	// Don't set turn signals for two road intersections.
	//
	bool twoRoadIntrsctn = m_pI->m_pPath->IsIntrsctnTwoRoad( m_pI->m_roadPos );
	if( twoRoadIntrsctn )  return;

	double turnAngle = m_pI->m_pPath->GetNextTurnAngle( m_pI->m_roadPos );
	bool signalRightTurn = (
				turnAngle >= cRIGHT_TURN_ANGLE_MIN &&
				turnAngle <= cRIGHT_TURN_ANGLE_MAX
				);
	bool signalLeftTurn = (
				turnAngle >= cLEFT_TURN_ANGLE_MIN &&
				turnAngle <= cLEFT_TURN_ANGLE_MAX
				);
	if( signalRightTurn )
	{
		m_pI->m_nextTurnSignal |= cCV_RIGHT_TURN_SIGNAL;
	}
	else if( signalLeftTurn )
	{
		m_pI->m_nextTurnSignal |= cCV_LEFT_TURN_SIGNAL;
	}
}  // end of ComputeIntersectionTurnSignal



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets a vehicle's turn signals for upcoming turns at
//   intersections.
//
// Remarks:  Does not set any turn signals if they are already set.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::SetTurnSignals()
{

	bool isLeftTurnSignalOn  = m_pI->m_lightState & cCV_LEFT_TURN_SIGNAL;
	bool isRightTurnSignalOn = m_pI->m_lightState & cCV_RIGHT_TURN_SIGNAL;

	//
	// Attempt to set turn signals only if they aren't already on.
	//
	bool noTurnSignalActive = !isLeftTurnSignalOn && !isRightTurnSignalOn;
	if( noTurnSignalActive )
	{
		double distToIntersection = m_pI->m_pPath->GetDistToNextIntrsctn(
													m_pI->m_roadPos
													);

		const double cSIGNAL_DIST = 100.0;   // feet
		bool withinRange = distToIntersection < cSIGNAL_DIST;
		if( withinRange )
		{
			m_pI->m_lightState |= m_pI->m_nextTurnSignal;
		}
	}

	CVehicleObj* pVehObj = dynamic_cast<CVehicleObj *>( m_pI->m_pObj );

	//
	// Set visual state for now.  This may be overwritten later if the
	// VisualState dial has been assigned a value.
	//
	TU16b visState = pVehObj->GetVisualStateImm();
	if( m_pI->m_lightState & cCV_LEFT_TURN_SIGNAL )
	{
		visState |= cCV_LEFT_TURN_SIGNAL;
	}
	else
	{
		visState &= ~cCV_LEFT_TURN_SIGNAL;
	}
	if( m_pI->m_lightState & cCV_RIGHT_TURN_SIGNAL )
	{
		visState |= cCV_RIGHT_TURN_SIGNAL;
	}
	else
	{
		visState &= ~cCV_RIGHT_TURN_SIGNAL;
	}
	pVehObj->SetVisualState( visState );

}  // end of SetTurnSignals


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Displays debugging information.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::DisplayDebugInfo()
{

	//
	// Debugging stuff.
	//
	char temp[255];
	string str;

	if (m_pI->m_pObj == NULL) return;

	sprintf( temp, "%d", m_pI->m_pObj->GetId() );
	str = temp;
	DebugGraphicalText(
				CHcsmDebugItem::eDEBUG_ROUTINE,
				str,
				*m_pI->m_pObj,
				CHcsmDebugItem::eWHITE,
				CHcsmDebugItem::eTOP_LEFT
				);
	sprintf( temp, "%.1f", m_pI->m_pObj->GetVelImm() * cMS_TO_MPH );
	str = temp;
	DebugGraphicalText(
				CHcsmDebugItem::eDEBUG_ROUTINE,
				str,
				*m_pI->m_pObj,
				CHcsmDebugItem::eWHITE,
				CHcsmDebugItem::eTOP_RIGHT
				);
	str = m_pI->m_acgoutHcsmText;
	DebugGraphicalText(
				CHcsmDebugItem::eDEBUG_ROUTINE,
				str,
				*m_pI->m_pObj,
				CHcsmDebugItem::eWHITE,
				CHcsmDebugItem::eBOTTOM_LEFT
				);


	//
	// Draw a circle at the end of path.  Make sure that the path is
	// valid and is not empty.
	//
	bool haveValidPath = m_pI->m_pPath->IsValid() && !m_pI->m_pPath->IsEmpty();
	if( haveValidPath )
	{
		CRoadPos pathEnd = m_pI->m_pPath->GetEnd();
		if( pathEnd.IsValid() )
		{
			DebugCircle(
						CHcsmDebugItem::eDEBUG_ROUTINE,
						pathEnd.GetXYZ(),
						10.0,
						CHcsmDebugItem::eRED,
						true
						);
		}
	}


	//
	// Display merge debug info.
	//
	vector<TMergeInfo>::iterator i;
	for( i = m_pI->m_mergeInfo.begin(); i != m_pI->m_mergeInfo.end(); i++ )
	{
		if( (*i).leadObjId != -1 )
		{
			DebugCircle(
				CHcsmDebugItem::eDEBUG_ROUTINE,
				i->pos,
				i->radius,
				CHcsmDebugItem::eRED,
				false
				);
		}
		else
		{
			DebugCircle(
				CHcsmDebugItem::eDEBUG_ROUTINE,
				i->pos,
				i->radius,
				CHcsmDebugItem::eGREEN,
				false
				);
		}
	}



}  // end of DisplayDebugInfo


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Performs the operations for an ADO when it's activated.
//
// Remarks:  This function should be called once an ADO has been activated.
//
// Arguments:
//   cpSnoBlock - The Ado's SNO block.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CAdo::Activated( const CAdoParseBlock* cpSnoBlock )
{
	static int cnt =0;
	double temp1;
	double temp2;

	m_pRootCollection->MemLog( 0, HLOG_ADO_ACTIVATED_IN, 0 );

#ifdef DEBUG_EXTERNAL_DRIVER_SPEED
	if (CHcsmCollection::m_ExternalDriverRehearsalControl && m_name == "ExternalDriver"){
		m_pI->m_velCntrl.targetVel = CHcsmCollection::m_sOwnVehicleSpeed;
		if ( CHcsmCollection::m_sMakeOwnVehicleChangeLanesLeft){
			m_dialLaneChange.SetValue("left;0.9;0.1");
		}
		if ( CHcsmCollection::m_sMakeOwnVehicleChangeLanesRight){
			m_dialLaneChange.SetValue("right;0.9;0.1");
		}
	}


	if (m_name == "ExternalDriver"){
		m_pI->m_velCntrl.targetVel = gEXTERNAL_DRIVER_SPEED ;
		if (0)
			m_dialLaneChange.SetValue("right;0.9;0.1");
		if (0)
			m_dialLaneChange.SetValue("left;0.9;0.1");
        if (0)
            m_dialForcedLaneOffset.SetValue("-12;1.5;500");
        if (0)
            m_dialForcedLaneOffset.SetValue("0");
		if (0){
			m_pI->m_aggressiveness = 1.0;
			m_pI->m_FollowParams.ttcThres1 = 1.0;
			m_pI->m_FollowParams.ttcThres2 = 2.0;
			m_pI->m_FollowParams.ttcThres3 = 3.0;
			m_pI->m_FollowParams.engageThres = 5;
			m_pI->m_FollowParams.useReactDelay = false;
		}
        if (0){
            m_dialForcedVelocity.SetValue("62");
        }
		if (0){
            m_pI->m_lcInfo.enableAvoidMergingObject = false;
            m_pI->m_lcInfo.enableSlowMovingObject = false;
            m_pI->m_lcInfo.enableVerySlowMovingObject = false;
            m_pI->m_lcInfo.enableNonPassingLane = false;
        }


	}
#endif

#ifdef DEBUG_GRAPHIC_DEBUG
	bool debugThisObj = m_pI->m_pObj->GetId() == DEBUG_GRAPHIC_DEBUG;
#endif

	DisplayDebugInfo();

	//
	// Check to see if the vehicle dynamics has seen the ADO off-road
	// for too long.
	//
	const int cQRY_TERRAIN_OFFROAD_MAX_COUNT = 6;
	CVehicleObj* pVehObj = dynamic_cast<CVehicleObj *>( m_pI->m_pObj );
	bool offroadTooLong =
		pVehObj->GetQryTerrainErrCount() > cQRY_TERRAIN_OFFROAD_MAX_COUNT;
	if( offroadTooLong )
	{
		gout << MessagePrefix();
		gout << "this vehicle has been off-road for too long...[SUICIDE]";
		gout << endl;
	}

	m_pI->m_prevVel = m_pI->m_currVel;
	m_pI->m_currVel = m_pI->m_pObj->GetVelImm();

	//
	// Get current position from CVED and subtract the vehicle CG in
	// the z-axis.
	//
	CPoint3D cartPos = m_pI->m_pObj->GetPosImm();
	bool lockOnForcedOffset;
	if( m_pI->m_haveForcedLaneOffset )
	{
		//
		// Obtain the lateral vector and scale it properly to obtain
		// the position which reflects the center of the lane.  This
		// position will be used to compute the correct RoadPos.  The
		// offset due to the ForcedLaneOffset dial will be applied
		// afterwards.  This method keeps the RoadPos from snapping
		// onto a different lane.
		//
		double offsetDiff = m_pI->m_forcedLaneOffset - m_currForcedLaneOffset;
		bool noLcInProgress = m_pI->m_pCurrLcCond == NULL;
		lockOnForcedOffset = fabs( offsetDiff ) < 1.0 && noLcInProgress;
		double scaleVal;
		if( lockOnForcedOffset )
		{
			scaleVal = m_pI->m_forcedLaneOffset;
		}
		else
		{
			scaleVal = m_currForcedLaneOffset;
		}
		CVector3D lat = m_pI->m_roadPos.GetRightVec();
		lat.Scale( scaleVal );
		cartPos.m_x -= lat.m_i;
		cartPos.m_y -= lat.m_j;
		cartPos.m_z -= lat.m_k;
#if 0
		gout << "=====currForcedLaneOffset = " << m_currForcedLaneOffset << endl;
		gout << "**offsetDiff         = " << offsetDiff << endl;
		gout << "**lcInProgress       = " << !noLcInProgress << endl;
		gout << "**lockOnForcedOffset = " << lockOnForcedOffset << endl;
		gout << "lat        = " << lat << endl;
		gout << "lat scaled = " << lat << endl;
		gout << " AFTER = " << cartPos << endl;
#endif
	}

	if( !m_pI->m_offroad )
	{
		//
		// Update RLD with the latest position.  Using SetXYZ
		// to allows the new RLD faster as it assumes that the new
		// position should be close to the old position.
		//
		if( m_crdr.IsValid() )
		{
			//
			// Inside an intersection.
			//
			m_pI->m_prevRoadPos = m_pI->m_roadPos;
//	gout << "1:" << MessagePrefix( m_pI->m_pObj->GetId(), m_pI->m_objName ) << m_pI->m_roadPos << endl;
			CCrdr curCrdr = m_pI->m_pPath->GetNextCrdr( m_pI->m_roadPos );
//			if( GetFrame() >= 112 )
//				gout << "curCrdr = " << curCrdr.GetString() << endl;
            CRoadPos tempPos(*cved,cartPos); //force a clean update, we need to match all the crdrs up we are on
            m_pI->m_roadPos = tempPos;       //this should be fixed so we don't have to do this
            m_pI->m_roadPos.SetCrdPriority(&curCrdr);
//	gout << "2:" << MessagePrefix( m_pI->m_pObj->GetId(), m_pI->m_objName ) << m_pI->m_roadPos << endl;
			if( !m_pI->m_roadPos.IsValid() )
			{
				m_pI->m_roadPos.SetXYZ( cartPos );
			}

//	gout << "3:" << MessagePrefix( m_pI->m_pObj->GetId(), m_pI->m_objName ) << m_pI->m_roadPos << endl;
			if( m_pI->m_roadPos.IsRoad() )
			{
				// on a road now, reset current corridor
				CCrdr blankCrdr;
				m_crdr = blankCrdr;
				m_pI->m_pastLcCond = eLC_NONE;
			}
		}
		else
		{
			//
			// On a road.
			//
			m_pI->m_prevRoadPos = m_pI->m_roadPos;
			CRoadPos tempRoadPos = m_pI->m_roadPos;
			tempRoadPos.SetXYZ( cartPos );
			if( !tempRoadPos.IsValid() )
			{
				gout << MessagePrefix();
				gout << "SetXYZ fails for cartPos = " << cartPos;
				gout << "  [SUICIDE]" << endl;
				gout << "  objName = " << m_pI->m_objName << endl;

				Suicide();
				return;
			}

			if( !tempRoadPos.IsRoad() )
			{
				//
				// Inside an intersection....make sure that it's the
				// right corridor in accordance with my path.
				//
				m_crdr = m_pI->m_pPath->GetNextCrdr( m_pI->m_roadPos );
                if (!m_crdr.IsValid()){
				    gout << MessagePrefix();
				    gout << "SetXYZ fails for cartPos = " << cartPos;
				    gout << "  [SUICIDE]" << endl;
				    gout << "  objName = " << m_pI->m_objName << endl;
                    if (!m_pI->m_pPath->Contains(m_pI->m_roadPos))
					{
                       gout << "ADO is not in the current path" << endl;
                    }
                    gout << " ADO cannot find the next Crdr for this postion in its path"<<endl;

				    Suicide();
                    return;
                }
				m_pI->m_roadPos = tempRoadPos;
                m_pI->m_roadPos.SetCrdPriority( &m_crdr);
			}
			else
			{
				m_pI->SetCurrRoadPos( tempRoadPos );
			}
		}

		//
		// This part of the code is a continuation from code placed before
		// the current RoadPos is computed.  This code deal with when the
		// ADO has a ForcedLaneOffset setting.
		//
		if( m_pI->m_haveForcedLaneOffset )
		{
			double newOffset;
			if( !lockOnForcedOffset )
			{
				double currOffset = m_pI->m_roadPos.GetOffset();
				newOffset = currOffset + m_currForcedLaneOffset;
			}
			else
			{
				newOffset = m_currForcedLaneOffset;
			}
			m_pI->m_roadPos.SetOffset( newOffset );
			m_currForcedLaneOffset = newOffset;

#if 0
			gout << "setting roadPos offset = " << newOffset << endl;
			gout << "**setting currForcedLaneOffset = ";
			gout << m_currForcedLaneOffset << endl;
#endif
		}

		if( !m_pI->m_roadPos.IsValid() )
		{
			gout << "BH[" << GetFrame() << "]: object (name=";
			gout << m_pI->m_objName << ") has invalid roadPos [SUICIDE]";
			gout << endl;
			CPoint3D cartPos = m_pI->m_pObj->GetPos();
			gout << "  currPos = " << cartPos << endl;

			Suicide();
		}

		//
		// Refresh my path?
		//
		if ( m_pI->IsExpiredPathRefreshCounter() ) {

//			m_pRootCollection->MemLog( 0, HLOG_ADO_CURV1, 0 );
			ExtendPath( m_pI );
			m_pI->ResetPathRefreshCounter();

			if( !m_pRootCollection->m_sDisableCurvature )
			{
//				m_pRootCollection->MemLog( 0, HLOG_ADO_CURV2, 0 );
				m_pI->m_curvature.RefreshCurvature(
										m_pI->m_roadPos,
										*m_pI->m_pPath,
										GetFrame()
										);
//				m_pRootCollection->MemLog( 0, HLOG_ADO_CURV3, 0 );
				m_pI->m_curvature.ResetBuckets();
				m_pI->m_curvature.UpdateBuckets(
										m_pI->m_roadPos,
										m_pI->m_pObj->GetVelImm(),
										m_pI->m_objLength * 0.5 * cFEET_TO_METER
										);
//				m_pRootCollection->MemLog( 0, HLOG_ADO_CURV4, 0 );
			}

			ComputeIntersectionTurnSignal();

#ifdef DEBUG_CURVATURE
			if ( GetFrame() >= DEBUG_CURVATURE ) {
				gout << MessagePrefix() << "=================" << endl;
				gout << "  startRoadPos = " << m_pI->m_roadPos << endl;

				gout << endl << endl;
				gout << "===" << MessagePrefix() << "  roadPos = ";
				gout << m_pI->m_roadPos << endl;
				m_pI->m_curvature.DebugCurvature();

				gout << "---" << MessagePrefix();
				gout << "  roadPos = " << m_pI->m_roadPos;
				gout << "  vel = " << m_pI->m_pObj->GetVelImm() * cMS_TO_MPH;
				gout << "mph" << endl;
				m_pI->m_curvature.DebugBuckets();
			}
#endif

		}
		else {

			if( !m_pRootCollection->m_sDisableCurvature )
			{
//				m_pRootCollection->MemLog( 0, HLOG_ADO_CURV1, 0 );
				m_pI->m_curvature.UpdateBuckets(
										m_pI->m_roadPos,
										m_pI->m_pObj->GetVelImm(),
										m_pI->m_objLength * 0.5 * cFEET_TO_METER
										);
			}

#ifdef DEBUG_CURVATURE
			if ( GetFrame() >= DEBUG_CURVATURE ) {
				gout << "---" << MessagePrefix();
				gout << "  roadPos = " << m_pI->m_roadPos;
				gout << "  vel = " << m_pI->m_pObj->GetVelImm() << endl;
				m_pI->m_curvature.DebugBuckets();
			}
#endif

		}

//		m_pRootCollection->MemLog( 0, HLOG_ADO_CURV5, 0 );

		//
		// Check to see if my path is ending.
		//

		const double cPATH_END_DIST = 20.0;  // feet
		double pathLength = m_pI->m_pPath->GetLength( &m_pI->m_roadPos );
		bool pathEnding = pathLength < cPATH_END_DIST;

		if( pathEnding )
		{

			//
			// My path is ending...time to kill myself.
			//
			gout << MessagePrefix() << "path is ending...suicide" << endl;
			gout << "CPath::GetLength" << endl;
			gout << "  roadPos = " << m_pI->m_roadPos << endl;
			gout << "  objName = " << m_pI->m_objName << endl;
			gout << *m_pI->m_pPath;
			gout << endl;

			Suicide();
		}

		ClearTurnSignals();
		PreActivityProcessButtonsAndDials(
					m_pI->m_velCntrl.initVel
					);
		if ( !m_pI->m_prevRoadPos.IsRoad() && m_pI->m_roadPos.IsRoad() )
		{
			ComputeIntersectionTurnSignal();

		}

	}  // end if not offroad
	else
	{
		m_pI->m_offroadCartPos = cartPos;
	}  // end else offroad

	CObjTypeMask objMask;
	objMask.Set( eCV_TRAJ_FOLLOWER );
	objMask.Set( eCV_VEHICLE );
	objMask.Set( eCV_TRAILER );
	objMask.Set( eCV_EXTERNAL_TRAILER );

	m_pRootCollection->MemLog( 0, HLOG_ADO_ACT_LISTS_1, 0 );

	m_pI->m_objsFwd.clear();
	cved->BuildFwdObjList(
				m_pI->m_pObj->GetId(),
				m_pI->m_roadPos,
				*m_pI->m_pPath,
				8,
				objMask,
				m_pI->m_objsFwd
				);

#if 0
	vector<CCved::TObjListInfo>::const_iterator i;
	gout << MessagePrefix( m_pI->m_pObj->GetId() ) << "Fwd: ";
	for ( i = m_pI->m_objsFwd.begin(); i != m_pI->m_objsFwd.end(); i++ ) {

		int fwdObjId = i->objId;

#ifdef DEBUG_GRAPHIC_DEBUG
		if( debugThisObj )
		{
			// Show graphic debugging.
			const CDynObj* pObj = cved->BindObjIdToClass( fwdObjId );
			if( !pObj || !pObj->IsValid() )		return;


			CPoint3D objPos = pObj->GetPosImm();
			CPoint3D ownerObjPos = m_pI->m_pObj->GetPosImm();
			double radius = pObj->GetXSize() * 0.5;

			DebugCircle(
					CHcsmDebugItem::eDEBUG_ROUTINE,
					objPos, // position of this object
					radius,  // half of length of this object
					CHcsmDebugItem::eRED,
					false
					);
		}
#endif

		gout << " " << i->objId << "[" << i->distFromOwner << "]";
	}
	gout << endl;
#endif



	m_pI->m_objsBack.clear();
	cved->BuildBackObjList(
				m_pI->m_pObj->GetId(),
				m_pI->m_roadPos,
				8,
				objMask,
				m_pI->m_objsBack
				);
#if 0
	vector<CCved::TObjListInfo>::const_iterator j;
	gout << MessagePrefix( m_pI->m_pObj->GetId() ) << "Bak: ";
	for ( j = m_pI->m_objsBack.begin(); j != m_pI->m_objsBack.end(); j++ ) {

		if( GetFrame() % 10 == 0 )
		{
			gout << " " << j->objId << "[" << j->distFromOwner << "]";
		}
	}
	gout << endl;
#endif



	//
	// Need to pass in the previous road of the roadPos as parameter when calling
	// BuildBackObjList2.
	//
	CRoad prevRoad;
	if( ! m_pI->m_roadPos.IsRoad() )
	{
		prevRoad = m_pI->m_roadPos.GetCorridor().GetSrcLn().GetRoad();
	}
	else
	{
		CIntrsctn currIntrsctn;
		currIntrsctn = m_pI->m_roadPos.GetLane().GetPrevIntrsctn();
		static vector<CCrdr> allCrdrs;
		allCrdrs.clear();
		currIntrsctn.GetAllCrdrs( allCrdrs );
		vector<CCrdr>::iterator itr;
		for( itr = allCrdrs.begin(); itr != allCrdrs.end(); itr++ )
		{
			if( (*itr).GetDstntnLn() == m_pI->m_roadPos.GetLane() )
			{
				CLane srcLane = (*itr).GetSrcLn();
				prevRoad = srcLane.GetRoad();
//				gout << " The corridor is: " << (*itr).GetRelativeId() << endl;
//				gout << " The prevRoad is: " << prevRoad.GetName() << endl;
				break;
			}
		}
	}



	m_pI->m_objsBack2.clear();
	cved->BuildBackObjList2(
				m_pI->m_pObj->GetId(),
				m_pI->m_roadPos,
				prevRoad,
				8,
				objMask,
				m_pI->m_objsBack2
				);


#if 0
	gout << " ado.cxx: size of back obj list = " << m_pI->m_objsBack2.size() << endl;
	vector<CCved::TObjListInfo>::const_iterator n;
	gout << MessagePrefix( m_pI->m_pObj->GetId() ) << "Bak2: ";
	for ( n = m_pI->m_objsBack2.begin(); n != m_pI->m_objsBack2.end(); n++ ) {

		int backObjId = n->objId;

#ifdef DEBUG_GRAPHIC_DEBUG
		if( debugThisObj )
		{
			// Show graphic debugging.
			const CDynObj* pObj = cved->BindObjIdToClass( backObjId );
			if( !pObj || !pObj->IsValid() )		return;

			CPoint3D objPos = pObj->GetPosImm();
			double radius = pObj->GetXSize() * 0.5;


			DebugCircle(
					CHcsmDebugItem::eDEBUG_ROUTINE,
					objPos, // position of this object
					radius,  // half of length of this object
					CHcsmDebugItem::eBLUE,
					false
					);

	//		DebugLine(
	//				CHcsmDebugItem::eDEBUG_ROUTINE,
	//				m_pI->m_pObj->GetPosImm(),  // position of owner object
	//				objPos,  // position of this object
	//				CHcsmDebugItem::eBLUE,
	//				true
	//				);
		}
#endif

	//	if( GetFrame() % 10 == 0 )
	//	{
			gout << " " << n->objId << "[" << n->distFromOwner << "]";
	//	}
	}
	gout << endl;

#endif


	m_pI->m_objsOncom.clear();
	cved->BuildOncomingObjList(
				m_pI->m_pObj->GetId(),
				m_pI->m_roadPos,
				*m_pI->m_pPath,
				8,
				objMask,
				m_pI->m_objsOncom
				);
#if 0
	vector<CCved::TObjListInfo>::const_iterator k;
	gout << MessagePrefix( m_pI->m_pObj->GetId() ) << "Oncoming: ";
	for ( k = m_pI->m_objsOncom.begin(); k != m_pI->m_objsOncom.end(); k++ ) {

//		if( GetFrame() % 10 == 0 )
//		{
			gout << " " << k->objId << "[" << k->distFromOwner << "]";
//		}
	}
	gout << endl;
#endif


	m_pI->m_objsApprch.clear();
	cved->BuildApprchObjList(
				m_pI->m_pObj->GetId(),
				m_pI->m_roadPos,
				*m_pI->m_pPath,
				20,
				objMask,
				m_pI->m_objsApprch
				);
	m_pRootCollection->MemLog(0, HLOG_ADO_ACT_LISTS_4, 0 );

#if 0
	vector<CCved::TObjListInfo>::const_iterator m;
	gout << MessagePrefix( m_pI->m_pObj->GetId() ) << "Apprch: ";
	for ( m = m_pI->m_objsApprch.begin(); m != m_pI->m_objsApprch.end(); m++ ) {

	//	if( GetFrame() % 10 == 0 )
	//	{
			gout << " " << m->objId << "[" << m->distFromOwner << "]";
	//	}
	}
	gout << endl;
#endif


#if 0
CCved::TOwnVehiclePositionInfo positionInfo;
	CCved::TOwnVehicleFollowInfo followInfo;
	gout << " ====== Ado.cxx testing  ========= " << endl;
	if( cved->GetOwnVehicleInfo(
					*m_pI->m_pPath,
					objMask,
					positionInfo,
					followInfo )
					)
	{
		gout << endl;
		gout << " ========= output driver info ================= " << endl;
		gout << " ownVeh driver laneDeviation = " << positionInfo.laneDeviation << endl;
		gout << " ownVeh driver distToLeftEdge = " << positionInfo.distToLeftEdge << endl;
		gout << " ownVeh driver distToRightEdge = " << positionInfo.distToRightEdge << endl;
		gout << " ownVeh driver distToRightmostEdge = " << positionInfo.distToRightmostEdge << endl;
		gout << " follow dist = " << followInfo.followDist << endl;
		gout << " bumper2bumperdist = " << followInfo.bumperToBumperDist << endl;
		gout << " follow time = " << followInfo.followTime << endl;
		gout << " ttc = " << followInfo.ttc << endl;
		gout << " lead veh id = " << ( followInfo.leadObjInfo ).vehicleId << endl;
		gout << " lead veh pos = " << ( followInfo.leadObjInfo ).XYZpoint << endl;
		gout << " lead veh roadPos = " << ( followInfo.leadObjInfo ).roadpos << endl;
		gout << " lead veh vel = " << ( followInfo.leadObjInfo ).velocity << endl;
		gout << " lead veh light state = " << ( followInfo.leadObjInfo ).lightState << endl;

	}
	else
	{
		gout << " no or incomplete driver info is obtained " << endl;
	}
	gout << " ======= end of Ado.cxx testing ====== " << endl;

#endif

	m_pRootCollection->MemLog( 0, HLOG_ADO_ACTIVATED_OUT, 0 );

}  // Activated


void
CAdo::UserPreActivity( const CAdoParseBlock* cpSnoBlock )
{
	//
	// Update my age in frames and time.
	//
	m_pI->m_ageFrame++;
	m_pI->m_pathRefreshCounter--;

	//
	// Get position from previous RLD.
	//
	if( !m_pI->m_roadPos.IsValid() )
	{
		gout << "BH[" << GetFrame() << "]:UserPreAcitivity: ";
		gout << "object (name=" << m_pI->m_objName;
		gout << ") has invalid roadPos [SUICIDE]" << endl;
		if (m_pI->m_pObj){
			CPoint3D cartPos = m_pI->m_pObj->GetPos();
			gout << "  currPos = " << cartPos << endl;
		}
		Suicide();
	}

	CPoint3D cartPos = m_pI->m_roadPos.GetXYZ();

	//
	// Get results from init conditions.
	//
	m_pI->m_initCondState = m_initConditions.Execute( cartPos, GetFrame() );
	const TCHAR* infoStates[] = {"eWAIT", "eACTIVATE", "eUNDER_CONTROL", "eDELETE", "eEXIT"};
	TRACE(TEXT("Ado %d state:%s\n"), this, infoStates[m_pI->m_initCondState]);

	switch( m_pI->m_initCondState )
	{
	case eWAIT:

		//
		// Waiting for InitCondition's approval.
		//
		break;

	case eACTIVATE:

		//
		// The Ado is now activated.....set the states of all descendent
		// HCSMs to eACTIVE and create CVED object.
		//
		//
		// Check to see if CVED object creation fails.
		//
		if( !CreateCvedObject( cpSnoBlock ) )  return;


		SetStateDescendents( eACTIVE );

		//
		// Now extend path to take my initial velocity into account.
		// I need a longer path when my initial velocity is high.
		//
		m_pI->m_prevVel = m_pI->m_currVel;
		m_pI->m_currVel = m_pI->m_pObj->GetVelImm();
		if( m_pI->m_currVel > 0.0 )
		{
			ExtendPath( m_pI );

			if( !m_pRootCollection->m_sDisableCurvature )
			{
				m_pI->m_curvature.RefreshCurvature(
										m_pI->m_roadPos,
										*m_pI->m_pPath,
										GetFrame()
										);
				m_pI->m_curvature.ResetBuckets();
				m_pI->m_curvature.UpdateBuckets(
										m_pI->m_roadPos,
										m_pI->m_pObj->GetVelImm(),
										m_pI->m_objLength * 0.5 * cFEET_TO_METER
										);
			}

			ComputeIntersectionTurnSignal();


#ifdef DEBUG_CURVATURE
			if ( GetFrame() >= DEBUG_CURVATURE ) {
				gout << endl << endl;
				gout << "===" << MessagePrefix() << "  roadPos = " << m_pI->m_roadPos << endl;
				m_pI->m_curvature.DebugCurvature();

				gout << "---" << MessagePrefix();
				gout << "  roadPos = " << m_pI->m_roadPos;
				gout << "  vel = " << m_pI->m_pObj->GetVelImm() << endl;
				m_pI->m_curvature.DebugBuckets();
			}
#endif
		}

		Activated( cpSnoBlock );

		SetInputpIForAutonomous( m_pI );
		SetInputpIForRemoteControl( m_pI );

		//
		// Add an entry to the activity log for HCSM activation.
		//
		m_pRootCollection->SetHcsmActivateLog(
					this,
					m_pI->m_roadPos.GetXYZ()
					);

		break;

	case eUNDER_CONTROL:

		//
		// The Ado is now ready to execute.
		//
		Activated( cpSnoBlock );
		SetInputpIForAutonomous( m_pI );
		SetInputpIForRemoteControl( m_pI );
		break;

	case eDELETE:


		DeleteHcsm( this );
		break;
	}

#ifdef _DEBUG
	// only do this in a debug build.
	DisplayDebugInfo();
#endif
}  // UserPreActivity


void
CAdo::UserPostActivity( const CAdoParseBlock* cpSnoBlock )
{
	//
	// Don't execute post-acitivity if waiting.
	//
	if( m_pI->m_initCondState == eWAIT )  return;

	//
	// Set turn signals for turning at intersections.
	//
	SetTurnSignals();
	PostActivityProcessButtonsAndDials();

	//
	// Update monitors.
	//
	SetMonitorRoadPos( m_pI->m_roadPos );
	CCrdr targCrdr = m_pI->m_pPath->GetNextCrdr( m_pI->m_roadPos, false );
	if( targCrdr.IsValid() )
	{
		SetMonitorTargCrdr( targCrdr );
	}

	if( HasValueOutputImTargCrdrsFromRemoteControl() )
	{
		SetMonitorImTargCrdrs( GetOutputImTargCrdrsFromRemoteControl() );
	}
	else
	{
		SetMonitorImTargCrdrsNoValue();
	}
	if( HasValueOutputHasStopSignTargetFromRemoteControl() )
	{
		SetMonitorHasStopSignTarget( GetOutputHasStopSignTargetFromRemoteControl() );
	}
	else
	{
		SetMonitorHasStopSignTargetNoValue();
	}
    if( HasValueOutputStoppedAtStopSignFrameFromRemoteControl() )
	{
        SetMonitorStoppedAtStopSignFrame( GetOutputStoppedAtStopSignFrameFromRemoteControl());
	}
	else
	{
        SetMonitorStoppedAtStopSignFrameNoValue();
	}
    if (HasValueOutputHasStopSignTargetFromRemoteControl()){
        SetMonitorHasStopSignTarget(GetOutputHasStopSignTargetFromRemoteControl());
    }else{
        SetMonitorHasStopSignTargetNoValue();
    }
	//
	// Get outputs from the active child.
	//
    CVehicleObj* pVehicleObj = dynamic_cast<CVehicleObj *>( m_pI->m_pObj );

	//
	// Write outputs to CVED control inputs.
	//
	CPoint3D targPos = GetOutputTargPosFromRemoteControl();

    pVehicleObj->StoreOldTargPos();
	pVehicleObj->SetTargPos( targPos );

	//
	// Set the new outputs...TargSteer and TargAccel.  TargSteer replaces
	// TargPos and TargAccel replaces TargVel and TargDist.
	//
	bool haveAccel = HasValueOutputTargAccelFromRemoteControl();
	pVehicleObj->SetHaveAccel( haveAccel );
	double targAccel;
	if( haveAccel )
	{
		targAccel = GetOutputTargAccelFromRemoteControl();
		pVehicleObj->SetTargAccel( targAccel );
	}

	bool haveForcedVelocity = m_dialForcedVelocity.HasValue();
	bool processForcedVelocity = (
			haveForcedVelocity ||
			m_dialForcedVelocity.HasBeenReset() ||
			m_pI->m_forcedVelMode == eFV_OV_VEL ||
			m_pI->m_forcedVelMode == eFV_OV_TRACK
			);
	if( processForcedVelocity )
	{
		//double targAccel;
		double targVel;
		bool haveAccel;
		bool success = ParseForcedVelocityDial(
							m_pI->m_pObj->GetVelImm(),
							targVel,
							haveAccel,
							targAccel
							);

		if( success )
		{
			if( haveAccel )
			{
				//
				// User specified a target acceleration.
				//
				targAccel = ForcedVelocityController(
									m_pI->m_pObj->GetVelImm(),
									targVel,
									targAccel
									);
                //cout << "##forced targAccel used = " << targAccel << " m/s2"
                //     << " Vel = "<<m_pI->m_pObj->GetVelImm()<<" target "<<targVel<<endl;
			}
			else
			{
				//
				// User didn't specify a target acceleration...come up with
				// one on our own.
				//
				if( targVel < m_pI->m_pObj->GetVelImm() )
				{
					targAccel = ForcedVelocityController(
										m_pI->m_pObj->GetVelImm(),
										targVel,
										-1.0
										);
				}
				else
				{
					targAccel = CalcTargAccelForNormalCruising(
										m_pI->m_currVel,
										m_pI->m_prevVel,
										targVel,
										m_pI->m_aggressiveness,
										m_pRootCollection->m_rng,
										m_pI->m_rngStreamId
										);
				}
			}
#if 0
			gout << MessagePrefix() << "##forced targAccel = " << targAccel << " m/s2" << endl;
//			gout << "  currAccel        = " << pVehicleObj->GetAccel() << endl;
//			gout << "  currAccel2       = " << (m_pI->m_currVel - m_pI->m_prevVel) * 30 << endl;
#endif
			pVehicleObj->SetTargAccel( targAccel );
		}
	}

	bool haveSteer = HasValueOutputTargSteerFromRemoteControl();
	pVehicleObj->SetHaveSteer( haveSteer );
	if( haveSteer )
	{
		pVehicleObj->SetTargSteer( GetOutputTargSteerFromRemoteControl() );
	}

    bool haveSteerMax = HasValueOutputMaxSteerFromRemoteControl();
    if (haveSteerMax){
        pVehicleObj->SetSteerMax(GetOutputMaxSteerFromRemoteControl());
    }else{
       pVehicleObj->SetSteerMax(cDefaultMaxSteerRateRadspS);
    }
#if 0
	if( m_pI->m_objName == "Ado2" )
	{
		gout << MessagePrefix();
		gout << "v=";
		gout << m_pI->m_pObj->GetVelImm() * cMS_TO_MPH << " mph";
		gout << "   a=" << targAccel << " m/s2" << endl;
	}
#endif
	if (pVehicleObj->GetVel() > 0.1){
		m_pI->m_hasMoved = true;
	}
	//if we do not have a visual state from a dail, get the time of day, and the accel rate
	TU16b visState = pVehicleObj->GetVisualStateImm();
	if (m_pI->m_autoControlHeadLightsState){

		//turn the headlights on
		if (::CHcsmCollection::m_sHour > 18 || ::CHcsmCollection::m_sHour < 7){
			visState|= cCV_OPERATING_LIGHTS;
			visState|= cCV_HIGHBEAM_LIGHTS;
		}else{
			visState&= ~cCV_OPERATING_LIGHTS;
			visState&= ~cCV_HIGHBEAM_LIGHTS;
		}
	}
	if (m_pI->m_autoControlBrakeLightState){
		//we are not "parked", control the brake lights
		if (m_pI->m_hasMoved){
			double accel = pVehicleObj->GetAccel();
			double speed = pVehicleObj->GetVel();
			//we found from coast downs that decell from goes from -0.8g at 80 mph to -0.5g at 40 mph, and slowly decresses bellow 40
			bool setBrakeLight = false;
			if (speed > 0.1){
				if (accel<0){
					if (speed> 13.2){// 30>mph
						if ( (-0.5 + (max((speed-17.6f),0.0) * -0.017)  > accel) ){
							setBrakeLight = true;
						}
					}else{
						if ( (-0.2 + (max((speed-2.2f),0.0) * -0.027)  > accel) ){
							setBrakeLight = true;
						}else{
							setBrakeLight = false;
						}
					}
				}else{
					setBrakeLight = false;
				}
			}else{
				setBrakeLight = true;
			}
			if (setBrakeLight){
				visState|= cCV_BRAKE_LIGHTS;
			}else{
				visState&= ~cCV_BRAKE_LIGHTS;
			}
		}
	}
	pVehicleObj->SetVisualState(visState);
	CPoint3D currPos = m_pI->m_pObj->GetPos();
	CVector3D currTan =  m_pI->m_pObj->GetTan();
	CVector3D currLat = m_pI->m_pObj->GetLat();
	CVector3D normal;
	normal.m_i = ( currTan.m_j * currLat.m_k ) - ( currTan.m_k * currLat.m_j );
	normal.m_j = ( currTan.m_k * currLat.m_i ) - ( currTan.m_i * currLat.m_k );
	normal.m_k = ( currTan.m_i * currLat.m_j ) - ( currTan.m_j * currLat.m_i );

	double roll = atan2( currLat.m_k, normal.m_k );
	double pitch = asin( currTan.m_k );
	double yaw = atan2( currTan.m_j, currTan.m_i );

	CPoint3D drvObjPos = m_pI->m_pObj->GetPos();

	if( m_pI->m_pLogFile )
	{
		*m_pI->m_pLogFile << drvObjPos.m_x << " " << drvObjPos.m_y << " ";
		*m_pI->m_pLogFile << drvObjPos.m_z << " " << roll << " ";
		*m_pI->m_pLogFile << pitch << " " << yaw << " " << targAccel << " " << pVehicleObj->GetAccel() << " " << pVehicleObj->GetVelImm() << endl;

		//CPoint3D cartPos = m_pI->m_roadPos.GetXYZ();


		//*m_pI->m_pLogFile << GetFrame() * GetTimeStepDuration() << " ";	// in seconds
		//*m_pI->m_pLogFile << m_pI->m_pObj->GetVelImm() * cMS_TO_MPH << " ";	// in mph
		//*m_pI->m_pLogFile << pVehicleObj->GetAccel() << " "; // in meters/seconds^2
		//*m_pI->m_pLogFile << targAccel << " ";	// in meters/seconds^2
		//*m_pI->m_pLogFile << cartPos.m_x << " " << cartPos.m_y << " " << cartPos.m_z << " "; // in ft
		//*m_pI->m_pLogFile << targPos.m_x << " " << targPos.m_y << " " << targPos.m_z << " "; // in ft
		//*m_pI->m_pLogFile << endl;
	}

	//
	// Debugging stuff.
	//
	CPoint3D cartPos = m_pI->m_pObj->GetPosImm();
	DebugLine(
			CHcsmDebugItem::eDEBUG_ROUTINE,
			cartPos,
			targPos,
			CHcsmDebugItem::eBLUE,
			true
			);

//	fprintf( m_debugFile, "%g\t%g\t%g\t%g\t%g\n", targPos.m_x, targPos.m_y, cartPos.m_x, cartPos.m_y, cartPos.m_z );
}  // end of UserPostActivity


void
CAdo::UserDeletion( const CAdoParseBlock* cpSnoBlock )
{
	//
	// Print deletion message to output.
	//
	PrintDeletionMessage();
    if (!m_pI){
        gout<<"Invalid deletion Ado has no info object"<<endl;
        return;
    }
	//
	// Check to make sure that the CVED object is still valid
	// before deleting it.
	//
 	bool cvedObjValid = m_pI->m_pObj && m_pI->m_pObj->IsValid();
	if( cvedObjValid )
	{
#if defined ADO_CONTROLLER
		static_cast<CCvedADOCtrl*>(cved)->DistriDeleteDynObj(m_pI->m_pObj);
#else
		cved->DeleteDynObj( m_pI->m_pObj );
#endif
	}

	//
	// Add an entry to the activity log for HCSM deletion.
	//
	m_pRootCollection->SetHcsmDeleteLog(
				this,
				m_pI->m_roadPos.GetXYZ()
				);

	//
	// Release the rng stream id.
	//
	m_pRootCollection->m_rng.ReleaseStream( m_pI->m_rngStreamId );

	if( m_pI->m_pLogFile )
	{
		delete m_pI->m_pLogFile;
	}
	if( m_pI->m_pPath )  delete m_pI->m_pPath;
	if( m_pI )
	{
		delete m_pI;
		m_pI = NULL;
	}

//	fclose( m_debugFile );

}  // UserDeletion




void
CAutonomous::AutonomousPreActivity()
{
	//gout << "***** Executing AUTONOMOUS Pre-Activty" << endl;
}  // AutonomousPreActivity


void
CAutonomous::AutonomousPostActivity()
{
	//gout << "Executing AUTONOMOUS Post-Activty" << endl;
}  // AutonomousPostActivity


bool
CAutonomous::AutoRemotePredicate()
{
	return ( GetInputpI()->m_runMode == CAdoInfo::eREMOTE_CONTROL );
}  // AutoRemotePredicate




void
CRemoteControl::RemoteControlPreActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	SetInputpIForFreeDrive( pI );
	SetInputpIForFollow( pI );
	SetInputpIForLaneChange( pI );
	SetInputpIForNavigateIntrsctn( pI );
	SetInputpIForMerge( pI );


	if( HasValueOutputTargVelFromFreeDrive() )
	{
		SetInputFreeDriveTargVelForLaneChange(
					GetOutputTargVelFromFreeDrive()
					);
	}

}  // RemoteControlPreActivity


void
CRemoteControl::RemoteControlPostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

	//
	// Resolve target position.
	//
    if( HasValueOutputTargPosFromLaneChange() && !pI->m_reporjectAndResetLaneOffsetButton )
	{
		SetOutputTargPos( GetOutputTargPosFromLaneChange() );
//		gout << MessagePrefix() << "LnCh: " << pI->m_roadPos.GetOffset() << endl;
	}
	else
	{
		SetOutputTargPos( GetOutputTargPosFromFreeDrive() );
//		gout << MessagePrefix() << "FrDr: " << pI->m_roadPos.GetOffset() << endl;
	}

    // For low speed lane changes, or high urgency manuevers we want to
    // tell dyna it can steer harder than its default lim
    if( HasValueOutputMaxSteerFromLaneChange() )
	{
        SetOutputMaxSteer( GetOutputMaxSteerFromLaneChange() );
    }
	else if( HasValueOutputMaxSteerFromFreeDrive() )
	{
        SetOutputMaxSteer( GetOutputMaxSteerFromFreeDrive() );
    }
	else
	{
        SetOutputMaxSteerNoValue();
    }

	//
	//
	// Process the new outputs: TargAccel and TargSteer.
	//
	double accelList[5];			// holds accelerations from HCSMs
	char*  labels[5] = { 0 };		// watch the size of the string
	int            size;
	double acc;
	int    mergeId= -1;		// YEP
    int   followId = -1;
	size = 0;
	if( HasValueOutputTargAccelFromFreeDrive() )
	{
		accelList[size] = GetOutputTargAccelFromFreeDrive();
		labels[size]    = "FrDr";
		size++;
	}

    bool emergFollowOnly = pI->m_followTtc <= pI->m_FollowParams.ttcThres1 || pI->m_followDist < 10;
	bool ignoreFollow    = pI->m_maintainGap.IsActive() && !emergFollowOnly;

	if( !ignoreFollow && HasValueOutputTargAccelFromFollow() )
	{
		accelList[size] = GetOutputTargAccelFromFollow();
		labels[size]    = "Foll";
        followId = size;
		size++;
	}
	if( HasValueOutputTargAccelFromLaneChange() )
	{
		accelList[size] = GetOutputTargAccelFromLaneChange();
		labels[size]    = "LnCh";
		size++;
	}

	if( HasValueOutputTargAccelFromNavigateIntrsctn() )
	{
		accelList[size] = GetOutputTargAccelFromNavigateIntrsctn();
		labels[size]    = "NvIn";
		size++;
	}

	if( HasValueOutputTargAccelFromMerge() )
	{
		accelList[size] = GetOutputTargAccelFromMerge();
		labels[size]    = "Merg";
		mergeId = size;	// YEP
		size++;
	}
	// NOTE: the following assumes that FreeDrive always returns a value!!!
	double dummy = 0.0;
	if( size > 0 )
	{
		double      targAccel;
		int         which;
		bool        onlyHaveFollAndFree;

		if( size == 2 && HasValueOutputTargAccelFromFreeDrive()	&& HasValueOutputTargAccelFromFollow() )
		{
			onlyHaveFollAndFree = true;
		}
		else
		{
			onlyHaveFollAndFree = false;
		}

		// make sure follow wins if it is the only one along with follow
		// and it is supposed to accelerate to catchup
		if( mergeId > 0 )
		{	//if we have both a merge and a follow, try to figure out who should win
            //Merge does a better job for dealing with merging vehicle, but not not track
            //vehicle in front of it, so if there is a vehicle in front of the driver
            //and the ADO is too close to the lead veh, use follow, if not use merge
            if (followId > 0){
                float halfWay = (pI->m_FollowParams.ttcThres2 - pI->m_FollowParams.ttcThres1)/2.0 + pI->m_FollowParams.ttcThres1;
                if (pI->m_followTtc > pI->m_FollowParams.ttcThres1 &&  pI->m_followDist > 6.0){
                    if (pI->m_followTtc < pI->m_FollowParams.ttcThres2){
                        bool isMergeTrackingLeadVeh = false;
                        for (int i =0; i < pI->m_mergeInfo.size(); i++){
                            if (pI->m_followTarget == pI->m_mergeInfo[i].leadObjId){
                                isMergeTrackingLeadVeh = true;
                            }
                        }
                        if (isMergeTrackingLeadVeh){
			                which = mergeId;		// YEP
                        }else{
                            if (pI->m_followTtc < halfWay)
                                which = followId;
                            else
			                    which = mergeId;		// YEP
                        }
                    }else{
                        which = mergeId;		// YEP
                    }
                }
                else{
                    which = followId;
                }
            }else{
                which = mergeId;
            }
		}			// YEP
		else		// YEP
		if( onlyHaveFollAndFree && pI->m_FollowParams.normal.accelToCatchUp )
		{
			which = 1;	// follow overrides
		}
		else
		{
			ResolveAccelConservative2( accelList, size, which );
		}

		targAccel = accelList[which];
		strncpy(pI->m_acgoutHcsmText, labels[which],sizeof(pI->m_acgoutHcsmText));
		SetOutputTargAccel( targAccel );

		dummy = targAccel;
	}
	else
	{
		SetOutputTargAccelNoValue();
		strncpy( pI->m_acgoutHcsmText, "---",sizeof(pI->m_acgoutHcsmText) );
	}

	//// Steering
	if( HasValueOutputTargSteerFromFreeDrive() )
	{
		SetOutputTargSteer( GetOutputTargSteerFromFreeDrive() );
	}
	else
	{
		SetOutputTargSteerNoValue();
	}

	if( HasValueOutputImTargCrdrsFromNavigateIntrsctn() )
	{
		SetOutputImTargCrdrs( GetOutputImTargCrdrsFromNavigateIntrsctn() );
	}
	else
	{
		SetOutputImTargCrdrsNoValue();
	}
    if (HasValueOutputStoppedAtStopSignFrameFromNavigateIntrsctn() ){
        SetOutputStoppedAtStopSignFrame(GetOutputStoppedAtStopSignFrameFromNavigateIntrsctn());
    }else{
        SetOutputStoppedAtStopSignFrameNoValue();
    }
    if (HasValueOutputHasStopSignTargetFromNavigateIntrsctn()){
        SetOutputHasStopSignTarget(GetOutputHasStopSignTargetFromNavigateIntrsctn());
    }else{
        SetOutputHasStopSignTargetNoValue();
    }
}  // RemoteControlPostActivity


bool
CRemoteControl::RemoteAutoPredicate()
{
	return ( GetInputpI()->m_runMode == CAdoInfo::eAUTONOMOUS );
}  // RemoteAutoPredicate
