/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: expeval.cxx,v 1.19 2015/11/17 16:43:14 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad, Yiannis Papelis
 *
 * Date:    December, 2003
 *
 * Description:  Contains the implementation for the CExpEval class.
 *
 ****************************************************************************/

#include "expeval.h"
#include <hcsmcollection.h>
#include <util.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include <pi_iostream>
using namespace std;

extern void 
StringFlip( const volatile char from[], char to[], int size );
extern void
StrncpyFlip( char* pDst, const char* cpSrc, int size );

////////////////////////////////////////////////////////////
//Common functions
///////////////////////////////////////////////////////////
double AbsoluteValue( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if (!invalidArgs) {
		double value = args[0].m_Num;
		return abs(value);
	}

	return 0.0;
}

double SquareRoot( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if (!invalidArgs) {
		double value = args[0].m_Num;
		if (value > 0.0) {
			return sqrt(value);
		}
	}

	return 0.0;
}

double RaiseToPower( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 2 || !args[0].m_IsNum || !args[1].m_IsNum;
	if (!invalidArgs) {
		double mantissa = args[0].m_Num;
		double exponent = args[1].m_Num;
		return pow(mantissa, exponent);
	}

	return 0.0;
}

double Minimum( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 2 || !args[0].m_IsNum || !args[1].m_IsNum;
	if (!invalidArgs) {
		double A = args[0].m_Num;
		double B = args[1].m_Num;
		if (A < B) {
			return A;
		} else {
			return B;
		}
	}

	return 0.0;
}

double Maximum( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 2 || !args[0].m_IsNum || !args[1].m_IsNum;
	if (!invalidArgs) {
		double A = args[0].m_Num;
		double B = args[1].m_Num;
		if (A > B) {
			return A;
		} else {
			return B;
		}
	}

	return 0.0;
}

double DegreesToRadians( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if (!invalidArgs) {
		double value = args[0].m_Num;
		return value * M_PI / 180.0;
	}

	return 0.0;
}

double RadiansToDegrees( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if (!invalidArgs) {
		double value = args[0].m_Num;
		return value * 180 / M_PI;
	}

	return 0.0;
}

double Sine( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if (!invalidArgs) {
		double value = args[0].m_Num;
		return sin(value);
	}

	return 0.0;
}

double Cosine( int argC, const CExprParser::CStrNum args[] )
{
	bool invalidArgs = argC != 1 || !args[0].m_IsNum;
	if (!invalidArgs) {
		double value = args[0].m_Num;
		return cos(value);
	}

	return 0.0;
}

double Tan( int argC, const CExprParser::CStrNum args[] ){
	if (argC != 1){
		gout<<"invalid argument count for CExpEval:Tan"<<endl;
		return 0;
	}
	if (!args[0].m_IsNum){
		gout<<"invalid argument for CExpEval:Tan"<<endl;
		return 0;
	}
	return (tan(args[0].m_Num));
}


//////////////////////////////////////////////////////////////////////////////
///
///  
///\brief
///		This function calculates a random number between 0 to 1
///
/// Arguments:
///   argC - The number of arguments.
///   args - The arguments.
///          arg #1: string - The object's name in the scenario.
///
/// Returns:  Returns a random number between 0-1
///
//////////////////////////////////////////////////////////////////////////////
double Rand( int argC, const CExprParser::CStrNum args[] ){
	CHcsmCollection* pHcsmRandNumGen;
	shared_ptr<mt19937> pRandNumGen;
	pRandNumGen = NULL;
	double result = 0;

	if (argC > 0){
		if(!args[0].m_IsNum){
			bool randExist;
			randExist = pHcsmRandNumGen->DoesRandomNumberGeneratorExist(args[0].m_Str);
			if(!randExist){
				gout << "CExpEval: unknown random generator '" << args[0].m_Str << "'" << endl;
				return 0;
			}
		}
		try{
			if (!args[0].m_IsNum && argC == 1){
				pRandNumGen = pHcsmRandNumGen->GetRandomNumberGenerator(args[0].m_Str);
				double randomNum =  (*pRandNumGen)();
				double max = pRandNumGen->max();
				result = randomNum/max;
				return result;
			}
			else if(args[0].m_IsNum && argC == 1){
				pRandNumGen = pHcsmRandNumGen->GetRandomNumberGenerator("");
				double randomNum =  (*pRandNumGen)();
				double max = pRandNumGen->max();
				result = randomNum/max;
				return result;
			}
			else if(argC >= 2){
				pRandNumGen = pHcsmRandNumGen->GetRandomNumberGenerator(args[0].m_Str);
				if(!args[1].m_IsNum){
					if((args[1].m_Str == "normal")||(args[1].m_Str == "Normal")){
						if(argC == 4){
							if((args[2].m_IsNum)&&(args[3].m_IsNum)){
								std::normal_distribution<double> normalDistribution(args[2].m_Num,args[3].m_Num);
								result = normalDistribution(*pRandNumGen);
								return result;
							}
							else{
								gout << "CExpEval: Invalid argument(s) for normal distribution" << endl;
								return 0;
							}
						}
						else{
							gout<<"CExpEval: Exactly two arguments are required for normal distribution" <<endl; 
							return 0;
						}
					}
					else if((args[1].m_Str == "gamma")||(args[1].m_Str == "Gamma")){
						if(argC == 4){
							if((args[2].m_IsNum)&&(args[3].m_IsNum)){
								std::gamma_distribution<double> gammaDistribution(args[2].m_Num,args[3].m_Num);
								result = gammaDistribution(*pRandNumGen);
								return result;
							}
							else{
								gout << "CExpEval: Invalid argument(s) for gamma distribution" << endl;
								return 0;
							}
						}
						else{
							gout<<"CExpEval: Exactly two arguments are required for gamma distribution" <<endl; 
							return 0;
						}
					}
					else if((args[1].m_Str == "uniformInt")||(args[1].m_Str == "UniformInt")){
						if(argC == 4){
							if((args[2].m_IsNum)&&(args[3].m_IsNum)){
								int range = 1;
								range = (int)args[3].m_Num - (int)args[2].m_Num;
								if(range < 0){
									gout << "CExpEval: Invalid arguments for uniform int distribution."<<endl;
									gout << "CExpEval: argument 1 can not be greater than argument 2"<<endl;
									return 0;
								}
								else{
									std::uniform_int_distribution<int> distribution((int)args[2].m_Num,(int)args[3].m_Num);
									result = distribution(*pRandNumGen);
									return result;
								}
							}
							else{
								gout << "CExpEval: Invalid argument(s) for uniform int distribution" << endl;
								return 0;
							}
						}
						else{
							gout<<"CExpEval: Exactly two arguments are required for uniform int distribution" <<endl; 
							return 0;
						}
					}
					else if((args[1].m_Str == "uniform")||(args[1].m_Str == "Uniform")){
						if(argC == 4){
							if((args[2].m_IsNum)&&(args[3].m_IsNum)){
								int range;
								range = (int)(args[3].m_Num - args[2].m_Num);
								if(range < 0){
									gout << "CExpEval: Invalid arguments for uniform distribution."<<endl;
									gout << "CExpEval: argument 1 can not be greater than argument 2"<<endl;
									return 0;
								}
								else{
									std::uniform_real_distribution<double> distribution(args[2].m_Num,args[3].m_Num);
									result = distribution(*pRandNumGen);
									return result;
								}
							}
							else{
								gout << "CExpEval: Invalid argument(s) for uniform distribution" << endl;
								return 0;
							}
						}
						else{
							gout<<"CExpEval: Exactly two arguments are required for uniform distribution" <<endl; 
							return 0;
						}
					}
					else{
						string temp1 = args[0].m_Str;
						string temp2 = args[1].m_Str;
						string temp3 = args[2].m_Str;
						string temp4 = args[3].m_Str;
						gout << "CExpEval: unknown distribution '"<<args[1].m_Str<<"'"<<endl;
						return 0;
					}
				}
			}
		}catch(std::exception &e){
			gout<< "CExpEval: Invalid arguments for the distribution"<<endl;
			return 0;
		}
	}
	return result;
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CExpEval::CExpEval()
{
}

CExpEval::CExpEval( const CExpEval& objToCopy )
{
	// call the assignment operator
	*this = objToCopy;
}

CExpEval& 
CExpEval::operator=( const CExpEval& objToCopy )
{
	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) 
	{

	}

	return *this;
}

CExpEval::~CExpEval()
{

}

double
CExpEval::EvaluateVariable( const char* cpName )
{
	map<string, double>::iterator p = m_variables.find( cpName );
	bool foundVariable = p != m_variables.end();

	if( foundVariable )
	{
		return p->second;
	}
	else
	{
		if (CHcsmCollection::ExprVariableExists( cpName )){
            return CHcsmCollection::GetExprVariable(cpName);
        }else{
			if (cpName){
				if (m_unkownVariables.find(cpName) == m_unkownVariables.end()){
					cerr << "CExpEval: unknown variable '" << cpName << "'" << endl;
					m_unkownVariables.insert(cpName);
				}
			}
        }
		
	}
	return 0.0;
}

bool
CExpEval::EvaluateFunction(
			const string& cName,
			int numArg,
			const CStrNum args[],
			CStrNum& result
			)
{
	map<string, pFunc>::iterator p = m_functions.find( cName );
	bool foundFunction = p != m_functions.end();

	if( foundFunction )
	{
		result.SetVal( (*p->second)( numArg, args ) );
		return true;
	}
	else
	{
		

		double val = 0;
		if (cName == "GetObjVel")			val = GetObjVel( numArg, args ) ; 
		else if (cName == "GetObjAccel")	val = GetObjAccel( numArg, args ); 
		else if (cName == "GetObjDistPow2") val = GetObjDistPow2( numArg, args ) ;
		else if (cName == "ReadCell")		val = ReadCell( numArg, args ) ;
		else if (cName == "OvVel")			val = GetOvVel( numArg, args );
		else if (cName == "CalcDist")		val = GetCalcDist(numArg, args);
        else if (cName == "Rand")           val = Rand(numArg, args);
        else if (cName == "CalcDistToOwnVeh") val = CalcDistToOwnVeh(numArg, args);
		else if (cName == "Tan")            val = Tan(numArg, args);
		else if (cName == "Sin")            val = Sine(numArg, args);
		else if (cName == "Cos")            val = Cosine(numArg, args);
		else if (cName == "Deg2Rad")        val = DegreesToRadians(numArg, args);
        else if (cName == "Rad2Deg")        val = RadiansToDegrees(numArg, args);
        else if (cName == "ReadVar")        val = ReadVar(numArg, args);
        else if (cName == "GetQueueSize")   val = GetQueueSize(numArg, args);
		else if (cName == "GetOvTtcToObj")  val = GetOvTtcToObj(numArg, args);
		else if (cName == "GetObjTtcToOv")  val = GetObjTtcToOv(numArg, args);
		else if (cName == "abs")			val = AbsoluteValue(numArg, args);
		else if (cName == "sqrt")			val = SquareRoot(numArg, args);
		else if (cName == "pow")			val = RaiseToPower(numArg, args);
		else if (cName == "min")			val = Minimum(numArg, args);
		else if (cName == "max")			val = Maximum(numArg, args);
		else return false; //we do not have a function
		
		result.SetVal(val);
		return true;
	}
}
double
CExpEval::GetObjVel( int argC, const CExprParser::CStrNum args[] )
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
CExpEval::GetQueueSize( int argC, const CExprParser::CStrNum args[] ){
    if (argC != 1){
        gout<<"GetQueueSize does takes 1 element"<<endl; 
        return 0;
    }
    if (args[0].m_IsNum){
        gout<<"GetQueueSize Invalid Param"<<endl; 
        return 0;
    }
    int size = 0;
    CHcsmCollection::GetVarQueueSize(args[0].m_Str,size);
    return (double)size;
}
double
CExpEval::GetObjAccel( int argC, const CExprParser::CStrNum args[] ){

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
CExpEval::GetObjDistPow2( int argC, const CExprParser::CStrNum args[] )
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
///		reads a cell value for the current time for forcedVel
///\remark
///		This function is a delegate used by Ado Expression Parser
///
///\returns double -Current value of the sin(T) 
////////////////////////////////////////////////////////////////////////
double CExpEval::ReadCell( int argC, const CExprParser::CStrNum args[] )
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
			return ((0-5507.7-(2218.8* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1))/((2218.8)/(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 2)
			return ((0-5062.2-(2664.3* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1))/(( 2664.3)/(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 3)
			return ((0-4616.6-(3109.9* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1))/(( 3109.9)/(1.46666*( ownVehVel-25.1)))/1.466666;
		if (index == 4)
			return ((0-4171.1-(3555.4* ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1))/(( 3555.4)/(1.46666*( ownVehVel-25.1)))/1.466666;	
		if (index == 5)
			return ((0-3725.5-(4001  * ownVehVel)/( ownVehVel-25.1)-700)-(0-15564.1))/(( 4001)  /(1.46666*( ownVehVel-25.1)))/1.466666;
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
	else if (CHcsmCollection::ReadCellNumeric != NULL){
		//bool (*func) (const string&, int, float&);// = CHcsmCollection::ReadCellNumeric;
		//func = (TReadCellFuncptr)CHcsmCollection::ReadCellNumeric;
		float result = 0;
		if (CHcsmCollection::ReadCellNumeric(cellName,index -1,result)){
			return result;
		}
		else return 0.0;
	}
/*	else if (cellName == "OwnVehAccel"){
		int ownVehObjId;
		cved->GetObj( "ExternalDriver", ownVehObjId );
		const CDynObj* pOwnVehObj = cved->BindObjIdToClass( ownVehObjId );
		return (pOwnVehObj->GetVel()  - pOwnVehObj->GetVelImm()) / CHcsmCollection::GetTimeStepDuration();	
	}*/
	else
	{
		gout << "ExpressionTrigger: unknown cell name '" << cellName;
		gout << "'" << endl;
	}

	return 0.0;
}
////////////////////////////////////////////////////////////////////////////////////////
///\brief
///     Reads a variable's value
////////////////////////////////////////////////////////////////////////////////////////
double CExpEval::ReadVar( int argC, const CExprParser::CStrNum args[] ){
    if (argC != 1){
        gout<<"Error Bad Param Count for ReadVar"<<endl;
        return 0;
    }
    if (args[0].m_IsNum){
        gout<<"Error Bad Param Type for ReadVar"<<endl;
        return 0;
    }
    return EvaluateVariable(args[0].m_Str.c_str());
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
double CExpEval::GetOvVel( int argC, const CExprParser::CStrNum args[] ){
	double targVel = 0;
	cved->GetOwnVehicleVel( targVel );
	return targVel * cMS_TO_MPH;
}//////////////////////////////////////////////////////////////////////////////
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
CExpEval::GetObjTtcToOv( int argC, const CExprParser::CStrNum args[] )
{
	// make sure CVED is valid
	if( !cved )  return 0;

	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	//if( invalidArgs )  PrintInvalidArgs( argC, args, "GetObjTtcToOv" );

	//
	// Given the object's name, find its CVED id.
	//
	string objName = args[0].m_Str;
	int objId;
	bool foundObj = cved->GetObj( objName, objId );
	if( !foundObj )
	{
		gout << "GetObjTtcToOv: unable to find object named '";
		gout << objName << "'" << endl;

		return 0.0;
	}

	//
	// Calculate the distance from the object to the OV.
	//
	CPoint3D objPos = cved->GetObjPos( objId );
	CPoint3D ovPos;
	bool gotOvInfo = cved->GetOwnVehiclePos( ovPos );
	if( !gotOvInfo )  return 0.0;
	double dist = fabs( (ovPos - objPos).Length() );

	//
	// Calculate the difference in velocities between the object and
	// the OV.
	// 
	double objVel = cved->GetObjVel( objId );
	double ovVel;
	gotOvInfo = cved->GetOwnVehicleVel( ovVel );
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
double 
CExpEval::GetOvTtcToObj( int argC, const CExprParser::CStrNum args[] )
{
	// make sure CVED is valid
	if( !cved )  return 0;

	bool invalidArgs = argC != 1 || args[0].m_IsNum;
	//if( invalidArgs )  PrintInvalidArgs( argC, args, "GetOvTtcToObj" );

	//
	// Given the object's name, find its CVED id.
	//
	string objName = args[0].m_Str;
	int objId;
	bool foundObj = cved->GetObj( objName, objId );
	if( !foundObj )
	{
		gout << "GetObjTtcToOv: unable to find object named '";
		gout << objName << "'" << endl;

		return 0.0;
	}

	//
	// Calculate the distance from the object to the OV.
	//
	CPoint3D objPos = cved->GetObjPos( objId );
	CPoint3D ovPos;
	bool gotOvInfo = cved->GetOwnVehiclePos( ovPos );
	if( !gotOvInfo )  return 0.0;
	double dist = fabs( (ovPos - objPos).Length() );

	//
	// Calculate the difference in velocities between the object and
	// the OV.
	//
	double objVel = cved->GetObjVel( objId );
	double ovVel;
	gotOvInfo = cved->GetOwnVehicleVel( ovVel );
	if( !gotOvInfo )  return 0.0;
	double velDiff = ( ovVel - objVel ) * cMETER_TO_FEET;  // convert to ft/s

	double ttc = dist / velDiff;
//	gout << objName << ":  ov ttc to obj = " << ttc << endl;
	return ttc;
}
//////////////////////////////////////////////////////////////////////////////
///
///  
///\brief
///		This function calculates the distance between two stored positions
///
/// Arguments:
///   argC - The number of arguments.
///   args - The arguments.
///          arg #1: string - The object's name in the scenario.
///
/// Returns:  The OwnVehicle's TTC to the given object.
///
//////////////////////////////////////////////////////////////////////////////
double CExpEval::GetCalcDist( int argC, const CExprParser::CStrNum args[] ){
	if (argC != 2 && args[0].m_IsNum || args[1].m_IsNum){
		return 1000000000000000000000.0f;
	}
	CPoint3D pos1 = CHcsmCollection::GetExprPosVariable(args[0].m_Str);
	CPoint3D pos2 = CHcsmCollection::GetExprPosVariable(args[1].m_Str);
	return pos1.Dist(pos2);

	
}
double CExpEval::CalcDistToOwnVeh(int argC, const CExprParser::CStrNum args[]){
    if (argC != 1){
        gout<<"invalid argument count for CalcDistToOwnVeh"<<endl;
        return 0;
    }
    CBoundingBox ownVehBox = cved->GetObjBoundBox(0);
    if (args[0].m_IsNum){
        gout<<"invalid argument for CalcDistToOwnVeh"<<endl;
        return 0;        
    }
    int id;
    if (!cved->GetObj(args[0].m_Str,id)){
        gout<<"cannot find veh:"<<args[0].m_Str<<" for CalcDistToOwnVeh"<<endl;
        return 0;              
    }
    double distToLeadObj;

    CPoint3D otherVehPos = cved->GetObjPos(id);
	CPoint3D leadVehPos = cved->GetObjPos(0 );
    distToLeadObj = leadVehPos.Dist(otherVehPos);

	double leadObjLength = cved->GetObjLength( id );
	double cabLength = cved->GetObjLength( 0 );
	double adjustDist = ( leadObjLength * 0.5f ) + ( cabLength * 0.5f );
	return distToLeadObj - adjustDist;

}