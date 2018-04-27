//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2003-2004 by National Advanced Driving Simulator and
// Simulation Center, The University of Iowa and the University of Iowa.
// All rights reserved.
//
// Version:		$Id: ScenarioControl.cxx,v 1.167 2016/10/28 20:48:54 IOWA\dheitbri Exp $
// Author(s):   Yiannis Papelis
// Date:        December, 2003
//
// Description: Implementation file for the CScenarioControl class.
//
/////////////////////////////////////////////////////////////////////////////

#include "ScenarioControl.h"
#include "SplineHermite.h"
#include <sstream>
#include "EnvVar.h"
#include "LibExternalObjectIfNetwork.h"
#include "CvedEDOCtrl.h"
//#include "cvedstrc.h"
#define _DIFF_DISTRI_SCENE

#include "util.h"
#define DEFAULT_ROAD_MARKING_WHEN_NO_TAGS


#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define	SIGN(x)	(((x) < 0)? -1 : 1)

int   CScenarioControl::m_sCollisionCount = 0;
int   CScenarioControl::m_sCollisionListSize = 0;
short CScenarioControl::m_sCollisionCvedObjId[cMAX_COLLISION_LIST_SIZE];
short CScenarioControl::m_sCollisionCvedObjType[cMAX_COLLISION_LIST_SIZE];
int   CScenarioControl::m_sCollisionSolObjId[cMAX_COLLISION_LIST_SIZE];
//int   CScenarioControl::m_sCollisionPrevCvedObjId[cMAX_COLLISION_LIST_SIZE];
int   CScenarioControl::m_sTrailerCollisionCount = 0;
int   CScenarioControl::m_sTrailerCollisionListSize = 0;
short CScenarioControl::m_sTrailerCollisionCvedObjId[cMAX_COLLISION_LIST_SIZE];
short CScenarioControl::m_sTrailerCollisionCvedObjType[cMAX_COLLISION_LIST_SIZE];
int   CScenarioControl::m_sTrailerCollisionSolObjId[cMAX_COLLISION_LIST_SIZE];
//int   CScenarioControl::m_sTrailerCollisionPrevCvedObjId[cMAX_COLLISION_LIST_SIZE];
float CScenarioControl::m_sOwnVehToLeadObjDist = -1.0f;
float CScenarioControl::m_sSpeedLimit = 45.0f;  // mph
float CScenarioControl::m_sVisibility = 100.0f;
float CScenarioControl::m_sWindSpeed = 0.0f;
float CScenarioControl::m_sWindDirection[2];
short CScenarioControl::m_sTrafLightSize;
short CScenarioControl::m_sTrafLightId[cTRAF_LIGHT_MAX_SIZE];
short CScenarioControl::m_sTrafLightState[cTRAF_LIGHT_MAX_SIZE];

CScenarioControl::CScenarioControl()
{
	m_haveError    = false;
	m_behavDeltaT  = (float) (1.0f / 30.0f);
	m_dynaMult     = 2;
	m_verbose      = 0;

	m_pCved        = 0;
	m_pRootColl    = 0;
	m_pHdrBlk      = 0;
	m_cpCabSolObj  = 0;
	m_cpCabTrailerSolObj = 0;
	m_sirenSpeed = -1; //off

	m_blankColor[0] = 0; //black
	m_blankColor[1] = 0;
	m_blankColor[2] = 0;

	m_scenarioHasBlanking = true;
	m_ownshipHeadlightsOn = false;

	ResetCollisionDetectionVars();

	m_sTrafLightSize = 0;
	int i;
	for( i = 0; i < cTRAF_LIGHT_MAX_SIZE; i++ )
	{
		m_sTrafLightId[i]    = 0;
		m_sTrafLightState[i] = 0;
	}

	m_pExternalObjCtrl = 0;
}

CScenarioControl::~CScenarioControl()
{
	delete m_pHdrBlk;
}

void
CScenarioControl::SetDefaults(
			float         behavDelta,
			int           dynaMult,
			int           verbose
			)
{
	m_behavDeltaT = behavDelta;
	m_dynaMult    = dynaMult;
	m_verbose     = verbose;
}


const char*
CScenarioControl::GetLastErrorString( void ) const
{
	return m_haveError ? m_lastError : "";
}

void
CScenarioControl::ExecMaintainer()
{
	assert( m_pCved );
	m_pCved->Maintainer();
}

void
CScenarioControl::ExecDynamics()
{
	assert( m_pCved );
	m_pCved->ExecuteDynamicModels();
}




void
CScenarioControl::ExecBehaviors()
{
	assert( m_pRootColl );
	m_pRootColl->ExecuteAllHcsm();
}


const CSol&
CScenarioControl::GetSol( void ) const
{
	return m_pCved->GetSol();
}

CCved&
CScenarioControl::GetCved( void )
{
	assert( m_pCved );
	return *m_pCved;
}

// CCvedDistri&
// CScenarioControl::GetDistriCved(void)
// {
// 	assert( m_pCved );
// 	return *dynamic_cast<CCvedDistri*>(m_pCved);
// }

CHcsmCollection&
CScenarioControl::GetHcsm( void )
{
	assert( m_pRootColl );
	return *m_pRootColl;
}

long
CScenarioControl::GetFrame( void )
{
	return m_pRootColl->GetFrame();
}

bool
CScenarioControl::ProcessFile(
			const string& cFileName,
			const char* cpEnvVarName,
			string& fileNameFull,
			bool testFileExistence
			)
{
    string envVar;
    NADS::GetEnvVar(envVar,cpEnvVarName);
	if( envVar != "" )
	{
		fileNameFull = envVar;
	}
	else
	{
		sprintf_s(
			m_lastError,
            sizeof(m_lastError),
			"No definition for environment var '%s'",
			cpEnvVarName
			);
		m_haveError = true;
		return false;
	}

	fileNameFull += cFileName;

	if( testFileExistence )
	{
		FILE* pTest = fopen( fileNameFull.c_str(), "r" );
		if( pTest == NULL )
		{
			fileNameFull ="../data/"+ cFileName;

            pTest = fopen( fileNameFull.c_str(), "r" );
            if( pTest == NULL )
            {
                sprintf_s(
			    	m_lastError,
                    sizeof(m_lastError),
			    	"Cannot open file %s",
			    	fileNameFull.c_str()
			    	);
			    m_haveError = true;
                return false;
            }
            fclose( pTest );
		}
		fclose( pTest );
	}

	return true;
}
//////////////////////////////////////////////////////////////////////////////
///\brief
///		Get HeadlightScenarioControl
//////////////////////////////////////////////////////////////////////////////
int
CScenarioControl::GetScenarioHeadlightSetting( void ){
	return CHcsmCollection::m_sHeadlightScenarioControl; //<are the headlights on or off?
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the prefix name of the LRI file--i.e. if the LRI
//  is named "WirelessUA.lri" then this function returns "WirelessUA".
//
// Remarks:
//
// Arguments:
//
// Returns:  The LRI name.
//
//////////////////////////////////////////////////////////////////////////////
const char*
CScenarioControl::GetLriName( void ) const
{
	// expand is ignored for now
	return m_lriFileNamePrefix.c_str();
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the name of the scenario file.
//
// Remarks:  InitSimultion has to be called before this function to ensure
//  that member varibles are properly initialized.
//
// Arguments:
//   expand - When true, the full name is returned.
//
// Returns:  The scenario file name.
//
//////////////////////////////////////////////////////////////////////////////
const char*
CScenarioControl::GetScnFileName( bool expand ) const
{
	if( expand )
	{
		return m_scnFileNameFull.c_str();
	}
	else
	{
		return m_scnFileName.c_str();
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the SOL type associated with the cab being used
//  in this scenario.
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize m_cabSolObjName.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
string
CScenarioControl::GetCabSolObjName( void ) const
{
	return m_cabSolObjName;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the SOL object associated with the cab being used
//  in this scenario.
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize m_cpCabSolObj.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
const CSolObj*
CScenarioControl::GetCabSolObj( void ) const
{
	return m_cpCabSolObj;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the SOL type associated with the cab trailer being
//  used in this scenario.
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize m_cabTrailerSolObjName. It will be empty if no
//  ownship trailer is specified.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
string
CScenarioControl::GetCabTrailerSolObjName( void ) const
{
	return m_cabTrailerSolObjName;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the SOL object associated with the cab trailer being
//  used in this scenario.
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize m_cpCabTrailerSolObj. It will be NULL if no ownship
//  trailer is specified.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
const CSolObj*
CScenarioControl::GetCabTrailerSolObj( void ) const
{
	return m_cpCabTrailerSolObj;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the model cigi id and the eyepoint offset
// with respect to the visual model's origin of the cab specified
// in the scenario file. The offset is in the visual model's
// coordinate system, i.e. y forward, x to the right, and z up.
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize m_cpCabSolObj.
//
// Arguments:
//  cabModelType - the variable to store the cab visual model type
//  showCab - the variable to store if the cab is to be shown
//  offset - the structure to store the eyepoint offset
//
// Returns:  true if successful.
//
//////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::GetCabInformation(
			string& cabSolObjName,
			int& cabModelType,
			bool& showCab,
			CPoint3D& offset,
			string& cabTrailerSolObjName,
			int& cabTrailerModelType
			)
{
//	const CSolObj* cabObj = m_pCved->GetSol().GetObj( m_cabSolObjName );
	if( m_cpCabSolObj == NULL )  return false;

	cabSolObjName = m_cabSolObjName;
	cabModelType = m_cpCabSolObj->GetVisModelCigiId();
	offset = ((const CSolObjVehicle*)m_cpCabSolObj)->GetEyepointOffset();
	cabTrailerSolObjName = m_cabTrailerSolObjName;
	if ( m_cpCabTrailerSolObj )
		cabTrailerModelType = m_cpCabTrailerSolObj->GetVisModelCigiId();
	showCab = m_showCab;

	return true;
}
//////////////////////////////////////////////////////////////////////////////
///\brief
///     Should the sim shake the cab when the driver hits something?
///
///
/////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::IsCabShakeCrashEffectEnabled(){
    if (!m_pHdrBlk)
        return false;
    return (m_pHdrBlk->GetCollisionEffectMask() & CHeaderParseBlock::eCAB_SHAKE) > 0;
}
//////////////////////////////////////////////////////////////////////////////
///\brief
///     Should the sim show a broken window at collisions?
///
///
/////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::IsBrokenWindshieldCrashEffectEnabled(){
    if (!m_pHdrBlk)
        return false;
    return (m_pHdrBlk->GetCollisionEffectMask() & CHeaderParseBlock::eCRACK_WINDSHIELD) > 0;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Set the ownship cab and trailer types
//
// Remarks:  Should be called during visual setup activities
//
// Arguments:
//  cabSOLName - SOL object name of the ownship cab
//  trailerSOLName - SOL object name of the ownship trailer (optional)
//
// Returns:  true if successful.
//
//////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::SetCabType( const char* cabSOLName, const char* trailerSOLName )
{
	//
	// Set the cab SOL name and object.
	//

	if ( cabSOLName == NULL || strlen( cabSOLName ) == 0 )
		return false;

	m_cabSolObjName = cabSOLName;
	m_cpCabSolObj = m_pCved->GetSol().GetObj( m_cabSolObjName );
	if( !m_cpCabSolObj )
	{
		sprintf_s(
			m_lastError,
            sizeof(m_lastError),
			"unable to get SOL info for cab '%s'",
			m_cabSolObjName.c_str()
			);
		return !(m_haveError=true);
	}

	//
	// Set the cab trailer SOL name and object if it exists
	//

	if ( trailerSOLName == NULL )
		return true;

	// map trailer type to sol obj
	// right now it will set m_cabTrailerSolObjName to what's in
	// the scenario during initialization, regardless of what value
	// has been set to it before that

	/*
	m_cabTrailerSolObjName = m_pHdrBlk->GetTrailerSolObjName();
	if( m_cabTrailerSolObjName.size() > 0 )
	{
		m_cpCabTrailerSolObj = m_pCved->GetSol().GetObj( m_cabTrailerSolObjName );
		if( !m_cpCabTrailerSolObj )
		{
			sprintf(
				m_lastError,
				"unable to get SOL info for cab trailer '%s'",
				m_cabTrailerSolObjName.c_str()
				);
			return !(m_haveError=true);
		}
	}
	*/

	return true;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns Ownship Headlight information
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize the Header block.
//
// Arguments:
//
// Returns:  true if ownship headlight is on.
//
//////////////////////////////////////////////////////////////////////////////bool
bool
CScenarioControl::GetOwnshipLights(
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
			)
{
	if (m_ownshipHeadlightsOn) {
		azimuth           = m_ownshipHeadlightsAzimuth;
		elevation         = m_ownshipHeadlightsElevation;
		beamWidth         = m_ownshipHeadlightsBeamWidth;
		beamHeight        = m_ownshipHeadlightsBeamHeight;
		constCoeff        = m_ownshipHeadlightsConstCoeff;
		linearCoeff       = m_ownshipHeadlightsLinearCoeff;
		quadCoeff         = m_ownshipHeadlightsQuadCoeff;
		heightFade        = m_ownshipHeadlightsHeightFade;
		intensity         = m_ownshipHeadlightsIntensity;
		cutOffDist        = m_ownshipHeadlightsCutOffDist;
		lampSeparation    = m_ownshipHeadlightsLampSeparation;
		lampForwardOffset = m_ownshipHeadlightsLampForwardOffset;
	}

	return m_ownshipHeadlightsOn;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns date and time information
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize the Header block.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////bool
void
CScenarioControl::GetDateAndTime(
			int& year,
			int& month,
			int& day,
			int& hour,
			int& minute
			)
{
	year = m_year;
	month = m_month;
	day = m_day;
	hour = m_hour;
	minute = m_minute;
}

void
CScenarioControl::SetDateAndTime(
				int year,
				int month,
				int day,
				int hour,
				int minute
				){
	m_year = year;
	m_month = month;
	m_day =  day;
	m_hour = hour;
	m_minute = minute;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns terrain coordinate information
//
// Remarks:  Must be called after call to InitSimulation so that it has a
//  chance to initialize the Header block.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////bool
void
CScenarioControl::GetTerrainCoordinates(
				float& longitude,
				float& latitude,
				float& altitude
				)
{
	longitude = m_longitude;
	latitude = m_latitude;
	altitude = m_altitude;
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///   Gets the blanking color, the color to set the projects too between runs
///\param [in] r Red   0..1
///\param [in] g Green 0..1
///\param [in] b Blue  0..1
/////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::GetBlankColor(float &r, float &g, float &b){
	r = m_blankColor[0];
	g = m_blankColor[1];
	b = m_blankColor[2];
}
void
CScenarioControl::GetSkyModelValues(
			bool& sunEntityEnabled,
			bool& sunlightEnabled,
			bool& moonEntityEnabled,
			bool& moonlightEnabled,
			float& sunlightIntensity,
			float& moonlightIntensity
			)
{
	sunEntityEnabled = m_sunEntityEnabled;
	sunlightEnabled = m_sunlightEnabled;
	moonEntityEnabled = m_moonEntityEnabled;
	moonlightEnabled = m_moonlightEnabled;
	sunlightIntensity = m_sunlightIntensity;
	moonlightIntensity = m_moonlightIntensity;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the ownvehicle's initial position and heading
//  according to the settings in the SCN file.
//
// Remarks:  If the SCN file header is missing then this function returns
//  0.0 for all values and sets the return value to false.
//
// Arguments:
//  initPos - (output) The ov's initial position (x,y,z) in LRI units.
//  heading - (output) The ov's initial heading (in radians).
//
// Returns:  A boolean indicating if this function is returning valid values.
//
//////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::GetDriverInitState(
			double initPos[3],
			double& heading
			)
{
	if( m_pHdrBlk == 0 )
	{
		initPos[0] = 0.0;
		initPos[1] = 0.0;
		initPos[2] = 0.0;
		heading    = 0.0;
		return false;
	}

	CPoint3D initPosVec = m_pHdrBlk->GetOwnVehPos();
	initPos[0] = initPosVec.m_x;
	initPos[1] = initPosVec.m_y;
	initPos[2] = initPosVec.m_z;

	CVector3D angle = m_pHdrBlk->GetOwnVehOri();
	heading = atan2( angle.m_j, angle.m_i );
	return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///\brief	This function finds all the objects that have to be "pre created" in some maner
///\remark
///		This functions scans every block, and looks for every objects that needs to be pre-created
///		this includes ADOs and DDOs, including those created by Traffic Manager. Since all virtual
///		objects must be pre-created, it gets each virtual object as well. <B>This Function is Recursive</B>
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void CScenarioControl::FindPreCreateObjects(CSnoParser::TIterator pBlock,CSnoParser::TIterator pEndBlock){
	//get all the "Pre-Parse Items
	CSnoParser::TIterator ptBlock;
	//ptBlock = pBlock;
	const CAdoParseBlock *pAdoBlock;
	const CDdoParseBlock *pDdoBlock;
	const CVirtualObjectParseBlock *pVoBlock;
	const CTrafMngrParseBlock * pManagerBlock;
	map<string, set<int> >::iterator modelPair;

	for (int x = 0; pBlock != pEndBlock ;pBlock++){
		string tempName = pBlock->GetBlockName();//GetName();
		if (tempName == "Ado" ){
			if ( pAdoBlock = static_cast<const CAdoParseBlock*>(&(*pBlock))){
				modelPair = m_modelOptionPairs.find(pAdoBlock->GetSolName());
				if (modelPair != m_modelOptionPairs.end()){
					if (modelPair->second.find(pAdoBlock->GetColorIndex()) == modelPair->second.end()){
						modelPair->second.insert(pAdoBlock->GetColorIndex());
					}
				}else{
					m_modelOptionPairs[pAdoBlock->GetSolName()].insert(pAdoBlock->GetColorIndex());
				}
			}
		}else if (tempName == "Ddo" ){
			if ( pDdoBlock = static_cast<const CDdoParseBlock*>(&(*pBlock))) {
				modelPair = m_modelOptionPairs.find(pDdoBlock->GetSolName());
				if (modelPair != m_modelOptionPairs.end()){
					if (modelPair->second.find(pDdoBlock->GetColorIndex()) == modelPair->second.end()){
						modelPair->second.insert(pDdoBlock->GetColorIndex());
					}
				}else{
					m_modelOptionPairs[pDdoBlock->GetSolName()].insert(pDdoBlock->GetColorIndex());
				}
			}
		}
		else if (tempName == "VirtualObject"){
			if (pVoBlock = static_cast<const CVirtualObjectParseBlock*>(&(*pBlock))){
				m_virtualObjects.push_back(*pBlock);
			}
		}
		else if (tempName == "TrafficManager"){
			if ( pManagerBlock = static_cast<const CTrafMngrParseBlock*>(&(*pBlock))) {
				auto sets = pManagerBlock->GetInputSets();
				for( auto itr = sets.begin(); itr != sets.end(); itr++){
					for (auto solItr = itr->solWeights.begin(); solItr!= itr->solWeights.end(); solItr++){
						const CSolObjVehicle* pObj = dynamic_cast<const CSolObjVehicle*>(CCved::GetSol().GetObj(solItr->name));
						if (!pObj)
							continue; //we have some kind of fault here......
						auto colors = pObj->GetNumColors();
						if (colors == 0){//every object has at least one "option"
							colors = 1;
						}
						modelPair = m_modelOptionPairs.find(solItr->name);
						if (modelPair != m_modelOptionPairs.end()){
							for (int i = 0; i < colors; i++){
								if (modelPair->second.find(i) == modelPair->second.end()){
									modelPair->second.insert(i);
								}
							}
						}else{
							for (int i  = 0; i < colors; i++){
								m_modelOptionPairs[solItr->name].insert(i);
							}
						}
					}
				}
			}
		}
		//CSnoBlock::
		FindPreCreateObjects(pBlock->BeginChild(),pBlock->EndChild());
	}
}
///////////////////////////////////////////////////////////////////////////////
///\brief get a copy of every virtual block
//////////////////////////////////////////////////////////////////////////////
void CScenarioControl::GetVirtObjectsBlocks(const vector<CSnoBlock>* &pBlocks) {
		pBlocks = &m_virtualObjects;
}
////////////////////////////////////////////////////////////////////////////////
///\brief get every sol Object:Color pair
///////////////////////////////////////////////////////////////////////////////

void CScenarioControl::GetSolColorNames(const map<string, set<int> >* &pSets){
		pSets = &m_modelOptionPairs;
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///    allow rehearsal mode (ISAT) to modify the external driver
///\remark
///    if a driver surrogate does not exist, this will do nothing, uses
///    target velocity for now
///\param  TargetVelocity -speed to set the target vel to, -1 reset this
///\param  TypeOfVelocity - forced or target velocity indicator
///\param  isStopping - external driver stopping indicator
/////////////////////////////////////////////////////////////////////////////
void CScenarioControl::SetRehearsalSpeedOfExternalDriver(float TargetVelocity,string TypeOfVelocity,bool isStopping){
	stringstream converter;
	if(!isStopping)
		converter<<TargetVelocity<<" "<<5.0;
	else if(isStopping)
		converter<<TargetVelocity<<" "<<8.0;

	auto driver = GetHcsm().GetExtDriverSurrogate();
	if (!driver)
		return;
	if(TypeOfVelocity == "Forced")
	{
		if (TargetVelocity >= 0){
			bool result = driver->SetDialByNameStr( "ForcedVelocity", converter.str() );
		}else if ( TargetVelocity < 0){
			driver->ResetDialByName("ForcedVelocity");
		}
	}
	else if(TypeOfVelocity == "Target")
	{
		if (TargetVelocity >= 0){
			bool result = driver->SetDialByNameStr( "TargetVelocity", converter.str() );
		}else if ( TargetVelocity < 0){
			driver->ResetDialByName("TargetVelocity");
		}
	}


	//bool result = pHcsm->SetDialByNameStr( m_dialName, m_dialValue );
	//if( !result )
	//{
	//	gout << "CSetDialActn::SetDialOnHcsm: unable to set dial";
	//	gout << endl;

	//	return;
	//}

}

/////////////////////////////////////////////////////////////////////////////
///\brief
///    Tell the driver to change lanes right
/////////////////////////////////////////////////////////////////////////////
void CScenarioControl::SetRehearsalChangeLaneRightExternalDriver(){
	auto driver = GetHcsm().GetExtDriverSurrogate();
	if (!driver)
		return;
	driver->SetButtonByName("ChangeLaneRight");
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///    Tell the driver to change lanes left
/////////////////////////////////////////////////////////////////////////////
void CScenarioControl::SetRehearsalChangeLaneLeftExternalDriver(){
	auto driver = GetHcsm().GetExtDriverSurrogate();
	if (!driver)
		return;
	driver->SetButtonByName("ChangeLaneLeft" );
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  The function loads configuration file of distributed system and
//  initializes all necessary data structures so it can execute the distributed
//  scenario.
//
// Remarks:
//
// Arguments:
//  simulateOwnVeh - Should the behaviors simulate a fake ownvehicle?
//
// Returns:  A boolean to indicate if everything initialize okay.  Use the
//  GetLastErrorString() function to get the error message.
//
//////////////////////////////////////////////////////////////////////////////

bool CScenarioControl::InitDistriEDOCtrlSim( bool simulateOwnVeh )
{
	if( m_pExternalObjCtrl ) ReleaseNetworkExternalObjectControl(m_pExternalObjCtrl);
	m_pExternalObjCtrl = CreateNetworkExternalObjectControl(DISVRLINK);
	CSnoParserDistri parser;
	bool initialized = parser.Init();
	if (initialized)
	{
		//
		// Initialize a copy of CVED.  We find the lri name from the
		// first scenario file block which should be a header.
		//
		CSnoParser::TIterator pBlock = parser.Begin();
		bool scenFileError = (
				pBlock == parser.End() ||
				pBlock->GetBlockName() != string( "Header" )
				);
		if( scenFileError )
		{
			sprintf_s(
				m_lastError,
				sizeof(m_lastError),
				"File is incomplete, or first block is not the header."
				);
			return !(m_haveError=true);
		}
		m_pHdrBlk = new CHeaderDistriParseBlock( *pBlock );

		if( m_pCved ) delete m_pCved;
		m_pCved = new CCvedEDOCtrl(m_pExternalObjCtrl);
		m_pCved->Configure( CCved::eCV_SINGLE_USER, m_behavDeltaT, m_dynaMult );
		string cvedErr;
		bool success = m_pCved->Init( m_pHdrBlk->GetLriFile(), cvedErr );
		if( !success )
		{
			sprintf_s( m_lastError, sizeof(m_lastError),"Cved::Init failed: %s", cvedErr.c_str() );
			return !(m_haveError=true);
		}

		initialized = m_pExternalObjCtrl->Initialize(static_cast<CHeaderDistriParseBlock&>(*m_pHdrBlk), m_pCved) //the configuration for localhost simulator will be identified
					&& InitSimulation(parser, simulateOwnVeh);
	}

	if (!initialized)
	{
		ReleaseNetworkExternalObjectControl(m_pExternalObjCtrl);
		m_pExternalObjCtrl = NULL;
	}

	return initialized;
}

bool CScenarioControl::InitDistriEDOCtrlSim(const char* filePath, bool simulateOwnVeh)
{
	if( m_pExternalObjCtrl ) ReleaseNetworkExternalObjectControl(m_pExternalObjCtrl);
	m_pExternalObjCtrl = CreateNetworkExternalObjectControl(DISVRLINK);

	//
	// Create a scenario name and perform translation.
	//
#ifdef _DIFF_DISTRI_SCENE
	m_scnFileName = "Distri_";
	m_scnFileName += filePath;
#else
	m_scnFileName = filePath;
#endif
	bool fileOk = ProcessFile(
						m_scnFileName,
						"NADSSDC_SCN",
						m_scnFileNameFull,
						true
						);
	if( !fileOk )  return false;

	//
	// parse the file using the sno parser
	//
	CSnoParserDistri parser;
	bool		  initialized = true;
	try
	{
		initialized = parser.ParseFile( m_scnFileNameFull.c_str() );
	}
	catch( CSnoParser::TError e )
	{
		sprintf_s(
			m_lastError,
            sizeof(m_lastError),
			"Parser failed with error %s",
			e.msg.c_str()
			);
		initialized = false;
	}
	catch( ... )
	{
		sprintf_s( m_lastError, sizeof(m_lastError), "Parser failed with unknown error" );
		initialized = false;
	}

	if (initialized)
	{
		//
		// Initialize a copy of CVED.  We find the lri name from the
		// first scenario file block which should be a header.
		//
		CSnoParser::TIterator pBlock = parser.Begin();
		bool scenFileError = (
				pBlock == parser.End() ||
				pBlock->GetBlockName() != string( "Header" )
				);
		if( scenFileError )
		{
			sprintf_s(
				m_lastError,
				sizeof(m_lastError),
				"File is incomplete, or first block is not the header."
				);
			return !(m_haveError=true);
		}
		m_pHdrBlk = new CHeaderDistriParseBlock( *pBlock );



		if( m_pCved ) delete m_pCved;
		m_pCved = new CCvedEDOCtrl(m_pExternalObjCtrl);
		m_pCved->Configure( CCved::eCV_SINGLE_USER, m_behavDeltaT, m_dynaMult );
		string cvedErr;
		bool success = m_pCved->Init( m_pHdrBlk->GetLriFile(), cvedErr );
		if( !success )
		{
			sprintf_s( m_lastError, sizeof(m_lastError),"Cved::Init failed: %s", cvedErr.c_str() );
			return !(m_haveError=true);
		}
		initialized = m_pExternalObjCtrl->Initialize(static_cast<CHeaderDistriParseBlock&>(*m_pHdrBlk), m_pCved) //the configuration for localhost simulator will be identified
					&& InitSimulation(parser, simulateOwnVeh);
	}

	if (!initialized)
	{
		ReleaseNetworkExternalObjectControl(m_pExternalObjCtrl);
		m_pExternalObjCtrl = NULL;
	}

	return initialized;
}

bool CScenarioControl::InitSimulation( CSnoParser& parser, bool simulateOwnVeh )
{
	FindPreCreateObjects(parser.Begin(),parser.End());

	//
	// Clean up any prior instances of CVED and HCSM
	//
	//if( m_pCved ) delete m_pCved;
	if( m_pRootColl ) delete m_pRootColl;

	//
	// Check for the existence of the CD1 and summary files and set
	// member variables.
	//
	string lriFileName = m_pHdrBlk->GetLriFile();
    m_lriName = lriFileName;
	basic_string <char>::size_type dotLocation = lriFileName.find_first_of( "." );
	m_lriFileNamePrefix = lriFileName.substr( 0, dotLocation );
	m_scenarioHasBlanking = m_pHdrBlk->GetBlanking();
	m_pHdrBlk->GetBlankColor(m_blankColor[0],m_blankColor[1],m_blankColor[2]);
	if (m_pHdrBlk->GetSirenOverSpeed()){
		//m_sirenSpeed = m_pHdrBlk->GetSirenSpeed() * cMPH_TO_MS;
		CHcsmCollection::m_sSirenSpeed = (m_pHdrBlk->GetSirenSpeed() * (float)cMPH_TO_MS);
	}else{
		//m_sirenSpeed = -1;
		CHcsmCollection::m_sSirenSpeed = -1;
	}

	//
	// Read motion base settings from header.
	//
	CHeaderParseBlock::TMotionPosition motionPos = m_pHdrBlk->GetMotionScenarioPosition();

	CHcsmCollection::m_sSCC_Scen_Pos_X_Crossbeam = (float) motionPos.m_crossbeamX;
	CHcsmCollection::m_sSCC_Scen_Pos_Y_Carriage  = (float) motionPos.m_carriageY;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_X       = (float) motionPos.m_hexX;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Y       = (float) motionPos.m_hexY;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Z       = (float) motionPos.m_hexZ;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Roll    = (float) motionPos.m_hexRoll;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Pitch   = (float) motionPos.m_hexPitch;
	CHcsmCollection::m_sSCC_Scen_Pos_Hex_Yaw     = (float) motionPos.m_hexYaw;
	CHcsmCollection::m_sSCC_Scen_Pos_TT          = (float) motionPos.m_turntable;

#if 1 // DEBUG_MPP
	fprintf(
		stdout,
		"MPP: reading from file (%.1f, %.1f) (%.1f %.1f %.1f) (%.1f %.1f %.1f) %.1f\n",
		CHcsmCollection::m_sSCC_Scen_Pos_X_Crossbeam,
		CHcsmCollection::m_sSCC_Scen_Pos_Y_Carriage,
		CHcsmCollection::m_sSCC_Scen_Pos_Hex_X,
		CHcsmCollection::m_sSCC_Scen_Pos_Hex_Y,
		CHcsmCollection::m_sSCC_Scen_Pos_Hex_Z,
		CHcsmCollection::m_sSCC_Scen_Pos_Hex_Roll,
		CHcsmCollection::m_sSCC_Scen_Pos_Hex_Pitch,
		CHcsmCollection::m_sSCC_Scen_Pos_Hex_Yaw,
		CHcsmCollection::m_sSCC_Scen_Pos_TT
		);
#endif

	//
	// Read the brake conditions from header.
	//
	vector<int> brakeConds = m_pHdrBlk->GetInitialBrakeConditions();
	vector<int>::iterator brakeItr;
	int cnt = 0;
	for( brakeItr = brakeConds.begin(); brakeItr != brakeConds.end(); brakeItr++ )
	{
		int value = *brakeItr;
		if( cnt < cBRAKE_COND_SIZE )
		{
			CHcsmCollection::m_sBrakeCond[cnt++] = value;
		}
		else
		{
			fprintf(
				stdout,
				"CScenarioControl: max brake conditions reached!!  "
				"Increase cBRAKE_COND_SIZE.\n"
				);
			break;
		}
	}

	//
	// Create and initialize CVED.
	//
	//string cvedErr;
	//m_pCved = new CCvedDistri();
	//m_pCved->Configure( CCved::eCV_SINGLE_USER, m_behavDeltaT, m_dynaMult );
	//bool success = m_pCved->Init( m_pHdrBlk->GetLriFile(), cvedErr );
	//if( !success )
	//{
	//	sprintf_s( m_lastError, sizeof(m_lastError),"Cved::Init failed: %s", cvedErr.c_str() );
	//	return !(m_haveError=true);
	//}


	//
	// Set the cab SOL name and object.
	//
	m_cabSolObjName = m_pHdrBlk->GetCabSolObjName();
	if( m_cabSolObjName.size() <= 0 )
	{
		sprintf_s(
			m_lastError,
            sizeof(m_lastError),
			"no cab specified in scenario"
			);
		return !(m_haveError=true);
	}
	m_cpCabSolObj = m_pCved->GetSol().GetObj( m_cabSolObjName );
	if( !m_cpCabSolObj )
	{
		sprintf_s(
			m_lastError,
            sizeof(m_lastError),
			"unable to get SOL info for cab '%s'",
			m_cabSolObjName.c_str()
			);
		return !(m_haveError=true);
	}

	//
	// Set the cab trailer SOL name and object if it exists
	//
	m_cabTrailerSolObjName = m_pHdrBlk->GetTrailerSolObjName();
	if( m_cabTrailerSolObjName.size() > 0 )
	{
		m_cpCabTrailerSolObj = m_pCved->GetSol().GetObj( m_cabTrailerSolObjName );
		if( !m_cpCabTrailerSolObj )
		{
			sprintf_s(
				m_lastError,
                sizeof(m_lastError),
				"unable to get SOL info for cab trailer '%s'",
				m_cabTrailerSolObjName.c_str()
				);
			return !(m_haveError=true);
		}
	}

	//
	// Check to see if the cab is to be shown
	//
	m_showCab = m_pHdrBlk->GetShowCab();

	//
	// Set the Ownship Light information
	//
	m_ownshipHeadlightsOn                = m_pHdrBlk->GetHeadlightsOn();
	m_ownshipHeadlightsAzimuth           = m_pHdrBlk->GetHeadlightsAzimuth();
	m_ownshipHeadlightsElevation         = m_pHdrBlk->GetHeadlightsElevation();
	m_ownshipHeadlightsBeamWidth         = m_pHdrBlk->GetHeadlightsBeamWidth();
	m_ownshipHeadlightsBeamHeight        = m_pHdrBlk->GetHeadlightsBeamHeight();
	m_ownshipHeadlightsConstCoeff        = m_pHdrBlk->GetHeadlightsConstCoeff();
	m_ownshipHeadlightsLinearCoeff       = m_pHdrBlk->GetHeadlightsLinearCoeff();
	m_ownshipHeadlightsQuadCoeff         = m_pHdrBlk->GetHeadlightsQuadCoeff();
	m_ownshipHeadlightsHeightFade        = m_pHdrBlk->GetHeadlightsHeightFade();
	m_ownshipHeadlightsIntensity         = m_pHdrBlk->GetHeadlightsIntensity();
	m_ownshipHeadlightsCutOffDist        = m_pHdrBlk->GetHeadlightsCutOffDist();
	m_ownshipHeadlightsLampSeparation    =m_pHdrBlk->GetHeadlightLampSeparation();
	m_ownshipHeadlightsLampForwardOffset=m_pHdrBlk->GetHeadlightForwardOffset();
	//
	// Set date and time information
	//
	m_year = (short)m_pHdrBlk->GetEnvYear();
	m_month = (short)m_pHdrBlk->GetEnvMonth();
	m_day = (short)m_pHdrBlk->GetEnvDay();
	m_hour = (short)m_pHdrBlk->GetEnvHour();
	m_minute = (short)m_pHdrBlk->GetEnvMinute();

	//
	//
	// Set terrain coordinate information
	m_longitude = (float)m_pHdrBlk->GetEnvLong();
	m_latitude = (float)m_pHdrBlk->GetEnvLat();
	m_altitude = (float)m_pHdrBlk->GetEnvAlt();

	//
	//
	// Set sky model values
	m_sunEntityEnabled = true;
	m_sunlightEnabled = true;
	m_moonEntityEnabled = true;
	m_moonlightEnabled = true;
	m_sunlightIntensity = 100.0;
	m_moonlightIntensity = (float)m_pHdrBlk->GetAmbientLightInt();

	CHcsmCollection::m_sHeadlightScenarioControl = 0;
	//
	// This SCN file has an external driver.  Make an ADO that will
	// represent the ExternalDriver in this scenario.
	//
	ostrstream o;
	o << parser;
	TRACE(TEXT("Before adding blk:%s\n"), o.str());
	if( simulateOwnVeh && m_pHdrBlk->HasOwnVeh() )
	{

		CAdoParseBlock block;
		block.SetName("ExternalDriver");
		block.SetSolName(m_cabSolObjName);
		CRoadPos pos(*m_pCved, m_pHdrBlk->GetOwnVehPos());
		if (pos.IsValid())
		{
			block.SetRoadPos(pos.GetString());
			block.SetPath(m_pHdrBlk->GetPath());
			parser.AddBlock(block);
		}

	}
	else
	{
		m_pCved->SetFakeExternalDriver( false );
	}
	ostrstream o2;
	o2 << parser;
	TRACE(TEXT("After adding blk:%s\n"), o2.str());
	//
	// Now load the initial set of HCSMs
	//
	m_pRootColl = new CHcsmCollection( m_behavDeltaT, m_pCved );

	CSnoParser::TIterator pBlock = parser.Begin();
	for( pBlock++ ; pBlock != parser.End(); pBlock++ )
	{
		if( m_verbose > 0 )
		{
			gout << "[" << m_pRootColl->GetFrame();
			gout << "] Creating hcsm '" << pBlock->GetBlockName() ;
			gout << "'." << endl;
		}

		CHcsm* pH = m_pRootColl->CreateHcsm( pBlock->GetBlockName(), *pBlock );
		if( pH == 0 )
		{
			sprintf_s(
				m_lastError,
                sizeof(m_lastError),
				"CHcsmCollection::CreateHcsm(%s, %s) failed", pBlock->GetName().c_str(),
				pBlock->GetBlockName().c_str()
				);
			return !(m_haveError=true);
		}

//		pH->SetDebugMode( m_DebugMode );
//		pH->SetDebugLevel( m_DebugLevel );

		if( "StaticObjManager" == pH->GetName() )
		{
			CSobjMngrParseBlock block( *pBlock);
			StaticObjManInitialSetup( &block, *m_pCved, *m_pRootColl, pH );
		}
	}

	vector<string> cellNames = m_pHdrBlk->GetWriteCellNames();
	vector<string>::iterator itr;
	for( itr = cellNames.begin(); itr != cellNames.end(); itr++ )
	{
		int currSize = CHcsmCollection::m_sWriteCellDataSize;
		bool haveEnoughSpace = currSize < cMAX_WRITE_CELL_DATA_SIZE;
		if( haveEnoughSpace )
		{
			CHcsmCollection::m_sWriteCellData[currSize].cellName = *itr;
			CHcsmCollection::m_sWriteCellDataSize++;
		}
		else
		{
			gout << "CScenarioControl::InitSimulation: ";
			gout << "need to increase size of cMAX_WRITE_CELL_DATA_SIZE";
			gout << endl;
		}
	}

	//
	// Tell the CHcsmCollection to preload any needed files.
	//
	m_pRootColl->PreloadFiles( m_pHdrBlk->GetPreloadFiles() );

	//
	// Need to do this to ensure that we don't get run-time errors where
	// m_ownVehRoadPos is used later.
	//
	CRoadPos tempRoadPos( *m_pCved );
	m_ownVehRoadPos = tempRoadPos;

	m_intVehRoadPos.SetCved( m_pCved );
	m_intVehRoadPos.SetXYZ( m_pHdrBlk->GetOwnVehPos() );

	if( m_intVehRoadPos.IsValid() )
	{
		CPoint3D pos = m_intVehRoadPos.GetXYZ();
		CHcsmCollection::m_sIsOnPath = true;
		CHcsmCollection::SetLastGoodPosition( (float)pos.m_x, (float)pos.m_y, (float)pos.m_z );
	}

//	CHcsmCollection::m_ownshipPath.SetCved( m_pCved );
	CHcsmCollection::m_sOwnshipPath.SetString( m_pHdrBlk->GetPath() );

	CHcsmCollection::m_sHour   = m_hour;
	CHcsmCollection::m_sMinute = m_minute;

	string visualSettings = m_pHdrBlk->GetVisualSettings();
	CHcsmCollection::SetVisualSettings( visualSettings );

	//clear the variables from the last run
	CHcsmCollection::ClearVariables();

	return !(m_haveError=false);
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  The function loads the specified scenario file and
//  initializes all necessary data structures so it can execute the
//  scenario.
//
// Remarks:
//
// Arguments:
//  cpScenarioFileName - The scenario file's name.
//  simulateOwnVeh - Should the behaviors simulate a fake ownvehicle?
//  isFile - (optional)
//
// Returns:  A boolean to indicate if everything initialize okay.  Use the
//  GetLastErrorString() function to get the error message.
//
//////////////////////////////////////////////////////////////////////////////
bool
CScenarioControl::InitSimulation(
		const char* cpScenarioFileName,
		bool        simulateOwnVeh,
		bool		isFile
		)
{
	//
	// Create a scenario name and perform translation.
	//
	m_scnFileName = cpScenarioFileName;
	bool fileOk = ProcessFile(
						m_scnFileName,
						"NADSSDC_SCN",
						m_scnFileNameFull,
						isFile
						);
	if( !fileOk )  return false;

	//
	// parse the file using the sno parser
	//
	CSnoParser    parser;
	bool		  parseWorked = true;
	try
	{
		if( isFile )
			parser.ParseFile( m_scnFileNameFull.c_str() );
		else
			parser.Parse( cpScenarioFileName );
	}
	catch( CSnoParser::TError e )
	{
		sprintf_s(
			m_lastError,
            sizeof(m_lastError),
			"Parser failed with error %s",
			e.msg.c_str()
			);
		parseWorked = false;
	}
	catch( ... )
	{
		sprintf_s( m_lastError, sizeof(m_lastError), "Parser failed with unknown error" );
		parseWorked = false;
	}

	//
	// Initialize a copy of CVED.  We find the lri name from the
	// first scenario file block which should be a header.
	//
	CSnoParser::TIterator pBlock = parser.Begin();
	bool scenFileError = (
				pBlock == parser.End() ||
				pBlock->GetBlockName() != string( "Header" )
				);
	if( scenFileError )
	{
		sprintf_s(
			m_lastError,
            sizeof(m_lastError),
			"File is incomplete, or first block is not the header."
			);
		return !(m_haveError=true);
	}

	m_pHdrBlk = new CHeaderParseBlock( *pBlock );

	if( m_pCved ) delete m_pCved;
	m_pCved = new CCved();
	m_pCved->Configure( CCved::eCV_SINGLE_USER, m_behavDeltaT, m_dynaMult );
	string cvedErr;
	bool success = m_pCved->Init( m_pHdrBlk->GetLriFile(), cvedErr );
	if( !success )
	{
		sprintf_s( m_lastError, sizeof(m_lastError),"Cved::Init failed: %s", cvedErr.c_str() );
		return !(m_haveError=true);
	}

	return parseWorked
		&& InitSimulation(parser, simulateOwnVeh);
}


/////////////////////////////////////////////////////////////////////////////
//
// Terminate the simulation.  It frees all data structures
//
//
void
CScenarioControl::TerminateSimulation( void )
{
	ResetCollisionDetectionVars();

	if( m_pExternalObjCtrl )
	{
		m_pExternalObjCtrl->UnInitialize(m_pCved);
		ReleaseNetworkExternalObjectControl(m_pExternalObjCtrl);
		m_pExternalObjCtrl = 0;
	}

	if( m_pRootColl )
	{
		delete m_pRootColl;
		m_pRootColl = 0;
	}

	if( m_pCved )
	{
		delete m_pCved;
		m_pCved     = 0;
	}


}


/////////////////////////////////////////////////////////////////////////////
//
// This function provides a straightforward way of obtaining the state
// of a list of objects whose identifiers is listed in the input array.
//
// The function returns -1 in case of errors.
//
int
CScenarioControl::GetObjectState(
			const int	objs[],		// object identifiers
			cvTObjState state[],	// placeholder for state information
			int			count		// size of the arrays
			)
{
	int i;
	for( i = 0; i < count; i++ )
	{
		m_pCved->GetObjStateInstant( objs[i], state[i] );
	}

	return 0;
}



/////////////////////////////////////////////////////////////////////////////
// NON REENTRANT
// This function provides a straightforward way of obtaining the new
// objects, deleted objects and remaining objects after each frame's
// execution.  It should be called only once between invokations of the
// maintainer.  It will copy the identifiers of existing, new and deleted
// objects in the respective buffers.
//
// Only dynamic CVED objects are considered.
//
// The function returns -1 in case of buffer overflow.
//
//
int
CScenarioControl::GetNewAndDeletedDynamicObjects(
		const CObjTypeMask&	cMask,		// input
		int&				currCount,	// count of current objects
		int					currList[],	// list of current objects
		int&				newCount,	// count of new objects
		int					newList[],	// list of new objects
		int&				delCount,	// count of deleted objects
		int					delList[],	// list of deleted objects
		int					bufSize		// size of all provided buffers
		)
{
	static vector<int>  sLatest;		// temporary buffer space, static for perf

	m_pCved->GetAllDynamicObjs(sLatest, cMask);

	unsigned int i, j;
	currCount = newCount = delCount = 0;

	for (i=0; i<sLatest.size(); i++) {
		bool found = false;
		for (j=0; j<m_objFromLastTime.size(); j++) {
			if ( sLatest[i] == m_objFromLastTime[j] ) {
				found = true;
				break;
			}
		}
		if ( found ) {
			if ( currCount == bufSize ) return -1;
			currList[currCount++] = sLatest[i];
		}
		else {
			if ( newCount == bufSize ) return -1;
			newList[newCount++] = sLatest[i];
		}
	}

	for (i=0; i<m_objFromLastTime.size(); i++) {
		bool found = false;
		for (j=0; j<sLatest.size(); j++) {
			if ( m_objFromLastTime[i] == sLatest[j] ) {
				found = true;
				break;
			}
		}
		if ( !found ) {
			if ( delCount == bufSize ) return -1;
			delList[delCount++] = m_objFromLastTime[i];
		}
	}

	m_objFromLastTime = sLatest;
	return 0;
}
int CScenarioControl::GetNewAndDeletedVirtualObjects(
				int&				currCount,	// count of current objects
				int					currList[],	// list of current objects
				int&				newCount,	// count of new objects
				int					newList[],	// list of new objects
				int&				delCount,	// count of deleted objects
				int					delList[],	// list of deleted objects
				int					bufSize
				){
	static vector<int>  sLatest;		// temporary buffer space, static for perf
	CObjTypeMask mask;
	mask.Clear();
	mask.Set(eCV_VIRTUAL_OBJECT);
	m_pCved->GetAllDynamicObjs(sLatest, mask);

	unsigned int i, j;
	currCount = newCount = delCount = 0;

	for (i=0; i<sLatest.size(); i++) {
		bool found = false;
		for (j=0; j<m_virtualObjFromLastTime.size(); j++) {
			if ( sLatest[i] == m_virtualObjFromLastTime[j] ) {
				found = true;
				break;
			}
		}
		if ( found ) {
			if ( currCount == bufSize ) return -1;
			currList[currCount++] = sLatest[i];
		}
		else {
			if ( newCount == bufSize ) return -1;
			newList[newCount++] = sLatest[i];
		}
	}

	for (i=0; i<m_virtualObjFromLastTime.size(); i++) {
		bool found = false;
		for (j=0; j<sLatest.size(); j++) {
			if ( m_virtualObjFromLastTime[i] == sLatest[j] ) {
				found = true;
				break;
			}
		}
		if ( !found ) {
			if ( delCount == bufSize ) return -1;
			delList[delCount++] = m_virtualObjFromLastTime[i];
		}
	}

	m_virtualObjFromLastTime = sLatest;
	return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// This function gets the first debug item from the CHcsmCollection.
//
// The function returns false if there are no debug items to get.
//
bool
CScenarioControl::GetDebugItem(	CHcsmDebugItem& item )
{
	return m_pRootColl->GetDebugItem(item);
}

/////////////////////////////////////////////////////////////////////////////
//
// This function turns the debug mode to the specified state for the
// specified object.
//
void
CScenarioControl::SetDebuggingState(
			int objId,              // Object id to modify the debug state of
			EDebugMode state        // Debug state to change to
			)
{
	CHcsm* pHcsm = m_pRootColl->GetHcsm( objId );
	if( pHcsm )
	{
		pHcsm->SetDebugMode( state );
		pHcsm->SetDebugLevel( CHcsmDebugItem::eDEBUG_ROUTINE );
	}
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Resets the state of the collision detection variables.
//  Should be called during state-mode INIT.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ResetCollisionDetectionVars( void )
{
	m_sCollisionCount = 0;
	m_sTrailerCollisionCount = 0;
	m_sCollisionListSize = 0;
	m_sTrailerCollisionListSize = 0;

	map<int, SCollisionInfo*>::iterator mitr;

	for ( mitr=m_CollisionInfoMap.begin(); mitr!=m_CollisionInfoMap.end(); ++mitr )
		if ( mitr->second != NULL )
			delete mitr->second;
	m_CollisionInfoMap.clear();

	for ( mitr=m_TrailerCollisionInfoMap.begin(); mitr!=m_TrailerCollisionInfoMap.end(); ++mitr )
		if ( mitr->second != NULL )
			delete mitr->second;
	m_TrailerCollisionInfoMap.clear();

	for ( int i=0; i<cMAX_COLLISION_LIST_SIZE; ++i )
	{
		m_sCollisionCvedObjId[i] = -1;
		m_sCollisionCvedObjType[i] = -1;
		m_sCollisionSolObjId[i] = -1;
//		m_sCollisionPrevCvedObjId[i] = -1;
	}
	for ( int i=0; i<cMAX_COLLISION_LIST_SIZE; ++i )
	{
		m_sTrailerCollisionCvedObjId[i] = -1;
		m_sTrailerCollisionCvedObjType[i] = -1;
		m_sTrailerCollisionSolObjId[i] = -1;
//		m_sTrailerCollisionPrevCvedObjId[i] = -1;
	}

#ifdef COLLISION_WITH_TERRAIN
	m_TerrainObjIdToTypeMap.clear();
#endif

}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function updates the current collision object list
// by comparing it with a raw list of current collision objects, and
// then determine if certain objects need to be removed and new ones added
//
// Remarks: The list has a size limit of cMAX_COLLISION_LIST_SIZE
//
// Arguments:
//
//     input/output: collisionObjListSize - size of the current list
//     input/output: collisionObjList - current collision object list
//     input/output: collisionInfo - collision information of the objects in
//                       the current list
//     input:        newCollisionObjs - raw list of latest collision objects
// Returns:
//     number of new collisions, -1 means the list is unchanged, which
//     implicitly indicates there are no new collisions. If the return
//     value is 0, it means there are no new collisions, but the list
//     is changed -- meaning there are objects that are removed from
//     the list.
//
//////////////////////////////////////////////////////////////////////////////
int CScenarioControl::UpdateCollisionObjList(
		int	&collisionObjListSize,
		short collisionObjList[cMAX_COLLISION_LIST_SIZE],
		map<int, CScenarioControl::SCollisionInfo*> &collisionInfo,
		vector<int> &newCollisionObjs
		)
{

#define DEBUG_NEW_COLLISION 1

	int i, newCollisions = 0;
	vector<short>::iterator sitr;
	vector<int>::iterator iitr;
	map<int, SCollisionInfo*>::iterator mitr;
	vector<short> currList;
	bool listChanged = false, inNewList, eraseObj;

	// duplicate the current list to a vector structure
	for ( i=0; i<collisionObjListSize; ++i )
		currList.push_back( collisionObjList[i] );

	// check every object in the current list to see if it's in the new list
	for ( i=0, sitr=currList.begin(); i<collisionObjListSize; ++i )
	{
		mitr = collisionInfo.find(collisionObjList[i]);
		if ( mitr == collisionInfo.end() )
		{
#if DEBUG_NEW_COLLISION > 0
			printf("ScenarioControl: odd, a current collision object does not have collision info.\n");
#endif
			collisionInfo[collisionObjList[i]] = new SCollisionInfo;
			mitr = collisionInfo.find(collisionObjList[i]);
		}

		inNewList = false;
		eraseObj = false;

		for ( iitr=newCollisionObjs.begin(); iitr!=newCollisionObjs.end(); ++iitr )
			if ( *iitr == collisionObjList[i] )
			{
				if ( mitr->second->StartCounting )
				{
					++mitr->second->CollisionCount; // collision count increases
					mitr->second->NonCollisionStreak = 0; // non collision streak ends
				}
				newCollisionObjs.erase( iitr );
				inNewList = true;
				break;
			}

		if ( !inNewList )
		// not in the new collision list, consider taking it off the current list
		{
			mitr->second->StartCounting = true; // start the collision count down if not yet
			++mitr->second->NonCollisionCount;
			++mitr->second->NonCollisionStreak;
			if ( ( mitr->second->NonCollisionStreak >= 3 &&
				mitr->second->NonCollisionCount > mitr->second->CollisionCount*2 ) ||
				mitr->second->NonCollisionStreak >= 10 )
			// end collision condition is satisfied
			{
				// remove object from previous list
				eraseObj = true;
				// remove object from collision info map
				delete collisionInfo[collisionObjList[i]];
				collisionInfo.erase( collisionObjList[i] );
				listChanged = true;
			}
		}

		if ( eraseObj )
			sitr = currList.erase( sitr );
		else
			++sitr;
	}

	// now check the rest of the objects in the new collision list
	for ( iitr=newCollisionObjs.begin(); iitr!=newCollisionObjs.end(); ++iitr )
		if ( currList.size() < cMAX_COLLISION_LIST_SIZE )
		// there is space available to add this new collision object
		{
			currList.push_back( *iitr );
			collisionInfo[*iitr] = new SCollisionInfo;
			listChanged = true;
			++newCollisions;

			if( CHcsmCollection::m_verbose )
			{
				fprintf(
					stdout,
					"***** collision with obj %d %s (cved type = '%s') collisionCount = %d\n",
					*iitr,
					m_pCved->GetObjName(*iitr),
					cvObjType2String( (cvEObjType) (m_pCved->GetObjType(*iitr)) ),
					m_sCollisionCount+newCollisions
					);
			}
		}
		else // list full, ignore the rest of the new collision objects
			break;

	if ( listChanged )
	{
		// update list size
		collisionObjListSize = (int)currList.size();
		// copy the object in the currList vector back to collisionObjList
		for ( sitr=currList.begin(), i=0; sitr!=currList.end(); ++sitr, ++i )
			collisionObjList[i] = *sitr;
		// clean up the rest of the entries in collisionObjList
		for ( i=collisionObjListSize; i<cMAX_COLLISION_LIST_SIZE; ++i )
			collisionObjList[i] = -1;
	}

	if ( listChanged )
		return newCollisions;
	else
		return -1;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function checks for collisions with the ownship and
//  then writes the resulting collision count to the associated static
//  member of the HCSM collection.
//
// Remarks: Only processes the first collision reported by CVED.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::DetectCollisionsWithOwnship( void )
{
	static long sLastCollisionFrame = -1;
	static long sLastTrailerCollisionFrame = -1;
	int i;
	const CSol& sol = m_pCved->GetSol();

#ifdef COLLISION_WITH_TERRAIN
	map<int, short>::iterator mitr;
	// terrain obj type
	// 500: non-collidable terrain (should not appear in the collision object list)
	// 501: wall
	// 502: bumper
	// 503: struct
#endif

	//
	// Check for collisions with the ownvehicle.
	//
	CObjTypeMask objMask(true);
	objMask.Set( eCV_TRAJ_FOLLOWER );
	objMask.Set( eCV_VEHICLE );
	objMask.Set( eCV_TRAILER );
	objMask.Set( eCV_OBSTACLE );
	objMask.Set( eCV_TRAFFIC_SIGN );
	objMask.Set( eCV_WALKER );

#ifdef COLLISION_WITH_TERRAIN
	objMask.Set( eCV_TERRAIN );
#endif

	vector<int> objIdVec, objIdVecModified;
	int numOfObj;
#ifdef COLLISION_WITH_TERRAIN
	vector<int> objIdVecWithTerrain;

	objIdVec.clear();
	objIdVecWithTerrain.clear();
	m_pCved->CollisionDetection( 0, objIdVecWithTerrain, objMask );

	vector<int>::iterator vitr;

	for ( vitr = objIdVecWithTerrain.begin(); vitr != objIdVecWithTerrain.end(); ++vitr )
	{
		if ( m_pCved->GetObjType( *vitr ) == eCV_TERRAIN )
		{
			mitr = m_TerrainObjIdToTypeMap.find(*vitr);
			if ( mitr != m_TerrainObjIdToTypeMap.end() )
			{
				// already in the terrain object to special type map
				// must be collidable
				objIdVec.push_back( *vitr );
			}
			else
			{
				char colObjName[256], colObjNameLC[256];
#if (_MSC_VER > 1500)
    #pragma warning( push )
    #pragma warning(disable:4996)
#endif
				strncpy( colObjName, m_pCved->GetObjName(*vitr),256 );
				strncpy( colObjNameLC, _strlwr(colObjName),256 );
#if (_MSC_VER > 1500)
    #pragma warning( pop )
#endif
				if ( colObjNameLC[0] == 'c' && colObjNameLC[1] == 'd' && colObjNameLC[2] == '_' )
				{
					objIdVec.push_back( *vitr );

					// update the terrain object to special type map
					if ( strstr( colObjNameLC, "wall" ) )
						m_TerrainObjIdToTypeMap[*vitr] = 501;
					else if ( strstr( colObjNameLC, "bumper" ) )
						m_TerrainObjIdToTypeMap[*vitr] = 502;
					else if ( strstr( colObjNameLC, "struct" ) )
						m_TerrainObjIdToTypeMap[*vitr] = 503;
				}
			}
		}
		else
		{
			// non-terrain object, add to the collision list
			objIdVec.push_back( *vitr );
		}
	}
	numOfObj = (int)objIdVec.size();
#else
	numOfObj = m_pCved->CollisionDetection( 0, objIdVec, objMask );
#endif

	objIdVecModified.clear();
	for ( vitr = objIdVec.begin(); vitr != objIdVec.end(); ++vitr )
	{
		int solId = m_pCved->GetObjSolId( *vitr );
		const CSolObj* cpSolObj = sol.GetObj( solId );
		if ( m_pCved->GetObjType( *vitr ) == eCV_OBSTACLE )
		{
			// filter out crosswalk obstacles
			if ( cpSolObj )
			{
				const string& solName = cpSolObj->GetName();
				if ( strstr( solName.c_str(), "SOL_" ) )
				{
					continue;
				}
			}
		}

		if ( cpSolObj )
		{
			int collisionSoundId = cpSolObj->GetCollisionSoundID();
			if ( collisionSoundId == -1 )  // if collision sound ID is set to -1, filter out the collision
			{
				continue;
				// if collision sound ID is set to -2, there should be no collision effect as well, but the collision
				// will be recorded and not filtered out here
			}
		}

		objIdVecModified.push_back( *vitr );
	}

	int cvedId;
	short objType;

	int newCollisions = UpdateCollisionObjList(
		m_sCollisionListSize, m_sCollisionCvedObjId,
		m_CollisionInfoMap, objIdVecModified);

	for ( i=0; i<m_sCollisionListSize; ++i )
	{
		cvedId = m_sCollisionCvedObjId[i];
		objType = m_pCved->GetObjType( cvedId);
#ifdef COLLISION_WITH_TERRAIN
		if ( objType == eCV_TERRAIN )
		{
			mitr = m_TerrainObjIdToTypeMap.find(cvedId);
			if ( mitr != m_TerrainObjIdToTypeMap.end() )
				objType = mitr->second;
		}
#endif
		m_sCollisionCvedObjType[i] = objType;
		const CSolObj* pObj = sol.GetObj(m_pCved->GetObjSolId( cvedId ));
		if (pObj)
			m_sCollisionSolObjId[i] = pObj->GetId();
	}
	for ( i=m_sCollisionListSize; i<cMAX_COLLISION_LIST_SIZE; ++i )
	{
		m_sCollisionCvedObjType[i] = -1;
		m_sCollisionSolObjId[i] = -1;
	}

	if ( newCollisions > 0 )
	{
		m_sCollisionCount += newCollisions;
		sLastCollisionFrame = GetFrame();
	}

	//
	// Now check for collisions with the external trailer, if there is one.
	//

	// check to see if external trailer is valid
	if ( !m_pCved->IsObjValid( 1 ) )
		return;

#ifdef COLLISION_WITH_TERRAIN
	objIdVec.clear();
	objIdVecWithTerrain.clear();
	m_pCved->CollisionDetection( 1, objIdVecWithTerrain, objMask );


	for ( vitr = objIdVecWithTerrain.begin(); vitr != objIdVecWithTerrain.end(); ++vitr )
	{
		if ( m_pCved->GetObjType( *vitr ) == eCV_TERRAIN )
		{
			mitr = m_TerrainObjIdToTypeMap.find(*vitr);
			if ( mitr != m_TerrainObjIdToTypeMap.end() )
			{
				// already in the terrain object to special type map
				// must be collidable
				objIdVec.push_back( *vitr );
			}
			else
			{
				char colObjName[256], colObjNameLC[256];
#if (_MSC_VER > 1500)
    #pragma warning( push )
    #pragma warning(disable:4996)
#endif
				strncpy( colObjName, m_pCved->GetObjName(*vitr),256 );
				strncpy( colObjNameLC, _strlwr(colObjName),256 );
#if (_MSC_VER > 1500)
    #pragma warning( pop )
#endif
				if ( colObjNameLC[0] == 'c' && colObjNameLC[1] == 'd' && colObjNameLC[2] == '_' )
				{
					// terrain object marked with "cd_", add to the collision list
					objIdVec.push_back( *vitr );

					// update the terrain object to special type map
					if ( strstr( colObjNameLC, "wall" ) )
						m_TerrainObjIdToTypeMap[*vitr] = 501;
					else if ( strstr( colObjNameLC, "bumper" ) )
						m_TerrainObjIdToTypeMap[*vitr] = 502;
					else if ( strstr( colObjNameLC, "struct" ) )
						m_TerrainObjIdToTypeMap[*vitr] = 503;
				}
			}
		}
		else
		{
			// non-terrain object, add to the collision list
			objIdVec.push_back( *vitr );
		}
	}
	numOfObj = (int)objIdVec.size();
#else
	numOfObj = m_pCved->CollisionDetection( 1, objIdVec, objMask );
#endif

	objIdVecModified.clear();
	for ( vitr = objIdVec.begin(); vitr != objIdVec.end(); ++vitr )
	{
		int solId = m_pCved->GetObjSolId( *vitr );
		const CSolObj* cpSolObj = sol.GetObj( solId );
		if ( m_pCved->GetObjType( *vitr ) == eCV_OBSTACLE )
		{
			// filter out crosswalk obstacles
			if ( cpSolObj )
			{
				const string& solName = cpSolObj->GetName();
				if ( strstr( solName.c_str(), "SOL_" ) )
				{
//					printf("Filtered out collision with crosswalk object, sol Id %d name %s, obj Id %d\n",
//						solId,
//						solName.c_str(), *vitr
//						);
					continue;
				}
			}
		}

		if ( cpSolObj )
		{
			int collisionSoundId = cpSolObj->GetCollisionSoundID();
//	 		printf("SOL obj #%d collision sound id is %d\n",
//				solId, collisionSoundId);
			if ( collisionSoundId == -1 )  // if collision sound ID is set to -1, filter out the collision
			{
				// if collision sound ID is set to -2, there should be no collision effect as well, but the collision
				// will be recorded and not filtered out here
				continue;
			}
		}

		objIdVecModified.push_back( *vitr );
	}

	newCollisions = UpdateCollisionObjList(
		m_sTrailerCollisionListSize, m_sTrailerCollisionCvedObjId,
		m_TrailerCollisionInfoMap, objIdVecModified);

	for ( i=0; i<m_sTrailerCollisionListSize; ++i )
	{
		cvedId = m_sTrailerCollisionCvedObjId[i];
		objType = m_pCved->GetObjType( cvedId );
#ifdef COLLISION_WITH_TERRAIN
		if ( objType == eCV_TERRAIN )
		{
			mitr = m_TerrainObjIdToTypeMap.find(cvedId);
			if ( mitr != m_TerrainObjIdToTypeMap.end() )
				objType = mitr->second;
		}
#endif
		m_sTrailerCollisionCvedObjType[i] = objType;
		const CSolObj* pObj = sol.GetObj(m_pCved->GetObjSolId( cvedId ));
		if (pObj)
			m_sTrailerCollisionSolObjId[i] = pObj->GetId();
	}
	for ( i=m_sTrailerCollisionListSize; i<cMAX_COLLISION_LIST_SIZE; ++i )
	{
		m_sTrailerCollisionCvedObjType[i] = -1;
		m_sTrailerCollisionSolObjId[i] = -1;
	}

	if ( newCollisions > 0 )
	{
		m_sTrailerCollisionCount += newCollisions;
		sLastTrailerCollisionFrame = GetFrame();
	}
}




void
StringFlip( const volatile char from[], char to[], int size )
{
	int i;
	int rem = size % 4;

	if( rem )
	{
		for( i=0; i<size-4; i+=4 )
		{
			to[i+3]  = from[i];
			to[i+2]  = from[i+1];
			to[i+1]  = from[i+2];
			to[i]    = from[i+3];
		}
		if( rem == 1 )
		{
			to[i] = from[i];
		}
		else if( rem == 2 )
		{
			to[i] = from[i];
			to[i+1] = from[i+1];
		}
		else if( rem == 3 )
		{
			to[i] = from[i];
			to[i+1] = from[i+1];
			to[i+2] = from[i+2];
		}
	}
	else
	{
		for( i=0; i<size; i+=4 )
		{
			to[i+3]  = from[i];
			to[i+2]  = from[i+1];
			to[i+1]  = from[i+2];
			to[i]    = from[i+3];
		}
	}
}

void
StrncpyFlip( char* pDst, const char* cpSrc, int size )
{
	if( (size % 4) != 0 )
	{
		assert( 0 );
	}

	StringFlip( cpSrc, pDst, size );
}


struct TObjData
{
	int cvedId;
	double distFromDriver;
};

static int
CompareDists( const void* cpElem1, const void* cpElem2 )
{
	return (int) (
		((TObjData*)cpElem1)->distFromDriver -
		((TObjData*)cpElem2)->distFromDriver
		);
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets information about the n closest traffic lights to the
//  ownvehicle.
//
// Remarks:
//
// Arguments:
//   ownVehCartPos - The own vehicle's position.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeTrafLightData( const CPoint3D& cOwnVehCartPos )
{
	//
	// Get the traffic lights.
	//
	int tlId[cTRAF_LIGHT_MAX_SIZE];
	int tlIdSize = m_pCved->GetTrafLightsNear(
					cOwnVehCartPos,
					&tlId[0],
					cTRAF_LIGHT_MAX_SIZE
					);
	if( tlIdSize > cTRAF_LIGHT_MAX_SIZE)  tlIdSize = cTRAF_LIGHT_MAX_SIZE;

	int i;
	for( i = 0; i < tlIdSize; i++ )
	{
		int cvedId =  tlId[i];
		int state = m_pCved->GetTrafficLightState( cvedId );

		m_sTrafLightId[i]    = (short) cvedId;
		m_sTrafLightState[i] = (short) state;
	}

	m_sTrafLightSize = tlIdSize;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets information about the n closest objects to the
//  ownvehicle.
//
// Remarks:
//
// Arguments:
//   ownVehCartPos - The own vehicle's position.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeDynObjData( const CPoint3D& cOwnVehCartPos )
{
	//
	// Build the mask.
	//
	CObjTypeMask objMask(true);
	objMask.Set( eCV_TRAJ_FOLLOWER );
	objMask.Set( eCV_VEHICLE );

	//
	// Get all dynamic objects that satisfy the criteria in the mask.
	//
	CCved::TIntVec objs;
	m_pCved->GetAllDynamicObjs( objs, objMask );

	//
	// Iterate through objs vector and make a list of
	// objs that are closest to the driver object
	//
	CPoint3D objPos;
	CCved::TIntVec::iterator i;

	// Structure to hold cvedId and respective distance^2 from driver
	TObjData objData[cNUM_DYN_OBJS];
	int objCntr = 0;

	// storing object distances
	for( i = objs.begin(); i != objs.end(); i++ )
	{
		int cvedId = *i;
		objPos = m_pCved->GetObjPos( cvedId );

		// calculate the distance btwn ADO's and driver obj.
		double sqrdDist = (
			( cOwnVehCartPos.m_x - objPos.m_x ) * ( cOwnVehCartPos.m_x - objPos.m_x ) +
			( cOwnVehCartPos.m_y - objPos.m_y ) * ( cOwnVehCartPos.m_y - objPos.m_y )
			);

		objData[objCntr].distFromDriver = sqrdDist;
		objData[objCntr].cvedId = cvedId;
		objCntr++;
	}

	//
	// quick sort the dist array.
	//
	qsort( objData, objCntr, sizeof(TObjData), CompareDists );

	memset( &(CHcsmCollection::m_sDynObjData), 0, sizeof(CHcsmCollection::m_sDynObjData) );

	//
	// Iterate through each dynamic object.
	//
	int arrSize = 0;
	CVector3D lat;
	CVector3D nrm;
	CVector3D tan;
	int cntr;
	for( cntr = 0; cntr < min(objCntr, cMAX_DYN_OBJ); cntr++ )
	{
		int cvedId = objData[cntr].cvedId;
		const CObj* pDynObj = m_pCved->BindObjIdToClass( cvedId );
		if( !pDynObj )  continue;
		cvEObjType objType = m_pCved->GetObjType( cvedId);

		objPos = pDynObj->GetPosImm();

		CHcsmCollection::m_sDynObjData.vel[ arrSize ] = (float) pDynObj->GetVelImm();
		if (objType == eCV_TRAJ_FOLLOWER) //DDO's use FPS, convert to M/S
			CHcsmCollection::m_sDynObjData.vel[ arrSize ] *= (float)cFEET_TO_METER;


		CHcsmCollection::m_sDynObjData.cvedId[ arrSize ] = cvedId;
		CHcsmCollection::m_sDynObjData.solId[ arrSize ] = m_pCved->GetObjSolId( cvedId );
		CHcsmCollection::m_sDynObjData.hcsmTypeId[ arrSize ] = m_pCved->GetObjHcsmTypeId( cvedId );
		CHcsmCollection::m_sDynObjData.colorIndex[ arrSize ] = (short) m_pCved->GetObjColorIndex( cvedId );

		char temp[cMAX_DYN_OBJ_NAME_SIZE] = { 0 };
		StrncpyFlip(
			temp,
			m_pCved->GetObjName( cvedId ),
			cMAX_DYN_OBJ_NAME_SIZE
			);
		memcpy(
			CHcsmCollection::m_sDynObjData.name + arrSize * cMAX_DYN_OBJ_NAME_SIZE,
			temp,
			cMAX_DYN_OBJ_NAME_SIZE
			);

		CHcsmCollection::m_sDynObjData.pos[ arrSize*3 ]    = (float) objPos.m_x;
		CHcsmCollection::m_sDynObjData.pos[ arrSize*3 + 1] = (float) objPos.m_y;
		CHcsmCollection::m_sDynObjData.pos[ arrSize*3 + 2] = (float) objPos.m_z;

        auto target     = pDynObj->GetPosTarget();
        auto targetacc  = pDynObj->GetAccelTarget();
        auto targetvell = pDynObj->GetVelTarget();

        CHcsmCollection::m_sDynObjData.tarPos[arrSize*3  ]    = (float)target.m_x;
        CHcsmCollection::m_sDynObjData.tarPos[arrSize*3+1]    = (float)target.m_y;
        CHcsmCollection::m_sDynObjData.tarPos[arrSize*3+2]    = (float)target.m_z;
        CHcsmCollection::m_sDynObjData.tarAccMs2[arrSize]     = (float)targetacc;
        CHcsmCollection::m_sDynObjData.tarVelMs[arrSize]      = (float)targetvell;
		lat = pDynObj->GetLat();
		lat.m_i = -lat.m_i;
		lat.m_j = -lat.m_j;
		lat.m_k = -lat.m_k;

		CVector3D tng = pDynObj->GetTan();
		CVector3D nrm;
		nrm.m_i = ( tng.m_j * lat.m_k ) - ( tng.m_k * lat.m_j );
		nrm.m_j = ( tng.m_k * lat.m_i ) - ( tng.m_i * lat.m_k );
		nrm.m_k = ( tng.m_i * lat.m_j ) - ( tng.m_j * lat.m_i );

		double roll  = atan2( lat.m_k, nrm.m_k );
		double pitch = asin( tng.m_k );
		double yaw   = atan2( tng.m_j, tng.m_i );

		CHcsmCollection::m_sDynObjData.heading[ arrSize ] = (float) yaw;
		CHcsmCollection::m_sDynObjData.rollPitch[ arrSize*2 ] = (float) roll;
		CHcsmCollection::m_sDynObjData.rollPitch[ arrSize*2 + 1 ] = (float) pitch;


		if( objType == eCV_TRAJ_FOLLOWER )
		{
			const CTrajFollowerObj* pTrajFollower = dynamic_cast<const CTrajFollowerObj*>( pDynObj );

			CHcsmCollection::m_sDynObjData.audioVisualState[ arrSize ] =
				((pTrajFollower->GetAudioState() & 0xFFFF) << 16 )
				+ (pTrajFollower->GetVisualState() & 0xFFFF);
		}
		else if( objType == eCV_VEHICLE )
		{
			const CVehicleObj* pVehicleObj = dynamic_cast<const CVehicleObj*>( pDynObj );

			CHcsmCollection::m_sDynObjData.audioVisualState[ arrSize ] =
				((pVehicleObj->GetAudioState() & 0xFFFF) << 16 )
				+ (pVehicleObj->GetVisualState() & 0xFFFF);
		}

		bool haveEnoughRoom = arrSize < cMAX_DYN_OBJ -1;
		if( haveEnoughRoom )
		{
			arrSize++;
		}
		else
		{
			break;
		}
	}

	CHcsmCollection::m_sDynObjDataSize = arrSize;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets information about the n closest static objects to the
//  ownvehicle.
//
// Remarks:
//
// Arguments:
//   ownVehCartPos - The own vehicle's position.
//
//   onlyObjWithNonZeroAVState - Whether only static objects with non-zero AV
//       states are computed. The default value is true
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeStatObjData( const CPoint3D& cOwnVehCartPos, bool onlyObjWithNonZeroAVState )
{

	//
	// Get all static objects.
	//
	CCved::TIntVec objs;
	m_pCved->GetAllStaticObjs(objs);

//	printf("ComputeStatObjData: Got %d static objects\n", objs.size());

	if ( objs.empty() )
	{
		CHcsmCollection::m_sStatObjDataSize = 0;
		return;
	}

	//
	// Iterate through objs vector and make a list of
	// objs that are closest to the driver object
	//
	CPoint3D objPos;
	cvTObjState state;
	CCved::TIntVec::iterator i;

	// Structure to hold cvedId and respective distance^2 from driver
	TObjData* objData = new TObjData[objs.size()];
	int objCntr = 0;

	// storing object distances
	for( i = objs.begin(); i != objs.end(); i++ )
	{
		int cvedId = *i;
		m_pCved->GetObjStateInstant( cvedId, state );

		if ( onlyObjWithNonZeroAVState &&
			(state.anyState.audioState & 0xFFFF)==0 && (state.anyState.visualState & 0xFFFF)==0 )
			// skip this one because it has zero AV state and we are only reporting non zero-AV static objects
			continue;

		objPos = state.anyState.position;

		// calculate the distance btwn ADO's and driver obj.
		double sqrdDist = (
			( cOwnVehCartPos.m_x - objPos.m_x ) * ( cOwnVehCartPos.m_x - objPos.m_x ) +
			( cOwnVehCartPos.m_y - objPos.m_y ) * ( cOwnVehCartPos.m_y - objPos.m_y )
			);

		objData[objCntr].distFromDriver = sqrdDist;
		objData[objCntr].cvedId = cvedId;
		objCntr++;
	}

	//
	// quick sort the dist array.
	//
	qsort( objData, objCntr, sizeof(TObjData), CompareDists );

	memset( &(CHcsmCollection::m_sStatObjData), 0, sizeof(CHcsmCollection::m_sStatObjData) );

	//
	// Iterate through each static object.
	//
	int arrSize = 0;
	CVector3D lat;
	CVector3D nrm;
	CVector3D tan;
	int cntr;

	for( cntr = 0; cntr < objCntr; cntr++ )
	{
		int cvedId = objData[cntr].cvedId;
		m_pCved->GetObjStateInstant( cvedId, state );

		objPos = state.anyState.position;
		tan = state.anyState.tangent;
		lat = state.anyState.lateral;

		CHcsmCollection::m_sStatObjData.cvedId[ arrSize ] = cvedId;
		CHcsmCollection::m_sStatObjData.solId[ arrSize ] = m_pCved->GetObjSolId( cvedId );
		CHcsmCollection::m_sStatObjData.hcsmTypeId[ arrSize ] = m_pCved->GetObjHcsmTypeId( cvedId );

		char temp[cMAX_STAT_OBJ_NAME_SIZE] = { 0 };
		StrncpyFlip(
			temp,
			m_pCved->GetObjName( cvedId ),
			cMAX_STAT_OBJ_NAME_SIZE
			);
		memcpy(
			CHcsmCollection::m_sStatObjData.name + arrSize * cMAX_STAT_OBJ_NAME_SIZE,
			temp,
			cMAX_STAT_OBJ_NAME_SIZE
			);

		CHcsmCollection::m_sStatObjData.pos[ arrSize*3 ] = (float) objPos.m_x;
		CHcsmCollection::m_sStatObjData.pos[ arrSize*3 + 1] = (float) objPos.m_y;
		CHcsmCollection::m_sStatObjData.pos[ arrSize*3 + 2] = (float) objPos.m_z;

		lat.m_i = -lat.m_i;
		lat.m_j = -lat.m_j;
		lat.m_k = -lat.m_k;

		nrm.m_i = ( tan.m_j * lat.m_k ) - ( tan.m_k * lat.m_j );
		nrm.m_j = ( tan.m_k * lat.m_i ) - ( tan.m_i * lat.m_k );
		nrm.m_k = ( tan.m_i * lat.m_j ) - ( tan.m_j * lat.m_i );

		double roll  = atan2( lat.m_k, nrm.m_k );
		double pitch = asin( tan.m_k );
		double yaw   = atan2( tan.m_j, tan.m_i );

		CHcsmCollection::m_sStatObjData.heading[ arrSize ] = (float) yaw;
		CHcsmCollection::m_sStatObjData.rollPitch[ arrSize*2 ] = (float) roll;
		CHcsmCollection::m_sStatObjData.rollPitch[ arrSize*2 + 1 ] = (float) pitch;

		CHcsmCollection::m_sStatObjData.audioVisualState[ arrSize ] =
			((state.anyState.audioState & 0xFFFF) << 16 )
			+ (state.anyState.visualState & 0xFFFF);

		bool haveEnoughRoom = arrSize < cMAX_STAT_OBJ-1;
		if( haveEnoughRoom )
		{
			arrSize++;
		}
		else
		{
			break;
		}
	}

	delete [] objData;

	CHcsmCollection::m_sStatObjDataSize = arrSize;
	//now lets get the list of changed static objects
	//CHcsmCollection::m_sChangedStatObjOptionId = 0;
	CHcsmCollection::m_sChangedStatObjDataSize = 0;

	CVED::CCved::TIntVec objItr;
    // Define a template class deque of int
    typedef deque<int, allocator<int> > IntDeque ;
	 IntDeque NumbersDeque(2 * 24) ;
	CPoint3D origin(0.0, 0.0, 0.0);
	CObjTypeMask mask;
	mask.SetAll();
	m_pCved->GetChangedStaticObjsNear(cOwnVehCartPos,10000,5,objItr);
	CHcsmCollection::m_sChangedStatObjDataSize = min(objItr.size(),size_t(cMAX_CHANGED_STAT_OBJ));
	int option = -1;
	for (int i = 0; i < CHcsmCollection::m_sChangedStatObjDataSize; i++){
		if (m_pCved->GetObjOption(objItr[i],option)){
			CHcsmCollection::m_sChangedStatObjOption[i] = option;
		}else{
			CHcsmCollection::m_sChangedStatObjOption[i] = -1; //invalid id
		}
		CHcsmCollection::m_sChangedStatObjId[i] = objItr[i];
	}

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the default values for the environment parameters.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::SetDefaultEnvValues( void )
{
	m_sVisibility = 100.0f;
	m_sWindSpeed = 0.0f;
	m_sWindDirection[0] = 0.0f;
	m_sWindDirection[1] = 0.0f;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets the environmental conditions around the given
//   position and writes the information to global variables.
//
// Remarks: Only handles "visibility" and "wind" for now.
//
// Arguments:
//   ownVehCartPos - The own vehicle's position.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeEnvironmentalConditions(
			const CPoint3D& cOwnVehCartPos
			)
{
	SetDefaultEnvValues();

	//
	// Get the current environmental area given the own vehicle's
	// position.
	//
	vector<CEnvArea> envArea;
	m_pCved->GetEnvArea( cOwnVehCartPos, envArea );
	if( envArea.empty() )  return;

	//
	// See which environemental conditions are active in this area.
	//
	vector<cvTEnviroInfo>::const_iterator xx;
	vector<CEnvArea>::const_iterator yy;
	vector<cvTEnviroInfo> conditions;
	for( yy = envArea.begin(); yy < envArea.end(); yy++ )
	{
		yy->GetCndtns( conditions );
		for( xx = conditions.begin(); xx != conditions.end(); xx++ )
		{
			switch( xx->type )
			{
			case eVISIBILITY:
				m_sVisibility = (float) xx->info.Visibility.dist;
				break;
			case eWIND:
				m_sWindSpeed        = (float) xx->info.Wind.vel;
				m_sWindDirection[0] = (float) xx->info.Wind.dir_i;
				m_sWindDirection[1] = (float) xx->info.Wind.dir_j;
				break;
#if 0
			case eLIGHTNING:
				if( xx->info.Lightning.degree > g_Lightning )
				{
					g_Lightning = xx->info.Lightning.degree;
				}
				break;
			case eHAZE:
				if( xx->info.Haze.dist < g_Haze )
				{
					g_Haze = xx->info.Haze.dist;
				}
				break;
			case eFOG:
				/*
				if( xx->info.Fog.dist < g_Fog )
				{
					g_Fog = xx->info.Fog.dist;
				}
				*/
				break;
			case eSMOKE:
				if( xx->info.Smoke.degree < g_Smoke )
				{
					g_Smoke = xx->info.Smoke.degree;
				}
				break;
			case eCLOUDS:
				if( xx->info.Clouds.altitude < g_Clouds[1] )
				{
					g_Clouds[0] = xx->info.Clouds.type;
					g_Clouds[1] = (int)xx->info.Clouds.altitude;
				}
				break;
			case eGLARE:
				if( xx->info.Glare.degree < g_Glare )
				{
					g_Glare = xx->info.Glare.degree;
				}
				break;
			case eSNOW:
				if( xx->info.Snow.degree > g_Precipitation[1] )
				{
					g_Precipitation[1] = xx->info.Snow.degree;
				}
				break;
			case eRAIN:
				if( xx->info.Rain.degree > g_Precipitation[0] )
				{
					g_Precipitation[0] = xx->info.Rain.degree;
				}
				break;
			}
#endif
			default:
//				fprintf( stdout, "Nix: unknown environment condition %d\n", xx->type );
				break;
			}
		}
	}
}


#if 0
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Writes information about the lead object to the global
//   variables.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeOwnVehToLeadObjDist( void )
{
	//
	// Calculate the distance from the front of the OwnVehicle to the rear
	// of the lead vehicle.
	//
	int leadObjId;
	double distToLeadObj;
	bool success = m_pCved->GetLeadObj( 0, leadObjId, distToLeadObj );

	if( success )
	{
		//
		// Found a lead object.  Since it returns the distance from CG to
		// CG, we have subtract one-half of the OwnVehicle's and lead object's
		// distances from the distance returned.
		//
		double leadObjLength = m_pCved->GetObjLength( leadObjId );
		double cabLength = m_cpCabSolObj->GetLength();
		double adjustDist = ( leadObjLength * 0.5f ) + ( cabLength * 0.5f );
		m_sOwnVehToLeadObjDist = (float) ( distToLeadObj - adjustDist );
	}
	else
	{
		m_sOwnVehToLeadObjDist = -1.0f;
	}
}
#endif


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the ownvehicle's lead vehicle information as long
//  as the ownvehicle is on a road and not on an intersection.
//
// Remarks: Writes data to the static member variable called
//  m_sFollowInfo.  This is array of 9 floats where each element
//  has the following info:
//   [0] -> identifier of object, -1 if none or error, 0 if no ownvehicle
//   [1] -> distance to lead vehicle (in feet)
//   [2] -> bumper-to-bumper time to lead vehicle (in seconds)
//   [3] -> bumper-to-bumper distance to lead vehicle (in feet)
//   [4] -> time-to-collision (in seconds)
//   [5] -> lead vehicle velocity (ft/s)
//   element 6..8 -> x,y,z coords of lead vehicle
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeLeadVehInfo( const CPoint3D& cOwnVehCartPos )
{
	//
	// Calculate the distance from the front of the OwnVehicle to the rear
	// of the lead vehicle.
	//
	int leadObjId;
	double distToLeadObj;
    bool isGoingCorrectDir = true;
	CHcsmCollection::m_sFollowInfo[0] = -2.0f;
    CRoadPos ownVehPos(*m_pCved,cOwnVehCartPos);
    if (!ownVehPos.IsValid()){
		m_sOwnVehToLeadObjDist = -1.0f;
		CHcsmCollection::m_sFollowInfo[0] = -1.0f;
		// these are being set since the titler is not able to look at
		// element 0 to see if the values in elements 1 and 4 are valid.
		CHcsmCollection::m_sFollowInfo[1] = 0.0f;
		CHcsmCollection::m_sFollowInfo[3] = 0.0f;
        return;
    }
    bool useSpecedCrdr = false; //if we are facing the wrong way on a corridor because of overlap, we may
                              // have create a path temporarly........
    int prefCrdr = -1; //neg -1 means no prefence.
    CVED::CPath tempPath(*m_pCved);
	CVector3D ovTangent;
	if (m_pCved->GetOwnVehicleTan( ovTangent )){
        CVector3D ovFrontv = ovTangent + CVector3D(cOwnVehCartPos.m_x,cOwnVehCartPos.m_y,cOwnVehCartPos.m_z);
        CPoint3D ovFrontp(ovFrontv.m_i,ovFrontv.m_j,ovFrontv.m_k);
	    CRoadPos ovFrontrp(*m_pCved,ovFrontp);
        if (ovFrontrp.IsValid()){ // we are facing backwards
            float dir = (float)ovFrontrp.GetDistance() - (float)ownVehPos.GetDistance();
            if (ovFrontrp.IsRoad() && ownVehPos.IsRoad()){
                if (ovFrontrp.GetRoad().GetId() == ownVehPos.GetRoad().GetId()){
                    if (((dir > 0 && ovFrontrp.GetLane().GetDirection() == eNEG) ||
                         (dir < 0 && ovFrontrp.GetLane().GetDirection() == ePOS) )){
                        isGoingCorrectDir = false;
                    }
                }else{
                    //we need to handle this edge condition.........
                    isGoingCorrectDir= true;
                }
            }else if (!ovFrontrp.IsRoad() && ! ownVehPos.IsRoad() && dir < 0){
                if (!CHcsmCollection::m_sOwnshipPath.IsValid()){
                    //if we have a path, the path should resolve to the correct cooridor for us...
                    //but since we do not have one, we need to build a resolve one for our selves...
                    vector< pair<int,double> > corridors;
                    vector< pair<int,double> > front_corridors;
                    bool corridor_found = false;;
                    //now lets try to find the corridor that we are facing the correct dirrection
                    //we are going to require that our center and front have the same number of corridors because for
                    //now just to eliminate having to handle alot more edge conditions......
                    if (ownVehPos.GetCorridors(corridors) && ovFrontrp.GetCorridors( front_corridors ) &&
                        corridors.size() == front_corridors.size() ){
                            for (unsigned int i =0; i < corridors.size(); i ++){
                                if (corridors[i].first == front_corridors[i].first &&
                                    front_corridors[i].second > corridors[i].second){
                                        //we have a winner....we are going in the correct dirrections, and
                                        prefCrdr = front_corridors[i].first;
                                        corridor_found = true;
                                        useSpecedCrdr = true;
                                        break;
                                }
                            }
                            if (!corridor_found){
                                isGoingCorrectDir = false;
                            }
                    }else{
                        isGoingCorrectDir = false;
                    }
                }
                //isGoingCorrectDir = false;
            }
        }
    }
    bool success = false;
    if (useSpecedCrdr){
         success = m_pCved->GetLeadObj( 0, leadObjId, distToLeadObj, NULL,isGoingCorrectDir,prefCrdr );
    }else{
        success = m_pCved->GetLeadObj( 0, leadObjId, distToLeadObj, &CHcsmCollection::m_sOwnshipPath,isGoingCorrectDir );
    }
	CHcsmCollection::m_sFollowInfo[0] = -2;
    memset(
		&CHcsmCollection::m_sFollowInfo[1],
		0,
		sizeof( CHcsmCollection::m_sFollowInfo[0] )  * 8
		);

	if( success )
	{
		CPoint3D leadVehPos = m_pCved->GetObjPos( leadObjId  );
		double leadVehVel = m_pCved->GetObjVel( leadObjId ) * cMETER_TO_FEET; //3.2808;
		double leadObjLength = m_pCved->GetObjLength( leadObjId );
		double cabLength = m_cpCabSolObj->GetLength();
		double adjustDist = ( leadObjLength * 0.5f ) + ( cabLength * 0.5f );
		double bumperTobumperDist = distToLeadObj - adjustDist;
		double ownVehVel = 0.0;
		bool success = m_pCved->GetOwnVehicleVel( ownVehVel );
		ownVehVel = ownVehVel * cMETER_TO_FEET; //3.2808;  // convert to ft/s
		double diffVel = ownVehVel - leadVehVel;

		CHcsmCollection::m_sFollowInfo[0] = (float) leadObjId;
		CHcsmCollection::m_sFollowInfo[1] = (float) distToLeadObj;
		if( success )
		{
			CHcsmCollection::m_sFollowInfo[2] = (float) ( bumperTobumperDist / ownVehVel );
			bool calcTtc = leadVehVel < ownVehVel;
			if( calcTtc )
			{
				CHcsmCollection::m_sFollowInfo[4] = (float) ( bumperTobumperDist / diffVel );
			}
			else
			{
				CHcsmCollection::m_sFollowInfo[4] = 10000.0f;
			}
		}
		else
		{
			CHcsmCollection::m_sFollowInfo[2] = 0.0f;
			CHcsmCollection::m_sFollowInfo[4] = 0.0f;
		}
		CHcsmCollection::m_sFollowInfo[3] = (float) bumperTobumperDist;
		CHcsmCollection::m_sFollowInfo[5] = (float) leadVehVel;
		CHcsmCollection::m_sFollowInfo[6] = (float) leadVehPos.m_x;
		CHcsmCollection::m_sFollowInfo[7] = (float) leadVehPos.m_y;
		CHcsmCollection::m_sFollowInfo[8] = (float) leadVehPos.m_z;

		m_sOwnVehToLeadObjDist = (float) bumperTobumperDist;

	}
	else
	{
		m_sOwnVehToLeadObjDist = -1.0f;
		CHcsmCollection::m_sFollowInfo[0] = -1.0f;
		// these are being set since the titler is not able to look at
		// element 0 to see if the values in elements 1 and 4 are valid.
		CHcsmCollection::m_sFollowInfo[1] = 0.0f;
		CHcsmCollection::m_sFollowInfo[3] = 0.0f;
	}

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Implements a lane departure warning algorithm.
//
// Remarks: If all 4 corners of the ownvehicle are on the same lane/crdr,
//   check to see if any corner within 2 feet of lane edge.  If driver not
//   steering away from that edge, then activate warning.
//
//   NOTES: Sets the CHcsmCollection::m_sLdwStatus static variable.
//
// Arguments:
//  cpOwnVehCartPos - The ownvehicle's current position.
//
// Returns: Nothing
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeLaneDepartureWarning( const CPoint3D& cOwnVehCartPos )
{
	//gout << "LDW..." << endl;

	CHcsmCollection::m_sLdwStatus = eLDW_NONE;
	CHcsmCollection::m_sLaneDepartWarn[0] = (float)CHcsmCollection::m_sLdwStatus;
	CHcsmCollection::m_sLaneDepartWarn[1] = 0.0;
	CHcsmCollection::m_sLaneDepartWarn[2] = 0.0;
	CHcsmCollection::m_sLaneDepartWarn[3] = 0.0;

	bool runLdwAlgorithm = CHcsmCollection::m_sLCW_IsOn;
	//if( !runLdwAlgorithm )  return;

	// check to make sure that the own vehicle exists
	bool ownVehValid = m_pCved->IsObjValid( 0 );
	if( !ownVehValid )  return;

	//
	// Check to make sure velocity is at least 60 km/h
	//
	double ownVehVelocity;
	bool success = m_pCved->GetOwnVehicleVel( ownVehVelocity );
	const double cLDW_MIN_VELOCITY = 3.0; // 60.0; // km/h
	bool notFastEnough = ownVehVelocity * cMS_TO_KPH < cLDW_MIN_VELOCITY;
	if( notFastEnough )
	{
//		gout << "  not fast enough " << ownVehVelocity * cMS_TO_KPH;
//		gout << " kph     " << ownVehVelocity * cMS_TO_MPH << " mph" << endl;
		return;
	}

	//
	// Calculate the 4 corners of the own vehicle starting at left front
	// and going counter-clockwise.
	//
	CVector3D ovTangent;
	success = m_pCved->GetOwnVehicleTan( ovTangent );
	if( !success )
	{
//		gout << "  unable to get tangent for ov at " << cOwnVehCartPos << endl;
		return;
	}
	CVector3D ovLateral;
	success = m_pCved->GetOwnVehicleLat( ovLateral );
	if( !success )
	{
//		gout << "  unable to get lateral for ov at " << cOwnVehCartPos << endl;
		return;
	}

	double vehlength = m_pCved->GetObjLength( 0 );
	double vehwheelbase = vehlength*0.6; // estimate wheelbase from length
	double vehwidth = m_pCved->GetObjWidth( 0 );
	ovTangent.Scale( 0.5 * vehwheelbase );
	ovLateral.Scale( 0.5 * vehwidth );

	const int cLEFT_FRONT = 0;
	const int cLEFT_REAR = 1;
	const int cRIGHT_REAR = 2;
	const int cRIGHT_FRONT = 3;
	const int cMID_FRONT = 4;
	const int cMID_REAR = 5;
	const int cCG = 6;
	CPoint3D ownVehSortedCorners[7];
	ownVehSortedCorners[cLEFT_FRONT]  = cOwnVehCartPos + ovTangent - ovLateral;
	ownVehSortedCorners[cLEFT_REAR]   = cOwnVehCartPos - ovTangent - ovLateral;
	ownVehSortedCorners[cRIGHT_REAR]  = cOwnVehCartPos - ovTangent + ovLateral;
	ownVehSortedCorners[cRIGHT_FRONT] = cOwnVehCartPos + ovTangent + ovLateral;
	ownVehSortedCorners[cMID_FRONT]   = cOwnVehCartPos + ovTangent;
	ownVehSortedCorners[cMID_REAR]    = cOwnVehCartPos - ovTangent;
	ownVehSortedCorners[cCG]          = cOwnVehCartPos;

	int leftMarking=-1;
	int rightMarking=-1;
	int rightAtrib=-1;
	int leftAtrib=-1;
	CRoadPos ownVehRoadPosSorted[7];
	int i;
	for( i = 0; i < 7; i++ )
	{
		ownVehRoadPosSorted[i].SetCved( m_pCved );
		bool success = ownVehRoadPosSorted[i].SetXYZ( ownVehSortedCorners[i] );
		if (!success) return;
	}
	CCrdr crd_CG;
	int crdId_CG = -1;

	// Get lane markings
	//m_ownVehRoadPos.GetRoadMarking(leftMarking, rightMarking, rightAtrib, leftAtrib);
	//cout << "left=" << leftMarking << " right=" << rightMarking << endl;
	leftMarking = CHcsmCollection::m_sLaneMarkingInfo[0];
	rightMarking  = CHcsmCollection::m_sLaneMarkingInfo[1];
	////
	//// Check to see if all four corners are on the same crdr or lane.
	////
	//bool allCornersOnSameCrdrLane = true;
	//int crdrLaneId;
	cvELnDir direction = eNONE;
	float crdrLaneWidth;
	static float crdrLaneWidthPrev;
	if( m_ownVehRoadPos.IsRoad() )
	{
		CLane currentLane = m_ownVehRoadPos.GetLane();
		//crdrLaneId    = currentLane.GetId();
		crdrLaneWidth = (float)currentLane.GetWidth();
		//typedef enum { eNONE, ePOS, eNEG } cvELnDir;
		direction = m_ownVehRoadPos.GetLane().GetDirection();
		if( crdrLaneWidth<1.0f ) {
			crdrLaneWidth = crdrLaneWidthPrev;
			//cout << "width1=" << crdrLaneWidth << endl;
		}
	}
	else
	{
		crd_CG = m_ownVehRoadPos.GetCorridor();
		if (crd_CG.IsValid())
			crdId_CG = crd_CG.GetId();
		crdrLaneWidth = (float)crd_CG.GetWidth( m_ownVehRoadPos.GetDistance() );
		if( crdrLaneWidth<1.0f ) {
			crdrLaneWidth = crdrLaneWidthPrev;
			//cout << "width2=" << crdrLaneWidth << endl;
		}
	}
	crdrLaneWidthPrev = crdrLaneWidth;

	//
	// Figure out which way the vehicle is head with respect to the road.
	//
	bool driverSignalingLeft = CHcsmCollection::m_sCisTurnSignal == eTURN_SIGNAL_LEFT;
	bool driverSignalingRight = CHcsmCollection::m_sCisTurnSignal == eTURN_SIGNAL_RIGHT;
	int crdrUsed = -1;
	double offsetFront = ownVehRoadPosSorted[cMID_FRONT].GetOffset(-1,&CHcsmCollection::m_sOwnshipPath);
	double offsetRear = ownVehRoadPosSorted[cMID_REAR].GetOffset(-1,&CHcsmCollection::m_sOwnshipPath);
	CVector3D tanRoad = ownVehRoadPosSorted[cMID_FRONT].GetTangent();

	// only 'false' case can happen on a road in the wrong lane.
	int myDirection;
	bool goingWithTraffic = true;
	if ( ( ownVehRoadPosSorted[cMID_FRONT].IsRoad() == ownVehRoadPosSorted[cMID_FRONT].IsRoad() ) && m_ownVehRoadPos.IsValid()) {
		if (ownVehRoadPosSorted[cMID_FRONT].GetDistance() > ownVehRoadPosSorted[cMID_REAR].GetDistance() ) {
			myDirection = ePOS;
		}else{
			myDirection = eNEG;
		}
		if (m_ownVehRoadPos.IsRoad()) {
			if (myDirection != m_ownVehRoadPos.GetLane().GetDirection()) {
				goingWithTraffic = false;
				//cout << myDirection << " " << m_ownVehRoadPos.GetLane().GetDirection() << endl;
			}
		}
	}

	double angleInLane = atan2(offsetFront-offsetRear,vehlength);
	double offsetLeft = offsetFront - cos(angleInLane) * vehwidth/2.0;
	double offsetRight = offsetFront + cos(angleInLane) * vehwidth/2.0;
	angleInLane = angleInLane * 180.0/M_PI;
	double distFromLeftEdge = (crdrLaneWidth / 2.0f) + offsetLeft;
	double distFromRightEdge = (crdrLaneWidth / 2.0f) - offsetRight;
	static bool vehicleHeadedRight;
	if( angleInLane > 0.5 ) vehicleHeadedRight = true;
	if( angleInLane < -0.5 ) vehicleHeadedRight = false;

	float margin = 3.0f;
	if (CHcsmCollection::m_sLDW_Severity==1)
		margin = 12.0f; // 12" before crossing inner edge of the lane marking
	else if (CHcsmCollection::m_sLDW_Severity==2)
		margin = 3.0f; // 3" before crossing inner edge of the lane marking
	else
		margin = 6.0f; // 6" before crossing inner edge of the lane marking

	float minDistFromLeftEdge = (2.0f+margin)/12.0f;  // feet
	float maxDistFromLeftEdge = 22.0f/12.0f;  // feet
	float minDistFromRightEdge = margin/12.0f;  // feet
	float maxDistFromRightEdge = 24.0f/12.0f;  // feet
	switch (leftMarking) {
		case cCV_ROAD_LSTYLE_NO_MARKING:
		case cCV_ROAD_LSTYLE_SINGLE_SOLID_W:
		case cCV_ROAD_LSTYLE_SINGLE_DASHED_W:
		case cCV_ROAD_LSTYLE_SINGLE_SOLID_Y:
		case cCV_ROAD_LSTYLE_SINGLE_DASHED_Y:
			minDistFromLeftEdge = (2.0f+margin)/12.0f;  // feet
			maxDistFromLeftEdge = 22.0f/12.0f;  // feet
			break;
		case cCV_ROAD_LSTYLE_DOUBLE_SOLID_W:
		case cCV_ROAD_LSTYLE_DOUBLE_DASHED_W:
		case cCV_ROAD_LSTYLE_DOUBLE_SOLID_Y:
			minDistFromLeftEdge = (9.0f+margin)/12.0f;  // feet
			maxDistFromLeftEdge = 15.0f/12.0f;  // feet
		case cCV_ROAD_LSTYLE_DOUBLE_DASHED_Y:
			minDistFromLeftEdge = (6.0f+margin)/12.0f;  // feet
			maxDistFromLeftEdge = 18.0f/12.0f;  // feet
	};
	switch (rightMarking) {
		case cCV_ROAD_LSTYLE_NO_MARKING:
		case cCV_ROAD_LSTYLE_SINGLE_SOLID_W:
			minDistFromRightEdge = margin/12.0f;  // feet
			maxDistFromRightEdge = 24.0f/12.0f;  // feet
			break;
		case cCV_ROAD_LSTYLE_SINGLE_DASHED_W:
		case cCV_ROAD_LSTYLE_SINGLE_SOLID_Y:
		case cCV_ROAD_LSTYLE_SINGLE_DASHED_Y:
			minDistFromRightEdge = margin/12.0f;  // feet
			maxDistFromRightEdge = 24.0f/12.0f;  // feet
			break;
		case cCV_ROAD_LSTYLE_DOUBLE_SOLID_W:
		case cCV_ROAD_LSTYLE_DOUBLE_DASHED_W:
		case cCV_ROAD_LSTYLE_DOUBLE_SOLID_Y:
		case cCV_ROAD_LSTYLE_DOUBLE_DASHED_Y:
			minDistFromRightEdge = (6.0f+margin)/12.0f;  // feet
			maxDistFromRightEdge = 18.0f/12.0f;  // feet
	};

	CHcsmCollection::m_sLdwStatus = eLDW_MONITORING;

	if( distFromLeftEdge < -maxDistFromLeftEdge &&
		!vehicleHeadedRight && !driverSignalingLeft ) {
			CHcsmCollection::m_sLdwStatus = eLDW_MONITORING;
	}
	else if( distFromLeftEdge < minDistFromLeftEdge &&
		!vehicleHeadedRight && !driverSignalingLeft ) {
			CHcsmCollection::m_sLdwStatus = eLDW_MONITORING;
			//if( leftMarking>=0 ) {
			//	if( leftMarking>0 ) CHcsmCollection::m_sLdwStatus = eLDW_LEFT;
			//} else {
			//	CHcsmCollection::m_sLdwStatus = eLDW_LEFT;
			//}
			if( leftMarking > 0  && goingWithTraffic ) {
				CHcsmCollection::m_sLdwStatus = eLDW_LEFT;
			}
			if (leftMarking == -1 && goingWithTraffic && m_ownVehRoadPos.IsRoad() ){
				CHcsmCollection::m_sLdwStatus = eLDW_LEFT;
			}
	}
	if( distFromRightEdge < -maxDistFromRightEdge &&
		vehicleHeadedRight && !driverSignalingRight ) {
			CHcsmCollection::m_sLdwStatus = eLDW_MONITORING;
	}
	else if( distFromRightEdge < minDistFromRightEdge &&
		vehicleHeadedRight && !driverSignalingRight ) {
			CHcsmCollection::m_sLdwStatus = eLDW_MONITORING;
			//if( rightMarking>=0 ) {
			//	if( rightMarking>0 ) CHcsmCollection::m_sLdwStatus = eLDW_RIGHT;
			//} else {
			//	CHcsmCollection::m_sLdwStatus = eLDW_RIGHT;
			//}
			if( rightMarking > 0 && goingWithTraffic ) {
				CHcsmCollection::m_sLdwStatus = eLDW_RIGHT;
			}
			if (rightMarking == -1 && goingWithTraffic && m_ownVehRoadPos.IsRoad() ){
				CHcsmCollection::m_sLdwStatus = eLDW_RIGHT;
			}
	}

	//static long int counter = 0;
	//if( counter==100 ) {
	//	counter = 0;
	//	cout << "direction=" << direction << endl;
	//	cout << "left=" << leftMarking << " right=" << rightMarking << endl;
	//	//cout << "distleft=" << distFromLeftEdge << " distright=" << distFromRightEdge << endl;
	//	//cout << "minleft=" << minDistFromLeftEdge << " minright=" << minDistFromRightEdge << endl;
	//	//cout << " road name = " << m_ownVehRoadPos.GetName();
	//	//cout << " lane width = " << CHcsmCollection::m_sLaneDevInfo[2] << endl;
	//}
	//else {
	//	counter+=1;
	//}

	// check that the car is not in the oncoming lane
	//if( CHcsmCollection::m_sLaneDevInfo[3]==0 ) CHcsmCollection::m_sLdwStatus = eLDW_MONITORING;

	CHcsmCollection::m_sLaneDepartWarn[0] = (float)CHcsmCollection::m_sLdwStatus;
	CHcsmCollection::m_sLaneDepartWarn[1] = (float)distFromLeftEdge;
	CHcsmCollection::m_sLaneDepartWarn[2] = (float)distFromRightEdge;
	CHcsmCollection::m_sLaneDepartWarn[3] = (float)angleInLane;
	//printf("angleInLane = %f, left:%f, right:%f\n",angleInLane,(float)vehicleHeadedLeft,(float)vehicleHeadedRight);

//	gout << "  system status = " << CHcsmCollection::m_sLdwStatus << endl;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Implements a backup detection warning algorithm.
//
// Remarks:
//
// Arguments:
//  cpOwnVehCartPos - The ownvehicle's current position.
//
// Returns: Nothing
//
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeBackupDetectionWarning( const CPoint3D& cOwnVehCartPos )
{
//	gout << "BUD..." << endl;
	//
	// Check to see if backup detection has been engaged by scenario.
	//

	bool runBudAlgorithm = CHcsmCollection::m_sBUD_IsOn;
	if( !runBudAlgorithm )  return;

	const double cBUD_MAX_DIST = 100.0f;
	CHcsmCollection::m_sBackUpDistance = (float)cBUD_MAX_DIST;

	//
	// Get all objects within a certain distance of the driver.
	//
	const double cOBJ_DETECT_DISTANCE = 60.0f;
	vector<int> objsInRadius;
	m_pCved->GetObjsNear( cOwnVehCartPos, cOBJ_DETECT_DISTANCE, objsInRadius );
	bool noObjectsNearDriver = objsInRadius.size() <= 0;
	if( noObjectsNearDriver)  return;

	//
	// Calculate the 4 corners of the own vehicle starting at left front
	// and going counter-clockwise.
	//
	CVector3D ovTangent;
	bool success = m_pCved->GetOwnVehicleTan( ovTangent );
	if( !success )
	{
		return;
	}
	CVector3D ovLateral;
	success = m_pCved->GetOwnVehicleLat( ovLateral );
	if( !success )
	{
		return;
	}
	ovTangent.Scale( 0.5 * m_pCved->GetObjLength( 0 ) );
	ovLateral.Scale( 0.5 * m_pCved->GetObjWidth( 0 ) );

	const int cLEFT_FRONT = 0;
	const int cLEFT_REAR = 1;
	const int cRIGHT_REAR = 2;
	const int cRIGHT_FRONT = 3;
	CPoint3D ownVehSortedCorners[4];
	ownVehSortedCorners[cLEFT_FRONT]  = cOwnVehCartPos + ovTangent - ovLateral;
	ownVehSortedCorners[cLEFT_REAR]   = cOwnVehCartPos - ovTangent - ovLateral;
	ownVehSortedCorners[cRIGHT_REAR]  = cOwnVehCartPos - ovTangent + ovLateral;
	ownVehSortedCorners[cRIGHT_FRONT] = cOwnVehCartPos + ovTangent + ovLateral;

//	gout << "  ov[lf] = " << ownVehSortedCorners[cLEFT_FRONT] << endl;
//	gout << "  ov[lr] = " << ownVehSortedCorners[cLEFT_REAR] << endl;
//	gout << "  ov[rr] = " << ownVehSortedCorners[cRIGHT_REAR] << endl;
//	gout << "  ov[rf] = " << ownVehSortedCorners[cRIGHT_FRONT] << endl;

	const double cBUD_LENGTH = 35.0;
	ovTangent.Normalize();
	ovTangent.Scale( cBUD_LENGTH );

	const double cBUD_WIDTH_EXTENDER = 3.0;
	ovLateral.Normalize();
	ovLateral.Scale( cBUD_WIDTH_EXTENDER );

//	gout << "  lat = " << ovLateral << endl;
//	gout << "  tan = " << ovTangent << endl;

	CPoint3D backupSortedCorners[4];
	backupSortedCorners[cLEFT_FRONT]  = ownVehSortedCorners[cLEFT_REAR] - ovLateral;
	backupSortedCorners[cLEFT_REAR]   = backupSortedCorners[cLEFT_FRONT] - ovTangent;
	backupSortedCorners[cRIGHT_FRONT] = ownVehSortedCorners[cRIGHT_REAR] + ovLateral;
	backupSortedCorners[cRIGHT_REAR]  = backupSortedCorners[cRIGHT_FRONT] - ovTangent;

//	gout << "  bud[lf] = " << backupSortedCorners[cLEFT_FRONT] << endl;
//	gout << "  bud[lr] = " << backupSortedCorners[cLEFT_REAR] << endl;
//	gout << "  bud[rr] = " << backupSortedCorners[cRIGHT_REAR] << endl;
//	gout << "  bud[rf] = " << backupSortedCorners[cRIGHT_FRONT] << endl;

	CBoundingBox ovBoundingBox(
		ownVehSortedCorners[cLEFT_FRONT].m_x,
		ownVehSortedCorners[cLEFT_FRONT].m_y,
		ownVehSortedCorners[cRIGHT_REAR].m_x,
		ownVehSortedCorners[cRIGHT_REAR].m_y
		);
	CBoundingBox ovBackBoundingBox(
		backupSortedCorners[cLEFT_FRONT].m_x,
		backupSortedCorners[cLEFT_FRONT].m_y,
		backupSortedCorners[cRIGHT_REAR].m_x,
		backupSortedCorners[cRIGHT_REAR].m_y
		);

	//
	// Figure out if the object intersects with the area behind the driver.
	//
	vector<int>::iterator itr;
	CPoint3D objCorners[4];
	double minDistSquared = cBUD_MAX_DIST * cBUD_MAX_DIST;
	bool foundDist = false;
	for( itr = objsInRadius.begin(); itr != objsInRadius.end(); itr++ )
	{
//		gout << "  looking at obj " << *itr << ": " << m_pCved->GetObjName( *itr ) << endl;
		if( *itr == 0 )  continue;  // skip external driver

		CBoundingBox objBoundingBox = m_pCved->GetObjBoundBox( *itr );
//		gout << "    ov  bb = ( " << ovBoundingBox.GetMinX() << ", " << ovBoundingBox.GetMinY() << " )  ( ";
//		gout << ovBoundingBox.GetMaxX() << ", " << ovBoundingBox.GetMaxY() << " )" << endl;
//		gout << "    obj bb = ( " << objBoundingBox.GetMinX() << ", " << objBoundingBox.GetMinY() << " )  ( ";
//		gout << objBoundingBox.GetMaxX() << ", " << objBoundingBox.GetMaxY() << " )" << endl;

		if( ovBoundingBox.Intersects( objBoundingBox ) )
		{
//			gout << "    ov intersects this obj" << endl;
			CHcsmCollection::m_sBackUpDistance = 0.0f;
			break;
		}
		else if( ovBackBoundingBox.Intersects( objBoundingBox ) )
		{
//			gout << "    intersects with ov" << endl;
			success = objBoundingBox.GetFourCorners( objCorners );
			if( !success )
			{
//				gout << "    unable to get corners" << endl;
				continue;
			}

			int i;
			for( i = 0; i < 4; i++ )
			{
				double squaredDistToObjCorner = objCorners[i].DistSq( ownVehSortedCorners[cLEFT_REAR] );
				if( squaredDistToObjCorner < minDistSquared )
				{
					minDistSquared = squaredDistToObjCorner;
					foundDist = true;
				}

				squaredDistToObjCorner = objCorners[i].DistSq( ownVehSortedCorners[cRIGHT_REAR] );
				if( squaredDistToObjCorner < minDistSquared )
				{
					minDistSquared = squaredDistToObjCorner;
					foundDist = true;
				}
			}

		}
	}

	if( foundDist )
	{
		CHcsmCollection::m_sBackUpDistance =(float) sqrt( minDistSquared );
	}

//	gout << "  result = " << CHcsmCollection::m_sBackUpDistance << " feet" << endl;
}


//////////////////////////////////////////////////////////////////////////////
///\brief
/// Description:  Computes the ownvehicle's lane deviation and curavature at
///  current location on the CVED road netowrk.
///
///\remark
///  Writes data to the static member variable called
///  m_sLaneDevInfo.  This is array of 4 floats where each element
///  has the following info:
///   [0] -> -1 (on a crdr), 1 (on a lane), 0 (error)
///   [1] -> offset from the center of lane/crdr
///   [2] -> width of lane  (crdr's width is not reported, too expensive)
///   [3] -> lane/crdr CVED id
///  Also write info to m_sOwnVehCurvature.
///
///  NOTE: Sets the m_ownVehRoadPos so make sure this function is called before
///   any of the other functions that use this local member.
///
///\param  cpOwnVehCartPos - The ownvehicle's current position.
///
/// Returns:
///
//////////////////////////////////////////////////////////////////////////////
void
CScenarioControl::ComputeOwnVehInfo( const CPoint3D& cOwnVehCartPos )
{
	bool ownVehValid = m_pCved->IsObjValid( 0 );
	if( ownVehValid )
	{
		static CCrdr    sPrevCrdr;
		static bool     sPrevFrameFail = true;

		//
		// Given the ownvehicle's cartesian position, try to compute the
		// ownvehicle's road position in CVED.  Use hint from previous
		// execution to speed up query.
		//
		bool success;
		bool tryCrdrHint = (
					m_ownVehRoadPos.IsValid() &&
					!m_ownVehRoadPos.IsRoad() &&
					sPrevCrdr.IsValid()
					) /*&& 0*/;
		if( tryCrdrHint )
		{
			//
			// Ownvehicle was on a corridor on the previous execution of
			// this function.
			//
			success = m_ownVehRoadPos.SetXYZ( cOwnVehCartPos, &sPrevCrdr );

			if( !success )
			{
				// try without the hint if not successful with hint
				success = m_ownVehRoadPos.SetXYZ( cOwnVehCartPos );

				// invalidate sPrevCrdr
				CCrdr crdr;
				sPrevCrdr = crdr;
			}
		}
		else
		{
			//
			// Ownvehicle was on a road on the previous execution of this
			// function.
			//
			success = m_ownVehRoadPos.SetXYZ( cOwnVehCartPos);

			// invalidate sPrevCrdr
			CCrdr crdr;
			sPrevCrdr = crdr;
		}

		if( success )
		{
			//
			// Successfully compute ownvehicle's road position.  Write to
			// appropriate variables.
			//
			if( m_ownVehRoadPos.IsRoad() )
			{
				double distance = m_ownVehRoadPos.GetDistanceOnLane();
				//printf("distance on lane %f\n", distance);
				CLane lane = m_ownVehRoadPos.GetLane();
				CLane splineLane = m_ownVehRoadPos.GetSplineLane();
//				if (CHcsmCollection::m_sOwnshipPath.IsValid() && CHcsmCollection::m_sOwnshipPath.Contains(m_ownVehRoadPos))
//				{
//
//				}

//				tempSpline.addPoint();
				double ofs  = m_ownVehRoadPos.GetOffset();

				CHcsmCollection::m_sLaneDevInfo[0] = 1.0;
				CHcsmCollection::m_sLaneDevInfo[1] = (float) ofs;
				CHcsmCollection::m_sLaneDevInfo[2] = (float) lane.GetWidth(distance);
				CHcsmCollection::m_sLaneDevInfo[3] = lane.GetRelativeId();

				CHcsmCollection::m_sSplineDevInfo[0] = 1.0;
				CHcsmCollection::m_sSplineDevInfo[1] = (float) m_ownVehRoadPos.GetSplineOffset();;
				CHcsmCollection::m_sSplineDevInfo[2] = (float) splineLane.GetWidth(distance);
				CHcsmCollection::m_sSplineDevInfo[3] = splineLane.GetRelativeId();

			}
			else
			{
				sPrevCrdr = m_ownVehRoadPos.GetCorridor();

				int crdId = -1;
				double ofs  = 0;
				int usedPathMode = 1;
				if( CHcsmCollection::m_sOwnshipPath.IsValid() && CHcsmCollection::m_sOwnshipPath.Contains(m_ownVehRoadPos))
				{
					ofs  = m_ownVehRoadPos.GetOffset(-1, &CHcsmCollection::m_sOwnshipPath, &crdId );
				}
				else
				{
					ofs  = m_ownVehRoadPos.GetOffset();
					crdId = sPrevCrdr.GetRelativeId();
					usedPathMode = 2;
				}
				//
				if( crdId > 0 && sPrevCrdr.IsValid() && crdId != sPrevCrdr.GetRelativeId())
				{
					//if we have a relative id of the crdr used, prioritize our
					//crdrs by that id
					//sPrevCrdr = m_ownVehRoadPos.GetCorridor(crdId);
					//m_ownVehRoadPos.SetCrdPriority(&sPrevCrdr);
				}



				CHcsmCollection::m_sLaneDevInfo[0] = -1.0f * usedPathMode;
				CHcsmCollection::m_sLaneDevInfo[1] = (float) ofs;
				CHcsmCollection::m_sLaneDevInfo[2] = 0.0;  // not efficient
				CHcsmCollection::m_sLaneDevInfo[3] = (float) crdId;//sPrevCrdr.GetRelativeId();

				CHcsmCollection::m_sSplineDevInfo[0] = -1.0f * usedPathMode;
				CHcsmCollection::m_sSplineDevInfo[1] = (float) ofs;
				CHcsmCollection::m_sSplineDevInfo[2] = 0.0;  // not efficient
				CHcsmCollection::m_sSplineDevInfo[3] = (float) crdId;//sPrevCrdr.GetRelativeId();


			}

			CHcsmCollection::m_sOwnVehCurvature = (float) m_ownVehRoadPos.GetExpandedCurvature();
			const float cMAX_CUVATURE = 10000.0;
			bool curvatureTooHighToMatter = CHcsmCollection::m_sOwnVehCurvature > cMAX_CUVATURE;
			if( curvatureTooHighToMatter )
				CHcsmCollection::m_sOwnVehCurvature = cMAX_CUVATURE;
			else
			{
				curvatureTooHighToMatter = CHcsmCollection::m_sOwnVehCurvature < -cMAX_CUVATURE;
				if( curvatureTooHighToMatter )
					CHcsmCollection::m_sOwnVehCurvature = -cMAX_CUVATURE;
			}
		}
		else
		{
			//
			// Unable to compute ov lane dev.  Print this message only once.
			//
			if( !sPrevFrameFail )
			{
#if 1 // do not disable this printf since if this statment is being printed then something needs to be fixed
				if (CHcsmCollection::m_verbose){
					fprintf(
						stderr,
						"**Nix: unable to compute ov lane dev at %.1f, %.1f, %.1f\n",
						cOwnVehCartPos.m_x, cOwnVehCartPos.m_y, cOwnVehCartPos.m_z
						);
					fflush( stderr );
				}
#endif
				CHcsmCollection::m_sLaneDevInfo[0] = 0.0;
				CHcsmCollection::m_sLaneDevInfo[1] = 0.0;
				CHcsmCollection::m_sLaneDevInfo[2] = 0.0;
				CHcsmCollection::m_sLaneDevInfo[3] = 0.0;
				CHcsmCollection::m_sOwnVehCurvature = 0.0f;
			}
		}

		sPrevFrameFail = !success;
	}
	else
	{
		CHcsmCollection::m_sLaneDevInfo[0] = 0.0;
		CHcsmCollection::m_sLaneDevInfo[1] = 0.0;
		CHcsmCollection::m_sLaneDevInfo[2] = 0.0;
		CHcsmCollection::m_sLaneDevInfo[3] = 0.0;
		CHcsmCollection::m_sOwnVehCurvature = 0.0f;
	}

	if( CHcsmCollection::m_sSirenSpeed > 0 )
	{
		//if they are over sirenSpeed, tell the audio to play a siren
		double myvel;
		if( m_pCved->GetOwnVehicleVel( myvel ) )
		{
			if( myvel > CHcsmCollection::m_sSirenSpeed )
			{
				//float overSpeed = myvel - m_sirenSpeed;
				float overSpeed = (float)myvel - CHcsmCollection::m_sSirenSpeed;
				CHcsmCollection::m_sSirenEffect = 0.1f + (overSpeed/(15 * (float)cMS_TO_MPH)) * 0.9f;
			}
			else
			{
				CHcsmCollection::m_sSirenEffect  = 0;
			}
		}
	}
	else
	{
		CHcsmCollection::m_sSirenEffect  = 0;
	}

	//now see if the ownship is on the ownship path
	if( m_ownVehRoadPos.IsValid() && CHcsmCollection::m_sOwnshipPath.IsValid() && CHcsmCollection::m_sOwnshipPath.Contains( m_ownVehRoadPos ) )
	{
		CPoint3D pos = m_ownVehRoadPos.GetXYZ();
		CHcsmCollection::m_sIsOnPath = true;
		CHcsmCollection::SetLastGoodPosition( (float)pos.m_x, (float)pos.m_y, (float)pos.m_z );
	}
	else
	{
		CHcsmCollection::m_sIsOnPath = false;
	}

	if( m_ownVehRoadPos.IsValid() &&
		m_intVehRoadPos.IsValid() &&
	    CHcsmCollection::m_sOwnshipPath.Contains( m_ownVehRoadPos ) &&
		CHcsmCollection::m_sOwnshipPath.Contains( m_intVehRoadPos )
		)
	{
			CHcsmCollection::m_sOwnVehDistOnPath = (float)CHcsmCollection::m_sOwnshipPath.GetLength( &m_intVehRoadPos, &m_ownVehRoadPos );
	}
	else
	{
		CHcsmCollection::m_sOwnVehDistOnPath = -1;
	}
	int leftmarking, rightmarking, rightAtrib, leftAtrib;
    //calcuate a forward vector for an overhead view
    if (CHcsmCollection::m_sIsOnPath){
        float collectiveWeights;
        //we are going to do a 7 point sample of the tangents
        // 2 4 8 4 2
        CVector3D pos = m_ownVehRoadPos.GetTangentInterpolated(true)*6.0;
        CPoint3D ownpos = m_ownVehRoadPos.GetVeryBestXYZ();
        CRoadPos tempRP;
        CLane tempLane;
        auto res = CHcsmCollection::m_sOwnshipPath.Travel(300,m_ownVehRoadPos,tempRP,tempLane);
        if (res > -1){
            CVector3D tpos(tempRP.GetBestXYZ()-ownpos);
            tpos.Normalize();
            pos=tpos;
        }
        pos.Normalize();
        CVector3D oldPos(
            CHcsmCollection::m_sSmoothForwardRoadVector[0],
            CHcsmCollection::m_sSmoothForwardRoadVector[1],
            0);
        oldPos = oldPos *0.99 + pos*0.01;
        oldPos.Normalize();
        CHcsmCollection::m_sSmoothForwardRoadVector[0] = oldPos.m_i;
        CHcsmCollection::m_sSmoothForwardRoadVector[1] = oldPos.m_j;
    }else{
        if (m_ownVehRoadPos.IsValid()){
            CVector3D pos = m_ownVehRoadPos.GetTangentInterpolated(true)*6.0;
            CRoadPos tempRP = m_ownVehRoadPos;
            CPoint3D ownpos = m_ownVehRoadPos.GetVeryBestXYZ();
            CLane tempLane;
            auto res = tempRP.Travel(300,nullptr,CRoadPos::eSTRAIGHT);
            if (res > -1){
                CVector3D tpos(tempRP.GetBestXYZ()-ownpos);
                tpos.Normalize();
                pos=tpos;
            }
            pos.Normalize();
            CVector3D oldPos(
                CHcsmCollection::m_sSmoothForwardRoadVector[0],
                CHcsmCollection::m_sSmoothForwardRoadVector[1],
                0);
            oldPos = oldPos * 0.99 + pos*0.01;
            oldPos.Normalize();
            CHcsmCollection::m_sSmoothForwardRoadVector[0] = oldPos.m_i;
            CHcsmCollection::m_sSmoothForwardRoadVector[1] = oldPos.m_j;
        }else{
            //CHcsmCollection::m_sownvhe
        }
    }

	m_ownVehRoadPos.GetRoadMarking(leftmarking,rightmarking, rightAtrib, leftAtrib);

	CHcsmCollection::m_sLaneMarkingInfo[0] = leftmarking;
	CHcsmCollection::m_sLaneMarkingInfo[1] = rightmarking;
}
/////////////////////////////////////////////////////////////////////////////////////////
///\brief
///   This return the name of the lri
///
////////////////////////////////////////////////////////////////////////////////////////
const string&
CScenarioControl::GetLriName(){
   return m_lriName;
}