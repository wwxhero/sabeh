/*****************************************************************************
 *
 * (C) Copyright 2015 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: GraphicsStateAdapter.cpp,v 1.21 2018/09/12 20:41:01 IOWA\dheitbri Exp $
 *
 * Author(s):    D.A. Heitbrink
 *
 * Date:		 March, 2015
 *
 * Description:  Addapter to convert from HCSM state to Graphics State.
 *
 ****************************************************************************/
#include <cvedpub.h>
#include "hcsmcollection.h"

#include "GraphicsStateAdapter.h"

#include "ScenarioControl.h"


#include "SplineHermite.h"
#include <sstream>

#include "util.h"
const double M_PI = 3.14159265;
const float M_PIf = 3.14159265f;
const int MAX_OBJ = 300;

CGraphicsStateAdapter::CGraphicsStateAdapter(CRendererIface *render,CScenarioControl* scenario ):
m_pRender(render),m_pScen(scenario),m_headLightsOn(false){
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///     This functions adds uniform + set switch commands to m_pRender
///\remark
///     This function adds visual commands that are outside objects states.
///     The scenario control does not really know or track the state of 
///     what we are adding here, and only tracks commands
////////////////////////////////////////////////////////////////////////////
bool
CGraphicsStateAdapter::AddVisualCommands(){
	//eFLOAT = 0,
	//eSHORT = 1,
	//eINT   = 2,
	//eSTRING= 3,
    //eNOTSET =4
	CCved& cved = m_pScen->GetCved();
	CAttachShader shaderCmd;
	CSetUniform uniformCmd;
    bool haveLock = CHcsmCollection::m_sLockVisualOptions.TryLock();
    if (!haveLock){
        DWORD ticks = ::GetTickCount();
        haveLock = CHcsmCollection::m_sLockVisualOptions.TryLock();
        while (!haveLock && ticks != ::GetTickCount())
            haveLock = CHcsmCollection::m_sLockVisualOptions.TryLock();
    }
    if (haveLock){
        try{
        
	        for (int i =0; i <CHcsmCollection::m_sScenarioWriteUniformDataSize; i++){
		        memset(&uniformCmd,0,sizeof(uniformCmd));
		        switch(CHcsmCollection::m_sScenarioWriteUniformData[i].cellType){		
			        case CActionParseBlock::eFLOAT:{
				        uniformCmd.m_type = 'f';
				        float* ptr  = (float*)&uniformCmd.m_data[0];
				
				        auto &data = CHcsmCollection::m_sScenarioWriteUniformData[i].floatData;
				        size_t elemCnt = data.size();
				        if (elemCnt > 16) elemCnt = 16;
				        uniformCmd.m_size = (unsigned int)elemCnt;
				        for (size_t i =0; i < data.size(); i++){
					        ptr[i]=data[i];
				        }
			        }
			        break;
			        case CActionParseBlock::eINT:{
				        uniformCmd.m_type = 'i';
				        int* ptr  = (int*)&uniformCmd.m_data[0];
			
				        auto &data = CHcsmCollection::m_sScenarioWriteUniformData[i].intData;
				        size_t elemCnt = data.size();
				        if (elemCnt > 16) elemCnt = 16;
				        uniformCmd.m_size = (unsigned int)elemCnt;
				        for (size_t i =0; i < data.size(); i++){
					        ptr[i]=data[i];
				        }
			        }break;
			        case CActionParseBlock::eSHORT:{
				        uniformCmd.m_type = 'i'; //OSG does not support short as a uniform type
				        int* ptr  = (int*)&uniformCmd.m_data[0];
			
				        auto &data = CHcsmCollection::m_sScenarioWriteUniformData[i].shortData;
				        size_t elemCnt = data.size();
				        if (elemCnt > 16) elemCnt = 16;
				        uniformCmd.m_size = (unsigned int)elemCnt;
				        for (size_t i =0; i < data.size(); i++){
					        ptr[i]=data[i];
				        }
			        }break;
			        default:
				        continue;
		        }
		        const string& name = CHcsmCollection::m_sScenarioWriteUniformData[i].cellName;
		        memset(uniformCmd.m_Name,0,sizeof(uniformCmd.m_Name));
		        memcpy(uniformCmd.m_Name,&name[0],min(sizeof(uniformCmd.m_Name),name.size()));
		        auto& ids =  CHcsmCollection::m_sScenarioWriteUniformData[i].cvedIDs;
		        for (unsigned j =0; j < ids.size(); j++){
			        uniformCmd.m_id = ids[j];
			        uniformCmd.m_switchId = cved.GetObjCigiId(uniformCmd.m_id);
			        m_pRender->SetUniform(uniformCmd);
		        }
				ids.clear();
	        }
	        CHcsmCollection::m_sScenarioWriteUniformDataSize = 0;
	        for ( int i =0; i <CHcsmCollection::m_sAttachShaderCmdsDataSize; i++){
		        memset(&shaderCmd,0,sizeof(shaderCmd));
		        size_t size;
		        size = CHcsmCollection::m_sAttachShaderCmds[i].cellName.size();
		        if (size == 0 || size > sizeof(shaderCmd.m_Name))
			        continue;
		        memcpy(shaderCmd.m_Name,&CHcsmCollection::m_sAttachShaderCmds[i].cellName[0],size);
		        size = CHcsmCollection::m_sAttachShaderCmds[i].pragmas.size();
		        if (size > sizeof(shaderCmd.m_pragmaSet))
			        continue;
		        if (size > 0 ){
			        memcpy(shaderCmd.m_pragmaSet,&CHcsmCollection::m_sAttachShaderCmds[i].pragmas[0],size);
		        }else{
			        CHcsmCollection::m_sAttachShaderCmds[i].pragmas = "\0";
		        }
		        auto& ids = CHcsmCollection::m_sAttachShaderCmds[i].cvedIDs;
		        for (unsigned int i = 0; i < ids.size(); i++){
			        shaderCmd.m_id = ids[i];
			        shaderCmd.m_switchId = cved.GetObjCigiId(shaderCmd.m_id);
			        m_pRender->AttachShader(shaderCmd);	
		        }
                ids.clear();
		
	        }
	        CHcsmCollection::m_sAttachShaderCmdsDataSize = 0;
	        CSetSwitch switchCmd;
            for ( int i =0; i <CHcsmCollection::m_sSetSwitchCmdsDataSize; i++){
		        memset(&switchCmd,0,sizeof(switchCmd));
                //m_Name
				auto & currCommand = CHcsmCollection::m_sSetSwitchCmds[i];
                switchCmd.m_switchId = CHcsmCollection::m_sSetSwitchCmds[i].id;
                //memcpy(switchCmd.m_Name
                size_t size = CHcsmCollection::m_sSetSwitchCmds[i].switchName.size();
		        if (size > sizeof(switchCmd.m_Name))
			        continue;
		        if (size > 0 ){
                    memcpy(switchCmd.m_Name,&CHcsmCollection::m_sSetSwitchCmds[i].switchName[0],size);
		        }else{
			        CHcsmCollection::m_sSetSwitchCmds[i].switchName[0] = '\0';
		        }

		        auto& ids = CHcsmCollection::m_sSetSwitchCmds[i].cvedIDs;
		        for (unsigned int i = 0; i < ids.size(); i++){
			        switchCmd.m_id = ids[i];
				    if (!cved.IsExplicitObj( switchCmd.m_id ) &&(
				    	currCommand.switchName == "Self" ||currCommand.switchName == ""))
				    {
				        SetLRIObjectStates( switchCmd.m_id, switchCmd.m_switchId);
				        continue;
				    }
                    m_pRender->SetSwitch(switchCmd);	
		        }
                ids.clear();
		
	        }
            vector<CSpotlightCommand> commands;
	        CHcsmCollection::m_sSetSwitchCmdsDataSize = 0;
            CHcsmCollection::GetSpotlightQueue(commands);
            for (auto itr = commands.begin(); itr != commands.end(); itr++){
                m_pRender->AddSpotlightCommand(*itr);
            }
            CHcsmCollection::ClearSpotlightQueue();
        }catch(...){
        }
        CHcsmCollection::m_sLockVisualOptions.UnLock();
    }
	return true;
}
void 
CGraphicsStateAdapter::CreateObject(int cvedId)
{
	int option,uniqueId, modelType ;
	CCved& cved = m_pScen->GetCved();
	auto objType = cved.GetObjType(cvedId);
	if (objType == eCV_TRAJ_FOLLOWER){
		auto id = cved.GetObjSolId(cvedId);
		if (id > 0){
			auto solObj = cved.GetSol().GetObj(id);
			if (solObj && solObj->GetCategoryName() == "DiGuy"){
				CreateDiGuy(cvedId, id);
				return;
			}
		}
	}
	if (objType == eCV_VIRTUAL_OBJECT){
        cvTObjState objState;
        cved.GetObjStateInstant( cvedId, objState );
        vector<float> params;
        params.push_back(objState.virtualObjectState.ParseBlockID);
        m_pRender->CreateVirtualOject(cvedId,objState.virtualObjectState.ParseBlockID);
        return;
	}
	cved.GetObjOption( cvedId, option );
	bool gotInfo = GetCvedObjCigiInfo( cved, cvedId, uniqueId, modelType );

	if( gotInfo )
	{
		int dynamicObjId = cvedId;
		m_tireYaw[cvedId] = 0;
		m_bodyYaw[cvedId] = 0;

		// temporary precreation code
		const char* objName = cved.GetObjName(cvedId);
		cvTObjState objState;
		double objPos[3], objRot[3];

		cved.GetObjStateInstant( cvedId, objState );
		objPos[0] = objState.anyState.position.x;
		objPos[1] = objState.anyState.position.y;
		objPos[2] = objState.anyState.position.z;

		if ( cved.GetObjType(cvedId) == eCV_TRAJ_FOLLOWER &&
			!strcmp(cved.GetSol().GetObj(cved.GetObjSolId(cvedId))->GetCategoryName().c_str(), "Walker") )
		{
			// walker type, set roll and pitch to 0 for now
			objRot[0] = 0.0;
			objRot[1] = 0.0;
		}
		else
		{
			if ( true/*g_app.m_useRoll*/ )
				objRot[0] = atan2( -objState.anyState.lateral.k,
					objState.anyState.lateral.i * objState.anyState.tangent.j
					- objState.anyState.lateral.j * objState.anyState.tangent.i );
			else
				objRot[0] = 0.0;
			if (true /*g_app.m_usePitch*/ )
				objRot[1] = asin( objState.anyState.tangent.k );
			else
				objRot[1] = 0.0;
		}
		objRot[2] = atan2(
			objState.anyState.tangent.j, 
			objState.anyState.tangent.i
			);

		m_pRender->CreateObject( modelType, uniqueId, objPos[0], objPos[1], objPos[2], objRot[0], objRot[1], objRot[2] );
		SetMovingObjectStates( cvedId, option, uniqueId );

	}
}
void 
CGraphicsStateAdapter::CreateDiGuy(int cvedId, int solId){
	CCved& cved = m_pScen->GetCved();
	auto solObj = cved.GetSol().GetObj(solId);
	auto options = solObj->GetOptions();
	string modelName = solObj->GetVisModelName();
	int option;
	int size;
	cvTObjState objState;
	float objPos[3], objRot[3];
	char		Model[64];			 
	char		Apperence[64];		 			 		 
	int			Flag[2] = {0,0};				 
	//int         size;                  
	float       path[cMAX_PATH_PNTS*4]= {0};
	char        Actions[cMAX_PATH_PNTS]={0};
	float       Durration[cMAX_PATH_PNTS]={0};

	cved.GetObjStateInstant( cvedId, objState );
	const CDynObj* cobj = cved.BindObjIdToClass(cvedId);

	const CTrajFollowerObj* pDDO = dynamic_cast<const CTrajFollowerObj*> ( cobj);
	bool startAtCreation = false;
	if (pDDO){
		startAtCreation  = pDDO->GetAnimationState();
	}
	Flag[0] = (int)startAtCreation;
	Flag[1] = (int)objState.trajFollowerState.classType - eCV_DIGUY;
	objPos[0] = (float)objState.anyState.position.x;
	objPos[1] = (float)objState.anyState.position.y;
	objPos[2] = (float)objState.anyState.position.z;
	objRot[0] = 0.0f;
	objRot[1] = 0.0f;
	objRot[2] = (float)atan2(
		objState.anyState.tangent.j, 
		objState.anyState.tangent.i
		);
	if (cved.GetObjOption(cvedId,option)){
		if (option < (int)options.size()){
			memset(Model,0,sizeof(Model));
			memset(Apperence,0,sizeof(Apperence));
			memcpy_s(Model,sizeof(Model),modelName.c_str(),modelName.size());
			memcpy_s(Apperence,sizeof(Apperence),options[option].name.c_str(),options[option].name.size());
			size = (int)modelName.size();
			vector<float> tpath;
			vector<char> tActions;
			vector<float> tTimes;
			CHcsmCollection::GetDiGuyPathInfo(cvedId,tpath,tActions,tTimes);
			int cnt = 0;
			for (auto itr = tpath.begin(); itr != tpath.end() && cnt < cMAX_PATH_PNTS*4; itr++,cnt++){
				path[cnt] = *itr; 
			}
			cnt = 0;
			for (auto itr = tActions.begin(); itr != tActions.end() && cnt < cMAX_PATH_PNTS; itr++,cnt++){
				Actions[cnt] = *itr; 
			}
			cnt = 0;
			for (auto itr = tTimes.begin(); itr != tTimes.end() && cnt < cMAX_PATH_PNTS; itr++,cnt++){
				Durration[cnt] = *itr; 
			}
			int numObject = (int)std::min((size_t)cMAX_PATH_PNTS*4,tpath.size());
			m_pRender->CreateDiGuyObject(
				Model,
				Apperence,
				cvedId,
				objPos,
				objRot,
				Flag,
				numObject,
				Actions,
				Durration,
				path);
		}
	}
}
void SetVirtualObject(
    CRendererIface *pRender,
    const cvTObjState::VirtualObjectState &objState,
    int id)
{
    CVirtualObjectCommand cmd;
    cmd.m_data.overlayPosition[0]=(float)objState.overlayPosition.x;
	cmd.m_data.overlayPosition[1]=(float)objState.overlayPosition.y;
	cmd.m_data.overlayPosition[2]=(float)objState.overlayPosition.z;
	cmd.m_data.rotation=          objState.rotation;         
	cmd.m_data.objType=           objState.objType;          
	cmd.m_data.color[0]=		  objState.color[0];
	cmd.m_data.color[1]=		  objState.color[1];
	cmd.m_data.color[2]=		  objState.color[2];
	cmd.m_data.color[3]=		  objState.color[3];
	cmd.m_data.colorBorder[0]=    objState.colorBorder[0];
	cmd.m_data.colorBorder[1]=    objState.colorBorder[1];  
	cmd.m_data.colorBorder[2]=    objState.colorBorder[2];  
	cmd.m_data.colorBorder[3]=    objState.colorBorder[3];  
	cmd.m_data.scale[0]=          objState.scale[0];
	cmd.m_data.scale[1]=          objState.scale[1];   
	cmd.m_data.parentId=          objState.parentId;         
	cmd.m_data.StateIndex=        objState.StateIndex;       
    cmd.m_data.DrawScreen=        objState.DrawScreen;       
	cmd.m_data.ParseBlockID=      objState.ParseBlockID;     
    cmd.m_data.lightID=           objState.lightID;     
    cmd.m_id= id;
    pRender->VirtualObjectCommand(cmd);
}

bool 
CGraphicsStateAdapter::UpdateDynamicObjects(){
	int				currList[MAX_OBJ], currCount;
	int				newList[MAX_OBJ], newCount;
	int				delList[MAX_OBJ], delCount;

	int				i, cvedId, uniqueId, option;
	CCved&          cved = m_pScen->GetCved();

	CPoint3D  pos;
	CVector3D tang;
	CVector3D lat;

	CPoint3D ownVehCartPos;
	CVector3D dir;


	m_pScen->GetCved().GetOwnVehiclePos(pos);

	m_pScen->GetCved().GetOwnVehicleTan(tang);
	m_pScen->GetCved().GetOwnVehicleLat(lat);

	CObjTypeMask mask;
	mask.Clear();
	mask.Set( eCV_TRAJ_FOLLOWER );
	mask.Set( eCV_VEHICLE );
    mask.Set( eCV_VIRTUAL_OBJECT);

	int rcode = m_pScen->GetNewAndDeletedDynamicObjects(
				mask,
				currCount, currList,
				newCount, newList,
				delCount, delList,
				MAX_OBJ
				);

	vector<int> changedObjs;
	cved.GetChangedStaticObjsNear( pos, 500000, 1, changedObjs );

	int explCount = 0;
	int lriCount = 0;
	for( i = 0; i < (int)changedObjs.size(); ++i )
	{
		cvedId = changedObjs[i];
		cved.GetObjOption( cvedId, option );
		auto objType = cved.GetObjType(cvedId);
	    if (objType == eCV_VIRTUAL_OBJECT){
            cvTObjState objState;
            cved.GetObjStateInstant( cvedId, objState );
            SetVirtualObject(m_pRender,objState.virtualObjectState,cvedId);
            continue;
	    }

		if( cved.IsExplicitObj( cvedId ) )
		{
			// Explicit static object
			int uniqueId = cvedId;
			double posori[6]; 
			CPoint3D  pos;
			CVector3D tang;
			CVector3D lat;
			cved.GetObjStateInstant( cvedId, pos, tang, lat );
			double initYaw = atan2( tang.m_j, tang.m_i );
			posori[0] = pos.m_x; posori[1] = pos.m_y; posori[2] = pos.m_z;
			posori[3] = 0; posori[4] = 0; posori[5] = initYaw;

			if ( cved.GetSol().GetObj( cved.GetObjSolId(cvedId) )->GetCategoryName() == "SpecialEffect" )
			{
			}
				
			else
				SetMovingObjectStates( cvedId, option, uniqueId, posori );
			explCount++;
		}else{
			SetLRIObjectStates( cvedId, option );
			lriCount++;
		}
	}

	//
	// Create new objects
	//
	for( i = 0; i < newCount; i++ ) 
	{
		cvedId = newList[i];
		CreateObject(cvedId);
	}
	vector<CDiGuyUpdateCommand> commands;
	CHcsmCollection::GetDiGuyCommandQueue(commands);
	for (auto itr = commands.begin(); itr != commands.end(); itr++){
		m_pRender->UpdateDiGuyObject(itr->m_Id,
			                         itr->m_size,
									 itr->m_command,
									 itr->m_params,
									 itr->m_dataLoad
									 );
	}
	CHcsmCollection::clearDiGuyCommandQueue();
	//
	// Delete objects
	//
	for( i = 0; i < delCount; i++ ) {
		m_pRender->DeleteObject(delList[i] );
	}

	m_pScen->GetObjectState( currList, m_state, currCount );
	for( i = 0; i < currCount; i++ ) {
		cvedId = currList[i];
		auto cstr = cved.GetObjName(cvedId);
		auto objType = cved.GetObjType(cvedId);
	    if (objType == eCV_VIRTUAL_OBJECT){
            cvTObjState objState;
            cved.GetObjStateInstant( cvedId, objState );
            SetVirtualObject(m_pRender,objState.virtualObjectState,cvedId);
            continue;
	    }
		bool notOwnVeh = ( cstr!= nullptr && string(cstr) != "Driver" );
		if( notOwnVeh ) {
			uniqueId = cvedId;
			double posori[6]; 
			posori[0] = m_state[i].anyState.position.x;
			posori[1] = m_state[i].anyState.position.y;
			posori[2] = m_state[i].anyState.position.z;

			if ( cved.GetObjType(cvedId) == eCV_TRAJ_FOLLOWER &&
				!strcmp(cved.GetSol().GetObj(cved.GetObjSolId(cvedId))->GetCategoryName().c_str(), "Walker") )
			{
				// walker type, set roll and pitch to 0 for now
				posori[3] = 0.0;
				posori[4] = 0.0;
			}
			else
			{
				if ( 1/*g_app.m_useRoll*/ )
					posori[3] = atan2( -m_state[i].anyState.lateral.k,
						m_state[i].anyState.lateral.i * m_state[i].anyState.tangent.j
						- m_state[i].anyState.lateral.j * m_state[i].anyState.tangent.i );
				else
					posori[3] = 0.0;
				if (1/* g_app.m_usePitch*/ )
					posori[4] = asin( m_state[i].anyState.tangent.k );
				else
					posori[4] = 0.0;
			}
			posori[5] = atan2(
				m_state[i].anyState.tangent.j, 
				m_state[i].anyState.tangent.i
				);



			// temporary precreation code
			const char* objName = cved.GetObjName(cvedId);
			SetMovingObjectStates( cvedId, -1, uniqueId, posori, &pos, &(m_state[i]) );
			
		}
	}
	vector<CDiguyRotationOverride> overrides;
	CHcsmCollection::GetDiGuyJointOverrides(overrides);
	CHcsmCollection::clearDiGuyJointOverides();
	map<int, COverrideDiGuyJointAngles> overOut;
	for (auto x : overrides) {
		int currentIndex = overOut[x.HCSMid]._numOverrides;
		overOut[x.HCSMid]._id = x.HCSMid;
		strncpy((overOut[x.HCSMid])._angles[currentIndex].name, x.joint.c_str(),32);
		auto& anglesDeg = overOut[x.HCSMid]._angles[currentIndex].angles;
		anglesDeg[0] = x.angles.RollA();
		anglesDeg[1] = x.angles.YawA();
		anglesDeg[2] = x.angles.PitchA();
		overOut[x.HCSMid]._numOverrides++;
	}
	for (auto x : overOut) {
		m_pRender->OverRideJointAngles(x.second);
	}
	
	return true;
}
 //////////////////////////////////////////////////////////////////////////////
//
// This function determines the state of a switch (identified by its CIGI 
// number) given the bitmap reflecting the state of the lights in 
// a vehicle.
//
// In effect, the function converts between the CVED convention of 
// representing lights and the CIGI model convention of representing lights.
//
/////////////////////////////////////////////////////////////////////////////// 
int
CGraphicsStateAdapter::VehicleSwitchStateFromVisualMask(
			int  switchNo, 
			int  visualStateMask,
			int modelType
			)
{
	static int lightStateMask[] = {
			0,
			cCV_HIGHBEAM_LIGHTS, 
			cCV_BRAKE_LIGHTS, 
			cCV_LEFT_TURN_SIGNAL,
			cCV_RIGHT_TURN_SIGNAL, 
			cCV_HAZARD_LIGHTS, 
			cCV_EMERGENCY,
			cCV_BACKUP_LIGHTS
	};

	static int doorStateMask[] = {
			cCV_DOOR_FRONTLEFT,
			cCV_DOOR_FRONTRIGHT,
			cCV_DOOR_BACKLEFT,
			cCV_DOOR_BACKRIGHT,
			cCV_DOOR_REARLEFT,
			cCV_DOOR_REARRIGHT 
	};

	if ( switchNo == 0 ) // driver's choice 
	{
		return ( ( visualStateMask >> 10 ) & 0x07 );
	}
	else if ( switchNo == 8 ) // license plate 
	{
		// not implemented in cved yet
	}
	// temporary solution, only back doors are animated
	else if ( switchNo == 9 ) // door animation
	{
		if ( (doorStateMask[2] & visualStateMask) || 
			(doorStateMask[3] & visualStateMask) )
		{
			return 4;  // child 4 of switch 9 is back door animation
		}
		else
		{
			return 0;  // child 0 of switch 9 is no animation
		}
	}

#ifdef IDOTMC
	if ( modelType>=cMotorcycleCigiIdMin && modelType<=cMotorcycleCigiIdMax )
		if ( switchNo==1 )
		{
			// headlights has three states, high beam, low beam and modulating
			// combine both the cCV_HIGHBEAM_LIGHTS bit and cCV_OPERATING_LIGHTS bit to
			// represent the three states
			if ( cCV_HIGHBEAM_LIGHTS & visualStateMask )
			{
				if ( cCV_OPERATING_LIGHTS & visualStateMask )
					return 3;  // modulating light
				else
					return 2;  // high beam
			}
			else
			{
				if ( cCV_OPERATING_LIGHTS & visualStateMask )
					return 1;  // low beam
				else
					return 0;  // no light
			}
		}
#endif

	if ( switchNo < 0 || switchNo >= sizeof(lightStateMask) / sizeof(lightStateMask[0]) ) 
	{
		return 0;
	}

	return (lightStateMask[switchNo] & visualStateMask) ? 1 : 0;
}
bool
CGraphicsStateAdapter::GetCvedObjCigiInfo( CCved& cved, int cvedId, int& uniqueId, int& modelType )
{
	int            solId    = cved.GetObjSolId( cvedId );
	const CSolObj* cpSolObj = cved.GetSol().GetObj( solId );


	//
	// Get the CIGI id.
	//
	int cigiId;
	if( cpSolObj )
	{
		cigiId = cpSolObj->GetVisModelCigiId();
		bool invalidCigiId = cigiId < 0;
		if( invalidCigiId )
		{
			cerr << "Nix: invalid cigiId (" << cigiId;
			cerr << ") for cvedId = " << cvedId << " solId = ";
			cerr << solId << "  name = " << cved.GetObjName( cvedId );
			cerr << " skipping..." << endl;

			return false;
		}
	}
	else
	{
		cerr << "Nix: unable to get VisMdoelCigiId for sol object = ";
		cerr << solId << endl;
		cigiId = 0;
	}

	//
	// Get the CIGI color.
	//
	int objColorIndex = cved.GetObjColorIndex( cvedId );
	const CSolObjVehicle* cpSolVehObj = dynamic_cast<const CSolObjVehicle*>( cpSolObj );
	int cigiColor = 0;
	if( cpSolVehObj )
	{
		cigiColor = cpSolVehObj->GetColor( objColorIndex ).cigi;
	}

	uniqueId = cvedId;
	modelType = cigiId + cigiColor;

//	cout << "$$ solId = " << solId << "  cigiId = " << cigiId << "  cigiColor = " << cigiColor << "  colorIndex = " << objColorIndex << endl;
	return true;
}
/////////////////////////////////////////////////////////////////////////////
// If we have a DiGuy Object, we will need to construct a slightly different
// updated procedure
//
/////////////////////////////////////////////////////////////////////////////
void CGraphicsStateAdapter::DoDiGuyUpdate(
			int cvedId,
			int option,
			int uniqueId,
			double posori[6], //= NULL,
			CPoint3D* eyepos, //= NULL,
			cvTObjState* state //= NULL	
			){
	//state->trajFollowerState.classType
	if (state->trajFollowerState.classType == eCV_DIGUY_GUIDE_CONTROL ||
		state->trajFollowerState.classType == eCV_DIGUY_DIR_CONTROL) {
		int command;
		if (state->trajFollowerState.classType == eCV_DIGUY_GUIDE_CONTROL) {
			command = 6;
		}
		else {
			command = 8;
		}
		float data[]{ posori[0],posori[1],posori[2],posori[3],0,0 };
		m_pRender->UpdateDiGuyObject(cvedId,
			0,
			command,
			data,
			nullptr
		);
	}
#ifdef USE_IGCOMM
	//process what ever here
	string testOutput = "Hello DiGuy";
	int size = testOutput.size();
	//m_networkRender.UpdateDiGuyObject(cvedId,&(testOutput[0]),&size);
#endif
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///	Update state of a moving object for the graphics API
///\remark
/// Given a moving(non-LRI) object's Ids, this function sets its
/// position, rotation as well as switches, DOF's and animations.
/// Null pointer to position array indicates position does not
/// need to be updated.
///\todo
///	This function needs to be re-written.
/////////////////////////////////////////////////////////////////////////////
void
CGraphicsStateAdapter::SetMovingObjectStates(
			int cvedId,
			int option,
			int uniqueId,
			double posori[6], //= NULL,
			CPoint3D* eyepos, //= NULL,
			cvTObjState* state// = NULL
			){
	static int frontLeftTireCigiNo = 2, frontRightTireCigiNo = 1, rearTiresCigiNo = 3;

	CCved& cved = m_pScen->GetCved(); 
	short switchState[MAX_SWITCHES_PER_MODEL];
	CDOFObjItem DOFState[MAX_DOFS_PER_MODEL], OSGDOFState[MAX_DOFS_PER_MODEL];
	CUpdateObjItem::SAnimState OSGAnimState[MAX_ANIMATIONS_PER_MODEL];
	int switchCount = 0, DOFCount = 0, OSGDOFCount = 0, animCount = 0, OSGAnimCount = 0;
	int i;
	double rollRate = 0, steerAngle = 0, steerRate = 0;
	double distance2;
	string categoryName;
	
	// temporary debug code 
	static bool fallGuyState = true;
	static int fallGuyMode = 0;
	cvEObjType objType = cved.GetObjType(cvedId);
	//check to see if we have a DDO, it could be a DiGuy Object
	if (objType == eCV_TRAJ_FOLLOWER){
		auto id = cved.GetObjSolId(cvedId);
		if (id > 0){
			auto solObj = cved.GetSol().GetObj(id);
			if (solObj && solObj->GetCategoryName() == "DiGuy"){
				DoDiGuyUpdate(cvedId,option,uniqueId,posori,eyepos,state);
				return;
			}
		}
	}
	switch( cved.GetObjType(cvedId) )
	{
	case eCV_TRAJ_FOLLOWER :
		categoryName = cved.GetSol().GetObj( cved.GetObjSolId(cvedId) )->GetCategoryName();
		if ( categoryName == "Vehicle" )
		{

			// mask upper bits, something is going on there that is currently unclear
			int visualStateMask = cved.GetVehicleVisualState(cvedId) & 0xFFFF; 
			int actualId, modelType; 
			bool gotInfo = GetCvedObjCigiInfo( cved, cvedId, actualId, modelType );
			if ( !gotInfo )
				modelType = -1;
			for ( i=0; i<MAX_SWITCHES_PER_MODEL; ++i ) {
				switchState[i] = VehicleSwitchStateFromVisualMask(i, visualStateMask, modelType);
			}
			switchCount = MAX_SWITCHES_PER_MODEL;
			
			double bodyYawDelta;
			if ( state )
			{
				bodyYawDelta = posori[5] - m_bodyYaw[cvedId]; 
				if ( bodyYawDelta > M_PI ) // >= M_PI*2.0 - 0.001 )
					bodyYawDelta -= M_PI*2.0;
				else if ( -bodyYawDelta > M_PI ) // >= M_PI*2.0 - 0.001 )
					bodyYawDelta += M_PI*2.0;
				rollRate = state->trajFollowerState.vel*63360/3600/20*180/M_PI; // vel is mph, assuming 20" tires
				if ( rollRate ) 
				{
						steerAngle = 2*atan(144.0*bodyYawDelta/
							(2*state->trajFollowerState.vel*63360/3600/60))*180/M_PI;
						steerAngle /= 2.0; // the difference in body yaw is for two frames, so halve it
				}
				else
					steerAngle = m_tireYaw[cvedId]; 
				
				m_tireYaw[cvedId] = steerAngle;
				m_bodyYaw[cvedId] = posori[5];
			}
			
			if ( posori && eyepos )
				distance2 = fabs(posori[0] - eyepos->m_x )
					+ fabs(posori[1] - eyepos->m_y )
					+ fabs(posori[2] - eyepos->m_z);
			else
				distance2 = 1500;


			if ( distance2 < 1000 ) // less than 1000 feet away, animate DOF
			{
				DOFCount = 6;
				DOFState[0].DOFId                = 100;
				DOFState[0].DOFEnabledFields     = DOF_H | DOF_R;
				DOFState[0].roll                 = state->vehicleState.vehState.tireRot[0];
				DOFState[0].yaw                  = (float)-steerAngle;   // YH 5/11/12: flipped the sign
											     
				DOFState[1].DOFId                = 101;
				DOFState[1].DOFEnabledFields     = DOF_H | DOF_R;
				DOFState[1].roll                 = state->vehicleState.vehState.tireRot[1];
				DOFState[1].yaw                  = (float)-steerAngle;   // YH 5/11/12: flipped the sign
											     
				DOFState[2].DOFId                = 102;
				DOFState[2].DOFEnabledFields     = DOF_R;
				DOFState[2].roll                 = state->vehicleState.vehState.tireRot[2];


				// additional wheels if they exist, mostly for semis with trailers
				DOFState[3].DOFId               = 103;
				DOFState[3].DOFEnabledFields    = DOF_R;
				DOFState[3].roll                = state->vehicleState.vehState.tireRot[0];
				DOFState[4].DOFId               = 104;
				DOFState[4].DOFEnabledFields    = DOF_R;
				DOFState[4].roll                = state->vehicleState.vehState.tireRot[0];
				DOFState[5].DOFId               = 105;
				DOFState[5].DOFEnabledFields    = DOF_R;
				DOFState[5].roll                = state->vehicleState.vehState.tireRot[0];

				// now set up OSG version of DOF states
				OSGDOFCount = 3;
				OSGDOFState[0].DOFId            = 0;
				OSGDOFState[0].DOFEnabledFields = DOF_H | DOF_P;
				OSGDOFState[0].pitch            = (float)(state->vehicleState.vehState.tireRot[0]*M_PI/180.0);
				OSGDOFState[0].yaw              = (float)(-steerAngle*M_PI/180.0);
				OSGDOFState[0].roll             = 0;
				OSGDOFState[0].x                = OSGDOFState[0].y = OSGDOFState[0].z = 0;

				OSGDOFState[1].DOFId            = 1;
				OSGDOFState[1].DOFEnabledFields = DOF_H | DOF_P;
				OSGDOFState[1].pitch            = state->vehicleState.vehState.tireRot[1]*M_PIf/180.0f;
				OSGDOFState[1].yaw              = (float)(-steerAngle*M_PI/180.0);
				OSGDOFState[1].roll             = 0;
				OSGDOFState[1].x                = OSGDOFState[1].y = OSGDOFState[1].z = 0;


				OSGDOFState[2].DOFId            = 2;
				OSGDOFState[2].DOFEnabledFields = DOF_P;
				OSGDOFState[2].pitch            = state->vehicleState.vehState.tireRot[2]*M_PIf/180.0f;
				OSGDOFState[2].yaw              = OSGDOFState[2].roll = 0;
				OSGDOFState[2].x                = OSGDOFState[2].y = OSGDOFState[2].z = 0;
			}
			else
				DOFCount = 0;

		}
		else if ( option >= 0 )
		{
			switchState[0] = option;
			switchCount    = 1;
		}
		
		// animation control not working as specified in the Mantis Cigi 2.0 extensions 
		// document, commented out for now

		// Check for animation control
		// We are temporarily borrowing visual state bitmap for animation
		// state with the assumption that no vehicles have controllable 
		// animations. Therefore there will not be a conflict between 
		// the various visual states associated with vehicles and the 
		// animation state of the walker.
		if ( categoryName != "Vehicle" )
		{


			if (state && (state->trajFollowerState.visualState & 0x8000))
			{
				// now set up OSG version of animation states
				OSGAnimState[0].animationId = 0; // first animation node of the object
				OSGAnimState[0].animationState = 0; // start animation right now
				OSGAnimState[0].animationVal1 = -1.0; // use initial frame duration values
				OSGAnimState[0].animationVal2 = 1; // one shot animation
				OSGAnimCount = 1;
			}
			else
			{
				// all the controlled-start animations are one shot,
				// no need to send a stop command to the visual server

				// in the future, we can use a special value for animation state
				// to indicate sending a stop command. This way there will still
				// be no need to send out stop command every time the animation
				// state is off.
			}
		}

		break;

	case eCV_VEHICLE :	{
		int actualId, modelType; 
		bool gotInfo = GetCvedObjCigiInfo( cved, cvedId, actualId, modelType );
		if ( !gotInfo )
			modelType = -1;
		// mask upper bits, something is going on there that is currently unclear
		int visualStateMask = cved.GetVehicleVisualState(cvedId) & 0xFFFF; 

		for ( i=0; i<MAX_SWITCHES_PER_MODEL; ++i ) {
			switchState[i] = VehicleSwitchStateFromVisualMask(i, visualStateMask, modelType);
		}
		switchCount = MAX_SWITCHES_PER_MODEL;
		
		if ( state )
		{
			rollRate   = state->vehicleState.vehState.vel/0.0254/20*180/M_PI; //  vel is m/s, assuming 20" tires
			steerAngle = state->vehicleState.vehState.steeringWheelAngle*180/M_PI;
			steerRate  = (steerAngle - m_tireYaw[cvedId])*60;
			steerAngle = steerAngle*0.15 + m_tireYaw[cvedId]*0.85;  // smooth it out a bit
			m_tireYaw[cvedId] = steerAngle;
		}
		
		if ( posori && eyepos )
		{
			distance2 = fabs(posori[0] - eyepos->m_x )
				+ fabs(posori[1] - eyepos->m_y )
				+ fabs(posori[2] - eyepos->m_z);
		}
		else
			distance2 = 1500;


//		distance2 = 800;
		// DOF's processed here
		if ( distance2 < 1000 ) // less than 1000 feet away, animate DOF
		{
			DOFCount = 6;
			DOFState[0].DOFId              = 100;
			DOFState[0].DOFEnabledFields   = DOF_H | DOF_R;
			DOFState[0].roll               = state->vehicleState.vehState.tireRot[0];
			DOFState[0].yaw                = (float)-steerAngle; // doesn't seem to be enough, triple up   // YH 5/11/12: flipped the sign
										   
			DOFState[1].DOFId              = 101;
			DOFState[1].DOFEnabledFields   = DOF_H | DOF_R;
			DOFState[1].roll               = state->vehicleState.vehState.tireRot[1];
			DOFState[1].yaw                = (float)-steerAngle*3;   // YH 5/11/12: flipped the sign
										   
			DOFState[2].DOFId              = 102;
			DOFState[2].DOFEnabledFields   = DOF_R;
			DOFState[2].roll               = state->vehicleState.vehState.tireRot[2];

			// additional wheels if they exist, mostly for semis with trailers
			DOFState[3].DOFId              = 103;
			DOFState[3].DOFEnabledFields   = DOF_R;
			DOFState[3].roll               = state->vehicleState.vehState.tireRot[0];
			DOFState[4].DOFId              = 104;
			DOFState[4].DOFEnabledFields   = DOF_R;
			DOFState[4].roll               = state->vehicleState.vehState.tireRot[0];
			DOFState[5].DOFId              = 105;
			DOFState[5].DOFEnabledFields   = DOF_R;
			DOFState[5].roll               = state->vehicleState.vehState.tireRot[0];

			// now set up OSG version of DOF states
			OSGDOFCount = 3;
			OSGDOFState[0].DOFId            = 0;
			OSGDOFState[0].DOFEnabledFields = DOF_H | DOF_P;
			OSGDOFState[0].pitch            = state->vehicleState.vehState.tireRot[0]*M_PIf/180.0f;
			OSGDOFState[0].yaw              = (float)(-steerAngle*3*M_PI/180.0); // doesn't seem to be enough, triple up
			OSGDOFState[0].roll             = 0;
			OSGDOFState[0].x                = OSGDOFState[0].y = OSGDOFState[0].z = 0;

			OSGDOFState[1].DOFId            = 1;
			OSGDOFState[1].DOFEnabledFields = DOF_H | DOF_P;
			OSGDOFState[1].pitch            = (float)(state->vehicleState.vehState.tireRot[1]*M_PI/180.0);
			OSGDOFState[1].yaw              = (float)(-steerAngle*3*M_PI/180.0);
			OSGDOFState[1].roll             = 0;
			OSGDOFState[1].x                = OSGDOFState[1].y = OSGDOFState[1].z = 0;

			OSGDOFState[2].DOFId            = 2;
			OSGDOFState[2].DOFEnabledFields = DOF_P;
			OSGDOFState[2].pitch            = (float)(state->vehicleState.vehState.tireRot[2]*M_PI/180.0);
			OSGDOFState[2].yaw              = OSGDOFState[2].roll = 0;
			OSGDOFState[2].x                = OSGDOFState[2].y = OSGDOFState[2].z = 0;
		}
		else
			DOFCount = 0;
		break;
	}

	case eCV_TRAILER :
		switchState[0] = option;
		switchCount = 1;
		break;

	case eCV_RAIL_VEH :
		switchState[0] = option;
		switchCount = 1;
		break;

	case eCV_TERRAIN :
		switchCount = 0;
		break;

	case eCV_TRAFFIC_LIGHT :  // TRAFFIC LIGHTS can't be explicit objs
		assert( 0 );
		break;

	case eCV_TRAFFIC_SIGN :
		switchState[0] = option;
		switchCount = 1;
		break;

	case eCV_COMPOSITE_SIGN :
		{
			vector<CCompositeComponent>::const_iterator p;
			const CSolObjComposite*						pObj;
			pObj = dynamic_cast<const CSolObjComposite *> 
						(cved.GetSol().GetObj(cved.GetObjSolId(cvedId)));
			if ( pObj  == 0 ) {
				fprintf(stderr, "Obj of composite type of not of proper SOL category");
				assert(0);
				exit(0);
			}

			switchCount = (int)pObj->GetReferences().size();
			cvTObjState state;
			//printf("HERE, object is %d ***, with %d switches\n", cvedId, switchCount);
			cved.GetObjStateInstant(cvedId, state);
			for (int ii=0; ii < switchCount; ii++) {
				switchState[ii] = state.compositeSignState.childrenOpts[ii];
				//printf("Option %d set to %d\n",
				//	ii, state.compositeSignState.childrenOpts[ii]);
			}
		};
		break;

	case eCV_OBSTACLE :
		switchState[0] = option;
		switchCount = 1;
		break;

	case eCV_POI :
		switchCount = 0;
		break;

	case eCV_COORDINATOR :
		switchCount = 0;
		break;

	case eCV_EXTERNAL_DRIVER :
		assert( 0 );
		break;

	case eCV_WALKER :
		switchState[0] = option;
		switchCount    = 1;
		break;

	default :
		fprintf( 
			stderr, 
			"Nix: SetMovingObjectStates: invalid CVED type = %d for objId = %d",
			cved.GetObjType(cvedId),
			cvedId
			);
		fflush( stderr );
		assert(0);

		return;
	}


	bool moving = (posori != NULL);

	if (true /*m_pRender*/){
		if ( moving )
			posori[5] -= 90.0 * cDEG_TO_RAD;
		
		const char* objName = cved.GetObjName(cvedId);

		if ( moving && (objName[0] == 'P' && objName[1] == 'A') ) // precreation, hide it
		{
			posori[0] = posori[1] = posori[2] = 0.0;
		}

		m_pRender->UpdateObject(
				uniqueId, 
				posori, moving,
				switchState, switchCount,
				OSGDOFState, OSGDOFCount,
				OSGAnimState, OSGAnimCount
				);
	}
}
bool 
CGraphicsStateAdapter::AddPreCreates(){
	CCved& cved = m_pScen->GetCved();
    //m_pScen->FindPreCreateObjects
///
	/// load pre-creation models for scenario objects
	///
	const map<string, set<int> > *pSceneObjMap;
	map<string, set<int> >::const_iterator mitr;
	m_pScen->GetSolColorNames( pSceneObjMap );

	int precCount = 0;
	for ( mitr = pSceneObjMap->begin(); mitr != pSceneObjMap->end(); ++mitr )
	{
		const CSolObj* cpSolObj = m_pScen->GetCved().GetSol().GetObj( mitr->first );
		//
		// Get the CIGI id.
		//
		int cigiId, modelType;

		if( cpSolObj )
		{
			cigiId = cpSolObj->GetVisModelCigiId();
			bool invalidCigiId = cigiId < 0;
			if( invalidCigiId )
			{
				cout<<"Error: Pre-creation: Sol obj"<<mitr->first.c_str()<<" has no valid cigi id";
				continue;
			}
		}
		else
		{
			cout<<"Error: Sol obj"<<mitr->first.c_str()<<" is invalid";
			continue;
		}

		//
		// Get the CIGI color.
		//
		const CSolObjVehicle* cpSolVehObj = dynamic_cast<const CSolObjVehicle*>( cpSolObj );
		if ( cpSolVehObj )
		{
			set<int>::const_iterator sitr;
			for ( sitr=mitr->second.begin(); sitr!=mitr->second.end(); ++sitr )
			{
				int objColorIndex = *sitr;
				int cigiColor = cpSolVehObj->GetColor( objColorIndex ).cigi;
				modelType = cigiId + cigiColor;
				// load the model
					//g_pOsgRend->CreateObject(modelType,40000 + precCount,0,0,0,0,0,0);
					//g_pOsgRend->DeleteObject(40000 + precCount);
				m_pRender->PreloadModel(modelType);

				++precCount;	
			}
				
		}
		else
		{
			// not a vehicle, ignore color indices
			const CSolObjDiGuy* cpDiGuyObj = dynamic_cast<const CSolObjDiGuy*>( cpSolObj );
			if (cpDiGuyObj)
				continue; //DiGuy Objects are precreated at start of the IG.
			modelType = cigiId;
			if (m_pRender){
				//g_pOsgRend->CreateObject(modelType,40000 + precCount,0,0,0,0,0,0);
				//g_pOsgRend->DeleteObject(40000 + precCount);
				//g_pOsgRend->PreloadModel(modelType);
			}
			++precCount;
		}
	}
    CScenarioControl::TVirtModelList models;
    m_pScen->GetVirtObjectsModels(models);
    for (auto itr = models.begin(); itr != models.end(); ++itr){
        m_pRender->CreateVirtualObjectModel(**itr);
    }
    return true;
}
bool 
CGraphicsStateAdapter::InitStaticObjects(){

	vector<int>				objs;
	vector<int>::iterator	pO;
	CCved& cved = m_pScen->GetCved(); 
	cved.GetAllStaticObjs(objs);

	bool g_foundClutter = false;

	for( int i = 0; i < (int)objs.size(); i++ )	{
		int cvedId = objs[i]; 
		int uniqueId;
		int modelType;

		CPoint3D  pos1;
		CVector3D tang1;
		CVector3D lat1;
		cved.GetObjStateInstant( cvedId, pos1, tang1, lat1 );

		int option;
		cved.GetObjOption( cvedId, option );
		if( cved.IsExplicitObj( cvedId ) ) 
		{
			// 
			// For explicit objects: create them and set their switch.
			//
			CPoint3D  pos;
			CVector3D tang;
			CVector3D lat;
			cved.GetObjStateInstant( cvedId, pos, tang, lat );
//			double initYaw = atan2( tang.m_j, tang.m_i );
			double initRot[3];

			initRot[0] = atan2( -lat.m_k, lat.m_i * tang.m_j - lat.m_j * tang.m_i );
			initRot[1] = asin( tang.m_k );
			initRot[2] = atan2( tang.m_j, tang.m_i );

			bool gotInfo = GetCvedObjCigiInfo( cved, cvedId, uniqueId, modelType );
			if( gotInfo )
			{	
				if ( cved.GetSol().GetObj( cved.GetObjSolId(cvedId) )->GetCategoryName() == "SpecialEffect" )
				{


					if ( modelType >= 3000 || modelType < 4000 ) // smoke effect
					{
						// The orientation of a smoke effect is determined by the wind 
						// condition, and its orientation needs to be set to 0
						initRot[0] = initRot[1] = initRot[2] = 0;
					}
					double posRot[6] = { pos.m_x, pos.m_y, pos.m_z, initRot[0], initRot[1], initRot[2] };
					if (m_pRender)
						m_pRender->CreateSpecialEffect(modelType, uniqueId, posRot,EMPTY_OBJ_ID, NULL);
				}
				else
				{
					if (m_pRender){
						m_pRender->CreateObject(
								modelType, 
								uniqueId,
								pos.m_x, pos.m_y, pos.m_z, 
								initRot[0], initRot[1], initRot[2],0,
                                cSTATIONARY_OBJECT
								);
					}

					// moving object for the time being since we're missing
					// some creates
					if (m_pRender){
						m_pRender->UpdateObject(uniqueId, pos.m_x, pos.m_y, pos.m_z, initRot[0], initRot[1], initRot[2] );
					}

					SetMovingObjectStates( cvedId, option, uniqueId );
				}
				
			}
		}
		else if ( option != 0 ) 
		{
#ifdef DEBUG_LRI_OBJ
			cout << "## Visuals setup: setting option for LRI static object named '";
			cout << m_pScen->GetCved().GetObjName( cvedId );
			cout << "'  objId " << cvedId << " cigiId  " << m_pScen->GetCved().GetObjCigiId( cvedId );
			cout << "  option = " << option << endl;
#endif

			SetLRIObjectStates( cvedId, option );
			// traffic lights won't be set here since its option is always 0,
			//  it's its state that's changing.
		}
	}
	return true;
}
/////////////////////////////////////////////////////////
///\brief 
///   copy headlight setting from scenario to graphics
////////////////////////////////////////////////////////
bool
CGraphicsStateAdapter::UpdateHeadlights(){

    CHcsmCollection::THeadlightSetting setting;

	m_pScen->GetOwnshipLights(setting);
	m_pRender->CreateOwnshipHeadlights( 
        setting.On, 
		setting.Azimuth, 
        setting.Elevation, 
        setting.BeamWidth, 
        setting.BeamHeight,
        setting.ConstCoeff, 
        setting.LinearCoeff, 
        setting.QuadCoeff, 
        setting.HeightFade, 
        setting.Intensity, 
        setting.CutOffDist,
        setting.LampSeparation,
        setting.LampForwardOffset
				);
	
	return true;

}
///////////////////////////////////////////////////////////////////////////////
///
/// This function determines the state of a switch (identified by its CIGI 
/// number) given the bitmap reflecting the CVED state of a traffic lights.
///
/// In effect, the function converts between the CVED convention of 
/// representing traffic lights and the CIGI model convention of representing 
/// traffic lights.
///
/// 
int 
CGraphicsStateAdapter::TrafLightSwitchStateFromTrafficLightState(
			int  cvedTrafficLightState) const
{
	int switchState;

	switch ( (eCVTrafficLightState)cvedTrafficLightState ) {
	case eOFF:
		switchState = 0;
		break;
	case eRED: 
		switchState = 1;
		break;
	case eGREEN:
		switchState = 2;
		break;
	case eFLASH_GREEN:
		switchState = 6;
		break;
	case eYELLOW:
		switchState = 3;
		break; 
	case eFLASH_YELLOW:
		switchState = 4;
		break; 
	case eFLASH_RED:
		switchState = 5;
		break;
	case eRED_TURN_LEFT:
		switchState = 7;
		break;
	case eYELLOW_TURN_LEFT:
		switchState = 9;
		break;
	case eGREEN_TURN_LEFT:
		switchState = 8;
		break;
	case eRED_TURN_RIGHT:
		switchState = 13;
		break;
	case eYELLOW_TURN_RIGHT:
		switchState = 15;
		break;
	case eGREEN_TURN_RIGHT:
		switchState = 14;
		break;
	case eFLASH_RED_TURN_LEFT:
		switchState = 11;
		break;
	case eFLASH_YELLOW_TURN_LEFT:
		switchState = 10;
		break;
	case eFLASH_GREEN_TURN_LEFT:
		switchState = 12;
		break;
	case eFLASH_RED_TURN_RIGHT:
		switchState = 17;
		break;
	case eFLASH_YELLOW_TURN_RIGHT:
		switchState = 16;
		break;
	case eFLASH_GREEN_TURN_RIGHT:
		switchState = 18;
		break;
	case eRED_STRAIGHT:
		switchState = 19;
		break;
	case eGREEN_STRAIGHT:
		switchState = 20;
		break;
	case eYELLOW_STRAIGHT:
		switchState = 21;
		break;
	case eFLASH_RED_STRAIGHT:
		switchState = 23;
		break;
	case eFLASH_YELLOW_STRAIGHT:
		switchState = 22;
		break;
	case eFLASH_GREEN_STRAIGHT:
		switchState = 24;
		break;
	default:
		switchState = 0;
		break;
	}

	return switchState;
}

void
CGraphicsStateAdapter::SetLRIObjectStates(
			int cvedId,
			int option
			)
{
	CCved& cved = m_pScen->GetCved(); 
	int cigiId = cved.GetObjCigiId( cvedId );
	if( cigiId < 0 )
	{
		fprintf(
			stdout,
			"LRI object #%d %s has cigiId = %d\n", 
			cvedId, cved.GetObjName( cvedId ),
			cigiId
			);
		fflush( stdout );
		return;
	}

	//if ( cigiId == 122 )   // hack!!!
	//	g_foundClutter = true;

	switch( cved.GetObjType(cvedId) )
	{
	case eCV_TRAFFIC_LIGHT :
	{
		int switchState = TrafLightSwitchStateFromTrafficLightState( cved.GetTrafficLightState( cvedId ) );
		m_pRender->CRendererIface::UpdateSwitch(
					cigiId, 
					switchState 
					);

		//if ( cigiId == 222 )
			//printf("Set: traffic light, id=%d, cigiNo=%d, state=%d\n", cvedId, cigiId, cved.GetTrafficLightState( cvedId ));
		break;
	}

	case eCV_TERRAIN :
		if (m_pRender){
			m_pRender->CRendererIface::UpdateSwitch(
					cigiId, 
					option 
					);
		}
		break;

	case eCV_TRAFFIC_SIGN :
		if (m_pRender){
			m_pRender->CRendererIface::UpdateSwitch(
					cigiId, 
					option 
					);
		}
//		printf("Set: traffic sign, id=%d, cigiNo=%d, state=%d\n", cvedId, cigiId, option	);
		break;

	case eCV_OBSTACLE :
		if (m_pRender){
			m_pRender->CRendererIface::UpdateSwitch(
					cigiId, 
					option 
					);
		}
		break;

	case eCV_POI :
		if (m_pRender){
			m_pRender->CRendererIface::UpdateSwitch(
					cigiId, 
					option 
					);
		}
		break;

	case eCV_COMPOSITE_SIGN :
		// work, multiple switches
		break;

	default :
		fprintf( 
			stderr, 
			"Nix: SetLRIObjectStates: invalid CVED object type = %d", 
			cved.GetObjType(cvedId) 
			);
		fflush( stderr );
		assert(0);

		return;
	}

	// set the states
}
void ConvEnvArea(const CEnvArea& area, CEnviroCond &env){
	float windSpeed = -1.0f, windDirection[2] = {1, 0};
    float fogDist = -1;
	float visibility = -10000.0f;
	vector<cvTEnviroInfo> conditions;
    area.GetCndtns( conditions );
    for( auto xx = conditions.begin(); xx != conditions.end(); xx++ )
    {
    	if ( xx->type == eWIND ) // and the flag for overwriting global condition is set 
    	{
    		windSpeed        = (float) xx->info.Wind.vel;
    		windDirection[0] = (float) xx->info.Wind.dir_i;
    		windDirection[1] = (float) xx->info.Wind.dir_j;
    	}
    	else if ( xx->type == eVISIBILITY )
    	{
               env.m_PrecipType = CEnviroCond::ESand;
               env.m_FogIntensity = 
    			(float) (xx->info.Visibility.dist );
    	}
        else if (xx->type == eFOG){ //densest fog wins
            env.m_PrecipType = CEnviroCond::EFog;
            if ( (xx->info.Fog.dist < env.m_FogDist) || env.m_FogDist == 0){
                env.m_EnableFog = true;
                env.m_FogDist = (float)xx->info.Fog.dist;
            }
        }
    }
	if ( windSpeed >=0 ) // found local wind
	{
		env.m_WindVel = windSpeed * 1609 / 3600;
		env.m_WindDir = float(-atan2( windDirection[1], windDirection[0] ) * cPI * 180);
	}
	if ( visibility >=0 ) // found local visibility
	{
        //env.m_FogDist = visibility;
    }
}
bool
CGraphicsStateAdapter::SetEnv(){
    CEnviroCond env;
    CCved& cved = m_pScen->GetCved();
    int year, month, day;
	m_pScen->GetDateAndTime( year, month, day, env.m_Hours, env.m_Mins );
	env.m_Date = year + month*1000000 + day*10000;

	bool sunEntityEnabled, sunlightEnabled, moonEntityEnabled, moonlightEnabled;
	float sunlightIntensity, moonlightIntensity;

	m_pScen->GetSkyModelValues( sunEntityEnabled, sunlightEnabled, 
		moonEntityEnabled, moonlightEnabled, sunlightIntensity, moonlightIntensity );
	env.m_EnableSunLight = moonlightEnabled;
	env.m_AmbientLightInt = moonlightIntensity;

	// set up environmental conditions 
	bool enviroChanged = false;

	// get local environment conditions
	vector<CEnvArea> envArea;
    CPoint3D pos;
    cved.GetOwnVehiclePos(pos);
    auto global = cved.GetGlobalEnvArea();
	ConvEnvArea(global,env);
	cved.GetEnvArea( pos, envArea );
	if( !envArea.empty() ) 
	{
		//
		// Look for wind effects.
		//
		vector<cvTEnviroInfo>::const_iterator xx;
		vector<CEnvArea>::const_iterator yy;
		for( yy = envArea.begin(); yy < envArea.end(); yy++ )
		{
			if ( yy->IsValid() )
			{
				ConvEnvArea(*yy,env);
			}
		}
	}
	m_pRender->SetEnvironment(env);
    return true;
}

bool CGraphicsStateAdapter::InitVirtualObjects(){
    CScenarioControl::TVirtModelList models;
    m_pScen->GetVirtObjectsModels(models);
    for (auto itr = models.begin(); itr != models.end(); ++itr){
        m_pRender->CreateVirtualObjectModel(**itr);
    }
    return true;
}
/*$Log: GraphicsStateAdapter.cpp,v $
/*Revision 1.21  2018/09/12 20:41:01  IOWA\dheitbri
/*added DiGuy update
/*
/*Revision 1.20  2018/09/07 14:32:43  IOWA\dheitbri
/*added diGuy actions for overriding joint angles
/*
/*Revision 1.19  2018/03/27 14:48:55  IOWA\dheitbri
/*took care of warnings
/*
/*Revision 1.18  2017/10/12 17:02:27  IOWA\dheitbri
/*updated headlight settings, added a constructor
/*
/*Revision 1.17  2017/08/16 20:20:25  IOWA\dheitbri
/*added new set env condition action, moved headlights to HCSMCollection
/*
/*Revision 1.16  2017/06/01 20:07:00  IOWA\dheitbri
/**** empty log message ***
/*
/*Revision 1.15  2017/05/03 16:58:48  IOWA\dheitbri
/*added new actions, updated headlights
/*
/*Revision 1.14  2017/01/30 21:08:43  IOWA\dheitbri
/*fixed some env issues
/*
/*Revision 1.13  2017/01/24 22:52:33  IOWA\dheitbri
/*fixed issue with bad compare
/*
/*Revision 1.12  2016/11/15 22:59:54  IOWA\dheitbri
/*updated the fog settings
/*
/*Revision 1.11  2016/10/28 20:52:46  IOWA\dheitbri
/*fixed some warnings
/*
/*Revision 1.10  2016/10/12 22:47:30  IOWA\dheitbri
/*removed alot of dead code
/*
/*Revision 1.9  2016/07/15 14:47:51  IOWA\dheitbri
/*switched the tire state to veh dynamics
/*
/*Revision 1.8  2016/01/29 15:43:21  IOWA\dheitbri
/*fixed type caste issue
/*
/*Revision 1.7  2016/01/15 17:25:22  iowa\dheitbri
/*added missing clear ids
/*
/*Revision 1.6  2016/01/08 00:28:13  IOWA\dheitbri
/**** empty log message ***
/*
/*Revision 1.5  2015/12/17 22:25:38  IOWA\dheitbri
/*added set switch cleaned up some of the set object code, this is still way dirty code.
/*
/*Revision 1.4  2015/09/17 16:16:06  IOWA\dheitbri
/*switched add shader commands to add visual commands, added set switch
/*
*/