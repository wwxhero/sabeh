/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: ddo.cxx,v 1.104 2014/04/10 23:40:31 IOWA\dheitbri Exp $
 *
 * Author(s):    Paolo Ammann, Hao Xu, Yefei He
 * Date:         January, 1999
 *
 * Description:  Contains code for the Deterministic Dynamic Object (DDO).
 *               A DDO is also known as a trajectory follower.
 *
 ****************************************************************************/
#define _USE_MATH_DEFINES

#include "genhcsm.h"
#include "hcsmcollection.h"
#include "util.h"
#include <boost/tokenizer.hpp>

#undef TF_DEBUG 
#undef DEBUG_DDO_MONITOR


#undef DEBUG_FILE
#ifdef DEBUG_FILE
ofstream*   m_pLogFile;
#endif

void 
CDdo::UserCreation( const CDdoParseBlock* cpSnoBlock )
{
	PrintCreationMessage();

	// 
	// Initialize member variables.
	//
	
	m_isDiGuy = false; //we will figure this out when we do CVED creation
	m_lastState = eWAIT;
	m_pDdo = NULL;
	m_prevRoadPosFrame = -1;
	m_doOverideSpeed = false;
	m_speedOveride = -1.0;
	m_isFirstRun = true; //we do not want to apply accel constraints the first iteration
	//
	// Read initialization parameters from the SCN file.
	//
	m_initConditions.SetObjectInitCond(
				*cved, 
				GetFrame(), 
				GetTimeStepDuration(),  
				cpSnoBlock->GetCrRad(),  
				cpSnoBlock->GetActvDel(), 
				cpSnoBlock->GetLifetime()
				);
	
	m_quitAtEnd = cpSnoBlock->GetQuitAtEnd();
	m_dependent = cpSnoBlock->GetDependent();

	m_trajIsGlobal = cpSnoBlock->GetTrajIsGlobal();
	m_parentName = cpSnoBlock->GetCoupledName(); // parent id will be assigned during user activity
	m_curModeDialVal = -1; 
	m_useInitVel = cpSnoBlock->GetUseInitVelocity();
	m_currentVel = cpSnoBlock->GetInitdDDOVelocity() * cMPH_TO_FTPS;
	m_enableAni = cpSnoBlock->GetEnableAni();
	// Initial position offset. In the object relative trajectory mode, the offset field 
	// in the state will be used to store current relative position with this initial offset 
	// already added in, but the GetControlInput() function needs the current position along 
	// the relative path, i.e. without the initial offset being added. Remember to update this 
	// offset when there is a mode transition, since the initial offset may be changed too.
	m_initPosOffset = cpSnoBlock->GetRelativeLocation();

	m_maxAccel_fps2 = cpSnoBlock->GetMaxAccel() * GetTimeStepDuration();
	m_maxDecel_fps2 = cpSnoBlock->GetMaxDecel() * GetTimeStepDuration(); 
	m_takeAccelIntoAccount = cpSnoBlock->GetAccountAccel();
	// 
	// Use the trajectory follower's path to initialize the spline.
	//
	double z = cpSnoBlock->GetInitZ(), x0 = 0, y0 = 0;
	bool relativeTraj;
	int c;
	vector<CDdoParseBlock::TTrajPoint> traj = cpSnoBlock->GetTraj();
	vector<CDdoParseBlock::TTrajPoint>::const_iterator pTrajPoint;
	vector<double> delays = cpSnoBlock->GetDelays();


	// first trajectory point
	if ( !traj.empty() )
	{
		x0 = traj.begin()->x;
		y0 = traj.begin()->y;
	}

	for(
		pTrajPoint = traj.begin(), c = 0; 
		pTrajPoint != traj.end(); 
		pTrajPoint++, c++ 
		) 
	{
		double velocity = pTrajPoint->vel * cMPH_TO_FTPS;
		m_velocityVector.push_back( velocity );
		m_delayVector.push_back( delays[c] );

		if ( m_trajIsGlobal )
			m_spline.addPoint( pTrajPoint->x, pTrajPoint->y, z );
		else
			m_spline.addPoint( pTrajPoint->x - x0, pTrajPoint->y - y0, 0 );
		m_spline.addTangent( c, pTrajPoint->i, pTrajPoint->j, 0 );
	}

	vector<double> orientations = cpSnoBlock->GetDirs();
	vector<double>::const_iterator pOri;
	for( pOri = orientations.begin(); pOri != orientations.end(); pOri++ ) 
	{
		m_orientVector.push_back( *pOri );
	}
	
	//
	// Read in the coordinates of the reference points for dependent
	// trajectory followers.
	//
	if( m_dependent ) 
	{
		m_vehicleReferencePoint = cpSnoBlock->GetDependentRefPoint();
		if( cpSnoBlock->GetRefPoint() == -1 ) 
		{
			m_spline.getPoint( m_spline.getCount()-1, m_referencePoint );
			m_target = m_spline.getCount()-1;
		} 
		else 
		{
			m_spline.getPoint( cpSnoBlock->GetRefPoint(), m_referencePoint );
			m_target = cpSnoBlock->GetRefPoint();
		}

		CPoint3D getPos;
		CPoint3D getDepRefPoint;
		getPos = cpSnoBlock->GetPosition();
		getDepRefPoint = cpSnoBlock->GetDependentRefPoint();
	}
		 
	m_spline.calc();
	m_spline.getPoint( 0, m_last3DPos );
	if ( !m_trajIsGlobal )
	// relative trajectory, add back the global position of the first post to 
	// m_last3DPos
	{
		m_last3DPos.m_x += x0;
		m_last3DPos.m_y += y0;
	}

#ifdef DEBUG_FILE
	//
	// Debugging.
	//
	string logFileName = "DDO.txt";
	bool wantToCreateLogFile = logFileName.size() > 0;
	if( wantToCreateLogFile  )
	{
		//
		// Open a data file for logging ADO information.
		//
		gout << MessagePrefix();
		gout << "Log file name is '" << logFileName << "'" << endl;

		m_pLogFile = new ofstream( logFileName.c_str() );

		if( !m_pLogFile )
		{
			gout << MessagePrefix() << "Cannot log DDO to file ";
			gout << "named '" << logFileName << "'" << endl;

			exit( -1 );
		}
	}  // end if haveLogFile
#endif

#if TF_DEBUG
	m_debugPointsFile = fopen("debugPoint.txt", "w");
	fprintf(
		m_debugPointsFile,
		"\t\tDDO Reference Point is: %lf\t%lf\n",
		m_vehicleReferencePoint.m_x, 
		m_vehicleReferencePoint.m_y 
		);
	fprintf( 
		m_debugPointsFile, 
		"\t\tDDDO Ref Point is: %lf\t%lf\n",
		m_referencePoint.m_x, 
		m_referencePoint.m_y 
		);
	fprintf(
		m_debugPointsFile, 
		"\t\tCVED Position           \tNew Position            \tDirection         \tVelocity\n"
		); 
	fprintf(
		m_debugPointsFile, 
		"\t\tX            Y          \tX            Y          \tX         Y       \tV\n"
		);	
#endif
}


void 
CDdo::UserActivity( const CDdoParseBlock* cpSnoBlock )
{	
	// Process any dial inputs
	HandleDials();

	// Process state of object init condition
	CPoint3D actualPosition;
	CVector3D actualOrientation;
	if( m_lastState == eUNDER_CONTROL ) 
	{
		actualPosition = m_pDdo->GetPosImm();
		actualOrientation = m_pDdo->GetTanImm();
	}
	else 
	{
		actualPosition = m_last3DPos;
	}

	int actualTime = GetFrame();
	EInitCond state;
	state = m_initConditions.Execute( actualPosition, actualTime );

	switch( state ) 
	{
		case eWAIT:
			// 
			// Waiting for InitCondition's approval.
			//
			break;

		case eACTIVATE:
			//
			// The trajectory follower is now activated.  Create CVED object.
			//
			actualOrientation.m_i = cos( m_orientVector[0] );
			actualOrientation.m_j = sin( m_orientVector[0] );
			CreateCvedObject( cpSnoBlock, actualPosition, actualOrientation );

			break;

		case eUNDER_CONTROL:
		{
			CTrajFollowerObj* pTrajFolObj = dynamic_cast<CTrajFollowerObj *>( m_pDdo );
					
			if 	( pTrajFolObj )
			{
				if ( (pTrajFolObj->GetCurrentMode() == eCV_OBJ_REL_TRAJ ||
					pTrajFolObj->GetCurrentMode() == eCV_COUPLED_OBJ) && (pTrajFolObj->GetParentId() == -1) )
				// parent id not set yet, attempt to find it
				{
					int parentId; 

					if ( cved->GetObj( m_parentName, parentId ) )
					{
						pTrajFolObj->SetParentId( parentId );
//						printf("Found parent %s of ddo %s as #%d\n", 
//							m_parentName.c_str(), cpSnoBlock->GetName().c_str(), parentId );
					}
				}

				if ( pTrajFolObj->GetCurrentMode() == eCV_OBJ_REL_TRAJ )
				{
					CPoint3D relativePosition;
					double offset[6];

					pTrajFolObj->GetOffset( offset );
					// subtract initial offset to get the relative position along the trajectory 
					relativePosition.m_x = offset[0] - m_initPosOffset.m_i;
					relativePosition.m_y = offset[1] - m_initPosOffset.m_j;
					relativePosition.m_z = offset[2] - m_initPosOffset.m_k;
					SetControlInputs( relativePosition );
				}
				else if ( pTrajFolObj->GetCurrentMode() == eCV_GROUND_TRAJ )
					SetControlInputs( actualPosition );
			}
			break;
		}

		case eDELETE:
			//
			// Lifetime has expired, so delete HCSM.
			//
			DeleteHcsm( this );
			break;

	} // end switch (state) 

	m_lastState = state;

#define USE_DDO_ROADPOS_MONITOR

#ifdef USE_DDO_ROADPOS_MONITOR
	{
		CPoint3D objPos = actualPosition;;

#ifdef DEBUG_DDO_MONITOR
		gout << "  ddo obj pos = " << objPos << endl;
#endif

		bool buildRoadPosFromScratch = (
					m_prevRoadPosFrame < 0 ||
					GetFrame() - m_prevRoadPosFrame > 1
					);
		if( buildRoadPosFromScratch )
		{
			CRoadPos roadPos( *cved, objPos );
			m_currRoadPos = roadPos;
		}
		else
		{
			m_currRoadPos.SetXYZ( objPos );
		}

		//
		// Set the RoadPos monitor.
		//
		if( m_currRoadPos.IsValid() )
		{
#ifdef DEBUG_DDO_MONITOR
			gout << "  valid roadPos = " << m_currRoadPos << endl;
#endif
			SetMonitorRoadPos( m_currRoadPos );
		}
		else
		{
#ifdef DEBUG_DDO_MONITOR
			gout << "  invalid roadPos" << endl;
#endif
			CRoadPos emptyRoadPos;
			SetMonitorRoadPos( emptyRoadPos );
		}

		if( m_currRoadPos.IsValid() )
			m_prevRoadPosFrame = GetFrame();
	}
#endif // USE_DDO_ROADPOS_MONITOR

#ifdef DEBUG_FILE
	if( m_pLogFile ) 
	{
		if( m_pDdo && m_pDdo->IsValid() ) 
		{
			CTrajFollowerObj* pTrajFolObj = dynamic_cast<CTrajFollowerObj *>( m_pDdo );
			CPoint3D currPos = pTrajFolObj->GetPos();
			*m_pLogFile << GetFrame() << " " << pTrajFolObj->GetVel() << " ";
			*m_pLogFile << currPos.m_x << " " << currPos.m_y << " " << currPos.m_z;
			*m_pLogFile << endl;
		}
	}
#endif
}  // UserActivity


void 
CDdo::UserDeletion( const CDdoParseBlock* cpSnoBlock )
{
	DebugText( CHcsmDebugItem::eDEBUG_ROUTINE, "Enter UserDeletion" );

	PrintDeletionMessage();

	//
	// Check to make sure that the CVED object is still valid
	// before deleting it.
	//
	bool cvedObjValid = m_pDdo && m_pDdo->IsValid();
	if( cvedObjValid ) 
	{
		cved->DeleteDynObj( m_pDdo );
	}

#ifdef DEBUG_FILE
	if( m_pLogFile )
	{
		gout << "Closing log file" << endl;
		delete m_pLogFile;
	}
#endif
}  // UserDeletion


//////////////////////////////////////////////////////////////////////////////
//	
//	Called by UserActivity()
//	Reads the dials and performs the necessary response.
//
//////////////////////////////////////////////////////////////////////////////
void 
CDdo::HandleDials( void ) 
{
	if( m_pDdo != 0 ) 
	{	
		if( m_dialMode.HasValue() && (GetDialMode()!= m_curModeDialVal) )
		{
			switch ( GetDialMode() ) {
			case CDdoParseBlock::eNORMAL:
				// shouldn't happen 
				break;
			case CDdoParseBlock::eFREE:
				// There are two cases, 1. An object rolling off another object. 
				// Initial position, rotation and velocities will be calculated 
				// from the current condition. 2. After an impact. Intial 
				// condition will be determined by the impact. The second 
				// case should not be triggered by a dial.
				cved->FreeObjectMotion( m_pDdo->GetId() );
				break;
			case CDdoParseBlock::eCOUPLED:
				// This is usually a transition after an impact when one object 
				// object is stuck to another. Again, is such transition determined 
				// by a dial setting?  

				cved->CoupledObjectMotion( m_pDdo->GetId() );
				break;
			case CDdoParseBlock::eRELATIVE:
				// This is usually a transition from coupled object mode.
				// Use the coupled object mode offset in position and rotation, 
				// and the same parent.
				double offset[6];

				m_pDdo->GetOffset( offset );
				m_initPosOffset.m_i = offset[0];
				m_initPosOffset.m_j = offset[1];
				m_initPosOffset.m_k = offset[2];

				cved->ObjRelTrajMotion( m_pDdo->GetId() );
				break;
			default:
				break;
			}
			m_curModeDialVal = GetDialMode();
		}

		if( m_dialVisualState.HasValue() ) 
		{
			m_pDdo->SetVisualState( GetDialVisualState() );

#if TF_DEBUG
			gout << MessagePrefix() << " visual state set to " 
				 << GetDialVisualState() << endl;
#endif

			m_dialVisualState.SetNoValue();
		}

		if( m_dialAudioState.HasValue() ) 
		{
			m_pDdo->SetAudioState( GetDialAudioState() );

#if TF_DEBUG
			gout << MessagePrefix() << " audio state set to " 
				 << GetDialAudioState() << endl;
#endif

			m_dialAudioState.SetNoValue();
		}
		if (m_dialDependent.HasValue()){
			m_dependent = m_dialDependent.GetValue();
			m_dialDependent.SetNoValue();
		}
		if (m_dialSpeedOverRide.HasValue() ){
			m_speedOveride = m_dialSpeedOverRide.GetValue();
			if (m_speedOveride >= 0){
				m_doOverideSpeed = true;
			}else{
				m_doOverideSpeed = false;
			}
			m_dialSpeedOverRide.SetNoValue();
		}
		if (m_isDiGuy &&m_dialDiGuyAction.HasValue() && m_dialDiGuyAction.GetValueStr() != m_currDiGuyActionDialString){
			typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
			boost::char_separator<char> sep(":;,");
			string dailstr = m_dialDiGuyAction.GetValueStr();
			m_currDiGuyActionDialString = dailstr;
			tokenizer tokens(dailstr, sep);
			tokenizer::iterator itr = tokens.begin();
			if (itr != tokens.end()){
				string command = *itr;
				if (command == "Action"){
					itr++;
					if (itr != tokens.end()){
						string actionName = *itr;
						float speed = -1;
						float duration = -1;
						itr++;
						if (itr != tokens.end()){
							stringstream converter;
							converter<<*itr;
							converter>>duration;
							itr++;
							if (itr != tokens.end()){
								stringstream converter;
								converter<<*itr;
								converter>>speed;
								m_speedOveride = speed;
							}//speed
						}//duration
						//we have a complete command for action
						CDiGuyUpdateCommand command;
						command.m_command = 1;
						command.m_Id = m_pDdo->GetId();
						command.m_params[0] = duration;
						command.m_params[1] = speed;
						command.m_size = actionName.size();
						strcpy_s(command.m_dataLoad,actionName.c_str());
						CHcsmCollection::AddDiGuyCommand(command);
					}//action Name
				}//command "Action"
				if (command == "Resume"){
					CDiGuyUpdateCommand command;
					command.m_command = 2;
					command.m_Id = m_pDdo->GetId();
					command.m_size = 0;
					CHcsmCollection::AddDiGuyCommand(command);
				}
				if (command == "SetSpeed"){
					CDiGuyUpdateCommand command;
					command.m_command = 3;
					command.m_Id = m_pDdo->GetId();
					command.m_size = 0;
					itr++;
					float speed = -1;
					if (itr != tokens.end()){
						string speedstr = *itr;
						stringstream converter;
						converter<<speedstr;
						converter>>speed;
						m_speedOveride = speed;
					}else{
						//what ever the default speed for this is;
					}
					command.m_params[0] = speed;
					CHcsmCollection::AddDiGuyCommand(command);
				}
				if (command == "Start"){
					CDiGuyUpdateCommand command;
					command.m_command = 4;
					command.m_Id = m_pDdo->GetId();
					command.m_size = 0;
					itr++;
					float speed = -1;
					if (itr != tokens.end()){
						string speedstr = *itr;
						stringstream converter;
						converter<<speedstr;
						converter>>speed;
						m_speedOveride = speed;
					}
					command.m_params[0] = speed;
					CHcsmCollection::AddDiGuyCommand(command);
				}
			}//if we have our first token
			m_dialDiGuyAction.SetNoValue();
		}
		if (GetButtonTurnOnAnimation()){
			m_isAnimationOn = true;
			m_pDdo->SetAnimationState(true);
			
		}
		if (GetButtonTurnOffAnimation()){
			m_isAnimationOn = false;
			m_pDdo->SetAnimationState(false);
		}
		
	}
}  // HandleDials


//////////////////////////////////////////////////////////////////////////////
//
// Called by UserActivity() when state = eACTIVATE.
// This function creates the CVED object for the trajectory follower.
// It first gets information about the object to be created from the SOL
// then creates the CVED object.
//
//////////////////////////////////////////////////////////////////////////////
void 
CDdo::CreateCvedObject( 
			const CDdoParseBlock* cpSnoBlock,
			const CPoint3D& cActualPos,
			const CVector3D& cActualOri
			)
{

	DebugText(CHcsmDebugItem::eDEBUG_ROUTINE, "Enter CreateCvedObject");

	const CSolObj* cpSolObj = cved->GetSol().GetObj( cpSnoBlock->GetSolName() );
	if( !cpSolObj ) 
	{
		cerr << MessagePrefix() << "Invalid SOL name '";
		cerr << cpSnoBlock->GetSolName() << "'...[SUICIDE]" << endl;

		Suicide();
		return;
	}
	const CSolObjDiGuy *pDiGuySol = dynamic_cast<const CSolObjDiGuy *>(cpSolObj);
	if (pDiGuySol){
		m_isDiGuy = true;
	}
	cvTObjAttr trajFollowerAttr = { 0 };
	trajFollowerAttr.solId = cpSolObj->GetId();
	trajFollowerAttr.xSize = cpSolObj->GetLength();
	trajFollowerAttr.ySize = cpSolObj->GetWidth();
	trajFollowerAttr.zSize = cpSolObj->GetHeight();
	trajFollowerAttr.colorIndex = cpSnoBlock->GetColorIndex();
	trajFollowerAttr.hcsmId = m_pRootCollection->GetHcsmId( this );
	
	// Create the CVED object.
	m_pDdo = dynamic_cast<CTrajFollowerObj*> ( cved->CreateDynObj(
				cpSnoBlock->GetName(),
				eCV_TRAJ_FOLLOWER, 
				m_typeId,
				trajFollowerAttr,
				&cActualPos,
				&cActualOri
				) );

	if( !m_pDdo || !m_pDdo->IsValid() ) 
	{
		cerr << MessagePrefix() << " Error creating CVED instance "
			 << "... [SUICIDE]" << endl;

		Suicide();
		return;
	}
	//for DiGuy Objects we must upload our entire path to DiGuy
	if (m_isDiGuy){
		vector<CDdoParseBlock::TTrajPoint> traj = cpSnoBlock->GetTraj();
		vector<float> m_tempPath;
		double initZ = cpSnoBlock->GetInitZ();
		double z;
		CVector3D norm;
		m_tempPath.reserve(traj.size() *4);
		m_doOverideSpeed = true;
		m_speedOveride = 0.0;
		vector<float> times = cpSnoBlock->GetDiGuyActionTime();
		vector<char> actions =cpSnoBlock->GetDiGuyActionType();
		for (vector<CDdoParseBlock::TTrajPoint>::iterator itr = traj.begin();
			itr != traj.end(); itr++)
		{
			m_tempPath.push_back(itr->x);
			m_tempPath.push_back(itr->y);
			z = initZ;
			cved->QryTerrain(itr->x,itr->y,initZ,z,norm,nullptr);
			m_tempPath.push_back(z);
			m_tempPath.push_back(atan2(itr->j,itr->i));
		}
		CHcsmCollection::SetDiGuyPathInfo(m_pDdo->GetId(),m_tempPath,actions,times);
	}
	//
	// Initialize the vehicle state and dynamics.  The SOL contains vehicle
	// dynamics settings particular to each vehicle.
	//
	CTrajFollowerObj* pTrajFolObj = dynamic_cast<CTrajFollowerObj *>( m_pDdo );

	//
	// Set the initial audio and visual state.
	//
	pTrajFolObj->SetAudioState( cpSnoBlock->GetAudioState() );
	pTrajFolObj->SetVisualState( cpSnoBlock->GetVisualState() );
	pTrajFolObj->InitModeValues();

	//CWalkerObj *pWalker = dynamic_cast<CWalkerObj *>(pTrajFolObj);
	//if (pWalker){
	pTrajFolObj->SetAnimationState( cpSnoBlock->GetEnableAni());
	//}

	int cvedId = pTrajFolObj->GetId();
	cved->SetObjOption( cvedId, cpSnoBlock->GetOption() );

	// 
	// Initialize mode values
	//
	CDdoParseBlock::EState initMode = cpSnoBlock->GetInitialState();

	switch ( initMode ) {
		case CDdoParseBlock::eNORMAL:
		{
			double initPosRot[6] = {0, 0, 0, 0, 0, 0}, initVel[6];
			CVector3D initLinearVel = cpSnoBlock->GetInitVelocityXYZ(),
				initAngularVel = cpSnoBlock->GetInitVelocityRPY();

			initVel[0] = initLinearVel.m_i * cMPH_TO_MS;
			initVel[1] = initLinearVel.m_j * cMPH_TO_MS;
			initVel[2] = initLinearVel.m_k * cMPH_TO_MS;
			initVel[3] = initAngularVel.m_i * M_PI / 180.0;
			initVel[4] = initAngularVel.m_j * M_PI / 180.0;
			initVel[5] = initAngularVel.m_k * M_PI / 180.0;

			pTrajFolObj->InitializeMode( eCV_GROUND_TRAJ );
			pTrajFolObj->SetInitPosRotVel( initPosRot, initVel );
			break;
		}
		case CDdoParseBlock::eFREE:
		{
			// need a better way to assign initial position, rotation, and 
			// a way to get initial velocities
			double initPosRot[6], initVel[6];
			CVector3D norm, 
				relPos = cpSnoBlock->GetRelativeLocation(),
				relRot = cpSnoBlock->GetRelativeOrientation(),
				initLinearVel = cpSnoBlock->GetInitVelocityXYZ(),
				initAngularVel = cpSnoBlock->GetInitVelocityRPY();

			initPosRot[0] = cActualPos.m_x + relPos.m_i;
			initPosRot[1] = cActualPos.m_y + relPos.m_j;
			cved->QryTerrain( initPosRot[0], initPosRot[1], cActualPos.m_z + relPos.m_k, 
				initPosRot[2], norm, NULL );
			initPosRot[2] += relPos.m_k;
			initPosRot[3] = relRot.m_i * M_PI / 180.0;
			initPosRot[4] = relRot.m_j * M_PI / 180.0;
			initPosRot[5] = atan2( cActualOri.m_j, cActualOri.m_i ) + relRot.m_k * M_PI / 180.0;
			initVel[0] = initLinearVel.m_i * cMPH_TO_MS;
			initVel[1] = initLinearVel.m_j * cMPH_TO_MS;
			initVel[2] = initLinearVel.m_k * cMPH_TO_MS;
			initVel[3] = initAngularVel.m_i * M_PI / 180.0;
			initVel[4] = initAngularVel.m_j * M_PI / 180.0;
			initVel[5] = initAngularVel.m_k * M_PI / 180.0;

			pTrajFolObj->InitializeMode( eCV_FREE_MOTION );
			pTrajFolObj->SetOffset( initPosRot ); // this to calculate the rotation matrix
			pTrajFolObj->SetInitPosRotVel( initPosRot, initVel );
			//printf("free object init posrot %.4f %.4f %.4f %.4f %.4f %.4f vel %.4f %.4f %.4f %.4f %.4f %.4f\n",
			//	initPosRot[0], initPosRot[1], initPosRot[2], initPosRot[3], initPosRot[4], initPosRot[5],
			//	initVel[0], initVel[1], initVel[2], initVel[3], initVel[4], initVel[5]); 

			break;
		}
		case CDdoParseBlock::eCOUPLED:
		case CDdoParseBlock::eRELATIVE:
		{
			double offset[6], initVel[6];
			CVector3D relPos = cpSnoBlock->GetRelativeLocation(),
				relRot = cpSnoBlock->GetRelativeOrientation(),
				initLinearVel = cpSnoBlock->GetInitVelocityXYZ(),
				initAngularVel = cpSnoBlock->GetInitVelocityRPY();


			offset[0] = relPos.m_i;
			offset[1] = relPos.m_j;
			offset[2] = relPos.m_k;
			offset[3] = relRot.m_i * M_PI / 180.0;
			offset[4] = relRot.m_j * M_PI / 180.0;
			offset[5] = relRot.m_k * M_PI / 180.0;
			initVel[0] = initLinearVel.m_i * cMPH_TO_MS;
			initVel[1] = initLinearVel.m_j * cMPH_TO_MS;
			initVel[2] = initLinearVel.m_k * cMPH_TO_MS;
			initVel[3] = initAngularVel.m_i * M_PI / 180.0;
			initVel[4] = initAngularVel.m_j * M_PI / 180.0;
			initVel[5] = initAngularVel.m_k * M_PI / 180.0;
			pTrajFolObj->InitializeMode( initMode==CDdoParseBlock::eRELATIVE?eCV_OBJ_REL_TRAJ:eCV_COUPLED_OBJ );
			// this is only used to set init velocities, in preparation for 
			// future transition to free motion mode, still need to call 
			// SetOffset() to not just set offset, but also to calculate 
			// rotation matrix
			pTrajFolObj->SetInitPosRotVel( offset, initVel ); 
			pTrajFolObj->SetOffset( offset );
			break;
		}
		default:
			break;
	}
	

}  // CreateCvedObject


////////////////////////////////////////////////////////////////////////
///\brief 
///		Updates the current position of the DDO
///
///\remark
///		Called by UserActivity() when state = eUNDER_CONTROL.
///		Sets the control inputs of the Traj Follower so that it continues
///		to follow the trajectory spline when not a dDDO.
///\par
///		When the DDO is dependant it uses the  equations of motion to 
///		try and track to arrive at the same time as to a target point
///		as the Own vehicle. The DDO and Own vehicle both have their own
///		target points. When useInitialVel is set true, the dDDO start with 
///		a fixed velocity and sets accelration to match the targets points
///		otherwise it uses a relatively constant velocity
///
///\note The equations of motion: http://en.wikipedia.org/wiki/Equations_of_Motion
////////////////////////////////////////////////////////////////////////
void 
CDdo::SetControlInputs( const CPoint3D& cCurrPos )
{
	const double cRAD_LEN = 1.0;

	double stepIncrement;
	//double velocity;
	CCubicSplinePos splinePos = m_lastSplinePos;
		bool segLocated = true;
	static int* pStart = NULL;
	int curtime;
	double delay = 0;
	int delayInd = -1;
	double oldSpeed = m_currentVel;
	if( !m_dependent || m_doOverideSpeed ) 
	{ 
		//
		// Normal trajectory followers.
		//
		int lastIndex;
		double lastT;
		m_lastSplinePos.getPos( lastIndex, lastT );

		if( m_velocityVector.size() < 2 )
		{
			// no path...velocity is 0
			m_currentVel = 0.0;
		}
		else
		{
			double D = (
				m_spline.getLength( lastIndex + 1 ) - 
				m_spline.getLength( lastIndex )
				);
//			gout << lastIndex << "  size=" << m_velocityVector.size() << "  D=" << D << endl;
			m_currentVel = (
				m_velocityVector[lastIndex] + 
				( m_velocityVector[lastIndex + 1] - m_velocityVector[lastIndex] ) * ( lastT / D )
				);
//			gout << "vel = " << velocity << endl;

			if( m_velocityVector[lastIndex] < cNEAR_ZERO ) 
			{
				delayInd = lastIndex;
				delay = m_delayVector[lastIndex];
			} 
			else if( m_velocityVector[lastIndex+1] < cNEAR_ZERO ) 
			{
				delayInd = lastIndex + 1;
				delay = m_delayVector[lastIndex + 1];
			}

			if( m_currentVel  < 0.05 && pStart == NULL ) 
			{
				pStart = new int();
				*pStart = GetFrame();
			} 
			else if( m_currentVel < 0.05 && pStart != NULL && delayInd != -1 ) 
			{
				curtime = GetFrame();
				if( ((curtime - (*pStart))*GetTimeStepDuration()) > delay ) 
				{
					delete pStart;
					pStart = NULL;
					m_currentVel = 1.0;
					m_velocityVector[delayInd] = m_currentVel;
				} 
				else 
				{
					m_currentVel = 0.0;
				}
			}
		}
		//this is probully not the best way to handle this
		//we may need to had something to interpolate between 
		//our over-ride speed and our path speed when we transition
		if (m_doOverideSpeed)
			m_currentVel = m_speedOveride;

		stepIncrement = m_currentVel * GetTimeStepDuration();
	}
	else 
	{ 
		//
		// Dependent trajectory followers.
		//

		int   lastIndex;
		double lastT;
		m_lastSplinePos.getPos( lastIndex, lastT );

		if( lastIndex < m_target )
		{
			double myDistToRefPoint = 0;
			CCubicSplinePos pos;
			segLocated = m_spline.locate( cCurrPos, 5.0, pos );
			if( segLocated ) 
			{
				int begin;
				double t;
				pos.getPos( begin, t );

				double distSoFarAlongSpline = 0;
				distSoFarAlongSpline = m_spline.getLength( begin );

				CPoint3D p0;
				m_spline.getPoint( begin, p0 );
				distSoFarAlongSpline += ( cCurrPos - p0 ).Length();
				myDistToRefPoint = m_spline.getLength( m_target ) - distSoFarAlongSpline;
			} 
			else 
			{
				myDistToRefPoint = ( cCurrPos - m_referencePoint ).Length();
			}
			int ownVehObjId;
			cved->GetObj( "ExternalDriver", ownVehObjId );
			if( ownVehObjId == -1 ) 
			{
				// if the ID is not valid
				cerr << MessagePrefix() << "Unable to locate ExternalDriver";
				cerr << " [SUICIDE]" << endl;

				Suicide();
				return;
#if 0
				if( myDistToRefPoint < cRAD_LEN ) 
				{
					DeleteHcsm( this );
					return;
				}
				stepIncrement = myDistToRefPoint;
#endif
			}
			else 
			{ 
				//
				// if the OwnVehicle object exists
				CPoint3D ownVehCurrentPos = cved->GetObjPos( ownVehObjId );
				double ownVehDistToRefPoint = 
					( ownVehCurrentPos - m_vehicleReferencePoint ).Length();



				if ( myDistToRefPoint > cRAD_LEN ) {
					const CDynObj* pOwnVehObj = cved->BindObjIdToClass( ownVehObjId );
					
					if( !pOwnVehObj->IsValid() ) {
						fprintf( stderr, "ERROR::object invalid\n");
					}
					double timetotarget;
					double ownvel = pOwnVehObj->GetVelImm() * cMETER_TO_FEET; 
					if (ownvel > 0.1){ //avoid divide by 0
						if (m_takeAccelIntoAccount){
							const CVehicleObj* cpVehObj = 
								dynamic_cast<const CVehicleObj *>( pOwnVehObj );
					
							double accel = cpVehObj->GetAccel()* cMETER_TO_FEET;

							double deltaSpeed = accel*GetTimeStepDuration();
							ownvel += deltaSpeed;
							//float accel = deltaSpeed/GetTimeStepDuration();
							if (fabs(accel) > 0.1){ //once accel falls out our error function is just to large to deal with
								double base = ownvel*ownvel + 2 * accel * ownVehDistToRefPoint;
								if (base > 0){ //we need to make sure they will reach the target point at their current accel/deccel
									double quadratic = -0.5 * (ownvel + sqrt(base));
									float root1 = quadratic/(0.5 * accel);
									float root2 =(0 - ownVehDistToRefPoint)/quadratic;
									if (root1 < root2 && root1 > 0)
										timetotarget = root1;
									else
										timetotarget = root2;
								}else{//we will assume the base = 0, and root2 is the solution
									timetotarget = 	ownVehDistToRefPoint/(0.5 * ownvel); 
								}
							}else{
								timetotarget = ownVehDistToRefPoint/ownvel;
							}
						}else{
							timetotarget = ownVehDistToRefPoint/ownvel;
						}	
					}else{ 
						timetotarget = 10000; //the participant is just not going to get there
					}

					//double timetotarget = ownVehDistToRefPoint/ownvel;
					// calculate the velocity and stepIncrement
					if (m_useInitVel){
						float myAccel = (2 * (myDistToRefPoint -  m_currentVel * timetotarget) )/(timetotarget*timetotarget);
						m_currentVel+= myAccel * GetTimeStepDuration();
					}
					else{
						m_currentVel = myDistToRefPoint / timetotarget;
						
					}
					stepIncrement = (m_currentVel) * GetTimeStepDuration();
					double mytimetotarget = myDistToRefPoint/m_currentVel;
#if 0
					cerr << "owndisttotarg = " << ownVehDistToRefPoint << endl <<
						    "ownvel        = " << ownvel << endl <<
							"owntimetoT    = " << timetotarget << endl <<
							"mydisttotarg  = " << myDistToRefPoint << endl <<
							"myvel         = " << velocity << endl <<
							"mytimetoT     = " << mytimetotarget << endl <<
							"target point  = " << m_target << endl;
#endif
					if( mytimetotarget < 0.5 )
					{
						m_dependent = 0;
					}
				}
				else 
				{
					if( lastIndex == m_spline.getCount()-2 && m_quitAtEnd ) 
					{
						DeleteHcsm( this );
						return;
					} 
					else if( lastIndex == m_spline.getCount()-2 )
					{
						stepIncrement = 0;
						m_currentVel = 0;
					} 
					else if( lastIndex < m_spline.getCount()-2 )
					{
						m_currentVel = m_velocityVector[lastIndex];
						stepIncrement = m_currentVel * GetTimeStepDuration();
					}
				}
			}
		} 
		else 
		{
			m_dependent = 0;
			m_currentVel = m_velocityVector[lastIndex];
			stepIncrement = m_currentVel * GetTimeStepDuration();
		}
	}  // else dependent

	//Now we need to check accel/decel limits
	if ( m_maxAccel_fps2 > 0 && m_currentVel - oldSpeed >  m_maxAccel_fps2 && !m_isFirstRun){
		m_currentVel  = oldSpeed + m_maxAccel_fps2;
		stepIncrement = m_currentVel * GetTimeStepDuration();
	}
	else if  ( m_maxDecel_fps2 < 0 && m_currentVel - oldSpeed  <  m_maxDecel_fps2){
		m_currentVel  = oldSpeed + m_maxDecel_fps2;
		stepIncrement = m_currentVel * GetTimeStepDuration();
	}

	//
	// If the spline cannot be successfully traversed to the
	// new position, then we've reached the end of the spline.
	//
	bool success = m_spline.traverse( splinePos, stepIncrement );
	if( !success )
	{
#if TF_DEBUG
		fprintf( 
			m_debugPointsFile, 
			"%s:\t%lf, %lf\n", 
			MessagePrefix().data(), 
			cCurrPos.m_x, 
			cCurrPos.m_y 
			); 
#endif

		if( m_quitAtEnd ) 
		{
			DeleteHcsm( this );
		}
		else 
		{
			m_pDdo->SetTargVel( 0 );
			m_pDdo->SetObjOri( m_lastOrientation );
			m_pDdo->SetTargDir( m_lastDirection );
		}

		return;
	}

	//
	// Calculate the new position and direction of the traj 
	// follower along the spline.
	//
	CPoint3D  newPosition;
	CVector2D newDirection;
	CVector2D newOrientation;
	CVector2D orientAtP0;
	CVector2D orientAtP1;
	double	  dummy;
	int		  index;
	double	  angleAtP0;
	double	  angleAtP1;
	double	  angleAtT;
	double	  chordLength;
	CVector2D error;
	double	  t;
	double	  tsq;
	CVector2D thirdDeriv;

	m_spline.eval( splinePos, newPosition );
	m_spline.evalTang(splinePos, 
		newDirection.m_i, newDirection.m_j, dummy);
	newDirection.Normalize();

	splinePos.getPos( index, t );

	tsq = t*t;

	angleAtP0 = m_orientVector[index];
	angleAtP1 = m_orientVector[index+1];
	chordLength = m_spline.getLength( index+1 ) - 
					m_spline.getLength( index );

	orientAtP0.m_i = cos( angleAtP0 );
	orientAtP0.m_j = sin( angleAtP0 );
	orientAtP1.m_i = cos( angleAtP1 );
	orientAtP1.m_j = sin( angleAtP1 );


	newOrientation.m_i = orientAtP0.m_i + (t/chordLength)*
		(orientAtP1.m_i - orientAtP0.m_i);
	newOrientation.m_j = orientAtP0.m_j + (t/chordLength)*
		(orientAtP1.m_j - orientAtP0.m_j);

	// Now, factor in the error due to linear interpolation.

	m_spline.evalTangTangTang( splinePos,
		thirdDeriv.m_i, thirdDeriv.m_j, dummy );


	error.m_i = thirdDeriv.m_i * 0.5 * ( tsq - chordLength * t );
	error.m_j = thirdDeriv.m_j * 0.5 * ( tsq - chordLength * t );

	CVector3D ori(newOrientation.m_i,newOrientation.m_j,0);
	CVector3D tan(newDirection.m_i,newDirection.m_j,0);
	ori.Normalize();
	tan.Normalize();

	// Determine if the orientation vector and the tangent
	// to the spline vector lie in the same half-plane.
	if( ori.DotP( tan ) < 0 ) {
		// then they lie in different half planes
		// so negate the error
		error.m_i *= -1;
		error.m_j *= -1;
	}
	// else the error is simply added and not subtracted.

	newOrientation.m_i += error.m_i;
	newOrientation.m_j += error.m_j;

	newOrientation.Normalize();

#if 0
	ori.m_i = newOrientation.m_i;
	ori.m_j = newOrientation.m_j;
	tan.m_i = newDirection.m_i;
	tan.m_j = newDirection.m_j;

	cerr << endl;
	cerr << "ind,d,t: " << index << "," << chordLength << "," << t << endl;
	cerr << "len err: " << error.Length() << endl;
	cerr << "Ori: " << newOrientation.m_i << "," << newOrientation.m_j << endl;
	cerr << "Tan: " << newDirection.m_i << "," << newDirection.m_j << endl;
	cerr << "Angle Diff: " << acos( ori.DotP( tan ) )/0.0174533 << endl;
#endif
//	cerr << endl;
//	cerr << "POS: " << cCurrPos.m_x << "," << cCurrPos.m_y << endl;
//	cerr << "Ori: " << newOrientation.m_i << "," << newOrientation.m_j << endl;

	// Now, set the velocity, orientation and the tangent
	// for the control inputs.
 	m_pDdo->SetTargVel( m_currentVel );
	m_pDdo->SetObjOri( newOrientation );
	m_pDdo->SetTargDir( newDirection );										

#if TF_DEBUG
	fprintf( 
		m_debugPointsFile, 
		"%s\t%lf, %lf\t", 
		MessagePrefix().data(), 
		cCurrPos.m_x, 
		cCurrPos.m_y 
		); 
	fprintf( 
		m_debugPointsFile, 
		"%lf, %lf\t", 
		newPosition.m_x, 
		newPosition.m_y 
		); 
	fprintf( 
		m_debugPointsFile, 
		"%lf, %lf\t", 
		newDirection.m_i, 
		newDirection.m_j
		); 
	fprintf( m_debugPointsFile, "%lf\n", velocity ); 
#endif

	m_lastSplinePos = splinePos;
	m_last3DPos = newPosition;
	m_lastOrientation = newOrientation;
	m_lastDirection = newDirection;
	m_isFirstRun = false;
} // SetControlInputs
