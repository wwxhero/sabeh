/*****************************************************************************
 *
 * (C) Copyright 2011 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: VisualObject.cxx,v 1.11 2016/01/28 23:36:56 IOWA\dheitbri Exp $
 *
 * Author(s):    David A Heitbrink
 * Date:         January, 2011
 *
 * Description:  Visual Object, this class is for objects that have a visual
 *               representation, but not may not exist in the data base, such
 *				 as overlay objects, or icons.
 *
 ***************************************************************************/
#define _USE_MATH_DEFINES

#include "genhcsm.h"
#include "hcsmcollection.h"
#include "util.h"
////////////////////////////////////////////////////////////////////////////////////////
///\class CVirtualObject
///\brief	Implemntation of Virtual Object
///\remark
///		The Virtual Object represent an object that has a visual representation
///		In the scene but no physical representation. This is ment to support
///		Augmented relatity, HUDS, Divided attention task and other tasks.
///
/////////////////////////////////////////////////////////////////////////////////////////
void CVirtualObject::UserCreation( 
			const CVirtualObjectParseBlock* pSnoBlock
			)
{
	
	//
	// Copy the name from SCN file if it has one.
	//
	if( pSnoBlock->GetName().size() > 0 )  m_name = pSnoBlock->GetName();

	//m_pRootCollection->MemLog( 0, HLOG_ADO_USRCREATE_IN, m_name.c_str());
	m_lastState = eWAIT;
	m_isBlinking = false;
	m_isMoving = false;
	m_isRotating = false;
	m_stateIndex =1;
    m_drawScreen = (int)pSnoBlock->GetScreen();
	m_initConditions.SetObjectInitCond(
				*cved, 
				GetFrame(), 
				GetTimeStepDuration(),  
				pSnoBlock->GetCrRad(),  
				pSnoBlock->GetActvDel(),
				pSnoBlock->GetLifetime()
				);
	// print creation message with template name and hcsm id
	PrintCreationMessage();

}

void CVirtualObject::HandleDials(){

	if (m_dialSetPosition.HasValue()){
		/*
			This sets the position the Virtual Object appears in the visual space, not where it located in CVED
		*/
		stringstream sstemp;
		stringstream converter;
		char tempBuff[20];
		memset(tempBuff,0,sizeof(tempBuff));
		m_OverLayPosition.m_x = 0;
		m_OverLayPosition.m_y = 0;
		m_OverLayPosition.m_z = 0;

		sstemp.str( m_dialSetPosition.GetValue());
		m_OverLayPosition.m_x;
		for (unsigned int i = 0; i < 3; i++){
			sstemp.getline(tempBuff,20,';');
			converter<<tempBuff<<" ";
			if (sstemp.fail()){
				break;
			}
		}
		converter>>m_last3DPos.m_x>>m_last3DPos.m_y>>m_last3DPos.m_z;
		m_dialSetPosition.SetNoValue();
	}

	if (m_dialSetAnimation.HasValue()){
		/*
			Sets an animation string
		*/
		m_isBlinking = false;
		m_isMoving = false;
		m_isRotating = false;

		m_isAnimationOn = true;
		m_startAnimationFrame  = GetFrame();

		stringstream sstemp;
		stringstream converter;
		string tag;

		char tempBuff[128];
		memset(tempBuff,0,sizeof(tempBuff));

		sstemp.str(m_dialSetAnimation.GetValueStr());
		//parse the thing
		while (sstemp.good()){
			sstemp.getline(tempBuff,128,';');
			converter.clear();
			converter.str(tempBuff);
			converter>>tag;
			float x =0 , y = 0, z = 0, rate = 0, bon = 0, boff = 0 ;
			if (tag == "blink"){
				m_isBlinking = true;
				converter>>bon>>boff;
				m_periodFrames = (int)(bon/GetTimeStepDuration() + boff/GetTimeStepDuration()) ;
				m_flashOnFrames = (int)(bon/GetTimeStepDuration()) ;
			}else if (tag == "rotate"){
				m_isRotating = true;
				converter>>m_degreesPerSec;
			}else if (tag == "move"){
				m_isMoving = true;
				converter>>x>>y>>z>>rate;
				m_targetPos.m_x = x;
				m_targetPos.m_y = y;
				m_targetPos.m_z = z;
				m_unitsPerSecMov = rate;

			}
		}
/*
	PRIV_DECL		bool m_isBlinking;
	PRIV_DECL		bool m_isMoving;
	PRIV_DECL		bool m_isRotating;
	
	PRIV_DECL		float m_rotateRate;
	PRIV_DECL		float m_degreesPerSec;
	PRIV_DECL		CPoint3D m_targetPos;
	PRIV_DECL		float m_unitsPerSecMov;
*/

		//fill in what we want here:
		m_dialSetAnimation.SetNoValue();	
	}
	if (m_dialSetRotation.HasValue()){
		/*
			Sets Rotation in Degrees
		*/
		float rot = m_dialSetRotation.GetValue();
		m_rotation.m_x = rot;
		m_dialSetRotation.SetNoValue();
	}

	if (m_dialSetStateIndex.HasValue()){
		string val = m_dialSetStateIndex.GetValue();
		if (val.size() > 0 && isalpha(val[0])){
			m_stateIndex = (int)CHcsmCollection::GetExprVariable(val);
		}else{
			m_stateIndex = 1;
			stringstream converter;
			converter<<val;
			converter>>m_stateIndex;
		}
		//m_pVisualObject->SetStateIndex(m_dialSetStateIndex.GetValue());
		m_dialSetStateIndex.SetNoValue();
	}
	if (GetButtonTurnOnAnimation()){
			m_isAnimationOn = true;
			m_startAnimationFrame = GetFrame();
			
	}
	if (GetButtonTurnOffAnimation()){
			m_isAnimationOn = false;
	}
	
}
/////////////////////////////////////////////////////////////////////////////
//
//
void CVirtualObject::UserActivity( 
			const CVirtualObjectParseBlock* pSnoBlock 
			)
{
	// Process any dial inputs
	HandleDials();
	int        code;
	double frameTimeS = GetTimeStepDuration();
	// Get current gmtrcpstn and position.
	int actualTime = GetFrame();

	CPoint3D actualPosition;
	if( m_lastState == eUNDER_CONTROL )
	{
		actualPosition = m_pVisualObject->GetPosImm();
	}
	else
	{
		actualPosition = m_last3DPos;
	}

	// Get the current state from the InitCondition object
	EInitCond state = m_initConditions.Execute(
										pSnoBlock->GetPosition(), 
										actualTime
										);
			
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

				m_pVisualObject = dynamic_cast<CVisualObjectObj *> (
								cved->CreateDynObj(
											pSnoBlock->GetName(),
											eCV_VIRTUAL_OBJECT, 
											m_typeId,
											triggerAttr,
											&actualPosition,
											0, 
											0
											)
								);
				if( !m_pVisualObject )
				{
					cerr << MessagePrefix() << pSnoBlock->GetName();
					cerr << " Unable to create self in CVED... [SUICIDE]" << endl;

					Suicide();
					return;
				}
				float r,g,b,a;
				CPoint3D pnt;
				pSnoBlock->GetBoarderColor(r,g,b,a);
				m_initBoarderColor[0] = r;
				m_initBoarderColor[1] = g;
				m_initBoarderColor[2] = b;
				m_initBoarderColor[3] = a;

				SetBoarderColor(r,g,b,a);

				pSnoBlock->GetColor(r,g,b,a);
				m_initFillColor[0] = r;
				m_initFillColor[1] = g;
				m_initFillColor[2] = b;
				m_initFillColor[3] = a;
				SetColor(r,g,b,a);

				m_pVisualObject->SetStateIndex(0);
				m_stateIndex = 0;
				
				m_last3DPos = pSnoBlock->GetDrawPosition();
				pnt = m_last3DPos;
				SetDrawPosition(pnt.m_x,pnt.m_y, pnt.m_z);
				m_rotation.m_x = pSnoBlock->GetOrientation();
				m_pVisualObject->SetRotation(m_rotation.m_x);
				

				m_size = pSnoBlock->GetDrawScale();
				m_pVisualObject->SetDrawSize(m_size.m_x,m_size.m_y);

				string tempS;
				tempS = pSnoBlock->GetTargetName();
                int objID = 0; 
                if (tempS != "" && cved->GetObj(tempS,objID)){
                    m_pVisualObject->SetTargetId(objID);
                }
				m_isAnimationOn = pSnoBlock->GetIsBlinking();
				if (m_isAnimationOn){
					m_startAnimationFrame = GetFrame();
				}
				m_periodFrames = (int)(pSnoBlock->GetBlinkOnDurr()/GetTimeStepDuration() + pSnoBlock->GetBlinkOffDurr()/GetTimeStepDuration() );
				m_flashOnFrames = (int)(pSnoBlock->GetBlinkOnDurr()/GetTimeStepDuration() );
				m_drawType = pSnoBlock->GetType();
				m_pVisualObject->SetDrawType( m_drawType,false );
				m_pVisualObject->SetDrawSize(m_size.m_x,m_size.m_y,false);
				m_pVisualObject->SetStateIndex(m_stateIndex,false);
				m_pVisualObject->SetDrawType( m_drawType,false );
				m_pVisualObject->SetRotation(m_rotation.m_x,false);
				m_pVisualObject->SetDrawPosition(m_last3DPos,false);
				m_pVisualObject->SetDrawScreen(m_drawScreen,false);
			}

			break;	// case eACTIVATE


		//////////////////////////////////////////////////////////////////////
		// eUNDER_CONTROL
		//
		case eUNDER_CONTROL:
			{
				if (m_isAnimationOn){
					int currFrame = GetFrame() - m_startAnimationFrame;
						int periodTime = currFrame % m_periodFrames;
						if (m_isBlinking){
							if (periodTime < m_flashOnFrames){
								SetBoarderColor(m_initBoarderColor[0],m_initBoarderColor[1],m_initBoarderColor[2],m_initBoarderColor[3]);
								SetColor(m_initFillColor[0],m_initFillColor[1],m_initFillColor[2],m_initFillColor[3]);
							}else{
								SetColor(m_initFillColor[0],m_initFillColor[1],m_initFillColor[2],0);
								SetBoarderColor(m_initBoarderColor[0],m_initBoarderColor[1],m_initBoarderColor[2],0);
							}
						}
						if (m_isRotating){
							m_rotation.m_x += frameTimeS * m_degreesPerSec;
						}
						if (m_isMoving){

							//first lets calcuate this distance we want to travel
							CPoint3D pos = m_last3DPos;
							//now lets create a unit vector
							//since the scenario can change the position at
							//any time we want to recalculate this every frame
							CVector3D vec;
							vec.m_i =m_targetPos.m_x -  pos.m_x;
							vec.m_j =m_targetPos.m_y  - pos.m_y;
							vec.m_k =m_targetPos.m_z  - pos.m_z;
								

							float dist = vec.Length();
							if (dist > m_unitsPerSecMov * frameTimeS){
								//if we cannot get there in 1 frame
								dist =  m_unitsPerSecMov * frameTimeS;
							}
							//now lets nomralize it
							vec.Normalize();
							pos.m_x += vec.m_i * dist;
							pos.m_y += vec.m_j * dist; 
							pos.m_z += vec.m_k * dist;
							SetDrawPosition(pos.m_x,pos.m_y,pos.m_z);
						}
				}
				m_pVisualObject->SetRotation(m_rotation.m_x);
				m_pVisualObject->SetDrawPosition(m_last3DPos);
				m_pVisualObject->SetDrawSize(m_size.m_x,m_size.m_y);
				m_pVisualObject->SetStateIndex(m_stateIndex);
				m_pVisualObject->SetDrawType( m_drawType );
				m_pVisualObject->SetDrawScreen(m_drawScreen);

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


}
void CVirtualObject::SetBoarderColor(float r, float g, float b, float a){
	m_pVisualObject->SetBoarderColor(r,g,b,a);
}
void CVirtualObject::SetColor(float r, float g, float b, float a){
	m_pVisualObject->SetColor(r,g,b,a);
}
void CVirtualObject::SetDrawPosition(float x, float y, float z){
	CPoint3D pos(x,y,z);
	m_last3DPos.m_x = x;
	m_last3DPos.m_y = y;
	m_last3DPos.m_z = z;

}
void CVirtualObject::UserDeletion( 
			const CVirtualObjectParseBlock* pSnoBlock
			)
{
	PrintDeletionMessage();
}

void CVirtualObject::UserPostActivity( const CVirtualObjectParseBlock* ){
	//if( m_initConditions. == eWAIT )  return;
}