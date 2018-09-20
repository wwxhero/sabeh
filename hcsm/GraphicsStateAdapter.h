#pragma once
/*****************************************************************************
 *
 * (C) Copyright 2015 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: GraphicsStateAdapter.h,v 1.7 2017/05/03 16:58:48 IOWA\dheitbri Exp $
 *
 * Author(s):    D.A. Heitbrink
 *
 * Date:		 March, 2015
 *
 * Description:  Addapter to convert from HCSM state to Graphics State.
 *
 ****************************************************************************/
class CRendererIface; 
class CScenarioControl;
#include <map>
#include <vector>
////////////////////////////////////////////////////////////////////////////
///\brief
///	Utility takes scenario control state, loads it to RenderInterface
///\remark
///		This class maybe more follows a bridge pattern than a adapter pattern. 
///  The basic point is to take state data from scenario control, then to 
///  send said state information to the RenderInterface.
///\remark
///		RenderInterface, is designed to provide a double buffering sceme
///   for loading state data into a IG. It does not dirrectly talk to a 
///   IG, or graphics engine. A few different implemenations of RenderInterface
///   exist. This class is designed to only deal with the loading the 
///   RenderInterface with state data from scenario control. This type of 
///   activitey is done in a few places, and this is meant to be a generic 
///   implementation of the this
///\note
///	  This class is not yet fully implemented and functions that may do the 
///   same thing may exist in other places in the code, this class is designed
///   to replace alot of bad cut and paste code. IG specif command still may
///   need to be used bypassing this interface altogher. This class is not
///   meant to handle all interactions between scenario control and a visualizer.
///  
//////////////////////////////////////////////////////////////////////////////
class CGraphicsStateAdapter{
public:
	CGraphicsStateAdapter(CRendererIface *render,CScenarioControl* scenario );
	bool InitStaticObjects();
	bool AddVisualCommands();
	bool UpdateDynamicObjects();
	bool UpdateHeadlights();
    bool InitVirtualObjects();
    bool SetEnv();
    bool AddPreCreates();
	int TrafLightSwitchStateFromTrafficLightState(int) const;
	static const int EMPTY_OBJ_TYPE = 0;
	static const int EMPTY_OBJ_ID = 9999;


private:
	CRendererIface *m_pRender;
	CScenarioControl* m_pScen;
	void SetMovingObjectStates(
			int cvedId,
			int option,
			int uniqueId,
			double posori[6] = NULL,
			CPoint3D* eyepos = NULL,
			cvTObjState* state = NULL
			);
	void DoDiGuyUpdate(
			int cvedId,
			int option,
			int uniqueId,
			double posori[6] = NULL,
			CPoint3D* eyepos = NULL,
			cvTObjState* state = NULL	
			);
    void CreateObject(int cvedId);
    void CreateDiGuy(int cvedId, int solId);
	bool GetCvedObjCigiInfo( CCved& cved, int cvedId, int& uniqueId, int& modelType );
    int VehicleSwitchStateFromVisualMask(int  switchNo, int  visualStateMask,int modelType = -1);
	//these objects should be removed, this class should be stateless,
	//but so far this class removes alot of bad bad code, its really bad
	std::map<int,double> m_tireYaw;
	std::map<int,double> m_bodyYaw;
	bool m_headLightsOn;
	void SetLRIObjectStates(int cvedId,int option);
	cvTObjState		m_state[300];
};