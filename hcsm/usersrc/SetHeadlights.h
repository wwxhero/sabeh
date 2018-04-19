/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2014 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: SetHeadlights.h,v 1.3 2014/05/13 16:33:43 iowa\dheitbri Exp $
//
// Author(s):   David Heibrink
//
// Date:        Spring 2014
//
// Description: The definition of CSetHeadlights, a subclass of CAction.  
// 	This class changes the state of the ownships headlight
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////
#pragma once
#include "action.h"
/////////////////////////////////////////////////////////////////////////////
///\brief
///		This class loads variables from a text file	
///\remark
///		action is for setting a state on the headlights. At this point
///		the functionality of this action is subject to change, and is
///		not finalized.
///\todo
///		This action right now just turns off and on headlights, it really 
///		needs fleshed out, it needs to register itself with the activity log
///		and print errors to gout. The mechanism for changing headlight states
///		is not ideal either. 
/////////////////////////////////////////////////////////////////////////////
class CSetHeadlights :
	public CAction
{
public:
	CSetHeadlights( const CActionParseBlock* pBlock, CHcsmCollection* pColl);
	~CSetHeadlights(void);
	CSetHeadlights& operator=( const CSetHeadlights& cRhs );
	CSetHeadlights(const CSetHeadlights&);
	void Execute( const set<CCandidate>* cObjs = 0 ) override;
	inline const char* GetName() const { return "SetHeadlight";};
private:
	int m_headlightAction; //<turn on/off action;
};
