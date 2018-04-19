/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: action.h,v 1.41 2016/05/05 22:13:33 IOWA\dheitbri Exp $
// Author(s):   Jillian Vogel
// Date:        June, 1999
//
// Description: The definition of the abstract class CAction.  This class
//  provides an interface to the various actions that may be executed 
//  when a Trigger fires. It also includes the definitions of the classes 
//  which subclass CAction.  This is an abstract class and cannot be 
//  instantiated.
//
/////////////////////////////////////////////////////////////////////////////


#ifndef _ACTION_H_
#define _ACTION_H_

class CHcsmCollection;
#include <hcsmspec.h>
#include "candidateset.h"

#define	ACTN_DEBUG	0

/////////////////////////////////////////////////////////////////////////////
//
// The abstract class that is executed when a Trigger fires.
//
/////////////////////////////////////////////////////////////////////////////
class CAction 
{
public:
	CAction();
	CAction( const CAction& cRhs );
	CAction& operator=( const CAction& cRhs );
	~CAction();
	typedef shared_ptr<CAction> TActionPtr;
	typedef vector< TActionPtr >::const_iterator TActionIterator;
	typedef vector<TActionPtr> TActionVec;
	// Abstract execute operation
	virtual void Execute( const set<CCandidate>* cObjs = 0 ) = 0;
	inline virtual bool IsFinished() { return true; };
	inline virtual const char* GetName() const = 0;
	double GetDelay() const;
	int GetTriggerId() { return m_triggerId; }
	void SetTriggerId(int triggerId) { m_triggerId = triggerId; }

	CCandidateSet	 m_candidateSet;
	bool			 m_useInstigators;

protected:
	void InitCandidateSet( const CActionParseBlock* cpBlock );
	static int		 m_sDataRedCounter;
	int              m_triggerId;
	double           m_delay;
};

// Function called to fill a vector of CAction* with the proper action types.
void GetActions(
			const CTriggerParseBlock* cpBlock, 
			CHcsmCollection* pHC, 
			CAction::TActionVec &actions,
			bool debug = false
			);

/////////////////////////////////////////////////////////////////////////////
//
// Concrete subclasses of CAction
//
/////////////////////////////////////////////////////////////////////////////
#include "createhcsmactn.h"
#include "deletehcsmactn.h"
#include "setbuttonactn.h"
#include "setdialactn.h"
#include "resetdialactn.h"
#include "playaudioactn.h"
#include "transitiontrffclghtactn.h"
#include "logdataactn.h"
#include "trmntsmltnactn.h"
#include "mtnbaseprpstnactn.h"
#include "mtnbasetuneactn.h"
#include "vehfailactn.h"
#include "phonecallactn.h"
#include "usetrafmansetactn.h"
#include "startdataredactn.h"
#include "stopdataredactn.h"
#include "setvaractn.h"
#include "setposvaractn.h"
#include "writecellactn.h"
#include "TurnGraphOnActn.h"
#include "turngraphoffact.h"
#include "visualDisplayTextAction.h"
#include "LoadVar.h"
#include "StoreVar.h"
#include "ChangeCabSetting.h"
#include "VarQueueOperations.h"
#include "SetHeadlights.h"
#include "CreateRandomGen.h"
#include "SetUniform.h"
#include "AttachShaderActn.h"
#include "SetSwitchActn.h"
#include "UpdateTODActn.h"
#endif	// _ACTION_H_

