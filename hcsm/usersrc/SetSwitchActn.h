/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2015 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: SetSwitchActn.h,v 1.1 2015/09/17 16:18:47 IOWA\dheitbri Exp $
//
// Author(s):  David Heitbrink
//
// Date:        September, 2004
//
// Description: The definition of CSetSwitchActn, a subclass of CAction.  
// 	This sets a switch option on a dynamic object.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#pragma once

#include "hcsmspec.h"
#include "action.h"

#include <string>
using namespace std;

class CHcsmCollection;
/////////////////////////////////////////////////////////////////////////////
///\brief
///  Set a uniform for the shader of the target objects
///\remark
///  A uniform is a variable that is externaly set for a shader, this action
/// works in concert with the Image Generator to change the visual apperence
/// of an object
/////////////////////////////////////////////////////////////////////////////
class CSetSwitchActn : public CAction 
{
public:
	CSetSwitchActn( const CActionParseBlock*, CHcsmCollection* );
	CSetSwitchActn( const CSetSwitchActn& );
	CSetSwitchActn& operator=( const CSetSwitchActn& );
	~CSetSwitchActn();

	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return m_sName; };
    inline static const char* Name() { return m_sName; };
protected:
	CHcsmCollection*	m_pHC;
	string              m_switchName;
	int                 m_value;
	const static char   m_sName[];
};

