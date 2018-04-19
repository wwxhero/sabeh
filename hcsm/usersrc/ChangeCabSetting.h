#pragma once
/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2013 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: 
// Author(s):   David Heitbrink
// Date:        March, 2013
//
// Description:  Definition of the CChangeCabSetting class.  This sets
//  a switch for the virtual cab a MiniSim only action
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////
#include "action.h"
class CChangeCabSetting :
	public CAction
{
public:
	CChangeCabSetting(const CActionParseBlock*, CHcsmCollection* );
	CChangeCabSetting(const CChangeCabSetting& );
	~CChangeCabSetting(void);

	CChangeCabSetting &operator=( const CChangeCabSetting& );

	void Execute( const set<CCandidate>* pInstigators = 0 );
	inline const char* GetName() const { return "ChangeCabSetting"; };

protected:
	CHcsmCollection* m_pHC;
	string           m_dialName;
	string           m_cabSettingString;
};

