/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2016 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: UpdateTODActn.h,v 1.2 2016/05/11 20:34:38 IOWA\dheitbri Exp $
//
// Author(s):   David Heitbrink
//
// Date:        May 2016
//
// Description: The definition of CUpdateTodActn, a subclass of CAction.  
// 	This class causes writes a value to a cell on the scramnet.
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
///Update the Time of Day  
///\remark
/// This actions changes the time of day. This will change the lighting 
/// conditions in the scene.
/////////////////////////////////////////////////////////////////////////////
class CUpdateTodActn : public CAction 
{
public:
	CUpdateTodActn( const CActionParseBlock*, CHcsmCollection* );
	CUpdateTodActn( const CUpdateTodActn& );
	CUpdateTodActn& operator=( const CUpdateTodActn& );
	~CUpdateTodActn();

	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return m_sName; };
    inline static const char* Name() { return m_sName; };
protected:
	CHcsmCollection*	m_pHC;
	string              m_setTimeCommand;
	const static char   m_sName[];
};

