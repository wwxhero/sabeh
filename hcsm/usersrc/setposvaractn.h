/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setposvaractn.h,v 1.3 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   David Heitbrink
//
// Date:        Dec 2003
//
// Description: The definition of CSetVarActn, a subclass of CAction.  
// 	This class causes an internal variable to be set to a specific value.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SET_VAR_ACTN_H_
#define _SET_VAR_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CSetPosVarActn : public CAction 
{
public:
	// Constructor
	CSetPosVarActn( const CActionParseBlock*, CHcsmCollection* );

	// CSetVarActn constructors
	CSetPosVarActn( const CSetPosVarActn& );

	// Destructor
	~CSetPosVarActn();

	// Assignment operator
	CSetPosVarActn& operator=( const CSetPosVarActn& );

	// Execute operation
	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return "SetPosVar"; };

protected:
	CHcsmCollection*	m_pHC;
	string				m_varName;
	string				m_varValue;
};

#endif	// _SET_VAR_ACTN_H_
