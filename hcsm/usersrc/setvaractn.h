/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setvaractn.h,v 1.3 2010/01/14 17:29:30 iowa\dheitbri Exp $
//
// Author(s):   Matt Schikore
//
// Date:        Dec 2003
//
// Description: The definition of CSetVarActn, a subclass of CAction.  
// 	This class causes an internal variable to be set to a specific value.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SET_POSVAR_ACTN_H_
#define _SET_POSVAR_ACTN_H_

#include "hcsmspec.h"
#include "action.h"
#include "expeval.h"

class CHcsmCollection;

class CSetVarActn : public CAction 
{
public:
	// Constructor
	CSetVarActn( const CActionParseBlock*, CHcsmCollection* );

	// CSetVarActn constructors
	CSetVarActn( const CSetVarActn& );

	// Destructor
	~CSetVarActn();

	// Assignment operator
	CSetVarActn& operator=( const CSetVarActn& );

	// Execute operation
	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return "SetVar"; };

protected:
	CHcsmCollection*	m_pHC;
	bool				m_isExpr;
	string				m_varName;
	string				m_varValue;
	CExpEval			m_expr;
	
};

#endif	// _SET_POSVAR_ACTN_H_

