/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: visualDisplayTextAction.h,v 1.2 2013/07/29 17:53:04 IOWA\vhorosewski Exp $
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

#ifndef _SET_DISPLAY_TEXT_VAR_ACTN_H_
#define _SET_DISPLAY_TEXT_VAR_ACTN_H_

#include "hcsmspec.h"
#include "action.h"
#include "expeval.h"

class CHcsmCollection;

class CSetDisplayText : public CAction 
{
public:
	// Constructor
	CSetDisplayText( const CActionParseBlock*, CHcsmCollection* );

	// CSetVarActn constructors
	CSetDisplayText( const CSetDisplayText& );

	// Destructor
	~CSetDisplayText();

	// Assignment operator
	CSetDisplayText& operator=( const CSetDisplayText& );

	// Execute operation
	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return "SetVisualDisplayText"; };

protected:
	CHcsmCollection*	m_pHC;
	bool				m_isExpr;
	string				m_varName;
	string				m_varValue;
	int					m_location;
	CExpEval			m_expr;

private:
	CSetDisplayText();
};

#endif	// _SET_DISPLAY_TEXT_VAR_ACTN_H_

