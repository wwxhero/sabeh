/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setbuttonactn.h,v 1.6 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description:  Definition of the CSetButtonActn class.  This action sets
// 	the given button of its candidates.
//
// NOTE: This file should not be directly included. Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SET_BUTTON_ACTN_H_
#define _SET_BUTTON_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CSetButtonActn : public CAction 
{
public:
	// Constructor
	CSetButtonActn( const CActionParseBlock*, CHcsmCollection* );

	// Copy constructors
	CSetButtonActn( const CSetButtonActn& );

	// Destructor
	~CSetButtonActn();

	// Assignment operator
	CSetButtonActn& operator=( const CSetButtonActn& );

	// Execute operation
	void Execute( const set<CCandidate>* instigators = 0 );

	inline const char* GetName() const { return "SetButton"; };

protected:
	CHcsmCollection*	m_pHC;
	string				m_buttonName;
};

#endif	// _SET_BUTTON_ACTN_H_

