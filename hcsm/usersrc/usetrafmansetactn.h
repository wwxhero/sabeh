/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: usetrafmansetactn.h,v 1.5 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Matt Schikore
//
// Date:        January 2001
//
// Description: The definition of CUseTrafManSetActn, a subclass of CAction.  
//	This class causes the active set in the traffic manager to be changed.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _USETRAFMANSETACTN_H_
#define _USETRAFMANSETACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CUseTrafManSetActn : public CAction 
{
public:
	// Constructor
	CUseTrafManSetActn( const CActionParseBlock*, CHcsmCollection* );

	// Copy constructors
	CUseTrafManSetActn( const CUseTrafManSetActn& );

	// Destructor
	~CUseTrafManSetActn();

	// Assignment operator
	CUseTrafManSetActn& operator=( const CUseTrafManSetActn& );

	// Execute operation
	void Execute( const set<CCandidate>* instigators = 0 );

	inline const char* GetName() const { return "UseTmSet"; };

protected:
	CHcsmCollection*	m_pHC;
	string				m_setName;
};

#endif	// _USETRAFMANSETACTN_H_
