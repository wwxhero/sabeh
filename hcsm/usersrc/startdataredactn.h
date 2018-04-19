/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: startdataredactn.h,v 1.4 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Matt Schikore
//
// Date:        Dec 2003
//
// Description: The definition of CStartDataredActn, a subclass of CAction.  
// 	This class causes a data reduction segment to begin.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _START_DATARED_ACTN_H_
#define _START_DATARED_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CStartDataredActn : public CAction 
{
public:
	// Constructor
	CStartDataredActn( const CActionParseBlock*, CHcsmCollection* );

	// CStartDataredActn constructors
	CStartDataredActn( const CStartDataredActn& );

	// Destructor
	~CStartDataredActn();

	// Assignment operator
	CStartDataredActn& operator=( const CStartDataredActn& );

	// Execute operation
	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return "StartDataReduc"; };

protected:
	CHcsmCollection*	m_pHC;
	string				m_data;
};

#endif	// _START_DATARED_ACTN_H_

