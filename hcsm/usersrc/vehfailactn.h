/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: vehfailactn.h,v 1.5 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CVehFailActn, a subclass of CAction.  
// 	This class causes a particular type of failure on the own-vehicle.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _VEH_FAIL_ACTN_H_
#define _VEH_FAIL_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CVehFailActn : public CAction 
{
public:
	// Constructor
	CVehFailActn( const CActionParseBlock*, CHcsmCollection* );

	// Copy constructors
	CVehFailActn( const CVehFailActn& );

	// Destructor
	~CVehFailActn();

	// Assignment operator
	CVehFailActn& operator=( const CVehFailActn& );

	// Execute operation
	void Execute( const set<CCandidate>* instigators = 0 );

	inline const char* GetName() const { return "VehicleFailure"; };

protected:
	CHcsmCollection*	m_pHC;
	string				m_failure;
};

#endif	// _VEH_FAIL_ACTN_H_
