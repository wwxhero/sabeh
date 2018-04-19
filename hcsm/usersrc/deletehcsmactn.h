/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: deletehcsmactn.h,v 1.8 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CDeleteHcsmActn, a subclass of CAction.  
// 	This class deletes the indicated HCSMs from the simulation.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _DELETE_HCSM_ACTN_H_
#define _DELETE_HCSM_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CDeleteHcsmActn : public CAction
{

	public:
		// Constructor
		CDeleteHcsmActn( const CActionParseBlock*, CHcsmCollection* );

		// Copy constructors
		CDeleteHcsmActn( const CDeleteHcsmActn& );

		// Destructor
		~CDeleteHcsmActn();

		// Assignment operator
		CDeleteHcsmActn& operator=( const CDeleteHcsmActn& );

		// Execute operation
		void Execute( const set<CCandidate>* instigators = 0 );

		inline const char* GetName() const { return "DeleteHcsm"; };

	protected:
		CHcsmCollection*	m_pHC;
};

#endif	// _DELETE_HCSM_ACTN_H_
