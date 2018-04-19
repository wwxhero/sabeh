/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: createhcsmactn.h,v 1.9 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        June, 1999
//
// Description: The definition of CCreateHcsmActn, a subclass of CAction.  
//				This class uses its CActionParseBlock to get the parser 
//				information for which HCSMs to create.
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _CREATE_HCSM_ACTN_H_
#define _CREATE_HCSM_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CCreateHcsmActn : public CAction 
{
	public:
		// Constructor
		CCreateHcsmActn( const CActionParseBlock*, CHcsmCollection* );

		// Copy constructors
		CCreateHcsmActn( const CCreateHcsmActn& );

		// Destructor
		~CCreateHcsmActn();

		// Assignment operator
		CCreateHcsmActn &operator=( const CCreateHcsmActn& );

		// Execute operation
		void Execute( const set<CCandidate>* cpObjs = 0 );

		inline const char* GetName() const { return "CreateHcsm"; };

	protected:
		const CActionParseBlock*	m_pActionBlock;
		CHcsmCollection*			m_pHC;
};

#endif	// _CREATE_HCSM_ACTN_H_

