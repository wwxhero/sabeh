/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: mtnbasetuneactn.h,v 1.6 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CMtnBaseTuneActn, a subclass of CAction.  
//	This class prepositions the motion base.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _MTN_BASE_TUNE_ACTN_H_
#define _MTN_BASE_TUNE_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CMtnBaseTuneActn : public CAction 
{
public:
	// Constructor
	CMtnBaseTuneActn( const CActionParseBlock*, CHcsmCollection* );

	// Copy constructors
	CMtnBaseTuneActn( const CMtnBaseTuneActn& );

	// Destructor
	~CMtnBaseTuneActn();

	// Assignment operator
	CMtnBaseTuneActn& operator=( const CMtnBaseTuneActn& );

	// Execute operation
	void Execute( const set<CCandidate>* instigators = 0 );

	inline const char* GetName() const { return "MotionBaseTune"; };

protected:
};

#endif	// _MTN_BASE_TUNE_ACTN_H_

