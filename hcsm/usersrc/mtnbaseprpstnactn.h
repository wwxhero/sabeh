/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: mtnbaseprpstnactn.h,v 1.8 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
// Author(s):   Jillian Vogel
// Date:        October, 1999
//
// Description: The definition of CMtnBasePrpstnActn, a subclass of CAction.  
//	This class prepositions the motion base.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _MTN_BASE_PRPSTN_ACTN_H_
#define _MTN_BASE_PRPSTN_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CMtnBasePrpstnActn : public CAction
{
public:
	CMtnBasePrpstnActn( const CActionParseBlock*, CHcsmCollection* );
	CMtnBasePrpstnActn( const CMtnBasePrpstnActn& );
	CMtnBasePrpstnActn& operator=( const CMtnBasePrpstnActn& );
	~CMtnBasePrpstnActn();

	void Execute( const set<CCandidate>* cpInstigators = 0 );
	inline const char* GetName() const { return "MotionBasePrepos"; };

protected:
	CActionParseBlock::pbTMotionPreposition m_pos;
};

#endif	// _MTN_BASE_PRPSTN_ACTN_H_

