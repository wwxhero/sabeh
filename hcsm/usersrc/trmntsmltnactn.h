/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: trmntsmltnactn.h,v 1.7 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
// Author(s):   Jillian Vogel
// Date:        October, 1999
//
// Description: The definition of CTrmntSmltnActn, a subclass of CAction.  
//	This class terminates the simulation.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _TRMNT_SMLTN_ACTN_H_
#define _TRMNT_SMLTN_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CTrmntSmltnActn : public CAction 
{
public:
	CTrmntSmltnActn( const CActionParseBlock*, CHcsmCollection* );
	CTrmntSmltnActn( const CTrmntSmltnActn& );
	~CTrmntSmltnActn();
	CTrmntSmltnActn& operator=( const CTrmntSmltnActn& );

	void Execute( const set<CCandidate>* cpInstigators = 0 );
	inline const char* GetName() const { return "TrmntSmltn"; };

protected:
};

#endif	// _TRMNT_SMLTN_ACTN_H_

