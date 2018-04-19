/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: resetdialactn.h,v 1.6 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
// Author(s):   Omar Ahmad
// Date:        August, 2002
//
// Description:  Definition of the CResetDialActn class.  This action sets
//  the given dial of its candidates to the given value.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _RESET_DIAL_ACTN_H_
#define _RESET_DIAL_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CResetDialActn : public CAction 
{
public:
	CResetDialActn( const CActionParseBlock*, CHcsmCollection* );
	CResetDialActn( const CResetDialActn& );
	~CResetDialActn();
	CResetDialActn &operator=( const CResetDialActn& );

	void Execute( const set<CCandidate>* pInstigators = 0 );
	inline const char* GetName() const { return "ResetDial"; };

protected:
	CHcsmCollection* m_pHC;
	string           m_dialName;
};

#endif	// _RESET_DIAL_ACTN_H_

