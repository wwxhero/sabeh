/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: phonecallactn.h,v 1.6 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
// Author(s):   Matt Schikore
// Date:        August 2000
//
// Description: The definition of CPhoneCallActn, a subclass of CAction.  
// 	This class causes a phone call to be placed.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _PHONE_CALL_ACTN_H_
#define _PHONE_CALL_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CPhoneCallActn : public CAction 
{
public:
	CPhoneCallActn( const CActionParseBlock*, CHcsmCollection* );
	CPhoneCallActn( const CPhoneCallActn& );
	CPhoneCallActn& operator=( const CPhoneCallActn& );
	~CPhoneCallActn();

	void Execute( const set<CCandidate>* cpInstigators = 0 );
	inline const char* GetName() const { return "PhoneCall"; };

protected:
	CHcsmCollection* m_pHC;
};

#endif	// _PHONE_CALL_ACTN_H_

