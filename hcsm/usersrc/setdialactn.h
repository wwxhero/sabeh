/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setdialactn.h,v 1.8 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description:  Definition of the CSetDialActn class.  This action sets
// 	             the given dial of its candidates to the given value.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SET_DIAL_ACTN_H_
#define _SET_DIAL_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CSetDialActn : public CAction 
{
public:
	// Constructor
	CSetDialActn( const CActionParseBlock*, CHcsmCollection*, bool debug = false );

	// Copy constructors
	CSetDialActn( const CSetDialActn& );

	// Destructor
	~CSetDialActn();

	// Assignment operator
	CSetDialActn &operator=( const CSetDialActn& );

	// Execute operation
	void Execute( const set<CCandidate>* pInstigators = 0 );

	bool IsFinished();

	inline const char* GetName() const { return "SetDial"; };

protected:
	CHcsmCollection* m_pHC;
	string           m_dialName;
	string           m_dialValue;

private:
	bool m_debug;

	typedef struct TResetHcsm
	{
		int  hcsmId;
		bool isActive;
	} TResetHcsm;

	vector<TResetHcsm> m_waitingForResetHcsm;

	void SetDialOnHcsm( int hcsmId );
};

#endif	// _SET_DIAL_ACTN_H_

