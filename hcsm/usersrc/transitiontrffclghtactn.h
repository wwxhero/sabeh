/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: transitiontrffclghtactn.h,v 1.6 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description: The definition of CTransitionTrffcLghtActn, a subclass of CAction.  
//	This class causes the indicated objects to play the indicated audios.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SET_TRFFC_LGHT_ACTN_H_
#define _SET_TRFFC_LGHT_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CTransitionTrffcLghtActn : public CAction 
{
public:
	// Constructor
	CTransitionTrffcLghtActn( const CActionParseBlock*, CHcsmCollection* );

	// Copy constructors
	CTransitionTrffcLghtActn( const CTransitionTrffcLghtActn& );

	// Destructor
	~CTransitionTrffcLghtActn();

	// Assignment operator
	CTransitionTrffcLghtActn& operator=( const CTransitionTrffcLghtActn& );

	// Execute operation
	void Execute( const set<CCandidate>* instigators = 0 );

	inline const char* GetName() const { return "TransitionTrafficLight"; };

protected:
	CHcsmCollection*	m_pHC;
	string				m_stateTime;
	vector<string>		m_byNameSet;
};

#endif	// _SET_TRFFC_LGHT_ACTN_H_

