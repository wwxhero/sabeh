/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: TurnGraphOnActn.h,v 1.3 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
//
// Author(s):   David Heitbrink
//
// Date:        Dec 2010
//
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SET_GRAPHON_ACTN_H_
#define _SET_GRAPHON_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;
/////////////////////////////////////////////////////////////////
///\brief
///		This action displays a "Graph" on the screen
//////////////////////////////////////////////////////////////////
class CTurnGraphOnActn : public CAction 
{
public:
	// Constructor
	CTurnGraphOnActn( const CActionParseBlock*, CHcsmCollection* );

	// CTurnGraphOnActn constructors
	CTurnGraphOnActn( const CTurnGraphOnActn& );

	// Destructor
	~CTurnGraphOnActn();

	// Assignment operator
	CTurnGraphOnActn& operator=( const CTurnGraphOnActn& );

	// Execute operation
	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return "CTurnGraphOnActn"; };

protected:
	CHcsmCollection*	m_pHC;
	CPoint2D			m_position;
	vector< TGraphVar>  m_graphItems;
};

#endif	// _SET_VAR_ACTN_H_