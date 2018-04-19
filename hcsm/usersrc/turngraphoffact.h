/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: turngraphoffact.h,v 1.3 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   David Heitbrink
//
// Date:        Dec 2010
//
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _SET_GRAPHOFF_ACTN_H_
#define _SET_GRAPHOFF_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;
/////////////////////////////////////////////////////////////////
///\brief
///		This action displays a "Graph" on the screen
//////////////////////////////////////////////////////////////////
class CTurnGraphOffActn : public CAction 
{
public:
	// Constructor
	CTurnGraphOffActn( const CActionParseBlock*, CHcsmCollection* );

	// CTurnGraphOnActn constructors
	CTurnGraphOffActn( const CTurnGraphOffActn& );

	// Destructor
	~CTurnGraphOffActn();

	// Assignment operator
	CTurnGraphOffActn& operator=( const CTurnGraphOffActn& );

	// Execute operation
	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return "CTurnGraphOffActn"; };

protected:
	CHcsmCollection*	m_pHC;
	CPoint2D			m_position;
	vector< TGraphVar>  m_graphItems;
};

#endif	// _SET_VAR_ACTN_H_