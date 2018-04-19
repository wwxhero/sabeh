/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: stopdataredactn.h,v 1.5 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
// Author(s):   Matt Schikore
// Date:        Dec 2003
//
// Description: The definition of CStopDataredActn, a subclass of CAction.  
// 	This class causes a data reduction segment to end.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _STOP_DATARED_ACTN_H_
#define _STOP_DATARED_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CStopDataredActn : public CAction 
{
public:
	CStopDataredActn( const CActionParseBlock*, CHcsmCollection* );
	CStopDataredActn( const CStopDataredActn& );
	~CStopDataredActn();
	CStopDataredActn& operator=( const CStopDataredActn& );

	void Execute( const set<CCandidate>* cpInstigators = 0 );
	inline const char* GetName() const { return "StopDataReduc"; };

protected:
	CHcsmCollection*	m_pHC;
	string				m_data;
};

#endif	// _STOP_DATARED_ACTN_H_

