/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:   $Id: logdataactn.h,v 1.9 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
// Author(s): Jillian Vogel
// Date:      October, 1999
//
// Description: The definition of CLogDataActn, a subclass of CAction.  
//	This class logs the simulation data to the indicated file.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _LOG_DATA_ACTN_H_
#define _LOG_DATA_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CLogDataActn : public CAction 
{
public:
	CLogDataActn( const CActionParseBlock*, CHcsmCollection* );
	CLogDataActn( const CLogDataActn& cRhs );
	CLogDataActn& operator=( const CLogDataActn& cRhs );
	~CLogDataActn();

	void Execute( const set<CCandidate>* cpInstigators = 0 );
	inline const char* GetName() const { return "LogData"; };

protected:
	CHcsmCollection* m_pHC;
	int              m_streamNum;
	double           m_streamVal;
};

#endif	// _LOG_DATA_ACTN_H_

