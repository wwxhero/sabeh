/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: writecellactn.h,v 1.4 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Omar Ahmad
//
// Date:        September, 2004
//
// Description: The definition of CWriteCellActn, a subclass of CAction.  
// 	This class causes writes a value to a cell on the scramnet.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _WRITE_CELL_ACTN_H_
#define _WRITE_CELL_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

#include <string>
using namespace std;

class CHcsmCollection;

class CWriteCellActn : public CAction 
{
public:
	CWriteCellActn( const CActionParseBlock*, CHcsmCollection* );
	CWriteCellActn( const CWriteCellActn& );
	CWriteCellActn& operator=( const CWriteCellActn& );
	~CWriteCellActn();

	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return "WriteCell"; };

protected:
	CHcsmCollection*	m_pHC;

	string              m_cellName;
	CActionParseBlock::ECellDataType m_cellType;
	vector<float>       m_floatData;
	vector<int>         m_intData;
	vector<short>       m_shortData;
	string              m_cellData;
	bool                m_isVariable;
};

#endif	// _WRITE_CELL_ACTN_H_

