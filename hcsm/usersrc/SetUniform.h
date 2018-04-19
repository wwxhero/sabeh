/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2015 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: SetUniform.h,v 1.2 2015/08/13 14:54:13 IOWA\dheitbri Exp $
//
// Author(s):   David Heitbrink
//
// Date:        Aug 2015
//
// Description: The definition of CWriteUniformActn, a subclass of CAction.  
// 	This class causes writes a value to a cell on the scramnet.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#pragma once

#include "hcsmspec.h"
#include "action.h"

#include <string>
using namespace std;

class CHcsmCollection;
/////////////////////////////////////////////////////////////////////////////
///\brief
///  Set a uniform for the shader of the target objects
///\remark
///  A uniform is a variable that is externaly set for a shader, this action
/// works in concert with the Image Generator to change the visual apperence
/// of an object
/////////////////////////////////////////////////////////////////////////////
class CWriteUniformActn : public CAction 
{
public:
	CWriteUniformActn( const CActionParseBlock*, CHcsmCollection* );
	CWriteUniformActn( const CWriteUniformActn& );
	CWriteUniformActn& operator=( const CWriteUniformActn& );
	~CWriteUniformActn();

	void Execute( const set<CCandidate>* cpInstigators = 0 );

	inline const char* GetName() const { return m_sName; };
    inline static const char* Name() { return m_sName; };
protected:
	CHcsmCollection*	m_pHC;
	string              m_varName;
	string              m_uniformName;
	CActionParseBlock::ECellDataType m_cellType;
	vector<float>       m_floatData;
	vector<int>         m_intData;
	vector<short>       m_shortData;
	bool                m_isVariable;
	const static char   m_sName[];
};

