/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2012 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: LoadVar.h,v 1.4 2014/04/23 19:23:54 IOWA\dheitbri Exp $
//
// Author(s):   David Heitbrink
//
// Date:        October, 1999
//
// Description: Header file for CLoadVar, loads a variable
//
/////////////////////////////////////////////////////////////////////////////
#pragma once
#include "action.h"
////////////////////////////////////////////////////////////////////////////////////////////////////
///\brief
///		This class loads variables from a text file	
///\remark
///		this sets varaibles based on the imput from a text a text file
///		the format of this file is the same as the store var action 
///		produces
////////////////////////////////////////////////////////////////////////////////////////////////////
class CLoadVar :
	public CAction
{
public:
	CLoadVar( const CActionParseBlock* pBlock, CHcsmCollection* pColl);
	~CLoadVar(void);
	CLoadVar& operator=( const CLoadVar& cRhs );
	CLoadVar(const CLoadVar&);
	void Execute( const set<CCandidate>* cObjs = 0 ) override;
	inline const char* GetName() const { return "LoadVar";};
private:
	string m_fileName; //<input filename;
};

