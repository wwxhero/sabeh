/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2012 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: LoadVar.cpp,v 1.1 2012/05/24 16:43:34 IOWA\dheitbri Exp $
// Author(s):   Heitbrink, David
// Date:        May 2012
//
// Description: This file is the implementation of the CLoadVar class
//
/////////////////////////////////////////////////////////////////////////////

#include "hcsmspec.h"
#include "action.h"
#include "LoadVar.h"


CLoadVar::CLoadVar( const CActionParseBlock* pBlock, CHcsmCollection* pColl)
{
	assert(pBlock);
	m_fileName = pBlock->GetFile();
}
CLoadVar& 
CLoadVar::operator=( const CLoadVar& cRhs ){
	if (this != &cRhs){
		m_fileName = cRhs.m_fileName;
	}
	return *this;
}
CLoadVar::CLoadVar(const CLoadVar&  cRhs){
	*this =  cRhs;
}
////////////////////////////////////////////////////////////////////////
///\remark
///		This loads values from the specified text file, and sets the values
///\remark
///		variables are assumed to be doubles... 
////////////////////////////////////////////////////////////////////////
void CLoadVar::Execute( const set<CCandidate>* cObjs ){
	ifstream iffs;
	iffs.open(m_fileName);
	if (!iffs.is_open()){
		gout<<"CStoreVar Failed!!! could not open file "<<m_fileName<<endl;
		return;
	}
	string name; double value;
	while (!iffs.fail() && !iffs.eof()){
		iffs>>name>>value;
		if (!iffs.fail()){
			CHcsmCollection::SetExprVariable(name,value);
		}
	}
	iffs.close();
	
}

CLoadVar::~CLoadVar(void)
{
}