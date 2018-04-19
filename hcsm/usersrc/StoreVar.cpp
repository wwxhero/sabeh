/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2012 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: StoreVar.cpp,v 1.1 2012/05/24 16:43:34 IOWA\dheitbri Exp $
// Author(s):   Heitbrink, David
// Date:        May 2012
//
// Description: This file is the implementation of the CStoreVar class
//
/////////////////////////////////////////////////////////////////////////////
#include "hcsmspec.h"
#include "action.h"
#include "StoreVar.h"


CStoreVar::CStoreVar( const CActionParseBlock* pBlock, CHcsmCollection* pColl)
{
	assert(pBlock);
	m_fileName = pBlock->GetFile();
	m_varName  = pBlock->GetVarName();
}
////////////////////////////////////////////////////////////////////////
///\remark
///		This stores values from to the specified text file
///\remark
///		variables are always to be doubles, variables are delimted by ;  
////////////////////////////////////////////////////////////////////////
void CStoreVar::Execute( const set<CCandidate>* cObjs  ){
	ofstream offs;
	offs.open(m_fileName);
	if (!offs.is_open()){
		gout<<"CStoreVar Failed!!! could not open file "<<m_fileName<<endl;
		return;
	}
	stringstream namepaerser(m_varName);
	stringstream trimmer;
	string	varname;
	offs.precision(12);
	char tempBuff[75]= "\0";
	while (!namepaerser.fail() && !namepaerser.eof()){
		namepaerser.getline(tempBuff,75,';');
		if (!namepaerser.fail()){
			trimmer.clear();
			trimmer.str("");
			trimmer<<tempBuff;//trim anything extra we may have;
			trimmer>>varname;
			offs<<varname<<"	"<<CHcsmCollection::GetExprVariable(varname)<<endl;
		}
	}
	offs.close();
	
}
CStoreVar& 
CStoreVar::operator=( const CStoreVar& cRhs ){
	if (this != &cRhs){
		m_fileName = cRhs.m_fileName;
		m_varName = cRhs.m_varName;
	}
	return *this;
}
CStoreVar::CStoreVar(const CStoreVar&  cRhs){
	*this =  cRhs;
}
CStoreVar::~CStoreVar(void)
{
}
