/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2015 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: UpdateTODActn.cpp,v 1.4 2016/07/15 14:50:38 IOWA\dheitbri Exp $
// Author(s):   David Heitbrink
// Date:        May, 2016
//
// Description: The definition of CUpdateTodActn, a subclass of CAction.  
// 	This class writes a value to variable that's mapped to a cell on the
//  scramnet.
//
/////////////////////////////////////////////////////////////////////////////

#include "UpdateTODActn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
const char CUpdateTodActn::m_sName[] ="SetTimeOfDay";

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:	
//	pBlock - A pointer to the CActionParseBlock associated with
//			this CAction.
//	pHC - A pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CUpdateTodActn::CUpdateTodActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	InitCandidateSet( cpBlock );
    m_setTimeCommand = cpBlock->GetSetTimeOfDayStr();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//	CUpdateTodActn.
//
// Remarks: 
//
// Arguments: CUpdateTodActn to be copied into current CUpdateTodActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CUpdateTodActn::CUpdateTodActn( const CUpdateTodActn& cRhs )
{
	*this = cRhs;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The destructor.
//
// Remarks: 
//
// Arguments: none
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CUpdateTodActn::~CUpdateTodActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//	current action to the parameter and returns a reference  to the 
//  current object.
//
// Remarks: 
//
// Arguments: A reference to the CUpdateTodActn to assign.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CUpdateTodActn&
CUpdateTodActn::operator=( const CUpdateTodActn& cRhs )
{
	if( this != &cRhs )
	{
		m_setTimeCommand = cRhs.m_setTimeCommand;
	}

	return *this;
}
	//else if (m_varName == "Hour"){
	//	if ( m_varValue == "++" ){
	//		CHcsmCollection::m_sHour++;
	//		if (CHcsmCollection::m_sHour >= 24){
	//			CHcsmCollection::m_sHour = 0;
	//		}
	//	}
	//	else{
	//		sscanf(m_varValue.c_str(), "%lf", &newValue);
	//		CHcsmCollection::m_sHour = (int)newValue;
	//	}
	//	CHcsmCollection::SetExprVariable( m_varName, CHcsmCollection::m_sHour );
	//	CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, CHcsmCollection::m_sHour );
	//}
	//else if (m_varName == "Minute"){
	//	if ( m_varValue == "++" ){
	//		CHcsmCollection::m_sMinute++;
	//		if (CHcsmCollection::m_sMinute >= 60){
	//			CHcsmCollection::m_sMinute = 0;
	//			CHcsmCollection::m_sHour++;
	//			if (CHcsmCollection::m_sHour >= 24){
	//				CHcsmCollection::m_sHour = 0;
	//			}
	//		}
	//		CHcsmCollection::SetExprVariable( "Minute", CHcsmCollection::m_sMinute );
	//		CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, CHcsmCollection::m_sMinute );
	//		CHcsmCollection::SetExprVariable( "Hour", CHcsmCollection::m_sHour );
	//		CHcsmCollection::SetActionSetVariableLog( m_triggerId, m_varName, CHcsmCollection::m_sHour );
	//	}
	//	else{
	//		sscanf(m_varValue.c_str(), "%lf", &newValue);
	//		CHcsmCollection::m_sMinute = (int)newValue;
	//	}
	//}
	//else if (m_varName == "UpdateTime"){
	//	sscanf(m_varValue.c_str(), "%lf", &newValue);
	//	if (newValue > 0){
	//		CHcsmCollection::m_sChangeTimeOfDay = true;
	//	}else{
	//		CHcsmCollection::m_sChangeTimeOfDay = false;
	//	}
	//}
/////////////////////////////////////////////////////////////////////////////
//
// Description: The Execute function, which is called when the action
//  should be executed
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CUpdateTodActn::Execute( const set<CCandidate>* ) 
{
/*  std::string str = ";;Hello|world||-foo--bar;yow;baz|";
  typedef boost::tokenizer<boost::char_separator<char> > 
    tokenizer;
  boost::char_separator<char> sep("-;|");
  tokenizer tokens(str, sep);
  for (tokenizer::iterator tok_iter = tokens.begin();
       tok_iter != tokens.end(); ++tok_iter)
    std::cout << "<" << *tok_iter << "> ";
  std::cout << "\n";   */ 
    m_setTimeCommand;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> sep("; ",":");
	string dailstr = m_setTimeCommand;
	tokenizer tokens(dailstr, sep);
	tokenizer::iterator itr = tokens.begin();
	int x = CHcsmCollection::m_sMinute;
	while (itr != tokens.end()){
		if (*itr == "Time"){
			itr++; if (itr == tokens.end()) break;
			//if (itr->find_first_not_of("1234567890") == string::npos) break;
			int hours = boost::lexical_cast<int>(*itr);
			itr++; if (itr == tokens.end()) break;
			if (itr->find_first_not_of("1234567890") == string::npos) break;
			itr++;
			int minutes = boost::lexical_cast<int>(*itr);
			if (hours < 0 || hours > 24) break;
			if (minutes < 0 || minutes > 59) break;
			CHcsmCollection::m_sHour = hours;
			CHcsmCollection::m_sMinute = minutes;
			CHcsmCollection::m_sChangeTimeOfDay = true;
			return;
		}
		if (*itr == "Minute++"){
			CHcsmCollection::m_sMinute++;
			if (CHcsmCollection::m_sMinute >= 60){
        		CHcsmCollection::m_sMinute = 0;
        		CHcsmCollection::m_sHour++;
        		if (CHcsmCollection::m_sHour >= 24){
        			CHcsmCollection::m_sHour = 0;
        		}
			}
			CHcsmCollection::m_sChangeTimeOfDay = true;
			return;
		}
		if (*itr == "Hour++"){
        	CHcsmCollection::m_sHour++;
        	if (CHcsmCollection::m_sHour >= 24){
        		CHcsmCollection::m_sHour = 0;
        	}
			CHcsmCollection::m_sChangeTimeOfDay = true;
			return;
		}
		else{
			itr++;
		}
	}
	gout<<"Syntax Error For "<< CUpdateTodActn::m_sName<<" Action in string->"<<m_setTimeCommand<<endl;
}
