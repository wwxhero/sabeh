/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: TurnGraphOnAct.cpp,v 1.3 2013/07/25 02:49:14 IOWA\vhorosewski Exp $
//
// Author(s):   Matt Schikore
//
// Date:        May 2004
//
// Description: The definition of CSetVarActn, a subclass of CAction.  
// 	This class causes an internal variable to be set to a specific value.
//
/////////////////////////////////////////////////////////////////////////////

#include "TurnGraphOnActn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"
#include <sstream>

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:	
//	pBlock - pointer to the CActionParseBlock associated with
//			this CAction.
//	pHC - pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTurnGraphOnActn::CTurnGraphOnActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	char delim;
	float x;
	float y;
	stringstream parser;
	string temps;
	TGraphVar curVar;

	memset(&curVar,0,sizeof(curVar));
	
	m_delay = cpBlock->GetDelay();

	parser.str(cpBlock->GetScreenPosition());
	parser>>m_position.m_x>>delim>>m_position.m_y;
	parser.clear();
	temps = cpBlock->GetVarName();
	unsigned int currPos = 0;
	char currChar = 0;

	bool addingVarName = true;
	bool addingGroupName = false;
	bool addingLbl = false;

	unsigned short varNamePos = 0;
	unsigned short groupNamePos = 0;
	unsigned short lblPos = 0;
	bool addedVar = false;
	bool haveQuote = false;
	while (currPos < temps.size()){
		currChar = temps[currPos];
		if (haveQuote && currChar == '\''){
			haveQuote = false;
			currPos++;
			continue;
		}
		else if (!haveQuote && currChar == '\''){
			haveQuote = true;
			currPos++;
			continue;		
		}

		if (addingVarName){
			if ((currChar == ' ' ||currChar == ';' || currChar == '\t') && addedVar == false && !haveQuote){ //end of var define
				varNamePos = 0;
				m_graphItems.push_back(curVar);
				memset(&curVar,0,sizeof(curVar));
				addedVar = true;
			}else if(currChar == ':'){
				varNamePos = 0;
				addingVarName = false;
				addingLbl = true;
			}else{ //we are adding to the name
				addedVar = false;
				curVar.varName[varNamePos++] = currChar;
			}
		}
		else if(addingLbl){
			if ( (currChar == ' ' ||currChar == ';'|| currChar == '\t') && !haveQuote ){ //end of var define
				lblPos = 0;
				addingVarName = true;
				addingLbl = false;
				m_graphItems.push_back(curVar);
				memset(&curVar,0,sizeof(curVar));
				addedVar = true;
			}else if (currChar == ':'){
				lblPos = 0;
				addingLbl = false;
				addingGroupName = true;
			}
			else{ //we are adding to the name
				addedVar = false;
				curVar.label[lblPos++] = currChar;
			}		
		}
		else if (addingGroupName){
			if ( (currChar == ' ' ||currChar == ';' || currChar == '\t') && !haveQuote){ //end of var define
				groupNamePos = 0;
				addingVarName = true;
				addingGroupName = false;
				m_graphItems.push_back(curVar);
				memset(&curVar,0,sizeof(curVar));
				addedVar = true;
			}else{ //we are adding to the name
				addedVar = false;
				curVar.group[groupNamePos++] = currChar;
			}		
		}
		currPos++;
	}
	if (temps.size() > 0 && addedVar == false){
		m_graphItems.push_back(curVar);
	}
	


	

	//vector< pair<string,string> > m_graphItems;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CSetVarActn.
//
// Remarks: 
//
// Arguments: CSetVarActn to be copied into current CSetVarActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTurnGraphOnActn::CTurnGraphOnActn( const CTurnGraphOnActn& cRhs )
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
CTurnGraphOnActn::~CTurnGraphOnActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CSetVarActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CTurnGraphOnActn&
CTurnGraphOnActn::operator=( const CTurnGraphOnActn& cRhs )
{
	if( this != &cRhs)
	{
		m_pHC = cRhs.m_pHC;
		m_position = cRhs.m_position;
		unsigned int size = m_graphItems.size();
		for (unsigned int i =0; i < size; i++){
			m_graphItems.push_back(cRhs.m_graphItems[i]);
		}	
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The Execute function, which is called when the action
//              should be executed
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////

void 
CTurnGraphOnActn::Execute( const set<CCandidate>* ) 
{
	CHcsmCollection::m_sDisplayGraph.m_vars.clear();
	unsigned int size = this->m_graphItems.size();
	for (unsigned int i =0; i < size; i++){
		CHcsmCollection::m_sDisplayGraph.m_vars.push_back(m_graphItems[i]);
	}
	CHcsmCollection::m_sDisplayGraph.m_type = COnScreenGraph::eBARGRAPH;
	CHcsmCollection::m_sDisplayGraph.m_position = m_position;
	CHcsmCollection::m_sGraphIsOn = true;
}