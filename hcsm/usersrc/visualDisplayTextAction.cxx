//+		m_varName	"VisualDisplayText"	std::basic_string<char,std::char_traits<char>,std::allocator<char> >
/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: visualDisplayTextAction.cxx,v 1.4 2015/05/26 16:56:45 IOWA\dheitbri Exp $
//
// Author(s):   Matt Schikore
//
// Date:        May 2004
//
// Description: The definition of CSetVarActn, a subclass of CAction.  
// 	This class causes an internal variable to be set to a specific value.
//
/////////////////////////////////////////////////////////////////////////////

#include "visualDisplayTextAction.h"
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
CSetDisplayText::CSetDisplayText(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC(pHC), m_location(1)
{
	m_delay = cpBlock->GetDelay();
	m_varName = cpBlock->GetVarName();
	m_varValue = cpBlock->GetVisualDisplayValue();

	stringstream converter;
	string locationText = cpBlock->GetVisualDisplayLocation();
	converter << locationText;
	converter >> m_location;

	//m_isExpr = cpBlock->GetIsVarValExpression();
	//if (m_isExpr)
	//	m_expr.Parse(m_varValue.c_str());
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
CSetDisplayText::CSetDisplayText( const CSetDisplayText& cRhs )
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
CSetDisplayText::~CSetDisplayText() {}

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
CSetDisplayText&
CSetDisplayText::operator=( const CSetDisplayText& cRhs )
{
	if( this != &cRhs)
	{
		m_pHC = cRhs.m_pHC;
		m_varName = cRhs.m_varName;
		m_varValue = cRhs.m_varValue;
		m_isExpr = cRhs.m_isExpr;
		if (m_isExpr){
			string temps;
			cRhs.m_expr.Store(temps);
			m_expr.Load(temps);
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
CSetDisplayText::Execute( const set<CCandidate>* ) 
{
	string::size_type pos, pos_end;
	string varName, varValue;

	stringstream converter;
    string varValOut = m_varValue;
	bool isDone = false;
	while (!isDone) {
		pos = varValOut.find('%');
		if (pos != string::npos) {
			pos_end = varValOut.find('%', pos + 1);
			if (pos_end != string::npos) {
				varValue = "";
				int varNameLength = pos_end - (pos + 1);
				varName = varValOut.substr(pos + 1, varNameLength);
				if (CHcsmCollection::ExprVariableExists(varName)) {
					converter.str("");
					converter.clear();
					converter.setf(ios::fixed);
					converter.precision(2);
					converter << CHcsmCollection::GetExprVariable(varName);
					converter >> varValue;
				} else {
					//gout << "Set Display Text Action: Warning Cannot Find Variable " << varName << endl;
				}
				varValOut.replace(pos, varNameLength + 2, varValue);
			} else {
				isDone = true;
				//gout << "Set Display Text Action: Warning missing '%' in string" << endl;
			}
		} else { 
			isDone = true;
		}
	}
	CHcsmCollection::SetVisualDisplayText(varValOut, m_location);
	//CHcsmCollection::SetActionSetVariableLog(m_triggerId, "VisualDisplayText", newValue);
}
