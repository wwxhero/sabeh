/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: expevalAdo.cxx,v 1.3 2012/06/14 23:26:24 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad, Yiannis Papelis
 *
 * Date:    December, 2003
 *
 * Description:  Contains the implementation for the CExpEval class.
 *
 ****************************************************************************/

#include "expevalado.h"
#include "hcsmpch.h"
#include <pi_iostream>
using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CExpEvalAdo::CExpEvalAdo()
{
	//m_pParent = parent;
}

CExpEvalAdo::CExpEvalAdo( const CExpEvalAdo& objToCopy )
{
	// call the assignment operator
	*this = objToCopy;
}

CExpEvalAdo& 
CExpEvalAdo::operator=( const CExpEvalAdo& objToCopy )
{
	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) 
	{

	}

	return *this;
}

CExpEvalAdo::~CExpEvalAdo()
{

}

double
CExpEvalAdo::EvaluateVariable( const char* cpName )
{
	map<string, double>::iterator p = m_variables.find( cpName );
	bool foundVariable = p != m_variables.end();

	if( foundVariable )
	{
		return p->second;
	}
	else
	{
		if (CHcsmCollection::ExprVariableExists( cpName )){
            return CHcsmCollection::GetExprVariable(cpName);
        }else{
            cerr << "CExpEval: unknown variable '" << cpName << "'" << endl;
            return 0.0;
        }
		
	}
}

bool
CExpEvalAdo::EvaluateFunction(
			const string& cName,
			int numArg,
			const CStrNum args[],
			CStrNum& result
			)
{
	map<string, pAdoFunc>::iterator p = m_functions.find( cName );
	bool foundFunction = p != m_functions.end();

	if( foundFunction )
	{
		pAdoFunc tempFunc = p->second;
		//(m_pParent->*tempFunc)( numArg, args );
		result.SetVal( (m_pParent->*tempFunc)( numArg, args ) );
		return true;
	}
	else
	{
		cerr << "CExpEvalAdo: unknown function '" << cName << "'" << endl;
		return false;
	}
}
