/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: 
 *
 * Author:  Omar Ahmad, Yiannis Papelis
 *
 * Date:    December, 2003
 *
 * Description:  Contains the implementation for the CExpEval class.
 *
 ****************************************************************************/

#include "expevalTTA.h"
#include "hcsmpch.h"
#include <pi_iostream>
using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CExpEvalTTA::CExpEvalTTA()
{
	//m_pParent = parent;
}

CExpEvalTTA::CExpEvalTTA( const CExpEvalTTA& objToCopy )
{
	// call the assignment operator
	*this = objToCopy;
}

CExpEvalTTA& 
CExpEvalTTA::operator=( const CExpEvalTTA& objToCopy )
{
	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) 
	{

	}

	return *this;
}

CExpEvalTTA::~CExpEvalTTA()
{

}

double
CExpEvalTTA::EvaluateVariable( const char* cpName )
{
	map<string, double>::iterator p = m_variables.find( cpName );
	bool foundVariable = p != m_variables.end();

	if( foundVariable )
	{
		return p->second;
	}
	else
	{
		cerr << "CExpEval: unknown variable '" << cpName << "'" << endl;
		return 0.0;
	}
}

bool
CExpEvalTTA::EvaluateFunction(
			const string& cName,
			int numArg,
			const CStrNum args[],
			CStrNum& result
			)
{
	map<string, pTTAFunc>::iterator p = m_functions.find( cName );
	bool foundFunction = p != m_functions.end();

	if( foundFunction )
	{
		pTTAFunc tempFunc = p->second;
		//(m_pParent->*tempFunc)( numArg, args );
		result.SetVal( (m_pParent->*tempFunc)( numArg, args ) );
		return true;
	}
	else
	{
		cerr << "CExpEvalTTA: unknown function '" << cName << "'" << endl;
		return false;
	}
}
