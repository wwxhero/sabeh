/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: 
 *
 * Author(s):    D.A. Heitbrink
 *
 * Date:		 March, 2008
 *
 * Description:  An expression evaluator.
 *
 ****************************************************************************/

#ifndef __CEXPEVALTTA_H
#define __CEXPEVALTTA_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
#pragma warning(disable:4786) 
#endif

#include <ExprParser.h>
#include <pi_string>
#include <map>
class CTimeToArrvlTrigger;
using namespace std;

//typedef double (*pFunc)(int, const CExprParser::CStrNum []);
typedef double (CTimeToArrvlTrigger::*pTTAFunc)(int, const CExprParser::CStrNum []);
//////////////////////////////////////////////////////////////////////////////
///\brief
///		Expression Parser for TTA use with Forced Velocity	
///\remark
///		This function parser, takes in CTTA member functions as input
///
//////////////////////////////////////////////////////////////////////////////
class CExpEvalTTA : public CExprParser
{

public:
	CExpEvalTTA();
	CExpEvalTTA( const CExpEvalTTA& );
	CExpEvalTTA& operator=( const CExpEvalTTA& );
	~CExpEvalTTA();

	double EvaluateVariable( const char* );
	bool EvaluateFunction( const string&, int, const CStrNum[], CStrNum& );
	void SetParent(CTimeToArrvlTrigger * parent){
		m_pParent = parent;
	}
	CTimeToArrvlTrigger *m_pParent;
	map<string, double> m_variables;
	map<string, pTTAFunc>  m_functions;

	
};

#endif // __CEXPEVAL_H
