/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: expeval.h,v 1.10 2014/09/20 04:00:05 IOWA\vhorosewski Exp $
 *
 * Author(s):    Omar Ahmad, Yiannis Papelis
 *
 * Date:		 December, 2003
 *
 * Description:  An expression evaluator.
 *
 ****************************************************************************/

#ifndef __CEXPEVAL_H
#define __CEXPEVAL_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
#pragma warning(disable:4786) 
#endif

#include <ExprParser.h>
#include <pi_string>
#include <map>
#include <cved.h>
#include "cvedpub.h"
#include "hcsmspec.h"

using namespace std;

typedef double (*pFunc)(int, const CExprParser::CStrNum []);
/////////////////////////////////////////////////////////////////////////////
///\brief
///    expression parser
///\remark
///	This function evaluates experssions. The base class will val
///
/////////////////////////////////////////////////////////////////////////////
class CExpEval : public CExprParser
{

public:	
	CExpEval();
	CExpEval( const CExpEval& );
	CExpEval& operator=( const CExpEval& );
	~CExpEval();

	virtual double EvaluateVariable( const char* ) override;
	virtual bool EvaluateFunction( const string&, int, const CStrNum[], CStrNum& ) override;

	map<string, double> m_variables;
	map<string, pFunc>  m_functions;
	set<string> m_unkownVariables;
	CVED::CCved* cved;
protected:
	//common functions
    double GetQueueSize(int argC, const CExprParser::CStrNum args[]);
	double GetObjVel( int argC, const CExprParser::CStrNum args[] );
	double GetObjAccel( int argC, const CExprParser::CStrNum args[] );
	double GetObjDistPow2( int argC, const CExprParser::CStrNum args[] );
	double ReadCell( int argC, const CExprParser::CStrNum args[] );
	double ReadVar( int argC, const CExprParser::CStrNum args[] );
	double GetOvVel( int argC, const CExprParser::CStrNum args[] );
	double GetObjTtcToOv( int argC, const CExprParser::CStrNum args[] );
	double GetOvTtcToObj( int argC, const CExprParser::CStrNum args[] );
	double GetCalcDist( int argC, const CExprParser::CStrNum args[] );
    double CalcDistToOwnVeh(int argC, const CExprParser::CStrNum args[]);
};

#endif // __CEXPEVAL_H
