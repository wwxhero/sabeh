/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: expevalAdo.h,v 1.6 2015/07/01 19:23:01 IOWA\dheitbri Exp $
 *
 * Author(s):    D.A. Heitbrink
 *
 * Date:		 March, 2008
 *
 * Description:  An expression evaluator.
 *
 ****************************************************************************/

#ifndef __CEXPEVALADO_H
#define __CEXPEVALADO_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
#pragma warning(disable:4786) 
#endif

#include <ExprParser.h>
#include <pi_string>
#include <map>
class CAdo;
using namespace std;

//typedef double (*pFunc)(int, const CExprParser::CStrNum []);
typedef double (CAdo::*pAdoFunc)(int, const CExprParser::CStrNum []);
//////////////////////////////////////////////////////////////////////////////
///\brief
///		Expression Parser for ADO use with Forced Velocity	
///\remark
///		This function parser, takes in CAdo member functions as input
///
//////////////////////////////////////////////////////////////////////////////
class CExpEvalAdo : public CExprParser
{

public:
	CExpEvalAdo();
	CExpEvalAdo( const CExpEvalAdo& );
	CExpEvalAdo& operator=( const CExpEvalAdo& );
	~CExpEvalAdo();

	virtual double EvaluateVariable( const char* ) override;
	virtual bool EvaluateFunction( const string&, int, const CStrNum[], CStrNum& ) override;
	void SetParent(CAdo * parent){
		m_pParent = parent;
	}
	CAdo *m_pParent;
	map<string, double> m_variables;
	map<string, pAdoFunc>  m_functions;
	
};

#endif // __CEXPEVAL_H
