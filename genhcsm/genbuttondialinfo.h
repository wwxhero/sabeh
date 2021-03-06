/*****************************************************************************
 *
 *  (C) Copyright 1998 by National Advanced Driving Simulator and
 *  Simulation Center, the University of Iowa and The University
 *  of Iowa. All rights reserved.
 *
 *  This file has been generated by the hcsm code generator.
 *  ### DO NOT EDIT DIRECTLY ###
 *
 */


#ifndef __BUTTON_DIAL_INFO_H_
#define __BUTTON_DIAL_INFO_H_

#include <string>
#include <vector>
using namespace std;

typedef struct TDialParam
{
	string m_name;
	string m_dataType;
	string m_units;
	string m_defValue;
	string m_comment;
	bool   m_optional;
} TDialParam;

typedef struct TButtonDialInfo
{
	string m_hcsmName;
	string m_name;
	string m_comment;
	bool   m_isDial;
	vector<TDialParam> m_params;
} TButtonDialInfo;

class CButtonDialInfo
{
public:
	CButtonDialInfo();
	~CButtonDialInfo();

	void GetButtonInfo( const string& cHcsmName, vector<TButtonDialInfo>& output );
	void GetDialInfo( const string& cHcsmName, vector<TButtonDialInfo>& output );
	void GetButtonDialInfo( vector<TButtonDialInfo>& output );

private:
	vector<TButtonDialInfo> m_buttonDialInfo;
};

#endif // __BUTTON_DIAL_INFO_H_

