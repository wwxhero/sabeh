/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: vehdyncommand.h,v 1.7 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         January, 2000
 *
 * Description:  Interface for the CVehDynCommand class.
 *
 ****************************************************************************/

#ifndef __CVEHDYNCOMMAND_H
#define __CVEHDYNCOMMAND_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#endif

#include <iostream>
#include <string>
#include <vector>
using namespace std;

//////////////////////////////////////////////////////////////////////////////
//
// This class contains the information structure for the ADO Vehicle
// Dynamics Class.
//
//////////////////////////////////////////////////////////////////////////////
class CVehDynCommand
{
public:
	CVehDynCommand();
	CVehDynCommand( const string& );
	CVehDynCommand( const CVehDynCommand& );
	CVehDynCommand& operator=( const CVehDynCommand& );
	~CVehDynCommand();

	enum EVehDynCommand { eCMD_NONE, eCMD_SPEED, eCMD_SPEED_DIST, eCMD_WAIT };

	typedef struct TVehDynCommand {
		EVehDynCommand  type;
		double           value1;
		double           value2;
	} TVehDynCommand;

	bool ParseCommandFile( const string& );
	void GetNextCommand( TVehDynCommand& );
	int  NumCommands();

private:
	vector<TVehDynCommand> m_commands;

};

//ostream& operator<<( ostream&, const CVehDynCommand& );
ostream& operator<<( ostream&, const CVehDynCommand::TVehDynCommand& );

#endif // __CVEHDYNCOMMAND_H
