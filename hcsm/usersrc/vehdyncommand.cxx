/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: vehdyncommand.cxx,v 1.4 2004/04/27 19:15:18 schikore Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    January, 2000
 *
 * Description:  Contains the implementation for the CVehDynCommand class.
 *
 ****************************************************************************/

#include "vehdyncommand.h"

#include <pi_iostream>
#include <pi_fstream>
using namespace std;

//////////////////////////////////////////////////////////////////////////////
//
// Description: Prints the contents of the TVehDynCommand parameter.
//
// Remarks: Prints the contents of the TVehDynCommand to the given ostream.
//
// Arguments:
// 	 out - An ostream reference to which the TVehDynCommand will be printed.
// 	 commands - A const reference to a TVehDynCommand instance.
//
// Returns: The ostream reference so that the operator<< may be nested.
//
//////////////////////////////////////////////////////////////////////////////
ostream& 
operator<<( ostream& out, const CVehDynCommand::TVehDynCommand& command )
{

	switch ( command.type ) {

	case CVehDynCommand::eCMD_SPEED:

		out << "SPEED( " << command.value1 << ", " << command.value2 << " )";
		break;

	case CVehDynCommand::eCMD_SPEED_DIST:

		out << "SPEEDDIST( " << command.value1 << ", ";
		out << command.value2 << " )";
		break;

	case CVehDynCommand::eCMD_WAIT:

		out << "WAIT( " << command.value1 << " )";
		break;

	case CVehDynCommand::eCMD_NONE:

		out << "NONE";
		break;

	default:

		out << "UNKNOWN TYPE";

	}

	return out;

} // end of operator<<


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CVehDynCommand::CVehDynCommand()
{

}

CVehDynCommand::CVehDynCommand( const string& cmdFileName )
{

	if ( !ParseCommandFile( cmdFileName ) ) {

		// throw exception

	}

}

CVehDynCommand::CVehDynCommand( const CVehDynCommand& objToCopy )
{

	// call the assignment operator
	*this = objToCopy;

}

CVehDynCommand& 
CVehDynCommand::operator=( const CVehDynCommand& objToCopy )
{

	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) {

		m_commands = objToCopy.m_commands;

	}

	return *this;

}

CVehDynCommand::~CVehDynCommand()
{

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Parses a vehicle dynamics command file.
//
// Remarks:  This function accepts a string that represents the name of
//   of a file that contains vehicle dynamics commands.  The function
//   parses this file and builds a list of commands.
//
// Arguments:
//   cmdFileName - The command file's name.
//
// Returns:  A boolean indicating if the file was successfully parsed.
//
//////////////////////////////////////////////////////////////////////////////
bool  
CVehDynCommand::ParseCommandFile( const string& cmdFileName )
{

	//gout << "cmd file size = " << cmdFileName.size() << endl;
	if ( cmdFileName.size() <= 0 )  return false;

	// open the command file for input
	//gout << "** Command file name is " << cmdFileName << endl;
	ifstream inFile( cmdFileName.c_str() );
	
	if ( inFile.fail() ) {

		cerr << "Cannot open command file named '" << cmdFileName;
		cerr << "'" << endl;

		return false;

	}
	else {

		while ( !inFile.eof() ) {

			TVehDynCommand command;

			string str;
			inFile >> str;

			//gout << "**str = " << str << endl;

			if ( str == "speed" ) {

				command.type = eCMD_SPEED;

				inFile >> command.value1;
				inFile >> command.value2;

				m_commands.push_back( command );

			}
			else if ( str == "speeddist" ) {

				command.type = eCMD_SPEED_DIST;

				inFile >> command.value1;
				inFile >> command.value2;

				m_commands.push_back( command );

			}
			else if ( str == "wait" ) {

				command.type = eCMD_WAIT;
				inFile >> command.value1;

				m_commands.push_back( command );

			}
			else if ( str == "" ) {

				// ignore

			}
			else {

				cerr << "CVehDynCommand: unknown command in file named '";
				cerr << cmdFileName << "'" << endl;
				cerr << "  command = " << str << endl;

				return false;

			}


		}  // while

		inFile.close();

	}

	if ( m_commands.size() <= 0 ) {

		cerr << "Command file named '" << cmdFileName;
		cerr << "' has no commands" << endl;

		return false;

	}
	else {

		return true;

	}
	
}  // ParseCommandFile


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the next command in the list.
//
// Remarks:  This functions returns the next command in the list.  It
//   also removes the command from the list.
//
// Arguments:
//
// Returns:  The next command in the list.
//
//////////////////////////////////////////////////////////////////////////////
void
CVehDynCommand::GetNextCommand( TVehDynCommand& command )
{

	if ( m_commands.size() > 0 ) {

		vector<TVehDynCommand>::iterator i;
		i = m_commands.begin();
		command = *i;
		m_commands.erase( i );

	}
	else {

		command.type = eCMD_NONE;

	}

}  // GetNextCommand



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the number of commands in the list.
//
// Remarks:  This function returns the current number of commands in
//   the list.
//
// Arguments:
//
// Returns:  A integer representing the number of commands.
//
//////////////////////////////////////////////////////////////////////////////
int
CVehDynCommand::NumCommands()
{

	return m_commands.size();

}  // NumCommands
