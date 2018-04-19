//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: outputparameter.cxx,v 1.5 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the COutputParameter class.
//
//////////////////////////////////////////////////////////////////////////////


#include "outputparameter.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

COutputParameter::COutputParameter( string name ):
	CHcsmStorage( name )
{

}

COutputParameter::COutputParameter( const COutputParameter& objToCopy ):
	CHcsmStorage( objToCopy.m_name )
{

	// call the assignment operator
	*this = objToCopy;

}

COutputParameter& COutputParameter::operator=( 
			const COutputParameter& objToCopy
			)
{

	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) {

		// call parent's assignment operator
		CHcsmStorage::operator=( objToCopy );

		// no members to copy

	}

	return *this;

}

COutputParameter::~COutputParameter()
{

}
