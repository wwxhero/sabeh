//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmstorage.cxx,v 1.7 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CHcsmStorage class.
//
//////////////////////////////////////////////////////////////////////////////


#include "hcsmstorage.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHcsmStorage::CHcsmStorage( string name )
{

	m_name = name;
	m_hasValue = false;

}

CHcsmStorage::CHcsmStorage( const CHcsmStorage& objToCopy )
{

	// call the assignment operator
	*this = objToCopy;

}

CHcsmStorage& CHcsmStorage::operator=( const CHcsmStorage& objToCopy )
{

	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) {

		m_name = objToCopy.m_name;
		m_hasValue = objToCopy.m_hasValue;

	}

	return *this;

}

CHcsmStorage::~CHcsmStorage()
{

}
