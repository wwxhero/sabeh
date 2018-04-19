//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmcommunicate.cxx,v 1.7 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CHcsmCommunicate class.
//
//////////////////////////////////////////////////////////////////////////////

#include "hcsmcommunicate.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHcsmCommunicate::CHcsmCommunicate(
			CHcsmCollection *pRootCollection, 
			const string& cName
			):
	CHcsmObject( pRootCollection ),
	m_setFrame( cINVALID_FRAME ),
	m_prevSetFrame( cINVALID_FRAME )
{

	m_name = cName;
	m_hasValueA = false;
	m_hasValueB = false;

}

CHcsmCommunicate::CHcsmCommunicate( const CHcsmCommunicate& cRhs ):
	CHcsmObject( cRhs.m_pRootCollection )
{

	// call the assignment operator
	*this = cRhs;

}

CHcsmCommunicate& CHcsmCommunicate::operator=( const CHcsmCommunicate& cRhs )
{

	// check to make sure that object passed in is not me
	if( this != &cRhs ) 
	{
		// call parent's assignment operator
		CHcsmObject::operator=( cRhs );

		m_name = cRhs.m_name;
		m_hasValueA = cRhs.m_hasValueA;
		m_hasValueB = cRhs.m_hasValueB;
		m_setFrame  = cRhs.m_setFrame;
		m_prevSetFrame = cRhs.m_prevSetFrame;
	}

	return *this;

}

CHcsmCommunicate::~CHcsmCommunicate()
{

}

//////////////////////////////////////////////////////////////////////
// Query
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the communication object's name.
//
// Remarks:  This function returns the communication object's  name.
//
// Arguments:
//
// Returns:  A reference to an STL string that represents the communication
//   object's name.
//
//////////////////////////////////////////////////////////////////////////////
const string& CHcsmCommunicate::GetName() const
{

	return m_name;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Indicates whether the communication object holds a 
//   valid value.
//
// Remarks:  This function returns a boolean indicating whether the
//   communication object holds a valid value.  A valid value is one that 
//   has been assigned specifically by the user.
//
// Arguments:
//
// Returns:  Returns a boolean that indicates whether the communication
//   object holds a valid value.
//
//////////////////////////////////////////////////////////////////////////////
bool CHcsmCommunicate::HasValue()
{
	bool writtenThisFrame = m_setFrame == GetFrame();
	if( writtenThisFrame )
	{
		return m_hasValueB;
	}
	else
	{
		return m_hasValueA;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function contains operations shared by all Hcsm
//   communication objects when a SetValue operation is performed on them.
//
// Remarks:  This function contains operations shared by all Hcsm
//   communication objects when a SetValue operation is performed on them.
//
// Arguments:
//
// Returns:  
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCommunicate::SetValue()
{

	// do nothing for now...don't know if we need this function

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets a communication object's value to be nothing or invalid.
//
// Remarks:  This function sets a communication object's value to be 
//   nothing or invalid.
//
// Arguments:
//
// Returns:  
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmCommunicate::SetNoValue()
{
	bool setThisFrame = m_setFrame == GetFrame();
	if( !setThisFrame )
	{
		m_hasValueB = m_hasValueA;
		m_prevSetFrame = m_setFrame;

		m_hasValueA = false;
		m_setFrame = GetFrame();
	}
}
