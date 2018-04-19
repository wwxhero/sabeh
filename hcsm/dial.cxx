//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: dial.cxx,v 1.9 2013/05/08 15:31:01 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CDial class.
//
//////////////////////////////////////////////////////////////////////////////

#include "dial.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CDial::CDial( CHcsmCollection *pRootCollection, const string& cName ):
	CHcsmCommunicate( pRootCollection, cName ),
	m_hasBeenResetA( false ),
	m_hasBeenResetB( false ),
	m_resetFrame( cINVALID_FRAME )
{

}

CDial::CDial( const CDial& cRhs ):
	CHcsmCommunicate( cRhs.m_pRootCollection, cRhs.m_name )
{
	// call the assignment operator
	*this = cRhs;
}

CDial& CDial::operator=( const CDial& cRhs )
{

	// check to make sure that object passed in is not me
	if( this != &cRhs ) 
	{
		// call parent's assignment operator
		CHcsmCommunicate::operator=( cRhs );

		m_hasBeenResetA = cRhs.m_hasBeenResetA;
		m_hasBeenResetB = cRhs.m_hasBeenResetB;
		m_resetFrame = cRhs.m_resetFrame;
	}

	return *this;

}

CDial::~CDial()
{

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function reports if a dial has been reset.
//
// Remarks:  
//
// Arguments:
//
// Returns:  A boolean.
//
//////////////////////////////////////////////////////////////////////////////
bool
CDial::HasBeenReset()
{
	bool resetThisFrame = m_setFrame == GetFrame();
	if( resetThisFrame )
	{
		return m_hasBeenResetB;
	}
	else
	{
		return m_hasBeenResetA;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function resets a dial.
//
// Remarks:  
//
// Arguments:
//
// Returns:  
//
//////////////////////////////////////////////////////////////////////////////
void
CDial::Reset()
{
	bool setThisFrame = m_setFrame == GetFrame();
	if( !setThisFrame )
	{
		m_hasValueB = m_hasValueA;
		m_hasBeenResetB = m_hasBeenResetA;
		m_prevSetFrame = m_setFrame;

		m_hasValueA = false;
		m_hasBeenResetA = true;
		m_setFrame = GetFrame();
		m_resetFrame = GetFrame();
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
void CDial::SetValue()
{
	bool setThisFrame = m_setFrame == GetFrame();
	if( !setThisFrame )
	{
		m_hasValueB = m_hasValueA;
		m_hasBeenResetB = m_hasBeenResetA;
		m_prevSetFrame = m_setFrame;

		m_hasValueA = true;
		m_hasBeenResetA = false;
		m_setFrame = GetFrame();
		m_resetFrame = cINVALID_FRAME;
	}
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
void CDial::SetNoValue()
{
	bool setThisFrame = m_setFrame == GetFrame();
	if( !setThisFrame )
	{
		m_hasValueB = m_hasValueA;
		m_hasBeenResetB = m_hasBeenResetA;
		m_prevSetFrame = m_setFrame;

		m_hasValueA = false;
		m_hasBeenResetA = false;
		m_setFrame = GetFrame();
		m_resetFrame = cINVALID_FRAME;
	}
}
