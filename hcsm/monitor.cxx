//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: monitor.cxx,v 1.6 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CMonitor class.
//
//////////////////////////////////////////////////////////////////////////////


#include "monitor.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CMonitor::CMonitor( CHcsmCollection *pRootCollection, string name ):
	CHcsmCommunicate( pRootCollection, name )
{

}

CMonitor::CMonitor( const CMonitor& objToCopy ):
	CHcsmCommunicate( objToCopy.m_pRootCollection, objToCopy.m_name )
{

	// call the assignment operator
	*this = objToCopy;

}

CMonitor& CMonitor::operator=( const CMonitor& objToCopy )
{

	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) {

		// call parent's assignment operator
		CHcsmCommunicate::operator=( objToCopy );

		// no members to copy

	}

	return *this;

}

CMonitor::~CMonitor()
{

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
void CMonitor::SetValue()
{
	bool setThisFrame = m_setFrame == GetFrame();
	if( !setThisFrame )
	{
		m_hasValueB = m_hasValueA;
		m_prevSetFrame = m_setFrame;

		m_hasValueA = true;
		m_setFrame = GetFrame();
	}
}
