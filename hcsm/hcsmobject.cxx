//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmobject.cxx,v 1.10 2013/05/08 15:31:02 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CHcsmObject class.
//
//////////////////////////////////////////////////////////////////////////////
#include <tchar.h>

#include "hcsmobject.h"
#include "hcsmcollection.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHcsmObject::CHcsmObject( CHcsmCollection* pRootCollection )
{

	m_pRootCollection = pRootCollection;
	cved = m_pRootCollection->GetCved();

}

CHcsmObject::CHcsmObject( const CHcsmObject& objToCopy )
{

	// call the assignment operator
	*this = objToCopy;

}

CHcsmObject& CHcsmObject::operator=( const CHcsmObject& objToCopy )
{

	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) {

		m_pRootCollection = objToCopy.m_pRootCollection;
		cved = objToCopy.cved;

	}

	return *this;

}

CHcsmObject::~CHcsmObject()
{

}

//////////////////////////////////////////////////////////////////////
// Query
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the current frame number.
//
// Remarks:  This function returns the current frame number of the HCSM
//   system by querying the root collection instance assigned to it.
//
// Arguments:
//
// Returns:  Returns an integer that represents the current HCSM system
//   frame number.
//
//////////////////////////////////////////////////////////////////////////////
int CHcsmObject::GetFrame() const
{

	return m_pRootCollection->GetFrame();

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the time step duration.
//
// Remarks:  This function returns the time step duration of the HCSM
//   system by querying the root collection instance assigned to it.
//
// Arguments:
//
// Returns:  Returns an double that represents the HCSM system time step
//   duration.
//
//////////////////////////////////////////////////////////////////////////////
double CHcsmObject::GetTimeStepDuration() const
{

	return m_pRootCollection->GetTimeStepDuration();

}

//////////////////////////////////////////////////////////////////////
// Create, Delete
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Creates an Hcsm.
//
// Remarks:  Given a template name and initialization information, this
//   function creates an instance of that template.
//
// Arguments:
//   hcsmName - The template's name.
//   snoBlock - Reference to an HCSM's initialization information.
//
// Returns:  Returns a pointer to the newly created Hcsm instance.
//
//////////////////////////////////////////////////////////////////////////////
CHcsm* CHcsmObject::CreateHcsm( string hcsmName, const CSnoBlock& snoBlock )
{

	return m_pRootCollection->CreateHcsm( hcsmName, snoBlock );

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Deletes an Hcsm.
//
// Remarks:  Given a pointer to an Hcsm instance, this function deletes
//   that instance.  It returns a boolean to indicate if the operation
//   was successful.
//
// Arguments:
//   pHcsm - A pointer to an Hcsm instance.
//
// Returns:  A boolean to indicate if the input Hcsm was successfully
//   deleted.
//
//////////////////////////////////////////////////////////////////////////////
bool CHcsmObject::DeleteHcsm( CHcsm* pHcsm )
{

	return m_pRootCollection->DeleteHcsm( pHcsm );

}
