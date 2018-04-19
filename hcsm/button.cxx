//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: button.cxx,v 1.9 2013/05/08 15:31:01 IOWA\vhorosewski Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CHcsmBtn class.
//
//////////////////////////////////////////////////////////////////////////////


#include "button.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHcsmBtn::CHcsmBtn( CHcsmCollection *pRootCollection, string name ):
	CHcsmCommunicate( pRootCollection, name )
{
}

CHcsmBtn::CHcsmBtn( const CHcsmBtn& objToCopy ):
	CHcsmCommunicate( objToCopy )
{

	// call the assignment operator
	*this = objToCopy;

}

CHcsmBtn& CHcsmBtn::operator=( const CHcsmBtn& objToCopy )
{

	// check to make sure that object passed in is not me
	if ( this != &objToCopy ) {

		// call parent's assignment operator
		CHcsmCommunicate::operator=( objToCopy );

	}

	return *this;

}

CHcsmBtn::~CHcsmBtn()
{

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the button's name.
//
// Remarks:  This function returns the button's  name.
//
// Arguments:
//
// Returns:  A reference to an STL string that represents the button's name.
//
//////////////////////////////////////////////////////////////////////////////
const string& CHcsmBtn::GetName() const
{

	return m_name;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Presses the button.
//
// Remarks:  This function changes a button's state to pressed.  Once a 
//   button is pressed in a given frame, it only reports itself as being
//   pressed in the following frame.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmBtn::PressButton()
{
	//
	// Check to make sure that the previous request was not in the frame,
	// multiple requests to press the button in the same frame are ignored.
	//
	bool setThisFrame = m_setFrame == GetFrame();
	if( !setThisFrame )
	{
		m_prevSetFrame = m_setFrame;
		m_setFrame = GetFrame();
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Is the button pressed?
//
// Remarks:  This function returns a boolean to indicate when a button is
//   pressed.  Once the user presses the button in a given frame, the button
//   reports itself as being pressed in only the following frame.
//
// Arguments:
//
// Returns:  A boolean to indicate if the button is pressed.
//
//////////////////////////////////////////////////////////////////////////////
bool CHcsmBtn::IsButtonPressed() const
{

	// get the current frame
	int currentFrame = GetFrame();

	// a button is pressed if a request was made to press it in the 
	// frame before the current frame
	bool buttonPressed = ( ( m_setFrame != cINVALID_FRAME && 
							 m_setFrame + 1 == currentFrame 
							 ) ||
						   ( m_prevSetFrame != cINVALID_FRAME &&
						     m_prevSetFrame + 1 == currentFrame
							 )
						   );

	return buttonPressed;

}
