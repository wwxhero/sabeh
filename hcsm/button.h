/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: button.h,v 1.10 2014/04/23 19:22:54 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         July, 1998
 *
 * Description:  Interface for the CHcsmBtn class.
 *
 ****************************************************************************/

#ifndef __CHCSMBTN_H
#define __CHCSMBTN_H

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#ifdef _WIN32
#pragma warning(disable:4786)  
#endif
#include <string>
using namespace std;

#include "hcsmcommunicate.h"

///////////////////////////////////////////////////////////////////////////////
///
/// This class defines the attributes and operations needed by all button
/// objects.  A button is a means of communication between Hcsms.  The 
/// communication involves "pressing" an Hcsm's button(s) which may cause
/// that Hcsm to do something in response.  A button does not have a value,
/// it only has a state:  pressed or normal (unpressed).  If a button is
/// pressed in any given execution frame, it only reports itself as being
/// pressed in the following frame only.
///
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsmBtn : public CHcsmCommunicate
{
public:
	CHcsmBtn( CHcsmCollection*, string );
	CHcsmBtn( const CHcsmBtn& );
	CHcsmBtn& operator=( const CHcsmBtn& );
	virtual ~CHcsmBtn();
	bool IsButtonPressed() const;
	void PressButton();
	const string& GetName() const;

};

#endif // __CHCSMBTN_H
