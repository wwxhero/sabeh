/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: hcsmcommunicate.h,v 1.12 2014/04/23 19:22:54 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         July, 1998
 *
 * Description:  Interface for the CHcsmCommunicate class.
 *
 ****************************************************************************/

#ifndef __CHCSMCOMMUNICATE_H
#define __CHCSMCOMMUNICATE_H

#if _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
#endif

#ifdef _WIN32
#pragma warning(disable:4786)  
#endif

#include <string>
using namespace std;

#include "hcsmobject.h"

const int cINVALID_FRAME = -1;

//////////////////////////////////////////////////////////////////////////////
///
/// This class defines the attributes and operations needed by all dial and
/// monitor objects.  Dials and monitors are a means of communication between
/// Hcsms.
///
/// This class is an abstract and therefore may not be instanced.
///
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsmCommunicate : public CHcsmObject
{
public:
	bool HasValue();
	virtual void SetNoValue();
	const string& GetName() const;

protected:
	CHcsmCommunicate( CHcsmCollection*, const string& );
	CHcsmCommunicate( const CHcsmCommunicate& );
	CHcsmCommunicate& operator=( const CHcsmCommunicate& );
	virtual ~CHcsmCommunicate();

	inline const char* MyName();
	string m_name;

	long m_setFrame;
	long m_prevSetFrame;

	bool m_hasValueA;
	bool m_hasValueB;

	void SetValue();
};

#include "hcsmcommunicate.inl"

#endif // __CHCSMCOMMUNICATE_H
