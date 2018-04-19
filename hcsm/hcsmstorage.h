/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: hcsmstorage.h,v 1.8 2014/04/23 19:22:54 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         July, 1998
 *
 * Description:  Interface for the CHcsmStorage class.
 *
 ****************************************************************************/

#ifndef __CHCSMSTORAGE_H
#define __CHCSMSTORAGE_H

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include <string>
using namespace std;

//////////////////////////////////////////////////////////////////////////////
///
/// This class defines the attributes and operations needed by all input
/// parameter, local variable and output parameter objects.  Input and output
/// parameters are a means of transfering information between parent and child
/// Hcsms.  Local variables provide Hcsms with a mechanism for storing
/// information between execution frames.
///
/// This class is an abstract and therefore may not be instanced.
///
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsmStorage  
{
public:
	inline const string& GetName() const;
	inline bool HasValue();
	inline void SetNoValue();

protected:
	CHcsmStorage( string );
	CHcsmStorage( const CHcsmStorage& );
	CHcsmStorage& operator=( const CHcsmStorage& );
	virtual ~CHcsmStorage();

	string m_name;
	inline const char* MyName();
	bool m_hasValue;
};

#include "hcsmstorage.inl"

#endif // __CHCSMSTORAGE_H
