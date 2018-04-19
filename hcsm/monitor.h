// Monitor.h: interface for the CMonitor class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __CMONITOR_H
#define __CMONITOR_H

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include "hcsmcommunicate.h"

///////////////////////////////////////////////////////////////////////////////
///
/// This class defines the attributes and operations needed by a monitor.  A 
/// monitor is a means of communication between Hcsms.  An Hcsm can write a
/// value to its monitor which can be read by other Hcsms.  Once an Hcsm
/// writes a value to its monitor in a given frame, the monitor reports
/// itself as having that value starting on the following frame.
///
/// This class is an abstract and therefore may not be instanced.  It needs
/// to be subclassed with a class which specifies the type of the value held
/// by the monitor.
///
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CMonitor : public CHcsmCommunicate  
{
public:
	CMonitor( CHcsmCollection*, string );
	CMonitor( const CMonitor& );
	CMonitor& operator=( const CMonitor& );
	virtual ~CMonitor();

	void SetValue();
};

#endif // __CMONITOR_H