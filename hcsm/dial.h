// Dial.h: interface for the CDial class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __CDIAL_H
#define __CDIAL_H

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include "hcsmcommunicate.h"

///////////////////////////////////////////////////////////////////////////////
///
/// This class defines the attributes and operations needed by a dial.  A 
/// dial is a means of communication between Hcsms.  An Hcsm can send a
/// message to another Hcsm by writing values to the latter Hcsm's dial(s).
/// Once a value is written to an Hcsm's dial during a given frame, the dial
/// reports itself as having that value starting on the following frame.
///
/// This class is an abstract and therefore may not be instanced.  It needs
/// to be subclassed with a class which specifies the type of the value held
/// by the dial.
///\ingroup HCSM
///////////////////////////////////////////////////////////////////////////////
class CDial : public CHcsmCommunicate  
{
public:
	void Reset();
	bool HasBeenReset();
	void SetNoValue();

protected:
	CDial( CHcsmCollection*, const string& );
	CDial( const CDial& );
	CDial& operator=( const CDial& );
	virtual ~CDial();

	bool m_hasBeenResetA;
	bool m_hasBeenResetB;

	void SetValue();

private:
	int m_resetFrame;
};

#endif // __CDIAL_H
