// HcsmObject.h: interface for the CHcsmObject class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __CHCSMOBJECT_H
#define __CHCSMOBJECT_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786)
#endif
#include <genericinclude.h>
#include <string>
using namespace std;

#include <snoblock.h>
#include <cved.h>

using namespace CVED;

class CHcsmCollection;
class CHcsm;

//////////////////////////////////////////////////////////////////////////////
///
/// This class sits at the top of the HCSM class hierarchy.  It contains a
/// pointer to an instance of the root collection class that keeps track of
/// all root Hcsms.  This allows all Hcsm objects to get access to the current
/// frame number and the ability to create and delete other Hcsms.
///
/// This is an abstract class and should not be instanced.
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsmObject
{
protected:
	CHcsmObject( CHcsmCollection* );
	CHcsmObject( const CHcsmObject& );
	CHcsmObject& operator=( const CHcsmObject& );
	virtual ~CHcsmObject();
	int GetFrame() const;
	double GetTimeStepDuration() const;
	CHcsm* CreateHcsm( string, const CSnoBlock& );
	bool DeleteHcsm( CHcsm* );

	CHcsmCollection* m_pRootCollection;
	CCved* cved;
};

#endif // __CHCSMOBJECT_H
