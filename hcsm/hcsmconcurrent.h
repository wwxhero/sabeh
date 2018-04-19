// HcsmConcurrent.h: interface for the CHcsmConcurrent class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_HCSMCONCURRENT_H__8CBC5691_4103_11D2_88B6_00104B9BF9C5__INCLUDED_)
#define AFX_HCSMCONCURRENT_H__8CBC5691_4103_11D2_88B6_00104B9BF9C5__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#ifdef _WIN32
#pragma warning( disable : 4786 )  
#endif
#include <string>
using namespace std;

#include <snoblock.h>
#include "hcsm.h"

class CHcsmCollection;

//////////////////////////////////////////////////////////////////////////////
///
/// This class inherits all attributes and operations from its parent Hcsm.
/// This class has been introduced for future flexibility in the HCSM system
/// even though the current structure doesn't necessitate this class.  The
/// major difference between a concurrent and sequential HCSM is that all
/// children of a current HCSM get executed each frame as opposed to only one
/// child of a sequential HCSM.
///
/// This is an abstract class and should not be instanced.
///
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsmConcurrent : public CHcsm  
{
protected:
	CHcsmConcurrent( CHcsmCollection*, const CSnoBlock*, string, bool = false, int typeId = -1 );
	CHcsmConcurrent( CHcsmCollection*, string, bool = false, int typeId = -1 );
	CHcsmConcurrent( const CHcsmConcurrent& );
	CHcsmConcurrent& operator=( const CHcsmConcurrent& );
	virtual ~CHcsmConcurrent();
};

#endif // !defined(AFX_HCSMCONCURRENT_H__8CBC5691_4103_11D2_88B6_00104B9BF9C5__INCLUDED_)
