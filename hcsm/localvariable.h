// LocalVariable.h: interface for the CLocalVariable class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LOCALVARIABLE_H__2A3982C1_46D6_11D2_88B9_00104B9BF9C5__INCLUDED_)
#define AFX_LOCALVARIABLE_H__2A3982C1_46D6_11D2_88B9_00104B9BF9C5__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include "hcsmstorage.h"

class CLocalVariable : public CHcsmStorage  
{
protected:
	CLocalVariable( string );
	CLocalVariable( const CLocalVariable& );
	CLocalVariable& operator=( const CLocalVariable& );
	virtual ~CLocalVariable();
};

#endif // !defined(AFX_LOCALVARIABLE_H__2A3982C1_46D6_11D2_88B9_00104B9BF9C5__INCLUDED_)
