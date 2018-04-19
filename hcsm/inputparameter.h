// InputParameter.h: interface for the CInputParameter class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_INPUTPARAMETER_H__F2227981_4729_11D2_88B9_00104B9BF9C5__INCLUDED_)
#define AFX_INPUTPARAMETER_H__F2227981_4729_11D2_88B9_00104B9BF9C5__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include <string>
using namespace std;
#include "hcsmstorage.h"

class CInputParameter : public CHcsmStorage  
{
protected:
	CInputParameter( string );
	CInputParameter( const CInputParameter& );
	CInputParameter& operator=( const CInputParameter& );
	virtual ~CInputParameter();
};

#endif // !defined(AFX_INPUTPARAMETER_H__F2227981_4729_11D2_88B9_00104B9BF9C5__INCLUDED_)
