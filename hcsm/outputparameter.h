// OutputParameter.h: interface for the COutputParameter class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_OUTPUTPARAMETER_H__0A1AA242_4BE9_11D2_88BA_00104B9BF9C5__INCLUDED_)
#define AFX_OUTPUTPARAMETER_H__0A1AA242_4BE9_11D2_88BA_00104B9BF9C5__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include "hcsmstorage.h"

class COutputParameter : public CHcsmStorage  
{
protected:
	COutputParameter( string );
	COutputParameter( const COutputParameter& );
	COutputParameter& operator=( const COutputParameter& );
	virtual ~COutputParameter();
};

#endif // !defined(AFX_OUTPUTPARAMETER_H__0A1AA242_4BE9_11D2_88BA_00104B9BF9C5__INCLUDED_)
