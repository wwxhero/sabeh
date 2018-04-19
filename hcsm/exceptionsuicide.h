// ExceptionSuicide.h: interface for the CExceptionSuicide class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_EXCEPTIONSUICIDE_H__38B130A1_5109_11D2_88BB_00104B9BF9C5__INCLUDED_)
#define AFX_EXCEPTIONSUICIDE_H__38B130A1_5109_11D2_88BB_00104B9BF9C5__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

//////////////////////////////////////////////////////////////////////////////
//
// This exception class has been created for situations where an Hcsm throws
// an exception due to committing suicide.  A suicide is not necessarily an 
// error.  It is a mechanism by which an Hcsm can cleanly remove itself from 
// the scenario at any given moment.
//
//////////////////////////////////////////////////////////////////////////////
class CExceptionSuicide  
{
public:
	CExceptionSuicide();
	virtual ~CExceptionSuicide();

};

#endif // !defined(AFX_EXCEPTIONSUICIDE_H__38B130A1_5109_11D2_88BB_00104B9BF9C5__INCLUDED_)
