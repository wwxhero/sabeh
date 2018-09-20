//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsmsequential.cxx,v 1.10 2004/04/27 19:20:33 schikore Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CHcsmSequential class.
//
//////////////////////////////////////////////////////////////////////////////


#include <genericinclude.h>
#include <string>
#include <tchar.h>
using namespace std;

#include "hcsmsequential.h"
#include "hcsmcollection.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHcsmSequential::CHcsmSequential(
			CHcsmCollection* pRootCollection,
			const CSnoBlock* cpSnoBlock,
			string hcsmName,
			bool rootState,
			int typeId
			) :
	CHcsm( pRootCollection, cpSnoBlock, hcsmName, rootState, typeId ),
	m_activeChild( NULL ),
	m_defaultActiveChild( NULL )
{

}

CHcsmSequential::CHcsmSequential(
			CHcsmCollection* pRootCollection,
			string hcsmName,
			bool rootState,
			int typeId
			) :
	CHcsm( pRootCollection, NULL, hcsmName, rootState, typeId ),
	m_activeChild( NULL ),
	m_defaultActiveChild( NULL )
{

}

CHcsmSequential::CHcsmSequential( const CHcsmSequential& cObjToCopy ):
	CHcsm(
		cObjToCopy.m_pRootCollection,
		cObjToCopy.m_pSnoBlock,
		cObjToCopy.m_name,
		cObjToCopy.m_root
		)
{
	// call the assignment operator
	*this = cObjToCopy;
}

CHcsmSequential& CHcsmSequential::operator=(
			const CHcsmSequential& cObjToCopy
			)
{
	// check to make sure that object passed in is not me
	if( this != &cObjToCopy )
	{
		// call parent's assignment operator
		CHcsm::operator=( cObjToCopy );

		// make a deep copy
		m_activeChild = cObjToCopy.m_activeChild;
		m_defaultActiveChild = cObjToCopy.m_defaultActiveChild;
	}

	return *this;
}

CHcsmSequential::~CHcsmSequential()
{

}

//////////////////////////////////////////////////////////////////////
// Query
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Return the default active child.
//
// Remarks:  This function returns a pointer to the default active child.
//   It first checks to make sure that the pointer is not a NULL pointer.
//   If the pointer is not NULL then it returns the pointer.
//
// Arguments:
//
// Returns:  A pointer to the default active child.
//
//////////////////////////////////////////////////////////////////////////////
CHcsm* CHcsmSequential::GetDefaultActiveChild()
{
	// error checking
	if( !m_defaultActiveChild )
	{
		gout << MyName() << "::GetDefaultActiveChild: no default active child";
		gout << endl;

		return NULL;
	}

	return m_defaultActiveChild;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Return the active child.
//
// Remarks:  This function returns a pointer to the active child.
//   It first checks to make sure that the pointer is not a NULL pointer.
//   If the pointer is not NULL then it returns the pointer.
//
// Arguments:
//
// Returns:  A pointer to the active child.
//
//////////////////////////////////////////////////////////////////////////////
CHcsm* CHcsmSequential::GetActiveChild()
{
	// error checking
	if( !m_activeChild )
	{
		gout << MyName() << "::GetActiveChild: no active child" << endl;

		return NULL;
	}

	return m_activeChild;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Resets the active child.
//
// Remarks:  This function resets the HCSM instance's active child pointer
//   to point to the default active child.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmSequential::ResetActiveChild()
{
	// error checking
	if ( !m_defaultActiveChild )
	{
		gout << MyName() << "::ResetActiveChild: no default active child";
		gout << endl;

		return;
	}

	m_activeChild = m_defaultActiveChild;
}

//////////////////////////////////////////////////////////////////////
// Execute
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes this HCSM.
//
// Remarks:  This function executes this HCSM.  It does the following in
//   order: call the pre-activity function, execute all transitions until
//   one transition has fired or all have been executed, execute the active
//   child and then call the post-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmSequential::Execute()
{
	try
	{
		//
		// Execute myself and my children only if I'm active.  This assumes
		// that if the parent HCSM is inactive then so are its descendent
		// HCSMs.
		//
		//enum EHcsmState { eBORN, eACTIVE, eINACTIVE, eDYING };
		const TCHAR* infoState[] = { "eBORN", "eACTIVE", "eINACTIVE", "eDYING" };
		//TRACE(TEXT("HcsmSequential:%d %s\n"), this, infoState[m_state]);

		if( m_state == eACTIVE )
		{
			// execute my pre-activity function
			PreActivity( m_pSnoBlock );

			// execute transitions
			if ( m_activeChild->GetState() == eACTIVE )  ExecuteTransitions();

			// check to see if any of my chilren are dead and, if so, mark
			// myself as being dead
			for( int i = 0; i < m_numChildren; i++ )
			{
				if( m_children[i]->GetState() == eDYING )
				{
					if ( m_root )  DeleteHcsm( this );
					m_state = eDYING;
					return;
				}
			}

			// execute active child....do some error checking first
			if( !m_activeChild )
			{
				gout << MyName() << "::Execute: no active child!" << endl;
			}
			else
			{
				m_activeChild->Execute();

				if( m_activeChild->GetState() == eDYING )
				{
					if ( m_root ) DeleteHcsm( this );
					m_state = eDYING;
					return;
				}
			}

			// execute my post-activity function
			PostActivity( m_pSnoBlock );
		}

	}
	catch( CExceptionSuicide s )
	{
		// check to see if I committed suicide
		return;
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes the predicate functions associated with the
//   active child's transitions.
//
// Remarks:  This function executes the predicate functions associated with
//   transitions that originate from the currently active child.  This
//   function is virtual since each derived class must reimplement this
//   function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CHcsmSequential::ExecuteTransitions()
{

}
