//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: hcsm.cxx,v 1.45 2016/08/22 14:32:16 IOWA\dheitbri Exp $
//
// Author(s):    Omar Ahmad
//
// Date:         July, 1998
//
// Description:  Implemention of the CHcsm class.
//
//////////////////////////////////////////////////////////////////////////////


#include <genericinclude.h>
#include <string>
#include <exception>
using namespace std;

#include "hcsm.h"
#include "hcsmcollection.h"

// MemLog constants
#define HLOG_CALL_CREAT_START       11000
#define HLOG_CALL_CREATE_DOONE_1    11001
#define HLOG_CALL_CREATE_DOONE_2    11002
#define HLOG_CALL_CREAT_END         11003

#define  HLOG_EXEC_START	        11010
#define  HLOG_EXEC_PREACT_1         11011
#define  HLOG_EXEC_PREACT_2         11012
#define  HLOG_EXEC_CHDRN_1          11013
#define  HLOG_EXEC_CHDRN_2          11014
#define  HLOG_EXEC_CHDRN_DIE_1      11015
#define  HLOG_EXEC_CHDRN_DIE_2      11016
#define  HLOG_EXEC_POST_1           11017
#define  HLOG_EXEC_POST_2           11018
#define  HLOG_EXEC_END              11019
#define  HLOG_EXEC_END_1            11020
#define  HLOG_EXEC_END_2            11021
#define  HLOG_EXEC_END_3            11022
#define  HLOG_EXEC_END_4            11023

#define  HLOG_SUIC                  11030

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHcsm::CHcsm(
			CHcsmCollection* pRootCollection, 
			const CSnoBlock* cpSnoBlock, 
			string name, 
			bool rootState,
			int typeId
			):
	CHcsmObject( pRootCollection ), 
	m_name( name ), 
	m_root( rootState ), 
	m_state( eBORN ), 
	m_numChildren( 0 ),
	m_pSnoBlock( cpSnoBlock ),
	m_executionTime( 0 ),
    m_priorityLevel(0),
	m_debugMode( eDEBUG_NONE ),
	m_debugLevel( CHcsmDebugItem::eDEBUG_NORMAL ),
	m_typeId( typeId )
{
	int i;
	for( i = 0; i < cMAX_CHILDREN; i++ ) 
	{
		m_children[i] = NULL;
	}
}

CHcsm::CHcsm( const CHcsm& cObjToCopy ):
	CHcsmObject( cObjToCopy.m_pRootCollection )
{
	// call the assignment operator
	*this = cObjToCopy;
}

CHcsm& 
CHcsm::operator=( const CHcsm& cObjToCopy )
{
	// check to make sure that object passed in is not me
	if( this != &cObjToCopy ) 
	{
		// call parent's assignment operator
		CHcsmObject::operator=( cObjToCopy );

		// make a deep copy
		m_name = cObjToCopy.m_name;
		m_root = cObjToCopy.m_root;
		m_state = cObjToCopy.m_state;
		m_typeId = cObjToCopy.m_typeId;

		int i;
		for( i = 0; i < cObjToCopy.m_numChildren; i++ ) 
		{
			// THIS IS NOT A DEEP COPY
			m_children[i] = cObjToCopy.m_children[i];
			m_numChildren++;
		}

		m_pSnoBlock = cObjToCopy.m_pSnoBlock;
		m_executionTime = cObjToCopy.m_executionTime;
		m_debugLevel = cObjToCopy.m_debugLevel;
		m_debugMode = cObjToCopy.m_debugMode;
        m_priorityLevel = cObjToCopy.m_priorityLevel;
	}

	return *this;
}

CHcsm::~CHcsm()
{
	// delete all children
	int i;
	for( i = 0; i < m_numChildren; i++ ) 
	{
		delete( m_children[i] );
	}

	// free the memory used by the sno block
	if ( m_pSnoBlock ){
        delete m_pSnoBlock;
        m_pSnoBlock = nullptr;
    }
}

//////////////////////////////////////////////////////////////////////
// Query functions
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets an HCSM descendents' state.
//
// Remarks:  This function sets the state of all descendents of an HCSM to
//   the given value.
//
// Arguments:
//   cState - An enumeration:
//             eACTIVE   --> set hcsm state to active
//             eINACTIVE --> set hcsm state to inactive
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::SetStateDescendents( const EHcsmState cState )
{
	int i;
	for( i = 0; i < m_numChildren; i++ ) 
	{
		m_children[i]->SetStateTree( cState );
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets an HCSM instance's and it descendent's state.
//
// Remarks:  This function accepts a value that sets an HCSM instance's
//   state and all of the instance's descendent HCSMs.  An HCSM can either 
//   be active or inactive.
//
// Arguments:
//   cState - An enumeration:
//             eACTIVE   --> set hcsm state to active
//             eINACTIVE --> set hcsm state to inactive
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::SetStateTree( const EHcsmState cState )
{
	SetState( cState );
	SetStateDescendents( cState );
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Adds a child Hcsm to an Hcsm's collection of children.
//
// Remarks:  This function accepts a pointer to a child Hcsm and adds
//   that pointer to an Hcsm's collection of children.
//
// Arguments:
//   pHcsm - A pointer to a child Hcsm.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::AddChild( CHcsm* pHcsm )
{
	// error checking
	if( m_numChildren == cMAX_CHILDREN ) 
	{
		gout << MyName() << "::AddChild: exceeded array bounds for ";
		gout << "maximum children" << endl;

		return;
	}

	// add a pointer to the child in the children array
	// and increment the child counter
	m_children[m_numChildren] = pHcsm;
	m_numChildren++;
}

//////////////////////////////////////////////////////////////////////////////
// Creation, Deletion, Activity and Execution
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  An Hcsm's creation function.
//
// Remarks:  This function is an Hcsm instance's creation function.  A 
//   creation function is a function that is automatically executed when an 
//   Hcsm is instanced.
//
//   This function is virtual so that if the derived class
//   redefines this function then the derived class' creation function is
//   executed instead of this one.  This function is being defined in this 
//   abstract base class so that each derived HCSM is not forced to define 
//   this function.
//
// Arguments:
//   cpBaseSnoBlock - Pointer to the base sno block.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::Creation( const CSnoBlock* cpBaseSnoBlock )
{
	if( m_name == "" ) 
	{
		gout << MyName() << "::Creation: invalid name!" << endl;
		// Suicide()
		return;
	}
}
void 
CHcsm::Creation()
{
	if( m_name == "" ) 
	{
		gout << MyName() << "::Creation: invalid name!" << endl;
		// Suicide()
		return;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  An Hcsm's deletion function.
//
// Remarks:  This function is an Hcsm instance's deletion function.  A 
//   deletion function is a function that is automatically executed when an 
//   Hcsm is deleted.
//
//   This function is virtual so that if the derived class
//   redefines this function then the derived class' deletion function is
//   executed instead of this one.  This function is being defined in this 
//   abstract base class so that each derived Hcsm is not forced to define 
//   this function.
//
// Arguments:
//   cpBaseSnoBlock - Pointer to the base sno block.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::Deletion( const CSnoBlock* cpBaseSnoBlock )
{
#if 0
	gout << "DELETING " << m_name << endl;
#endif
}

void 
CHcsm::Deletion()
{
#if 0
	gout << "DELETING " << m_name << endl;
#endif
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  An Hcsm's pre-activity function.
//
// Remarks:  This function is an Hcsm instance's pre-activity function.  A 
//   pre-activity function is automatically executed as the whole HCSM tree
//   is being traversed during execution.  The pre-activity function is
//   executed when an Hcsm is encountered for the first time during a 
//   pre-order traversal.  Therefore, a parent Hcsm's pre-activity
//   function is executed before any of children's pre-activity function.
//
//   This function is virtual so that if the derived class
//   redefines this function then the derived class' pre-activity function is
//   executed instead of this one.  This function is being defined in this 
//   abstract base class so that each child Hcsm is not forced to define 
//   this function.
//
// Arguments:
//   cpBaseSnoBlock - Pointer to the base sno block.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::PreActivity( const CSnoBlock* cpBaseSnoBlock )
{
//	DebugText(CHcsmDebugItem::eDEBUG_ROUTINE, "Default PreActivity");
}

void 
CHcsm::PreActivity()
{
//	DebugText(CHcsmDebugItem::eDEBUG_ROUTINE, "Default PreActivity");
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  An Hcsm's post-activity function.
//
// Remarks:  This function is an Hcsm instance's post-activity function.  A 
//   post-activity function is automatically executed as the whole HCSM tree
//   is being traversed during execution.  The post-activity function is
//   executed when an Hcsm is encountered for the last time during a 
//   pre-order traversal.  Therefore, a parent Hcsm's post-activity
//   function is executed after any of children's post-activity function.
//
//   This function is virtual so that if the derived class
//   redefines this function then the derived class' post-activity function is
//   executed instead of this one.  This function is being defined in this 
//   abstract base class so that each child Hcsm is not forced to define 
//   this function.
//
// Arguments:
//   cpBaseSnoBlock - Pointer to the base sno block.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::PostActivity( const CSnoBlock* cpBaseSnoBlock )
{
//	DebugText(CHcsmDebugItem::eDEBUG_ROUTINE, "Default PostActivity");
}

void 
CHcsm::PostActivity()
{
//	DebugText(CHcsmDebugItem::eDEBUG_ROUTINE, "Default PostActivity");
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calls the HCSM's creation function, and the creation functions
//	for its children.
//
// Remarks:  This function calls an HCSM's creation function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsm::CallCreation( void )
{
	int id = m_pRootCollection->GetHcsmId( this );

	m_pRootCollection->MemLog( id, HLOG_CALL_CREAT_START, MyName() );

	try 
	{
		if( m_pSnoBlock ) 
		{
			m_pRootCollection->MemLog( id, HLOG_CALL_CREATE_DOONE_1, 0 );
			Creation( m_pSnoBlock );
			m_pRootCollection->MemLog( id, HLOG_CALL_CREATE_DOONE_2, 0 );
		}
		else 
		{
			m_pRootCollection->MemLog( id, HLOG_CALL_CREATE_DOONE_1, 0 );
			Creation();
			m_pRootCollection->MemLog( id, HLOG_CALL_CREATE_DOONE_2, 0 );
		}

	}
	catch( CExceptionSuicide s ) 
	{
		// check to see if I committed suicide
		gout << MyName() << ": " << m_name << " committed suicide while creating." << endl;
		return;
	}
	catch( cvCInternalError s ) 
	{
		// caught CVED exception
		s.Notify();
		gout << MyName() << ": caught CVED cvCInternalError while creating ";
		gout << m_name << " HCSM" << endl;
		return;
	}
	catch( cvCError s ) 
	{
		// caught CVED exception
		s.Notify();
		gout << MyName() << ": caught CVED exception while creating ";
		gout << m_name << " HCSM" << endl;
		return;
	}
	catch( ... ) 
	{
		// caught unknown exception
		gout << MyName() << ": caught unknown exception while creating ";
		gout << m_name << " HCSM" << endl;
		return;
	}

	//
	// Recursively call CallCreation functions of all the descendents.
	//
	int i;
	for( i = 0; i < m_numChildren; i++ ) 
	{
		m_children[i]->CallCreation();
	}

	m_pRootCollection->MemLog( id, HLOG_CALL_CREAT_END, 0 );
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes an Hcsm.
//
// Remarks:  This function executes an Hcsm instance.  Executing an Hcsm
//   involves doing the following in order: call the pre-activity function, 
//   execute all active children and then call the post-activity function.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::Execute()
{
	m_pRootCollection->MemLog( 999, HLOG_EXEC_START, MyName() );

	int id = m_pRootCollection->GetHcsmId( this );

	m_pRootCollection->MemLog( id, HLOG_EXEC_START, MyName() );

	try 
	{
		//
		// Execute myself and my children only if I'm active.  This assumes
		// that if the parent HCSM is inactive then so are its descendent
		// HCSMs.
		//
		if( m_state == eACTIVE ) 
		{
			// execute my pre-activity function
			if( m_pSnoBlock ) 
			{
				m_pRootCollection->MemLog( id, HLOG_EXEC_PREACT_1, 0 );
				PreActivity( m_pSnoBlock );
				m_pRootCollection->MemLog( id, HLOG_EXEC_PREACT_2, 0 );
			}
			else
			{
				m_pRootCollection->MemLog( id, HLOG_EXEC_PREACT_1, 0 );
				PreActivity();
				m_pRootCollection->MemLog( id, HLOG_EXEC_PREACT_2, 0 );
			}

			// execute all my children
			int i;
			for( i = 0; i < m_numChildren; i++ ) 
			{
				m_pRootCollection->MemLog( id, HLOG_EXEC_CHDRN_1, 0 );
				m_children[i]->Execute();
				m_pRootCollection->MemLog( id, HLOG_EXEC_CHDRN_2, 0 );

				if( m_children[i]->GetState() == eDYING ) 
				{	
					if ( m_root )
					{
						m_pRootCollection->MemLog( id, HLOG_EXEC_CHDRN_DIE_1, 0 );
						DeleteHcsm( this );
						m_pRootCollection->MemLog( id, HLOG_EXEC_CHDRN_DIE_2, 0 );
					}

					m_state = eDYING;
					return;
				}
			}

			// execute my post-activity function
			if( m_pSnoBlock ) 
			{
				m_pRootCollection->MemLog( id, HLOG_EXEC_POST_1, 0 );
				PostActivity( m_pSnoBlock );
				m_pRootCollection->MemLog( id, HLOG_EXEC_POST_2, 0 );
			}
			else 
			{
				m_pRootCollection->MemLog( id, HLOG_EXEC_POST_1, 0 );
				PostActivity();
				m_pRootCollection->MemLog( id, HLOG_EXEC_POST_2, 0 );
			}
		}
	}
	catch( CExceptionSuicide s ) 
	{	
		// check to see if I committed suicide
		m_pRootCollection->MemLog( id, HLOG_EXEC_END_1, 0 );
		m_pRootCollection->MemLog( id, HLOG_EXEC_END, 0 );
		return;
	}
	catch( cvCError s ) 
	{
		// caught CVED exception
		s.Notify();
		gout << GetFrame() << ":";
		gout << MyName() << ": caught CVED exception while executing ";
		gout << m_name << " HCSM" << endl;
		m_pRootCollection->MemLog( id, HLOG_EXEC_END_2, 0 );
		m_pRootCollection->MemLog( id, HLOG_EXEC_END, 0 );
		return;
	}
	catch( exception e ) 
	{
		gout << GetFrame() << ":";
		gout << MyName() << ": caught  exception " << e.what() << endl;
		gout << m_name << " HCSM" << endl;
	}
	catch( ... ) 
	{
		// caught unknown exception
		gout << GetFrame() << ":";
		gout << MyName() << ": caught unknown exception while executing ";
		gout << m_name << " HCSM" << endl;
		m_pRootCollection->MemLog( id, HLOG_EXEC_END_3, 0 );
		m_pRootCollection->MemLog( id, HLOG_EXEC_END, 0 );
		return;
	}
	m_pRootCollection->MemLog( id, HLOG_EXEC_END_4, 0 );
	m_pRootCollection->MemLog( id, HLOG_EXEC_END, 0 );
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes an Hcsm's predicate function.
//
// Remarks:  This function executes an Hcsm's predicate function.  The
//   input argument specifies which predicate function the Hcsm should
//   execute.  Only Hcsms whose parent is a sequential Hcsm have predicate
//   functions and they should override this function in their class
//   definition.  For all other Hcsm's, this virtual function will get
//   called and return value will be false.
//
// Arguments:
//   functionId - An integer that specifies which predicate function to
//     execute.
//
// Returns:  A boolean which represents the result of executing the predicate
//   function.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CHcsm::ExecutePredicate( int functionId )
{
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Kills the Hcsm including all its ancestors and descendents.
//
// Remarks:  This function deletes the Hcsm from which it called including
//   the Hcsm's ancestors and descendents.  This function can be called from
//   any Hcsm in an Hcsm tree.  The HCSM system ceases execution of the Hcsm 
//   tree from which this function is called.  The Hcsm tree is deleted at 
//   the end of the current frame.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::Suicide()
{
	m_pRootCollection->MemLog( 
				m_pRootCollection->GetHcsmId( this), 
				HLOG_SUIC, 
				MyName()
				);

	// mark all my children for deletion
	int i;
	for( i = 0; i < m_numChildren; i++ ) 
	{
		if( m_children[i]->GetState() != eDYING ) 
		{	
			m_children[i]->Suicide();
		}
	}

	// inform the Hcsm collection if I am the root
	if( m_root )
	{		
		DeleteHcsm( this );

		//
		// Throw an exception only if this is the root HCSM....if a
		// child HCSM throws an exception then the ancestors never 
		// properly get deleted.
		//
		CExceptionSuicide exception;
		throw exception;
	}
	else 
	{
		m_state = eDYING;

		//
		// DO NOT throw an exception....only the root HCSM gets to 
		// throw the exception.
		// 
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Presses a button on an HCSM.
//
// Remarks:  This function presses a button on an HCSM.  The input string
//   names the button that should be pressed.  This function is a virtual
//   function and therefore should be implemented by all HCSMs that contain
//   buttons.
//
// Arguments:
//   cButtonName - A string that indicates the button's name.
//
// Returns:  A boolean that indicates whether the operation was successful.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CHcsm::SetButtonByName( const string& cButtonName )
{

	// If the user uses the HCSM system properly, this function should
	// never be called because a sub-class should reimplement this function
	// if it has buttons.  If this function gets called, then the user is
	// most likely trying to set the button for an HCSM that has no buttons
	// defined inside it or has forgotten to define this function inside the
	// class that represents the HCSM.
	gout << MyName() << "::SetButtonByName: this HCSM has no button ";
	gout << "named " << cButtonName << endl;

	return false;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns a output message prefix.
//
// Remarks:  This function creates a string that can be used as the prefix
//   for an output message by an HCSM.  The string contains the current
//   frame and the HCSM's id enclosed in square brackets.
//
// Arguments:
//
// Returns:  An string with the message prefix.
//
//////////////////////////////////////////////////////////////////////////////
string
CHcsm::MessagePrefix()
{
	char prefixBuf[64];
	sprintf_s( 
		prefixBuf, "[%d:%d:%s] ", 
		GetFrame(), 
		m_pRootCollection->GetHcsmId( this ),
		GetName().c_str()
		);

	string prefix = prefixBuf;
	return prefix;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns a output message prefix.
//
// Remarks:  This function creates a string that can be used as the prefix
//   for an output message by an HCSM.  The string contains the current
//   frame and the given id enclosed in square brackets.
//
// Arguments:
//   id - The number to be written to the output after the frame.
//
// Returns:  An string with the message prefix.
//
//////////////////////////////////////////////////////////////////////////////
string
CHcsm::MessagePrefix( int id )
{
	char prefixBuf[64];
	sprintf_s( prefixBuf, "[%d:%d:%s] ", GetFrame(), id, GetName().c_str() );

	string prefix = prefixBuf;
	return prefix;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns a output message prefix.
//
// Remarks:  This function creates a string that can be used as the prefix
//   for an output message by an HCSM.  The string contains the current
//   frame and the given id and name enclosed in square brackets.
//
// Arguments:
//   id - The number to be written to the output after the frame.
//   cName - The name to be written to the output after the id.
//
// Returns:  An string with the message prefix.
//
//////////////////////////////////////////////////////////////////////////////
string
CHcsm::MessagePrefix( int id, const string& cName )
{
	char prefixBuf[64];
	sprintf_s( prefixBuf, "[%d:%d:%s] ", GetFrame(), id, cName.c_str() );

	string prefix = prefixBuf;
	return prefix;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Prints a creation message to the standard output.
//
// Remarks:  This function prints a message to the standard output
//   saying that the HCSM has been created.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsm::PrintCreationMessage()
{
#if 0
	gout << MessagePrefix() << "CREATING a " << m_name << endl;
#endif
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Prints a deletion message to the standard output.
//
// Remarks:  This function prints a message to the standard output
//   saying that the HCSM has been deleted.
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsm::PrintDeletionMessage()
{
#if 0
	gout << MessagePrefix() << "DELETING a " << m_name << endl;
#endif
}


//////////////////////////////////////////////////////////////////////////////
// Debugging
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Log a debugging text message.
//
// Remarks:  This function logs the specified message if the current
//  debugging mode includes strings, and if the importance of the
//  message is at least as high as the current HCSM's level.  Debugging
//  messages have to be retrieved by a client program for display, they
//  don't automatically appear anywhere.
//
// Arguments:
//  level - the importance of the message; one of eDEBUG_ROUTINE,
//          eDEBUG_NORMAL, or eDEBUG_URGENT, all within the CHcsmDebugItem 
//			namespace.
//  msg - the message to print
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::DebugText( CHcsmDebugItem::ELevel level, const string& msg )
{
	if( m_debugMode == eDEBUG_TEXT_AND_GRAPHICS || m_debugMode == eDEBUG_TEXT ) 
	{
		if( m_debugLevel <= level ) 
		{
			CHcsmDebugItem  item;

			item.SetLevel( level );
			item.SetHcsmId( m_pRootCollection->GetHcsmId( this ) );
			item.SetFrame( GetFrame() );
			item.SetText( msg );
			m_pRootCollection->LogDebugItem( item );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Draw a line to help debugging the HCMS.
//
// Remarks:  This function logs the specified graphic if the current
//  debugging mode includes graphics, and if the importance of the
//  message is at least as high as the current HCSM's level.  Debugging
//  graphics have to be retrieved by a client program for display, they
//  don't automatically appear anywhere.
//
// Arguments:
//  level - the importance of the message; one of eDEBUG_ROUTINE,
//          eDEBUG_NORMAL, or eDEBUG_URGENT, all within the CHcsmDebugItem 
//			namespace.
//  p1    - first end point of the line
//  p2    - second endpoint of the line
//  color - the color to use when drawing the line
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::DebugLine(
			CHcsmDebugItem::ELevel  level, 
			const CPoint3D&         p1,
			const CPoint3D&         p2,
			CHcsmDebugItem::EColor  color,
			bool					arrow
			)
{
	if( m_debugMode == eDEBUG_TEXT_AND_GRAPHICS || m_debugMode == eDEBUG_GRAPHICS ) 
	{
		if( m_debugLevel <= level ) 
		{
			CHcsmDebugItem  item;

			item.SetLevel( level );
			item.SetHcsmId( m_pRootCollection->GetHcsmId( this ) );
			item.SetFrame( GetFrame() );
			item.SetLine( p1.m_x, p1.m_y, p2.m_x, p2.m_y, color, arrow );
			m_pRootCollection->LogDebugItem( item );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Draw text to help debugging the HCMS.
//
// Remarks:  This function logs the specified graphic if the current
//  debugging mode includes graphics, and if the importance of the
//  message is at least as high as the current HCSM's level.  Debugging
//  graphics have to be retrieved by a client program for display, they
//  don't automatically appear anywhere.
//
// Arguments:
//  level   - the importance of the message; one of eDEBUG_ROUTINE,
//            eDEBUG_NORMAL, or eDEBUG_URGENT, all within the CHcsmDebugItem 
//			  namespace.
//  str     - the string to display
//  cvedObj - the cved object that the string will be displayed relative to
//  color   - the color to use when drawing the line
//  align   - where the text will be relative to the cvedObj's bounding
//            box; one of eTOP_LEFT, eTOP_RIGHT, eBOTTOM_LEFT, eBOTTOM_RIGHT,
//            all defined in the CHcsmDebugItem namespace.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CHcsm::DebugGraphicalText(
			CHcsmDebugItem::ELevel level, 
			const string& str,
			const CObj& cvedObj,
			CHcsmDebugItem::EColor color, 
			CHcsmDebugItem::EAlign align
			)
{
	if( m_debugMode == eDEBUG_TEXT_AND_GRAPHICS || m_debugMode == eDEBUG_GRAPHICS ) 
	{
		if( m_debugLevel <= level ) 
		{
			CHcsmDebugItem  item;

			item.SetLevel( level );
			item.SetHcsmId( m_pRootCollection->GetHcsmId( this ) );
			item.SetFrame( GetFrame() );

			// get the cved item's bounding box;
			CBoundingBox box = cvedObj.GetBoundBox();
			double x, y;
			bool balign;
			switch ( align ) {
			default:
			case CHcsmDebugItem::eTOP_LEFT:
				x = box.GetMinX();
				y = box.GetMaxY();
				balign = true;
				break;

			case CHcsmDebugItem::eTOP_RIGHT:
				x = box.GetMaxX();
				y = box.GetMaxY();
				balign = false;
				break;

			case CHcsmDebugItem::eBOTTOM_LEFT:
				x = box.GetMinX();
				y = box.GetMinY();
				balign = true;
				break;

			case CHcsmDebugItem::eBOTTOM_RIGHT:
				x = box.GetMaxX();
				y = box.GetMinY();
				balign = false;
				break;
			}

			item.SetGraphicalText( str, x, y, color, balign );

			m_pRootCollection->LogDebugItem( item );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Draw a rectangle to help debugging the HCMS.
//
// Remarks:  This function logs the specified graphic if the current
//  debugging mode includes graphics, and if the importance of the
//  message is at least as high as the current HCSM's level.  Debugging
//  graphics have to be retrieved by a client program for display, they
//  don't automatically appear anywhere.
//
// Arguments:
//  level - the importance of the message; one of eDEBUG_ROUTINE,
//          eDEBUG_NORMAL, or eDEBUG_URGENT, all within the CHcsmDebugItem 
//			namespace.
//  p1    - lower left corner of the rectangle
//  p2    - upper right corner of the rectangle
//  color - the color to use when drawing the rectangle
//  filled - if true, the rectangle will be filled with the color
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::DebugRect(
			CHcsmDebugItem::ELevel  level, 
			const CPoint3D&         p1, 
			const CPoint3D&         p2, 
			CHcsmDebugItem::EColor  color, 
			bool                    filled
			)
{
	if( m_debugMode == eDEBUG_TEXT_AND_GRAPHICS || m_debugMode == eDEBUG_GRAPHICS ) 
	{
		if( m_debugLevel <= level ) 
		{	
			CHcsmDebugItem  item;

			item.SetLevel( level );
			item.SetHcsmId( m_pRootCollection->GetHcsmId( this ) );
			item.SetFrame( GetFrame() );
			item.SetRect( p1.m_x, p1.m_y, p2.m_x, p2.m_y, color, filled );
			m_pRootCollection->LogDebugItem( item );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Draw a circle to help debugging the HCMS.
//
// Remarks:  This function logs the specified graphic if the current
//  debugging mode includes graphics, and if the importance of the
//  message is at least as high as the current HCSM's level.  Debugging
//  graphics have to be retrieved by a client program for display, they
//  don't automatically appear anywhere.
//
// Arguments:
//  level - the importance of the message; one of eDEBUG_ROUTINE,
//          eDEBUG_NORMAL, or eDEBUG_URGENT, all within the CHcsmDebugItem 
//			namespace.
//  center - center of the circle
//  radius - the circle's radius
//  color - the color to use when drawing the circle
//  filled - if true, the circle will be filled with the color
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CHcsm::DebugCircle(
			CHcsmDebugItem::ELevel level,
			const CPoint3D&        cCenter, 
			double                 radius,
			CHcsmDebugItem::EColor color, 
			bool                   filled
			)
{
	if( m_debugMode == eDEBUG_TEXT_AND_GRAPHICS || m_debugMode == eDEBUG_GRAPHICS ) 
	{
		if( m_debugLevel <= level ) 
		{
			CHcsmDebugItem  item;

			item.SetLevel( level );
			item.SetHcsmId( m_pRootCollection->GetHcsmId( this ) );
			item.SetFrame( GetFrame() );
			item.SetCircle( cCenter.m_x, cCenter.m_y, radius, color, filled );
			m_pRootCollection->LogDebugItem( item );
		}
	}
}

void CHcsm::DebugTriggerFire( CHcsmDebugItem::ELevel level, int hcsmId )
{
	if( m_debugLevel <= level ) 
	{
		CHcsmDebugItem item;
		item.SetTriggerFire( hcsmId );

		m_pRootCollection->LogDebugItem( item );
	}
}


