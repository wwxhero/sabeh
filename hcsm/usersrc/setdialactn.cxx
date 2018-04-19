/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: setdialactn.cxx,v 1.19 2012/02/10 17:03:48 iowa\dheitbri Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        October, 1999
//
// Description:  Implementation of CSetDialActn class, which sets the 
// 				 dial of the given name.
//
/////////////////////////////////////////////////////////////////////////////

#include "setdialactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"

#define SET_DIAL_DEBUG 0


/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks: The pointer to the root HCSM collection is where the new HCSMs
//			will be deleted.
//
// Arguments:	
//	pBlock - A pointer to the CActionParseBlock associated with	this CActn.
//	pHC - A pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetDialActn::CSetDialActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC,
			bool debug
			) :
			m_debug( debug )
{
	m_pHC = pHC;
	CActionParseBlock::pbTDial dial = cpBlock->GetDial();
	m_dialName = dial.name;
	m_dialValue = dial.value;
	if( m_debug )
	{
		gout << "## dial '" << dial.name << "' has value = ";
		gout << dial.value << endl;
	}
	InitCandidateSet( cpBlock );
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CSetDialActn
//
// Remarks: 
//
// Arguments: CSetDialActn to be copied into current CSetDialActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetDialActn::CSetDialActn( const CSetDialActn& cRhs )
{
	*this = cRhs;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The destructor.
//
// Remarks: 
//
// Arguments: none
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetDialActn::~CSetDialActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CSetDialActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CSetDialActn&
CSetDialActn::operator=( const CSetDialActn& cRhs )
{

	if( this != &cRhs )
	{
		m_pHC            = cRhs.m_pHC;
		m_candidateSet   = cRhs.m_candidateSet;
		m_useInstigators = cRhs.m_useInstigators;
		m_dialName       = cRhs.m_dialName;
	}

	return *this;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Sets the dial for an HCSM given its id.
//
// Remarks:
//
// Arguments: 
//  hcsmId - The id of the HCSM of which to set the dial for.
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CSetDialActn::SetDialOnHcsm( int hcsmId )
{

	CHcsm* pHcsm = m_pHC->GetHcsm( hcsmId );
	if( !pHcsm )
	{
		gout << "CSetDialActn::SetDialOnHcsm: unable to get pointer ";
		gout << "to hcsm id = " << hcsmId << endl;

		return;
	}


	bool result = pHcsm->SetDialByNameStr( m_dialName, m_dialValue );
	if( !result )
	{
		gout << "CSetDialActn::SetDialOnHcsm: unable to set dial";
		gout << endl;

		return;
	}

#if 0
	//
	// Set the dial activities log.
	//
	if( g_logActivities )
	{
		m_pRootCollection->SetDialButSetLog( pHcsm, m_dialName, m_dialValue )
	}
#endif

#if 0
	gout << m_dialName << " was set to " << m_dialValue ;
	gout << " on " << pHcsm->GetName() << endl; 
#endif
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: Finds out if the dial associated with this action has
//  finished its task (inactivated itself).
//
// Remarks:  All HCSMs there were initially activated with this dial
//  must inactivate themselves before this function will return true.
//
// Arguments: 
//
// Returns: A boolean.
//
/////////////////////////////////////////////////////////////////////////////
bool 
CSetDialActn::IsFinished()
{
	//
	// Iterate through all of the HCSMs.  If an HCSM is already active
	// then check to see if it was un-activated during this frame.
	//
	vector<TResetHcsm>::iterator i;
	for( 
		i = m_waitingForResetHcsm.begin();
		i != m_waitingForResetHcsm.end();
		i++
		)
	{
		if( i->isActive )
		{
			CHcsm* pHcsm = m_pHC->GetHcsm( i->hcsmId );
			if( !pHcsm )
			{
				gout << "CSetDialActn::IsFinished: unable to ";
				gout << "get pointer to hcsm id = " << i->hcsmId << endl;

				gout <<"With Dial Name :"<<m_dialName<< " Value "<<m_dialValue<<endl;
				CHcsm* pTrigHcsm  = m_pHC->GetHcsm(m_triggerId);
				if (pTrigHcsm){
					gout<<"Fromt Trigger "<<pTrigHcsm->GetName()<<endl;
					CPoint3D pos = m_pHC->GetCved()->GetObjPos(m_triggerId);
					gout<<"Located at"<<pos.m_x<<" "<<pos.m_y<<endl;
				}
				
				// may as well mark it as having been reset
				i->isActive = false;
				continue;
			}

			i->isActive = pHcsm->IsDialActiveByNameStr( m_dialName );
		}
	}

	//
	// Iterate through all of the HCSMs again.  Return true only if
	// ALL of the HCSMs are no longer active.
	//
	for( 
		i = m_waitingForResetHcsm.begin();
		i != m_waitingForResetHcsm.end();
		i++
		)
	{
		if( i->isActive )  return false;
	}
#if 0
	gout << m_dialName << " is finished" << endl; 
#endif
	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function iterates through the children of its Actn Block
//	and deletes the HCSMs listed there.  If the HCSM specifications are
//	invalid, then an error message is displayed.
//
// Arguments:
//  cpInstigators - The instigators.
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CSetDialActn::Execute( const set<CCandidate>* cpInstigators )
{
	if( m_waitingForResetHcsm.size() > 0 )
	{
		m_waitingForResetHcsm.clear();
	}

	CHcsmCollection::SetActionSetDialLog( m_triggerId, m_dialName, m_dialValue );

	set<CCandidate> candidateSet;
 	m_candidateSet.GetCandidates( *(m_pHC->GetCved()), candidateSet, *m_pHC );

	//
	// If we use the instigator set and the user gave us a set of 
	// instigators, then Execute the action on the instigators that 
	// are in the candidate set.
	//
	bool useInstigators = m_useInstigators && cpInstigators != 0;
	if( useInstigators )
	{
#if SET_DIAL_DEBUG > 1
		gout << "Using instigator set." << endl;
#endif

		set<CCandidate>::const_iterator itr;
		for( 
			itr = cpInstigators->begin(); 
			itr != cpInstigators->end(); 
			itr++ 
			)
		{
			SetDialOnHcsm( itr->m_hcsmId );

			TResetHcsm node;
			node.hcsmId = itr->m_hcsmId;
			node.isActive = true;
			m_waitingForResetHcsm.push_back( node );
		}
	}
	//
	// If we don't use the instigator set, then disregard the 
	// instigators and Execute the action on the candidates.
	//
	else
	{
#if SET_DIAL_DEBUG > 1
		gout << "Not using instigator set." << endl;
#endif

		set<CCandidate>::const_iterator itr;
		for( itr = candidateSet.begin(); itr != candidateSet.end(); itr++ )
		{
			SetDialOnHcsm( itr->m_hcsmId );

			TResetHcsm node;
			node.hcsmId = itr->m_hcsmId;
			node.isActive = true;
			m_waitingForResetHcsm.push_back( node );
		}
	}
}
