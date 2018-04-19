/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: hcsm.h,v 1.32 2014/04/23 19:22:54 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         July, 1998
 *
 * Description:  Interface for the CHcsm class.
 *
 ****************************************************************************/

#ifndef __CHCSM_H
#define __CHCSM_H

#ifdef _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#pragma warning(disable:4786) 
#include <string>
#elif __sgi
#include <string.h>
#elif _PowerMAXOS
#include <string>
#endif

using namespace std;

#include <snoblock.h>
#include "hcsmobject.h"
#include "exceptionsuicide.h"
#include "debugitem.h"

enum EHcsmState { eBORN, eACTIVE, eINACTIVE, eDYING };
enum EDebugMode { 
			eDEBUG_NONE, 
			eDEBUG_TEXT, 
			eDEBUG_GRAPHICS, 
			eDEBUG_TEXT_AND_GRAPHICS
			};
const int cMAX_CHILDREN = 10;

class CHcsmCollection;

typedef struct TSequentialDial
{
	string dialName;
	string dialVal;
} TSequentialDial;
///////////////////////////////////////////////////////////////////////////////
///\defgroup HCSM HCSM - Highlevel Concurent State Machine 
///		
//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///\brief
///     This is the base class for HCSM objects
///\remark
/// This class defines the attributes and operations needed by all HCSMs
/// An Hcsm instance can contain other Hcsm instances which are 
/// its children.  These children can also contain children and so on.
/// Thus, the Hcsm instance hierarchy can be viewed as a tree and the Hcsm
/// at the top of the tree is called the root Hcsm.  Root Hcsms have certain
/// attributes and operations which differentiate them from other Hcsms.
///
/// This is an abstract class and should not be instanced.
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsm : public CHcsmObject  
{
public:
	virtual ~CHcsm();
	inline const string& GetName() const;
	inline EHcsmState GetState();
	inline void SetState( const EHcsmState );
	void SetStateTree( const EHcsmState );
	void SetStateDescendents( const EHcsmState );
	inline void AddToExecutionTime( const double );
	inline double GetExecutionTime();
	inline bool IsRoot();
	inline int NumChildren();
    inline int GetPriorityLevel();
	virtual void Execute();
	virtual bool ExecutePredicate( int );
	virtual bool SetButtonByName( const string& );
	void CallCreation();

	inline void SetDebugLevel( CHcsmDebugItem::ELevel );
	inline CHcsmDebugItem::ELevel GetDebugLevel(void) const;

	inline void SetDebugMode( EDebugMode );
	inline EDebugMode GetDebugMode(void) const;
	inline const char* MyName();

	#include "hcsmdialmonitor.inl"

protected:
	CHcsm( CHcsmCollection*, const CSnoBlock*, string, bool = false, int typeId = -1 );
	CHcsm( const CHcsm& );
	CHcsm& operator=( const CHcsm& );
	virtual void Creation();
	virtual void Creation( const CSnoBlock* );
	virtual void Deletion();
	virtual void Deletion( const CSnoBlock* );
	virtual void PreActivity();
	virtual void PreActivity( const CSnoBlock* );
	virtual void PostActivity();
	virtual void PostActivity( const CSnoBlock* );
	void Suicide();
	void AddChild( CHcsm* );
	string MessagePrefix();
	string MessagePrefix( int id );
	string MessagePrefix( int id, const string& cName );
	void PrintCreationMessage();
	void PrintDeletionMessage();
	virtual void DebugText( CHcsmDebugItem::ELevel, const string& );
	virtual void DebugLine( 
				CHcsmDebugItem::ELevel, 
				const CPoint3D&,
				const CPoint3D&,
				CHcsmDebugItem::EColor color = CHcsmDebugItem::eWHITE,
				bool arrow = false
				);
	virtual void DebugRect(
				CHcsmDebugItem::ELevel, 
				const CPoint3D&, 
				const CPoint3D &, 
				CHcsmDebugItem::EColor color = CHcsmDebugItem::eWHITE, 
				bool filled = false
				);
	virtual void DebugCircle(
				CHcsmDebugItem::ELevel, 
				const CPoint3D&, 
				double,
				CHcsmDebugItem::EColor color = CHcsmDebugItem::eWHITE, 
				bool filled = false
				);
	virtual void DebugGraphicalText(
				CHcsmDebugItem::ELevel, 
				const string&,
				const CObj&,
				CHcsmDebugItem::EColor color = CHcsmDebugItem::eWHITE, 
				CHcsmDebugItem::EAlign align = CHcsmDebugItem::eTOP_LEFT
				);

	virtual void DebugTriggerFire(
				CHcsmDebugItem::ELevel,
				int hcsmId
				);

	string m_name;
	EHcsmState m_state;
	bool m_root;
	CHcsm* m_children[cMAX_CHILDREN];
	int m_numChildren;
	const CSnoBlock* m_pSnoBlock;
	EDebugMode m_debugMode;
	CHcsmDebugItem::ELevel m_debugLevel;
	int	m_typeId;
    int m_priorityLevel; 

private:
	double m_executionTime;
};

#include "hcsm.inl"

#endif // __CHCSM_H
