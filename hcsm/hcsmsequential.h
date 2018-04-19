/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: hcsmsequential.h,v 1.11 2014/04/23 19:22:54 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         July, 1998
 *
 * Description:  Interface for the CHcsmSequential class.
 *
 ****************************************************************************/

#ifndef __CHCSMSEQUENTIAL_H
#define __CHCSMSEQUENTIAL_H

#if _WIN32
#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
#endif

#include <string>
using namespace std;

#include <snoblock.h>
#include "hcsm.h"

class CHcsmCollection;

//////////////////////////////////////////////////////////////////////////////
///
/// This class inherits all attributes and operations from its parent Hcsm.
/// The major difference between a sequential and concurrent HCSM is that
/// only one child of a sequential HCSM gets executed each frame as opposed
/// to all children of a concurrent HCSM.  A sequential HCSM contains
/// transitions between its children.  Each transition has a predicate
/// function associated with it that gets executed each frame.  The results
/// of predicate function execution determines which child will be active
/// each frame.
///
/// This is an abstract class and should not be instanced.
///
///\ingroup HCSM
//////////////////////////////////////////////////////////////////////////////
class CHcsmSequential : public CHcsm  
{
public:
	CHcsm* GetActiveChild();
	CHcsm* GetDefaultActiveChild();
	void ResetActiveChild();
	virtual void Execute();

protected:
	CHcsmSequential( CHcsmCollection*, const CSnoBlock*, string, bool = false, int typeId = -1 );
	CHcsmSequential( CHcsmCollection*, string, bool = false, int typeId = -1 );
	CHcsmSequential( const CHcsmSequential& );
	CHcsmSequential& operator=( const CHcsmSequential& );
	virtual ~CHcsmSequential();
	virtual void ExecuteTransitions() = 0;

	CHcsm* m_activeChild;
	CHcsm* m_defaultActiveChild;
};

#endif // __CHCSMSEQUENTIAL_H
