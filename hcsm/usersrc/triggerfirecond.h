/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: triggerfirecond.h,v 1.7 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        August, 1999
//
// Description: The definition of the class that manages fire states of a trigger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _FIRE_CONDITION_H_
#define _FIRE_CONDITION_H_  

#include <string>
using namespace std;

#ifdef _WIN32
#include <ostream>
#include <iostream>
#elif __sgi				// {secret}
#include <iostream.h>
#elif _PowerMAXOS
#include <iostream>
#endif

// The different states that an instance of the class CTriggerFireCond 
// can be in.  They occur under the following conditions:
//	eNOT_INIT:		Default initial condition.  Indicates that the object has
//					not been initialized.
//	eEVAL_PRED:		Indicates that the predicate should be evaluated by the 
//					Trigger and its result passed to setPredicate(bool).
//	eFIRE_DELAY:	Indicates that predicate was evaluated to true, and that
//					the object is waiting for the fire delay to elapse.
//	eFIRE:			Indicates that the fire delay has elapsed, and the trigger
//					is free to fire.  If the trigger has only one shot, then
//					the next state is eDEAD.  Otherwise, it's eDEBOUNCE.
//	eDEBOUNCE:		Occurs immediately after eFIRE.  Indicates that the object 
//					is waiting for the debounce period to elapse.  Cycles back
//					to eEVAL_PRED once debounce is over. 
//	eDEAD:			Indicates the the trigger has already fired its one shot.
enum EFireCond { eNOT_INIT, eEVAL_PRED, eFIRE_DELAY, eFIRE, eDEBOUNCE, eDEAD };

/////////////////////////////////////////////////////////////////////////////
//
// The class that manages the initialization parameters of each object. 
// This parameters are the creation radius, the creation delay and the
// lifetime.
//
/////////////////////////////////////////////////////////////////////////////
class CTriggerFireCond {

	public:
		// Default constructor.  Object must be initialized to begin the cycle.
		CTriggerFireCond ();

		// This function will be called once at the beginning to initialize 
		// the object
		void InitFireCondition (const double deltaT,
								const int fireDelay,
								const double debounce,
								const int oneShot);
		
		// This function will be called at every HCSM step and will return the
		// next state of the object
		EFireCond Execute(const int actualTime);

		// Returns the current state of the object, without executing the next
		//	step in the cycle.
		EFireCond GetState();

		// Sets the local m_predValue variable to the parameter.
		void SetPredicate(bool val);

	protected: 
		EFireCond	m_state;
		double		m_deltaT;
		int         m_fireDelay;
		double		m_debounce;
		int			m_oneShot;
		bool		m_predValue;
		int			m_startPredTrue;
		int			m_startDebounce;

		void		DoPredTransition(const int actualTime);
};

#endif //	_FIRE_CONDITION_H_  
