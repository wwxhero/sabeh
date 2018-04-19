/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: triggerfirecond.cxx,v 1.8 2004/06/25 20:35:17 schikore Exp $
//
// Author(s):   Jillian Vogel
//
// Date:        August, 1999
//
// Description: The definition of the class that manages fire states of a trigger
//
/////////////////////////////////////////////////////////////////////////////

#include "triggerfirecond.h"

#define	FC_STATE_DEBUG	0

#if FC_STATE_DEBUG
	// String equivilents for the EFireCond enums
	string eFireCondStr [] = {"eNOT_INIT", "eEVAL_PRED", 
		"eFIRE_DELAY", "eFIRE", "eDEBOUNCE", "eDEAD"};
#endif

/////////////////////////////////////////////////////////////////////////////
//
// Description: The default constructor, indicates that it hasn't yet been 
//		initialized with initFireCondition(...)
//
/////////////////////////////////////////////////////////////////////////////
CTriggerFireCond::CTriggerFireCond() {

	m_state = eNOT_INIT;
	m_deltaT = 0.0;
	m_fireDelay = 0;
	m_debounce = 0.0;
	m_oneShot = 0;
	m_predValue = false;
	m_startPredTrue = 0;
	m_startDebounce = 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function will be called once at the beginning to 
//		initialize the object
//
/////////////////////////////////////////////////////////////////////////////
void CTriggerFireCond::InitFireCondition (const double deltaT,
										  const int fireDelay,
										  const double debounce,
										  const int oneShot)
{
	m_state = eEVAL_PRED;
	m_deltaT = deltaT;
	m_fireDelay = fireDelay;
	m_debounce = debounce;
	m_oneShot = oneShot;
	m_startPredTrue = 0;
	m_startDebounce = 0;
	m_predValue = false;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function will be called at each time step to indicate
//		the state of the object.
//
/////////////////////////////////////////////////////////////////////////////
EFireCond CTriggerFireCond::Execute(const int actualTime)
{

	double timeDelay;

	#if FC_STATE_DEBUG
		gout << "CTriggerFireCond::Execute:start state = " 
			 << eFireCondStr[m_state] << endl;
	#endif

	switch (m_state) {

		case eNOT_INIT:
			break;

		case eEVAL_PRED:
			DoPredTransition(actualTime);
			break;

		case eFIRE_DELAY:
			timeDelay = (actualTime-m_startPredTrue);
			if (timeDelay > m_fireDelay)
				m_state = eFIRE;
			break;

		case eFIRE:
			if (m_oneShot)
				m_state = eDEAD;
			else {
				if (m_debounce > 0) {
					m_startDebounce = actualTime;
					m_state = eDEBOUNCE;
				}
				else
					m_state = eEVAL_PRED;
			}
			break;

		case eDEBOUNCE:
			timeDelay = (actualTime-m_startDebounce)*m_deltaT;
			if (timeDelay > m_debounce)
				m_state = eEVAL_PRED;
			break;

		case eDEAD:
			break;

	}

	#if FC_STATE_DEBUG
		gout << "CTriggerFireCond::Execute:end state = " 
			 << eFireCondStr[m_state] << endl;
	#endif

	return m_state;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function will return the current state of the object,
//		without executing the next step in the cycle.
//
/////////////////////////////////////////////////////////////////////////////
EFireCond CTriggerFireCond::GetState() 
{
	return m_state;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function sets the local m_predValue variable to the
//		value in the parameter.
//
// Remarks: This is how the Trigger communicates to this class what the 
//		current predicate condition is.
//
/////////////////////////////////////////////////////////////////////////////
void CTriggerFireCond::SetPredicate(bool val) 
{
	m_predValue = val;
}


void CTriggerFireCond::DoPredTransition(const int actualTime) {

	// Protected method to prevent repetition in the code.
	//	Checks the timing constriants to determine which state
	//	the object should be in eFIRE.  If the fire delay is 0,  
	//	then the eFIRE_DELAY state is skipped.  Similarly, if  
	//	the evaluation function is true, the eEVAL_PRED state 
	//	is skipped.
	if (m_predValue) {
		if (m_fireDelay > 0) {
			m_startPredTrue = actualTime;
			m_state = eFIRE_DELAY;
		}
		else
			m_state = eFIRE;
	}
	else
		m_state = eEVAL_PRED;
}
