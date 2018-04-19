/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: objectinitcond.h,v 1.7 2004/01/05 15:44:14 oahmad Exp $
//
// Author(s):   Paolo Ammann
//
// Date:        October, 1998
//
// Description: The definition of the class that manages the initialization 
//              parameters of each object.
//
/////////////////////////////////////////////////////////////////////////////


#ifndef _OBJECTINITCOND_H_
#define _OBJECTINITCOND_H_  

#include <point3d.h>
#include <cved.h>

// The different states that an instance of the class CObjectInitCond 
// can be in.
enum EInitCond { eWAIT, eACTIVATE, eUNDER_CONTROL, eDELETE, eEXIT };


/////////////////////////////////////////////////////////////////////////////
//
// The class that manages the initialization parameters of each object. 
// This parameters are the creation radius, the creation delay and the
// lifetime.
//
/////////////////////////////////////////////////////////////////////////////
class CObjectInitCond 
{
public:
	CObjectInitCond();
	void SetObjectInitCond(
			const CVED::CCved& cved, 
			const int actualTime,
			double deltaT, 
			double radius, 
			double activationDelay, 
			double lifetime
			);
	// This function will be called at every HCSM step and will return the
	// next status of the object.
	EInitCond Execute( const CPoint3D &objectPosition, const int actualTime );
	
private: 
	bool InRadius( const CPoint3D& pos );

	int m_startLifetime;        // The start time for the life time
	int m_startActivationTime;  // The start time for the activation 
	double m_radius;            // Min. distance between the object and the truck
	double m_activationDelay;   // Activation delay of the object
	double m_lifetime;          // How long will the object be alife
	double m_deltaT;            // Time constant for each step
	const CVED::CCved *m_cCved; // The reference to cved
	EInitCond m_objectState;    // The actual state of the object			
};


#endif // _OBJECTINITCOND_H_
