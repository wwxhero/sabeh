/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: objectinitcond.cxx,v 1.12 2004/08/26 20:59:42 schikore Exp $
//
// Author(s):   Paolo Ammann
//
// Date:        October, 1998
//
// Description: The class that manages the initialization parameters of 
//              each object.
//
/////////////////////////////////////////////////////////////////////////////

#include "hcsmpch.h"
#include "objectinitcond.h"
#include <math.h>

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, set evriting to 0 never an object will
// be created.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
CObjectInitCond::CObjectInitCond()
{		
	m_radius = 0; 
	m_activationDelay = 0;
	m_lifetime = 0;
	m_objectState = eEXIT;
	m_startLifetime = 0;	
	m_startActivationTime = 0; 
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function initilize the class.
//
// Remarks: This function will be called once at the beginn to initialize 
//		the class with the parameters that are after used to 
//		determinate in wich state the class is.
//
// Arguments: The actualTime, deltaT is the creation delay, radius is the 
//		creation / deletion radius
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
void 
CObjectInitCond::SetObjectInitCond(
			const CVED::CCved& cCved, 
			const int actualTime, 
			double deltaT, 
			double radius, 
			double activationDelay, 
			double lifetime
			)		
{
	
	m_radius = radius;
	m_activationDelay = activationDelay;
	m_lifetime = lifetime;
	m_objectState = eWAIT;
	m_deltaT = deltaT;
	m_startActivationTime = actualTime; 
	m_cCved = &cCved;
}

/////////////////////////////////////////////////////////////////////////////
// This function returns true if the distance between the object and the 
// ownship is less than specified radius => m_Radius
//
// The Z component of the object and the ownship will be ignored.
/////////////////////////////////////////////////////////////////////////////

bool 
CObjectInitCond::InRadius( const CPoint3D& cObjPos )
{
	bool result;
	const CPoint2D cPos2d(cObjPos.m_x, cObjPos.m_y);
			
	// 
	// If no creation radius is specified ( = 0) the function will always 
	// return true.
	//
	bool noCreationRadius = fabs( m_radius ) < cNEAR_ZERO;
	if( noCreationRadius )
	{
		result = true;
	}
	else
	{
		//
		// Get the OwnVehicle's position and see if it's within my radius.
		//
		CPoint3D ownVehPos( 0, 0, 0 );
		bool foundOwnVeh = m_cCved->GetOwnVehiclePos( ownVehPos );
		if( foundOwnVeh )
		{
			CPoint2D ownVehPos2D( ownVehPos.m_x, ownVehPos.m_y );
			// Calculate the distance between the ownship and the object, ignoring
			// the Z components.
			double distSq = cPos2d.DistSq( ownVehPos );
			
			// If the distance is less then the specified radius it return true
			// otherweise false.
			result = distSq < (m_radius * m_radius);
		}
		else
		{
			result = false;
		}
	}

	return result;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: This function will be called at every HCSM step and will 
//		return the next status of the object
//
// Remarks: This function will be called at every HCSM step and will return 
//		the next status of the object
//
// Arguments: cObjPos is the position of the object, and actualTime is
//		the current time.
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
EInitCond 
CObjectInitCond::Execute( 
		const CPoint3D& cObjPos, 
		const int actualTime
		)
{
	double deltaTime = 0;
	
	// looks the current state of the object
	switch( m_objectState ) 
	{
		case eWAIT: 
			// If the distance is less then the creation radius it 
			// increment the creation delay timer otherweise it set
			// the creation delay timer to 0
			if( InRadius( cObjPos ) )
			{
				// If the creation delay timer is higher than tha 
				// creation delay the object goes to the Create state
				deltaTime = (actualTime - m_startActivationTime) * m_deltaT;
				if( deltaTime > m_activationDelay )
				{
					m_objectState = eACTIVATE;
				}
			}
			else
			{
				m_startActivationTime = actualTime;
			}	
	
			break;

		case eACTIVATE:
			// The object goes from the Creation state to the 
			// Control state
			m_objectState = eUNDER_CONTROL;
			m_startLifetime = actualTime;
			break;

		case eUNDER_CONTROL:
			{
				// If the lifetime expires or the distance between the object 
				// and the ownship is bigger than the specified radius the 
				// object goes to the Delete state. Otherwise it will only 
				// increment the timer.
				
				deltaTime = (actualTime - m_startLifetime) * m_deltaT;
		
				bool inRadius = InRadius( cObjPos );
				bool lifetimeOver = m_lifetime != 0 && deltaTime > m_lifetime;
				bool deleteDriver = !inRadius || lifetimeOver;
				if( deleteDriver )
				{
					bool inRadius = InRadius( cObjPos );

					m_objectState = eDELETE;
				}
			}

			break;

		case eDELETE:
			// The object goes from the Creation state to the Control state
			m_objectState = eEXIT;
			break;

		case eEXIT:
			break;
	}
	
	return m_objectState;
}
	
