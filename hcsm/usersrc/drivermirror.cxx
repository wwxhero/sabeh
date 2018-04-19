/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: drivermirror.cxx,v 1.14 2014/01/16 15:55:55 iowa\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    May, 2001
 *
 * Description:  Contains code for the DriverMirror HCSM.
 *
 ****************************************************************************/

#include "genericinclude.h"
#include "genhcsm.h"
#include "hcsmcollection.h"

#include <point3d.h>

#include <pi_iostream>
using namespace std;

#undef DEBUG_DM

void 
CDriverMirror::UserCreation()
{
	PrintCreationMessage();
	m_prevRoadPosFrame = -1;
	m_prevTrailerPosFrame = -1;
	m_registeredHcsmId = false;
}  // end of UserCreation


void 
CDriverMirror::UserPreActivity()
{

#ifdef DEBUG_DM
	gout << MessagePrefix();
	gout << "=== DRIVER MIRROR=================" << endl;
#endif

	//
	// Check to see if the OwnVehicle is alive and valid.
	//
	bool validObj = cved->IsObjValid( 0 );
	if( !validObj )
	{
#ifdef DEBUG_DM
		gout << "  invalid obj 0...exit" << endl;
		return;
#endif
	}

	//
	// Register the HCSM id if needed.  Make sure we're not running a fake
	// external driver.
	//
	if( !m_registeredHcsmId && !cved->HaveFakeExternalDriver() ) 
	{
		cved->SetExternalDriverHcsmId( m_pRootCollection->GetHcsmId( this ) );
	}

	//
	// Look at the OwnVehicle's current geometric position and convert
	// it into a RoadPos.
	//
	CPoint3D objPos = cved->GetObjPosInstant( 0 );

#ifdef DEBUG_DM
	gout << "  obj pos = " << objPos << endl;
#endif

	bool buildRoadPosFromScratch = (
				m_prevRoadPosFrame < 0 ||
				GetFrame() - m_prevRoadPosFrame > 1
				);
	if( buildRoadPosFromScratch )
	{
		CRoadPos roadPos( *cved, objPos );

#ifdef DEBUG_DM
		gout << "  before obj pos = " << objPos << "  valid roadPos = " <<	roadPos << endl;
#endif
		// update corridor if on intersection to make sure it's the same as the path if a path exists
		bool updateCrdr = !roadPos.IsRoad() && CHcsmCollection::m_sOwnshipPath.IsValid();
		if( updateCrdr )
		{
			int crdrId = -1;
			bool success = CHcsmCollection::m_sOwnshipPath.GetCrdrFromIntrscn( 
						roadPos.GetIntrsctn().GetId(), 
						crdrId,
						&roadPos
						);
			if( success )
			{
				// if the path says that we should be on a different corridor than what the 
				// current roadPos has, then we need to reprioritize the current roadPos so 
				// that the correct corridor has the priority
				CCrdr currCrdr( roadPos.GetIntrsctn(), crdrId );
				success = roadPos.SetCrdPriority( &currCrdr );
			}
		}

		m_currRoadPos = roadPos;
	}
	else
	{
		if( m_prevTargCrdr.IsValid() )
		{
			m_currRoadPos.SetXYZ( objPos, &m_prevTargCrdr );
		}
		else
		{
			m_currRoadPos.SetXYZ( objPos );
		}
	}

	//
	// Set the RoadPos monitor.
	//
	if( m_currRoadPos.IsValid() )
	{
#ifdef DEBUG_DM
		gout << "  obj pos = " << objPos << "  valid roadPos = " << m_currRoadPos << endl;
#endif
		SetMonitorRoadPos( m_currRoadPos );
	}
	else
	{
#ifdef DEBUG_DM
		gout << "  obj pos = " << objPos << "  invalid roadPos" << endl;
#endif
		CRoadPos emptyRoadPos;
		SetMonitorRoadPos( emptyRoadPos );
	}

	//
	// Set the TargCrdr monitor.
	//
	if( m_currRoadPos.IsValid() )
	{
		CCrdr targCrdr;
		bool haveCrdr = ComputeTargCrdr( targCrdr );
		if( haveCrdr )
		{
			SetMonitorTargCrdr( targCrdr );
			m_prevTargCrdr = targCrdr;
		}
		else
		{
			SetMonitorTargCrdrNoValue();
		}
	}
	else
	{
		SetMonitorTargCrdrNoValue();
	}

	SetMonitorImTargCrdrsNoValue();

	//
	// Write road pos info to previous.
	//
	if( m_currRoadPos.IsValid() )
	{
		m_prevRoadPos = m_currRoadPos;
		m_prevRoadPosFrame = GetFrame();
	}

	if ( cved->IsObjValid( 1 ) ) // check for external trailer
	{
		CPoint3D trailerPos = cved->GetObjPosInstant( 1 ); // external trailer position
		//trailerPos.m_z = objPos.m_z;  // hack, trailer position is too high
		bool buildTrailerPosFromScratch = (
					m_prevTrailerPosFrame < 0 ||
					GetFrame() - m_prevTrailerPosFrame > 1
					);
		if( buildTrailerPosFromScratch )
		{
			CRoadPos trailerRoadPos( *cved, trailerPos );
			m_currTrailerPos = trailerRoadPos;
		}
		else
		{
			m_currTrailerPos.SetXYZ( trailerPos );
		}

		//
		// Set the RoadPos monitor.
		//
		if( m_currTrailerPos.IsValid() )
		{
#ifdef DEBUG_DM
			gout << "  trailer pos = " << trailerPos << "  valid trailerRoadPos = " << m_currTrailerPos << endl;
#endif
			SetMonitorTrailerPos( m_currTrailerPos );
		}
		else
		{
#ifdef DEBUG_DM
			gout << "  trailer pos = " << trailerPos << "  invalid trailerRoadPos" << endl;
#endif
			CRoadPos emptyRoadPos;
			SetMonitorTrailerPos( emptyRoadPos );
		}

		if( m_currTrailerPos.IsValid() )
			m_prevTrailerPosFrame = GetFrame();
	}
	else
	{
#ifdef DEBUG_DM
		gout << "  external trailer not present" << endl;
#endif
	}

	//
	// Look at intersections along my path and register myself with the
	// ones that fall within the register range.
	//
    if( m_currRoadPos.IsValid() && CHcsmCollection::m_sOwnshipPath.IsValid() )
	{
		const double cINTRSCTN_REGISTER_RANGE = 500.0;  // ft.
		vector<int> intrsctns;
		CHcsmCollection::m_sOwnshipPath.GetIntrsctnsWithinDist(
							m_currRoadPos,
							cINTRSCTN_REGISTER_RANGE,
							intrsctns,
							true,
							true
							);

		//
		// Eliminate registered intersections in the signaled list that are no
		// longer on the new list.
		//
		// Register new intersections.
		//
		vector<int>::iterator newItr;
		deque<int>::iterator oldItr = m_signaledIntrsctns.begin();
		for( 
			newItr = intrsctns.begin(); 
			newItr != intrsctns.end(); 
			newItr++ 
			)
		{ 
			//
			// Eliminate dummy intersections.
			//
#ifdef DEBUG_DM
			gout << "already have these [size = ";
			gout << m_signaledIntrsctns.size() << "]:" << endl;
			deque<int>::iterator i;
			for( 
				i = m_signaledIntrsctns.begin(); 
				i != m_signaledIntrsctns.end(); 
				i++ 
				)
			{
				CIntrsctn intrsctn( *cved, *i );
				gout << "  " << intrsctn.GetName() << endl;
			}

			CIntrsctn intrsctn( *cved, *newItr );
			gout << "checking intersection " << intrsctn.GetName();
			gout << endl;
#endif		

			while( oldItr != m_signaledIntrsctns.end() && *oldItr != *newItr )
			{
#ifdef DEBUG_DM
				CIntrsctn intrsctn( *cved, *oldItr );
				gout << "  removing intrsctn " << intrsctn.GetName();
				gout << endl;
#endif
			
				m_signaledIntrsctns.pop_front();
				oldItr = m_signaledIntrsctns.begin();
			}

			bool alreadyRegistered = (
				oldItr != m_signaledIntrsctns.end() &&
				*oldItr == *newItr
				);
			if( !alreadyRegistered )
			{
				m_pRootCollection->ImRegister( 0, *newItr );
				m_signaledIntrsctns.push_back( *newItr );
				oldItr = m_signaledIntrsctns.end();

#ifdef DEBUG_DM
				CIntrsctn intrsctn( *cved, *newItr );
				gout << "  registering with intrsctn " << intrsctn.GetName();
				gout << "  size = " << m_signaledIntrsctns.size();
				gout << endl;
#endif
			}
			else
			{
				if( oldItr != m_signaledIntrsctns.end() ) oldItr++;
			}

			//
			// NOTE:  This needs to be removed later.
			//
			// For now, only support registration with one intersection.
			//
			break;
		}
	}
}  // end of UserPreActivity


void 
CDriverMirror::UserPostActivity()
{
	//
	// Look at the intersections that I'm registered with and write my
	// target corridors thru each of those intersections to my output.
	//
	deque<int>::iterator i;
	bool first = true;
	stringstream imTargCrdrs;
	if (!m_currRoadPos.IsValid())
		return;
	for( i = m_signaledIntrsctns.begin(); i != m_signaledIntrsctns.end(); i++ )
	{
		int crdrId;
		bool success = false;
		if( m_currRoadPos.IsRoad() )
		{
			//This needs to be changed so we follow our current lane to the intersection, but this 
			//version of the CVED call does not, it will only look to see if the imediate lane is connected to 
			//the intersection that is passed in.
			success = CHcsmCollection::m_sOwnshipPath.GetCrdrFromIntrscn( 
								*i,
								crdrId,
								&m_currRoadPos,
								m_currRoadPos.GetLane().GetId()
								);
			//we may be two+ intersections out, if so our target lane will not match,
			//if we are that far out we will assume we can change lanes......
			if( !success ) 
			{
				success = CHcsmCollection::m_sOwnshipPath.GetCrdrFromIntrscn( 
									*i,
									crdrId,
									&m_currRoadPos
									);
			}
		}
		else
		{
			success = CHcsmCollection::m_sOwnshipPath.GetCrdrFromIntrscn( 
								*i,
								crdrId,
								&m_currRoadPos
								);		
		}

		if( success )
		{
			if( first )
			{
				imTargCrdrs<<*i<<" "<<crdrId;
				first = false;
			}
			else
			{
				imTargCrdrs<<" "<<*i<<" "<<crdrId;
			}
		}
	}

	if( imTargCrdrs.str().size() > 0 )
	{
//		gout << imTargCrdrs.str() << endl;
		SetMonitorImTargCrdrs( imTargCrdrs.str() );
	}
	else
	{
		SetMonitorImTargCrdrsNoValue();
	}
}  // end of UserPostActivity


void
CDriverMirror::UserDeletion()
{
	PrintDeletionMessage();
}  // end of UserDeletion


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Computes the OwnVehicle's target corridor thru the
//   current or upcoming intersection.
//
// Remarks:  
//
// Arguments:
//   cCrdr - (output) The computed target corridor.
//
// Returns:  A boolean to indicate if this function is successfully
//   able to compute the target corridor.
//
//////////////////////////////////////////////////////////////////////////////
bool
CDriverMirror::ComputeTargCrdr( CCrdr& targCrdr )
{
#ifdef DEBUG_DM
	gout << "computing targ crdr...";
#endif
	
	if( m_currRoadPos.IsRoad() )
	{
#ifdef DEBUG_DM
		gout << "on a road" << endl;
#endif

		//
		// On a road.
		//
		CLane currLane = m_currRoadPos.GetLane();
		CIntrsctn intrsctn = currLane.GetNextIntrsctn();
		TCrdrVec possibleCrdrs;
		intrsctn.GetCrdrsStartingFrom( currLane, possibleCrdrs );

		if( possibleCrdrs.size() <= 0 )
		{
#ifdef DEBUG_DM
			gout << "  NO targCrdr" << endl;
#endif
			
			return false;
		}
		else if( possibleCrdrs.size() == 1 )
		{
			TCrdrVec::iterator i = possibleCrdrs.begin();
			targCrdr = *i;

#ifdef DEBUG_DM
			gout << "  targCrdr = " << targCrdr.GetRelativeId() << endl;
#endif

			return true;
		}
		else
		{
			//
			// More than one corridor.
			//
			// First, figure out if the OwnVehicle has turned on it's left
			// or right turn signal.
			//
			TU16b lightState = cved->GetVehicleVisualState( 0 );

			bool isLeftTurnSignalOn  = lightState & cCV_LEFT_TURN_SIGNAL;
			bool isRightTurnSignalOn = lightState & cCV_RIGHT_TURN_SIGNAL;

#ifdef DEBUG_DM
			gout << "  leftTurnSignalOn  = " << isLeftTurnSignalOn << endl;
			gout << "  rightTurnSignalOn = " << isRightTurnSignalOn << endl;
#endif

			CCrdr::ECrdrDirection ownVehicleDir;
			if( isLeftTurnSignalOn )
			{
				ownVehicleDir = CCrdr::eLEFT;
			}
			else if( isRightTurnSignalOn )
			{
				ownVehicleDir = CCrdr::eRIGHT;
			}
			else
			{
				ownVehicleDir = CCrdr::eSTRAIGHT;
			}

#ifdef DEBUG_DM
			gout << "  ownVehicle is planning to go to direction = ";
			gout << ownVehicleDir << endl;
#endif

			//
			// Insert into resolve crdr the crdrs that need to be picked between
			// as more than one corridor can quality for each direction.
			//
			TCrdrVec::iterator i;
			TCrdrVec resolveCrdrs;
			for( i = possibleCrdrs.begin(); i != possibleCrdrs.end(); i++ )
			{
				CCrdr crdr = *i;
				CCrdr::ECrdrDirection crdrDir = crdr.GetCrdrDirection();

				if( crdrDir == ownVehicleDir )
				{
					resolveCrdrs.push_back( *i );
				}
			}

			if( resolveCrdrs.size() == 1 )
			{
				TCrdrVec::iterator itr = resolveCrdrs.begin();
				targCrdr = *itr;
#ifdef DEBUG_DM
				gout << "  **picked only available crdr = " << itr->GetRelativeId() << endl;
#endif
				return true;
			}
			else if( resolveCrdrs.size() > 1 )
			{
				//
				// We have more than one corridor that qualifies for 
				// the direction we need.  Need to decide which one to 
				// pick.
				//
				TCrdrVec::iterator itr;
				TCrdrVec::iterator bestCrdrItr;
				bool firstTime = true;
				for( itr = resolveCrdrs.begin(); itr != resolveCrdrs.end(); itr++ )
				{
#ifdef DEBUG_DM
					gout << "  looking at crdr = " << itr->GetRelativeId() << endl;
#endif
					if( firstTime )
					{
						bestCrdrItr = itr;
						firstTime = false;
					}
					else
					{
						double currK = itr->GetCrdrDirectionK();
						double bestK = bestCrdrItr->GetCrdrDirectionK();
#ifdef DEBUG_DM
						gout << "    currK = " << currK << "  bestKSoFar = " << bestK << endl;
#endif
						if( ownVehicleDir == CCrdr::eLEFT )
						{
							if( currK < bestK )  bestCrdrItr = itr;
						}
						else if( ownVehicleDir == CCrdr::eRIGHT )
						{
							if( currK > bestK )  bestCrdrItr = itr;
						}
						else
						{
							if( fabs( currK ) < fabs( bestK ) )  bestCrdrItr = itr;
						}
					}
				}

				targCrdr = *bestCrdrItr;
#ifdef DEBUG_DM
				gout << "  **picked best crdr = " << bestCrdrItr->GetRelativeId() << endl;
#endif

				return true;
			}

			//
			// If we've made it this far then it's not obvious where the
			// own vehicle is going.
			//
			for( i = possibleCrdrs.begin(); i != possibleCrdrs.end(); i++ )
			{
				CCrdr crdr = *i;
				CCrdr::ECrdrDirection crdrDir = crdr.GetCrdrDirection();

				if( crdrDir == CCrdr::eLEFT )
				{
					if( !isLeftTurnSignalOn && !isRightTurnSignalOn )
					{
						targCrdr = crdr;
#ifdef DEBUG_DM
						gout << "  no signal, turning left...";
						gout << "targCrdr = " << targCrdr.GetRelativeId();
						gout << endl;
#endif
						return true;
					}
				}
				else if( crdrDir == CCrdr::eRIGHT )
				{
					if( !isLeftTurnSignalOn && !isRightTurnSignalOn )
					{
						targCrdr = crdr;
#ifdef DEBUG_DM
						gout << "  no signal, turning right...";
						gout << "targCrdr = " << targCrdr.GetRelativeId();
						gout << endl;
#endif
						return true;
					}
				}
			}

			//
			// If we've made it this far then simply pick the first
			// corridor on the list.
			//
			i = possibleCrdrs.begin();
			targCrdr = *i;

#ifdef DEBUG_DM
			gout << "  picking at random...";
			gout << "targCrdr = " << targCrdr.GetRelativeId();
			gout << endl;
#endif

			return true;

		}  // end -- more than one corridor

		
#ifdef DEBUG_DM
		gout << "  NO targCrdr" << endl;
#endif
		return false;
	}
	else
	{
		//
		// On an intersection, simply return the current corridor.
		//
		targCrdr = m_currRoadPos.GetCorridor();
#ifdef DEBUG_DM
		gout << "on an intersection" << endl;
		gout << "  targCrdr = ";
		gout << targCrdr.GetRelativeId() << endl;
#endif
		return true;
	}
}  // end of ComputeTargCrdr
