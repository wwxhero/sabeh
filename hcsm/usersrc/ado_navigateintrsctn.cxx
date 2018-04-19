/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: ado_navigateintrsctn.cxx,v 1.25 2015/09/03 15:22:22 IOWA\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    October, 2001
 *
 * Description:  Contains code for the NavigteIntrsctn HCSM.
 *
 ****************************************************************************/

#include "hcsmpch.h"
#include "controllers.h"

#include <pi_iostream>
using namespace std;

//
// Debugging macros.
//
#undef	DEBUG_NI //2
#undef	DEBUG_NI2 //3

//
// Constants.
//
const double cSTOP_DIST_FROM_HOLD_OFFSET = 2.0; // ft.
const double cDIST_TO_STOPSIGN = 5.0; // ft


void 
CNavigateIntrsctn::Creation()
{

	m_madeStopSignCheck = false;
	m_stopSignCrdr = false;
	m_stopSignHldOfsDist = 0.0;
	m_madeTheNeededStop = false;
	m_tempStartFrames = 0;
	m_firstFrame = true;
    m_StoppedAtStopSignFrame = -1;
	
}  // end of Creation


void
CNavigateIntrsctn::ResetStopSignVars()
{
	m_madeStopSignCheck = false;
	m_stopSignCrdr = false;
	m_stopSignHldOfsDist = 0.0;
	m_madeTheNeededStop = false;
	m_firstFrame = true;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  The NavigateIntrsctn HCSM pre-activity function.  In this
//   function, the ADO registers itself with any upcoming intersections'
//   intersection managers within a given range.
//
// Remarks:  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CNavigateIntrsctn::PreActivity()
{

	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();

#ifdef DEBUG_NI
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_NI;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== NAVIGATEINTRSCTN PRE-ACTIVITY ==============" << endl;
	}
#endif

	const double cINTRSCTN_REGISTER_RANGE = 500.0;  // ft.

#ifdef DEBUG_NI
	if( debugThisObj )
	{
		gout << "registered with these intersections already [size = ";
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
	}
#endif
	
	//
	// Check to see if I'm headed towards a corridor with a stop sign.
	//
	double distToNextIntrsctn = pI->m_pPath->GetDistToNextIntrsctn( 
														pI->m_roadPos 
														);
	bool closeToIntrsctn = ( 
				distToNextIntrsctn > 0.0 &&
				distToNextIntrsctn < cINTRSCTN_REGISTER_RANGE
				);
	if( closeToIntrsctn && !m_madeStopSignCheck )
	{
		CCrdr nextCrdr = pI->m_pPath->GetNextCrdr( pI->m_roadPos, false );
		if( !nextCrdr.IsValid() )  return;

		vector<CHldOfs> hldOfsList;
		nextCrdr.GetAllHldOfs( hldOfsList );

		m_stopSignCrdr = false;

		vector<CHldOfs>::iterator i;
		for( i = hldOfsList.begin(); i != hldOfsList.end(); i++ )
		{
			vector<CHldOfs> hldOfsList;
			nextCrdr.GetAllHldOfs( hldOfsList );

			m_stopSignCrdr = false;

			vector<CHldOfs>::iterator i;
			for( i = hldOfsList.begin(); i != hldOfsList.end(); i++ )
			{
				CHldOfs hldOfs = *i;
				
				if( !hldOfs.IsValid() )  continue;
				
				int hldOfsId = hldOfs.GetObjId();
				cvEObjType objType = cved->GetObjType( hldOfsId );

				// Only interested in traffic sign holdoffs.
				if( objType == eCV_TRAFFIC_SIGN )
				{
					//now lets get the name of the sign;
                    string signName = hldOfs.GetName();
                    int id;
                    if (cved->GetObj(signName,id)){
                        int solId = cved->GetObjSolId(id);
                        int solOption;
                        if (cved->GetObjOption(id,solOption)){
                            vector<CSolOption> options = cved->GetSol().GetObj(solId)->GetOptions();
                            if ((unsigned int) solOption < options.size() && solOption >= 0 ){
                                string optionName = options[solOption].name;
                                std::transform(optionName.begin(), optionName.end(), optionName.begin(), ::tolower);
                                if (optionName == "off"){
                                   m_stopSignCrdr = false; 
                                }else{
                                    m_stopSignCrdr = true;
                                }
                            }else{
                                m_stopSignCrdr = true;
                            }
                        }else{
                            m_stopSignCrdr = true;
                        }
                    }else{
                        m_stopSignCrdr = true;
                    }
                    
                    m_stopSignHldOfsDist = hldOfs.GetDistance();
					

#ifdef DEBUG_NI
					if( debugThisObj )
					{
						gout << "need to stop at stop sign...hold ofs dist = ";
						gout << m_stopSignHldOfsDist << endl;
					}
#endif
				}

				//
				// NOTE:
				//
				// Stopping after the first hold offset...this needs to be
				// changed.
				//
				break;
			}

			m_madeStopSignCheck = true;
		}
	}

#ifdef DEBUG_NI
	if( debugThisObj )
	{
		gout << "stop sign crdr = " << m_stopSignCrdr << endl;
		gout << "made the needed stop = " << m_madeTheNeededStop << endl;
	}
#endif

	if( m_stopSignCrdr && !m_madeTheNeededStop )  
	{
		double distToHldOfs;
		if( pI->m_roadPos.IsRoad() )
		{
			if( pI->m_roadPos.GetLane().GetDirection() == ePOS )
			{
				distToHldOfs = (
					m_stopSignHldOfsDist + 
					pI->m_roadPos.GetRoad().GetLinearLength() - 
					pI->m_roadPos.GetDistance()
					);
			}
			else
			{
				distToHldOfs = m_stopSignHldOfsDist + pI->m_roadPos.GetDistance();
			}
		}
		else
		{
			distToHldOfs = m_stopSignHldOfsDist - pI->m_roadPos.GetDistance();
		}

		double neededDistFromHldOfs = (
					cSTOP_DIST_FROM_HOLD_OFFSET + 
					1.0 + 
					( pI->m_objLength * 0.5 )
					);
		m_madeTheNeededStop = ( 
					pI->m_pObj->GetVelImm() < 0.2 &&
					distToHldOfs < neededDistFromHldOfs
					);
        if (m_madeTheNeededStop)
            m_StoppedAtStopSignFrame = GetFrame();
#ifdef DEBUG_NI
		if( debugThisObj )
		{
			gout << "dist to hold offset = " << distToHldOfs << " ft" << endl;
			gout << "needed dist for hold offset = " << neededDistFromHldOfs;
			gout << " ft" << endl;
			gout << "vel = " << pI->m_pObj->GetVelImm() * cMS_TO_MPH;
			gout << " mph" << endl;
			gout << "madeTheNeededStop = " << m_madeTheNeededStop << endl;
		}
#endif

		if( !m_madeTheNeededStop )
		{
#ifdef DEBUG_NI
			if( debugThisObj )
			{
				gout << "still need to stop at stop sign" << endl;
			}
#endif

			return;
		}
	}else{
        if (!m_stopSignCrdr)
            m_StoppedAtStopSignFrame = -1;
        else if (m_StoppedAtStopSignFrame> 0){
            if (!pI->m_roadPos.IsRoad()){
                double distPastHldOfs = pI->m_roadPos.GetDistance() > m_stopSignHldOfsDist;
                if (distPastHldOfs > pI->m_objLength){
                    m_StoppedAtStopSignFrame = -1;
                }
            }else{
                m_StoppedAtStopSignFrame = -1;
            }
        }
    }

	//
	// Look at intersections along my path and register myself with the
	// ones that fall within the register range.
	//
	vector<int> intrsctns;
	pI->m_pPath->GetIntrsctnsWithinDist(
						pI->m_roadPos,
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
#ifdef DEBUG_NI
		if( debugThisObj )
		{
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
		}
#endif		

		while( oldItr != m_signaledIntrsctns.end() && *oldItr != *newItr )
		{
#ifdef DEBUG_NI
			if( debugThisObj )
			{
				CIntrsctn intrsctn( *cved, *oldItr );
				gout << "  removing intrsctn " << intrsctn.GetName();
				gout << endl;
			}
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
			m_pRootCollection->ImRegister( pI->m_pObj->GetId(), *newItr );
			m_signaledIntrsctns.push_back( *newItr );
			oldItr = m_signaledIntrsctns.end();

#ifdef DEBUG_NI
			if( debugThisObj )
			{
				CIntrsctn intrsctn( *cved, *newItr );
				gout << "  registering with intrsctn " << intrsctn.GetName();
				gout << "  size = " << m_signaledIntrsctns.size();
				gout << endl;
			}
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

}  // end of PreActivity


//////////////////////////////////////////////////////////////////////////////
//
// Description:  In this function, the ADO looks at the IMStop dial
//   to see if it has to stop at a given place.
//
// Remarks:  
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void 
CNavigateIntrsctn::PostActivity()
{
	//
	// Get the vehicle info pointer.
	//
	CAdoInfoPtr pI = GetInputpI();
#ifdef DEBUG_NI2
	bool debugThisObj = pI->m_pObj->GetId() == DEBUG_NI2;
	if( debugThisObj )
	{
		gout << MessagePrefix( pI->m_pObj->GetId() );
		gout << "======== NAVIGATEINTRSCTN POST-ACTIVITY ==========" << endl;
		gout << "stop sign crdr = " << m_stopSignCrdr << endl;
		gout << "command from IM = " << pI->m_imStopHasValue << endl;
	}
#endif

	if( !pI->m_imStopHasValue )
	{
		//
		// The dial ImStop has no value....look for stop signs.
		//
		bool needToStopForStopSign = m_stopSignCrdr && !m_madeTheNeededStop;
		if( needToStopForStopSign )
		{
			//
			// Compute the target velocity and distance.
			//
			double distToStop;
			CCrdr targCrdr;

			if( pI->m_roadPos.IsRoad() )
			{
				distToStop = pI->m_pPath->GetDistToNextIntrsctn( pI->m_roadPos );
				distToStop += m_stopSignHldOfsDist;
				targCrdr = pI->m_pPath->GetNextCrdr( pI->m_roadPos );
			}
			else
			{
				distToStop = m_stopSignHldOfsDist - pI->m_roadPos.GetDistance();
				targCrdr = pI->m_roadPos.GetCorridor();
			}

			distToStop -= ( pI->m_objLength * 0.5 + cSTOP_DIST_FROM_HOLD_OFFSET );
			if( distToStop < -3.0 )
			{
				ResetStopSignVars();
				distToStop = 0.0;
			}
			else if( distToStop < 0.0 )
			{
				distToStop = 0.0;
			}

			double targAccel;
			bool haveTargAccel = false;
			//check to see if we are already stopped and not at our target point
			if ( pI->m_currVel < 2.5 &&  
				distToStop >  (6*pI->m_currVel)/ cMETER_TO_FEET && 
				distToStop > 15 &&
				distToStop * cFEET_TO_METER > ( pI->m_currVel * pI->m_currVel )/2.0f){
				haveTargAccel =  false; //we are already going too slow
			}else{
				haveTargAccel = StopController( 
							pI->m_currVel, 
							distToStop * cFEET_TO_METER, 
							pI->m_aggressiveness, 
							m_firstFrame,
							m_firstDist,
							m_minDist,
							targAccel
							);
			}
			if( haveTargAccel )
			{
#ifdef DEBUG_NI2
				if( debugThisObj )
				{
					cout << "stopping at hold offset for stop sign" << endl;
					cout << "  targAccel = " << targAccel << "m/s2   distToStop = ";
					cout << distToStop << "ft" << endl;
				}
#endif
				

#ifdef DEBUG_NI2
				if( debugThisObj )
				{
					gout << " targCrdr id = " << targCrdr.GetRelativeId() << endl;
					gout << "stopping at hold offset for stop sign" << endl;
					gout << "  targAccel = " << targAccel << "m/s2   distToStop = ";
					gout << distToStop << "ft" << endl;
				}
#endif

				//assume 3 m/s decell
				if( targAccel < 0.0 && distToStop > /*cDIST_TO_STOPSIGN +*/ (6*pI->m_currVel)/ cMETER_TO_FEET)
				{
					int leadObjId;
					double follDist;
					bool hasLeadObj = cved->GetLeadObj( pI->m_pObj->GetId(), leadObjId, follDist );

#ifdef DEBUG_NI2
					if( debugThisObj )
					{
						cout << " hasLeadObj = " << hasLeadObj << endl;
					}
#endif
					//if we have a lead vehicle, and the lead vehicle is stoping, 
					//we want to let the follow behavoir control our stopping
					if( hasLeadObj )
					{
						if (!pI->m_roadPos.IsRoad()){
							const CDynObj* pObj = cved->BindObjIdToClass( leadObjId );

							CHcsm* pLeadHcsm = m_pRootCollection->GetHcsm( pObj->GetHcsmId() );
							//pLeadHcsm
							if (pLeadHcsm){
									CRoadPos leadRoadPos;
									bool haveValFromMonitor;
									haveValFromMonitor = pLeadHcsm->GetMonitorByName( 
																						"RoadPos", 
																						&leadRoadPos
																						);
									if (haveValFromMonitor){
										bool hasStopSignTarget;
										haveValFromMonitor = pLeadHcsm->GetMonitorByName( 
											                                           "HasStopSignTarget", 
																						&hasStopSignTarget
																						);
										if (leadRoadPos.IsValid() &&!leadRoadPos.IsRoad() && 
											leadRoadPos.GetIntrsctn().GetId() == pI->m_roadPos.GetIntrsctn().GetId()){
												targAccel = 1.0;
										}
									}
							}
						}
					}	
				}
			
				SetOutputTargAccel( targAccel );
			}
			else
			{
				SetOutputTargAccelNoValue();
			}
		}
		else
		{
			bool resetStopSignVars = ( 
						!pI->m_roadPos.IsRoad() &&
						pI->m_roadPos.GetDistance() > m_stopSignHldOfsDist
						);
			if( resetStopSignVars )
			{
				ResetStopSignVars();
			}

#ifdef DEBUG_NI2
			if( debugThisObj )
			{
				gout << "no need to stop" << endl;
			}
#endif

			SetOutputTargAccelNoValue();
		}
	}
	else
	{
		//
		// Compute the target velocity and distance.
		//
		double distToStop;

		if( pI->m_roadPos.IsRoad() )
		{
			distToStop = pI->m_pPath->GetDistToNextIntrsctn( pI->m_roadPos );
			distToStop += pI->m_imStopHldOfsDist;
		}
		else
		{
			//distToStop = pI->m_imStopHldOfsDist - pI->m_roadPos.GetDistance();
			distToStop = pI->m_imStopHldOfsDist;
		}

		distToStop -= ( pI->m_objLength * 0.5 + cSTOP_DIST_FROM_HOLD_OFFSET );
		if( distToStop < 0.0 )  distToStop = 0.0;

		double targAccel;
		bool haveTargAccel = StopController( 
					pI->m_currVel, 
					distToStop * cFEET_TO_METER, 
					pI->m_aggressiveness, 
					m_firstFrame,
					m_firstDist,
					m_minDist,
					targAccel
					);

		if( haveTargAccel )
		{
//			m_firstFrame = false;

#ifdef DEBUG_NI2
			if( debugThisObj )
			{
				gout << "stopping due to IM" << endl;
				gout << "  targAccel = " << targAccel << "m/s2   distToStop = ";
				gout << distToStop << "ft" << endl;
			}
#endif

			SetOutputTargAccel( targAccel );
		}
		else
		{
			SetOutputTargAccelNoValue();
		}
	}

	//
	// Look at the intersections that I'm registered with and write my
	// target corridors thru each of those intersections to my output.
	//
	deque<int>::iterator i;
	bool first = true;
	stringstream imTargCrdrs;
	for( i = m_signaledIntrsctns.begin(); i != m_signaledIntrsctns.end(); i++ )
	{
		int crdrId;
		bool success = false;
		if (pI->m_roadPos.IsRoad()){
			//This needs to be changed so we follow our current lane to the intersection, but this 
			//version of the CVED call does not, it will only look to see if the imediate lane is connected to 
			//the intersection that is passed in.
			success = pI->m_pPath->GetCrdrFromIntrscn( 
								*i,
								crdrId,
								&pI->m_roadPos,
								pI->m_roadPos.GetLane().GetId()
								);
			//we may be two+ intersections out, if so our target lane will not match,
			//if we are that far out we will assume we can change lanes......
			if (!success){
				success = pI->m_pPath->GetCrdrFromIntrscn( 
									*i,
									crdrId,
									&pI->m_roadPos
									);
			}
		}else{
			success = pI->m_pPath->GetCrdrFromIntrscn( 
								*i,
								crdrId,
								&pI->m_roadPos
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
		SetOutputImTargCrdrs( imTargCrdrs.str() );
	}
	else
	{
		SetOutputImTargCrdrsNoValue();
	}
	if( m_stopSignCrdr && !m_madeTheNeededStop )  
	{
		SetOutputHasStopSignTarget(true);
	}else{
		SetOutputHasStopSignTarget(false);
	}
    SetOutputStoppedAtStopSignFrame(m_StoppedAtStopSignFrame);
}