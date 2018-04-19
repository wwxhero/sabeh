/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: clg.cxx,v 1.8 1999/10/28 22:13:44 
//
// Author(s):   Omar Ahmad, Matt Schikore, Yiannis Papelis
//
// Date:        November, 1999
// Rehaul:      April, 2004
//
// Description: The definition of the class CClg.  This class
//              provides an interface to the coordinated lights
//              group used as member in traffic light manager
/////////////////////////////////////////////////////////////////////////////
#include "genhcsm.h"
#include "clg.h"
#include "cvedpub.h"

/* borrowed from cved/libsrc/ */
eCVTrafficLightState cvStringToTrafficLightState(const char *pStr);

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:
//  clgBlock - reference to the CClgParseBlock associated with
//          this CAction.
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
CClg::CClg(CClgParseBlock& clgBlock, CVED::CCved& cved, double currTimeInSecs)
{
	m_count = 0;

	CClgParseBlock::TClgTable clgTable = clgBlock.GetClgTable();

	m_active = true;
	m_startTime = currTimeInSecs;
	m_curIdx = 0;
	m_cycleStartTime = m_startTime;

	m_factor = 1.0;
	m_dialTargState = eOFF;
	m_dialActive = false;
	m_dialLight = 0;
	m_dialStartTime = 0.0;

	m_noChangeState = false;
	
	m_intrsctnName = clgTable.intrsctnName;
	m_duration = clgTable.duration;
	
	double accuDuration = 0.0;
	vector<double>::const_iterator fItr = m_duration.begin();
	for (; fItr != m_duration.end(); fItr++){
		accuDuration += (*fItr);
		m_accumDuration.push_back(accuDuration);
	}
	m_numOfPatterns = m_duration.size();
	m_cycleTime = accuDuration;

	vector<CClgParseBlock::TClgRow>::iterator cItr = clgTable.clgRows.begin();
	for(; cItr != clgTable.clgRows.end(); cItr++){
		TLight light;
		if ( cved.GetObj(cItr->lightName, light.id) ) {
		
			vector<string>::iterator pItr = cItr->pattern.begin();
			for (; pItr != cItr->pattern.end(); pItr++){
				MakeStringUpper((*pItr));
				light.pattern.push_back(StringToLightState(*pItr));
			}
			m_lights.push_back(light);
		}
		else {
			printf("CClg::CClg warning: light name '%s' not "
				"found in LRI file\n", cItr->lightName.c_str());
            gout<<"CClg::CClg warning: light name '%s' not "
				"found in LRI file\n"<< cItr->lightName.c_str();
			m_active = false;
		}
	}
    if (clgTable.clgRows.size() != m_lights.size()){
        gout<<"Error Bad Traffic Light Table:"<<__FILE__<<":"<<__LINE__<<endl;
    }
    m_numOfLights = m_lights.size(); //clgTable.clgRows.size();

//	Dump( cved );
	SendTrafficLights( cved );
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//              current predicate to the parameter and returns a reference
//              to the current object
//
// Remarks:
//
// Arguments: reference to the CClg to assign
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
CClg&
CClg::operator=(const CClg& r){
	if( this != &r ){
		m_active          = r.m_active;
		m_startTime       = r.m_startTime;
		m_cycleStartTime  = r.m_cycleStartTime;
		m_accumDuration   = r.m_accumDuration;
		m_intrsctnName    = r.m_intrsctnName;
		m_duration        = r.m_duration;
		m_lights          = r.m_lights;
		m_cycleTime       = r.m_cycleTime;
		m_numOfPatterns   = r.m_numOfPatterns;
		m_numOfLights     = r.m_numOfLights;
		m_curIdx          = r.m_curIdx;
		m_factor          = r.m_factor;
		m_dialTargState   = r.m_dialTargState;
		m_dialActive      = r.m_dialActive;
		m_dialLight       = r.m_dialLight;
		m_dialStartTime   = r.m_dialStartTime;
		m_count           = r.m_count;
		m_noChangeState   = r.m_noChangeState;
	}
	return (*this);
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
CClg::~CClg() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current
//              CClg
//
// Remarks:
//
// Arguments: CClg to be copied into current CClg
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
CClg::CClg(const CClg &r)
{
	*this = r;
}
		
/////////////////////////////////////////////////////////////////////////////
//
// Description: Dump the member information
//
// Remarks:
//
// Arguments: 
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
void
CClg::Dump(CVED::CCved& cved) const
{
	gout<<"\n StartUpTime : \t\t"<<m_startTime;
	gout<<"\n cycleStartUpTime : \t\t"<<m_cycleStartTime;
	gout<<"\n Intersection : \t\t"<<m_intrsctnName<<endl;
	gout<<" CycleTime: \t\t\t"<<m_cycleTime<<endl;
	gout<<" NumOfPatterns: \t\t"<<m_numOfPatterns<<endl;
	gout<<" NumOfLights: \t\t\t"<<m_numOfLights<<endl;

	vector<double>::const_iterator dItr = m_duration.begin();
	gout<<" Duration : \t\t\t";
	for(; dItr != m_duration.end(); dItr++)
		gout<<(*dItr)<<"\t";
	gout<<endl;

	gout<<" AccuDuration : \t\t";
	dItr = m_accumDuration.begin();
	for(; dItr != m_accumDuration.end(); dItr++)
		gout<<(*dItr)<<"\t";
	gout<<endl;
	
	vector<TLight>::const_iterator lItr = m_lights.begin();
	for (; lItr != m_lights.end(); lItr++){
		gout<<" Light : "<<lItr->id<<" ";
		gout<<cved.GetObjName(lItr->id)<<"\t";
		vector<eCVTrafficLightState>::const_iterator 
												pItr = lItr->pattern.begin();
		for (; pItr != lItr->pattern.end(); pItr++){
			/*
			if ((*pItr) == eOFF)
				gout<<"OFF\t";
			else if ((*pItr) == eYELLOW)
				gout<<"Y\t";
			else if ((*pItr) == eRED)
				gout<<"R\t";
			else if ((*pItr) == eGREEN)
				gout<<"G\t";
			else if ((*pItr) == eFLASH_YELLOW)
				gout<<"FY\t";
			else if ((*pItr) == eFLASH_RED)
				gout<<"FR\t";
			*/
			switch (*pItr ) {
			case eOFF:
				gout<<"OFF\t";
				break;
			case eRED: 
				gout<<"R\t";
				break;
			case eGREEN:
				gout<<"G\t";
				break;
			case eFLASH_GREEN:
				gout<<"FG\t";
				break;
			case eYELLOW:
				gout<<"Y\t";
				break; 
			case eFLASH_YELLOW:
				gout<<"FY\t";
				break; 
			case eFLASH_RED:
				gout<<"FR\t";
				break;
			case eRED_TURN_LEFT:
				gout<<"LTR\t";
				break;
			case eYELLOW_TURN_LEFT:
				gout<<"LTY\t";
				break;
			case eGREEN_TURN_LEFT:
				gout<<"LTG\t";
				break;
			case eRED_TURN_RIGHT:
				gout<<"RTR\t";
				break;
			case eYELLOW_TURN_RIGHT:
				gout<<"RTY\t";
				break;
			case eGREEN_TURN_RIGHT:
				gout<<"RTG\t";
				break;
			case eFLASH_RED_TURN_LEFT:
				gout<<"FLTR\t";
				break;
			case eFLASH_YELLOW_TURN_LEFT:
				gout<<"FLTY\t";
				break;
			case eFLASH_GREEN_TURN_LEFT:
				gout<<"FLTG\t";
				break;
			case eFLASH_RED_TURN_RIGHT:
				gout<<"FRTR\t";
				break;
			case eFLASH_YELLOW_TURN_RIGHT:
				gout<<"FRTY\t";
				break;
			case eFLASH_GREEN_TURN_RIGHT:
				gout<<"FRTG\t";
				break;
			case eRED_STRAIGHT:
				gout<<"SR\t";
				break;
			case eGREEN_STRAIGHT:
				gout<<"SG\t";
				break;
			case eYELLOW_STRAIGHT:
				gout<<"SY\t";
				break;
			case eFLASH_RED_STRAIGHT:
				gout<<"FSR\t";
				break;
			case eFLASH_YELLOW_STRAIGHT:
				gout<<"FSY\t";
				break;
			case eFLASH_GREEN_STRAIGHT:
				gout<<"FSG\t";
				break;
			default:
				break;
			}
		}
		gout<<endl;
	}
}

void
CClg::SendTrafficLights( CCved& cved )
{
	// set traffic light state
	int i;
	assert(m_lights.size() >= m_numOfLights);
	int count = m_lights.size();
    for( i = 0; i < count; i++ ) 
	{
		int cvedId                     = m_lights[i].id;
		if ( m_lights[i].pattern.size() > m_curIdx){
			eCVTrafficLightState currState = m_lights[i].pattern[m_curIdx];
			cved.SetTrafficLightState( m_lights[i].id, m_lights[i].pattern[m_curIdx] );
		}else{
			gout<<"Invalid index for light"<<__FILE__<<":"<<__LINE__<<endl;
		}
//		printf("***CHANGING STATE OF %d TO %d\n", cvedId, currState );
	}
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: calculate the index used to find the status of each light
//              when lookup up in the CLG(coordinated light group) table
//
// Remarks:
//
// Arguments: 
// t - a double indicating the current time
// timeStep - time for a frame
//
// Returns:
//
//\todo Validate this against documentation
/////////////////////////////////////////////////////////////////////////////
void
CClg::Execute(double currTimeInSecs, double timeStep, CVED::CCved& cved) 
{
	if( !m_active )  return;

	// Find out the relative time difference in order to look up status 
	// in the table.
	if( m_dialActive )
	{
		vector<TLight>::const_iterator tItr;
		for( tItr = m_lights.begin(); tItr != m_lights.end(); tItr++ )
		{
			if( tItr->id == m_dialLight && tItr->pattern[m_curIdx] == m_dialTargState )
			{
				m_factor = 1.0;
				m_dialActive = false;
				if( m_curIdx == 0 )
					m_cycleStartTime = currTimeInSecs;
				else
					m_cycleStartTime = currTimeInSecs - m_accumDuration[m_curIdx - 1];
				m_cycleStartTime += 0.001;
				m_dialStartTime = 0.0;
//				fprintf( stdout, "@@ dial effect ending  cycleStart = %.1f\n", m_cycleStartTime );
				break;
			}
		}
	}

	// Compute the offset from the start of the current overall cycle.
	bool timeToTransition;
	if( m_dialActive )
	{
		// TODO: validate this against documentation
		double offsetInCurrentCycle = currTimeInSecs - m_cycleStartTime;
		timeToTransition = m_factor * offsetInCurrentCycle >= m_accumDuration[m_curIdx];
	}
	else
	{
		double offsetInCurrentCycle = currTimeInSecs - m_cycleStartTime;
		timeToTransition = offsetInCurrentCycle >= m_accumDuration[m_curIdx];
	}

	if( timeToTransition )
	{
		m_curIdx++;
		if( m_curIdx == m_numOfPatterns )
		{
			// Reset the start of the overall cycle.
			m_cycleStartTime = currTimeInSecs;
			m_curIdx = 0;
//			fprintf( stdout, "@@ start new cycle  startTime = %.1f  currentTimeInSecs = %.1f\n", m_cycleStartTime, currTimeInSecs );
		}

		SendTrafficLights( cved );
//		fprintf( stdout, "!! now is %.1f transitioning, (curIdx = %d)\n", currTimeInSecs, m_curIdx );
	}
}

void
CClg::DumpStates(int nth, CVED::CCved& cved, double t)
{
	eCVTrafficLightState state = cved.GetTrafficLightState(m_lights[nth].id);
	string str;
	if (state == eOFF)
		str = "OFF";
	else if (state == eYELLOW)
		str = "YELLOW";
	else if (state == eGREEN)
		str = "GREEN";
	else if (state == eRED)
		str = "RED";
	else if (state == eFLASH_YELLOW)
		str = "FLASH_YELLOW";
	else if (state == eFLASH_RED)
		str = "FLASH_RED";
	else
		str = "STRANGE!!!";

	gout<<"\n\tlight "<<nth<< "\tid "<<m_lights[nth].id<<"\tstate\t"<<str;
	gout<<"\t\ttime : "<<t;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: get the current status of each traffic light and store them
//              in the inpute parameter.
//
// Remarks:
//
// Arguments: 
// out - a vector of type eCVTrafficLightState containing the status of 
//       every traffic light.
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
void
CClg::GetState(vector<eCVTrafficLightState>& out) const
{
	out.clear();
    int size = m_lights.size();
	out.reserve(size);
	int i;
	for (i=0; i<size; i++){
		out.push_back(m_lights[i].pattern[m_curIdx]);
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: make every letter in the string uppercase
//
// Remarks:
//
// Arguments:
// s - a string that will be converted if necessary to uppercase
//
// Returns:
//
/////////////////////////////////////////////////////////////////////////////
void
CClg::MakeStringUpper(string& s)
{
	int i = 0;
	int diff = 'a' - 'A';
	for (i = 0; i < s.size(); i++){
		if ( (s[i] <= 'z') && (s[i] >= 'a') ){
			s[i] -= diff;
		}
	}
}

void
CClg::SetTargetStateAndFactor( 
			int lightId,
  			eCVTrafficLightState state,
			double factor, 
			double currTimeInSecs
			)
{
	m_dialLight = lightId;
	m_dialTargState = state;
	m_factor = factor;
	m_dialActive = true;
//	fprintf( stdout, "## setting factor = %g\n", factor );
	if( m_curIdx == 0 )
	{
		m_dialStartTime = m_cycleStartTime + currTimeInSecs;
	}
	else
	{
		m_dialStartTime = m_cycleStartTime + ( currTimeInSecs - m_accumDuration[m_curIdx - 1] );
	}

//	fprintf( stdout, "## setting m_dialStartTime = %g\n", m_dialStartTime );
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Calculates the time between now and when the given light
//  state is supposed to come up.
//
// Remarks: 
//
// Arguments:
//
// Returns: -1 --> if light is not part of pattern
//           0 --> if we're already at that state
//         > 0 --> otherwise (time to that state)
//
/////////////////////////////////////////////////////////////////////////////
double 
CClg::TimeToLightState( 
			double               currTimeInSecs, 
			int                  lightId, 
			eCVTrafficLightState state 
			) const
{
	double offsetInCycle = currTimeInSecs - m_cycleStartTime;

	vector<TLight>::const_iterator tItr;
	for( tItr = m_lights.begin(); tItr != m_lights.end(); tItr++ )
	{
		if( lightId == tItr->id ) 
		{
			int idx;
			double accumulatedTime = 0;
			for( idx = 0; idx < m_numOfPatterns; idx++ )
			{
				if( tItr->pattern[idx] == state ) 
				{
					if( idx == m_curIdx )
						// this is our current light state
						return 0.0;
					else if( idx < m_curIdx )
					{
						// the lightState is before our current light state...cycle thru the end
						double currentTime = accumulatedTime + offsetInCycle;
						double retVal = m_accumDuration[m_numOfPatterns - 1] - currentTime;
#if 0
						fprintf( 
							stdout, 
							"  retVal = %.1f   accumDur = %.1f  accumTime = %.1f   offset = %.1f\n",
							retVal,
							m_accumDuration[m_numOfPatterns - 1],
							accumulatedTime,
							offsetInCycle
							);
#endif
						return retVal;
					}
					else
						return accumulatedTime - offsetInCycle;
				}
				accumulatedTime += m_duration[idx];
//				fprintf( stdout, "   idx %d:  accumulatedTime = %.1f,  duration = %.1f  %.1f\n", idx, accumulatedTime, m_duration[idx], m_accumDuration[idx] );
			}
		}
	}

	return -1.0;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: convert the string to eCVTrafficLightState
//
// Remarks:
//
// Arguments:
// str - a string that will be converted 
//
// Returns: a variable of type eCVTrafficLightState corresponding to str
//
/////////////////////////////////////////////////////////////////////////////
eCVTrafficLightState
CClg::StringToLightState(const string& str)
{
	if (str == "O" || str == "OFF")
		return(eOFF);
	else if (str == "Y")
		return(eYELLOW);
	else if (str == "G")
		return(eGREEN);
	else if (str == "R")
		return(eRED);
	else if (str == "FR")
		return(eFLASH_RED);
	else if (str == "FY")
		return(eFLASH_YELLOW);
	else if (str == "FG")
		return(eFLASH_GREEN);
	else if (str == "LTY")
		return(eYELLOW_TURN_LEFT);
	else if (str == "LTG")
		return(eGREEN_TURN_LEFT);
	else if (str == "LTR")
		return(eRED_TURN_LEFT);
	else if (str == "RTY")
		return(eYELLOW_TURN_RIGHT);
	else if (str == "RTG")
		return(eGREEN_TURN_RIGHT);
	else if (str == "RTR")
		return(eRED_TURN_RIGHT);
	else if (str == "FLTY")
		return(eFLASH_YELLOW_TURN_LEFT);
	else if (str == "FLTG")
		return(eFLASH_GREEN_TURN_LEFT);
	else if (str == "FLTR")
		return(eFLASH_RED_TURN_LEFT);
	else if (str == "FRTY")
		return(eFLASH_YELLOW_TURN_RIGHT);
	else if (str == "FRTG")
		return(eFLASH_GREEN_TURN_RIGHT);
	else if (str == "FRTR")
		return(eFLASH_RED_TURN_RIGHT);
	else if (str == "SY")
		return(eYELLOW_STRAIGHT);
	else if (str == "SG")
		return(eGREEN_STRAIGHT);
	else if (str == "SR")
		return(eRED_STRAIGHT);
	else if (str == "FSY")
		return(eFLASH_YELLOW_STRAIGHT);
	else if (str == "FSG")
		return(eFLASH_GREEN_STRAIGHT);
	else if (str == "FSR")
		return(eFLASH_RED_STRAIGHT);
	else 
		return(eOFF);
}
