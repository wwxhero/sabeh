/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: clg.h,v 1.8 1999/10/28 22:13:44
//
// Author(s):   Omar Ahmad, Matt Schikore, Yiannis Papelis
//
// Date:        November, 1999
// Rehaul:      April, 2004
//
// Description: The definition of the class CClg.  This class
//              provides an interface to the Coordinated Lights
//              Group used as member in traffic light manager 
/////////////////////////////////////////////////////////////////////////////

#ifndef _CLG_H_
#define _CLG_H_

#include <cvedpub.h>


class CClg {
public:
	struct TLight{
		int  id;
		vector<eCVTrafficLightState> pattern;
	};

	CClg(CClgParseBlock&, CVED::CCved&, double time);
	CClg(const CClg&);
	~CClg();
	CClg &operator=(const CClg&);


	bool IsActive() const { return m_active; }
	void GetState(vector<eCVTrafficLightState>&) const;
	void Execute(double time, double timeStep, CVED::CCved&);
	void MakeStringUpper(string& s);
	eCVTrafficLightState StringToLightState(const string&); 

	double TimeToLightState( double currTimeInSecs, int lightId, eCVTrafficLightState state ) const;

	void Dump(CVED::CCved&) const;
	void DumpStates(int, CVED::CCved&, double);

	void SetTargetStateAndFactor( int lightId, eCVTrafficLightState state, double factor, double currTimeInSecs );
	void SetCycleStartTime( double currTimeInSecs ) 
	{ 
		if( m_curIdx == 0 )
			m_cycleStartTime = currTimeInSecs;
		else
			m_cycleStartTime = currTimeInSecs - m_accumDuration[m_curIdx - 1];
	};

private:
	void SendTrafficLights( CCved& cved );

	bool    m_active;
	bool	m_noChangeState;
	double   m_startTime;

	double   m_factor;
	eCVTrafficLightState m_dialTargState;  // effects of dials erased, once state reached
	bool     m_dialActive;
	int      m_dialLight;
	double   m_dialStartTime;

	/////////////////////////////
	// these three below is used
	// to represent CLG table
	////////////////////////////
	string  m_intrsctnName;
	vector<double> m_duration;
	vector<TLight> m_lights;

	int     m_curIdx;         //used to index into table for current state
	double	m_cycleTime;      //the time for a whole cycle of this table
	double  m_cycleStartTime;
	int     m_numOfPatterns;  //the number of the patterns of a light
	int     m_numOfLights;
	vector<double> m_accumDuration;

	int     m_count;

};


#endif // _CLG_H_	
