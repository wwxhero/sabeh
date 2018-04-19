/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: actvlog.h,v 1.13 2014/01/25 00:03:58 iowa\dheitbri Exp $
 *
 * Author:  Yiannis Papelis
 *
 * Date:    May, 2003
 *
 * Description:  The header file for the classes that implement the
 *   activity log mechanism.
 *
 * Note: to add a new type follow these steps:
 *
 * 1) Copy one of the subclasses of CRootEvent and modify as necessary.
 *    Note that you can't use classes as members of the m_data structure.
 * 2) Extend the EActvLogType structure.
 * 3) Extend the CreateEventByType function.
 * 4) Verify that the size of the m_data field does not exceed 
 *    the MAX_ACTIVITY_LOG_SIZE constant.
 * 5) Determine if the s/w version has to change and if so, change
 *    the g_ActvLogVersion variable.
 *
 ****************************************************************************/

#ifndef _ACTVLOG_H_
#define _ACTVLOG_H_

#include <assert.h>

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

#include <point3d.h>
#include <cved.h>
#include <actionparseblock.h>

// The amount of data stored within a node; we want to minimize memory 
// allocations but not waste too much space either
#define ACTIV_NODE_DATA_SIZE (32*1024)

// This tells us the maximum size of the data associated with any
// of the events.  It is provided as a convenience so the caller can
// use fixed size arrays as opposed to dynamic memory.  If a new event
// type is added and the size of its m_data field is larger, this has
// to be updated as well.
#define MAX_ACTIVITY_LOG_SIZE   128

// This is the version of the code; it is meant to be used as insurance
// against using incompatible versions of a file with the software.  For
// now, it has to be manually changed every time the s/w changes.
// The string is written in the header of each file so it can be 
// compared at load time.
const char g_ActvLogVersion[8] = "1.0";

// Enumeration of all the types that can be logged in the activity log.
enum EActvLogType {
	eMAGIC   = 0xA1B1C1DD,  // used to mark the file, not to be used
	eTEST_EVENT = 0,        // this is for debugging

	eHCSM_CREATE = 1,       // Hcsm creation
	eHCSM_DELETE,           // Hcsm deletion
	eHCSM_ACTIVATE,         // Hcsm activation
	eCVED_CREATE,           // Cved object creation
	eTRIGGER_FIRE,          // Trigger firing
	eDIAL_BUT_SET,          // Someone's dial/button was set
	eACTION_START_DATARED,  // Action - Start Data Reduction
	eACTION_STOP_DATARED,   // Action - End Data Reduction
	eACTION_USE_TRAFMAN_SET,// Action - Use Traffic Manager Set
	eACTION_PLAY_AUDIO,     // Action - Play Audio
	eACTION_VEHICLE_FAILURE,// Action - Vehicle Failure
	eACTION_TRAFFIC_LIGHT,  // Action - Traffic Light
	eACTION_LOG_DATA,       // Action - Log Data
	eACTION_TERMINATE_SIMULATION,  // Action - Terminate Simulation
	eACTION_PREPOS_MOTION,  // Action - Preposition Motion Base
	eACTION_TUNE_MOTION,    // Action - Tune Motion Base
	eACTION_PHONE_CALL,     // Action - Place Phone Call
	eACTION_RESET_DIAL,     // Action - Reset Dial
	eACTION_SET_VARIABLE,   // Action - Set Variable
	eACTION_CREATE_HCSM,    // Action - Create HCSM
	eACTION_DELETE_HCSM,    // Action - Delete HCSM
	eACTION_SET_DIAL,       // Action - Set Dial
	eACTION_SET_BUTTON,     // Action - Set Button

	eLAST_EVENT_MARKER      // dummy terminator, not to be used
};


/////////////////////////////////////////////////////////////////////////////
//
// This is the base abstract class for all objects that can be logged.
// The Print function should print a nicely formatted description of the
// event to standard output.  The GetSize function returns the size of
// the structure that is specific to each object type.  The GetData function
// returns a pointer to the data of the object.  The GetID returns the
// type of the object, and the CopyData function copies the data portion
// from the provided buffer to the internal object buffer.
//
class CRootEvent {
public:
	CRootEvent() {};
	~CRootEvent() {};

	static  CRootEvent*    CreateEventByType(EActvLogType type);

	virtual void           Print( int, ostream& s = cout) const = 0;
	virtual int            GetSize( void )       const = 0;
	virtual const void*    GetData( void )       const = 0;
	virtual EActvLogType   GetId( void )         const = 0;
	virtual const char*    GetIdStr( void )      const = 0;

	virtual void           CopyData( void* )           = 0;
};


//
// Represents HCSM creation
// 
class CEventHcsmCreate : public CRootEvent 
{
public:
	CEventHcsmCreate() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventHcsmCreate() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eHCSM_CREATE; };
	virtual const char*   GetIdStr(void) const { return "CreateHcsm"; };
	virtual void          CopyData( void* p )   { memcpy( &m_data, p, sizeof(m_data) ); };
	void                  SetData( 
								int hcsmId, 
								int hcsmType, 
								const string cHcsmName, 
								const CPoint3D& cPos 
								)
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_hcsmType = hcsmType;
		int maxStrSize = (int)cHcsmName.size() + 1;
		if( maxStrSize > 31 )  maxStrSize = 31;
		strncpy_s( m_data.m_name, cHcsmName.c_str(), maxStrSize );
		m_data.m_pos[0] = cPos.m_x;
		m_data.m_pos[1] = cPos.m_y;
		m_data.m_pos[2] = cPos.m_z;
	}

	virtual void Print(int frame, ostream& out = cout) const
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;
		out << "HcsmId = " << m_data.m_hcsmId << ", Type =" << m_data.m_hcsmType << endl;
		out << "Name = '" << m_data.m_name << "'" << endl;
		out << "Pos = (" 
			<< m_data.m_pos[0] << ", "
			<< m_data.m_pos[1] << ", " 
			<<  m_data.m_pos[2] << ")" << endl;
	}


	struct 
	{
		int   m_hcsmId;         // hcsm id
		int   m_hcsmType;       // hcsm type
		char  m_name[32];       // hame of hcsm
		double m_pos[3];         // position where it was created
	} m_data;
};


//
// Represents HCSM deletion
// 
class CEventHcsmDelete : public CRootEvent 
{
public:
	CEventHcsmDelete() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventHcsmDelete() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eHCSM_DELETE; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	virtual const char*   GetIdStr(void) const { return "DeleteHcsm"; };
	void                  SetData( int hcsmId, const CPoint3D& cPos )
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_pos[0] = cPos.m_x;
		m_data.m_pos[1] = cPos.m_y;
		m_data.m_pos[2] = cPos.m_z;
	}

	virtual void Print(int frame, ostream& out = cout) const
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "Pos = (" 
			<< m_data.m_pos[0] << ", "
			<< m_data.m_pos[1] << ", " 
			<<  m_data.m_pos[2] << ")" << endl;
	};

	struct 
	{
		int   m_hcsmId;			// hcsm id
		double m_pos[3];			// position where it was created
	} m_data;
};


//
// Represents HCSM activation
// 
class CEventHcsmActivate : public CRootEvent 
{
public:
	CEventHcsmActivate() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventHcsmActivate() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eHCSM_ACTIVATE; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	virtual const char*   GetIdStr(void) const { return "ActivateHcsm"; };
	void                  SetData( int hcsmId, const CPoint3D& cPos )
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_pos[0] = cPos.m_x;
		m_data.m_pos[1] = cPos.m_y;
		m_data.m_pos[2] = cPos.m_z;
	}

	virtual void Print(int frame, ostream& out = cout) const {
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;
		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "Pos = (" 
			<< m_data.m_pos[0] << ", "
			<< m_data.m_pos[1] << ", " 
			<<  m_data.m_pos[2] << ")" << endl;
	};

	struct 
	{
		int   m_hcsmId;			// hcsm id
		double m_pos[3];			// position where it was activated
	} m_data;
};


//
// Represents VRED object creation
// 
class CEventCvedCreate : public CRootEvent 
{
public:
	CEventCvedCreate() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventCvedCreate() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eCVED_CREATE; };
	virtual const char*   GetIdStr(void) const { return "CvedCreate"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
								int hcsmId, 
								int cvedId, 
								cvEObjType cvedType, 
								const CPoint3D& cPos 
								)
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_cvedId = cvedId;
		m_data.m_cvedType = cvedType;
		m_data.m_pos[0] = cPos.m_x;
		m_data.m_pos[1] = cPos.m_y;
		m_data.m_pos[2] = cPos.m_z;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "CvedId = " << m_data.m_cvedId << ",  CvedType = " << m_data.m_cvedType
			<< endl;
		out << "Pos = (" 
			<< m_data.m_pos[0] << ", "
			<< m_data.m_pos[1] << ", " 
			<<  m_data.m_pos[2] << ")" << endl;
	};

	struct 
	{
		int   m_hcsmId;			// hcsm id
		int   m_cvedId;			// internal Cved id of object created
		int   m_cvedType;		// internal CVed object type
		double m_pos[3];			// position where it was activated
	} m_data;
};


//
// Represents Trigger firing
// 
class CEventTriggerFire : public CRootEvent 
{
public:
	CEventTriggerFire() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventTriggerFire() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eTRIGGER_FIRE; };
	virtual const char*   GetIdStr(void) const { return "TriggerFire"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
								int hcsmId, 
								int instigatorHcsmId, 
								int* candidateSet, 
								int candidateSetSize
								)
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_instigatorId = instigatorHcsmId;
		int i;
		for( i = 0; i < candidateSetSize; i++ )
		{
			m_data.m_candidateSet[i] = candidateSet[i];
		}
		m_data.m_candidateSetSize = candidateSetSize;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << ", InstigId =" << m_data.m_instigatorId
			<< endl;
		
		int i;
		
		out << "Candidate Set:" << endl;
		for (i=0; i<m_data.m_candidateSetSize; i++) 
			out << m_data.m_candidateSet[i] << " ";
		out << endl;
	};

	struct 
	{
		int   m_hcsmId;             // hcsm id
		int   m_instigatorId;       // hcsm id of instigator
		int   m_candidateSet[16];   // candidate set (hcsm id)
		int   m_candidateSetSize;   // how many entries valid in CandidateSet
	} m_data;
};

//
// Represents Start Data Reduction action
// 
class CEventActionStartDataRed : public CRootEvent 
{
public:
	CEventActionStartDataRed() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionStartDataRed() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_START_DATARED; };
	virtual const char*   GetIdStr(void) const { return "ActionStartDataRed"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId,
								int segment,
								const char* pColumn,
								const char* pParams
								)
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_segment = segment;
		strncpy_s( m_data.m_column, pColumn, cDR_COLSIZE );
		strncpy_s( m_data.m_params, pParams, cDR_DATASIZE );
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "Segment = " << m_data.m_segment << ", Column = " << m_data.m_column << ", Params = " << m_data.m_params << endl;
	};

	struct 
	{
		int   m_hcsmId;              // hcsm id
		int   m_segment;             // Segment data
		char  m_column[cDR_COLSIZE+1]; // Column Name
		char  m_params[cDR_DATASIZE+1]; // Segment Params
	} m_data;
};

//
// Represents Start Data Reduction action
// 
class CEventActionStopDataRed : public CRootEvent 
{
public:
	CEventActionStopDataRed() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionStopDataRed() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_STOP_DATARED; };
	virtual const char*   GetIdStr(void) const { return "ActionStopDataRed"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
		                        int hcsmId,
								int segment, 
								const char* params
								)
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_segment = segment;
		memcpy(m_data.m_params, params, cDR_COLSIZE + cDR_DATASIZE);
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "Segment = " << m_data.m_segment << ", Params =" << m_data.m_params << endl;
	};

	struct 
	{
		int   m_hcsmId;              // hcsm id
		int   m_segment;             // Segment data
		char  m_params[cDR_COLSIZE + cDR_DATASIZE+1]; // Segment Params
	} m_data;
};

//
// Represents UseTrafManSet action
// 
class CEventActionUseTrafManSet : public CRootEvent 
{
public:
	CEventActionUseTrafManSet() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionUseTrafManSet() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_USE_TRAFMAN_SET; };
	virtual const char*   GetIdStr(void) const { return "ActionUseTrafManSet"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
		                        int hcsmId,
								const string& cSetName 
								)
	{
		m_data.m_hcsmId = hcsmId;
		strncpy_s( m_data.m_setName, cSetName.c_str(), 31 );
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "TrafManSet = " << m_data.m_setName << endl;
	};

	struct 
	{
		int   m_hcsmId;
		char  m_setName[32];
	} m_data;
};


//
// Represents PlayAudio action
// 
class CEventActionPlayAudio : public CRootEvent 
{
public:
	CEventActionPlayAudio() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionPlayAudio() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_PLAY_AUDIO; };
	virtual const char*   GetIdStr(void) const { return "ActionPlayAudio"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;
		// TBD
	} m_data;
};


//
// Represents Vehicle Failure action
// 
class CEventActionVehicleFailure : public CRootEvent 
{
public:
	CEventActionVehicleFailure() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionVehicleFailure() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_VEHICLE_FAILURE; };
	virtual const char*   GetIdStr(void) const { return "ActionVehicleFailure"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;
		// TBD
	} m_data;
};

//
// Represents TrafficLight action
// 
class CEventActionTrafficLight : public CRootEvent 
{
public:
	CEventActionTrafficLight() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionTrafficLight() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_TRAFFIC_LIGHT; };
	virtual const char*   GetIdStr(void) const { return "ActionTrafficLight"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId,
								int trafLightId,
								int trafLightState,
								double time
								)
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_trafLightId = trafLightId;
		m_data.m_trafLightState = trafLightState;
		m_data.m_time = time;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "TL id = " << m_data.m_trafLightId << ", State = " << m_data.m_trafLightState << 
			", time = " << m_data.m_time << endl;
	};

	struct 
	{
		int   m_hcsmId;
		int   m_trafLightId;
		int   m_trafLightState;
		double m_time;
	} m_data;
};


//
// Represents LogData action
// 
class CEventActionLogData : public CRootEvent 
{
public:
	CEventActionLogData() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionLogData() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_LOG_DATA; };
	virtual const char*   GetIdStr(void) const { return "ActionLogData"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId,
								int log,
								double value
								)
	{
		m_data.m_hcsmId = hcsmId;
		m_data.m_log = log;
		m_data.m_value = value;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << "LogNum = " << m_data.m_log << ", Value = " << m_data.m_value << endl;
	};

	struct 
	{
		int   m_hcsmId;
		int   m_log;
		double m_value;
	} m_data;
};

//
// Represents TerminateSimulation action
// 
class CEventActionTerminateSimulation : public CRootEvent 
{
public:
	CEventActionTerminateSimulation() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionTerminateSimulation() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_TERMINATE_SIMULATION; };
	virtual const char*   GetIdStr(void) const { return "ActionTerminateSimulation"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;
	} m_data;
};

//
// Represents PrepositionMotion action
// 
class CEventActionPreposMotion : public CRootEvent 
{
public:
	CEventActionPreposMotion() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionPreposMotion() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_PREPOS_MOTION; };
	virtual const char*   GetIdStr(void) const { return "ActionPrepositionMotion"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;
		// TBD?
	} m_data;
};

//
// Represents TuneMotion action
// 
class CEventActionTuneMotion : public CRootEvent 
{
public:
	CEventActionTuneMotion() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionTuneMotion() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_TUNE_MOTION; };
	virtual const char*   GetIdStr(void) const { return "ActionTuneMotion"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;
		// TBD?
	} m_data;
};

//
// Represents PhoneCall action
// 
class CEventActionPhoneCall : public CRootEvent 
{
public:
	CEventActionPhoneCall() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionPhoneCall() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_PHONE_CALL; };
	virtual const char*   GetIdStr(void) const { return "ActionPhoneCall"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;
		// TBD?
	} m_data;
};

//
// Represents ResetDial action
// 
class CEventActionResetDial : public CRootEvent 
{
public:
	CEventActionResetDial() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionResetDial() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_RESET_DIAL; };
	virtual const char*   GetIdStr(void) const { return "ActionResetDial"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId,
								const string& cDialName
								)
	{
		m_data.m_hcsmId = hcsmId;
		strncpy_s( m_data.m_dialName, cDialName.c_str(), 31 );
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << ", Name = " << m_data.m_dialName << endl;
	};

	struct 
	{
		int   m_hcsmId;
		char  m_dialName[32];
	} m_data;
};

//
// Represents SetVariable action
// 
class CEventActionSetVariable : public CRootEvent 
{
public:
	CEventActionSetVariable() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionSetVariable() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_SET_VARIABLE; };
	virtual const char*   GetIdStr(void) const { return "ActionSetVariable"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData(
		                        int hcsmId,
								const string& varName,
								double value
								)
	{
		m_data.m_hcsmId = hcsmId;
		strncpy_s(m_data.m_varName, varName.c_str(), 16);
		m_data.m_value = value;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
		out << m_data.m_varName << " = " << m_data.m_value << endl;
	};

	struct 
	{
		int   m_hcsmId;
	    char  m_varName[17];
		double m_value;
	} m_data;
};


//
// Represents Dial action
// 
class CEventActionSetDial : public CRootEvent 
{
public:
	CEventActionSetDial() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionSetDial() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_SET_DIAL; };
	virtual const char*   GetIdStr(void) const { return "ActionSetDial"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
								int hcsmId, 
								const string& cDialName, 
								const string& cSetting
								)
	{
		m_data.m_hcsmId = hcsmId;
		int maxStrSize = (int)cDialName.size() + 1;
		if( maxStrSize > 31 )  maxStrSize = 31;
		strncpy_s( m_data.m_name, cDialName.c_str(), maxStrSize );
		maxStrSize = (int)cSetting.size() + 1;
		if( maxStrSize >= 80 )  maxStrSize = 79;
		strncpy_s( m_data.m_setting, cSetting.c_str(), maxStrSize );
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << ", Name = '" << m_data.m_name << "'" << endl;
		out << "Setting = " << m_data.m_setting << endl;
	};

	struct 
	{
		int   m_hcsmId;				// receipient hcsm id
		char  m_name[32];			// dial name
		char  m_setting[80];		// dial setting
	} m_data;
};

//
// Represents Button action
// 
class CEventActionSetButton : public CRootEvent 
{
public:
	CEventActionSetButton() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionSetButton() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_SET_BUTTON; };
	virtual const char*   GetIdStr(void) const { return "ActionSetButton"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
								int hcsmId, 
								const string& cButtonName
								)
	{
		m_data.m_hcsmId = hcsmId;
		int maxStrSize = (int)cButtonName.size() + 1;
		if( maxStrSize > 31 )  maxStrSize = 31;
		strncpy_s( m_data.m_name, cButtonName.c_str(), maxStrSize );
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << ", Name = '" << m_data.m_name << "'" << endl;
	};

	struct 
	{
		int   m_hcsmId;				// receipient hcsm id
		char  m_name[32];			// dial name
	} m_data;
};

//
// Represents Create action
// 
class CEventActionCreate : public CRootEvent 
{
public:
	CEventActionCreate() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionCreate() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_CREATE_HCSM; };
	virtual const char*   GetIdStr(void) const { return "ActionCreateHCSM"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
								int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;				// trigger's hcsm id
	} m_data;
};

//
// Represents Delete action
// 
class CEventActionDelete : public CRootEvent 
{
public:
	CEventActionDelete() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventActionDelete() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eACTION_DELETE_HCSM; };
	virtual const char*   GetIdStr(void) const { return "ActionDeleteHCSM"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
								int hcsmId
								)
	{
		m_data.m_hcsmId = hcsmId;
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << endl;
	};

	struct 
	{
		int   m_hcsmId;				// trigger's hcsm id
	} m_data;
};

//
// Represents Button/Dial setting firing
// 
class CEventDialButSet : public CRootEvent 
{
public:
	CEventDialButSet() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventDialButSet() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eDIAL_BUT_SET; };
	virtual const char*   GetIdStr(void) const { return "SetDialBut"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };
	void                  SetData( 
								int hcsmId, 
								const string& cDialButtonName, 
								const string& cSetting
								)
	{
		m_data.m_hcsmId = hcsmId;
		int maxStrSize = (int)cDialButtonName.size() + 1;
		if( maxStrSize > 31 )  maxStrSize = 31;
		strncpy_s( m_data.m_name, cDialButtonName.c_str(), maxStrSize );
		maxStrSize = (int)cSetting.size() + 1;
		if( maxStrSize > 80 )  maxStrSize = 79;
		strncpy_s( m_data.m_setting, cSetting.c_str(), maxStrSize );
	}

	virtual void Print(int frame, ostream& out = cout) const 
	{
		char buf[20];
		sprintf_s(buf, "%-.3f", 1.0 * frame / 240.0);
		out << GetIdStr() << endl;
		out << "Frame =" << frame << ",  time =" << buf << endl;

		out << "HcsmId = " << m_data.m_hcsmId << ", Name = '" << m_data.m_name << "'" << endl;
		out << "Setting = " << m_data.m_setting << endl;
	};

	struct 
	{
		int   m_hcsmId;				// receipient hcsm id
		char  m_name[32];			// dial/button name
		char  m_setting[80];		// dial setting
	} m_data;
};


//
// A test event to be used for debugging
// 
class CEventTestEvent : public CRootEvent {
public:
	CEventTestEvent() { assert(MAX_ACTIVITY_LOG_SIZE > sizeof(m_data)); };
	~CEventTestEvent() {};

	virtual int           GetSize( void ) const { return sizeof(m_data); };
	virtual const void*   GetData( void ) const { return &m_data; };
	virtual EActvLogType  GetId( void )   const { return eTEST_EVENT; };
	virtual const char*   GetIdStr(void) const { return "TestEvent"; };
	virtual void          CopyData( void* p )   { memcpy(&m_data, p, sizeof(m_data)); };

	virtual void Print(int frame, ostream& out = cout) const {
		out << "Not ready yet" << endl;
	};
	struct 
	{
		int  p1;
		int  p2;
		short p3[4];
		double p4;
	} m_data;
};


/////////////////////////////////////////////////////////////////////////////
//
// This function instances an appropriate typed subclass of CRootEvent
// according to the specified type.  Note that the class is undefined, i.e.,
// it contains no valid data.
//
// Make sure to free the object once done using it otherwise its a memory 
// leak.
//
//
inline CRootEvent*
CRootEvent::CreateEventByType(EActvLogType type)
{
	switch ( type ) 
	{
		case eTEST_EVENT                  : return new CEventTestEvent();
		case eHCSM_CREATE                 : return new CEventHcsmCreate();
		case eHCSM_DELETE                 : return new CEventHcsmDelete();
		case eHCSM_ACTIVATE               : return new CEventHcsmActivate();
		case eCVED_CREATE                 : return new CEventCvedCreate();
		case eTRIGGER_FIRE                : return new CEventTriggerFire();
		case eDIAL_BUT_SET                : return new CEventDialButSet();
		case eACTION_START_DATARED        : return new CEventActionStartDataRed();
		case eACTION_STOP_DATARED         : return new CEventActionStopDataRed();
		case eACTION_USE_TRAFMAN_SET      : return new CEventActionUseTrafManSet();
		case eACTION_PLAY_AUDIO           : return new CEventActionPlayAudio();
		case eACTION_VEHICLE_FAILURE      : return new CEventActionVehicleFailure();
		case eACTION_TRAFFIC_LIGHT        : return new CEventActionTrafficLight();
		case eACTION_LOG_DATA             : return new CEventActionLogData();
		case eACTION_TERMINATE_SIMULATION : return new CEventActionTerminateSimulation();
		case eACTION_PREPOS_MOTION        : return new CEventActionPreposMotion();
		case eACTION_TUNE_MOTION          : return new CEventActionTuneMotion();
		case eACTION_PHONE_CALL           : return new CEventActionPhoneCall();
		case eACTION_RESET_DIAL           : return new CEventActionResetDial();
		case eACTION_SET_VARIABLE         : return new CEventActionSetVariable();
		case eACTION_SET_DIAL             : return new CEventActionSetDial();
		case eACTION_SET_BUTTON           : return new CEventActionSetButton();
		case eACTION_CREATE_HCSM          : return new CEventActionCreate();
		case eACTION_DELETE_HCSM          : return new CEventActionDelete();
		default                           : return 0;
	}
}



/////////////////////////////////////////////////////////////////////////////
//
// This class supports logging of activity events during execution of the
// HCSM system.  It is meant to log distinct events that happen 
// occationally during execution, not as a way to collect continuous data.
//
// The class, once initialized, can be used to collect events.  Each 
// event can be any among a set of predefined event types.  Each event is
// represented by its own structure that contains relevant information
// about that event.
//
// The maximum amount of events that can be logged is limited only by
// available memory.  Memory is allocated only when needed.  Each allocation
// is for relatively large chunks so there should be little load on the system
// due to memory allocation requests.
//
// Once a run is completed, the class provides a function that can be used
// to store the data into a file.  A symmetric function can be used to load
// an existing file in memory.  Once events are loaded in memory, they
// can be accessed either randomnly or sequentially.
//
// Storage layout within a Node or in file:
//     Miniheader, includes type, frame #, length; probably 12 bytes
//     Data, contents vary; length is equal to 'len' field in header
//     Miniheader, includes type, frame #, length; probably 12 bytes
//     Data, contents vary; length is equal to 'len' field in header
//
// When storing in a Node, when a whole record doesn't fit, create a new node.
// That means some blank space may remain at the end of each Node
//
class CActvLog 
{
public:
	CActvLog() { Init(); };
	~CActvLog() {};

	void Init();
	bool Store( const string& cFileName );
	bool Load( const string& cFileName, bool ignVersion = false );

	void Add( int frame, const CRootEvent* pItem );

	void SetVerbose( bool value );

	bool Get( int which, char buf[MAX_ACTIVITY_LOG_SIZE], EActvLogType& type, int& );
	bool GetNext( char buf[MAX_ACTIVITY_LOG_SIZE], EActvLogType& type, int& );

	int  GetCount( void ) const;

private:
	struct ActvNode 
	{
		int                m_numItems;					// number of items in buffer
		int                m_nextFreeByte;				// index of next free byte
		char               m_data[ACTIV_NODE_DATA_SIZE];// the buffer
	};
	struct miniHeader 
	{							// header added to each record
		EActvLogType  type;						// record type
		int           frame;					// frame number when logged
		int           len;						// length of record (excludes header)
	};

	bool              m_verbose;	// print each event when true
	list<ActvNode>    m_nodes;		// linked list of nodes
	ActvNode          m_curNode;	// node currently in use, when full 
	int               m_nextIter;	// next to return; used for GetNext();

	bool StoreNode( FILE*, ActvNode& );
	bool GetFromNode(
				ActvNode&, 
				int, 
				int, 
				char buf[MAX_ACTIVITY_LOG_SIZE], 
				EActvLogType&, 
				int&
				);

	CActvLog( CActvLog& );
	CActvLog& operator=( CActvLog& );
};

#endif
