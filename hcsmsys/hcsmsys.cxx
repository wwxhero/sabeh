/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: hcsmsys.cxx,v 1.86 2012/12/18 16:03:21 iowa\dheitbri Exp $
 *
 * Author:       Omar Ahmad, Yiannis Papelis
 * Date:         January, 1999
 *
 * Description:  The HCSM system driver program.
 *
 ****************************************************************************/
#define _USE_MATH_DEFINES

#include <cvedpub.h>
#include <hcsmcollection.h>
#include <hcsmspec.h>
#include <snoparse.h>
#include <filename.h>
#include <ctype.h>
#include <staticobjmanagerutil.h>
#include <sol2.h>
#ifdef sgi
#include <unistd.h>
#include <sys/time.h>
#endif

#include "sdcaudio.h"

#ifdef _WIN32
#include <winhrt.h>
#endif

#include <pi_fstream>
#include <pi_iostream>
#include <pi_map>
#include <pi_string>
#include <pi_vector>
#include <time.h>
using namespace std;

//const int cOBJ_BTNDIAL_SIZE = 128;
//char g_objBtnDialValToHcsm[cOBJ_BTNDIAL_SIZE];

//const int cNUM_LOG_STREAMS = 5;
//float g_logStreams[cNUM_LOG_STREAMS];

char g_FullLogFileName[256] = { 0 };

//char g_runInst[128] = "";

typedef map<int, ostream*> TLogMap;


//////////////////////////////////////////////////////////////////////////////
//
//  How to use this application.
//
//////////////////////////////////////////////////////////////////////////////
void Usage( void )
{

	cerr << "Usage: " << endl;
	cerr << "hcsmsys [flags] snofile [frames]  [ snofile frames ]" << endl;
	cerr << "The following flags are available: " << endl;
	cerr << "  -cveddebug NUM         Set cved debug level to NUM" << endl;
	cerr << "  -freq                  Frequency at which to run CVED" << endl;
	cerr << "  -dynaMult              Execute dynamics n times for each ";
	cerr << "behavior step" << endl;
	cerr << "  -log CVED_OBJECT_NAME  Log data to file for object name";
	cerr << endl;
	cerr << "  -fulllog fname         Log full data for replay to 'fname'" << endl;
	cerr << "                         This option eliminates all other options" << endl;
	cerr << endl;
	cerr << "  -vlog INTVL FNAME LG   Log all data for code change verification" << endl;
	cerr << "                         INTVL => how often (in frames) to log" << endl;
	cerr << "                         FNAME => name of file containing log" << endl;
	cerr << "                         LG => if 0 don't log trf lights" << endl;
	cerr << "  -rt                    Run in pseudo real-time mode" << endl;
	cerr << "  -laneDev               compute lane deviation" << endl;
	cerr << "  -verbose               If NUM non zero, run verbose mode";
	cerr << endl;
	cerr << "  -dbgMode NUM           NUM=0 => none, 1=>Text, 2=>Graphics, ";
	cerr << "3=>both" << endl;
	cerr << "  -time                  Print timing information" << endl;
	cerr << "  -ttime                 Print timing information in CSV (for excel)" << endl;
	cerr << "  -dtime                 Print detailed timing information" << endl;
	cerr << "  -audio                 Play audio" << endl;
	cerr << "  -nocurv                Disable curvature" << endl;
	cerr << "  -ode radius            Schedule traj follower mode transition during the run, scatter them" << endl; 
    cerr << "                             in the given radius" << endl;
	exit(0);

}

class CScenFile {
public:
	CScenFile() : Name(""), frames(100) {};
	~CScenFile() {};

	string Name;
	int    frames;
};

/// global flags & variables
int     g_CvedDebug  = 0;
float   g_Freq       = 30.0f;
int     g_DynaMult   = 2;
bool    g_Verbose    = false;
bool    g_RealTime   = false;
bool    g_Timing     = false;
bool    g_DetTiming  = false;
bool    g_CSVTiming  = false;
bool    g_LaneDev    = false;
bool    g_VerLog     = false;
bool    g_VerLogLights = false;
bool    g_DoAudio    = false;
bool    g_logActivities = true;
bool    g_printActvLog = false;
bool	g_ode = false;
float   g_odeRadius = 25.0f;

string  g_VerLogFileName;
int     g_VerLogIntrvl;
FILE*   g_pVerLogFile;
EDebugMode g_DebugMode = eDEBUG_NONE;
CHcsmDebugItem::ELevel g_DebugLevel = CHcsmDebugItem::eDEBUG_ROUTINE;

string  g_Log;
vector<CScenFile>  g_SnoFiles;


//////////////////////////////////////////////////////////////////////////////
//
// Load a file into a string and parse it using the snoparser.
//
//////////////////////////////////////////////////////////////////////////////
bool LoadFile( const string& filename, CSnoParser& parser )
{
	parser.Clear();

	// parse the file using the sno parser
	bool rcode;
	try {
		rcode = parser.ParseFile(filename);
	}
	catch (CSnoParser::TError e) {

		cerr << "***Parse failed @ " << __FILE__ << ":" << __LINE__ 
			 << endl << " with error: " << e.msg << endl;
		rcode = false;
	}
	catch (...) {
		cerr << "***Parse failed @ " << __FILE__ << ":" << __LINE__ 
			 << endl << " with unknown error." << endl;
		rcode = false;
	}
	
	return rcode;
}


//////////////////////////////////////////////////////////////////////////////
//
// Puts the path on a file name.
//
// NOTE: I made a function in CFileName called TranslatePath that does 
// 	essentially the same thing as this.  I did that so bot hcsmsys and
// 	cved would have access to it.  This function is no longer used, and
// 	can be removed. -jvogel 
//
//////////////////////////////////////////////////////////////////////////////
void TranslateName( const string& name, const char* pEnvVar, string& out )
{
#ifdef _WIN32
	char dirSeparator = '\\';
#else
	char dirSeparator = '/';
#endif
	string::size_type pos = name.find_last_of(dirSeparator);
	if ( pos == string::npos ) {
		char *pDir;
		string dir;

		pDir = getenv(pEnvVar);
		if ( pDir != 0 ) {
			dir = pDir;
			if ( dir[dir.size()] != dirSeparator )
				dir += dirSeparator;
		}
			
		out = dir + name;
	}
	else {
		out = name;
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Parses command-line arguments.  WARNING:  this function writes to global
// variables.
//
//////////////////////////////////////////////////////////////////////////////
void ParseCommandLineArguments( int argc, char** argv )
{
	int arg;
	for(arg = 1; arg < argc; arg++) 
	{
		if( argv[arg][0] == '-' ) 
		{
			//
			// All of these arguments begin with a dash.
			//
			if( !strcmp( argv[arg], "-cveddebug" ) ) 
			{
				if( arg + 1 >= argc )  Usage();

				arg++;
				g_CvedDebug = atoi( argv[arg] );
			}
			else if( !strcmp( argv[arg], "-verbose" ) ) 
			{
				g_Verbose = true;
			}
			else if( !strcmp( argv[arg], "-rt" ) ) 
			{
				g_RealTime = true;
			}
			else if( !strcmp( argv[arg], "-time" ) ) 
			{
				g_Timing = true;
			}
			else if( !strcmp( argv[arg], "-ttime" ) ) 
			{
				g_Timing = true;
				g_CSVTiming = true;
			}
			else if( !strcmp( argv[arg], "-dtime" ) ) 
			{
				g_DetTiming = true;
			}
			else if( !strcmp( argv[arg], "-log" ) ) 
			{
				if( arg + 1 >= argc )  Usage();

				arg++;
				g_Log = argv[arg];
			}
			else if ( !strcmp( argv[arg], "-fulllog" ) ) 
			{
				if( arg + 1 >= argc )  Usage();

				arg++;
				strcpy(g_FullLogFileName, argv[arg]);
			}
			else if( !strcmp( argv[arg], "-noactvlog" ) )
			{
				g_logActivities = false;
			}
			else if( !strcmp( argv[arg], "-printactvlog" ) )
			{
				g_printActvLog = true;
			}
			else if( !strcmp( argv[arg], "-freq" ) ) 
			{
				if( arg + 1 >= argc )  Usage();

				arg++;
				g_Freq = (float)atof( argv[arg] );
			}
			else if( !strcmp( argv[arg], "-dynaMult" ) ) 
			{
				if( arg + 1 >= argc )  Usage();

				arg++;
				g_DynaMult = atoi( argv[arg] );
			}
			else if( !strcmp( argv[arg], "-dbgMode" ) ) 
			{
				if( arg + 1 >= argc ) Usage();

				arg++;
				g_DebugMode = (EDebugMode) atoi( argv[arg] );
			}
			else if( !strcmp( argv[arg], "-dbgLevel" ) ) 
			{
				if( arg + 1 >= argc ) Usage();

				arg++;
				g_DebugLevel = (CHcsmDebugItem::ELevel) atoi( argv[arg] );
			}
			else if( !strcmp( argv[arg], "-laneDev" ) ) 
			{
				g_LaneDev = true;				
			}
			else if( !strcmp( argv[arg], "-audio" ) ) 
			{
				g_DoAudio = true;
			}
			else if( !strcmp( argv[arg], "-vlog" ) ) 
			{
				if( arg + 2 >= argc ) Usage();
				arg++;
				g_VerLogIntrvl   = atoi( argv[arg] );
				arg++;
				g_VerLogFileName = argv[arg];
				arg++;
				g_VerLogLights = atoi( argv[arg] ) ? true : false;
				g_VerLog         = true;
			}
			else if( !strcmp( argv[arg], "-nocurv" ) ) 
			{
				CHcsmCollection::m_sDisableCurvature = true;
			}
			else if( !strcmp( argv[arg], "-ode" ) ) 
			{
				if( arg + 1 >= argc ) Usage();
				g_ode = true;
				arg++;
				g_odeRadius = (float) atof( argv[arg] );
			}
			else 
			{
				// user has given an invalid command-line argument
				Usage();
			}
		}  // end if argv[arg][0]
		else 
		{
			//
			// The only argument that doesn't begin with a dash is the
			// SCN file name.
			// 
			CScenFile   f;

			f.Name = argv[arg];
			if( argc > arg + 1 && isdigit( argv[arg+1][0]) ) 
			{
				f.frames = atoi( argv[arg+1] );
				arg++;
			}
			g_SnoFiles.push_back( f );
		}  // end else arg[arg][0]
	}  // end for
}  // ParseCommandLineArguments


//////////////////////////////////////////////////////////////////////////////
//
// Initializes CVED.
//
//////////////////////////////////////////////////////////////////////////////
static void InitializeCved( string lriFileName, CCved& cved )
{

	string cvedErr;

	double deltaT = 1.0 / g_Freq;
	cved.Configure( CCved::eCV_MULTI_USER, deltaT, g_DynaMult );
	if ( cved.Init( lriFileName, cvedErr ) == false ) {

		cerr << "Cved initialization failed with lri file '"; 
		cerr << lriFileName << "'." << endl << "Error message: " ;
		cerr << cvedErr << endl;
		cerr << "Quiting ...." << endl;

		exit( -1 );

	}

	cved.SetDebug( g_CvedDebug );

}

//////////////////////////////////////////////////////////////////////////////
//
// Dumps all CVED objects into the verification log file.  The verification
// log file is meant to be used to verify that changes to the code 
// do not affect the outcome of the simulation.  For example, one can 
// run a simulation creating a verification log and then implement code changes
// (maybe for performance enhancement) that are not meant to change the
// outcome.  Running the same scenario file after the changes should 
// produce the same verification log file.
//
static void
DumpToVerLog(int frame, FILE *pF, CCved& cved)
{
	// first get all cved objects
	CObjTypeMask objMask;
	objMask.SetAll();
	objMask.Clear(eCV_TRAFFIC_SIGN);
	objMask.Clear(eCV_OBSTACLE);
	objMask.Clear(eCV_POI);
	if ( !g_VerLogLights ) {
		objMask.Clear(eCV_TRAFFIC_LIGHT);
	}

	vector<int>                  objs;
	vector<int>::const_iterator  pO;
	int                          nObj;
	
	cved.GetAllObjs( objs, objMask );
	nObj = objs.size();

	// three lines per object
	fprintf(pF, "Frm: %d %d\n", frame, nObj);

	for (pO=objs.begin(); pO!=objs.end(); pO++) {

		// remaining two lines contain the following fields:  
		//     name cvedId hcsmtypeid solid hcsmid
		//     x y z tanI tanJ tanK latI latJ latK
		fprintf(pF, "%s %d %d %d %d\n",
			cved.GetObjName(*pO), *pO, cved.GetObjHcsmTypeId(*pO), 
			cved.GetObjSolId(*pO), cved.GetObjHcsmId(*pO));

		CPoint3D  objPos;
		CVector3D objTan, objLat;
		cved.GetObjState( *pO, objPos, objTan, objLat );

		fprintf(pF, "%-9.2f %-9.2f %-9.2f %-7.4f %-7.4f %-7.4f %-7.4f %-7.4f %-7.4f\n",
			objPos.m_x, objPos.m_y, objPos.m_z,
			objTan.m_i, objTan.m_j, objTan.m_k,
			objLat.m_i, objLat.m_j, objLat.m_k);
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Gets all objects from CVED.
//
//////////////////////////////////////////////////////////////////////////////
static void 
GetAllObjsFromCved( CCved& cved, vector<int>& objs )
{
	// build the object mask to include all object types
	CObjTypeMask objMask;
	objMask.Set( eCV_TRAJ_FOLLOWER );
	objMask.Set( eCV_VEHICLE );
	objMask.Set( eCV_TRAILER );
	objMask.Set( eCV_RAIL_VEH );

	// call the query only if there are more than zero objects
	if( cved.GetNumObjects( objMask ) > 0 ) 
	{
		cved.GetAllObjs( objs, objMask );
	}
	else 
	{
		vector<int> empty;
		objs.clear();
		objs = empty;
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Accepts two vectors of objects and finds the objects in the 2nd vector 
// that don't exist in the 1st vector and then opens log files for those
// objects.
//
//////////////////////////////////////////////////////////////////////////////
static void OpenLogFilesForNewObjs( 
			CCved& cved,
			vector<int>& prevObjs, 
			vector<int>& objs, 
			TLogMap& logFileMap )
{

	// get the difference of the 2 vectors
	vector<int> diff( 200 );
	vector<int>::iterator diffEnd;

	diffEnd = set_difference(
					objs.begin(),
					objs.end(), 
					prevObjs.begin(), 
					prevObjs.end(),
					diff.begin()
					);

	vector<int>::iterator i;
	for ( i = diff.begin(); i != diffEnd; i++ ) {
		
		int objId = *i;

		// only log objects whose name matches the name of CVED objects
		// being logged (the name was passed in as a command-line 
		// argument
		string objName = cved.GetObjName( objId );

		if ( strcmp(g_Log.c_str(), "__all__") && objName != g_Log )  continue;

		if ( g_Verbose ) {
			cout << "HCSM Logger: opening log file for object ";
			cout << objId << endl;
		}

		if ( logFileMap.find( objId ) != logFileMap.end() ) {

			cerr << "HCSMSYS: output file already exists...something ";
			cerr << "is wrong" << endl;
			exit( -1 );

		}
		else {

			// build the file name and open the file
			char fileName[128];
			sprintf( fileName, "%s_%d.log", objName.c_str(), objId );
			//string fileNameStr = objName + "_" + objIdStr + ".log";
			string fileNameStr = fileName;
			ofstream* pOutFile = new ofstream( fileNameStr.c_str() );

			if ( !(*pOutFile) ) {

				cerr << "Cannot log " << objName << " HCSM to file ";
				cerr << "named '" << fileNameStr << endl;

				exit( -1 );

			}

			logFileMap[ objId ] = pOutFile;

		}

	}

}


//////////////////////////////////////////////////////////////////////////////
//
// Accepts two vectors of objects and finds the objects in the 1st vector 
// that don't exist in the 2nd vector and then closes log files for those
// objects.
//
//////////////////////////////////////////////////////////////////////////////
static void CloseLogFilesForDeadObjs(
			vector<int>& prevObjs, 
			vector<int>& objs, 
			TLogMap& logFileMap )
{

	// get the difference of the 2 vectors
	vector<int> diff( 200 );
	vector<int>::iterator diffEnd;

	diffEnd = set_difference( prevObjs.begin(), prevObjs.end(), 
							  objs.begin(), objs.end(),
							  diff.begin() );

	vector<int>::iterator i;
	for ( i = diff.begin(); i != diffEnd; i++ ) {
		
		int objId = *i;

		if ( g_Verbose ) {
			cout << "HCSM Logger: looking to close file for obj ";
			cout << objId << endl;
		}

		if ( logFileMap.find( objId ) != logFileMap.end() ) {

			// close the log file and erase the entry from the map
			delete logFileMap[ objId ];
 			logFileMap.erase( objId );

		}

	}


}


//////////////////////////////////////////////////////////////////////////////
//
// Closes all currently open log files.
//
//////////////////////////////////////////////////////////////////////////////
static void CloseAllLogFiles( TLogMap& logFileMap )
{

	TLogMap::const_iterator i;

	for ( i = logFileMap.begin(); i != logFileMap.end(); i++ ) {
		
		int objId = i->first;

		if ( g_Verbose ) {
			cout << "HCSM Logger: looking to close file for obj ";
			cout << objId << endl;
		}

		// close the log file and erase the entry from the map
		delete logFileMap[ objId ];

	}

	logFileMap.clear();

}


//////////////////////////////////////////////////////////////////////////////
//
// Writes information to files that are open for CVED objects in the map.
//
//////////////////////////////////////////////////////////////////////////////
static void LogCvedData( CCved& cved, TLogMap& logFileMap )
{

	TLogMap::iterator i;

	for ( i = logFileMap.begin(); i != logFileMap.end(); i++ ) {

		int objId          = i->first;
		ostream* pOutFile  = i->second;

		CPoint3D objPos = cved.GetObjPosInstant( objId );
		*pOutFile << objPos.m_x << " " << objPos.m_y << " " << objPos.m_z;
		*pOutFile << endl;

	}

}

void
dumpDebugInfo(CHcsmCollection&  hcsms)
{
	CHcsmDebugItem          item;
	string                  str;
	double                   x1, y1, x2, y2, rad;
	CHcsmDebugItem::EColor   color;
	bool                    fill;

	while ( hcsms.GetDebugItem(item) ) {
		char L = item.GetLevelChar();

		switch ( item.GetType() ) {
			case CHcsmDebugItem::eNONE   :
				cerr << "Debug item of no type ???" << endl;
				break;

			case CHcsmDebugItem::eTEXT   :
				item.GetText(str);
				cerr << "F" << item.GetFrame()
					<< "(" << item.GetHcsmId() << ")" << L
					<< ": " << str << endl;
				break;

			case CHcsmDebugItem::eGRAPHIC_TEXT   :
				item.GetGraphicalText(str, x1, y1, color, fill);
				cerr << "Fr " << item.GetFrame()
					<< ":  Hcsm " << item.GetHcsmId()
					<< ":  Text = " << str
					<< ", " << x1 << ", " << y1 
					<< ",  Color = " << color
					<< ",  Orient = " << fill 
					<< endl;
				break;

			case CHcsmDebugItem::eLINE   :
				item.GetLine(x1, y1, x2, y2, color, fill);
				cerr << "Fr " << item.GetFrame()
					<< ":  Hcsm " << item.GetHcsmId() 
					<< ":  Line = " << x1 << ", " << y1 << ", " 
					<< x2 << ", " << y2 << ", color = " << color 
					<< ", fil = " << fill << endl;
				break;

			case CHcsmDebugItem::eRECT   :
				item.GetRect(x1, y1, x2, y2, color, fill);
				cerr << "Fr " << item.GetFrame()
					<< ":  Hcsm " << item.GetHcsmId() 
					<< ":  Rect = " << x1 << ", " << y1 << ", " 
					<< x2 << ", " << y2 << ", color = " << color 
					<< ", fil = " << fill << endl;
				break;

			case CHcsmDebugItem::eCIRCLE :
				item.GetCircle(x1, y1, rad, color, fill);
				cerr << "Fr " << item.GetFrame()
					<< ":  Hcsm " << item.GetHcsmId() 
					<< ":  Circle = " << x1 << ", " << y1 << ", " 
					<< rad << ", color = " << color 
					<< ", fil = " << fill << endl;
				break;

			default:
				cerr << "Debug item of unknown type ???" << endl;
		}
	}
}

static void
TestCollDet(CCved& cved)
{
	int n;

	vector<int>   out;

	CObjTypeMask mask;

	mask = CObjTypeMask::m_all;

	n = cved.CollisionDetection(2, out, mask);
	printf("Collision Detection: rcode=%d, data = ", n);
	for (unsigned int i=0; i<out.size(); i++) {
		printf("%d ", out[i]);
	}
	printf("\n");
}


#include "ScenarioControl.h"

void
printList(
		FILE*		pOut,
		CCved&		cv,
		int			curCnt, 
		int*		curList, 
		int			newCnt, 
		int*		newList, 
		int			delCnt, 
		int*		delList
		)
{
	int obj;
	static int m_Frame = 0;
	static double m_Time = 0.0;

	fprintf(
		pOut, 
		"Frm %d Time %.4f  %d %d %d\n", m_Frame, m_Time,
		newCnt, 
		curCnt, 
		delCnt
		);

	for( obj = 0; obj < newCnt; obj++ )
	{
		const CSol&        sol = cv.GetSol();
		const CSolObj* pSolObj = sol.GetObj( cv.GetObjSolId( newList[obj] ) );
		const char*     pModel = 0;

		string cat = pSolObj->GetCategoryName();

		if( !strcmp(cat.c_str(), "Vehicle") ) 
		{
			const CSolObjVehicle* pVeh = dynamic_cast<const CSolObjVehicle*> (pSolObj);
			assert( pVeh );
			pModel = pVeh->GetVisModelName().c_str();
		}
		else if( !strcmp(cat.c_str(), "Walker") ) 
		{
			const CSolObjWalker* pWalker = dynamic_cast<const CSolObjWalker*> (pSolObj);
			assert( pWalker );
			pModel = pWalker->GetVisModelName().c_str();			
		}
		else 
		{
			printf(
				"Found object whose category (%s) I can't support\n",
				cat.c_str()
				);
			assert( 0 );
		}


		fprintf(pOut, "   %d ", newList[obj]);				// object id
		fprintf(pOut, "%d ", cv.GetObjType(newList[obj]));	// obj type
		fprintf(pOut, "%20s ", pSolObj->GetCategoryName().c_str()); // obj type name
		fprintf(pOut, "%d ", cv.GetObjSolId(newList[obj]));	// model id (sol)
		fprintf(pOut, "%20s\n", pModel );	// moel name (from sol)

	}
	if ( curCnt ) {
		for (obj=0; obj<curCnt; obj++) {
			CPoint3D  objPos;
			CVector3D objTan, objLat;

			cv.GetObjStateInstant( curList[obj], objPos, objTan, objLat );

			fprintf(pOut, "   %4d", curList[obj]);
			fprintf(pOut, " %6d", cv.GetObjSolId(curList[obj]));
			fprintf(pOut, "  %14E %14E %14E   %14E %14E %14E    ",
				objPos.m_x, objPos.m_y, objPos.m_z,
				objTan.m_i, objTan.m_j, objTan.m_k);

			int ls = cv.GetVehicleVisualState(curList[obj]);
			fprintf(pOut, "%d ", ls);
			fprintf(pOut, "\n");
		}
	}

	for (obj=0; obj<delCnt; obj++) {
		fprintf(pOut, "%d\n", delList[obj]);
	}
	m_Frame++;
	m_Time += 1.0 / 60.0;
}

static bool
GetCvedObjCigiInfo( CCved& cved, int cvedId, int& uniqueId, int& modelType )
{
	int            solId    = cved.GetObjSolId( cvedId );
	const CSolObj* cpSolObj = cved.GetSol().GetObj( solId );

	//
	// Get the CIGI id.
	//
	int cigiId;
	if( cpSolObj )
	{
		cigiId = cpSolObj->GetVisModelCigiId();
		bool invalidCigiId = cigiId < 0;
		if( invalidCigiId )
		{
			cerr << "Nix: invalid cigiId (" << cigiId;
			cerr << ") for cvedId = " << cvedId << " solId = ";
			cerr << solId << "  name = " << cved.GetObjName( cvedId );
			cerr << " skipping..." << endl;

			return false;
		}
	}
	else
	{
		cerr << "Nix: unable to get VisMdoelCigiId for sol object = ";
		cerr << solId << endl;
		cigiId = 0;
	}

	//
	// Get the CIGI color.
	//
	int objColorIndex = cved.GetObjColorIndex( cvedId );
	const CSolObjVehicle* cpSolVehObj = dynamic_cast<const CSolObjVehicle*>( cpSolObj );
	int cigiColor = 0;
	if( cpSolVehObj )
	{
		cigiColor = cpSolVehObj->GetColor( objColorIndex ).cigi;
	}

	uniqueId = cvedId;
	modelType = cigiId + cigiColor;

//cout << "$$ solId = " << solId << "  cigiId = " << cigiId << "  cigiColor = " << cigiColor << "  colorIndex = " << objColorIndex << endl;
	return true;
}

void TemTest(
		const char *pScenName, 
		const char *pOutFile,
		int         maxFrames)
{
	CScenarioControl sc;
	int   curCnt, newCnt, delCnt;
	int   curList[100], newList[100], delList[100];
	short	totTimer;
	short	dynaTimer;
	short	behTimer;
	short	maintTimer;
	short   indTimer;
	int i, cvedId, uniqueId, modelType;
	static bool attached1 = false;
	double offset[6];

	totTimer   = hrt_timer_alloc(HRT_MICROSECOND, "");
	dynaTimer  = hrt_timer_alloc(HRT_MICROSECOND, "");
	behTimer   = hrt_timer_alloc(HRT_MICROSECOND, "");
	maintTimer = hrt_timer_alloc(HRT_MICROSECOND, "");
	indTimer   = hrt_timer_alloc(HRT_MICROSECOND, "");

	FILE *pOut = fopen(pOutFile, "w");
	if ( pOut == 0 ) {
		perror("Cannot open output file");
		exit(-1);
	}

	if ( sc.InitSimulation(pScenName, true) == false ) {
		fprintf(stderr, "Failed: %s\n", sc.GetLastErrorString());
		exit(-1);
	}
	fprintf(stderr, "Initialization completed, simulation starting ...\n");

	// warmup
	sc.ExecBehaviors();
	sc.ExecDynamics();
	sc.ExecDynamics();

	// prepare for object queries
	CObjTypeMask mask;

	mask.SetAll();
	mask.Clear(eCV_TRAFFIC_LIGHT);
	mask.Clear(eCV_TRAFFIC_SIGN);
	mask.Clear(eCV_COORDINATOR);

	hrt_timer_start(totTimer);
	hrt_timer_start(dynaTimer);
	hrt_timer_suspend(dynaTimer);
	hrt_timer_start(behTimer);
	hrt_timer_suspend(behTimer);
	hrt_timer_start(maintTimer);
	hrt_timer_suspend(maintTimer);

	static int root = -1, 
		leaves[20] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	static int leafCount = 0, fallenLeaves = 0;

	int frm = 0;
	curCnt = newCnt = delCnt = 0;
	while ( 1 ) {
		if ( frm && (frm % 300 == 0) ) printf("Frame %d\n", 2*frm);

		hrt_timer_start( indTimer );
		hrt_timer_resume( maintTimer );
		sc.ExecMaintainer();
		hrt_timer_suspend( maintTimer );
		hrt_timer_stop( indTimer );

		hrt_timer_start( indTimer );
		hrt_timer_resume( dynaTimer );
		sc.ExecDynamics();
		hrt_timer_suspend( dynaTimer );
		hrt_timer_stop( indTimer );

		sc.GetNewAndDeletedDynamicObjects(mask, curCnt, curList, newCnt, newList, 
						delCnt, delList, 100);

		printList(pOut, sc.GetCved(), curCnt, curList, 0, 0, delCnt, delList);

		if ( g_ode )
		{
			for( i = 0; i < newCnt; i++ ) 
			{
				cvedId = newList[i]; 
				bool gotInfo = GetCvedObjCigiInfo( sc.GetCved(), cvedId, uniqueId, modelType );
				if( gotInfo )
				{		
					if ( root == -1 )
						if ( *(sc.GetCved().GetObjName( cvedId )) == 'R' )
						{
//							printf("Root <%s> is %d\n", sc.GetCved().GetObjName( cvedId ), cvedId);
							root = cvedId;
						}
					if ( leafCount < 20 )
						if ( *(sc.GetCved().GetObjName( cvedId )) == 'L' )
						{
//							printf("Leaf #%d <%s> is %d\n", leafCount, sc.GetCved().GetObjName( cvedId ), cvedId);
							leaves[leafCount++] = cvedId;
						}
				}
			}
			
			for ( i=0; i<6; ++i )
				offset[i] = 0;
			if ( (!attached1) && (root >= 0) && (leafCount == 20) )
			{
				srand( time(NULL) ); // seed random number generator 
				for ( i=0; i<leafCount; ++i )
				{
					offset[0] = 5.0 + g_odeRadius * ( 1.0 - 2.0 * rand() / (RAND_MAX + 1.0) );
					offset[1] = -25.0 + g_odeRadius * (1.0 - 2.0 * rand() / (RAND_MAX + 1.0) );
					offset[2] = 40.0 + 120.0 * rand() / (RAND_MAX + 1.0);
					offset[3] = - M_PI / 12.0 + M_PI / 6.0 * rand() / (RAND_MAX + 1.0);
					offset[4] = - M_PI / 12.0 + M_PI / 6.0 * rand() / (RAND_MAX + 1.0);
					offset[5] = M_PI * 2.0 * rand() / (RAND_MAX + 1.0);
					if ( !sc.GetCved().CoupledObjectMotion( leaves[i], root, offset ) )
						printf("Failed to couple object %d to %d\n", leaves[i], root);
					else
					{
//						printf("Done attaching object %d to %d, offset (%.4f %.4f %.4f %.4f %.4f %.4f)\n", 
//							leaves[i], root, offset[0], offset[1], offset[2], offset[3], offset[4], offset[5]);
					}
				}
				attached1 = true;
			}

			if ( (frm == 50 + fallenLeaves * 5)  && root>=0 && leafCount == 20 && fallenLeaves<20 )
			{
				if ( !sc.GetCved().FreeObjectMotion( leaves[ fallenLeaves ] ) )
					printf("Failed to transition object %d to free motion mode\n", leaves[ fallenLeaves ]);
				++fallenLeaves;		
			}
		}

		hrt_timer_start( indTimer );
		hrt_timer_resume( dynaTimer );
		sc.ExecDynamics();
		hrt_timer_suspend( dynaTimer );
		hrt_timer_stop( indTimer );

		printList(pOut, sc.GetCved(), curCnt, curList, newCnt, newList, 0, 0);

		hrt_timer_start( indTimer );
		hrt_timer_resume( behTimer );
		sc.ExecBehaviors();
		hrt_timer_suspend( behTimer );
		hrt_timer_stop( indTimer );

		// NOTE: The above should work even if we flipped dyna/behaviors
		frm++;

		if ( frm > maxFrames/2 ) 
		{
			hrt_timer_stop(totTimer);
			hrt_timer_stop(behTimer);
			hrt_timer_stop(dynaTimer);
			hrt_timer_stop(maintTimer);

			//if ( g_Timing ) 
			{
				string info;
				double   behTime   = hrt_timer_elapsedsecs(behTimer);
				double   dynaTime  = hrt_timer_elapsedsecs(dynaTimer);
				double   maintTime = hrt_timer_elapsedsecs(maintTimer);
				double   overhead  = hrt_timer_elapsedsecs(totTimer) 
					- (behTime + dynaTime + maintTime);

				if ( g_CSVTiming ) {
					printf("%s,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
						pScenName,
						behTime,
						maintTime,
						dynaTime,
						overhead,
						hrt_timer_elapsedsecs(totTimer));
				}
				else {
					printf("======================== Timing report ==========================\n");
					printf(": \n");
					printf("Frames executed  :  %d, wall-clock time: %.2f, sim time: %.2f\n",
							frm, hrt_timer_elapsedsecs(totTimer), frm / g_Freq);
					printf("Behaviors time   :  %6.2f secs,  (%5.2f mSec/iter)\n",
						behTime, 1000.0 * behTime / frm);
					printf("Maintainer time  :  %6.2f secs,  (%5.2f mSec/iter)\n",
						maintTime, 1000.0 * maintTime / frm);
					printf("Dynamics time    :  %6.2f secs,  (%5.2f mSec/iter)\n",
						dynaTime, 1000.0 * dynaTime / frm);
					printf("Hcsmsys overhead :  %6.2f secs,  (%5.2f mSec/iter)\n\n",
						overhead, 1000.0 * overhead / frm);
					printf("%s\n", info.c_str());
					printf("==================================================================\n");
				}
			}

			return;
		}
	}
	if (pOut){
		fclose(pOut);
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Program entry point.
//
//////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
	short	totTimer;
	short	dynaTimer;
	short	behTimer;
	short	maintTimer;
	short   indTimer;
	vector<CScenFile>::const_iterator pScen;

	int RunHcsmExecMode( int, char ** );

	// If the name of the file is hcsmexec, we run the ISAT client mode
	// program.  We do this instead of creating another executable, so as
	// to minimize the number of workspaces and separate executables that
	// have to be managed.
	
	if ( NULL != strstr(argv[0], "hcsmexec") ) {
		return RunHcsmExecMode(argc, argv);
	}


	// check for arguments
	if ( argc < 2 )  Usage();

	// parse command line arguments
	ParseCommandLineArguments( argc, argv );

	if ( g_FullLogFileName[0] ) {
		pScen=g_SnoFiles.begin(); 
		assert(pScen != g_SnoFiles.end() );
		TemTest(pScen->Name.c_str(), g_FullLogFileName, pScen->frames);
		exit(0);
	}

	bool logging = ( g_Log.length() != 0 );


	//
	// Output values of command-line parameters.
	//
	if ( !g_CSVTiming && !g_DetTiming )	{	// don't corrupt CSV format

		cout << "--- Executing HCSMSYS";
		if ( g_Verbose ) {

			cout << " ---" << endl;
			cout << "  Cved debug flag is " << g_CvedDebug << endl;
			cout << "  Verbose is " << g_Verbose << endl;
			cout << "  Realtime is " << g_RealTime << endl;
			cout << "  Freq is " << g_Freq << " Hz" << endl;
			cout << "  DynaMult is " << g_DynaMult << endl;
			cout << "  Verification log is " << (g_VerLog ? "ON" : "OFF") << endl;
			if ( g_VerLog ) {
				cout << "      Interval = " << g_VerLogIntrvl << ", file = "
					<< g_VerLogFileName << endl;
			}
			cout << "  Scenario files are: " << endl;
			for (pScen=g_SnoFiles.begin(); pScen != g_SnoFiles.end(); pScen++) {
				cout << "     " << pScen->Name << " for " << pScen->frames;
				cout << " frames." << endl;
			}
		}
		else {
			cout << " ( " << g_Freq << " Hz, x";
			cout << g_DynaMult << " dyna ) ---" << endl;
		}
	}

#ifdef _WIN32
	totTimer   = hrt_timer_alloc(HRT_MICROSECOND, "");
	dynaTimer  = hrt_timer_alloc(HRT_MICROSECOND, "");
	behTimer   = hrt_timer_alloc(HRT_MICROSECOND, "");
	maintTimer = hrt_timer_alloc(HRT_MICROSECOND, "");

	indTimer   = hrt_timer_alloc(HRT_MICROSECOND, "");
#endif

	if ( g_DoAudio ) {
		if ( !InitAudio() ) {
			g_DoAudio = false;
		}
	}

#ifdef _WIN32
	short alarmHandle;

	alarmHandle = hrt_alarm_alloc(HRT_MICROSECOND);
#endif


	//
	// Load each file and parse into a sno block.
	//
	for( pScen = g_SnoFiles.begin(); pScen != g_SnoFiles.end(); pScen++ ) 
	{
		CFileName snoFile( pScen->Name );
		snoFile.TranslatePath( "NADSSDC_SCN" );

		CSnoParser parser;
		if( !LoadFile( snoFile.GetFullPathFileName(), parser ) ) 
		{
			cerr << "Cannot open or parse the input file." << endl;
			continue;
		}

		// Initialize a copy of CVED.  We find the lri name from the
		// first scenario file block which should be a header.
		CSnoParser::TIterator pBlock;
		pBlock = parser.Begin();
		if( pBlock == parser.End() || pBlock->GetBlockName() != string( "Header" ) ) 
		{
			cerr << "File is incomplete, or first block is not the header." ;
			cerr << endl;
			exit( 0 );
		}
		
		CHeaderParseBlock hdrBlk(*pBlock);
		if( g_Verbose ) cout << "LRi file is " << hdrBlk.GetLriFile() << endl;

		CCved  cved;
		InitializeCved( hdrBlk.GetLriFile(), cved );

		if( hdrBlk.HasOwnVeh() ) 
		{
			//
			// This SCN file has an external driver.  Make an ADO that will
			// represent the ExternalDriver in this scenario.
			//
			CAdoParseBlock block;
			block.SetName( "ExternalDriver" );
			block.SetSolName( "ChevyBlazerRed" );
			CRoadPos pos( cved, hdrBlk.GetOwnVehPos() );
			if( pos.IsValid() )
			{
				block.SetRoadPos( pos.GetString() );
				block.SetPath( hdrBlk.GetPath() );
				parser.AddBlock( block );
			}
		}


		// Start the remaining hcsms on a new instance of an HCSM collection
		CHcsmCollection rootCollection( 1.0f / g_Freq , &cved );

		pBlock = parser.Begin();
		for( pBlock++ ; pBlock != parser.End(); pBlock++ ) 
		{
			if( g_Verbose ) 
			{
				cout << "[" << rootCollection.GetFrame();
				cout << "] Creating hcsm '" << pBlock->GetBlockName() ;
				cout << "'." << endl;
			}

			CHcsm *pH = 
				rootCollection.CreateHcsm( pBlock->GetBlockName(), *pBlock );
			if( pH == 0 ) 
			{
				cerr << "CHcsmCollection::CreateHcsm(";
				cerr << pBlock->GetBlockName() << ") failed." << endl;
				exit( -1 );
			}

			pH->SetDebugMode( g_DebugMode );
			pH->SetDebugLevel( g_DebugLevel );
			if( "StaticObjManager" == pH->GetName() ) 
			{
				CSobjMngrParseBlock block( *pBlock );
				StaticObjManInitialSetup( &block, cved, rootCollection, pH );
			}
		}
		
		cved.Maintainer();

		//
		// Logging stuff.
		//
		vector<int> objs;
		vector<int> prevObjs;
		TLogMap logFileMap;
		if( logging ) 
		{	
			GetAllObjsFromCved( cved, objs );

			// Open log files for newly created CVED objects and close log
			// files for CVED objects that have been deleted since the last
			// frame.
			OpenLogFilesForNewObjs( cved, prevObjs, objs, logFileMap );
			CloseLogFilesForDeadObjs( prevObjs, objs, logFileMap );
		}

		if( g_VerLog ) 
		{
			g_pVerLogFile = fopen( g_VerLogFileName.c_str(), "w" );
			if( g_pVerLogFile == 0 ) 
			{
				perror( "Can't open output verification log file" );
				exit( -1 );
			}
		}

#ifdef _WIN32
		hrt_timer_start(totTimer);

		hrt_timer_start(dynaTimer);
		hrt_timer_suspend(dynaTimer);
		hrt_timer_start(behTimer);
		hrt_timer_suspend(behTimer);
		hrt_timer_start(maintTimer);
		hrt_timer_suspend(maintTimer);
		
		vector<double> dynaTimes;
		vector<double> behavTimes;
		vector<double> maintTimes;

		if ( g_DetTiming ) {
			dynaTimes.reserve(g_DynaMult * pScen->frames);
			behavTimes.reserve(pScen->frames);
			maintTimes.reserve(pScen->frames);
		}
#endif

		if( g_DoAudio )  InitAudioForRun( &cved );

		int frm;
		for( frm = 0; frm < pScen->frames; frm++ ) 
		{
#ifdef sgi
			struct timeval tm1, tm2;
			int    elapsedSecs;
			int    elapsedMicroSecs;
			int    sleepyTime;
			int    periodMicroSecs;

			if( g_RealTime ) 
			{
				gettimeofday( &tm1 );
				periodMicroSecs = 1000000.0 / g_Freq;
			}
#endif
#ifdef _WIN32
			if( g_RealTime ) 
			{
				long periodMicroSecs;

				periodMicroSecs = (long)(1000000.0 / g_Freq);
				hrt_alarm_set( alarmHandle, periodMicroSecs, HRT_ONCE );
			}
#endif
			if( g_Verbose ) 
			{
				cout << "### EXECUTING FRAME " << frm << " ###" << endl;
			}

			// Execute dynamics and logging g_DynaMult times per frame.
#ifdef _WIN32
			int dyn;
			for( dyn = 0; dyn < g_DynaMult; dyn++ ) 
			{
				hrt_timer_start( indTimer );
				hrt_timer_resume( dynaTimer );

				cved.ExecuteDynamicModels();

				hrt_timer_suspend( dynaTimer );
				hrt_timer_stop( indTimer );
				if( logging )  LogCvedData( cved, logFileMap );
				if( g_DetTiming && frm > 0 ) 
				{
					dynaTimes.push_back(hrt_timer_lastelapsedsecs(indTimer)*1000.0);
				}
			}
#else
			int dyn;
			for( dyn = 0; dyn < g_DynaMult; dyn++ ) 
			{
				cved.ExecuteDynamicModels();
				if( logging )  LogCvedData( cved, logFileMap );
			}
#endif

//			TestCollDet(cved);

			// Execute HCSMs 1 time per frame
#ifdef _WIN32
			hrt_timer_start( indTimer );
			hrt_timer_resume( behTimer );
			rootCollection.ExecuteAllHcsm();
			hrt_timer_suspend( behTimer );
			hrt_timer_stop( indTimer );
			if( g_DetTiming && frm > 0 ) 
			{
				double f = hrt_timer_lastelapsedsecs(indTimer)*1000.0;
				behavTimes.push_back(hrt_timer_lastelapsedsecs(indTimer)*1000.0);
			}
#else
			rootCollection.ExecuteAllHcsm();
#endif

			dumpDebugInfo( rootCollection );

#ifdef _WIN32
			hrt_timer_start( indTimer );
			hrt_timer_resume( maintTimer );
			cved.Maintainer();
			hrt_timer_suspend( maintTimer );
			hrt_timer_stop( indTimer );
#else
			cved.Maintainer();
#endif

			if( g_DetTiming && frm > 0  ) 
			{
				maintTimes.push_back(hrt_timer_lastelapsedsecs(indTimer)*1000.0);
			}

			if( g_VerLog && (frm % g_VerLogIntrvl) == 0 ) 
			{
				DumpToVerLog( frm, g_pVerLogFile, cved );
			}

			if( g_DoAudio ) 
			{
				AudioExec();
			}

			// Update the 'place' phonecall flag
			if( CHcsmCollection::m_sSCC_PlacePhoneCall ) 
			{
				if( frm > CHcsmCollection::m_sPlacePhoneCallAge + 200 ) 
				{
					CHcsmCollection::m_sSCC_PlacePhoneCall = 0;
				}
			}

			//
			///////// Lane deviation calculations
			//
			if( g_LaneDev ) 
			{
				double  laneDevValid = 0;
				double  laneDevFromCurLane = 0;
				double  laneWidth = 0;
				double  onRoadFlag = 0;
				int  laneId = -1;
				CPoint3D   pt;

				if( cved.IsObjValid(0) ) 
				{
					CVector3D  tang, lat;
					CRoadPos   rpos( cved );

					cved.GetObjState( 0, pt, tang, lat );

					if( rpos.SetXYZ( pt ) ) 
					{
						if( rpos.IsRoad() ) 
						{
							CLane lane;
							CRoad road;
							double ofs;

							lane = rpos.GetLane();
							road = rpos.GetRoad();
							ofs  = rpos.GetOffset();

							laneDevValid       = 1.0;
							laneDevFromCurLane = ofs;
							onRoadFlag         = 1.0;
							laneId             = lane.GetId();
							laneWidth          = lane.GetWidth();
						}
						else  
						{
							CCrdr crdr;
							double ofs;

							ofs  = rpos.GetOffset();
							crdr = rpos.GetCorridor();

							laneDevValid         = 1.0;
							laneDevFromCurLane   = ofs;
							laneWidth            = 0.0;
							onRoadFlag           = 0.0;
						}
					}
				}

				if( g_Verbose ) 
				{
					printf( "%.1f,%.1f: ldValid:%2.0f ", pt.m_x, pt.m_y, laneDevValid );
					if( laneDevValid > .001 ) 
					{
						printf(
							" LaneDev=%5.2f, LaneWidth=%5.2f, %s",
							laneDevFromCurLane, 
							laneWidth, 
							onRoadFlag > 0.1 ? "on road":"on intersection\n"
							);
						if( onRoadFlag ) 
						{
							printf( "(lane=%d)\n", laneId );
						}
					}
					else 
					{
						printf( "\n" );
					}
				}
			}

			if ( logging ) {

				// This statement exists because on the Concurrent, if 
				// 	objs is empty and it is assigned to prevObjs, a 
				// 	segmentation fault occurs.
				prevObjs.clear();

				prevObjs = objs;
				GetAllObjsFromCved( cved, objs );

				// Open log files for newly created CVED objects and close log
				// files for CVED objects that have been deleted since the last
				// frame.
				OpenLogFilesForNewObjs( cved, prevObjs, objs, logFileMap );
				CloseLogFilesForDeadObjs( prevObjs, objs, logFileMap );

			}

#ifdef sgi
			if ( g_RealTime ) {
				gettimeofday(&tm2);

				elapsedSecs      = tm2.tv_sec - tm1.tv_sec;
				elapsedMicroSecs = tm2.tv_usec - tm1.tv_usec;

				if ( elapsedMicroSecs < 0 ) {
					elapsedSecs--;
					elapsedMicroSecs = -elapsedMicroSecs;
				}
				elapsedMicroSecs += elapsedSecs * 1000000;
				sleepyTime        = periodMicroSecs - elapsedMicroSecs;

				if ( sleepyTime > 0 ) sginap(sleepyTime / 10000);
			}
#endif
#ifdef _WIN32
			if ( g_RealTime ) {
				while ( hrt_alarm_check(alarmHandle) == 0 ) {
					//Sleep(1);
				}
			}
#endif

		}

		if ( logging ) {

			// close all log files
			CloseAllLogFiles( logFileMap );

		}

		if ( g_VerLog ) {
			fclose(g_pVerLogFile);
		}

		if ( g_DoAudio ) {
			TermAudioForRun();
		}

#ifdef _WIN32
		hrt_timer_stop(totTimer);
		hrt_timer_stop(behTimer);
		hrt_timer_stop(dynaTimer);
		hrt_timer_stop(maintTimer);

		if ( g_Timing ) {
			string info;
			double   behTime   = hrt_timer_elapsedsecs(behTimer);
			double   dynaTime  = hrt_timer_elapsedsecs(dynaTimer);
			double   maintTime = hrt_timer_elapsedsecs(maintTimer);
			double   overhead  = hrt_timer_elapsedsecs(totTimer) 
				- (behTime + dynaTime + maintTime);

			cved.QryTerrainPerfCheck(info, true);
			if ( g_CSVTiming ) {
				printf("%s,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
					pScen->Name.c_str(),
					behTime,
					maintTime,
					dynaTime,
					overhead,
					hrt_timer_elapsedsecs(totTimer));
			}
			else {
				printf("======================== Timing report ==========================\n");
				printf(": \n");
				printf("Frames executed  :  %d, wall-clock time: %.2f, sim time: %.2f\n",
						frm, hrt_timer_elapsedsecs(totTimer), frm / g_Freq);
				printf("Behaviors time   :  %6.2f secs,  (%5.2f mSec/iter)\n",
					behTime, 1000.0 * behTime / frm);
				printf("Maintainer time  :  %6.2f secs,  (%5.2f mSec/iter)\n",
					maintTime, 1000.0 * maintTime / frm);
				printf("Dynamics time    :  %6.2f secs,  (%5.2f mSec/iter)\n",
					dynaTime, 1000.0 * dynaTime / frm);
				printf("Hcsmsys overhead :  %6.2f secs,  (%5.2f mSec/iter)\n\n",
					overhead, 1000.0 * overhead / frm);
				printf("%s\n", info.c_str());
				printf("==================================================================\n");
			}
		}

		if ( g_DetTiming ) {
			fprintf(stderr, "Detailed timing: behav maint dyna [dyna ...] total %d %d\n",
				behavTimes.size(), dynaTimes.size());
			double tot;

			for (unsigned int i=0; i<behavTimes.size(); i++) {
				tot = 0.0;
				printf("%6.3f %6.3f ", behavTimes[i], maintTimes[i]);
				tot += behavTimes[i];
				tot += maintTimes[i];

				for (int j=0; j<g_DynaMult; j++) {
					printf("%6.3f ", dynaTimes[i * g_DynaMult + j]);
					tot += dynaTimes[i * g_DynaMult + j];
				}

				printf("%6.2f\n", tot);
			}
		}
#endif

	}  // end for (iterate across all scenario files)

	if ( g_DoAudio ) {
		TermAudio();
	}

	return 0;
}
