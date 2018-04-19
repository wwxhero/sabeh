/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: ado_genlcdata.cxx,v 1.22 2016/10/28 20:59:24 IOWA\dheitbri Exp $
 *
 * Author:  Sunil Bulusu
 *
 * Created:    June, 2001
 * Updated:    May, 2001
 *
 * Description:  Contains code for the generating the ADO lane-change
 *				 timing tables.
 *
 *   1.Creates SCN files 
 *   2.Executes the batch file and outputs as longDist.txt 
 *   3.Using the text file, inputs longDist and time into 
 *	   2 arrays. cLONGDIST[][][], cTIME[][][]
 *   4.Outputs ado_lc_data.cxx, and ado_lc_data.h
 *
 ****************************************************************************/

#include <cved.h>
#include <sol2.h>

using namespace CVED;

#define DEBUG_VECTOR
#define DEBUG_TABLES
#define DEBUG_GENERAL

#define MAX_SOL_OBJS 100   		// Default 100.maximum no. of the sol objects
#define MAX_ADO_VEL 120			// mph
#define MAX_URGENCY 1.0			// max urgency

//[total solids][velocity][urgency]
const int cMAX_ROWS = int(MAX_ADO_VEL/5);
const int cMAX_COLS = int(MAX_URGENCY*10.0);

static double	LongDist[MAX_SOL_OBJS][cMAX_ROWS][cMAX_COLS];	
static double	Time[MAX_SOL_OBJS][cMAX_ROWS][cMAX_COLS]; 

//////////////////////////////////////////////////////////////////////////////
//
//  How to use this application.
//
//////////////////////////////////////////////////////////////////////////////
void Usage( void )
{

	cerr << "Usage: " << endl;
	cerr << "ado_genlcdata [flags] fileName.txt " << endl;
	cerr << "[flags]: " << endl;
	cerr << "  -havedatafile		creates the .cxx and .h files from the text file" << endl;
	cerr << "  -velRange		lowerlimit(>=5.0) upperlimit(<=120.0) " << endl; 
	cerr << "  -urgRange		give the lowerlimit(>=0.1) and the upper limit(<=1.0) " << endl;
	exit(0);

}


// globals
static string  g_DatafileName("longDist.txt");		// default longDist file 
bool		   g_HaveDataFile	=  false;			// Need not execute the whole scenario files
double		   g_UpperVelLimit  = 120.0f;			// Upper Velocity Limit
double		   g_LowerVelLimit  = 5.0f;				// Lower Velocity Limit
double		   g_UpperUrgLimit  = 1.0f;				// Upper Urgency Limit
double		   g_LowerUrgLimit  = 0.1f;				// Lower Urgency Limit


/////////////////////////////////////////////////////////////////////
// Checking the command-line parameters
/////////////////////////////////////////////////////////////////////
bool 
CheckCmdLineParameters()
{
	bool checkedParameters = true;

	checkedParameters = ( g_LowerUrgLimit <= g_UpperUrgLimit ) && 
		( g_LowerUrgLimit >= 0.1 ) &&
		( g_UpperUrgLimit >= 0.1 && g_UpperUrgLimit <= 1.0 ) &&
		( g_LowerVelLimit <= g_UpperVelLimit ) && 
		( g_LowerVelLimit >= 5.0 ) &&
		( g_UpperVelLimit >= 5.0 && g_UpperUrgLimit <= 120.0 );
	
	return checkedParameters;
}


//////////////////////////////////////////////////////
// Parses command-line arguments
//
//////////////////////////////////////////////////////

void ParseCommandLineArguments( int argc, char** argv)
{

	int arg;

	for (arg = 1; arg < argc; arg++) {

		if ( argv[arg][0] == '-' ) {

			if ( !strcmp(argv[arg], "-havedatafile")) {

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_HaveDataFile = true;
				
				g_DatafileName = argv[arg];
				cout << "File Name: " << g_DatafileName << endl;
			}
			else if ( !strcmp(argv[arg], "-velRange")) {

				if ( arg + 1 >= argc )  Usage();
				arg++;
				g_LowerVelLimit = atof(argv[arg]);

				if ( arg + 1 >= argc )  Usage();
				arg++;
				g_UpperVelLimit = atof(argv[arg]);

				if(!CheckCmdLineParameters()) Usage();

			}
			else if ( !strcmp(argv[arg], "-urgRange")) {

				if ( arg + 1 >= argc )  Usage();
				arg++;
				g_LowerUrgLimit = atof(argv[arg]);

				if ( arg + 1 >= argc )  Usage();	
				arg++;
				g_UpperUrgLimit = atof(argv[arg]);
				
				if( !CheckCmdLineParameters() ) Usage();

			}

			else {

				// user has given an invalid command-line argument
				Usage();

			}

		}
	}

}  // End of ParseCommandLineArguments


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Creates an SCN file.
//
// Remarks:
//
// Arguments:
//   velocity - The initial and target velocity of the ADO.
//   urgency  - The urgency with which to perform the lane change.
//
// Returns:  String name of the SCN file.
//
//////////////////////////////////////////////////////////////////////////////

static string 
sWriteSCNFile( double velocity, double urgency, string& solName )
{
	char fileName[101];
	sprintf(fileName, "%s-%1.2f-%1.2f.scn", solName.c_str(), velocity, urgency);
	// fileName is put into a character array...

	string strFileName(fileName);
	ofstream scnFile( fileName );
	cout << fileName << endl; 

	if( !scnFile ) {
		cout<<"Can't open file"<<endl;
		exit(-1);
	}

	scnFile << "Header" << endl;
	scnFile << "LriFile \"interstate.bli\" " << endl;
	scnFile << "ScenPosX 3.4402313E+001 " << endl;
	scnFile << "ScenPosY 3.8694646E+002 " << endl;
	scnFile << "ScenZoom 1.8076518E-001 " << endl;
	scnFile << "&&&&End&&&&" << endl;
	
	scnFile << "HCSM Gateway" << endl;
	scnFile << "&&&&End&&&&" << endl;
	scnFile << "HCSM VehFail" << endl;
	scnFile << "&&&&End&&&&" << endl;
	
	//
	//	Autonomous Dynamic Object
	//
	scnFile << "HCSM Ado" << endl;
	scnFile << "  RunMode \"eREMOTE_CONTROL\" " << endl;
	scnFile << "  RoadPos \"R_INTERSTATE1:3:870.21:0.00\" " << endl;
	scnFile << "  SolName \"" << solName.c_str() << "\"" << endl; 
	scnFile << "  RandomSol 0 " << endl;
	scnFile << "  Name \"Ado\" "<< endl;
	scnFile << "  DynModel \"Non Linear\" " << endl;
	scnFile << "  LogFile \"\""  << endl; 
	scnFile << "  UseReaDel 1 " << endl;
	scnFile << "  StdToAccType 0 " << endl;
	scnFile << "  StdToDecType 0 " << endl;
	scnFile << "  StdToDecVal1 9.0000000E-001 " << endl;
	scnFile << "  StpToAccType 0 " << endl;
	scnFile << "  DecToAccType 0 " << endl;
	scnFile << "  FollowTimeMin 1.0000000E+000 " << endl;
	scnFile << "  FollowTimeMax 2.0000000E+000 " << endl;
	scnFile << "  EmergDecClip -1.1000000E+001 " << endl;
	scnFile << "  Accel2Catch 0 " << endl;
	scnFile << "  NormVel2Kp 7.0000000E-001" << endl; 
	scnFile << "  LcvFall 1.5000000E+000 2.0000000E+000 " << endl;
	scnFile << "  LcvFreq 3.0000000E-002 5.0000000E-002 " << endl;
	scnFile << "  LcvRAmpl 1.0000000E-001 5.0000000E-001 " << endl;

	scnFile.setf( ios::scientific );
	scnFile << "  VelCtrlInitVel " << velocity << endl;
	scnFile.unsetf( ios::scientific );

	scnFile << "  VelCtrlDistType 0 " << endl;

	scnFile.setf( ios::scientific );
	scnFile << "  VelCtrlDistVal1 " << velocity << endl; 
	scnFile.unsetf( ios::scientific );

	scnFile << "  VelCtrlDistVal2 9.4759781E+289 " << endl;
	scnFile << "&&&&End&&&&" << endl;
	
	//
	//	Roadpad Trigger
	//
	scnFile << "HCSM RoadPadTrigger " << endl;
	scnFile << "Path \"R:R_INTERSTATE1:0[962.87:1022.44]:1[962.87:1022.44]:";
	scnFile << "2[962.87:1022.44]:3[962.87:1022.44]:4[962.87:1022.44]:";
	scnFile << "5[962.87:1022.44]:6[962.87:1022.44]:7[962.87:1022.44]\" "<< endl;
	scnFile << "Position 4.2395504E+001 9.7797353E+002 0.0000000E+000 " << endl;
	scnFile << "DrawPosition 1.1739550E+002 9.7797353E+002 0.0000000E+000 " << endl;
	scnFile << "ByTypeSet \"Vehicle\"" << endl;
	scnFile << "NthFromStart 0 " << endl;
	scnFile << "NthFromEnd 0 " << endl;
	scnFile << "VehicleAhead 0 " << endl;
	scnFile << "VehicleBehind 0 " << endl;
	scnFile << "ActvDel 0.0000000E+000 " << endl;
	scnFile << "CrRad 0.0000000E+000 " << endl;
	scnFile << "Debounce 0.0000000E+000 " << endl;
	scnFile << "FireDel 0.0000000E+000 " << endl;
	scnFile << "Lifetime 0.0000000E+000 " << endl;
	scnFile << "Name \"RoadPadTrigger\" " << endl;
	scnFile << "OneShot 1 " << endl;
	scnFile << "SeqAct 0 " << endl;
	scnFile << "  HCSM SetDial " << endl;
	scnFile << "    Comment \"Lane Change\" " << endl;
	scnFile << "    ByNameSet \"Ado\" "<< endl;
	scnFile << "    Delay 0.0000000E+000 " << endl;
	scnFile << "    InstigatorSet 0 " << endl;
	scnFile << "    Dial \"LaneChange\" \"left;" << urgency << ";0\"" << endl; 
	scnFile << "    ButtonDialPath \"Ado/LaneChange\" " << endl; 
	scnFile << "  &&&&End&&&& " << endl;
	scnFile << "&&&&End&&&& " << endl;

	scnFile << "HCSM StaticObjManager" << endl;
	scnFile << "&&&&End&&&&" << endl;
 
	scnFile.close();
	return strFileName;
} // sWriteSCNFile

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Test Batch file function.
//
// Remarks:  Executes only one file to see if longDist.txt gets created
//
// Arguments:
//   pBinPath  - The path to the main bin directory.
//   pDataPath - The path to the main data directory.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////

bool 
ExecuteTestBatch( const char* pBinPath, const char* pDataPath )
{	
	bool txtFileExists = false;
	//
	// Copying all the scenario files into main 'data' directory.
	//
	char command[4096];
	strncpy( command, "move *.scn ",4096 );
	strncat( command, pDataPath,4096 );
	system( command );

	//
	// Testing if 'longdist.txt' file get created
	//
	cout << "\nTesting if 'longdist.txt' file get created" << endl;
	system( "testTextFile.bat" );


	ifstream ifsInputFile;
	//
	// Opening the txt file
	//
	string fileName = g_DatafileName;
	ifsInputFile.open(fileName.c_str());
	if(!ifsInputFile) {
		cout << "FILE NOT CREATED!!" << endl;
		txtFileExists = false;
	}
	else{
		cout << "FILE CREATED" << endl;
		// Deleting the batch file
		strncpy( command, "del ",4096);
		strncat( command, "testTextFile.bat",4096 );
		system( command );
		// Deleting the text file
		strncpy( command, "ren ", 4096);
		strncat( command, fileName.c_str(),4096 );
		strncat( command, " deleteNow.txt",4096 );
		system( command );
		strncpy( command, "del ",4096);
		strncat( command, "deleteNow.txt",4096 );
		system( command );
		txtFileExists = true;
	}
	return txtFileExists;
} // ExecuteTestBatch

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Batch file function.
//
// Remarks:  Always try to comment this function in the main program.  It 
//   will take a lot of time to execute the HCSMSYS and scenario file.
//
// Arguments:
//   pBinPath  - The path to the main bin directory.
//   pDataPath - The path to the main data directory.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////

static void 
ExecuteBatchScenarios( const char* pBinPath, const char* pDataPath )
{	
	//
	// Copying all the scenario files into main 'data' directory.
	//
	char command[4096];
	strcpy( command, "move *.scn " );
	strcat( command, pDataPath );
	system( command );

	//
	// Move deletion batch file to the main 'data' directory.
	//
	strcpy( command, "move del_ado_genlcdata.bat " );
	strcat( command, pDataPath );
	system( command );


	//
	// Executing the batch file from the current directory.
	//
	cout << endl << endl << "EXECUTING the Scenarios......" << endl << endl;
	system( "ado_genlcdata.bat" );

	//
	// Delete the batch file in the nadssdc directory.
	//
	//strcpy( command, "del ");
	//strcat( command, pBinPath );
	//strcat( command, "ado_genlcdata.bat" );
	//system( command );
}  // end of ExecuteBatchScenarios

////////////////////////////////////////////////////////////////////////
//
// Decription: Inputs the solObjs into the Vector gets called in the 
//			   main program. Also updates the solIdToIdexMap 
// 
// Arguments :
//
//
// Return Type: static Void
//
////////////////////////////////////////////////////////////////////////

CCved* cved;

vector<int> AllSolIds;
vector<int>::iterator itr; 

static int					solIdIndex = 0; // solId -> solIdIndex
static int					indexSolName = 0; // solNameIndex -> solName
static string				solName;
static int					totalObjs = 0; // Holds the total objects in the Vector

map<int,int> solIdToIndexMap; // solId -> solIdIndex
map<int,string> IndexToSolNameMap; // solNameIndex -> solName

void 
sAddSolObjsToVector()
{
	vector<int> solCatIds;
	const int typeMask[] = {cSOL_TRUCK, cSOL_CAR, cSOL_SUV, cSOL_VAN, cSOL_UTILITY, 
		cSOL_BUS, cSOL_POLICE, cSOL_AMBULANCE, cSOL_COMMERCIAL};

	const int canBeMask = cSOL_ADO;
	const int cOBJ_CATS = 9;
	int totalObjCount = 0;
	vector<int>::iterator itr;

	for( int i = 0; i < cOBJ_CATS; i++ )
	{	
		int catCount =	cved->GetSol().GetAllVehicles(solCatIds, canBeMask, typeMask[i]);
		
		// Removing duplicate 'ChevyBlazer' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "ChevyBlazer" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'TestCar' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "TestCar" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'SemiTrailer_60' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "SemiTrailer_60" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'AudiTest' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "AudiTest" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'NadsCabTaurus' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "NadsCabTaurus" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'Truckcargo_DAF' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "Truckcargo_DAF" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'ConstructionVan' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "ConstructionVan" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'NadsCabCherokee' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "NadsCabCherokee" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'Combine_6600' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "Combine_6600" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'Tractor_8400' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "Tractor_8400" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'NadsCabMalibu' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "NadsCabMalibu" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Removing 'NadsCabTruck' in Car category
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			const CSolObj* solObj = cved->GetSol().GetObj(*itr);
			if( i == 1 && solObj->GetName() == "NadsCabTruck" )
			{
				itr = solCatIds.erase(itr);
			}
		}
		// Pushing SolIds into global vector
		for( itr = solCatIds.begin(); itr != solCatIds.end(); itr++ ) 
		{
			AllSolIds.push_back(*itr);
		}
	}
	//Populating SolId map
	for( itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++ ) 
	{
		//const CSolObj* solObj = cved->GetSol().GetObj(*itr);
		solIdToIndexMap[*itr] = solIdIndex;

#ifdef DEBUG_VECTOR
		cout << *itr << " --- " << solIdIndex << endl;
#endif
		solIdIndex++;
		totalObjs++;

	}
		//Populating SolId map
	for( itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++ ) 
	{
		const CSolObj* solObj = cved->GetSol().GetObj(*itr);
		string solName = solObj->GetName();
		IndexToSolNameMap[indexSolName] = solName;

#ifdef DEBUG_VECTOR
		cout << solName << " --- " << indexSolName << endl;
#endif

		indexSolName++;
	}
	//
	// Check totalObjs with MAX_SOL_OBJS 
	//
	if ( totalObjs > MAX_SOL_OBJS) 
	{
		cout << "\nWARNING!! total sol objects exceeded MAX_SOL_OBJS.." << endl;
		cout << "update compiler preprocessor 'MAX_SOL_OBJS' \n " << endl;
		totalObjs = MAX_SOL_OBJS;

	}
} // sAddSolObjsToVector


////////////////////////////////////////////////////////////////////////////// 
//
// Description:  Inputs data into arrays.
//
// Remarks:
// We have the text file consisting of Long Dist and Time.
// Plan 1:
// Make Two Tables--->1.LongDist 2.Time
//
//
//
//							Urgency
//	Example 		 0	 1	 2   4	 5			|      longDist.txt
//		Velocity	0.1 0.2 0.3 0.4 0.5			|	Vel	Urg LongDist Time
//		--------------------------------		|	55	0.4	 400.50	 4.96
//		0	5		500 600 700	800	900			|
//		1	10		300	400	470	600	800			|
//	
// Arguments:
//   velocity - The initial and target velocity of the ADO.
//   urgency  - The urgency with which to perform the lane change.
//
// Returns:  Void
//
//////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
//
// This function takes in a double and calculates the 
// nearest integer divisible by 5 and returns the 
// quotient that is used to input into the arrays
// 
// Necessary because sometimes the velocities go over/under the
// specified in lane change and we have to input the dist and
// time taken exactly at the right spot in the array.
//
//
int round( double num )
{
	// 40 -- 41 -- 45
	int val = (int)(num > 0.0 ? num + 0.5 : num - 0.5);
	int deductNum = 0;
	int dNum = 0;
	int addNum = 0;
	int aNum = 0;
	if( (val%5) == 0) // value divisible by 5, so exit
	{
#if 0 
		cout << "Rounded val [" << num << " -> " << val << "] is divisible by 5" << endl;
#endif
		return val/5;
	}
	for(deductNum = 0; deductNum < 5; deductNum++)
	{
		// Deduct '1' and see if divisible by 5. Continue
		// deduction till divisible by 5
		if((val-deductNum)%5 == 0 && val > 0)
		{
#if 0
			cout << "DeductNum: " << deductNum << endl;
#endif
			dNum = deductNum;
			break;
		}
	}

	for(addNum = 0; addNum < 5; addNum++)
	{
		// Add '1' and see if divisible by 5. Continue
		// adding till divisible by 5
		if( (val+addNum)%5 == 0 )
		{
#if 0
			cout << "Add Num: " << addNum << endl;
#endif
			aNum = addNum;
			break;
		}
	}	
	if( aNum < dNum )
	{
#if 0
		cout << "(val+aNum)/5: " << (val+aNum) << endl;
#endif
		return (val+aNum)/5;
	}
	else
	{
#if 0
		cout << "(val-dNum)/5: " << (val-dNum) << endl;
#endif
		return (val-dNum)/5;
	}

}

static void 
InputIntoArrays( string& dataFileName )
{
	
	double velocity;
	double urgency;
	double longDist;
	double time;
	int solId;


	//		
	// Opening the text file which got created during the execution 
	// of the scenario files
	//
	ifstream ifsInputFile;
	
	//
	// Below is the Test text file fullTable.txt,default is longDist.txt
	//
	ifsInputFile.open(dataFileName.c_str());
	//cout << "---------- " << dataFileName << " ---------" << endl;

	
	if(!ifsInputFile) {
		
		cout << "FILE DOESN'T EXIST!!" << endl;
		exit(-1);
	}

	// Inputting values into the Arrays
	while (ifsInputFile)
	{
		ifsInputFile >> velocity  >> urgency >> longDist >> time >> solId;
		double urg = urgency * 10;

		// Getting the Index of the particular SolId
		solIdIndex = solIdToIndexMap[solId];

		// Inserting values from the text file into the array
		//cout << velocity << " - " << round(velocity) - 1 << " "<< urgency << " - " << int(urg - 1) << endl;
		if( round(velocity) == 0 ) // 1.3, 1.7mph etc.
		{
			LongDist[solIdIndex][0][int(urg - 1)] = longDist;
			Time[solIdIndex][0][int(urg - 1)] = time;	
		}
		else
		{
			LongDist[solIdIndex][(round(velocity) - 1 )][int(urg - 1)] = longDist;
			Time[solIdIndex][(round(velocity) - 1)][int(urg - 1)] = time;	
		}
		if (ifsInputFile.eof()) {
			break;
		}
	}


#ifdef DEBUG_TABLES

	// Printing LongDist Table
	int id;		// Sol Id
	int i, j;	// rows and columns in the LongDist array
	for ( id = 0; id < MAX_SOL_OBJS; id++)
	{
		for ( i = 0; i < cMAX_ROWS; i++ )
		{
			for ( j = 0; j < cMAX_COLS; j++ )
			{
				if ( LongDist[id][i][j] != NULL )
				{
					cout << "LongDist[" << i << "][" << j << "][";
					cout << id << "]" << " = " << LongDist[id][i][j] << endl;
				}
			}
		}
	}
	cout << "End of Long Dist table" << endl << endl;

	// Printing Time-Table
	int p, q;	// rows and columns in the Time array 
	for ( id = 0; id < MAX_SOL_OBJS; id++)
	{
		for ( p = 0; p < cMAX_ROWS; p++ )
		{
			for ( q = 0; q < cMAX_COLS; q++ )
			{
				if ( Time[id][p][q] != NULL )
				{
					cout << "Time[" << p << "][" << q << "][";
					cout << id << "]" << " = " << Time[id][p][q] << endl;
				}
			}
		}
	}
	cout << "End of Time table" << endl << endl;

#endif	// DEBUG_TABLES

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Creates the ado_lc_data.cxx and ado_lc_data.h files.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
///////////////////////////////////////////////////////////////////////////////
static void 
CreateLcDataFiles()
{

	ofstream hFile( "ado_lc_data.h" );

	hFile << "/*****************************************************************************" << endl;
	hFile << " *" << endl;
	hFile << " * (C) Copyright 1998 by National Advanced Driving Simulator and" << endl;
	hFile << " * Simulation Center, the University of Iowa and The University" << endl;
	hFile << " * of Iowa. All rights reserved." << endl;
	hFile << " *" << endl;
	hFile << " *" << endl;
	hFile << " * Version: $Id: ado_genlcdata.cxx,v 1.22 2016/10/28 20:59:24 IOWA\dheitbri Exp $\n\n"; 
	hFile << " * Author(s): Sunil Bulusu" << endl;
	hFile << " *" << endl;
	hFile << " * Date: April, 2004" << endl;
	hFile << " * Updated: May, 2005" << endl;
	hFile << " *" << endl;
	hFile << " * Description:  Interface for the ado_lc_data.cxx source file." << endl;
	hFile << " *" << endl;
	hFile << " ****************************************************************************/" << endl;
	hFile << "#ifndef __ADO_LC_DATA_H" << endl;
	hFile << "#define __ADO_LC_DATA_H" << endl << endl;

	hFile << "bool LcLookupDistTime(" << endl;
	hFile << "			double velocity," << endl;
	hFile << "			double urgency," << endl; 
	hFile << "			int solId, "			<< endl;
	hFile << "			double &longDist," << endl;
	hFile << "			double &time" << endl;
	hFile << "			);" << endl << endl;

	hFile << "bool LcLookupVelocityfromDist(" << endl;
	hFile << "			double dist," << endl;					 
	hFile << "			double urgency," << endl; 
	hFile << "			int solId, " << endl;
	hFile << "			double& velocity" << endl;
	hFile << "			);" << endl << endl;

	hFile << "bool LcLookupVelocityfromDist(" << endl;
	hFile << "			double dist," << endl;
	hFile << "			int solId, " << endl;
	hFile << "			vector<vector<double> >&" << endl;
	hFile << "			);" << endl << endl;

	hFile << "bool LcLookupVelocityfromTime(" << endl;
	hFile << "			double time," << endl;
	hFile << "			double urgency," << endl;
	hFile << "			int solId, " << endl;
	hFile << "			double& velocity	" << endl;
	hFile << "			);" << endl << endl;

	hFile << "bool LcLookupVelocityfromTime(" << endl;
	hFile << "			double dist," << endl;
	hFile << "			int solId," << endl; 
	hFile << "			vector<vector<double> >&" << endl;
	hFile << "			);" << endl << endl;

	hFile << "bool LcLookupUrgencyfromDist(" << endl;
	hFile << "			double velocity," << endl;
	hFile << "			double Dist,	" << endl;
	hFile << "			int solId, " << endl;
	hFile << "			double &urgency" << endl;
	hFile << "			);" << endl << endl;

	hFile << "bool LcLookupUrgencyfromTime( " << endl;
	hFile << "			double velocity," << endl;
	hFile << "			double time,	" << endl;
	hFile << "			int solId, " << endl;
	hFile << "			double &urgency" << endl;
	hFile << "			);" << endl << endl << endl;
	hFile << "// End of 'ado_lc_data.h' Header file" << endl << endl;
	hFile << "#endif  // __ADO_LC_DATA_H" << endl;

	hFile.clear();
	hFile.close();

	
	//
	// ado_lc_data.cxx file
	//
	ofstream cxxFile ( "ado_lc_data.cxx" );

	cxxFile << "/***********************************************************************************" << endl;
	cxxFile << " * " << endl;
	cxxFile << " * Version:ado_lc_data.cxx" << endl;
	cxxFile << " *" << endl;
	cxxFile << " * Version: $Id: ado_genlcdata.cxx,v 1.22 2016/10/28 20:59:24 IOWA\dheitbri Exp $\n\n"; 
	cxxFile << " * Author:SUNIL BULUSU" << endl << endl;
	cxxFile << " * Created: April, 2004 " << endl;
	cxxFile << " * Updated: May, 2005 " << endl;
	cxxFile << " *" << endl;
	cxxFile << " * Contains the following: " << endl;
	cxxFile << " * 1. cLONGDIST[" << totalObjs << "][" << cMAX_ROWS <<"][" << cMAX_COLS << "]" << endl;
	cxxFile << " * 2. cTIME[" << totalObjs << "][" << cMAX_ROWS <<"][" << cMAX_COLS << "]" << endl;
	cxxFile << " * " << endl;
	cxxFile << " * LcLookupDistTime(double velocity, double urgency, int solId, double &longDist, double &time)" << endl;
	cxxFile << " * --------------------------------------------------------------------------------" << endl;
	cxxFile << " * 3. LookupDistTime function is used to interpolate the Long Dist and" << endl;
	cxxFile << " *    time when Velocity and corresponding Urgency are given" << endl;
	cxxFile << " *    Parameters: " << endl;
	cxxFile << " *    Velocity, Urgency, references to Long Dist and Time" << endl;
	cxxFile << " * " << endl;
	cxxFile << " * LcLookupVelocityfromDist(double dist, double urgency, int solId, double& velocity) " << endl;
	cxxFile << " * ---------------------------------------------------------------------------------" << endl;
	cxxFile << " * 4. LookupVelocityfromDist functions takes input as the forward distance," << endl;
	cxxFile << " *	  urgency and solID and returns a velocity with which the distance can be achieved" << endl;
	cxxFile << " * " << endl;
	cxxFile << " *    Parameters:" << endl;
	cxxFile << " *	  Distance, Urgency, solId, and reference to Velocity" << endl;
	cxxFile << " * " << endl;
	cxxFile << " * LcLookupVelocityfromDist(double dist, int solId, vector<vector<double> >)" << endl;
	cxxFile << " * ---------------------------------------------------------------------------------" << endl;
	cxxFile << " * 5. LookupVelocityfromDist functions takes input as the forward distance," << endl;
	cxxFile << " *	  and solID and returns a vector that consists of matching velocity-urgency pairs" << endl;
	cxxFile << " * " << endl;
	cxxFile << " *    Parameters:" << endl;
	cxxFile << " *	  Distance, solId, and reference to a Vector" << endl;
	cxxFile << " * " << endl;
	cxxFile << " * LcLookupVelocityfromTime(double time, double urgency, int solId, double& velocity) " << endl;
	cxxFile << " * ---------------------------------------------------------------------------------" << endl;
	cxxFile << " * 6. LookupVelocityfromTime functions takes input as time, urgency and solID and " << endl;
	cxxFile << " *	  returns a velocity with which Lc can be achieved in the given time" << endl;
	cxxFile << " *" << endl;
	cxxFile << " *    Parameters:" << endl;
	cxxFile << " *	  Time, Urgency, solId, and reference to Velocity" << endl;
	cxxFile << " *" << endl;
	cxxFile << " * LcLookupVelocityfromTime(double dist, int solId, vector<vector<double> >)" << endl;
	cxxFile << " * ---------------------------------------------------------------------------------" << endl;
	cxxFile << " * 7. LookupVelocityfromTime functions takes input as time, solId and " << endl;
	cxxFile << " *	  and returns a vector that consists of matching velocity-urgency pairs with which" << endl;
	cxxFile << " *	  Lc can be achieved" << endl;
	cxxFile << " *" << endl;
	cxxFile << " *    Parameters:" << endl;
	cxxFile << " *	  Distance, solId, and reference to a Vector" << endl;
	cxxFile << " * LcLookupUrgencyfromDist(double velocity, double Dist, int solId, double &urgency)" << endl;
	cxxFile << " * ---------------------------------------------------------------------" << endl;
	cxxFile << " * 8. Searching the Table for the Urgency when Velocity and Distance are given" << endl;
	cxxFile << " *	  Parameters:" << endl;
	cxxFile << " *	  Velocity, Distance and reference to the Urgency variable" << endl;
	cxxFile << " *" << endl;
	cxxFile << " * LcLookupUrgencyfromTime(double Velocity, double time, int solId, double &urgency)" << endl;
	cxxFile << " * --------------------------------------------------------------------" << endl;
	cxxFile << " * 9. Searching the Table for the Urgency when Velocity and Time are given" << endl;
	cxxFile << " *    Parameters:" << endl;
	cxxFile << " *	  Velocity, Time and reference to the Urgency variable		" << endl;
	cxxFile << " *" << endl;
	cxxFile << " * Return Type: bool" << endl;
	cxxFile << " *" << endl;
	cxxFile << " *" << endl;
	cxxFile << " * WARNING: THIS FILE IS AUTOMATICALLY GENERATED. PLEASE DO NOT ALTER IT" << endl;
	cxxFile << " *" << endl;
	cxxFile << " ***********************************************************************************/" << endl << endl;

	cxxFile << "#include <iostream>" << endl;
	cxxFile << "#include <map>" << endl;
	cxxFile << "#include <vector>" << endl;

	cxxFile << endl;
	cxxFile << "using namespace std;" << endl;
	cxxFile << "using std::map;" << endl << endl;
	
	cxxFile << "#undef DEBUG_MAIN			// Test program" << endl;
	cxxFile << "#undef DEBUG_CALC			// Printing the Calculations" << endl;
	cxxFile << "#undef DEBUG_SEARCH_DIST	// Debugging the LookupUrgencyfromDist function" << endl;
	cxxFile << "#undef DEBUG_SEARCH_TIME	// Debugging the LookupUrgencyfromTime function" << endl;
	cxxFile << "#undef DEBUG_SEARCH_VEL		// Debugging the LookupVelocityfromDist function" << endl;

	cxxFile << endl << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Mapping function" << endl;
	cxxFile << "//" << endl;
	cxxFile << "static int solIdIndex=0;" << endl;
	cxxFile << "map<int,int> solIdToIndexMap; // solIdIndex -> solId" << endl << endl;

	cxxFile << "//" << endl;
	cxxFile << "// Longitudinal Distance" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// [" << totalObjs << "] corresponds to various SOL MODELS" << endl;
	cxxFile << "// [" << cMAX_ROWS << "] corresponds to Velocities from 5mph - 120mph" << endl;
	cxxFile << "// [" << cMAX_COLS << "] corresponds to Urgencies from 0.1 - 1.0" << endl;
	
	


	cxxFile << "const double cLONGDIST[" << totalObjs << "][" << cMAX_ROWS << "][" << cMAX_COLS << "] = " << endl; 
	cxxFile.setf(ios::fixed, ios::floatfield);     
	cxxFile.precision(2);
	cxxFile << "{" << endl;

	//
	// Printing the array in the CXX file  
	//
	int sol;			// # of Sol Objects in the array
	int rows, columns;	// rows and columns in the array
	
	for ( sol = 0; sol < totalObjs; sol++ )
	{
		// Mapping Comes here
		string slName = IndexToSolNameMap[sol];
		// Sol Model gets printed here
		cxxFile << "	/***** " << slName << " - " 
			    << cved->GetSol().GetObj(slName)->GetId() << " *****/" << endl; 
		cxxFile << "	{" << endl;

		for ( rows = 0; rows < cMAX_ROWS; rows++)
		{
			cxxFile << "		{ ";
			for ( columns = 0; columns < cMAX_COLS; columns++)
			{
				cxxFile << LongDist[sol][rows][columns];
				
				if ( columns != (cMAX_COLS - 1) ) {
					cxxFile << ", ";
				}
				else if ( rows != (cMAX_ROWS - 1) ){
					if (columns == (cMAX_COLS - 1) ) {
						cxxFile << " }, " << endl;
					}
				}
				else if (rows == (cMAX_ROWS - 1) && columns == (cMAX_COLS - 1)){
					cxxFile << " } " << endl;
				}
			}
			if ( rows == (cMAX_ROWS - 1) && sol != totalObjs ) {
				cxxFile << "	}," << endl << endl;
			}
			else if ( rows == (cMAX_COLS - 1) && sol == totalObjs ) {
				cxxFile << "	}" << endl << endl;
			}
		}
	}
	cxxFile << "};" << endl<< endl;
	
		
	cxxFile << "//" << endl;
	cxxFile << "// Time Taken" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// [" << totalObjs << "] corresponds to various SOL MODELS" << endl;	
	cxxFile << "// [" << cMAX_ROWS << "] corresponds to Velocities from 5mph - 120mph" << endl;
	cxxFile << "// [" << cMAX_COLS << "] corresponds to Urgencies from 0.1 - 1.0" << endl << endl;
		
	cxxFile << "const double cTIME[" << totalObjs << "][" << cMAX_ROWS << "][" << cMAX_COLS << "] = " << endl;
	cxxFile.setf(ios::fixed, ios::floatfield);     
	cxxFile.precision(2);
	cxxFile << "{" << endl;
	
	for ( sol = 0; sol < totalObjs; sol++ )
	{
		// Mapping Comes here
		string slName = IndexToSolNameMap[sol];
		// Sol Model gets printed here
		cxxFile << "	/***** " << slName << " - " 
			    << cved->GetSol().GetObj(slName)->GetId() << " *****/" << endl; 		
		cxxFile << "	{" << endl;
		for ( rows = 0; rows < cMAX_ROWS; rows++)
		{
			cxxFile << "		{ ";
			for ( columns = 0; columns <cMAX_COLS; columns++)
			{
				cxxFile << Time[sol][rows][columns];
				
				if ( columns != (cMAX_COLS - 1) ) { //(cMAX_ROWS - 1) (cMAX_COLS - 1)
					cxxFile << ", ";
				}
				else if ( rows != (cMAX_ROWS - 1) ){
					if (columns == (cMAX_COLS - 1) ) {
						cxxFile << " }, " << endl;
					}
				}
				else if (rows == (cMAX_ROWS - 1) && columns == (cMAX_COLS - 1)){
					cxxFile << " } " << endl;
				}
			}
			if ( rows == (cMAX_ROWS - 1) && sol != totalObjs ) {
				cxxFile << "	}," << endl << endl;
			}
			else if ( rows == (cMAX_ROWS - 1) && sol == totalObjs ) {
				cxxFile << "	}" << endl << endl;
			}
		}
	}
	cxxFile << "};" << endl<< endl;
		
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Description: Given velocity and urgency, this function queries and " << endl;
	cxxFile << "//				interpolates time and distance taken to execute a lanchange" << endl;
	cxxFile << "// Remarks:" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Arguments:" << endl; 
	cxxFile << "//   velocity	- velocity in 'mph' ranging from 5.0 - 120.0 mph" << endl;
	cxxFile << "//   urgency	- urgency ranging from 0.1 - 1.0" << endl;
	cxxFile << "//   solId		- SolId of vehicle" << endl;
	cxxFile << "//   longDist	- (output) The distance travelled to complete the lane" << endl;
	cxxFile << "//				   change (in feet)." << endl;
	cxxFile << "//   time		- (output) Time taken to complete lanechange  " << endl;
	cxxFile << "//				  (in seconds)." << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Returns: A boolean to indicate if distance and time were found." << endl;
	cxxFile << "//" << endl;
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "bool" << endl;
	cxxFile << "LcLookupDistTime(" << endl;
	cxxFile << "			double velocity, " << endl;
	cxxFile << "			double urgency, " << endl;
	cxxFile << "			int solId, " << endl;
	cxxFile << "			double& longDist, " << endl;
	cxxFile << "			double& time " << endl;
	cxxFile << "			)" << endl;
	cxxFile << "{" << endl;
	cxxFile << "	// Adding  solid to map" << endl;
	
	int solIdCounter = 0;
	for(itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++)
	{
		//
		// Starting the SolId Index from the Map and using to insert values into the array
		//
		cxxFile << "	solIdToIndexMap[" << *itr << "] = " << solIdCounter << ";" << endl;			
		solIdCounter++;

	}

	cxxFile << endl << endl;	
	cxxFile << "	// Getting the position of the particluar SolModel in the array " << endl;
	cxxFile << "	solIdIndex = solIdToIndexMap[solId]; " << endl << endl;
			
	cxxFile << "	// Testing the input velocity" << endl;	
	cxxFile << "	bool velLimits = (velocity >= 5.0 && velocity <= 120.0);" << endl;
	cxxFile << "	// Testing the input urgency"	<< endl;
	cxxFile << "	bool urgLimits = (urgency >= 0.1 && urgency <= 1.0);" << endl;	
	cxxFile << "	// Both velLimits and urgLimits have to be satified" << endl;
	cxxFile << "	bool withinLimits = (velLimits && urgLimits); " << endl << endl;

	cxxFile << "	if (withinLimits)" << endl;
	cxxFile << "	{" << endl;
	cxxFile << "		// " << endl;
	cxxFile << "		// velocity is a multiple of 5" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		if ( int(velocity)%5 == 0 )" << endl;
	cxxFile << "		{" << endl;
	cxxFile << "			int V1 = int(velocity/5 - 1); " << endl;
	cxxFile << "			double Urg = (urgency * 10);" << endl;
	cxxFile << "			longDist = cLONGDIST[solIdIndex][V1][int(Urg - 1)];" << endl;
	cxxFile << "			time = cTIME[solIdIndex][V1][int(Urg - 1)];" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		// velocity is NOT a multiple of 5 " << endl;
	cxxFile << "		// example:23, 57,74 etc" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		else {" << endl;
	cxxFile << "			int V1 = int((velocity)/5 -1);" << endl;
	cxxFile << "			int V2 = V1 + 1;" << endl;
	cxxFile << "			double Urg = (urgency * 10);" << endl;
	cxxFile << "			double diffDist = cLONGDIST[solIdIndex][V2][int(Urg - 1)] - cLONGDIST[solIdIndex][V1][int(Urg - 1)];" << endl;
	cxxFile << "			double diffTime = cTIME[solIdIndex][V2][int(Urg - 1)] - cTIME[solIdIndex][V1][int(Urg - 1)];" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		// Longitudinal Distance" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		longDist = cLONGDIST[solIdIndex][V1][int(Urg - 1)] " << endl;
	cxxFile << "			+ ( diffDist * (velocity - (double(V1) * 5.0 + 5.0) ) " << endl;
	cxxFile << "			/ ((double(V2) * 5.0 + 5.0) - (double(V1) * 5.0 + 5.0)) ); " << endl;
	cxxFile << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		// Time" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		time = cTIME[solIdIndex][V1][int(Urg - 1)]" << endl;
	cxxFile << "			+ ( diffTime * (velocity - (double(V1) * 5.0 + 5.0) ) " << endl;
	cxxFile << "			/ ((double(V2) * 5.0 + 5.0) - (double(V1) * 5.0 + 5.0)) ); " << endl;
	cxxFile << endl;
	
	cxxFile << "#ifdef DEBUG_CALC" << endl;
	cxxFile << "		gout << V1 << \",\" << int(Urg - 1) << endl;" << endl;
	cxxFile << "		gout << V2 << \",\" << int(Urg - 1) << endl << endl;" << endl;
	cxxFile << "		gout << cLONGDIST[solIdIndex][V2][int(Urg - 1)] << \" - \";" << endl;
	cxxFile << "		gout << cLONGDIST[solIdIndex][V1][int(Urg - 1)] << \" = \" << diffDist << endl;" << endl;
	cxxFile << "		gout << \"Vel-1 \" << (double(V1) * 5.0 + 5.0) << endl;" << endl;
	cxxFile << "		gout << \"Vel-2 \" << (double(V2) * 5.0 + 5.0) << endl;" << endl;
	cxxFile << "		gout << \"Diff: \" << ((double(V2) * 5.0 + 5.0) - (double(V1) * 5.0 + 5.0));" << endl;
	cxxFile << "		gout << endl << endl; " << endl;
	cxxFile << endl;	
	cxxFile << "		gout << cTIME[solIdIndex][V2][int(Urg - 1)] << \" - \";" << endl;

	cxxFile << "		gout << cTIME[solIdIndex][V1][int(Urg - 1)] << \" = \" << diffTime << endl;" << endl;

	cxxFile << "		gout << \"Vel-1 \" << (double(V1) * 5.0 + 5.0) << endl;" << endl;
	cxxFile << "		gout << \"Vel-2 \" << (double(V2) * 5.0 + 5.0) << endl;" << endl;
	cxxFile << "		gout << \"Diff: \" << ((double(V2) * 5.0 + 5.0) - (double(V1) * 5.0 + 5.0)) << endl;" << endl;
	cxxFile << "		gout << \"Long Dist: \" << longDist << \"ft\"<< endl;" << endl;
	cxxFile << "		gout << \"Time: \"<< time << \"seconds\" << endl;" << endl;
	cxxFile << "#endif //DEBUG_CALC" << endl;
	cxxFile << endl;
	cxxFile << "			}" << endl; 
	cxxFile << "	}" << endl;
	cxxFile << "	else {" << endl;
	cxxFile << "		withinLimits = false;" << endl;
	cxxFile << "	}" << endl;
	cxxFile << "	return withinLimits;" << endl;
	cxxFile << "} // End of LcLookupDistTime() " << endl << endl;

	
	
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Description: This function when distance and solId are given queries the data " << endl;
	cxxFile << "//				tables and returns the velocity and urgency with which the " << endl;
	cxxFile << "//				desired distance can be achieved. If the desired distance " << endl;
	cxxFile << "//				is not exactly matched, it will give the velocity of that " << endl;
	cxxFile << "//				distance that is closest to the input distance.  " << endl;
	cxxFile << "// Remarks:" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Arguments:" << endl; 
	cxxFile << "//   dist		- distance travelled to execute a lanechange (in feet)" << endl;
	cxxFile << "//   urgency	- urgency ranging from 0.1 - 1.0" << endl;
	cxxFile << "//   solId		- SolId of vehicle" << endl;
	cxxFile << "//   velocity	- (output) The velocity needed to complete the lane" << endl;
	cxxFile << "//    			  change in given distance (in mph)." << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Returns: A boolean to indicate if velocity found." << endl;
	cxxFile << "//" << endl;
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "bool " << endl;
	cxxFile << "LcLookupVelocityfromDist(" << endl;
	cxxFile << "				double dist," << endl;
	cxxFile << "				double urgency," << endl;
	cxxFile << "				int solId, " << endl;
	cxxFile << "				double& velocity " << endl;
	cxxFile << "				)" << endl;
	cxxFile << "{" << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	// Checking inputs " << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	bool withinLimits = (dist >= 0.0 && urgency >= 0.1 && urgency <= 1.0);" << endl;
	cxxFile << "	bool matchFound = false;" << endl;
	cxxFile << "	if (withinLimits) " << endl;
	cxxFile << "	{ " << endl;

	solIdCounter = 0;
	for(itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++)
	{
		//
		// Starting the SolId Index from the Map and using to insert values into the array
		//
		cxxFile << "	solIdToIndexMap[" << *itr << "] = " << solIdCounter << ";" << endl;			
		solIdCounter++;

	}

	cxxFile << endl << endl;

	cxxFile << "		// Getting the position of the particluar SolModel in the array " << endl;
	cxxFile << "		solIdIndex = solIdToIndexMap[solId]; " << endl << endl;

	cxxFile << "		// Coverting the urgency into a position in the array" << endl;
	cxxFile << "		int urgCounter = int(urgency*10) - 1; " << endl << endl;
	cxxFile << "		for (int velCounter = " << (cMAX_ROWS - 1) << " ; velCounter >= 0; velCounter--) " << endl;
	cxxFile << "		{ " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			// Exact Match with the dist in the Array" << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			bool exactMatch = ((dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) > -0.05 &&" << endl;
	cxxFile << "				(dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) < 0.00); " << endl << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			// Need a velocity that makes LC in less distance than actual dist to intrscn " << endl;
	cxxFile << "			// 5ft less in this case " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			bool matchTier1 = ((dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) > 0.00 && " << endl;
	cxxFile << "				(dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) < 5.00); " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			// 10ft less in this case " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			bool matchTier2 = ((dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) > 5.00 && " << endl;
	cxxFile << "				(dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) < 15.00); " << endl << endl;

	cxxFile << "			if (exactMatch) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				velocity = double(velCounter + 1) * 5.0f; " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "				gout << endl << \"Dist-exactMatch---velocity: \" << velocity << \"  velCounter: \";  " << endl;
	cxxFile << "				gout << velCounter << \"   UrgCounter: \"; " << endl;
	cxxFile << "				gout << urgCounter << endl <<  endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "				matchFound = exactMatch;" << endl;
	cxxFile << "				break; " << endl;
	cxxFile << "			} " << endl; 
	cxxFile << "			else if (matchTier1) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				velocity = double(velCounter + 1) * 5.0f; " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "				gout << \"Dist-matchTier1---velocity: \" << velocity << \"  velCounter: \"; " << endl;
	cxxFile << "				gout << velCounter << \"   UrgCounter: \"; " << endl;
	cxxFile << "				gout << urgCounter << endl <<  endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "				matchFound = matchTier1;" << endl;
	cxxFile << "				break; " << endl;
	cxxFile << "			} " << endl;
	cxxFile << "			else if (matchTier2) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				velocity = double(velCounter + 1) * 5.0f; " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "				gout << \"Dist-matchTier2---velocity: \" << velocity << \"  velCounter:\";  " << endl;
	cxxFile << "				gout << velCounter << \"   UrgCounter: \"; " << endl;
	cxxFile << "				gout << urgCounter << endl <<  endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "				matchFound = matchTier2;" << endl;
	cxxFile << "				break; " << endl;
	cxxFile << "			} " << endl;
	cxxFile << "		} " << endl;
	cxxFile << "	}  " << endl;
	cxxFile << "	return matchFound; " << endl;
	cxxFile << "} // End of LcLookupVelocityfromDist() " << endl << endl;


	cxxFile << "//////////////////////////////////////////////////////////////////////////// " << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Description: Overloaded function with extra parameter as the " << endl;
	cxxFile << "//				reference to a vector having list of matched " << endl;
	cxxFile << "//				velocities. When dist, solId  are given," << endl;
	cxxFile << "//				this function returns the velocity with " << endl;
	cxxFile << "//				which lane change can be achieved in the.  " << endl;
	cxxFile << "//				desired distance." << endl;
	cxxFile << "// Remarks:" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Arguments:" << endl; 
	cxxFile << "//   dist		- distance travelled to execute a lanechange (in feet)" << endl;
	cxxFile << "//   solId		- SolId of vehicle" << endl;
	cxxFile << "//   velocity	- (output) Matched velocities needed to complete the lane" << endl;
	cxxFile << "//    			  change in given distance are stored in this vector (in mph)." << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Returns: A boolean to indicate if velocity found." << endl;
	cxxFile << "//" << endl;
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "bool  " << endl;
	cxxFile << "LcLookupVelocityfromDist( " << endl;
	cxxFile << "				double dist,  " << endl;
	cxxFile << "				int solId,  " << endl;
	cxxFile << "				vector<vector<double> >& v " << endl;
	cxxFile << "				) " << endl;
	cxxFile << "{ " << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	// Checking inputs " << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	bool withinLimits = (dist >= 0.0); " << endl;
	cxxFile << "	bool matchFound = false; " << endl;
	cxxFile << "	if(withinLimits) " << endl;
	cxxFile << "	{ " << endl;
	cxxFile << "		// Adding  solid to map " << endl;

	solIdCounter = 0;
	for(itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++)
	{
		//
		// Starting the SolId Index from the Map and using to insert values into the array
		//
		cxxFile << "	solIdToIndexMap[" << *itr << "] = " << solIdCounter << ";" << endl;			
		solIdCounter++;

	}

	cxxFile << endl << endl;

	cxxFile << "		// Getting the position of the particluar SolModel in the array " << endl;
	cxxFile << "		solIdIndex = solIdToIndexMap[solId];  " << endl;
	cxxFile << "		 " << endl;
	cxxFile << "		// " << endl;
	cxxFile << "		// Vector that holds the matching vel-urg pair " << endl;
	cxxFile << "		// " << endl;
	cxxFile << "		vector<double> matchPair; " << endl << endl;
	cxxFile << "		for (int velCounter = " << (cMAX_ROWS - 1) << " ; velCounter >= 0; velCounter--) " << endl;
	cxxFile << "		{ " << endl;
	cxxFile << "			for (int urgCounter = " << (cMAX_COLS - 1) << "; urgCounter >= 0; urgCounter--) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				// Exact Match with the dist in the Array " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				bool exactMatch = ((dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) > -0.01 && " << endl;
	cxxFile << "					(dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) <= 0.00); " << endl << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				// Need a velocity that makes LC in less distance than actual dist to intrscn " << endl;
	cxxFile << "				// 5ft less in this case " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				bool matchTier1 = ((dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) > 0.00 && " << endl;
	cxxFile << "					(dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) < 5.00); " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				// 10ft less in this case " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				bool matchTier2 = ((dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) > 1.00 && " << endl;
	cxxFile << "					(dist - cLONGDIST[solIdIndex][velCounter][urgCounter]) < 15.00); " << endl << endl;

	cxxFile << "				if (exactMatch) " << endl;
	cxxFile << "				{ " << endl;
	cxxFile << "					// " << endl;
	cxxFile << "					// Pushing the velocity and urgency into the vector matchPair " << endl;
	cxxFile << "					// " << endl;
	cxxFile << "					matchPair.push_back(double(velCounter + 1) * 5.0f); " << endl;
	cxxFile << "					matchPair.push_back(double(urgCounter + 1) / 10.0f); " << endl << endl;
	cxxFile << "					// Pushing this pair vector into the main vector " << endl;
	cxxFile << "					v.push_back(matchPair); " << endl << endl;
	cxxFile << "					// Clearing the vector " << endl;
	cxxFile << "					matchPair.clear(); " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "					gout << \"Dist-ExactMatch  \";  " << endl;
	cxxFile << "					gout << \"vel: \" << double(velCounter + 1) * 5.0f << \" urg: \"; " << endl;
	cxxFile << "					gout << double(urgCounter + 1) / 10.0f; " << endl;
	cxxFile << "					gout << \"   Vector: \" << matchPair[0] << \"|\" << matchPair[1] << endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "					matchFound = exactMatch; " << endl;
	cxxFile << "					exactMatch = false; " << endl;
	cxxFile << "				} " << endl;
	cxxFile << "				else if(matchTier1) " << endl;
	cxxFile << "				{ " << endl;
	cxxFile << "					matchPair.push_back(double(velCounter + 1) * 5.0f); " << endl;
	cxxFile << "					matchPair.push_back(double(urgCounter + 1) / 10.0f); " << endl << endl;
	cxxFile << "					// Pushing this pair vector into the main vector " << endl;
	cxxFile << "					v.push_back(matchPair); " << endl << endl;
	cxxFile << "					// Clearing the vector " << endl;
	cxxFile << "					matchPair.clear(); " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "					gout << \"Dist-matchTier1  \"; " << endl;
	cxxFile << "					gout << \"vel: \" << double(velCounter + 1) * 5.0f << \" urg: \"; " << endl;
	cxxFile << "					gout << double(urgCounter + 1) / 10.0f; " << endl;
	cxxFile << "					gout << \"   Vector: \" << matchPair[0] << \"|\" << matchPair[1] << endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "					matchFound = matchTier1; " << endl;
	cxxFile << "					matchTier1 = false; " << endl;
	cxxFile << "				} " << endl;
	cxxFile << "				else if (matchTier2) " << endl;
	cxxFile << "				{ " << endl;
	cxxFile << "					matchPair.push_back(double(velCounter + 1) * 5.0f); " << endl;
	cxxFile << "					matchPair.push_back(double(urgCounter + 1) / 10.0f); " << endl << endl;
	cxxFile << "					// Pushing this pair vector into the main vector " << endl;
	cxxFile << "					v.push_back(matchPair); " << endl << endl;
	cxxFile << "					// Clearing the vector " << endl;
	cxxFile << "					matchPair.clear(); " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "					gout << \"Dist-matchTier2  \"; " << endl;
	cxxFile << "					gout << \"vel: \" << double(velCounter + 1) * 5.0f << \" urg: \"; " << endl;
	cxxFile << "					gout << double(urgCounter + 1) / 10.0f; " << endl;
	cxxFile << "					gout << \"   Vector: \" << matchPair[0] << \"|\" << matchPair[1] << endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "					matchFound = matchTier2; " << endl;
	cxxFile << "					matchTier2 = false; " << endl;
	cxxFile << "				} " << endl;
	cxxFile << "			} " << endl;
	cxxFile << "		} " << endl;
	cxxFile << "	} " << endl;
	cxxFile << "	else { " << endl;
	cxxFile << "		matchFound = false;" << endl;
	cxxFile << "	} " << endl;
	cxxFile << "	return matchFound; " << endl;
	cxxFile << "} // End of LcLookupVelocityfromDist() " << endl << endl;



	cxxFile << "/////////////////////////////////////////////////////////////////////////////// " << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Description: This function when time, urgency and solId are " << endl;
	cxxFile << "//				given, queries the data tables, interpolates " << endl;
	cxxFile << "//				and returns the velocity to needed to complete the" << endl;
	cxxFile << "//				lanechange in the desired time" << endl;
	cxxFile << "//				If the desired time is not exactly matched,it " << endl;
	cxxFile << "//				will give the velocity of that time that is closest " << endl;
	cxxFile << "//				to the input time " << endl;
	cxxFile << "// Remarks:" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Arguments:" << endl; 
	cxxFile << "//   time		- time taken to execute a lanechange (in seconds)" << endl;
	cxxFile << "//   urgency	- urgency ranging from 0.1 - 1.0" << endl;
	cxxFile << "//   solId		- SolId of vehicle" << endl;
	cxxFile << "//   velocity	- (output) Matched velocity needed to complete the lane" << endl;
	cxxFile << "//    			  change in given time(in mph)." << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Returns: A boolean to indicate if velocity found." << endl;
	cxxFile << "//" << endl;
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "bool  " << endl;
	cxxFile << "LcLookupVelocityfromTime( " << endl;
	cxxFile << "				double time,  " << endl;
	cxxFile << "				double urgency, " << endl;
	cxxFile << "				int solId,  " << endl;
	cxxFile << "				double& velocity " << endl;
	cxxFile << "				) " << endl;
	cxxFile << "{ " << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	// Checking inputs " << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	bool withinLimits = (time >= 0.0 && urgency >= 0.1 && urgency <= 1.0); " << endl << endl;
	cxxFile << "	bool matchFound = false;" << endl; 

	cxxFile << "	if(withinLimits) " << endl;
	cxxFile << "	{ " << endl;
	cxxFile << "		// Adding  solid to map " << endl;

	solIdCounter = 0;
	for(itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++)
	{
		//
		// Starting the SolId Index from the Map and using to insert values into the array
		//
		cxxFile << "	solIdToIndexMap[" << *itr << "] = " << solIdCounter << ";" << endl;			
		solIdCounter++;
	}

	cxxFile << endl << endl;

	cxxFile << "		// Getting the position of the particluar SolModel in the array  " << endl;
	cxxFile << "		solIdIndex = solIdToIndexMap[solId];  " << endl << endl;

	cxxFile << "		// Coverting the urgency into a position in the array " << endl;
	cxxFile << "		int urgCounter = int(urgency*10) - 1; " << endl;
	cxxFile << "		for (int velCounter = " << (cMAX_ROWS - 1) << "; velCounter >= 0; velCounter--) " << endl;
	cxxFile << "		{ " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			// Exact Match with the dist in the Array " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			bool exactMatch = ((time - cTIME[solIdIndex][velCounter][urgCounter]) > -0.01 && " << endl;
	cxxFile << "				(time - cTIME[solIdIndex][velCounter][urgCounter]) <= 0.00); " << endl << endl;

	cxxFile << "			// " << endl;
	cxxFile << "			// Need a velocity that makes LC in less distance than actual time to intrscn " << endl;
	cxxFile << "			// 5ft less in this case " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			bool matchTier1 = ((time - cTIME[solIdIndex][velCounter][urgCounter]) > 0.00 && " << endl;
	cxxFile << "				(time - cTIME[solIdIndex][velCounter][urgCounter]) < 1.00); " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			// 10ft less in this case " << endl;
	cxxFile << "			// " << endl;
	cxxFile << "			bool matchTier2 = ((time - cTIME[solIdIndex][velCounter][urgCounter]) > 1.00 && " << endl;
	cxxFile << "				(time - cTIME[solIdIndex][velCounter][urgCounter]) < 1.50); " << endl << endl;

	cxxFile << "			if (exactMatch) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				velocity = double(velCounter + 1) * 5.0f; " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "				gout << endl << \"Time-exactMatch---velocity: \" << velocity << \"  velCounter: \";  " << endl;
	cxxFile << "				gout << velCounter << \"   UrgCounter: \"; " << endl;
	cxxFile << "				gout << urgCounter << endl <<  endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "				matchFound = exactMatch; " << endl;
	cxxFile << "				break; " << endl;
	cxxFile << "			} " << endl;
	cxxFile << "			else if (matchTier1) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				velocity = double(velCounter + 1) * 5.0f; " << endl;
	cxxFile << "				urgency = double(urgCounter + 1) / 10.0f; " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "				gout << \"Time-matchTier1---velocity: \" << velocity << \"  velCounter: \";  " << endl;
	cxxFile << "				gout << velCounter << \"   UrgCounter: \"; " << endl;
	cxxFile << "				gout << urgCounter << endl <<  endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "				matchFound = matchTier1; " << endl;
	cxxFile << "				break; " << endl;
	cxxFile << "			} " << endl;
	cxxFile << "			else if (matchTier2) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				velocity = double(velCounter + 1) * 5.0f; " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "				gout << \"Time--matchTier2---velocity: \" << velocity << \"  velCounter: \";  " << endl;
	cxxFile << "				gout << velCounter << \"   UrgCounter: \"; " << endl;
	cxxFile << "				gout << urgCounter << endl << endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "				matchFound = matchTier2; " << endl;
	cxxFile << "				break; " << endl;
	cxxFile << "			} " << endl;
	cxxFile << "		} " << endl;
	cxxFile << "	} " << endl;
	cxxFile << "	else " << endl;
	cxxFile << "	{" << endl;
	cxxFile << "		matchFound = false; " << endl;
	cxxFile << "	}" << endl;
	cxxFile << "	return matchFound; " << endl;
	cxxFile << "} // End of LcLookupVelocityfromTime() " << endl << endl;


	cxxFile << "///////////////////////////////////////////////////////////////////////////// " << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Description: Overloaded function with extra parameter as the " << endl;
	cxxFile << "//				reference to a vector having list of matched " << endl;
	cxxFile << "//				velocities. When time and solId  are given," << endl;
	cxxFile << "//				this function returns a vector consisting of " << endl;
	cxxFile << "//				velocities that complete lanechange in the " << endl;
	cxxFile << "//				desired time" << endl;
	cxxFile << "//				If the desired time is not exactly matched, it " << endl;
	cxxFile << "//				will give the velocity of that time that is closest " << endl;
	cxxFile << "//				to the input time " << endl << endl;
	cxxFile << "// Remarks:" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Arguments:" << endl; 
	cxxFile << "//   time		- time taken to execute a lanechange (in seconds)" << endl;
	cxxFile << "//   solId		- SolId of vehicle" << endl;
	cxxFile << "//   velocity	- (output) Matched velocities needed to complete the lane" << endl;
	cxxFile << "//    			  change in desired time are stored in this vector (in mph)." << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Returns: A boolean to indicate if velocity found." << endl;
	cxxFile << "//" << endl;
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "bool  " << endl;
	cxxFile << "LcLookupVelocityfromTime( " << endl;
	cxxFile << "				double time,  " << endl;
	cxxFile << "				int solId,  " << endl;
	cxxFile << "				vector <vector<double> > &v " << endl;
	cxxFile << "				) " << endl;
	cxxFile << "{ " << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	// Checking inputs " << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	bool withinLimits = (time >= 0.0); " << endl;
	cxxFile << "	bool matchFound = false; " << endl;
	cxxFile << "	if (withinLimits) " << endl;
	cxxFile << "	{ " << endl;

	cxxFile << "		// Adding  solid to map " << endl;
	solIdCounter = 0; 
	for(itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++)
	{
		//
		// Starting the SolId Index from the Map and using to insert values into the array
		//
		cxxFile << "	solIdToIndexMap[" << *itr << "] = " << solIdCounter << ";" << endl;			
		solIdCounter++;
	}

	cxxFile << endl << endl;

	cxxFile << "		// Getting the position of the particluar SolModel in the array  " << endl;
	cxxFile << "		solIdIndex = solIdToIndexMap[solId];  " << endl << endl;

	cxxFile << "		// " << endl;
	cxxFile << "		// Pair vector to hold the matching vel and urg " << endl;
	cxxFile << "		// " << endl;
	cxxFile << "		vector<double> matchPair; " << endl << endl;
	cxxFile << "		for (int velCounter = " << (cMAX_ROWS - 1) << "; velCounter >= 0; velCounter--) " << endl;
	cxxFile << "		{ " << endl;
	cxxFile << "			for (int urgCounter = " << (cMAX_COLS - 1) << "; urgCounter >= 0; urgCounter--) " << endl;
	cxxFile << "			{ " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				// Exact Match with the time in the TIME Array " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				bool exactMatch = ((time - cTIME[solIdIndex][velCounter][urgCounter]) > -0.01 && " << endl;
	cxxFile << "					(time - cTIME[solIdIndex][velCounter][urgCounter]) <= 0.00); " << endl << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				// Need a velocity that makes LC in less time than actual time to intrscn " << endl;
	cxxFile << "				// 0.5 second less in this case " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				bool matchTier1 = ((time - cTIME[solIdIndex][velCounter][urgCounter]) > 0.00 && " << endl;
	cxxFile << "					(time - cTIME[solIdIndex][velCounter][urgCounter]) < 0.50); " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				// Between 1.0 and 1.5 secs " << endl;
	cxxFile << "				// " << endl;
	cxxFile << "				bool matchTier2 = ((time - cTIME[solIdIndex][velCounter][urgCounter]) > 1.00 && " << endl;
	cxxFile << "					(time - cTIME[solIdIndex][velCounter][urgCounter]) < 1.00); " << endl << endl;

	cxxFile << "				if (exactMatch) " << endl;
	cxxFile << "				{ " << endl;
	cxxFile << "					// " << endl;
	cxxFile << "					// Pushing the velocity and urgency into the vector matchPair " << endl;
	cxxFile << "					// " << endl;
	cxxFile << "					matchPair.push_back(double(velCounter + 1) * 5.0f); " << endl;
	cxxFile << "					matchPair.push_back(double(urgCounter + 1) / 10.0f); " << endl << endl;
	cxxFile << "					// Pushing this pair vector into the main vector " << endl;
	cxxFile << "					v.push_back(matchPair); " << endl << endl;

	cxxFile << "					// Clearing the vector " << endl;
	cxxFile << "					matchPair.clear(); " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "					gout << \"Time-ExactMatch  \";  " << endl;
	cxxFile << "					gout << \"vel: \" << double(velCounter + 1) * 5.0f << \" urg: \"; " << endl;
	cxxFile << "					gout << double(urgCounter + 1) / 10.0f; " << endl;
	cxxFile << "					gout << \"   Vector: \" << matchPair[0] << \"|\" << matchPair[1] << endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "					matchFound = exactMatch; " << endl;
	cxxFile << "					exactMatch = false; " << endl << endl;
	cxxFile << "				} " << endl;
	cxxFile << "				else if(matchTier1) " << endl;
	cxxFile << "				{ " << endl;
	cxxFile << "					matchPair.push_back(double(velCounter + 1) * 5.0f); " << endl;
	cxxFile << "					matchPair.push_back(double(urgCounter + 1) / 10.0f); " << endl << endl;
	cxxFile << "					// Pushing this pair vector into the main vector " << endl;
	cxxFile << "					v.push_back(matchPair); " << endl << endl;
	cxxFile << "					// Clearing the vector " << endl;
	cxxFile << "					matchPair.clear(); " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "					gout << \"Time-matchTier1  \"; " << endl;
	cxxFile << "					gout << \"vel: \" << double(velCounter + 1) * 5.0f << \" urg: \"; " << endl;
	cxxFile << "					gout << double(urgCounter + 1) / 10.0f; " << endl;
	cxxFile << "					gout << \"   Vector: \" << matchPair[0] << \"|\" << matchPair[1] << endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "					matchFound = matchTier1; " << endl;
	cxxFile << "					matchTier1 = false; " << endl << endl;
	cxxFile << "				} " << endl;
	cxxFile << "				else if (matchTier2) " << endl;
	cxxFile << "				{ " << endl;
	cxxFile << "					matchPair.push_back(double(velCounter + 1) * 5.0f); " << endl;
	cxxFile << "					matchPair.push_back(double(urgCounter + 1) / 10.0f); " << endl << endl;

	cxxFile << "					// Pushing this pair vector into the main vector " << endl;
	cxxFile << "					v.push_back(matchPair); " << endl << endl;

	cxxFile << "					// Clearing the vector " << endl;
	cxxFile << "					matchPair.clear(); " << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_VEL " << endl;
	cxxFile << "					gout << \"Time-matchTier2  \"; " << endl;
	cxxFile << "					gout << \"vel: \" << double(velCounter + 1) * 5.0f << \" urg: \"; " << endl;
	cxxFile << "					gout << double(urgCounter + 1) / 10.0f; " << endl;
	cxxFile << "					gout << \"   Vector: \" << matchPair[0] << \"|\" << matchPair[1] << endl; " << endl;
	cxxFile << "#endif " << endl;
	cxxFile << "					matchFound = matchTier2; " << endl;
	cxxFile << "					matchTier2 = false; " << endl << endl;
	cxxFile << "				} " << endl;
	cxxFile << "			} " << endl;
	cxxFile << "		} " << endl;
	cxxFile << "	} " << endl;
	cxxFile << "	else { " << endl;
	cxxFile << "		matchFound = false; " << endl;
	cxxFile << "	} " << endl;
	cxxFile << "	return matchFound; " << endl;
	cxxFile << "} // End of LcLookupVelocityfromTime() " << endl << endl;


	cxxFile << "///////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Description: This function when velocity and distance are " << endl;
	cxxFile << "//				given, queries the data tables and returns the  .  " << endl;
	cxxFile << "//				urgency with which lanechange can be achieved" << endl;
	cxxFile << "//				in the desired distance.If the desired distance, " << endl;
	cxxFile << "//				is not exactly matched, it will give the urgency " << endl; 
	cxxFile << "//				that is closest to the input Distance." << endl;
	cxxFile << "// Remarks:" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Arguments:" << endl; 
	cxxFile << "//   velocity	- velocity in 'mph' ranging from 5.0 - 120.0 mph" << endl;
	cxxFile << "//   longDist	- distance travelled to complete the lane (in feet)" << endl;
	cxxFile << "//   solId		- SolId of vehicle" << endl;
	cxxFile << "//   urgency	- (output) Urgency applied to complete the lane" << endl;
	cxxFile << "//    			  change in given distance ( ranges from 0.1 - 1.0)." << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Returns: A boolean to indicate if urgency was found." << endl;
	cxxFile << "//" << endl;
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "bool" << endl;
	cxxFile << "LcLookupUrgencyfromDist(" << endl;
	cxxFile << "			double velocity, " << endl;
	cxxFile << "			double Dist, " << endl;
	cxxFile << "			int solId, " << endl;
	cxxFile << "			double& urgency " << endl;
	cxxFile << "			)" << endl;
	cxxFile << "{" << endl;
	cxxFile << "	// Adding  solid to map" << endl;

	solIdCounter = 0;
	for(itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++)
	{
		//
		// Starting the SolId Index from the Map and using to insert values into the array
		//
		cxxFile << "	solIdToIndexMap[" << *itr << "] = " << solIdCounter << ";" << endl;			
		solIdCounter++;
	}

	cxxFile << endl << endl;
	cxxFile << "	// Getting the position of the particluar SolModel in the array " << endl;
	cxxFile << "	solIdIndex = solIdToIndexMap[solId]; " << endl;
	cxxFile << "	" << endl;
	cxxFile << "	double tempDist[10];" << endl << endl;
	cxxFile << "	bool withinLimits = (velocity > 0.0 && velocity <= 120.0); // Testing the input velocity;" << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	// velocity is a multiple of 5" << endl;
	cxxFile << "	//" << endl;
	cxxFile << "	if ( int(velocity)%5 == 0 )" << endl;
	cxxFile << "	{" << endl;
	cxxFile << "		int V1 = int(velocity/5 - 1);" << endl << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		// Creating a new array to hold the Distances" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		for ( int i = 0; i < " << cMAX_COLS <<"; i++)" << endl;
	cxxFile << "		{" << endl;
	cxxFile << "			tempDist[i] = cLONGDIST[solIdIndex][V1][i];" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "	}" << endl << endl;
	cxxFile << "	//" << endl;
	cxxFile << "	// velocity is NOT a multiple of 5 " << endl;
	cxxFile << "	// example:23, 57,74 etc" << endl;
	cxxFile << "	//" << endl;
	cxxFile << "	else {" << endl;
	cxxFile << "		int V1 = int((velocity)/5 -1);" << endl;
	cxxFile << "		int V2 = V1 + 1;" << endl << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		// Longitudinal Distance" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		for ( int i = 0; i < 10; i++)" << endl;
	cxxFile << "		{" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			// Interpolating the Distances for a given Velocity and " << endl;
	cxxFile << "			// sending that into another array tempDist" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			double diffDist = cLONGDIST[solIdIndex][V2][i] - cLONGDIST[solIdIndex][V1][i];" << endl;
	cxxFile << "				" << endl;
	cxxFile << "			tempDist[i] = cLONGDIST[solIdIndex][V1][i] " << endl;
	cxxFile << "			+ ( diffDist * (velocity - (double(V1) * 5.0 + 5.0) ) " << endl;
	cxxFile << "				/ ((double(V2) * 5.0 + 5.0) - (double(V1) * 5.0 + 5.0)) ); " << endl << endl;
	cxxFile << "		}" << endl;
	cxxFile << "	}" << endl;
	cxxFile << "	" << endl;
	cxxFile << "	bool exactMatchFound = false; " << endl;
	cxxFile << "	bool functionMatchFound = false;" << endl << endl;
	cxxFile << "	for (int i = 0; i < 10; i++)" << endl;
	cxxFile << "	{ " << endl;
	
	cxxFile << "		//" << endl;
	cxxFile << "		// Dist exactly matches the one given in the table" << endl;
	cxxFile << "		//" << endl;
	
	cxxFile << "		exactMatchFound = ((Dist - tempDist[i]) >= 0.0 && (Dist - tempDist[i]) <= 1.0);" << endl;
	cxxFile << "		if(exactMatchFound) {" << endl;
	cxxFile << "			" << endl;
	cxxFile << "			urgency = (double(i+1))/10;" << endl << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_DIST" << endl;
	cxxFile << "			gout << Dist << \" \" << tempDist[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl << endl;
	cxxFile << "			functionMatchFound = exactMatchFound;" << endl;
	cxxFile << "			break;" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "	}" << endl << endl;

	cxxFile << "	if(!exactMatchFound) " << endl;
	cxxFile << "	{ " << endl;
	cxxFile << "		for (int i = 0; i < 10; i++)" << endl;
	cxxFile << "		{ " << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			// Dist exactly matches the one given in the table" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			bool approxDistMatchTier1 = ((Dist - tempDist[i]) >= -3.0 && (Dist - tempDist[i]) < 0.0);" << endl;
	cxxFile << "			bool approxDistMatchTier2 = ( (Dist -  tempDist[i]) > 1.0 && (Dist - tempDist[i]) <= 2.5);" << endl;
	cxxFile << "			bool approxDistMatchTier3 = ( (Dist -  tempDist[i]) > 2.5 && (Dist - tempDist[i]) <= 5.5);" << endl << endl;
	
	cxxFile << "			if(approxDistMatchTier1) {" << endl << endl;
	cxxFile << "				urgency = (double(i+1))/10;" << endl << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_DIST" << endl;
	cxxFile << "				gout << Dist << \" \" << tempDist[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl << endl;
	cxxFile << "				functionMatchFound = approxDistMatchTier1;" << endl;
	cxxFile << "				break;" << endl;
	cxxFile << "			}" << endl << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			// 1.0 - 2.5 feet greater than the input Dist" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			else if(approxDistMatchTier2) {" << endl << endl; 
	cxxFile << "				urgency = (double(i+1))/10;" << endl << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_DIST" << endl;
	cxxFile << "				gout << Dist << \" \" << tempDist[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl;
	cxxFile << "				functionMatchFound = approxDistMatchTier2;" << endl;
	cxxFile << "				break;" << endl;
	cxxFile << "			}" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			// 2.5 - 5.5 feet greater than the input Dist" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			else if(approxDistMatchTier3) {" << endl << endl;
	cxxFile << "				urgency = (double(i+1))/10;" << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_DIST" << endl;
	cxxFile << "				gout << Dist << \" \" << tempDist[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl;
	cxxFile << "				functionMatchFound = approxDistMatchTier3;" << endl;
	cxxFile << "				break;" << endl;
	cxxFile << "			}" << endl;
	cxxFile << "			else {" << endl;
	cxxFile << "				functionMatchFound = false;" << endl;
	cxxFile << "			}" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "	}" << endl;
	cxxFile << "	return (functionMatchFound && withinLimits);" << endl;
	cxxFile << "} // End of LcLookupUrgencyfromDist() " << endl;
	cxxFile << endl;
	

	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Description: This function when velocity and time are " << endl;
	cxxFile << "//				given, queries the data tables and returns the  .  " << endl;
	cxxFile << "//				urgency with which lanechange can be " << endl;
	cxxFile << "//				achieved in the desired time.If the desired " << endl;
	cxxFile << "//				time is not exactly matched,it will give the urgency " << endl; 
	cxxFile << "//				that is closest to the input time." << endl;
	cxxFile << "// Remarks:" << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Arguments:" << endl; 
	cxxFile << "//   velocity	- velocity in 'mph' ranging from 5.0 - 120.0 mph" << endl;
	cxxFile << "//   time		- time in which to complete the lane (in seconds)" << endl;
	cxxFile << "//   solId		- SolId of vehicle" << endl;
	cxxFile << "//   urgency	- (output) Urgency applied to complete the lane" << endl;
	cxxFile << "//    			  change in given distance ( ranges from 0.1 - 1.0)." << endl;
	cxxFile << "//" << endl;
	cxxFile << "// Returns: A boolean to indicate if urgency was found." << endl;
	cxxFile << "//" << endl;
	cxxFile << "//////////////////////////////////////////////////////////////////////////////" << endl;
	cxxFile << "bool" << endl;
	cxxFile << "LcLookupUrgencyfromTime(" << endl;
	cxxFile << "			double velocity, " << endl;
	cxxFile << "			double time, " << endl;
	cxxFile << "			int solId, " << endl;
	cxxFile << "			double& urgency " << endl;
	cxxFile << "			)" << endl;
	cxxFile << "{" << endl;
	cxxFile << "	// Adding  solid to map" << endl;

	solIdCounter = 0;
	for(itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++)
	{
		//
		// Starting the SolId Index from the Map and using to insert values into the array
		//
		cxxFile << "	solIdToIndexMap[" << *itr << "] = " << solIdCounter << ";" << endl;			
		solIdCounter++;
	}

	
	cxxFile << endl << endl;
	cxxFile << "	// Getting the position of the particluar SolModel in the array " << endl;
	cxxFile << "	solIdIndex = solIdToIndexMap[solId]; " << endl;
	
	cxxFile << "	double tempTime[10];" << endl << endl;
	cxxFile << "	bool withinLimits = (velocity > 0.0 && velocity <= 120.0); // Testing the input velocity;" << endl;
	cxxFile << "	// " << endl;
	cxxFile << "	// velocity is a multiple of 5" << endl;
	cxxFile << "	//" << endl;
	cxxFile << "	if ( int(velocity)%5 == 0 )" << endl;
	cxxFile << "	{" << endl;
	cxxFile << "		int V1 = int(velocity/5 - 1);" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		// Creating a new array to hold the Distances" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		for ( int i = 0; i < " << cMAX_COLS << "; i++)" << endl;
	cxxFile << "		{" << endl;
	cxxFile << "			tempTime[i] = cTIME[solIdIndex][V1][i];" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "	} " << endl;
		
	cxxFile << "	//" << endl;
	cxxFile << "	// velocity is NOT a multiple of 5" << endl; 
	cxxFile << "	// example:23, 57,74 etc" << endl;
	cxxFile << "	//" << endl;
	cxxFile << "	else {" << endl;
	cxxFile << "		int V1 = int((velocity)/5 -1);" << endl;
	cxxFile << "		int V2 = V1 + 1;" << endl;

	cxxFile << "		//" << endl;
	cxxFile << "		// Longitudinal Distance" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		for ( int i = 0; i < 10; i++)" << endl;
	cxxFile << "		{" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			// Interpolating the Distances for a given velocity and " << endl;
	cxxFile << "			// sending that into another array tempDist" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			double diffTime = cTIME[solIdIndex][V2][i] - cTIME[solIdIndex][V1][i];" << endl;
				
	cxxFile << "			tempTime[i] = cTIME[solIdIndex][V1][i] " << endl;
	cxxFile << "			+ ( diffTime * (velocity - (double(V1) * 5.0 + 5.0) ) " << endl;
	cxxFile << "			/ ((double(V2) * 5.0 + 5.0) - (double(V1) * 5.0 + 5.0)) ); " << endl;

	cxxFile << "		}" << endl;
	cxxFile << "	}" << endl << endl;
	cxxFile << "	bool exactMatchFound = false;" << endl;
	cxxFile << "	bool functionMatchFound;" << endl << endl;
	cxxFile << "	for (int i = 0; i < 10; i++)" << endl;
	cxxFile << "	{ " << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		// time exactly matches the one given in the table" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		exactMatchFound = ( (time - tempTime[i]) >= 0.0 && (time - tempTime[i]) <= 0.01);" << endl;
	cxxFile << "		if(exactMatchFound) {" << endl;
	cxxFile << "			urgency = (double(i+1))/10;" << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_TIME" << endl;
	cxxFile << "			gout << time << \" \" << tempTime[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl;
	cxxFile << "			functionMatchFound = exactMatchFound;" << endl;
	cxxFile << "			break;" << endl;
	cxxFile << "		}" << endl;	
	cxxFile << "	} " << endl << endl;

	cxxFile << "	if(!exactMatchFound) " << endl;
	cxxFile << "	{ " << endl;
	cxxFile << "		for (int i = 0; i < 10; i++)" << endl;
	cxxFile << "		{ " << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			// Looking for match between -0.10 < given time < 1.5" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			bool approxTimeMatchTier1 = ( (time - tempTime[i]) >= -0.10 && (time - tempTime[i]) < 0.0);" << endl;
	cxxFile << "			bool approxTimeMatchTier2 = ( (time -  tempTime[i]) > 0.01 && (time - tempTime[i]) <= 0.5);" << endl;
	cxxFile << "			bool approxTimeMatchTier3 = ( (time -  tempTime[i]) > 0.5 && (time - tempTime[i]) <= 1.5);" << endl << endl;
	cxxFile << "			if (approxTimeMatchTier1) {" << endl;
	cxxFile << "				urgency = (double(i+1))/10;" << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_TIME" << endl;
	cxxFile << "				gout << time << \" \" << tempTime[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl;
	cxxFile << "				functionMatchFound = approxTimeMatchTier1;" << endl;
	cxxFile << "				break;" << endl;
	cxxFile << "			}" << endl;			
	cxxFile << "			//" << endl;
	cxxFile << "			// 0.01 - 1.0 seconds greater than the input Dist" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			else if(approxTimeMatchTier2) {" << endl;
	cxxFile << "				urgency = (double(i+1))/10;" << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_TIME" << endl;
	cxxFile << "				gout << time << \" \" << tempTime[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl;
	cxxFile << "				functionMatchFound = approxTimeMatchTier2;" << endl;
	cxxFile << "				break;" << endl;
	cxxFile << "			}" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			// 1.0 - 2.0 seconds greater than the input Dist" << endl;
	cxxFile << "			//" << endl;
	cxxFile << "			else if (approxTimeMatchTier3) {" << endl;
	cxxFile << "				urgency = (double(i+1))/10;" << endl;
	cxxFile << "#ifdef DEBUG_SEARCH_TIME" << endl;
	cxxFile << "				gout << time << \" \" << tempTime[i] << \" \" << i << \" \" << (double(i+1))/10 << endl;" << endl;
	cxxFile << "#endif" << endl;
	cxxFile << "				functionMatchFound = approxTimeMatchTier3;" << endl;
	cxxFile << "				break;" << endl;
	cxxFile << "			}" << endl;	
	cxxFile << "			else {" << endl;
	cxxFile << "				functionMatchFound = false;" << endl;
	cxxFile << "			}" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "	}" << endl;
	cxxFile << "	return (functionMatchFound && withinLimits);" << endl;
	cxxFile << "} // End of LcLookupUrgencyfromTime() " << endl << endl;

 
	cxxFile << "#ifdef DEBUG_MAIN" << endl;

	cxxFile << "	//" << endl;
	cxxFile << "	// Test Program" << endl;
	cxxFile << "	//" << endl;
	cxxFile << "	int main()" << endl;
	cxxFile << "	{" << endl;
	cxxFile << "		double longDist;" << endl;
	cxxFile << "		double time;" << endl;
	cxxFile << "		double urgency;" << endl << endl;

	cxxFile << "		double distVel;" << endl; 
	cxxFile << "		double timeVel;" << endl;
	cxxFile << "		double timeUrg = 0.50f;" << endl;

	cxxFile << "		bool urgencyFromDist = false;" << endl;
	cxxFile << "		bool urgencyFromTime = false;" << endl;

	cxxFile << "		//" << endl;
	cxxFile << "		// Interpolation Function" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		bool testDistUrg = LcLookupDistTime(55.0f, 0.6f, 224, longDist, time );" << endl;

	cxxFile << "		if(testDistUrg) {" << endl;
	cxxFile << "			gout << \"Long Dist: \" << longDist << endl;" << endl;
	cxxFile << "			gout << \"Time: \" << time << endl;" << endl;
	cxxFile << "		} else{ " << endl;
	cxxFile << "			gout << \"OUT OF SCOPE - No Value exists \" << endl; " << endl;
	cxxFile << "		}" << endl;
			
	cxxFile << "		//" << endl;
	cxxFile << "		// Searching Urgency given Dist" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		urgencyFromDist = LcLookupUrgencyfromDist( 55.0f, 356.0f, 224, urgency );" << endl;
	cxxFile << "		if(urgencyFromDist){" << endl;
	cxxFile << "			gout << \"Given Dist, Urgency is: \" << urgency << endl;" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "		else{" << endl;
	cxxFile << "			gout << \"OUT OF SCOPE - No URGENCY value found from Dist!!! \" << endl;" << endl;
	cxxFile << "		}" << endl;

	cxxFile << "		//" << endl;
	cxxFile << "		// Searching Urgency given Time" << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		urgencyFromTime = LcLookupUrgencyfromTime( 55.0f, 4.5f, 224, urgency );" << endl;
	cxxFile << "		if(urgencyFromTime){" << endl;
	cxxFile << "			gout << \"Given Time, Urgency is: \" << urgency << endl;" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "		else{" << endl;
	cxxFile << "			gout << \"OUT OF SCOPE - No URGENCY value found from Time!!! \" << endl;" << endl;
	cxxFile << "		}" << endl;

	cxxFile << "		// Search Velocity from LONGDIST array" << endl << endl;

	cxxFile << "		bool velocityFromDist = LcLookupVelocityfromDist( 379.80f, 0.50f, 14, distVel);" << endl;
	cxxFile << "		if (velocityFromDist) {" << endl << endl;
	cxxFile << "			//gout << \"Dist---Velocity: \" << distVel << endl << endl;" << endl;
	cxxFile << "		}" << endl << endl;

	cxxFile << "		//" << endl;
	cxxFile << "		// Vector to hold the matching vel-urg pairs." << endl;
	cxxFile << "		//" << endl;
	cxxFile << "		vector <vector<double> > distVelUrg;" << endl;
	cxxFile << "		bool velocityFromDistVector = LcLookupVelocityfromDist( 379.80f, 14, distVelUrg );" << endl;
	cxxFile << "		if (velocityFromDistVector)" << endl;
	cxxFile << "		{" << endl;
	cxxFile << "			for (int i = 0; i < distVelUrg.size(); i++)" << endl;
	cxxFile << "			{" << endl;
	cxxFile << "				for ( int j = 0; j < distVelUrg[i].size(); j++)" << endl;
	cxxFile << "				{" << endl;
	cxxFile << "					gout << distVelUrg[i][j] << \"|\";" << endl;
	cxxFile << "				}" << endl;
	cxxFile << "				gout << endl;" << endl;
	cxxFile << "			}	" << endl;
	cxxFile << "		}" << endl;
	cxxFile << "		gout << endl << endl;" << endl << endl;

	cxxFile << "		// Search Velocity from TIME array" << endl;
	cxxFile << "		bool velocityFromTime = LcLookupVelocityfromTime( 7.1f, 0.50f, 14, timeVel);" << endl;
	cxxFile << "		if (velocityFromTime) {" << endl << endl;

	cxxFile << "			//gout << \"Time---Velocity: \" << timeVel << endl;" << endl;
	cxxFile << "		}" << endl << endl;

	cxxFile << "		vector <vector<double> > timeVelUrg;" << endl;
	cxxFile << "		bool velocityFromTimeVector = LcLookupVelocityfromTime( 7.1f, 14, timeVelUrg);" << endl;
	cxxFile << "		if (velocityFromTimeVector)" << endl;
	cxxFile << "		{" << endl;
	cxxFile << "			for (int i = 0; i < timeVelUrg.size(); i++)" << endl;
	cxxFile << "			{" << endl;
	cxxFile << "				for ( int j = 0; j < timeVelUrg[i].size(); j++)" << endl;
	cxxFile << "				{" << endl;
	cxxFile << "					gout << timeVelUrg[i][j] << \"|\";" << endl;
	cxxFile << "				}" << endl;
	cxxFile << "				gout << endl;" << endl;
	cxxFile << "			} " << endl;
	cxxFile << "		} " << endl << endl << endl;

	cxxFile << "		return 0;" << endl;
	cxxFile << "	}" << endl;

	cxxFile << "#endif // DEBUG_MAIN" << endl;
	
	cxxFile.clear();
	cxxFile.close();

}
///////////////////////////////////////////////////////////////////////////////
//
// Description:  Moves the .cxx and .h files to the respective folders
//
// Remarks:
//
// Arguments:
//   pDataPath - The path to the main data directory.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void 
MoveCodeFiles( const char* pDataPath )
{	
	//
	// Move the H and CXX file to 'sabeh/hcsm/usersrc'.
	//
	char command[4096];
	strcpy( command, "move ado_lc_data.h " );
	strcat( command, pDataPath );
	strcat( command, "..\\sabeh\\hcsm\\usersrc" );
	system( command );

	strcpy( command, "move ado_lc_data.cxx " );
	strcat( command, pDataPath );
	strcat( command, "..\\sabeh\\hcsm\\usersrc" );
	system( command );
}

///////////////////////////////////////////////////////////////////////////////
//
// Description:  Deletes the text files, .scn files produced by this program.
//
// Remarks:
//
// Arguments:
//   pDataPath - The path to the main data directory.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void 
DeleteFiles( const char* pDataPath )
{	
	////
	//// Move the H and CXX file to 'sabeh/hcsm/usersrc'.
	////
	//char command[4096];
	//strcpy( command, "move ado_lc_data.h " );
	//strcat( command, pDataPath );
	//strcat( command, "..\\sabeh\\hcsm\\usersrc" );
	//system( command );

	//strcpy( command, "move ado_lc_data.cxx " );
	//strcat( command, pDataPath );
	//strcat( command, "..\\sabeh\\hcsm\\usersrc" );
	//system( command );

	//
	// Delete the Text File in the current directory.
	//
	//strcpy( command, "del " );
	//strcat( command, g_DatafileName.c_str() );
	//system( command );

	char command[4096];

	//
	// Delete the ado_genlcdata.bat file
	//
	strcpy( command, "del ado_genlcdata.bat" );
	system( command );

	// Change to the NADSSDC-DATA directory 
	// Execute the batch file
	// Delete the batch file
	//
	// Copying all the scenario files into main 'data' directory.
	//
	strcpy( command, "cd " );
	strcat( command, pDataPath );
	strcat( command, " && del_ado_genlcdata.bat");
	strcat( command, " && del del_ado_genlcdata.bat");
	system( command );


}  // end of DeleteFiles


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Builds the paths to the various directories and stores
//   them in the given character arrays.
//
// Remarks:
//
// Arguments:
//   pBinPath  - The path to the main bin directory.
//   pDataPath - The path to the main data directory.
//   pSrcPath  - The path to the src directory.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void
GeneratePaths( char* pBinPath, char* pDataPath, char* pSrcPath )
{
	//
	// Get the path to the bin directory.
	//
	if( getenv( "NADSSDC_BIN" ) ) 
	{
		strcpy( pBinPath, getenv( "NADSSDC_BIN" ) );
		//strcat( pBinPath, "\\" );
	}
	else
	{
		cerr << "ado_genlcdata ERROR: undefined system ";
		cerr << "variable 'NADSSDC_BIN'" << endl;

		exit( -1 );
	}

	//
	// Get the path to the data directory.
	//
	if( getenv( "NADSSDC_SCN" ) ) 
	{
		strcpy( pDataPath, getenv( "NADSSDC_SCN" ) );
	}
	else
	{
		cerr << "ado_genlcdata ERROR: undefined system ";
		cerr << "variable 'NADSSDC_SCN'" << endl;

		exit( -1 );
	}

	//strcpy( pSrcPath, pDataPath );
	strcpy( pSrcPath, "E:\\nadssdc\\sabeh\\utils\\ado_genlcdata\\src\\" ); // HACK

}  // end of GeneratePaths


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Generates the interstate BLI.
//
// Remarks:
//
// Arguments:
//   pBinPath  - The path to the main bin directory.
//   pDataPath - The path to the main data directory.
//   pSrcPath  - The path to the src directory.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void
GenerateBli( const char* pBinPath, const char* pDataPath, const char* pSrcPath )
{
	char command[4096];
	strcpy( command, pBinPath );
	strcat( command, "lriccd " );
	strcat( command, pSrcPath );
	strcat( command, "interstate.lri " );
	//strcat( command, pDataPath );
	strcat( command, "interstate.bli && move interstate.bli " );
	strcat( command, pDataPath );
	system( command );

}  // end of GenerateLri


//////////////////////////////////////////////////////////////////////////////
//
// Main Program 
//
int 
main( int argc, char** argv )
{
	//
	// Generate paths to bin and data directories.
	//
	char binPath[4096];
	char dataPath[4096];
	char srcPath[4096];
	GeneratePaths( binPath, dataPath, srcPath );

	cout << binPath << endl;
	cout << dataPath << endl;
	cout << srcPath << endl;

#if 1
	// parse command line arguments
	ParseCommandLineArguments( argc, argv );
	
	//
	// Add SolObjects to the vector
	//
	sAddSolObjsToVector();

	if (!g_HaveDataFile)
	{
		//
		// Generate binary lri.
		//
		GenerateBli( binPath, dataPath, srcPath );

		//
		// Generate the batch files for executing the scenario files.
		//
		ofstream scenario( "ado_genlcdata.bat" );
		ofstream deleteSCN( "del_ado_genlcdata.bat" );
		
		int lowerVel = int(g_LowerVelLimit/5) - 1;
		int upperVel = int(g_UpperVelLimit/5) - 1;
		int lowerUrg = int(g_LowerUrgLimit*10 - 1);
		int upperUrg = int(g_UpperUrgLimit*10 - 1);
	
		int countVel;		// Velocity Counter
		int countUrg;		// Urgency Counter

		int cntTotalFilesCreated = 0;
		string trackVehCatLimit = "\nTotal Objs exceeded Limit\n";

		cout << endl << endl << "Creating the Scenario files with " << endl;
		cout.precision(2);
		cout.setf(ios_base::fixed, ios_base::floatfield);
		cout << "Velocity Range " << g_LowerVelLimit << "-"<< g_UpperVelLimit<< endl;
		cout << "Urgency Range " << g_LowerUrgLimit << "-"<< g_UpperUrgLimit << endl << endl;

		//
		// Using a try-catch block to link up the # ofSol objects(totalObjs, 
		// MAX_SOL_OBJS) reqd. and # of Scenario files created.
		// Used this try-catch block to 'exit out of multiple for-loops
		//
		// This is of good use for 'quick' testing.
		// When we need only data of fewer (ex:3) objs, we can set the 
		// MAX_SOL_OBJS to 3 and only scenario files of 3 vehicle categories will be 
		// generated and executed intead of 50 odd vehicle category files getting created 
		// and executed
		try
		{
			for( itr = AllSolIds.begin(); itr != AllSolIds.end(); itr++ )
			{
				for( countVel = lowerVel; countVel <= upperVel; countVel++ )
				{
					for( countUrg = lowerUrg; countUrg <= upperUrg; countUrg++ )
					{
						string solName = cved->GetSol().GetObj(*itr)->GetName();
						double velocity = 5.0 + ( 5.0 * countVel );
						double urgency = ( 0.1 * countUrg ) + 0.1;
						
						//
						// Track the number of vehicle categories reqd
						// throw exception to exit out of multiple
						// for loops
						//
#ifdef DEBUG_GENERAL
						cout << "TotalObjs: " << totalObjs << " SolName: " << solName;
						cout << " File #: " << cntTotalFilesCreated << endl;
#endif
						
						string fileName = sWriteSCNFile( velocity, urgency, solName);

						//
						// Batch File consisting of hcsmsys (scenario file) (frames) 
						//			
						scenario << "nixd -ig 0 -nonads " << fileName << " " << 2000 << endl;
						
						/* Work on it later 
						// Adding another execution command to check if lanechnage.txt is 
						// being created. If not updated after each scenario, no need to execute
						// all the scenarios. STOP at that moment and look at ado_lanechange.cxx
						// for the compiler pre-processor DEBUG_OUTPUT. This had to be defined
						// to create the 'lanechange.txt' file
						//
						if (catIndex == 0 && countVel == lowerVel && countUrg == lowerUrg)
						{

							ofstream testCreationTextFile( "testTextFile.bat" );
							testCreationTextFile << "hcsmsys " << fileName << " " << 1000 << endl;
							testCreationTextFile.close();
						}
						*/

						//
						// Batch file consisting of the delete command and the SCN files
						//
						deleteSCN << "del " << fileName << endl;
					
					}
					cntTotalFilesCreated++; // work on it later.
				}
			}
			
		}
		//
		// Catch block to exit out of nested for-loops
		//
		catch( string loopExit ) // catch exception
		{
			cout << loopExit << endl;
		}
		scenario.close();
		deleteSCN.close();

		//
		// Execute the Batch File
		//
		ExecuteBatchScenarios( binPath, dataPath );
	}

	//
	// Inputs the values of the text file into the array.
	//
	InputIntoArrays(g_DatafileName);

		
	//
	// Creates the .CXX and .H files.
	//
	CreateLcDataFiles();
	
	//
	// Moves the generated code files
	//
	MoveCodeFiles( dataPath );

	//
	// Deletes the text files, batch files, scenario files 
	// generated by this program. Only do it when we generate
	// complete scenarios. Not reqd. when having a data file
	//
	if(!g_HaveDataFile)
	{
		
		DeleteFiles( dataPath );
	}
#endif

	return EXIT_SUCCESS;
}
