
/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: main.cxx,v 1.3 2016/08/22 14:34:48 IOWA\dheitbri Exp $
 *
 * Author:  Vijay Gade
 *
 * Date:    May, 2001
 *
 * Description:  Generates radius of curvature data for ADOs.  Running the
 *   the ADO given a distance, velocity and radius, the code here generates
 *   lookup tables and associated functions that return the time.
 *
 * Notes:
 * StabilizeDist: Distance at which the given velocity is attained
 * StabilizeVel : Velocity given as input, along with the distance
 *
 * -----------------------------------------------------------------------
 * |                   VERY VERY IMPORTANT POINT:                        |
 * -----------------------------------------------------------------------
 * DYNAMICS UNITS ARE IN METERS (DISTANCE) AND METERS PER SECOND (VELOCITY)
 * CVED UNITS ARE IN FEET AND MILES PER HOUR
 *
 ****************************************************************************/

#include <pi_iostream>
#include <cved.h>
#include <pi_vector>
#include <stdio.h>
#include <math.h>
#include <iomanip>
#include <vector>

void WriteToLookUpCxx(
			int,
			FILE*,
			float time[][6][12][10],
			int,
			int*
			);
void WriteToLookUpHeader(int, FILE*, int);
void WriteToInterpolationFunction(int, FILE*, double minRadArray[][10], int);

using namespace std;
using namespace CVED;

float   g_Freq       = 30.0f;
int     g_DynaMult   = 2;
int     g_CvedDebug  = 0;
bool    g_Verbose    = false;

//
// Command-line argument global variables.
//
int g_startVel = 0;
int g_endVel = 55;
int g_startDist = 0;
int g_endDist = 100;
string g_dataFileName;
string g_bliFileName;

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the variables associated with the 
//   vehicle dynamics.
//
// Remarks:  
//
// Arguments:
//   slVehicleObj - A SOL object indicating the vehicle type, model, etc.
//   pVehicleObj - A pointer to the vehicle object.
//
// 
//////////////////////////////////////////////////////////////////////////////
void
InitializeDynamicsVars( 
			const cvEObjType objType,
			const slObj* pSolObj, 
			CVehicleObj* pVehicle
			)
{
	double axleHeight;

	// Get a reference to a SOL object.
	switch ( objType ) {

		case eCV_BUS :
			{
				const slBusesObj* cpSolObj = 
					dynamic_cast<const slBusesObj*> ( pSolObj );

				if (!cpSolObj) cout << "Could not open bus" << endl;
				axleHeight = cpSolObj->GetDynaAxleHeight();

				double suspStif = ( cpSolObj->GetDynaSuspStifMin() + 
									cpSolObj->GetDynaSuspStifMax() ) / 2;
				pVehicle->SetSuspStif( suspStif );
				pVehicle->SetSuspStifImm( suspStif );

				double suspDamp = ( cpSolObj->GetDynaSuspDampMin() + 
									cpSolObj->GetDynaSuspDampMax() ) / 2;
				pVehicle->SetSuspDamp( suspDamp );
				pVehicle->SetSuspDampImm( suspDamp );

				double tireStif = ( cpSolObj->GetDynaTireStifMin() + 
									cpSolObj->GetDynaTireStifMax() ) / 2;
				pVehicle->SetTireStif( tireStif );
				pVehicle->SetTireStifImm( tireStif );
				double tireDamp = ( cpSolObj->GetDynaTireDampMin() + 
									cpSolObj->GetDynaTireDampMax() ) / 2;
				pVehicle->SetTireDamp( tireDamp );
				pVehicle->SetTireDampImm( tireDamp );
				pVehicle->SetDynaFidelity( eNON_LINEAR );
				pVehicle->SetDynaFidelityImm( eNON_LINEAR );

			}

			break;

		case eCV_EMERGENCY :
			{
				const slEmergVehObj* cpSolObj = 
					dynamic_cast<const slEmergVehObj*> ( pSolObj );

				axleHeight = cpSolObj->GetDynaAxleHeight();

				double suspStif = ( cpSolObj->GetDynaSuspStifMin() + 
									cpSolObj->GetDynaSuspStifMax() ) / 2;
				pVehicle->SetSuspStif( suspStif );
				pVehicle->SetSuspStifImm( suspStif );

				double suspDamp = ( cpSolObj->GetDynaSuspDampMin() + 
									cpSolObj->GetDynaSuspDampMax() ) / 2;
				pVehicle->SetSuspDamp( suspDamp );
				pVehicle->SetSuspDampImm( suspDamp );

				double tireStif = ( cpSolObj->GetDynaTireStifMin() + 
									cpSolObj->GetDynaTireStifMax() ) / 2;
				pVehicle->SetTireStif( tireStif );
				pVehicle->SetTireStifImm( tireStif );
				double tireDamp = ( cpSolObj->GetDynaTireDampMin() + 
									cpSolObj->GetDynaTireDampMax() ) / 2;
				pVehicle->SetTireDamp( tireDamp );
				pVehicle->SetTireDampImm( tireDamp );
				pVehicle->SetDynaFidelity( eNON_LINEAR );
				pVehicle->SetDynaFidelityImm( eNON_LINEAR );

			}

			break;

		case eCV_TRUCK :
			{
				const slTruckObj* cpSolObj = 
					dynamic_cast<const slTruckObj*> ( pSolObj );


				double suspStif = ( cpSolObj->GetDynaSuspStifMin() + 
									cpSolObj->GetDynaSuspStifMax() ) / 2;
				pVehicle->SetSuspStif( suspStif );
				pVehicle->SetSuspStifImm( suspStif );

				double suspDamp = ( cpSolObj->GetDynaSuspDampMin() + 
									cpSolObj->GetDynaSuspDampMax() ) / 2;
				pVehicle->SetSuspDamp( suspDamp );
				pVehicle->SetSuspDampImm( suspDamp );

				double tireStif = ( cpSolObj->GetDynaTireStifMin() + 
									cpSolObj->GetDynaTireStifMax() ) / 2;
				pVehicle->SetTireStif( tireStif );
				pVehicle->SetTireStifImm( tireStif );
				double tireDamp = ( cpSolObj->GetDynaTireDampMin() + 
									cpSolObj->GetDynaTireDampMax() ) / 2;
				pVehicle->SetTireDamp( tireDamp );
				pVehicle->SetTireDampImm( tireDamp );
				pVehicle->SetDynaFidelity( eNON_LINEAR );
				pVehicle->SetDynaFidelityImm( eNON_LINEAR );

				axleHeight = cpSolObj->GetDynaAxleHeight();


			}

			break;

			//
			//For now, we do not use the utility vehicles
			//
		case eCV_UTILITY_VEH :
			{

	#if 0
				const slUtilVehObj* cpSolObj = 
					dynamic_cast<const slUtilVehObj*> ( pSolObj );

				axleHeight = cpSolObj->GetDynaAxleHeight();
	//#endif

				double suspStif = ( cpSolObj->GetDynaSuspStifMin() + 
									cpSolObj->GetDynaSuspStifMax() ) / 2;
				pVehicle->SetSuspStif( suspStif );
				pVehicle->SetSuspStifImm( suspStif );

				double suspDamp = ( cpSolObj->GetDynaSuspDampMin() + 
									cpSolObj->GetDynaSuspDampMax() ) / 2;
				pVehicle->SetSuspDamp( suspDamp );
				pVehicle->SetSuspDampImm( suspDamp );

				double tireStif = ( cpSolObj->GetDynaTireStifMin() + 
									cpSolObj->GetDynaTireStifMax() ) / 2;
				pVehicle->SetTireStif( tireStif );
				pVehicle->SetTireStifImm( tireStif );
				double tireDamp = ( cpSolObj->GetDynaTireDampMin() + 
									cpSolObj->GetDynaTireDampMax() ) / 2;
				pVehicle->SetTireDamp( tireDamp );
				pVehicle->SetTireDampImm( tireDamp );
				pVehicle->SetDynaFidelity( eNON_LINEAR );
				pVehicle->SetDynaFidelityImm( eNON_LINEAR );


				cerr << "VehicleDynamics: GetAxleHeight: ";
				cerr << "utility vehicle needs DynaAxleHeight SOL parameter";

				assert(0);
     #endif
			}

			break;



		case eCV_VEHICLE :
			{
				const slVehicleObj* cpSolObj =
					dynamic_cast<const slVehicleObj*> ( pSolObj );
				if ( !cpSolObj ) {

					cerr << "InitializeDynamicVars: unable to get reference ";
					cerr << "to SOL object of category = ";
					cerr << pSolObj->GetCategoryName()<<"...[SUICIDE]" <<endl;
					
					return;

				}

				double suspStif = ( cpSolObj->GetDynaSuspStifMin() + 
									cpSolObj->GetDynaSuspStifMax() ) / 2;
				pVehicle->SetSuspStif( suspStif );
				pVehicle->SetSuspStifImm( suspStif );

				double suspDamp = ( cpSolObj->GetDynaSuspDampMin() + 
									cpSolObj->GetDynaSuspDampMax() ) / 2;
				pVehicle->SetSuspDamp( suspDamp );
				pVehicle->SetSuspDampImm( suspDamp );

				double tireStif = ( cpSolObj->GetDynaTireStifMin() + 
									cpSolObj->GetDynaTireStifMax() ) / 2;
				pVehicle->SetTireStif( tireStif );
				pVehicle->SetTireStifImm( tireStif );
				double tireDamp = ( cpSolObj->GetDynaTireDampMin() + 
									cpSolObj->GetDynaTireDampMax() ) / 2;
				pVehicle->SetTireDamp( tireDamp );
				pVehicle->SetTireDampImm( tireDamp );
				pVehicle->SetDynaFidelity( eNON_LINEAR );
				pVehicle->SetDynaFidelityImm( eNON_LINEAR );

			}

			break;

		default :

			cerr << "InitializeDynamicVars: objects of type ";
			cerr <<cvObjType2String( objType );
			cerr <<" not supported yet..[SUICIDE]";
			cerr << endl;
			return;
	}
}//End of Definition of InitializeDynamicsVars()


///WriteToLookUpCxx///////////////////////////////////////////////////////////
//
// Descritption:
// Writes the time taken for a particular radius, velocity, distance for a 
// number of vehicle objects to a .cxx file (lookUp.cxx).
//
// Arguments:
// (int) numRcPoints - The number of starting points, which inturn
//                     defines the index of the radius in the 4D array.
// pFile             - File pointer to the lookUp.cxx file.
// time              - The 3D array that has in it radius(x), velocity(y) and
//                     the distance(z). The actual value that it contains is
//                     the time taken to reach a distance at a velocity
//                     for a radius.
// (int) totNumObjs  - Number of Objects the main file runs.
// pIdList           - Pointer to idArray that holds the names of all the 
//                     vehicles that are run.
//
// Returns / Remarks - None
// 
//////////////////////////////////////////////////////////////////////////////
	
void 
WriteToLookUpCxx(
			int numRcPoints,
			FILE* pFile,
			float time[][6][12][10],
			int totNumObjs,
			int* pIdList
			)
{
	//
	//Writing the initial include files and variable declarations
	//
	fprintf(pFile, "/**************************************************************************\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* (C) copyright 1998 by National Advanced Driving Simulator and Simulation\n");
	fprintf(pFile,
		"* Center, the University of Iowa and The University of Iowa. All rights reserved.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Version:		$Id: main.cxx,v 1.3 2016/08/22 14:34:48 IOWA\dheitbri Exp $\n"); 
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Author(s):	Vijay Gade, Huidi Tang\n");  
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Date:	October, 2001\n");
	fprintf(pFile, "*\n");
    fprintf(pFile,
		"* Description:	This File holds look up table containing the time\n");
	fprintf(pFile,
		"*             for given radius, velocity and distance. Also holds the list of\n");
	fprintf(pFile, 
		"*			   Sol IDs in an array. Has no input and ouput\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* WARNING:		THIS FILE IS AUTOMATICALLY GENERATED. DO NOT EDIT IT DIRECTLY.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile, "**************************************************************************/\n\n");
	fprintf(pFile, "\n");
	fprintf(pFile, "#include <stdio.h>\n");
	fprintf(pFile, "#include <pi_iostream>\n");
	fprintf(pFile, "#include <math.h>\n");
	fprintf(pFile, "using namespace std;\n\n\n");
	fprintf(pFile, "double time[%d][", totNumObjs);
	fprintf(pFile, "%d", numRcPoints);
	fprintf(pFile, "][12][10] = {\n");

	int x = 0;
	int y = 0;

	for (int o = 0; o < totNumObjs; o++)
	{
		for (int p = 0; p < numRcPoints; p++)
		{
			for (x = 0; x < 12; x++)
			{
				for(y = 0; y < 10; y++)
				{
					//
					//Writing all the time values, and also fixing up the
					//format to be written like "{-,-,-...}" into the file
					//
					if ((o==totNumObjs-1) &&
						(p==numRcPoints-1) &&
						(x==11) && (y==9)) 
					{
						fprintf(pFile,
								"%f",
								time[o][p][x][y]); 
					}
					else {
						fprintf(pFile,
								"%f, ",
								time[o][p][x][y]);
					}
				}
				fprintf(pFile, "\n");
			}
		}
	}
	fprintf(pFile, "};\n\n\n");

	//
	//Sending the idArray, which helps in mapping the Sol ID querry.
	//
	fprintf(pFile, "double idArray[%d] = \n{", totNumObjs);
	for (int qq = 0; qq < totNumObjs; qq++) {
		if (qq == totNumObjs-1) fprintf(pFile, "%d};", pIdList[qq]);
		else fprintf(pFile, "%d, ", pIdList[qq]);
		if (qq/10 == 1) fprintf(pFile, "\n");
	}

	//
	//Closing the file pointer
	//
	fclose(pFile);
}//WriteToLookUpCxx


///WriteToLookUpHeader////////////////////////////////////////////////////////
//		
// Description:
// Header file that contains the initialization of the 4D array and the
// idArray as extern variables. This is basically to avoid problems arising
// from multiple initializing of the array.
//
// Arguments:
// numRcPoints - Number of starting Points actually used on the scenario,
//               which inturn define the index of radius in the 4D array. 
// pFile       - File pointer to the header file.
// totNumObjs  - The total Number of Objects the main function runs.
//
// Returns / Remarks: None
//
//////////////////////////////////////////////////////////////////////////////


void 
WriteToLookUpHeader(
			int numRcPoints,
			FILE *pFile,
			int totNumObjs
			)
{
	fprintf(pFile, "/**************************************************************************\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* (C) copyright 1998-2016 by National Advanced Driving Simulator and Simulation\n");
	fprintf(pFile,
		"* Center, the University of Iowa and The University of Iowa. All rights reserved.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Version:		$Id: main.cxx,v 1.3 2016/08/22 14:34:48 IOWA\dheitbri Exp $\n"); 
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Author(s):	Vijay Gade, Huidi Tang\n");  
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Date:	October, 2001\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Description:	Defines the array as extern so that there\n");
	fprintf(pFile,
		"*            will not be any problems for multiple declarations\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* WARNING:		THIS FILE IS AUTOMATICALLY GENERATED by ado_genrcdata. DO NOT EDIT IT DIRECTLY.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile, "**************************************************************************/\n\n");
	fprintf(pFile, "\n\n");
	fprintf(pFile,
		   "#ifndef	ADO_RC_LOOKUP_H\n");
	fprintf(pFile,
		   "#define	ADO_RC_LOOKUP_H\n");
	fprintf(pFile, "\n");
	fprintf(pFile,
		   "extern double time[%d][",
		   totNumObjs);
	fprintf(pFile, "%d", numRcPoints);
	fprintf(pFile, "][12][10];\n");
	fprintf(pFile, "extern double idArray[%d];\n", totNumObjs);
	fprintf(pFile, "\n");
	fprintf(pFile,
		   "double	AdoRcLookup( double distance, double velocity, double radius, int solId );\n");
	fprintf(pFile, "\n");  
	fprintf(pFile,
		   "#endif\n");
	
	//
	//Closing the file pointer
	//
	fclose(pFile);
}//WriteToLookUpHeader


///WriteToInterpolationFunction///////////////////////////////////////////////
//		
// Description: The Interpolation Function calculates the interpolated 
//   time for a given radius at a given velocity and distance. The way
//   it does is it gets the 8 points surrounding the point of reference
//   and interpolates with a formula.
//               
// Arguments:
//   numRcPoints - The number of starting Points, which inturn
//                 defines the index of the radius (x) in the 3D array.
//   pFile       - File pointer to the file.
//   minRadArray - The array that holds all the radii
//                 calculated for the scenarios.
//
// Returns / Remarks: None
//
//////////////////////////////////////////////////////////////////////////////

void WriteToInterpolationFunction(
			int numRcPoints,
			FILE* pFile,
			double minRadArray[][10],
			int totNumObjs
			)
{
	//
	//Writing the initial description, arguments and the include files
	//
	fprintf(pFile, "/**************************************************************************\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* (C) copyright 1998 by National Advanced Driving Simulator and Simulation\n");
	fprintf(pFile,
		"* Center, the University of Iowa and The University of Iowa. All rights reserved.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Version:		$Id: main.cxx,v 1.3 2016/08/22 14:34:48 IOWA\dheitbri Exp $\n"); 
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Author(s):	Vijay Gade, Huidi Tang\n");  
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Date:	October, 2001\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Description:		Calculates the interpolated time for ");
	fprintf(pFile,
		"a given velocity and distance.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Inputs(s):	Velocity (mph) and Distance (feet)\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Output(s):	Asks the user for vel and dist\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Returns:		interpolated time (seconds)\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Functions used:	sortduplctRadii - arranges the radii array in an ascending order.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* Notes:	The duplctRadii array was sorted for \n");
	fprintf(pFile,
		"*          convenient out of bounds settings.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile,
		"* WARNING:		THIS FILE IS AUTOMATICALLY GENERATED. DO NOT EDIT IT DIRECTLY.\n");
	fprintf(pFile, "*\n");
	fprintf(pFile, "**************************************************************************/\n\n");
	fprintf(pFile, "#include <stdio.h>\n");
	fprintf(pFile, "#include <pi_iostream>\n");
	fprintf(pFile, "#include <math.h>\n");
	fprintf(pFile, "#include \"ado_rc_lookup.h\"\n");
	fprintf(pFile, "using namespace std;\n\n\n");

	//
	//Writing the sort array function Header Definition only
	//
	fprintf(pFile, "\nvoid SortRadii(");
	fprintf(pFile, "double radii[][%d],\n", numRcPoints);
	fprintf(pFile,
			"\t\t\t   double pDuplctRadii[][%d], \n",
			numRcPoints);
	fprintf(pFile,
		"\t\t\t   int size,\n\t\t\t   int numberOfObj,\n");
	fprintf(pFile,
			"\t\t\t   double actualRadii[][%d]);\n",
			numRcPoints);

	fprintf(pFile,
			"double AdoRcLookup(double distance,double velocity,double radius,");
	fprintf(pFile, "int solId)\n");
	fprintf(pFile, "{\n\t");
	fprintf(pFile, "double u, v, w, itpTime;\n");
	fprintf(pFile, "\tdouble minX = 0.0;\n");
	fprintf(pFile,
		"\tdouble minY = 0.0; //Setting the min X and Y\n");
	fprintf(pFile,
		"\tdouble stepX = 10.0, stepY = 5.0; //Setting the dist between the two intervals for distance and velocity\n");
	fprintf(pFile, "\tdouble minZ = 0, tempMinZ = 0.0;\n");
	fprintf(pFile, "\tdouble realZ;\n");
	fprintf(pFile, "\tdouble stepZ = 1, numZ = 6;\n");
	fprintf(pFile,
		"\tint numX = 10, numY = 12; //Giving the number of grids for the array\n");
	fprintf(pFile, "//For getting the radius values...\n\n");

	//
	//Depending on the solID index, mapping the 4D array
	//
	fprintf(pFile, "//\n//Depending on the solID input \n");
	fprintf(pFile, "//by user, idArray is mapped\n//\n");
	fprintf(pFile, "\tint iDIndex = 0;\n");
	fprintf(pFile, "\tint numberOfObj = %d;", totNumObjs);
	fprintf(pFile,
			"\n\tfor(int y=0; y<numberOfObj; y++) {\n");
	fprintf(pFile, "\t\tif(solId == idArray[y]) ");
	fprintf(pFile, "iDIndex = y;\n\t}\n\n");

	//
	//Writing all the minRads into the radii, which holds the minimum radii
	//for all the scenarios that had been run.
	//
	fprintf(pFile,
			"\tdouble radii[%d][%d] = ",
			totNumObjs,
			numRcPoints);

	fprintf(pFile, "{");
	for (int ii = 0; ii < totNumObjs; ii++) {
		for (int i = 0; i < numRcPoints; i++) {
			if (ii == totNumObjs-1 && i == numRcPoints-1) {
				fprintf(pFile, "%f", minRadArray[ii][i]);
			}
			else fprintf(pFile, "%f, ", minRadArray[ii][i]);
		}
		fprintf(pFile, "\n\t");
	}
	fprintf(pFile, "\t};\n\n");

	//
	//Writing all the radii values into the duplctRadii array
	//
	//Notes: A duplicate radii array is created for unsorting the radii Array
	//       again. In the defintion of the sort function, the radii array is
	//       sorted. But this happens in the first for loop. Now, when this 
	//       loop is executed for the second time, there is a condition which 
	//       says:---enter only if not sorted---. But we need to sort out the
	//       radii array each time the for loop executes because based on
	//       sorting the radii array, we have to sort out the 4D array. Why
	//       sort the 4D array? Because the vel and dist depend on the radii.
	//       So, when we sort the radii, even the vels and dists should be
	//       sorted out according to the order of radii sorted in the radii
	//       array. 
	//       Now, why sort the radii array? Because it helps in interpolation.
	//
	//       The next important array is the actualRadii array, which,
	//       also is like the duplicate Radii array. Then why do we need it? 
	//       Because it is this array that we actually use for all calculation
	//       purposes. While sorting the radii array function, it is required
	//       that we unswap the radii array each time it loops. But what we 
	//       want is a completely swapped radii array. So, it is best if we
	//       use the radii array to unswap itself during the for loop and
	//       sumultaneously swap the "actualRadii" array.
	//
	fprintf(pFile,
			"\n\tdouble actualRadii[%d][%d];\n",
			totNumObjs, numRcPoints);
	fprintf(pFile,
			"\n\tdouble duplctRadii[%d][%d];\n",
			totNumObjs,
			numRcPoints);

	//
	//Copying the idArray into the duplicate radii array
	//as well as into the actualRadii
	//
	fprintf(pFile,
			"\tfor (int xx = 0; xx < %d; xx++) {\n",
			totNumObjs);
	fprintf(pFile,
			"\t\tfor (int x = 0; x < %d; x++) {\n",
			numRcPoints);
	fprintf(pFile,
			"\t\t\tduplctRadii[xx][x]=radii[xx][x];\n");
	fprintf(pFile,
			"\t\t\tactualRadii[xx][x]=radii[xx][x];\n");
	fprintf(pFile, "\t\t}\n\t}\n\n");
	
	//
	//Call the Sort Function
	//
	fprintf(pFile,
			"\t//\n\t//Calling the sort function\n\t//\n");
	fprintf(pFile,
			"\tSortRadii(radii, duplctRadii, ");
	fprintf(pFile, "%d , numberOfObj, actualRadii);\n", numRcPoints);

	//
	//Writing the calculations for the variables that have to be used in the
	//formula for claculating the interpolated time.
	//
	fprintf(pFile, "\n\tfor (int i=0;i<");
	fprintf(pFile, "%d", numRcPoints);
	fprintf(pFile, ";i++){\n");
	fprintf(pFile,
			"\t\tif ((radius >= actualRadii[iDIndex][i]) && ");
	fprintf(pFile, "(radius < actualRadii[iDIndex][i+1])){\n");
	fprintf(pFile, "\t\trealZ = i;\n");
	fprintf(pFile, "\t\t\tif(actualRadii[iDIndex][i]");
	fprintf(pFile, "<=actualRadii[iDIndex][i+1])");
	fprintf(pFile, " minZ=actualRadii[iDIndex][i];\n");
	fprintf(pFile, "\t\t\telse minZ =");
		fprintf(pFile, " actualRadii[iDIndex][i+1];\n");
	fprintf(pFile, "\t\tstepZ=fabs");
	fprintf(pFile, "(actualRadii[iDIndex][i+1]-");
	fprintf(pFile, "actualRadii[iDIndex][i]);\n");
	fprintf(pFile, "\t\t}\n");
	fprintf(pFile, "\t}\n\n");

	//
	//Writing the out of bounds exceptions for the Vel, Dist and Rad
	//
	fprintf(pFile,
	"\t//\n\t//Setting out of bounds exception for Vel, Dist, Rad\n\t//\n");
	fprintf(pFile, "\tint illegalInput = 0;\n");
	fprintf(pFile,
		"\tif ((distance < minX) || (distance >= minX + numX*stepX)){\n");
	fprintf(pFile,
		"\t\tcout << \"Input distance is out of Bounds\" << endl;\n");
	fprintf(pFile, "\t\tillegalInput = 1;\n");
	fprintf(pFile, "\t}\n");
	fprintf(pFile,
		"\tif ((velocity < minY) || (velocity >= minY + numY*stepY)){\n");
	fprintf(pFile,
		"\t\tcout << \"Input Velocity is out of Bounds\" << endl;\n");
	fprintf(pFile, "\t\tillegalInput = 1;\n");
	fprintf(pFile, "\t}\n");
	fprintf(pFile,
		"\tif ((radius < actualRadii[iDIndex][0]) || ");
	fprintf(pFile, "(radius >= actualRadii[iDIndex][");
	fprintf(pFile, "%d", numRcPoints-1);
	fprintf(pFile, "])){\n");
	fprintf(pFile,
		"\t\tcout << \"Input Radius is out of Bounds\" << endl;\n");
	fprintf(pFile, "\t\tillegalInput = 1;\n");
	fprintf(pFile, "\t}\n");

	//
	//Writing the calculations for the 8 points that have to be used in 
	//interpolating the point of reference.
	//
	fprintf(pFile,
		"\t//\n\t// Assigning the 8 points around the point for ");
	fprintf(pFile, "interpolation.\n\t//\n");
	fprintf(pFile, "\tint x000 = (distance - minX)/stepX;\n");
	fprintf(pFile, "\tint x010 = x000 + 1;\n");
	fprintf(pFile, "\tint y000 = (velocity - minY)/stepY;\n");
	fprintf(pFile, "\tint y100 = y000 + 1;\n");
	fprintf(pFile, "\tint x110 = x010;\n");
	fprintf(pFile, "\tint y110 = y100;\n");
	fprintf(pFile, "\tint x001 = x000;\n");
	fprintf(pFile, "\tint x011 = x010;\n");
	fprintf(pFile, "\tint y001 = y000;\n");
	fprintf(pFile, "\tint y101 = y100;\n");
	fprintf(pFile, "\tint x111 = x110;\n");
	fprintf(pFile, "\tint y111 = y110;\n");
	fprintf(pFile, "\tint z000 = realZ;");
	fprintf(pFile, "\tint z001 = z000 + 1;\n");

	//
	//Writing out the formula that is used in interpolating the time
	//
	fprintf(pFile,"\tu = (distance - (minX+x000*stepX))/stepX;\n");
	fprintf(pFile,"\tv = (velocity - (minY+y000*stepY))/stepY;\n");
	fprintf(pFile,"\tw = (radius - minZ)/stepZ ;\n");
	fprintf(pFile,"\n\titpTime = u * v * w * time");
	fprintf(pFile, "[iDIndex][z001][y110][x110] +\n");
	fprintf(pFile, "\t(1-u) * v * w * time");
	fprintf(pFile, "[iDIndex][z001][y100][x000] +\n");
	fprintf(pFile, "\tu * (1-v) * w * time");
	fprintf(pFile, "[iDIndex][z001][y000][x010] +\n");
	fprintf(pFile, "\t(1-u) * (v) * (1-w) * time");
	fprintf(pFile, "[iDIndex][z000][y100][x000] +\n");
	fprintf(pFile, "\t(1-u) * (1-v) * (w) * time");
	fprintf(pFile, "[iDIndex][z001][y000][x000] +\n");
	fprintf(pFile, "\t(u) * (1-v) * (1-w) * time");
	fprintf(pFile, "[iDIndex][z000][y000][x010] +\n");
	fprintf(pFile, "\t(u) * (v) * (1-w) * time");
	fprintf(pFile, "[iDIndex][z000][y100][x010] +\n");
	fprintf(pFile, "\t(1-u) * (1-v) * (1-w) * time");
	fprintf(pFile, "[iDIndex][z000][y000][x000];\n");

	//
	//Writing out the output interpolated time with the condition that the
	//input variables are not out of bounds.
	//
	fprintf(pFile, "\tif (illegalInput == 1) return 0;\n");
	fprintf(pFile, "\telse return itpTime;\n");
	fprintf(pFile, "}\n");

	//
	//Writing the sort function definition
	//
	fprintf(pFile, "\nvoid SortRadii(");
	fprintf(pFile, "double pRadii[][%d],\n", numRcPoints);
	fprintf(pFile,
			"\t\t\t   double pDuplctRadii[][%d], \n", numRcPoints);
	fprintf(pFile,
			"\t\t\t   int size,\n\t\t\t   int numberOfObj, ");
	fprintf(pFile,
			"\n\t\t\t   double pActualRadii[][%d])\n{\n",
			numRcPoints);
	fprintf(pFile, "\tdouble temp;\n");
	fprintf(pFile, "\tdouble actualTemp;\n");
	fprintf(pFile, "\tdouble tempVelDist[12][10];\n");

	fprintf(pFile,
			"\tfor (int k = 0; k < numberOfObj; k++) {\n");
	fprintf(pFile, "\t\tfor (int i = 0; i < size; i++){\n");
	fprintf(pFile,
			"\t\t\tfor (int j = 0; j < size; j++){\n");
	fprintf(pFile,
			"\t\t\t\tif ( pRadii[k][i] < pRadii[k][j] ) {\n");
	fprintf(pFile,"\t\t\t\t\ttemp = pRadii[k][i];\n");
    fprintf(pFile,"\t\t\t\t\t\tactualTemp = ");
	fprintf(pFile,"pActualRadii[k][i];\n");

	fprintf(pFile,
			"\t\t\t\t\tpRadii[k][i] = pRadii[k][j];\n");
	fprintf(pFile,
			"\t\t\t\t\t\tpActualRadii[k][i] = pActualRadii[k][j];\n");
	fprintf(pFile,"\t\t\t\t\tpRadii[k][j] = temp;\n");
	fprintf(pFile,
			"\t\t\t\t\t\tpActualRadii[k][j] = actualTemp;\n");
	fprintf(pFile, "\n//\n//Sorting the timeAtRa... ");
	fprintf(pFile, "array with respect to their\n");
	fprintf(pFile, "//corresponding radii Array.\n//\n");
	fprintf(pFile, "\t\t\t\t\tfor(int p=0;p<12;p++) {\n");
	fprintf(pFile,
			"\t\t\t\t\t\tfor(int pp=0;pp<10;pp++) {\n");
	fprintf(pFile, "\t\t\t\t\t\t\ttempVelDist[p][pp]=");
	fprintf(pFile, "time[k][i][p][pp];\n");
	fprintf(pFile, "\t\t\t\t\t\t}\n\t\t\t\t\t}\n");
	fprintf(pFile, "\t\t\t\t\tfor(int q=0;q<12;q++) {\n");
	fprintf(pFile,
			"\t\t\t\t\t\tfor(int qq=0;qq<10;qq++) {\n");
	fprintf(pFile,
			"\t\t\t\t\t\t\ttime[k][i][q][qq]=\n");
	fprintf(pFile,
			"\t\t\t\t\t\t\t\ttime[k][j][q][qq];\n");
	fprintf(pFile, "\t\t\t\t\t\t}\n\t\t\t\t\t}\n");
	fprintf(pFile, "\t\t\t\t\tfor(int r=0;r<12;r++) {\n");
	fprintf(pFile,
			"\t\t\t\t\t\tfor(int rr=0;rr<10;rr++) {\n");
	fprintf(pFile,
			"\t\t\t\t\t\t\ttime[k][i][r][rr]=");
	fprintf(pFile, "tempVelDist[r][rr];\n");
	fprintf(pFile, "\t\t\t\t\t\t}\n\t\t\t\t\t}\n");

	//
	//Thing is, initially, sorting of radii array is required. But then,
	//unsorting the radii array is required every outer most 'for' loop
	//except for the last count of that loop. If we do not unsort it, then
	//the outer most 'for' loop will stop once the radii array is sorted.
	//

	//
	//Unswapping the Radii Except for the last count...
	//
	fprintf(pFile, "\t\t\t\t}//if\n\t\t\t}//j\n");

	fprintf(pFile, "\t\t}//i\n");
	
	fprintf(pFile,
			"\t\tfor(int unswap=0;unswap<size;unswap++) {\n");
	fprintf(pFile,
			"\t\t\tpRadii[k][unswap]=pDuplctRadii[k][unswap];\n");
	fprintf(pFile, "\t\t}\n\t}\n}");
	fprintf(pFile, "//end of sortRadii\n");

	//
	//Closing the file pointer
	//
	fclose(pFile);
}//WriteToInterpolationFunction


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// Generate the list for vehicles
//
// Arguments:
// &cved -  Reference to CCved to create sol object
// &temp - Reference to the vector that holds the names of the vehicles
//
// Remarks:	initialize sSolCateList and sSolObjList
//
// Returns: Number of objects the temp Vector has stored in itself
//
//////////////////////////////////////////////////////////////////////////////

int GetVehicles(CCved& cved, vector <char*> &temp)
{
	//
	//Declaring the required variables
	//
	static vector <char*> sSolCateList;
	static vector < vector <char*> > sSolObjList;
	
	//
	//Creating a Cved object...
	//
	int numObj = 0;	// store the number of objects in each category;
	const int MAX_OBJS_PER_CATEGORY = 50;
	numObj = MAX_OBJS_PER_CATEGORY;
	sSolCateList.push_back("Vehicle");
	const slVehicleObj* pList0[MAX_OBJS_PER_CATEGORY];
	numObj = cved.GetSol().QryAllVehicleObjects(pList0,
												MAX_OBJS_PER_CATEGORY);
	int objIndex = 0;
	for(objIndex = 0; objIndex < numObj; objIndex++)
	{
		char* pobjName= new char[(pList0[objIndex]->GetObjName()).length()+1];
		(pList0[objIndex]->GetObjName( )).copy(pobjName, string::npos);
		pobjName[(pList0[objIndex]->GetObjName( )).length()] = 0;

		temp.push_back(pobjName);
	}
	sSolObjList.push_back(temp);
	//temp.clear( );
	return numObj;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// Generate the list for the Buses
//
// Arguments:
// &cved -  Reference to CCved to create sol object
// &temp - Reference to the vector that holds the names of the vehicles
//
// Remarks:	initialize sSolCateList and sSolObjList
//
// Returns: Number of objects the temp Vector has stored in itself
//
//////////////////////////////////////////////////////////////////////////////

int GetBuses(CCved& cved, vector <char*> &temp)
{
	//
	//Declaring the required variables
	//
	static vector <char*> sSolCateList;
	static vector < vector <char*> > sSolObjList;
	
	int numObj = 0;	// store the number of objects in each category;
	const int MAX_OBJS_PER_CATEGORY = 50;
	numObj = MAX_OBJS_PER_CATEGORY;
	sSolCateList.push_back("Buses");
	const slBusesObj* pList0[MAX_OBJS_PER_CATEGORY];
	numObj = cved.GetSol().QryAllBusesObjects(pList0, MAX_OBJS_PER_CATEGORY);
	int objIndex = 0;
	for(objIndex = 0; objIndex < numObj; objIndex++)
	{
		char* pobjName= new char[(pList0[objIndex]->GetObjName()).length()+1];
		(pList0[objIndex]->GetObjName( )).copy(pobjName, string::npos);
		pobjName[(pList0[objIndex]->GetObjName( )).length()] = 0;

		temp.push_back(pobjName);
	}
	sSolObjList.push_back(temp);
	//temp.clear( );
	return numObj;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// Generate the list for the Emergency Vehicles
//
// Arguments:
// &cved -  Reference to CCved to create sol object
// &temp - Reference to the vector that holds the names of the vehicles
//
// Remarks:	initialize sSolCateList and sSolObjList
//
// Returns: Number of objects the temp Vector has stored in itself
//
//////////////////////////////////////////////////////////////////////////////

int GetEmergVeh(CCved& cved, vector <char*> &temp)
{
	//
	//Declaring the required variables
	//
	static vector <char*> sSolCateList;
	static vector < vector <char*> > sSolObjList;
	
	int numObj = 0;	// store the number of objects in each category;
	const int MAX_OBJS_PER_CATEGORY = 50;
	numObj = MAX_OBJS_PER_CATEGORY;
	sSolCateList.push_back("EmergVeh");
	const slEmergVehObj* pList0[MAX_OBJS_PER_CATEGORY];
	numObj =
	cved.GetSol().QryAllEmergVehObjects(pList0,MAX_OBJS_PER_CATEGORY);
	int objIndex = 0;
	for(objIndex = 0; objIndex < numObj; objIndex++)
	{
		char* pobjName= new char[(pList0[objIndex]->GetObjName()).length()+1];
		(pList0[objIndex]->GetObjName( )).copy(pobjName, string::npos);
		pobjName[(pList0[objIndex]->GetObjName( )).length()] = 0;

		temp.push_back(pobjName);
	}
	sSolObjList.push_back(temp);
	//temp.clear( );
	return numObj;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// Generate the list for trucks
//
// Arguments:
// &cved -  Reference to CCved to create sol object
// &temp - Reference to the vector that holds the names of the vehicles
//
// Remarks:	initialize sSolCateList and sSolObjList
//
// Returns: Number of objects the temp Vector has stored in itself
//
//////////////////////////////////////////////////////////////////////////////

int GetTrucks(CCved& cved, vector <char*> &temp)
{
	//
	//Declaring the required variables
	//
	static vector <char*> sSolCateList;
	static vector < vector <char*> > sSolObjList;
	
	int numObj = 0;	// store the number of objects in each category;
	const int MAX_OBJS_PER_CATEGORY = 50;
	numObj = MAX_OBJS_PER_CATEGORY;
	sSolCateList.push_back("Truck");
	const slTruckObj* pList0[MAX_OBJS_PER_CATEGORY];
	numObj =
	cved.GetSol().QryAllTruckObjects(pList0,MAX_OBJS_PER_CATEGORY);
	int objIndex = 0;
	for(objIndex = 0; objIndex < numObj; objIndex++)
	{
		char* pobjName= new char[(pList0[objIndex]->GetObjName()).length()+1];
		(pList0[objIndex]->GetObjName( )).copy(pobjName, string::npos);
		pobjName[(pList0[objIndex]->GetObjName( )).length()] = 0;

		temp.push_back(pobjName);
	}
	sSolObjList.push_back(temp);
	//temp.clear( );
	return numObj;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Generate the list for Utility vehicles.
//
// Arguments:
//   cved - Reference to CCved to create sol object.
//   temp - Reference to the vector that holds the names of the vehicles.
//
// Remarks:	Initialize sSolCateList and sSolObjList.
//
// Returns: Number of objects the temp Vector has stored in itself.
//
//////////////////////////////////////////////////////////////////////////////

int 
GetUtilVeh( CCved& cCved, vector<char*> &temp)
{
	//
	// Declaring the required variables.
	//
	static vector <char*> sSolCateList;
	static vector < vector <char*> > sSolObjList;	
	int numObj = 0;	// store the number of objects in each category;
	const int MAX_OBJS_PER_CATEGORY = 50;
	numObj = MAX_OBJS_PER_CATEGORY;
	sSolCateList.push_back( "UtilVeh" );
	const slUtilVehObj* pList0[MAX_OBJS_PER_CATEGORY];
	numObj = cCved.GetSol().QryAllUtilVehObjects( 
									pList0, 
									MAX_OBJS_PER_CATEGORY 
									);

	int objIndex;
	for( objIndex = 0; objIndex < numObj; objIndex++ )
	{
		char* pObjName = new char[(pList0[objIndex]->GetObjName()).length() + 1];
		(pList0[objIndex]->GetObjName( )).copy(pObjName, string::npos);
		pObjName[(pList0[objIndex]->GetObjName( )).length()] = 0;

		temp.push_back( pObjName );
	}
	sSolObjList.push_back( temp );
	//temp.clear( );
	return numObj;
}

//Function: Usage////////////////
//
//To provide the user with the proper usage of the command line arguments
//that have to be in the form:
//winnt.ext -lriFile lriFileName -dataFile dataFileName
//Note that the dataFileName should be given along with it's path. As of now, 
//it is ok if we give "dataFile.txt" directly.
//
void Usage( void )
{

	cerr << "Usage: " << endl;
	cerr << "winnt -dataFile <name> -bliFile <name> [-startVel #] [-endVel #]" << endl;
	cerr << "  -dataFile  The file with the starting points (required)" << endl;
	cerr << "  -bliFile   The BLI on which the points are located (required)" << endl; 
	cerr << "  -startVel  Sample data starting at this velocity (default = 0mph)" << endl;
	cerr << "  -endVel    Sample data ending at this velocity (default = 55mph)" << endl;

	exit(0);
}
//End of Usage////////


void ParseCommandLineArguments( int argc, char** argv)
{

	int arg;

	for( arg = 1; arg < argc; arg++ )
	{
		if ( argv[arg][0] == '-' ) 
		{
			if ( !strcmp(argv[arg], "-bliFile") ) 
			{

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_bliFileName = argv[arg];

			}
			else if ( !strcmp(argv[arg], "-dataFile")) 
			{

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_dataFileName = argv[arg];

			}
			else if ( !strcmp(argv[arg], "-startVel")) 
			{

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_startVel = atof( argv[arg] );

			}
			else if ( !strcmp(argv[arg], "-endVel")) 
			{

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_endVel = atof( argv[arg] );

			}
			else if ( !strcmp(argv[arg], "-startDist")) 
			{

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_startDist = atof( argv[arg] );

			}
			else if ( !strcmp(argv[arg], "-endDist")) 
			{

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_endDist = atof( argv[arg] );

			}
			else 
			{

				// user has given an invalid command-line argument
				Usage();

			}

		}
	}

	if( g_dataFileName.size() <= 0 )
	{
		cerr << "Need a data file name on the command-line." << endl;
		exit( -1 );
	}

	if( g_bliFileName.size() <= 0 )
	{
		cerr << "Need an BLI file name on the command-line." << endl;
	}

}  // End of ParseCommandLineArguments


//////////////////////////////////////////////////////////////////////////////
// 
// Description:  The main program.  Calculates the Time(sec) for a 
//   given radius(feet), velocity (mph) and distance (ft) for a particular 
//   vehicle name and type
//
// Arguments: Default
//
// Inputs: Reads data from the data file (datapoints.txt). The data file has in
//   it, the starting points in the form of x y z. All the points are
//   seperated by a space. A  new point is in the next line.
//
// Outputs: Minimum Radius for each scenario with their respective starting
//    points.  Also outputs to three files. The interpolation part, 
//    producing the look up table, and the header file part are the three 
//    parts respectively.
// 
//////////////////////////////////////////////////////////////////////////////

void main( int argc, char** argv )
{
	const double cMPH_TO_MS = 0.44703;    // mph to meters/sec
	const double cMS_TO_MPH = 2.23700;    // meters/sec to mph
	const double cRAD_TO_DEG = 57.29577;
	const double cDEG_TO_RAD = 0.01745;
	const double cMETERS_TO_FEET = 3.28;
	const double cFEET_TO_METERS = 0.304878;

	const double cSTABILIZED_VEL = 35.0;   // mph
	const double cSTABILIZED_DIST = 20.0;  // m
	const double cLC_TARGET_ANGLE = 15.0;  // degrees
	const double cDIST_TO_GO      = 50;     // feet

	double stabilizeVel = cSTABILIZED_VEL;
	double stabilizeDist = cSTABILIZED_DIST;
	double targetAngle = cLC_TARGET_ANGLE;

	//
	//time taken to travel a distance at a given velocity and radius
	//
	double timeTaken;

	//
	//minRadArray takes in all the minimum radii of all the scenarios run
	//with their respective starting points.
	//
	double minRadArray[42][10];

	//
	//The 4DArray that takes in the time(s) taken. The indeces are as follows
	//timeAt...[i][j][k][l] where i represents loop index for a vehicle name.
	//There are 41 such vehicle names. j represents radius, k - velocity and 
	//l - distance.
	//
	float time[42][6][12][10];

	//
	//Getting data from the command line
	//
	ParseCommandLineArguments( argc, argv );

	//
	//Initializing CVED with tireFailure.bli
	//
	CCved cved;
	string cvedErr;
	float deltaT = 1.0 / g_Freq;

	//
	//Checking to see if the configuration is a success or failure
	//
	bool success =
		cved.Configure( CCved::eCV_MULTI_USER, deltaT, g_DynaMult );
	if (!success)
	{
		cout << "Configure Failure" << endl;
		exit(-1);
	}
	
	if ( cved.Init(g_bliFileName, cvedErr ) == false ) 
	{
		cerr << "Cved initialization failed with lri file '"; 
		cerr << g_bliFileName << "'." << endl << "Error message: " ;
		cerr << cvedErr << endl;
		cerr << "Quiting ...." << endl;

		exit( -1 );

	}
	else
	{
		cout <<"CVED Initialization has worked successfully" << endl;
		cved.SetDebug( g_CvedDebug );
	}

	//
	//Read starting points to be started in the scenario from the data file
	//
	FILE *fpDataFile;
	if ( (fpDataFile = fopen(g_dataFileName.c_str(), "r")) == NULL) {
		cout << "File does not exist" << endl;
		exit(1);
	}

	//
	//Declaring a CPoint3D for storing the starting points from dataFile.txt
	//
	CPoint3D storeStartPos[20];	
	
	char str[300], *stopStr;
	int c, iterator = 1;

	//
	//numRcPoints is the number of times the scenario is run. Or, it is the 
	//number of starting points that the user specifies in the dataFile.txt
	//
	int numRcPoints = 0;

	while ( (c = fscanf(fpDataFile, "%s", str)) != EOF ){
		if (iterator%3 == 1){
			storeStartPos[numRcPoints].m_x = strtod(str, &stopStr);
		}
		if (iterator%3 == 2){
			storeStartPos[numRcPoints].m_y = strtod(str, &stopStr);
		}
		if (iterator%3 == 0){
			storeStartPos[numRcPoints].m_z = strtod(str, &stopStr);
		}
		numRcPoints = iterator/3;
		iterator++;
	}
	int radCount = iterator - 2;

	//
	//temp holds the names of all the vehicles
	//
	vector <char*> temp;

	//
	//Call functions that get ALL the vehicles
	//
	int numObj1 = GetVehicles(cved, temp);
	int numObj2 = GetBuses(cved, temp);
	int numObj3 = GetEmergVeh(cved, temp);
	int numObj4 = GetTrucks(cved, temp);

	//
	//Utility Vehicles do not work as of today (Aug/06/2001). Note that when
	//they do work, the (int)numObj5 should be added to (int)totNumObjs.
	//int numObj5 = GetUtilVeh(cved, temp); should also be called first
	//

	//
	//total Number of Objects stored in the temp vector defined above
	//
	int totNumObjs = numObj2 + numObj1 + numObj3 + numObj4;

	//
	//Printing out the vehicle IDs
	//
	for (int veh = 0; veh < totNumObjs; veh++) {
		const slObj* pSolObjs = cved.GetSol().QryByName( temp[veh] );
		cout << temp[veh];
		cout << "\tID["<<veh+1<<"]= " << pSolObjs -> GetId() << endl;
	}

	//
	//Array to store all the SOL IDs with a particular index
	//
	int idArray[45];

	int rad = 0;
	cvEObjType objType;

	for (int obj = 0; obj < totNumObjs; obj++)
	{
		const string objName = temp[obj];
		const slObj* pSolObj = cved.GetSol().QryByName( objName );
		string solName;

		//
		//If name valid, assign that type (eCV) and name to the type of object
		//and to the name of object respectively
		//
		if (pSolObj) {
			const string categoryName = pSolObj -> GetCategoryName();
			if (categoryName == "Vehicle")
				objType = eCV_VEHICLE;
			else if (categoryName == "Buses")
				objType = eCV_BUS;
			else if (categoryName == "EmergVeh")
				objType = eCV_EMERGENCY;
			else if (categoryName == "Truck")
				objType = eCV_TRUCK;
			else objType = eCV_UTILITY_VEH;
				
			int id = pSolObj -> GetId();
			idArray[obj] = id;
		}

		//
		//Using objName as the vehicle for all the generated values
		//
		solName = objName;

		pSolObj = cved.GetSol().QryByName( solName ); 
		if ( !pSolObj ) {
			//
			// invalid SOL name...suicide
			//
			cerr << "The Dynamic Object does not EXIST due to an INVALID SOL";
			cerr << " name"<< endl;		
			exit(-1);
		}

		//
		//Begin looping for the number of starting points in the data file
		//
		for (rad = 0; rad < numRcPoints; rad++)
		{
			CPoint3D InitPos;
			//
			//Declaring the radius of Curvature variable for some maximum
			//amount
			//
			double minRadiusOfCurvature = 10000000000;

			//
			//Begin looping for the velocities at an interval of 5
			//
			int velvar;
			for (velvar = g_startVel; velvar <= g_endVel; velvar+= 5)
			{
				//
				//Beging looping for the distance to be run with interval of 10
				//
				//originally, distvar <= 150
				//
				int distvar;
				for (distvar = g_startDist; distvar <= g_endDist; distvar+= 10)
				{
					// Initialize the vehicle attributes.

					cvTObjAttr attr = { 0 };
					attr.solId = pSolObj->GetId();
					attr.xSize = pSolObj->GetXSize();
					attr.ySize = pSolObj->GetYSize();
					attr.zSize = pSolObj->GetZSize();
					attr.hcsmId = 0; 	

					//
					//Initial Position is assigned from the CPoint3D
					//containing the x, y, z coordinates for each loop.
					//
					InitPos.m_x = storeStartPos[rad].m_x;
					InitPos.m_y = storeStartPos[rad].m_y;
					InitPos.m_z = storeStartPos[rad].m_z;

					//
					//First, creating a RoadPos for the initial position
					//
					CRoadPos initRoadPos (cved, InitPos);

					//
					//Now to get the InitTan and InitLat for a curved road
					//
					CVector3D InitTan = initRoadPos.GetTangent();
					CVector3D InitLat = initRoadPos.GetRightVec();

					//
					// The CreateDynObj returns a pointer of type 'CDynObj'. 
					//
					CDynObj* pDynObj = cved.CreateDynObj( 
													solName, 
													objType, 
													attr, 
													&InitPos, 
													&InitTan, 
													&InitLat
													); 
					if ( !pDynObj ) 
					{
						cout << "Error!! stopVehicle the CreateDynObj";
						cout << " function" << endl;
					}
					if ( !pDynObj -> IsValid() )
					{
						cout << "Error!! The pDynObj is not valid" << endl;
					}
					else{}

					CVehicleObj* pVehicle =
						dynamic_cast <CVehicleObj *> ( pDynObj );

					InitializeDynamicsVars ( objType,
											 pSolObj,
											 pVehicle );
					
					long currFrame = 0;

					int stableVelFrameCount = 0;
					double lastVel = pVehicle -> GetVel();
					CPoint3D startingPosition = pVehicle -> GetPos();
					float initialStabilizeDist = stabilizeDist;
					CRoadPos startRoadPos(cved, startingPosition);

					//
					//Note: Assigning velvar and distvar to stabilizeVel and 
					//stabilizeDist respectively.
					//distvar also assigned to targFinalDist because
					//stabilizeDist has to change but not targFinalDist
					//
					stabilizeVel = (double)velvar * cMPH_TO_MS;
					stabilizeDist = (double)distvar * cFEET_TO_METERS;
					float targFinalDist = (double)distvar * cFEET_TO_METERS;
					int count = 0;

					//
					//Begin dynamics
					//

					//
					//It is better to keep the below lines for testing purpose
					//in future...just in case!!!!
					//
					//Testing............
					//cout << "(obj) " << obj << "\t(rad) ";
					//cout << rad << "\t(vel) ";
					//cout << velvar << "\t(dist) " << distvar;
					//cout << "\tBefore and";
					//End Testing........
					while ( 1 )
					{
						//
						//Creating Dist to Advance based on Length of the Obj.
						//First, getting the ID of the Vehicle Obj
						//
						int ID = pDynObj -> GetId();

						//
						//Create the Object Attribute
						//
						CObjAttr objectAttr(cved, ID);

						//
						//Get length of the object
						//
						double objLength = objectAttr.GetXSize(); // feet

						//
						//Getting the current position
						//
						CPoint3D currPos = pVehicle -> GetPos();

						//
						//Creating a RoadPos
						//
						CRoadPos targRoadPos(cved, currPos);

						//
						//BREAK OFF the program if targRoadPos not valid
						//
						if( !targRoadPos.IsValid() ) break;

						//
						//Computing targPos for curved pavements.
						//
						float distToAdvance = objLength/2 + 1.2;

						//
						//Modify targRoadPos by the distance to advance
						//
						targRoadPos.Travel(distToAdvance);

						//
						//Declaring targPos as a CPoint3D
						//
						CPoint3D targPos = targRoadPos.GetXYZ();

						//
						// Setting control inputs.
						//
						pVehicle -> SetTargPos ( targPos );
						pVehicle -> SetTargDist ( stabilizeDist );
						pVehicle -> SetTargVel ( stabilizeVel );

						if ( currFrame == 0 ) {
							pVehicle -> SetInitDynamics(true);
						}
						else {
							pVehicle -> SetInitDynamics(false);
						}

						//
						// Run maintainer and execute dynamics.
						//
						cved.Maintainer();
						cved.ExecuteDynamicModels();
						cved.ExecuteDynamicModels();

						//
						//Calculations for exiting the while loop ...
						//Requirement to exit: Exit if the distance is reached
						//
						CRoadPos currentRoadPos (cved, currPos);

						float testDist = currentRoadPos.GetDistance() -
							startRoadPos.GetDistance();

						float actTestDist = fabs(testDist * cFEET_TO_METERS);
						float distanceTravelled = actTestDist;

						//
						//stabDist > 1.5(m) then stabDist has to decrease as
						//the vehicle keeps going forward.
						//If < 1.5(m), keep stabDist 1.5(m) so as facilitate
						//for the length of the car.
						//
						if (stabilizeDist > 1.5) {
							stabilizeDist = targFinalDist - distanceTravelled;
						}
						else if (stabilizeDist <= 1.5) stabilizeDist = 1.5;
											
						timeTaken = currFrame/30.0;

						//
						//Calculations of min radius of curvature
						//
						double radiusOfCurvature =
							currentRoadPos.GetCurvature();

						if (radiusOfCurvature <= minRadiusOfCurvature){
							minRadiusOfCurvature = radiusOfCurvature;
						}

						//
						//Better to keep the testing code rather than taking
						//it off...just in case!!!!
						//
						//
						//TESTING BY PRINTING OUT currPos---------------
						//cout << "dist- ";
						//cout << distanceTravelled*cMETERS_TO_FEET;
						//cout << "\tvel- ";
						//cout << (pVehicle -> GetVel())*cMS_TO_MPH;
						//cout << "\trad = " << radiusOfCurvature;
						//cout << "\tcurrPos = " << currPos << endl;
						//END TESTING-----------------------------------
						//

						//
						//Writing minRad into an array. So the array keeps
						//track of only the radius that is minimum fo
						//each scenario and the object for which it had run
						//
						minRadArray[obj][rad] = minRadiusOfCurvature;

						CVector3D currTan = currentRoadPos.GetTangent();
						CVector3D currLat = currentRoadPos.GetRightVec();

						//	
						//if the current position reaches the finalTargDist,
						//quit
						//
						if (distanceTravelled >= targFinalDist) break;

						//
						//Also, quit if vel attained does not make sense with
						//distance covered
						//Note: velocity is in terms of velvar and
						//      distance is in terms of distvar
						//

						if ((velvar == 0) || (distvar == 0)) break;

						if ((velvar == 5) && (distvar >= 50)) break;
//						if ((velvar == 10) && (distvar > 90)) break;

						//
						//Increment only if the loop does not quit
						//
						currFrame++;

					}//end of while loop

					//
					//Purposefully not taking the line off
					//
					//cout << " After" << endl;
					//

					//
					//Catch basically catches the "eTOffRoad" exception and
					//helps the user to continue with the loop, without
					//abnormally quitting the loop
					//

					//
					//As of now, Aug 06, 2001, it is better not to erase
					//this catch loop. But here, it has been commented out.
					//NOTE: I have erased the 'try' part of this program
					//
#if 0
					catch (...){
						//
						//Print out obj, rad, vel and dist where exception
						//occurs
						//
						cout << endl << "Exception Here:" << endl;
						cout << "object#: " << obj;
						cout << "\tradLoop# - " << rad;
						cout << "\tvelLoop# - " << velvar;
						cout << "\tdistLoop# - " << distvar << endl << endl;
					}
#endif

					//
					//Storing the time taken in the 3D Array
					//
					time[obj][rad][velvar/5][distvar/10] =
						timeTaken;
					
					CPoint3D currPos1 = pVehicle -> GetPos();

					CCved::TIntVec objs;
					cved.GetAllDynamicObjs( objs );

					//
					//cout << "number of objects is: " << objs.size() << endl;
					//

					//Kill the dynamics now. It will again be created at the
					//top. This is because we do not want multiple dynamics
					//on a lane
					//
					pDynObj->~CDynObj();

				}//end of "distvar" for loop
				
			}//end of "velvar" for loop

		}//end of "rad" for loop

	}//end of "objects" for loop

	//
	//Printing out the starting Points along with their minimum radii
	//
	cout << endl << endl << "The Points Scanned Are:" << endl << endl;
	for (int jj = 0; jj < totNumObjs; jj++) {
		for (int ii = 0; ii < numRcPoints; ii++) {
			cout << "Starting Position: ";
			cout << "x = "<< storeStartPos[ii].m_x;
			cout << "\ty = " << storeStartPos[ii].m_y;
			cout << "\tz = " << storeStartPos[ii].m_z;
			cout << "\tMinimum Radius is: " << minRadArray[jj][ii] << endl;
			cout << "tot = " << totNumObjs << endl;
			cout << endl;
		}
	}

	//
	// Build a path to the sabeh/hcsm/usersrc directory.  The generated
	// files should be placed in this directory.
	//
	char userSrcPath[4096];
	if( getenv( "NADSSDC_BIN" ) ) 
	{
		strcpy( userSrcPath, getenv( "NADSSDC_BIN" ) );
		//strcat( pBinPath, "\\" );
	}
	else
	{
		cerr << "AdoRcData: the NADSSDC_BIN environment variable needs ";
		cerr << "to be defined.  Exit." << endl;
		exit( -1 );
	}
	strcat( userSrcPath, "..\\sabeh\\hcsm\\usersrc\\" );

	//
	// Open the interpolation function source file.
	//
	char interpFileName[4096];
	strcpy( interpFileName, userSrcPath );
	strcat( interpFileName, "ado_rc_interp.cxx" );
	FILE* pInterpFuncFile = fopen( interpFileName, "w" );

	WriteToInterpolationFunction(
			numRcPoints,
			pInterpFuncFile,
			minRadArray,
			totNumObjs
			);

	//
	//This file takes the look up table which is just the 4Darray and the
	//array that holds the sol ID list for ALL the vehicles
	//
	char lookupSourceFileName[4096];
	strcpy( lookupSourceFileName, userSrcPath );
	strcat( lookupSourceFileName, "ado_rc_lookup.cxx" );
	FILE* pLookupSourceFile = fopen( lookupSourceFileName, "w" );

	WriteToLookUpCxx(
			numRcPoints,
			pLookupSourceFile,
			time,
			totNumObjs,
			idArray
			);

	//
	//This file takes up just the name of the array defined as extern (so as 
	//to counter multiple declaration problems
	//
	char lookupHeaderFileName[4096];
	strcpy( lookupHeaderFileName, userSrcPath );
	strcat( lookupHeaderFileName, "ado_rc_lookup.h" );
	FILE* pLookupHeaderFile = fopen( lookupHeaderFileName, "w" );

	WriteToLookUpHeader(numRcPoints, pLookupHeaderFile, totNumObjs);
}
//END OF MAIN FUNCTION