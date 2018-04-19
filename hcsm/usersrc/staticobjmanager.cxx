/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: staticobjmanager.cxx,v 1.30 2011/05/02 20:49:29 iowa\dheitbri Exp $
 *
 * Author:  Omar Ahmad
 *
 * Date:    May, 2000
 *
 * Description:  Contains code for the static object manager HCSM.
 *
 ****************************************************************************/

#include "hcsmpch.h"

#include <pi_fstream>
#include <pi_iostream>
#include <pi_string>
#include <transmat.h>
#include <sstream>
using namespace std;

const CSol& solLib()
{
	return CCved::GetSol();
}

void
CStaticObjManager::Creation( const CSobjMngrParseBlock* cpSnoBlock )
{
	PrintCreationMessage();
}


bool
CStaticObjManager::SetObjectOptions( const string& cName, int *pValues )
{
	int cvedId;
	bool success = cved->GetObj( cName, cvedId );
	if( success )
	{
		cvEObjType objType = cved->GetObjType( cvedId );
		bool compositeObj = objType == eCV_COMPOSITE_SIGN;
		if( compositeObj )
		{
			success = cved->SetObjCompositeOptions( cvedId, pValues );
			return success;
		}
		else
		{
			// send only the first option for non-composite objects
//		gout << "StaticObjMan: SetOption(" << name << " " << value << endl;
			cved->SetObjOption( cvedId, pValues[0] );
			return true;
		}
	}

	return false;
}
bool 
CStaticObjManager::SetObjectAudioState( const string& name, short val){
	int cvedId;
	bool success = cved->GetObj( name, cvedId );
	if( success )
	{
		cved->SetObjAudioState(cvedId,val,true);
	}
	return success;
}
bool 
CStaticObjManager::SetObjectVisualState( const string& name, short val ){
	int cvedId;
	bool success = cved->GetObj( name, cvedId );
	if( success )
	{
		cved->SetObjVisualState(cvedId,val,true);
	}
	return success;
}

void 
CStaticObjManager::PostActivity( const CSobjMngrParseBlock* cpSnoBlock )
{
	const int cBUFSIZE = 1024;
	static char buffer[cBUFSIZE];
	char ch;
	static int* spNums = new int[cMAX_PIECES_IN_COMPOSITE_SIGN];

	//
	// Dial string format:
	//   name : option [ : name : option ... ]
	//
	if( m_dialSetOption1.HasValue() ) 
	{
		// reset values in composite structure
		int n;
		for( n = 0; n < cMAX_PIECES_IN_COMPOSITE_SIGN; n++ )
		{
			spNums[n] = -1;
		}

		// read the dial
		string value = GetDialSetOption1();
		istringstream istr( value.c_str() );
		while( !istr.eof() ) 
		{
			istr.get( buffer, cBUFSIZE, ':' );	// read the static obj name
			istr.get( ch );                     // read the ':'

			int childNum = 0;
			do 
			{
				istr >> spNums[childNum];              // switch
				istr >> ch;
				childNum++;
			} 
			while( (ch == ',') && (childNum < 10) );
			if( childNum > 0 ) SetObjectOptions( buffer, spNums );
			if( !istr.eof() )  istr.get( ch );        // :
		}

		m_dialSetOption1.SetNoValue();
	}
	if( m_dialSetOption2.HasValue() ) 
	{
		// reset values in composite structure
		int n;
		for( n = 0; n < cMAX_PIECES_IN_COMPOSITE_SIGN; n++ )
		{
			spNums[n] = -1;
		}

		string value = GetDialSetOption2();
		istringstream istr( value.c_str() );
		while( !istr.eof() ) 
		{
			istr.get( buffer, cBUFSIZE, ':' );
			istr.get( ch );
			istr >> spNums[0];
			if( istr ) SetObjectOptions( buffer, spNums );
			if( !istr.eof() )  istr.get( ch );        // :
		}

		m_dialSetOption2.SetNoValue();
	}
	if (m_dialVisualState.HasValue()){
		string value = m_dialVisualState.GetValueStr();
		stringstream istr(value);
		istr.get( buffer, cBUFSIZE, ':' );
		istr.get( ch );
		unsigned short bitMask =0;
		istr>>bitMask;
		SetObjectVisualState(buffer,bitMask);
		m_dialVisualState.SetNoValue();


	}
	if (m_dialAudioState.HasValue()){
		string value = m_dialAudioState.GetValueStr();
		stringstream istr(value);
		istr.get( buffer, cBUFSIZE, ':' );
		istr.get( ch );
		unsigned short bitMask =0;
		istr>>bitMask;
		SetObjectAudioState(buffer,bitMask);
		m_dialAudioState.SetNoValue();
	}

}  // end of PostActivity


void 
CStaticObjManager::Deletion( const CSobjMngrParseBlock* cpSnoBlock )
{
	PrintDeletionMessage();
}