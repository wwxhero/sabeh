/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: ChangeCabSetting.cpp,v 1.4 2013/03/25 15:09:13 IOWA\dheitbri Exp $
// Author(s):   David Heitbrink
// Date:        August, 2002
//
// Description:  Definition of the CChangeCabSetting class.  
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////
#include "ChangeCabSetting.h"


#include "hcsmcollection.h"
#include "genhcsm.h"
#include <boost/tokenizer.hpp>
#include <string>

#define SET_DIAL_DEBUG 0

/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks: The pointer to the root HCSM collection is where the new HCSMs
//			will be deleted.
//
// Arguments:	
//	pBlock - A pointer to the CActionParseBlock associated with	this CActn.
//	pHC - A pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CChangeCabSetting::CChangeCabSetting(
			const CActionParseBlock *cpBlock, 
			CHcsmCollection *pHC
			)
{
	m_pHC = pHC;
	CActionParseBlock::pbTDial dial = cpBlock->GetDial();
	m_dialName = dial.name;
	m_cabSettingString = cpBlock->GetCabSettingsString();
    m_delay = cpBlock->GetDelay();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CResetDialActn
//
// Remarks: 
//
// Arguments: CResetDialActn to be copied into current CResetDialActn
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CChangeCabSetting::CChangeCabSetting( const CChangeCabSetting& cRhs )
{
	*this = cRhs;
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
CChangeCabSetting::~CChangeCabSetting() {}

/////////////////////////////////////////////////////////////////////////////
///\remark 
/// Description: 
/// The assignment operator, which assigns the contents of the
///	current action to the parameter and returns a reference  to the 
/// urrent object.
///
///
///\par  cRhs reference to the CChangeCabSetting to assign
/// 
///
/////////////////////////////////////////////////////////////////////////////
CChangeCabSetting&
CChangeCabSetting::operator=( const CChangeCabSetting& cRhs )
{

	if( this != &cRhs )
	{
		m_pHC            = cRhs.m_pHC;
		m_dialName       = cRhs.m_dialName;
		m_cabSettingString = cRhs.m_cabSettingString;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
///\brief
///     Write out to the cab operations string
///\remark
///     This function uses a lock to on cabSettings, it can block here
///     forever
/////////////////////////////////////////////////////////////////////////////
void 
CChangeCabSetting::Execute( const set<CCandidate>* cpInstigators )
{
	int currPos = CHcsmCollection::m_CabOperationsSize;
    typedef boost::tokenizer<boost::char_separator<char> > 
      tokenizer;
	stringstream converter;
    boost::char_separator<char> sep("-;|, \t:");
    tokenizer tokens(m_cabSettingString, sep);
    CHcsmCollection::m_cabSettingsCriticalSection.Lock();
    for (tokenizer::iterator itr = tokens.begin(); itr != tokens.end(); ++itr){
		string tok = *itr;
		if (tok == "Switch"){
			CHcsmCollection::m_sCabOperations[currPos].eOptionType = eSwitch;
			itr++; 
			if (itr == tokens.end()){
				gout<<"CChangeCabSetting error in dialstring:"<<m_cabSettingString<<endl;
			}else{
				CHcsmCollection::m_sCabOperations[currPos].element = *itr;
				itr++; 
				if (itr == tokens.end()){
					gout<<"CChangeCabSetting error in dialstring:"<<m_cabSettingString<<endl;
				}else{
					string tok = *itr;
					converter<<tok;
					converter>>CHcsmCollection::m_sCabOperations[currPos].option1;
					if (converter.fail()){
						gout<<"CChangeCabSetting error in dialstring, expecting number:"<< m_cabSettingString<<endl;
					}
					CHcsmCollection::m_CabOperationsSize++;
				}
			}
		}
    }
    CHcsmCollection::m_cabSettingsCriticalSection.UnLock();
}