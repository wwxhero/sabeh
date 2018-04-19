/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: startdataredactn.cxx,v 1.20 2007/10/24 21:18:05 yhe Exp $
//
// Author(s):   Matt Schikore
//
// Date:        Dec 2003
//
// Description: The definition of CStartDataredActn, a subclass of CAction.  
// 	This class causes a data reduction segment to begin.
//
/////////////////////////////////////////////////////////////////////////////

#include "startdataredactn.h"
#include "hcsmcollection.h"
#include "genhcsm.h"


/////////////////////////////////////////////////////////////////////////////
//
// Description: The constructor, which initializes its local variables.
//
// Remarks:
//
// Arguments:	
//	pBlock - pointer to the CActionParseBlock associated with
//			this CAction.
//	pHC - pointer to the root HCSM collection.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CStartDataredActn::CStartDataredActn(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			)
			: m_pHC( pHC )
{
	m_delay = cpBlock->GetDelay();
	m_data  = cpBlock->GetDataRedVarsStr();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CStartDataredActn.
//
// Remarks: 
//
// Arguments: CStartDataredActn to be copied into current CStartDataredActn.
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CStartDataredActn::CStartDataredActn( const CStartDataredActn& cRhs )
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
CStartDataredActn::~CStartDataredActn() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CStartDataredActn to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CStartDataredActn&
CStartDataredActn::operator=( const CStartDataredActn& cRhs )
{
	if( this != &cRhs)
	{
		m_pHC = cRhs.m_pHC;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The Execute function, which is called when the action
//              should be executed
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CStartDataredActn::Execute( const set<CCandidate>* ) 
{
	vector<CActionParseBlock::pbTDataRedVar> vars = CActionParseBlock::StringToDataRedVars(m_data);
	int blockNum;
	sscanf(m_data.c_str(), "%d", &blockNum);
	blockNum++; // we're now using entry #0 as a counter

	for (vector<CActionParseBlock::pbTDataRedVar>::iterator i = vars.begin(); i != vars.end(); i++) {
		int bitnum = 0;
		int type = i->type;
		while (type > 1) {
			type >>= 1;
			bitnum++;
		}

		int before = CHcsmCollection::m_sSCC_DataRed_Segments[blockNum];
		CHcsmCollection::m_sSCC_DataRed_Segments[blockNum] |= i->type;
		if (CHcsmCollection::m_sSCC_DataRed_Segments[blockNum] != before) {  // only run this if the data has actually changed.
			CHcsmCollection::SetActionStartDataRedLog(m_triggerId, i->type + 32 * blockNum, i->data, &(i->data[8]) );
//			gout << "START DATARED - " << i->type << ": " << i->data << " : " << &(i->data[8]) << endl;
			int dataRedCounter = ++m_sDataRedCounter;
			HANDLE hThread = GetCurrentThread();
			DWORD threadPri = GetThreadPriority(hThread);
			//Begin Critical Section
#if USING_RTEXPSLITE == 0
			if (CHcsmCollection::m_spRTEX)
				CHcsmCollection::m_spRTEX->m_ObjCells.LockOutpElems();
#endif
			
			//We have just locked output elements tell the OS- DO NOT INTERUPT THIS THREAD
			SetThreadPriority(hThread, REALTIME_PRIORITY_CLASS);

//			gout << " (Count " << dataRedCounter << ")";
			CHcsmCollection::m_sSCC_DataRed_Segments[0] = dataRedCounter;
			
			int copyTo = (cDR_COLSIZE + cDR_DATASIZE) * bitnum + ((blockNum-1) * (cDR_COLSIZE + cDR_DATASIZE) * 32);
			memcpy( &CHcsmCollection::m_sSCC_DataRed_Params[copyTo], i->data, cDR_COLSIZE + cDR_DATASIZE);
			memcpy( &CHcsmCollection::m_sSCC_DataRed_Params[cNUM_DATARED_PARAMS], &dataRedCounter, sizeof(int));
			//End Critical Section
#if USING_RTEXPSLITE == 0
			if (CHcsmCollection::m_spRTEX)
				CHcsmCollection::m_spRTEX->m_ObjCells.UnLockOutpElems();
#endif

			//Restore the original thread priority
			SetThreadPriority(hThread,threadPri);

//			gout << " - Segments[" << blockNum << "] |= " << i->type << "( " << CHcsmCollection::m_sSCC_DataRed_Segments[blockNum] << ")";
//			gout << " - Params[" << copyTo << "] = " << i->data << " : " << &(i->data[8]) << endl;
		}
		
	}
}

