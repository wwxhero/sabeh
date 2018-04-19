#include "CreateRandomGen.h"
#include "hcsmcollection.h"

const char CCreateRandomGen::m_sName[] = "CreateRandomNumberGenerator";
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
CCreateRandomGen::CCreateRandomGen(
			const CActionParseBlock* cpBlock, 
			CHcsmCollection* pHC
			) 
{
	m_pHC = pHC;
	InitCandidateSet( cpBlock );
	m_name = cpBlock->GetVarName();
	m_seed = cpBlock->GetSeed();
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The copy constructor, copy the parameter into the current 
//				CCreateRandomGen
//
// Remarks: 
//
// Arguments: CCreateRandomGen to be copied into current CCreateRandomGen
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CCreateRandomGen::CCreateRandomGen( const CCreateRandomGen& cRhs )
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
CCreateRandomGen::~CCreateRandomGen() {}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator, which assigns the contents of the
//				current action to the parameter and returns a reference  
//				to the current object
//
// Remarks: 
//
// Arguments: reference to the CCreateRandomGen to assign
//
// Returns: 
//
/////////////////////////////////////////////////////////////////////////////
CCreateRandomGen &
CCreateRandomGen::operator=( const CCreateRandomGen& cRhs )
{

	if(this != &cRhs)
	{
		m_pHC			 = cRhs.m_pHC;
		m_candidateSet	 = cRhs.m_candidateSet;
		m_useInstigators = cRhs.m_useInstigators;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: The evaluate operation, which deletes the indicated HCSMs
//
// Remarks: This function deletes all the HCSMs that pass through the 
// 	candidate set filters.  If the instigator set is used, then the 
// 	filters are applied to the instigator set.  Otherwise, they are
// 	applied to all the objects in simulation.
//
// Arguments: 
// 	cpInstigators - An optional parameter that contains the HCSM IDs of the
//		objects which caused the trigger to fire.
//  sequentialActions - An optional parameters that is ignored.
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void 
CCreateRandomGen::Execute( const set<CCandidate>* cpInstigators )
{
	m_pHC->CreateRandomNumberGenerator(m_name,m_seed);
}

