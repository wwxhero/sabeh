#include "hcsmspec.h"
#include "action.h"
#include "SpotlightAction.h"
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

CSpotlightAction::CSpotlightAction( const CActionParseBlock* pBlock, CHcsmCollection* pColl)
{
	assert(pBlock);
	m_pHC = pColl;
	m_actionStr = pBlock->GetCommand(); //<turn on/off action;
    m_targetLight = pBlock->GetTargetID();	
    InitCandidateSet( pBlock );
}
CSpotlightAction& 
CSpotlightAction::operator=( const CSpotlightAction& cRhs ){
	if (this != &cRhs){
        m_pHC = cRhs.m_pHC;
	}
	return *this;
}
CSpotlightAction::CSpotlightAction(const CSpotlightAction&  cRhs){
	*this =  cRhs;
}
////////////////////////////////////////////////////////////////////////
///\remark
///		copies headlight command to m_sHeadlightScenarioControl
////////////////////////////////////////////////////////////////////////
void CSpotlightAction::Execute( const set<CCandidate>* cpInstigators ){
	
	set<CCandidate> candidateSet;
 	m_candidateSet.GetCandidates( *(m_pHC->GetCved()), candidateSet, *m_pHC );
    CSpotlightCommand cmd;
	//
	// If we use the instigator set and the user gave us a set of 
	// instigators, then Execute the action on the instigators that 
	// are in the candidate set.
	//
    int targetId = -1;
	bool useInstigators = m_useInstigators && cpInstigators != 0;
	if( useInstigators )
	{
#if SET_DIAL_DEBUG > 1
		gout << "Using instigator set." << endl;
#endif

		set<CCandidate>::const_iterator itr;
		for( 
			itr = cpInstigators->begin(); 
			itr != cpInstigators->end(); 
			itr++ 
			)
		{
            if (itr->m_cvedId > 0){
                targetId = itr->m_cvedId;
                break;
            }
		}
	}
	//
	// If we don't use the instigator set, then disregard the 
	// instigators and Execute the action on the candidates.
	//
	else
	{
#if SET_DIAL_DEBUG > 1
		gout << "Not using instigator set." << endl;
#endif

		set<CCandidate>::const_iterator itr;
		for( itr = candidateSet.begin(); itr != candidateSet.end(); itr++ )
		{
            if (itr->m_cvedId > 0){
                targetId = itr->m_cvedId;
                break;
            }

		}
	}
    cmd.m_target = targetId;
	if (m_actionStr == "Attach"){
        if ( targetId == -1){
            gout<<"Syntax Error For "<< CSpotlightAction::Name()<<" Attach has no target (missing predicate set?)"<<m_actionStr<<endl;
            return;
        }
        
        cmd.m_command = CSpotlightCommand::eAttach;
        cmd.m_color[0] = (float)(m_targetLight);
    }else if (m_actionStr== "TurnOn"){
        cmd.m_command = CSpotlightCommand::eTurnOn;	
        cmd.m_target = (int)(m_targetLight);
	}else if (m_actionStr== "TurnOff"){
        cmd.m_command = CSpotlightCommand::eTurnOff;
        cmd.m_target = (int)(m_targetLight);
	}else{
        gout<<"Syntax Error For "<< CSpotlightAction::Name()<<" Action in string->"<<m_actionStr<<endl;
    }
    CHcsmCollection::AddSpotlightCommand(cmd);
	
}

CSpotlightAction::~CSpotlightAction(void)
{
}