#pragma once
#include "action.h"
/////////////////////////////////////////////////////////////////////////////
///\brief
///		This class loads variables from a text file	
///\remark
///		action is for setting a state on the headlights. At this point
///		the functionality of this action is subject to change, and is
///		not finalized.
///\todo
///		This action right now just turns off and on headlights, it really 
///		needs fleshed out, it needs to register itself with the activity log
///		and print errors to gout. The mechanism for changing headlight states
///		is not ideal either. 
/////////////////////////////////////////////////////////////////////////////
class CSetEnvCondition :
	public CAction
{
public:
	CSetEnvCondition( const CActionParseBlock* pBlock, CHcsmCollection* pColl);
	~CSetEnvCondition(void);
	CSetEnvCondition& operator=( const CSetEnvCondition& cRhs );
	CSetEnvCondition(const CSetEnvCondition&);
	void Execute( const set<CCandidate>* cObjs = 0 ) override;
	inline const char* GetName() const { return Name(); }
    inline static const char* Name() {return "SetEnvCondition";};
private:
	std::string m_setting; //<turn on/off action;
    std::string m_value;
    bool m_isVar;
    CHcsmCollection* m_pHC;
};
