#pragma once
////////////////////////////////////////////////////////////////////////////////////////////////////
///\brief
///		this class performs operations on Variable Queues
////////////////////////////////////////////////////////////////////////////////////////////////////
class CVarQueueOperationAction :
	public CAction
{
public:
	CVarQueueOperationAction( const CActionParseBlock*, CHcsmCollection* );
	CVarQueueOperationAction(  const CVarQueueOperationAction& );
	~CVarQueueOperationAction(void);
	CVarQueueOperationAction& operator=( const CVarQueueOperationAction& cRhs );
	void Execute( const set<CCandidate>* cObjs = 0 ) override;
	inline const char* GetName() const  {return "VarQueueOperation";};
private:
	string m_varOperation; //<tab delimited list of variables
};