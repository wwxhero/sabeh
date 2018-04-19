#pragma once
////////////////////////////////////////////////////////////////////////////////////////////////////
///\brief
///		This class saves variables to a text file	
///\remark
///		this class saves variables delimited by ; to a text file
///		the format of this file is:
///		varname	 value
///		with each variable name and value pair being on its own line
////////////////////////////////////////////////////////////////////////////////////////////////////
class CStoreVar :
	public CAction
{
public:
	CStoreVar( const CActionParseBlock*, CHcsmCollection* );
	CStoreVar(  const CStoreVar& );
	~CStoreVar(void);
	CStoreVar& operator=( const CStoreVar& cRhs );
	void Execute( const set<CCandidate>* cObjs = 0 ) override;
	inline const char* GetName() const  {return "StoreVar";};
private:
	string m_fileName; //< name of the text file to save the variables to
	string m_varName; //<tab delimited list of variables
};

