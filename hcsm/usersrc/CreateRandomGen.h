#pragma once
#include "hcsmspec.h"
#include "action.h"

class CCreateRandomGen :
	public CAction
{
public:
	public:
		// Constructor
		CCreateRandomGen( const CActionParseBlock*, CHcsmCollection* );

		// Copy constructors
		CCreateRandomGen( const CCreateRandomGen& );

		// Destructor
		~CCreateRandomGen();

		// Assignment operator
		CCreateRandomGen& operator=( const CCreateRandomGen& );

		// Execute operation
		void Execute( const set<CCandidate>* instigators = 0 );

		inline const char* GetName() const { return m_sName; };
        inline static const char* Name() { return m_sName; };
	protected:
		CHcsmCollection*	m_pHC;
		std::string m_name;
		std::vector<long> m_seed;
		static const char m_sName[];
};

