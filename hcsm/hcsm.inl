#include <typeinfo.h>

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the class' name.
//
// Remarks:  This inline function returns a pointer to a string containing 
//   this class' name.  It has been designed this way so that it can be 
//   used directly in cout statement.
//
// Arguments:
//
// Returns:  Returns an pointer to a const char containing the class' name.
//
//////////////////////////////////////////////////////////////////////////////
inline const char* 
CHcsm::MyName()
{

	// return the class' name
	return typeid( *this ).name();

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the Hcsm's name.
//
// Remarks:  This function returns the Hcsm's template name.
//
// Arguments:
//
// Returns:  A reference to an STL string that represents the Hcsm's
//   template name.
//
//////////////////////////////////////////////////////////////////////////////
inline const string& 
CHcsm::GetName() const
{

	return m_name;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the number of children for an HCSM.
//
// Remarks:  This function returns a number indicating how many child HCSMs
//   an HCSM has.
//
// Arguments:
//
// Returns:  An integer that indicates the number of child HCSMs.
//
//////////////////////////////////////////////////////////////////////////////
inline int 
CHcsm::NumChildren()
{

	// return the number of child hcsms
	return m_numChildren;

}
inline int 
CHcsm::GetPriorityLevel(){
    return m_priorityLevel;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Indicates whether an HCSM is a root HCSM.
//
// Remarks:  This function returns a boolean that indicates whether an
//   HCSM instance is a root or top-level HCSM.  A root HCSM has no parent
//   HCSMs.
//
// Arguments:
//
// Returns:  An boolean that indicates whether an HCSM is a root HCSM.
//
//////////////////////////////////////////////////////////////////////////////
inline bool 
CHcsm::IsRoot()
{

	// is this hcsm instance a root hcsm
	return m_root;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns an HCSM instance's state.
//
// Remarks:  This function returns a value to indicate whether an HCSM
//   instance is active or inactive.  An active HCSM executes its activity
//   functions everytime its Execute function is called.
//
// Arguments:
//
// Returns:  An enumeration:
//   eACTIVE   --> this hcsm is active
//   eINACTIVE --> this hcsm is inactive
//
//////////////////////////////////////////////////////////////////////////////
inline EHcsmState 
CHcsm::GetState()
{

	return m_state;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets an HCSM instance's state.
//
// Remarks:  This function accepts a value that sets an HCSM instance's
//   state.  An HCSM can either be active or inactive.
//
// Arguments:
//   state - An enumeration:
//             eACTIVE   --> set hcsm state to active
//             eINACTIVE --> set hcsm state to inactive
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CHcsm::SetState( const EHcsmState state )
{

	m_state = state;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Gets the HCSM's execution time.
//
// Remarks:  This function returns a value to indicate how long it takes
//   to execute an HCSM.  The HCSM collection sets this value.
//
// Arguments:
//
// Returns:  A double that represents the time it takes to execute an HCSM.
//
//////////////////////////////////////////////////////////////////////////////
inline double 
CHcsm::GetExecutionTime()
{

	return m_executionTime;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the HCSM's execution time.
//
// Remarks:  This function accepts a value and adds it to the HCSM exeuction
//   time.
//
// Arguments:
//   executionTime - The time it takes to execute this HCSM.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CHcsm::AddToExecutionTime( const double executionTime )
{

	m_executionTime += executionTime;

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns an HCSM instance's debugging level.
//
// Remarks:  This function returns a value to indicate an HCSM's
//   debugging level.
//
// Arguments:
//
// Returns:  An enumeration: EDebugLevel.
//
//////////////////////////////////////////////////////////////////////////////
inline CHcsmDebugItem::ELevel 
CHcsm::GetDebugLevel() const
{
	return m_debugLevel;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets an HCSM instance's debugging level.
//
// Remarks:  This function accepts a value that sets an HCSM instance's
//   debugging level.
//
// Arguments:
//   state - An enumeration: EDebugLevel.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CHcsm::SetDebugLevel( const CHcsmDebugItem::ELevel debugLevel )
{

	m_debugLevel = debugLevel;

}

inline void 
CHcsm::SetDebugMode( EDebugMode mode )
{
	m_debugMode = mode;
}

inline EDebugMode 
CHcsm::GetDebugMode(void) const
{
	return m_debugMode;
}


