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

inline const char* CHcsmCollection::MyName()
{

	// return the class' name
	return typeid( *this ).name();

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the number of top-level or root HCSM instances.
//
// Remarks:  This function returns the current number of top-level or 
//   root HCSM instances.
//
// Arguments:
//
// Returns:  Returns an integer that represents the number of root HCSM
//   instances.
//
//////////////////////////////////////////////////////////////////////////////
int CHcsmCollection::NumHcsm() const
{

	// return the number of root HCSMs
	return m_numHcsm;

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the current frame number.
//
// Remarks:  This function returns the current frame number of the HCSM 
//   system.  The frame number is increased by one everytime ExecuteAllHcsms
//   is called.
//
// Arguments:
//
// Returns:  Returns an integer that represents the current HCSM system 
//   frame number.
//
//////////////////////////////////////////////////////////////////////////////
int CHcsmCollection::GetFrame() const
{

	// return the current frame number
	return m_frame;

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns a time step's duration in seconds.
//
// Remarks:  This function returns a time step's duration in seconds.
//
// Arguments:
//
// Returns:  Returns an integer that represents the current HCSM system 
//   frame number.
//
//////////////////////////////////////////////////////////////////////////////
double CHcsmCollection::GetTimeStepDuration() const
{

	return m_timeStepDuration;

}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  add a debug item to the collection's queue
//
// Remarks:  This function returns a time step's duration in seconds.
//
// Arguments:
//
// Returns:  Returns an integer that represents the current HCSM system 
//   frame number.
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CHcsmCollection::LogDebugItem(const CHcsmDebugItem &item)
{
	m_DebugItems.push(item);
	if ( m_DebugItems.size() > cMAX_DEBUG_ITEMS ) {
		m_DebugItems.pop();
	}
}


inline bool 
CHcsmCollection::GetDebugItem(CHcsmDebugItem &item)
{
	if ( m_DebugItems.empty() ) return false;
	
	item = m_DebugItems.front();
	m_DebugItems.pop();
	return true;
}


