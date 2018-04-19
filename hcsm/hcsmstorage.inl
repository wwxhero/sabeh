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

inline const char* CHcsmStorage::MyName()
{

	// return the class' name
	return typeid( *this ).name();

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Indicates whether the storage area holds a valid value.
//
// Remarks:  This function returns a boolean indicating whether the storage
//   area holds a valid value.  A valid value is one that has been
//   assigned specifically by the user.
//
// Arguments:
//
// Returns:  Returns a boolean that indicates whether the storage area
//   holds a valid value.
//
//////////////////////////////////////////////////////////////////////////////
inline bool CHcsmStorage::HasValue()
{

	return m_hasValue;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets a storage area's value to be nothing or invalid.
//
// Remarks:  This function sets a storage area's value to be nothing or
//   invalid.
//
// Arguments:
//
// Returns:  
//
//////////////////////////////////////////////////////////////////////////////
inline void CHcsmStorage::SetNoValue()
{

	m_hasValue = false;

}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the HcsmStorage element's name.
//
// Remarks:  This function returns the HcsmStorage element's name.
//
// Arguments:
//
// Returns:  A reference to an STL string that represents the Hcsm's
//   template name.
//
//////////////////////////////////////////////////////////////////////////////
inline const string& CHcsmStorage::GetName() const
{

	return m_name;

}
