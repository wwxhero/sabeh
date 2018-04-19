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
#include <typeinfo.h>
inline const char* CHcsmCommunicate::MyName()
{

	// return the class' name
	return typeid( *this ).name();

}

