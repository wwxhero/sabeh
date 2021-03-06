/*****************************************************************************
 *
 *  (C) Copyright 1998 by National Advanced Driving Simulator and
 *  Simulation Center, the University of Iowa and The University
 *  of Iowa. All rights reserved.
 *
 *  This file has been generated by the hcsm code generator.
 *  ### DO NOT EDIT DIRECTLY ###
 *
 */


#include "genstorage.h"
#include <genericinclude.h>

//////////////////////////////////////////////////////////////
// CInputParCAdoInfoPtr
//////////////////////////////////////////////////////////////

CInputParCAdoInfoPtr::CInputParCAdoInfoPtr( string name ):
    CInputParameter( name )
{

}

CInputParCAdoInfoPtr::CInputParCAdoInfoPtr( const CInputParCAdoInfoPtr& objToCopy ):
    CInputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CInputParCAdoInfoPtr& CInputParCAdoInfoPtr::operator=(
            const CInputParCAdoInfoPtr& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

CInputParCAdoInfoPtr::~CInputParCAdoInfoPtr()
{

}

CAdoInfoPtr CInputParCAdoInfoPtr::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void CInputParCAdoInfoPtr::SetValue( CAdoInfoPtr value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

//////////////////////////////////////////////////////////////
// CInputPardouble
//////////////////////////////////////////////////////////////

CInputPardouble::CInputPardouble( string name ):
    CInputParameter( name )
{

}

CInputPardouble::CInputPardouble( const CInputPardouble& objToCopy ):
    CInputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CInputPardouble& CInputPardouble::operator=(
            const CInputPardouble& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

CInputPardouble::~CInputPardouble()
{

}

double CInputPardouble::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void CInputPardouble::SetValue( double value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

//////////////////////////////////////////////////////////////
// CInputParCLcCondsPtr
//////////////////////////////////////////////////////////////

CInputParCLcCondsPtr::CInputParCLcCondsPtr( string name ):
    CInputParameter( name )
{

}

CInputParCLcCondsPtr::CInputParCLcCondsPtr( const CInputParCLcCondsPtr& objToCopy ):
    CInputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CInputParCLcCondsPtr& CInputParCLcCondsPtr::operator=(
            const CInputParCLcCondsPtr& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

CInputParCLcCondsPtr::~CInputParCLcCondsPtr()
{

}

CLcCondsPtr CInputParCLcCondsPtr::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void CInputParCLcCondsPtr::SetValue( CLcCondsPtr value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

//////////////////////////////////////////////////////////////
// COutputParCPoint3D
//////////////////////////////////////////////////////////////

COutputParCPoint3D::COutputParCPoint3D( string name ):
    COutputParameter( name )
{

}

COutputParCPoint3D::COutputParCPoint3D( const COutputParCPoint3D& objToCopy ):
    COutputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

COutputParCPoint3D& COutputParCPoint3D::operator=(
            const COutputParCPoint3D& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

COutputParCPoint3D::~COutputParCPoint3D()
{

}

CPoint3D COutputParCPoint3D::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void COutputParCPoint3D::SetValue( CPoint3D value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

//////////////////////////////////////////////////////////////
// COutputPardouble
//////////////////////////////////////////////////////////////

COutputPardouble::COutputPardouble( string name ):
    COutputParameter( name )
{

}

COutputPardouble::COutputPardouble( const COutputPardouble& objToCopy ):
    COutputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

COutputPardouble& COutputPardouble::operator=(
            const COutputPardouble& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

COutputPardouble::~COutputPardouble()
{

}

double COutputPardouble::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void COutputPardouble::SetValue( double value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

//////////////////////////////////////////////////////////////
// COutputParstring
//////////////////////////////////////////////////////////////

COutputParstring::COutputParstring( string name ):
    COutputParameter( name )
{

}

COutputParstring::COutputParstring( const COutputParstring& objToCopy ):
    COutputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

COutputParstring& COutputParstring::operator=(
            const COutputParstring& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

COutputParstring::~COutputParstring()
{

}

string COutputParstring::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void COutputParstring::SetValue( string value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

//////////////////////////////////////////////////////////////
// COutputParbool
//////////////////////////////////////////////////////////////

COutputParbool::COutputParbool( string name ):
    COutputParameter( name )
{

}

COutputParbool::COutputParbool( const COutputParbool& objToCopy ):
    COutputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

COutputParbool& COutputParbool::operator=(
            const COutputParbool& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

COutputParbool::~COutputParbool()
{

}

bool COutputParbool::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void COutputParbool::SetValue( bool value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

//////////////////////////////////////////////////////////////
// COutputParint
//////////////////////////////////////////////////////////////

COutputParint::COutputParint( string name ):
    COutputParameter( name )
{

}

COutputParint::COutputParint( const COutputParint& objToCopy ):
    COutputParameter( objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

COutputParint& COutputParint::operator=(
            const COutputParint& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_value = objToCopy.m_value;

    }

    return *this;

}

COutputParint::~COutputParint()
{

}

int COutputParint::GetValue()
{

    // check to see if the input parameter has a value
    if ( !m_hasValue ) {

        cout << MyName() << "::GetValue:  input parameter has no value!";
        cout << endl;

    }

    return m_value;

}

void COutputParint::SetValue( int value )
{

    // set the input parameter's value
    m_value = value;
    m_hasValue = true;

}

