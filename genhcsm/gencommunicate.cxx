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


#include "gencommunicate.h"
#include <genericinclude.h>
#include <hcsmcollection.h>

//////////////////////////////////////////////////////////////
// CDialint
//////////////////////////////////////////////////////////////

CDialint::CDialint( CHcsmCollection* pRootCollection, string name ):
    CDial( pRootCollection, name )
{

}

CDialint::CDialint( const CDialint& objToCopy ):
    CDial( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CDialint& CDialint::operator=(
            const CDialint& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CDialint::~CDialint()
{

}

int CDialint::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CDialint::GetValueStr()
{
    string strValue = "";
    int curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    sprintf(&*strValue.begin(), "%d", curValue);
    return strValue;

}

void CDialint::SetValue( int value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CDial::SetValue();
}

void CDialint::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = atoi(value.c_str());
    }
    
    CDial::SetValue();
}

//////////////////////////////////////////////////////////////
// CDialdouble
//////////////////////////////////////////////////////////////

CDialdouble::CDialdouble( CHcsmCollection* pRootCollection, string name ):
    CDial( pRootCollection, name )
{

}

CDialdouble::CDialdouble( const CDialdouble& objToCopy ):
    CDial( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CDialdouble& CDialdouble::operator=(
            const CDialdouble& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CDialdouble::~CDialdouble()
{

}

double CDialdouble::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CDialdouble::GetValueStr()
{
    string strValue = "";
    double curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    sprintf(&*strValue.begin(), "%lf", curValue);
    return strValue;

}

void CDialdouble::SetValue( double value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CDial::SetValue();
}

void CDialdouble::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = atof(value.c_str());
    }
    
    CDial::SetValue();
}

//////////////////////////////////////////////////////////////
// CDialstring
//////////////////////////////////////////////////////////////

CDialstring::CDialstring( CHcsmCollection* pRootCollection, string name ):
    CDial( pRootCollection, name )
{

}

CDialstring::CDialstring( const CDialstring& objToCopy ):
    CDial( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CDialstring& CDialstring::operator=(
            const CDialstring& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CDialstring::~CDialstring()
{

}

string CDialstring::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CDialstring::GetValueStr()
{
    string strValue = "";
    string curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    strValue = curValue;
    return strValue;

}

void CDialstring::SetValue( string value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CDial::SetValue();
}

void CDialstring::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CDial::SetValue();
}

//////////////////////////////////////////////////////////////
// CDialfloat
//////////////////////////////////////////////////////////////

CDialfloat::CDialfloat( CHcsmCollection* pRootCollection, string name ):
    CDial( pRootCollection, name )
{

}

CDialfloat::CDialfloat( const CDialfloat& objToCopy ):
    CDial( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CDialfloat& CDialfloat::operator=(
            const CDialfloat& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CDialfloat::~CDialfloat()
{

}

float CDialfloat::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CDialfloat::GetValueStr()
{
    string strValue = "";
    float curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    sprintf(&*strValue.begin(), "%f", curValue);
    return strValue;

}

void CDialfloat::SetValue( float value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CDial::SetValue();
}

void CDialfloat::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = atof(value.c_str());
    }
    
    CDial::SetValue();
}

//////////////////////////////////////////////////////////////
// CDialbool
//////////////////////////////////////////////////////////////

CDialbool::CDialbool( CHcsmCollection* pRootCollection, string name ):
    CDial( pRootCollection, name )
{

}

CDialbool::CDialbool( const CDialbool& objToCopy ):
    CDial( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CDialbool& CDialbool::operator=(
            const CDialbool& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CDialbool::~CDialbool()
{

}

bool CDialbool::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CDialbool::GetValueStr()
{
    string strValue = "";
    bool curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    strValue = (curValue? "true" : "false");
    return strValue;

}

void CDialbool::SetValue( bool value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CDial::SetValue();
}

void CDialbool::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = (value == "true");
    }
    
    CDial::SetValue();
}

//////////////////////////////////////////////////////////////
// CMonitorCRoadPos
//////////////////////////////////////////////////////////////

CMonitorCRoadPos::CMonitorCRoadPos( CHcsmCollection* pRootCollection, string name ):
    CMonitor( pRootCollection, name )
{

}

CMonitorCRoadPos::CMonitorCRoadPos( const CMonitorCRoadPos& objToCopy ):
    CMonitor( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CMonitorCRoadPos& CMonitorCRoadPos::operator=(
            const CMonitorCRoadPos& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CMonitorCRoadPos::~CMonitorCRoadPos()
{

}

CRoadPos CMonitorCRoadPos::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CMonitorCRoadPos::GetValueStr()
{
    string strValue = "";
    CRoadPos curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    strValue = curValue.GetString();
    return strValue;

}

void CMonitorCRoadPos::SetValue( CRoadPos value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CMonitor::SetValue();
}

void CMonitorCRoadPos::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA.SetString(value);
    }
    
    CMonitor::SetValue();
}

//////////////////////////////////////////////////////////////
// CMonitorCCrdr
//////////////////////////////////////////////////////////////

CMonitorCCrdr::CMonitorCCrdr( CHcsmCollection* pRootCollection, string name ):
    CMonitor( pRootCollection, name )
{

}

CMonitorCCrdr::CMonitorCCrdr( const CMonitorCCrdr& objToCopy ):
    CMonitor( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CMonitorCCrdr& CMonitorCCrdr::operator=(
            const CMonitorCCrdr& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CMonitorCCrdr::~CMonitorCCrdr()
{

}

CCrdr CMonitorCCrdr::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CMonitorCCrdr::GetValueStr()
{
    string strValue = "";
    CCrdr curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    strValue = curValue.GetString();
    return strValue;

}

void CMonitorCCrdr::SetValue( CCrdr value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CMonitor::SetValue();
}

void CMonitorCCrdr::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA.SetString(value);
    }
    
    CMonitor::SetValue();
}

//////////////////////////////////////////////////////////////
// CMonitorint
//////////////////////////////////////////////////////////////

CMonitorint::CMonitorint( CHcsmCollection* pRootCollection, string name ):
    CMonitor( pRootCollection, name )
{

}

CMonitorint::CMonitorint( const CMonitorint& objToCopy ):
    CMonitor( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CMonitorint& CMonitorint::operator=(
            const CMonitorint& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CMonitorint::~CMonitorint()
{

}

int CMonitorint::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CMonitorint::GetValueStr()
{
    string strValue = "";
    int curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    sprintf(&*strValue.begin(), "%d", curValue);
    return strValue;

}

void CMonitorint::SetValue( int value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CMonitor::SetValue();
}

void CMonitorint::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = atoi(value.c_str());
    }
    
    CMonitor::SetValue();
}

//////////////////////////////////////////////////////////////
// CMonitorbool
//////////////////////////////////////////////////////////////

CMonitorbool::CMonitorbool( CHcsmCollection* pRootCollection, string name ):
    CMonitor( pRootCollection, name )
{

}

CMonitorbool::CMonitorbool( const CMonitorbool& objToCopy ):
    CMonitor( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CMonitorbool& CMonitorbool::operator=(
            const CMonitorbool& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CMonitorbool::~CMonitorbool()
{

}

bool CMonitorbool::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CMonitorbool::GetValueStr()
{
    string strValue = "";
    bool curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    strValue = (curValue? "true" : "false");
    return strValue;

}

void CMonitorbool::SetValue( bool value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CMonitor::SetValue();
}

void CMonitorbool::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = (value == "true");
    }
    
    CMonitor::SetValue();
}

//////////////////////////////////////////////////////////////
// CMonitorstring
//////////////////////////////////////////////////////////////

CMonitorstring::CMonitorstring( CHcsmCollection* pRootCollection, string name ):
    CMonitor( pRootCollection, name )
{

}

CMonitorstring::CMonitorstring( const CMonitorstring& objToCopy ):
    CMonitor( objToCopy.m_pRootCollection, objToCopy.m_name )
{

    // call the assignment operator
    *this = objToCopy;

}

CMonitorstring& CMonitorstring::operator=(
            const CMonitorstring& objToCopy
            )
{

    // check to see if the object passed in is really me
    if ( this != &objToCopy ) {

        // make a deep copy
        m_valueA = objToCopy.m_valueA;
        m_valueB = objToCopy.m_valueB;

    }

    return *this;

}

CMonitorstring::~CMonitorstring()
{

}

string CMonitorstring::GetValue()
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        return m_valueA;
    }
}

string CMonitorstring::GetValueStr()
{
    string strValue = "";
    string curValue;


    bool writtenThisFrame = m_setFrame == GetFrame();
    if( writtenThisFrame )
    {
        // make sure that dial has a value
        if( !m_hasValueB )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueB;
    }
    else
    {
        // make sure that dial has a value
        if( !m_hasValueA )
        {
            cerr << MyName() << "::GetValue: dial has no value" << endl;
        }

        curValue = m_valueA;
    }
    
    strValue = curValue;
    return strValue;

}

void CMonitorstring::SetValue( string value )
{
    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CMonitor::SetValue();
}

void CMonitorstring::SetValueStr( const string& value )
{

    bool writtenThisFrame = m_setFrame == GetFrame();
    if( !writtenThisFrame )
    {
        m_valueB = m_valueA;
        m_valueA = value;
    }
    
    CMonitor::SetValue();
}
