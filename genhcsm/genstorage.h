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


#ifndef __genstorage_h_INCLUDED_
#define __genstorage_h_INCLUDED_

#include "genhcsmglobal.h"
#include "inputparameter.h"
#include "outputparameter.h"
#include <string>
using namespace std;

//////////////////////////////////////////////////////////////
// CInputParCAdoInfoPtr
//////////////////////////////////////////////////////////////

class CInputParCAdoInfoPtr : public CInputParameter
{
public:
    CInputParCAdoInfoPtr( string );
    CInputParCAdoInfoPtr( const CInputParCAdoInfoPtr& );
    CInputParCAdoInfoPtr& operator=( const CInputParCAdoInfoPtr& );
    virtual ~CInputParCAdoInfoPtr(); 
    CAdoInfoPtr GetValue();
    void SetValue( CAdoInfoPtr );

private:
    CAdoInfoPtr m_value;
};

//////////////////////////////////////////////////////////////
// CInputPardouble
//////////////////////////////////////////////////////////////

class CInputPardouble : public CInputParameter
{
public:
    CInputPardouble( string );
    CInputPardouble( const CInputPardouble& );
    CInputPardouble& operator=( const CInputPardouble& );
    virtual ~CInputPardouble(); 
    double GetValue();
    void SetValue( double );

private:
    double m_value;
};

//////////////////////////////////////////////////////////////
// CInputParCLcCondsPtr
//////////////////////////////////////////////////////////////

class CInputParCLcCondsPtr : public CInputParameter
{
public:
    CInputParCLcCondsPtr( string );
    CInputParCLcCondsPtr( const CInputParCLcCondsPtr& );
    CInputParCLcCondsPtr& operator=( const CInputParCLcCondsPtr& );
    virtual ~CInputParCLcCondsPtr(); 
    CLcCondsPtr GetValue();
    void SetValue( CLcCondsPtr );

private:
    CLcCondsPtr m_value;
};

//////////////////////////////////////////////////////////////
// COutputParCPoint3D
//////////////////////////////////////////////////////////////

class COutputParCPoint3D : public COutputParameter
{
public:
    COutputParCPoint3D( string );
    COutputParCPoint3D( const COutputParCPoint3D& );
    COutputParCPoint3D& operator=( const COutputParCPoint3D& );
    virtual ~COutputParCPoint3D(); 
    CPoint3D GetValue();
    void SetValue( CPoint3D );

private:
    CPoint3D m_value;
};

//////////////////////////////////////////////////////////////
// COutputPardouble
//////////////////////////////////////////////////////////////

class COutputPardouble : public COutputParameter
{
public:
    COutputPardouble( string );
    COutputPardouble( const COutputPardouble& );
    COutputPardouble& operator=( const COutputPardouble& );
    virtual ~COutputPardouble(); 
    double GetValue();
    void SetValue( double );

private:
    double m_value;
};

//////////////////////////////////////////////////////////////
// COutputParstring
//////////////////////////////////////////////////////////////

class COutputParstring : public COutputParameter
{
public:
    COutputParstring( string );
    COutputParstring( const COutputParstring& );
    COutputParstring& operator=( const COutputParstring& );
    virtual ~COutputParstring(); 
    string GetValue();
    void SetValue( string );

private:
    string m_value;
};

//////////////////////////////////////////////////////////////
// COutputParbool
//////////////////////////////////////////////////////////////

class COutputParbool : public COutputParameter
{
public:
    COutputParbool( string );
    COutputParbool( const COutputParbool& );
    COutputParbool& operator=( const COutputParbool& );
    virtual ~COutputParbool(); 
    bool GetValue();
    void SetValue( bool );

private:
    bool m_value;
};

//////////////////////////////////////////////////////////////
// COutputParint
//////////////////////////////////////////////////////////////

class COutputParint : public COutputParameter
{
public:
    COutputParint( string );
    COutputParint( const COutputParint& );
    COutputParint& operator=( const COutputParint& );
    virtual ~COutputParint(); 
    int GetValue();
    void SetValue( int );

private:
    int m_value;
};



#endif
