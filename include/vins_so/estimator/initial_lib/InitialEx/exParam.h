#ifndef EXPARAM_H
#define EXPARAM_H

#include "vins_so/type/transform.h"

class exParam
{
    public:
    exParam( )
    : m_exParamSet( false )
    {
    }
    exParam( const Tf& tf_ic_in )
    : m_Tf_ic( tf_ic_in )
    , m_exParamSet( true )
    {
    }
    exParam( const exParam& ex_ic_in )
    : m_Tf_ic( ex_ic_in.m_Tf_ic )
    , m_exParamSet( true )
    {
    }
    exParam( const Eigen::Matrix3d& Ric, Eigen::Vector3d& Tic )
    : m_Tf_ic( Ric, Tic )
    , m_exParamSet( true )
    {
    }
    exParam& operator=( const Tf& other )
    {
        m_Tf_ic.R    = other.R;
        m_Tf_ic.T    = other.T;
        m_exParamSet = true;
        return *this;
    }
    Tf operator( )( void ) const { return m_Tf_ic; }
    void set( const exParam& ex_ic_in )
    {
        m_Tf_ic      = ex_ic_in.m_Tf_ic;
        m_exParamSet = ex_ic_in.m_exParamSet;
    }
    void set( const Tf& tf_ic_in )
    {
        m_Tf_ic      = tf_ic_in;
        m_exParamSet = true;
    }
    void set( const Eigen::Matrix3d R, const Eigen::Vector3d T )
    {
        m_Tf_ic.R    = R;
        m_Tf_ic.T    = T;
        m_exParamSet = true;
    }
    bool isSet( ) const { return m_exParamSet; }

    public:
    Tf m_Tf_ic;
    bool m_exParamSet;
};

#endif // EXPARAM_H
