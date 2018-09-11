#ifndef WHEELMANAGER_H
#define WHEELMANAGER_H

#include "../vins_parameters.h"
#include <list>
#include <vector>

using namespace std;

class VelPerWheel
{
    public:
    VelPerWheel( ) {}
    VelPerWheel( const int id )
    : m_wheelId( id )
    {
    }
    VelPerWheel( const double r )
    : m_radius( r )
    {
    }
    VelPerWheel( const int id, const double vel )
    : m_wheelId( id )
    , m_vel( vel )
    {
    }

    void setOmega( const double omega )
    {
        m_omega = omega;
        m_vel   = m_omega * m_radius;
    }

    public:
    int m_wheelId;
    double m_radius;
    double m_omega;
    double m_vel;
};

class VelPerFrame
{
    public:
    VelPerFrame( const Eigen ::Vector4d vels, const Eigen ::Vector3d omegaB )
    {
        m_omegaB = omegaB;

        m_numOfWheel = 4;
        for ( uint i = 0; i < 4; ++i )
        {
            VelPerWheel vel( i, vels( i ) );
            wheelPerFrame.push_back( vel );
        }
    }
    VelPerFrame( const vector< double > vels, const Eigen ::Vector3d omegaB )
    {
        m_omegaB = omegaB;

        m_numOfWheel = vels.size( );
        for ( uint i = 0; i < vels.size( ); ++i )
        {
            VelPerWheel vel( i, vels[i] );
            wheelPerFrame.push_back( vel );
        }
    }

    public:
    int m_numOfWheel;
    std::vector< VelPerWheel > wheelPerFrame;
    Eigen ::Vector3d m_omegaB;
};

class WheelManager
{
    public:
    WheelManager( ) {}
    ~WheelManager( );

    Eigen::Vector3d getOmega( const int frame );
    Eigen::Vector4d getWheelVel( const int frame );
    bool addNewMeasurement( const vector< double > vels, //
                            const Eigen::Vector3d gyrs );
    void clearState( );
    void removeBack( );
    void removeFront( );

    public:
    int m_startFrame;
    vector< VelPerFrame > wheels;
};

#endif // WHEELMANAGER_H
