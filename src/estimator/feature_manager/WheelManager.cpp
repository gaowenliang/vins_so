#include "vins_so/estimator/feature_manager/WheelManager.h"

WheelManager::~WheelManager( ) { wheels.clear( ); }

Eigen::Vector3d
WheelManager::getOmega( const int frame )
{
    return wheels[frame].m_omegaB;
}

Eigen::Vector4d
WheelManager::getWheelVel( const int frame )
{
    return Eigen::Vector4d( wheels[frame].wheelPerFrame[0].m_vel,
                            wheels[frame].wheelPerFrame[1].m_vel,
                            wheels[frame].wheelPerFrame[2].m_vel,
                            wheels[frame].wheelPerFrame[3].m_vel );
}

bool
WheelManager::addNewMeasurement( const vector< double > vels, const Eigen::Vector3d gyrs )
{
    VelPerFrame vel( vels, gyrs );
    if ( wheels.size( ) < ( WINDOW_SIZE + 1 ) )
        wheels.push_back( vel );
    else
        wheels[WINDOW_SIZE] = vel;
}

void
WheelManager::clearState( )
{
    wheels.clear( );
}

void
WheelManager::removeBack( )
{
    for ( int i = 0; i < WINDOW_SIZE; i++ )
    {
        wheels[i] = wheels[i + 1];
    }
}

void
WheelManager::removeFront( )
{
    wheels[WINDOW_SIZE - 1] = wheels[WINDOW_SIZE];
}
