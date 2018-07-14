#ifndef SLIDEWINDOWVELBIAS_H
#define SLIDEWINDOWVELBIAS_H

#include "slidewindowbase.h"
#include "vins_so/estimator/factor/integration_base.h"

namespace slidewindow
{

class SlideWindowIMU : public SlideWindowBase
{
    public:
    SlideWindowIMU( int window_size );

    void clearWindow( );

    void windowToDouble( double Bias[][6] );
    void setBiasAccIndex( int index, Vector3d bias ) { m_Bas[index] = bias; }
    void setBiasGyroIndex( int index, Vector3d bias ) { m_Bgs[index] = bias; }
    void slidewindow( );
    void addImuInfo( double dt, int frame_count, const Vector3d& acc, const Vector3d& gyr );
    void predictWindow( Tf& _tf, Vector3d& _vel, double dt, int frame_count, const Vector3d& acc, const Vector3d& gyr );
    void predictLast( ) {}
    void setG( const Vector3d& G );
    bool checkObservibility( );
    SlideWindowIMU& operator=( const SlideWindowIMU& other );
    void setPreIntegrationsIndex( ImuPreintPtr preIntegration, int index );

    std::vector< ImuPreintPtr > m_preIntegrations;
    std::vector< Eigen::Vector3d > m_Bas;
    std::vector< Eigen::Vector3d > m_Bgs;
    Vector3d m_accUn;
    Vector3d m_gyrUn;
    Vector3d m_accLast;
    Vector3d m_gyrLast;
    Vector3d m_G;
    bool first_imu;
};

typedef boost::shared_ptr< SlideWindowIMU > SlideWindowIMUPtr;
}
#endif // SLIDEWINDOWVELBIAS_H
