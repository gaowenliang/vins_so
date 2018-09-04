#ifndef INITIALSTEREOVIO_H
#define INITIALSTEREOVIO_H

#include "vins_so/estimator/initial_lib/InitialBase/initialbase.h"
#include "vins_so/estimator/slideWindow/slidewindowimu.h"

namespace InitVio
{

class InitialStereoVio : public Initial
{
    public:
    InitialStereoVio( int window_size = 10, int camera_size = 2 ) {}

    bool getEx( std::vector< Tf >& exparam ) const { return false; }
    void pushImage( double time, int _frame_count, const FeatureData& _points )
    {
        InitVio::ImageImuFrame image_imu_info( _points, m_tmpPreIntegration, time );

        m_imageFrameAll.insert( std::make_pair( time, image_imu_info ) );
    }
    void copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new, //
                           slidewindow::SlideWindowIMUPtr imu_new,
                           Vector3d& _g )
    {
    }
    void slideWindow( const std::vector< Tf > tf_ic ) {}
    bool initial( )
    {
        if ( checkEx( ) )
        {
        }
        return true;
    }
};
typedef boost::shared_ptr< InitialStereoVio > InitialStereoVioPtr;
}
#endif // INITIALSTEREOVIO_H
