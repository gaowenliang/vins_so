#ifndef INITIALSTEREOVIOPNP_H
#define INITIALSTEREOVIOPNP_H

#include "../InitialEx/InitialExRotationCamCam.h"
#include "vins_so/estimator/initial_lib/InitialBase/initialbase.h"
#include "vins_so/estimator/initial_lib/InitialEx/InitialExRotationCamCam.h"
#include "vins_so/estimator/initial_lib/InitialEx/InitialExRotationCamImu.h"
#include "vins_so/estimator/slideWindow/slidewindowimu.h"
#include <code_utils/cv_utils/pnp/pnp.h>

namespace InitVio
{

class InitialStereoVioPnP : public Initial
{
    public:
    InitialStereoVioPnP( int window_size = 10, int camera_size = 2 );
    ~InitialStereoVioPnP( ) {}

    bool initial( );
    void pushImage( double time, int _frame_count, const FeatureData& _points );
    bool getEx( std::vector< Tf >& exparam ) const { return false; }
    void slideWindow( const std::vector< Tf > tf_ic );
    void copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new, //
                           slidewindow::SlideWindowIMUPtr imu_new,
                           Vector3d& _g );

    private:
    Tf Tf_icl( );
    Tf Tf_icr( );
    void calcEx( int _frame_count );
    void calcExCamCam( );
    void initPnP( int frame );
    void stereoTriangulate( int frame );

    Tf m_Tf_rl, m_Tf_lr;
    double baseline;
    bool m_isExCalced;

    initialEx::InitialExRotationCamImuPtr m_RicInitial_l;
    initialEx::InitialExRotationCamImuPtr m_RicInitial_r;
    initialEx::InitialExRotationCamCamPtr m_RccInitial;
};

typedef boost::shared_ptr< InitialStereoVioPnP > InitialStereoVioPnPPtr;
}

#endif // INITIALSTEREOVIOPNP_H
