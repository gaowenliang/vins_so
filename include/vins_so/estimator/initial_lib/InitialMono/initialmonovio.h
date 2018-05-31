#ifndef INITIALMONOVIO_H
#define INITIALMONOVIO_H

#include "initial_sfm.h"
#include "solve_5pts.h"
#include "vins_so/estimator/initial_lib/InitialBase/initialbase.h"
#include "vins_so/estimator/initial_lib/InitialEx/InitialExRotationCamImu.h"
#include "vins_so/estimator/initial_lib/VisualImuAlignment/VisualImuAlignmentVelScale.h"
#include "vins_so/estimator/slideWindow/slidewindowimu.h"
#include "vins_so/utility/tic_toc.h"
#include <code_utils/cv_utils/pnp/pnp.h>

namespace InitVio
{

class InitialMonoVio : public Initial
{
    public:
    InitialMonoVio( int window_size = 10, int camera_size = 1 );
    ~InitialMonoVio( ) { m_RicInitial.reset( ); }

    void pushImage( double time, int _frame_count, const FeatureData& _points );
    bool initial( );
    bool getEx( std::vector< Tf >& exparam ) const {}
    void slideWindow( const std::vector< Tf > tf_ic );
    void copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new,
                           slidewindow::SlideWindowIMUPtr imu_new,
                           Vector3d& _g );

    private:
    void calcEx( int _frame_count );
    bool initialStructure( );
    bool visualInitialAlign( int align_camera_index );
    FeatureData getFeatureCamIndex( int cam_index, const FeatureData& _points, double z_the );
    bool relativePose( Matrix3d& relative_R, Vector3d& relative_T, int& l, bool& is_solved );

    private:
    MotionEstimator m_estimator;
    initialEx::InitialExRotationCamImuPtr m_RicInitial;
};

typedef boost::shared_ptr< InitialMonoVio > InitialMonoVioPtr;
}

#endif // INITIALMONOVIO_H
