#include "vins_so/estimator/initial_lib/InitialSys/initialsys.h"

bool
InitVio::InitialSys::initial( )
{
    if ( !done )
        for ( auto& initStereoTmp : vioInitStereo )
        {
            done = initStereoTmp->initial( );
            if ( done )
                break;
        }

    if ( !done )
        for ( auto& initMonoTmp : vioInitMono )
        {
            done = initMonoTmp->initial( );
            if ( done )
                break;
        }
}

void
InitVio::InitialSys::slideWindow( const std::vector< Tf > tf_ic )
{
    for ( auto& initStereoTmp : vioInitStereo )
        initStereoTmp->slideWindow( tf_ic );

    for ( auto& initMonoTmp : vioInitMono )
        initMonoTmp->slideWindow( tf_ic );
}

void
InitVio::InitialSys::pushImage( double time, int _frame_count, const FeatureData& _points )
{
    for ( auto& initStereoTmp : vioInitStereo )
        initStereoTmp->pushImage( time, _frame_count, _points );

    for ( auto& initMonoTmp : vioInitMono )
        initMonoTmp->pushImage( time, _frame_count, _points );
}

void
InitVio::InitialSys::setEx( const std::vector< exParam >& ex_ics )
{
    m_Ex_ics = ex_ics;

    for ( auto& initStereoTmp : vioInitStereo )
        initStereoTmp->setEx( ex_ics );

    for ( auto& initMonoTmp : vioInitMono )
        initMonoTmp->setEx( ex_ics );

    checkEx( );
}

void
InitVio::InitialSys::pushImu( const double dt, const Vector3d& acc, const Vector3d& gyr )
{
    for ( auto& initStereoTmp : vioInitStereo )
        initStereoTmp->pushImu( dt, acc, gyr );

    for ( int camera_index = 0; camera_index < num_camera; ++camera_index )
        vioInitMono[camera_index]->pushImu( dt, acc, gyr );
}

void
InitVio::InitialSys::resetImu( const Vector3d& _acc_0,
                               const Vector3d& _gyr_0,
                               const Vector3d& _linearized_ba,
                               const Vector3d& _linearized_bg )
{
    for ( auto& initStereoTmp : vioInitStereo )
        initStereoTmp->resetImu( _acc_0, _gyr_0, _linearized_ba, _linearized_bg );

    for ( auto& initMonoTmp : vioInitMono )
        initMonoTmp->resetImu( _acc_0, _gyr_0, _linearized_ba, _linearized_bg );
}

void
InitVio::InitialSys::copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new,
                                       slidewindow::SlideWindowIMUPtr imu_new,
                                       Vector3d& _g )
{
    if ( done )
    {
        for ( auto& initStereoTmp : vioInitStereo )
        {
            if ( initStereoTmp->Done( ) )
            {
                initStereoTmp->copyInitInfoBack( window_new, imu_new, _g );
                return;
            }
        }

        for ( auto& initMonoTmp : vioInitMono )
        {
            if ( initMonoTmp->Done( ) )
            {
                initMonoTmp->copyInitInfoBack( window_new, imu_new, _g );
                return;
            }
        }
    }
}

void
InitVio::InitialSys::initClear( )
{

    {
        if ( vioInitStereo.empty( ) )
        {
            vioInitStereo.resize( num_stereo );
        }
        if ( vioInitMono.empty( ) )
        {
            vioInitMono.resize( num_camera );
        }
    }

    {
        for ( auto& vioInit : vioInitStereo )
            vioInit.reset( );
        for ( auto& vioInit : vioInitMono )
            vioInit.reset( );
    }
}

void
InitVio::InitialSys::initReset( )
{

    if ( !done )
        for ( int camera_index = 0; camera_index < num_camera; ++camera_index )
        {
            if ( vioInitMono[camera_index] != nullptr )
                vioInitMono[camera_index].reset( );

            vioInitMono[camera_index] = InitialGen::instance( )->generateInit( InitVio::MONO );

            vioInitMono[camera_index]->clear( );
            vioInitMono[camera_index]->setAlignCameraIndex( camera_index );
        }

    for ( int camera_index = 0; camera_index < num_stereo; ++camera_index )
    {
        if ( vioInitStereo[camera_index] != nullptr )
            vioInitStereo[camera_index].reset( );

        vioInitStereo[camera_index] = InitialGen::instance( )->generateInit( InitVio::STEREO_PNP );
        vioInitStereo[camera_index]->clear( );
    }
}

bool
InitVio::InitialSys::Done( ) const
{
    return done;
}
