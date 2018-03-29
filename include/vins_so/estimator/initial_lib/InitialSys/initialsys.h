#ifndef INITIALSYS_H
#define INITIALSYS_H

#include "vins_so/estimator/initial_lib/InitialBase/initialgen.h"
#include "vins_so/estimator/initial_lib/InitialEx/InitialExRotationCamCam.h"
#include "vins_so/estimator/initial_lib/InitialEx/InitialExRotationCamImu.h"
#include "vins_so/estimator/initial_lib/InitialEx/exParam.h"
#include <iostream>
#include <vector>

namespace InitVio
{

class InitialSys
{
    public:
    InitialSys( int _num_camera, int _num_stereo )
    : done( false )
    , num_stereo( _num_stereo )
    , num_camera( _num_camera )
    , m_exParamSet( false )
    {
        m_Ex_ics.resize( _num_camera );

        initClear( );
        initReset( );
    }
    InitialSys( int _num_camera, int _num_stereo, std::vector< Tf > tf_ics )
    : done( false )
    , num_stereo( _num_stereo )
    , num_camera( _num_camera )
    , m_exParamSet( false )
    {
        if ( tf_ics.size( ) != _num_camera )
        {
            std::cout
            << "[ERROR] error eith camera extrinsic parameters set in InitialSys.\n";
        }
        m_Ex_ics.resize( _num_camera );

        setEx( tf_ics );
        initClear( );
        initReset( );
    }

    bool initial( );
    void slideWindow( const std::vector< Tf > tf_ic );
    void pushImage( double time, int _frame_count, const FeatureData& _points );

    void setEx( const std::vector< Tf >& tf_ics )
    {
        for ( int camera_index = 0; camera_index < num_camera; ++camera_index )
            m_Ex_ics.push_back( exParam( tf_ics[camera_index] ) );

        checkEx( );
    }
    void setEx( const std::vector< exParam >& m_Ex_ics );
    void pushImu( const double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr );
    void resetImu( const Eigen::Vector3d& _acc_0,
                   const Eigen::Vector3d& _gyr_0,
                   const Eigen::Vector3d& _linearized_ba,
                   const Eigen::Vector3d& _linearized_bg );

    void copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new, //
                           slidewindow::SlideWindowIMUPtr imu_new,
                           Vector3d& _g );
    void initClear( );
    void initReset( );
    bool Done( ) const;

    private:
    bool checkEx( )
    {
        if ( m_exParamSet )
        {
            return true;
        }
        else
        {
            bool isSet = true;
            for ( auto& ex : m_Ex_ics )
            {
                isSet &= ex.isSet( );
            }
            m_exParamSet = isSet;
            return m_exParamSet;
        }
    }

    private:
    bool done;
    int num_stereo;
    int num_camera;

    std::vector< exParam > m_Ex_ics;
    bool m_exParamSet;

    private:
    std::vector< InitVio::InitialPtr > vioInitStereo;
    std::vector< InitVio::InitialPtr > vioInitMono;
};

typedef boost::shared_ptr< InitialSys > InitialSysPtr;
}

#endif // INITIALSYS_H
