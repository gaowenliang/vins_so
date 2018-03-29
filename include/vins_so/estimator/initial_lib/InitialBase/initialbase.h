#ifndef INITIALBASE_H
#define INITIALBASE_H

#include "vins_so/estimator/factor/integration_base.h"
#include "vins_so/estimator/feature_manager/feature_manager.h"
#include "vins_so/estimator/initial_lib/InitialEx/exParam.h"
#include "vins_so/estimator/slideWindow/slidewindowimu.h"
#include "vins_so/estimator/slideWindow/slidewindowpose.h"
#include "vins_so/estimator/slideWindow/slidewindowposevel.h"
#include "vins_so/type/transform.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>

namespace InitVio
{

class ImageImuFrame
{
    public:
    ImageImuFrame( ) {}
    ImageImuFrame( const FeatureData& _points, const ImuPreintPtr preIntegrationTmp, double _t )
    : points{ _points }
    , t{ _t }
    , m_isKeyFrame{ false }
    {
        preIntegration = preIntegrationTmp;
    }
    ///
    /// \brief points
    ///      feature_id                  camera_id    capture vector
    FeatureData points;
    ImuPreintPtr preIntegration;

    Tf tf_wb;
    //    Eigen::Matrix3d R_wb;
    //    Eigen::Vector3d T_wb;
    double t; // time stamp

    bool m_isKeyFrame;
};

enum InitType
{
    MONO = 0,
    STEREO_PNP,
    STEREO_NONLINEAR,
    MIX
};

class Initial
{
    public:
    Initial( int window_size = 10, int camera_size = 1 );
    ~Initial( );
    Initial( InitType _type )
    : m_type( _type )
    {
    }

    virtual bool initial( )                                   = 0;
    virtual void slideWindow( const std::vector< Tf > tf_ic ) = 0;
    virtual void pushImage( double time, int _frame_count, const FeatureData& _points ) = 0;
    virtual bool getEx( std::vector< Tf >& exparam ) const = 0;
    virtual void copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new, //
                                   slidewindow::SlideWindowIMUPtr imu_new,
                                   Vector3d& _g )
    = 0;

    void setEx( const std::vector< Eigen::Matrix3d >& Ric, const std::vector< Eigen::Vector3d >& Tic );
    void setEx( const std::vector< exParam >& ex_ic_in );
    void setEx( const exParam& ex_ic_in, int camera_index );
    bool checkEx( );

    void setAlignCameraIndex( int cameraIndex );
    void clear( );
    void pushImu( const double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr );
    void resetImu( const Eigen::Vector3d& _acc_0,
                   const Eigen::Vector3d& _gyr_0,
                   const Eigen::Vector3d& _linearized_ba,
                   const Eigen::Vector3d& _linearized_bg );

    bool Done( ) const { return m_done; }

    public:
    InitType m_type;
    int m_cameraIndex; // align image and camera
    int m_windowSize;

    ///
    /// \brief imageFrameAll
    /// double: time stamp
    /// ImageFrame: image frames
    std::map< double, ImageImuFrame > m_imageFrameAll;
    ImuPreintPtr m_tmpPreIntegration;

    std::vector< exParam > m_exParams;
    bool m_exParamSet;

    slidewindow::SlideWindowPosePtr m_window;
    std::vector< Vector3d > m_Bgs;
    VectorXd m_Xtmp;
    Vector3d m_g;
    bool m_done;
};

typedef boost::shared_ptr< Initial > InitialPtr;
}
#endif // INITIALBASE_H
