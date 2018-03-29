#pragma once

#include "../utility/tic_toc.h"
#include "../utility/utility.h"

#include "factor/ProjectionFactorMultiCam.h"
#include "factor/ProjectionFactorSingleCam.h"
#include "factor/imu_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/monoprojectionfactor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/stereotimeprojectionfactor.h"

#include "feature_manager/feature_manager.h"

#include "initial_lib/InitialEx/exParam.h"
#include "initial_lib/InitialSys/initialsys.h"

#include "slideWindow/slidewindowimu.h"
#include "slideWindow/slidewindowposevel.h"

#include "vins_parameters.h"

#include <ceres/ceres.h>
#include <code_utils/cv_utils/pnp/pnp.h>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <unordered_map>

class Estimator
{
    public:
    Estimator( );
    void initEstimator( int window_size, int num_of_camera );
    void setParameter( );

    // interface
    void processIMU( double t,
                     const Vector3d& linear_acceleration, //
                     const Vector3d& angular_velocity );
    void processImage( const FeatureData& image, //
                       const std_msgs::Header& header );

    // internal
    void clearState( );
    bool initialStructure( );
    bool visualInitialAlign( int align_camera_index );
    bool relativePose( vector< Matrix3d >& relative_R,
                       vector< Vector3d >& relative_T,
                       int& l,
                       vector< bool >& is_solved );
    void slideWindow( );
    void solveOdometry( );
    void optimization( );
    void vector2double( );
    void double2vector( );
    bool failureDetection( );

    enum SolverFlag
    {
        INITIAL = 0,
        NONLINEAR
    };

    SolverFlag solver_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    std::vector< Tf > tf_ics;
    std::vector< Tf > tf_rls;
    std::vector< Tf > tf_lrs;
    std::vector< double > baselines;

    Tf back_tf, last_tf0;

    Tf last_Pose;
    Eigen::Vector3d last_vel;

    Vector3d acc_0, gyr_0;

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    MotionEstimator m_estimator;
    initialEx::InitialExRotationCamImuPtr initial_ex_rotation;

    bool is_valid, is_key;
    bool failure_occur;

    vector< Vector3d > point_cloud;
    vector< Vector3d > margin_cloud;
    vector< Vector3d > key_poses;
    double initial_timestamp;

    double paraPose[WINDOW_SIZE + 1][SIZE_POSE];
    double paraSpeed[WINDOW_SIZE + 1][SIZE_SPEED];
    double paraBias[WINDOW_SIZE + 1][SIZE_BIAS];
    double paraFeature[NUM_OF_F][SIZE_FEATURE];
    double paraExPose[MAX_NUM_OF_CAMS][SIZE_POSE];
    double paraRetrivePose[SIZE_POSE];

    std::vector< int > para_Feature_CameraID;

    bool relocalize;
    Vector3d relocalize_t;
    Matrix3d relocalize_r;

    MarginalizationInfoPtr last_marginalization_info;
    vector< double* > last_marginalization_parameter_blocks;

    slidewindow::SlideWindowIMUPtr pImu;
    slidewindow::SlideWindowPoseVelPtr pWindow;

    InitVio::InitialSysPtr vioInitialSys;
};
