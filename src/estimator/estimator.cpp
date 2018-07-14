#include "vins_so/estimator/estimator.h"
#include <Eigen/StdVector>

//#define DUAL_FISHEYE 1
#define MARG 1

using namespace slidewindow;

Estimator::Estimator( ) {}

void
Estimator::initEstimator( int window_size, int num_of_camera )
{
    pWindow = SlideWindowPoseVelPtr( new SlideWindowPoseVel( window_size, //
                                                             num_of_camera ) );
    ROS_INFO( "init SlideWindowPoseVel " );

    pImu = SlideWindowIMUPtr( new SlideWindowIMU( window_size ) );
    ROS_INFO( "init SlideWindowIMU " );

    failure_occur = 0;
    clearState( );
}

void
Estimator::setParameter( )
{
    tf_ics.clear( );
    tf_ics.resize( NUM_OF_CAM );

    std::vector< exParam > ex_ics;
    for ( int i = 0; i < NUM_OF_CAM; i++ )
    {
        tf_ics[i].setRT( RIC[i], TIC[i] );
        ex_ics.push_back( exParam( tf_ics[i] ) );
    }

    if ( ESTIMATE_EXTRINSIC != 2 )
    {
        if ( NUM_OF_CAM >= 2 )
        {
            for ( int i = 0; i < NUM_OF_CAM - 1; i++ )
            {
                //  croppeed images is rectified
                Tf tf_rl = tf_ics[i + 1].inverse( ) + tf_ics[i];

#if DUAL_FISHEYE
                Matrix3d rrl;
                rrl << -1., 0., 0., 0., 1., 0., 0., 0., -1.;
                tf_rl.setR( rrl );
#endif

                Tf tf_lr        = tf_rl.inverse( );
                double baseline = tf_lr.norm( );

                std::cout << "baseline cam " << i + 1 << " " << i << " : " << baseline << std::endl;
                std::cout << " tf_rl cam " << i + 1 << " " << i << " : " << tf_rl << std::endl;

                tf_rls.push_back( tf_rl );
                tf_lrs.push_back( tf_lr );
                baselines.push_back( baseline );

                pWindow->m_featureManager.setBaseline( baseline );
                pWindow->m_featureManager.setTfrl( tf_rl );
            }
        }
        else
        {
            ROS_WARN_STREAM( "MONO, no tf_rl and baseline." );
        }

        vioInitialSys->setEx( ex_ics );
    }
}

void
Estimator::clearState( )
{
    pWindow->clearWindow( );

    pImu->clearWindow( );

    tf_ics.clear( );
    tf_ics.resize( NUM_OF_CAM );

    for ( int i = 0; i < NUM_OF_CAM; i++ )
    {
        tf_ics[i].setZero( );
    }

    if ( vioInitialSys == nullptr )
        vioInitialSys.reset( );
    vioInitialSys = InitVio::InitialSysPtr( new InitVio::InitialSys( NUM_OF_CAM, //
                                                                     NUM_OF_STEREO,
                                                                     tf_ics ) );

    sum_of_back       = 0;
    sum_of_front      = 0;
    frame_count       = 0;
    solver_flag       = INITIAL;
    initial_timestamp = 0;
    relocalize        = false;

    relocalize_t = Eigen::Vector3d( 0, 0, 0 );
    relocalize_r = Eigen::Matrix3d::Identity( );

    last_marginalization_info.reset( );
    last_marginalization_parameter_blocks.clear( );

    pWindow->m_featureManager.clearState( );
}

void
Estimator::processIMU( double dt, const Vector3d& linear_acceleration, const Vector3d& angular_velocity )
{

    if ( frame_count != 0 )
        if ( solver_flag == INITIAL )
        {
            vioInitialSys->pushImu( dt, linear_acceleration, angular_velocity );
        }

    if ( solver_flag == NONLINEAR )
    {
        pImu->addImuInfo( dt, frame_count, linear_acceleration, angular_velocity );

        int j = frame_count;
        if ( frame_count != 0 )
            pImu->predictWindow( pWindow->Pose[j], pWindow->Vel[j], dt, frame_count, linear_acceleration, angular_velocity );
    }

    pImu->m_accLast = linear_acceleration;
    pImu->m_gyrLast = angular_velocity;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void
Estimator::processImage( const FeatureData& image, const std_msgs::Header& header )
{
    ROS_DEBUG( "new image coming ------------------------------------------" );

    // data input
    if ( solver_flag == NONLINEAR )
    {
        pWindow->addFeaturesToWindow( frame_count, image );
        pImu->setMargFlag( pWindow->margFlag( ) );

        ROS_DEBUG( "this frame is--------------------%s", pWindow->margFlag( ) ? "reject" : "accept" );
        ROS_DEBUG( "%s", pWindow->margFlag( ) ? "Non-keyframe" : "Keyframe" );
        ROS_DEBUG( "Solving %d", frame_count );
        ROS_DEBUG( "number of feature: %d", pWindow->m_featureManager.getFeatureCount( ) );
        pWindow->Stamps[frame_count] = header.stamp.toSec( );
    }

    if ( solver_flag == INITIAL )
    {
        vioInitialSys->pushImage( header.stamp.toSec( ), frame_count, image );
        vioInitialSys->resetImu( acc_0, gyr_0, pImu->m_Bas[frame_count], pImu->m_Bgs[frame_count] );
    }

    if ( solver_flag == INITIAL )
    {
        if ( frame_count == WINDOW_SIZE )
        {
            if ( ( header.stamp.toSec( ) - initial_timestamp ) > 0.1 )
            {
                vioInitialSys->initial( );

                initial_timestamp = header.stamp.toSec( );
            }
            if ( vioInitialSys->Done( ) )
            {
                solver_flag = NONLINEAR;
                vioInitialSys->copyInitInfoBack( pWindow, pImu, g );

                solveOdometry( );
                slideWindow( );
                pWindow->m_featureManager.removeFailures( );
                ROS_INFO( "Initialization finish!" );

                last_Pose = pWindow->poseLast( );
                last_vel  = pWindow->lastVel( );
                last_tf0  = pWindow->Pose[0];
            }
            else
                slideWindow( );
        }
        else
            frame_count++;
    }
    else
    {

        TicToc t_solve;
        solveOdometry( );
        ROS_DEBUG( "solver costs: %fms", t_solve.toc( ) );

        if ( failureDetection( ) )
        {
            ROS_WARN( "failure detection!" );
            failure_occur = 1;
            clearState( );
            setParameter( );
            ROS_WARN( "system reboot!" );
            return;
        }

        TicToc t_margin;
        slideWindow( );
        pWindow->m_featureManager.removeFailures( );
        ROS_DEBUG( "marginalization costs: %fms", t_margin.toc( ) );
        // prepare output of VINS
        key_poses.clear( );
        for ( int i = 0; i <= WINDOW_SIZE; i++ )
            key_poses.push_back( pWindow->Pose[i].T );

        last_Pose = pWindow->poseLast( );
        last_vel  = pWindow->lastVel( );
        last_tf0  = pWindow->Pose[0];
    }
}

void
Estimator::solveOdometry( )
{
    if ( solver_flag == NONLINEAR )
    {
        TicToc t_tri;
        pWindow->m_featureManager.triangulate( pWindow->Pose, tf_ics );
        ROS_DEBUG( "triangulation costs %f", t_tri.toc( ) );
        optimization( );
    }
}

void
Estimator::vector2double( )
{

    pWindow->windowToDouble( paraPose, paraSpeed );
    pImu->windowToDouble( paraBias );

    for ( int i = 0; i < NUM_OF_CAM; i++ )
        tf_ics[i].TfToDouble( paraExPose[i] );

    int num_of_depth = 0;
    VectorXd dep = pWindow->m_featureManager.getDepth( num_of_depth, para_Feature_CameraID );
    ROS_WARN_STREAM( "feature size " << pWindow->m_featureManager.getFeatureCount( ) << ", mono "
                                     << pWindow->m_featureManager.getFeatureCountMono( ) << ", stereo "
                                     << pWindow->m_featureManager.getFeatureCountStereo( ) );

    for ( int i = 0; i < num_of_depth; i++ )
    {
        paraFeature[i][0] = dep( i );
    }
}

void
Estimator::double2vector( )
{
    Vector3d origin_R0 = Utility::R2ypr( pWindow->Pose[0].R );
    Vector3d origin_P0 = pWindow->Pose[0].T;

    if ( failure_occur )
    {
        origin_R0     = Utility::R2ypr( last_tf0.R );
        origin_P0     = last_tf0.T;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr( Quaterniond( paraPose[0][6], //
                                                       paraPose[0][3],
                                                       paraPose[0][4],
                                                       paraPose[0][5] )
                                          .toRotationMatrix( ) );
    double y_diff       = origin_R0.x( ) - origin_R00.x( );

    Matrix3d rot_diff = Utility::ypr2R( Vector3d( y_diff, 0, 0 ) );

    for ( int i = 0; i <= WINDOW_SIZE; i++ )
    {
        pWindow->setPoseIndex( i,
                               rot_diff
                               * Vector3d( paraPose[i][0] - paraPose[0][0],
                                           paraPose[i][1] - paraPose[0][1],
                                           paraPose[i][2] - paraPose[0][2] )
                               + origin_P0,
                               rot_diff
                               * Quaterniond( paraPose[i][6], //
                                              paraPose[i][3],
                                              paraPose[i][4],
                                              paraPose[i][5] )
                                 .normalized( )
                                 .toRotationMatrix( ) );

        pWindow->setVelIndex( i,
                              rot_diff
                              * Vector3d( paraSpeed[i][0], //
                                          paraSpeed[i][1],
                                          paraSpeed[i][2] ) );

        pImu->setBiasAccIndex( i, Vector3d( paraBias[i][0], paraBias[i][1], paraBias[i][2] ) );
        pImu->setBiasGyroIndex( i, Vector3d( paraBias[i][3], paraBias[i][4], paraBias[i][5] ) );
    }

    if ( ESTIMATE_EXTRINSIC )
    {
        for ( int i = 0; i < NUM_OF_CAM; i++ )
        {
            tf_ics[i].setRT( Quaterniond( paraExPose[i][6],
                                          paraExPose[i][3], //
                                          paraExPose[i][4],
                                          paraExPose[i][5] )
                             .toRotationMatrix( ),
                             Vector3d( paraExPose[i][0], //
                                       paraExPose[i][1],
                                       paraExPose[i][2] ) );

            ROS_DEBUG_STREAM( " ex ics " << i << endl << tf_ics[i] );
        }
        for ( int i = 0; i < NUM_OF_CAM - 1; i++ )
        {
            Tf tf_rl        = tf_ics[i + 1].inverse( ) + tf_ics[i];
            Tf tf_lr        = tf_rl.inverse( );
            double baseline = tf_lr.norm( );

            tf_rls.push_back( tf_rl );
            tf_lrs.push_back( tf_lr );
            baselines.push_back( baseline );

            pWindow->m_featureManager.setBaseline( baseline );
            pWindow->m_featureManager.setTfrl( tf_rl );
        }
    }

    int num_of_depth = 0;
    VectorXd dep = pWindow->m_featureManager.getDepth( num_of_depth, para_Feature_CameraID );
    for ( int i = 0; i < num_of_depth; i++ )
        dep( i ) = paraFeature[i][0];
    //    ROS_DEBUG_STREAM( " dep " << endl << dep );
    pWindow->m_featureManager.setDepth( dep );
}

bool
Estimator::failureDetection( )
{
    if ( pWindow->m_featureManager.last_track_num < 2 )
    {
        ROS_INFO( " little feature %d", pWindow->m_featureManager.last_track_num );
        return true;
    }
    if ( pImu->m_Bas[WINDOW_SIZE].norm( ) > 2.5 )
    {
        ROS_INFO( " big IMU acc bias estimation %f", pImu->m_Bas[WINDOW_SIZE].norm( ) );
        return true;
    }
    if ( pImu->m_Bgs[WINDOW_SIZE].norm( ) > 1.0 )
    {
        ROS_INFO( " big IMU gyr bias estimation %f", pImu->m_Bgs[WINDOW_SIZE].norm( ) );
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = pWindow->Pose[WINDOW_SIZE].T;
    if ( ( tmp_P - last_Pose.T ).norm( ) > 5 )
    {
        ROS_INFO( " big translation" );
        return true;
    }
    if ( abs( tmp_P.z( ) - last_Pose.T.z( ) ) > 1 )
    {
        ROS_INFO( " big z translation" );
        return true;
    }
    //    Matrix3d tmp_R   = Rs[WINDOW_SIZE];
    Matrix3d tmp_R   = pWindow->Pose[WINDOW_SIZE].R;
    Matrix3d delta_R = tmp_R.transpose( ) * last_Pose.R;
    Quaterniond delta_Q( delta_R );
    double delta_angle;
    delta_angle = acos( delta_Q.w( ) ) * 2.0 / 3.14 * 180.0;
    if ( delta_angle > 50 )
    {
        ROS_INFO( " big delta_angle " );
        return true;
    }
    return false;
}

void
Estimator::optimization( )
{
    ceres::Problem problem;
    ceres::LossFunction* loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss( 1.0 );
    for ( int i = 0; i < WINDOW_SIZE + 1; i++ )
    {
        ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization( );
        problem.AddParameterBlock( paraPose[i], SIZE_POSE, local_parameterization );
        problem.AddParameterBlock( paraSpeed[i], SIZE_SPEED );
        problem.AddParameterBlock( paraBias[i], SIZE_BIAS );
    }
    for ( int i = 0; i < NUM_OF_CAM; i++ )
    {
        ROS_DEBUG_STREAM( "optimization CAM " << i );
        ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization( );
        problem.AddParameterBlock( paraExPose[i], SIZE_POSE, local_parameterization );

        if ( ESTIMATE_EXTRINSIC )
        {
            if ( !pImu->checkObservibility( ) )
            {
                ROS_DEBUG( "fix extinsic param" );
                problem.SetParameterBlockConstant( paraExPose[i] );
            }
        }
        else
        {
            ROS_DEBUG( "fix extinsic param" );
            problem.SetParameterBlockConstant( paraExPose[i] );
        }
    }

    TicToc t_whole, t_prepare;
    vector2double( );

#if MARG
    if ( last_marginalization_info )
    {
        // construct new marginlization_factor
        MarginalizationFactor* marginalization_factor
        = new MarginalizationFactor( last_marginalization_info );
        problem.AddResidualBlock( marginalization_factor, //
                                  NULL,
                                  last_marginalization_parameter_blocks );
    }
#endif

    for ( int i = 0; i < WINDOW_SIZE; i++ )
    {
        int j = i + 1;
        if ( pImu->m_preIntegrations[j]->sum_dt > 10.0 )
            continue;
        IMUFactor* imu_factor = new IMUFactor( pImu->m_preIntegrations[j] );
        problem.AddResidualBlock( imu_factor,
                                  NULL,
                                  paraPose[i],
                                  paraSpeed[i],
                                  paraBias[i], //
                                  paraPose[j],
                                  paraSpeed[j],
                                  paraBias[j] );
    }

    TicToc t_feature;
    int f_m_cnt       = 0;
    int feature_index = -1;
    for ( auto& it_per_id : pWindow->m_featureManager.feature )
    {

        // ROS_DEBUG_STREAM( "id " << it_per_id.m_featureId << " keep "
        //                        << frame_count - it_per_id.m_startFrame );

        it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );
        if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;

        ++feature_index;

        int imu_i = it_per_id.m_startFrame;
        int imu_j = imu_i - 1;

        Vector3d pts_i  = it_per_id.m_fPerFrame[0].m_fPerCam[0].m_measPoint;
        int camera_id_i = it_per_id.m_fPerFrame[0].m_fPerCam[0].m_cameraId;

        for ( auto& it_per_frame : it_per_id.m_fPerFrame )
        {
            imu_j++;
            if ( imu_i == imu_j )
            {
                continue;
            }

            for ( auto& it_per_cam : it_per_frame.m_fPerCam )
            {
                if ( camera_id_i == it_per_cam.m_cameraId )
                {
                    Vector3d pts_j  = it_per_cam.m_measPoint;
                    double errAngle = it_per_cam.m_errAngle;

                    ProjectionFactorSingleCam* f
                    = new ProjectionFactorSingleCam( pts_i, pts_j, errAngle );

                    problem.AddResidualBlock( f,
                                              loss_function,
                                              paraPose[imu_i],
                                              paraPose[imu_j],
                                              paraFeature[feature_index],
                                              paraExPose[camera_id_i] );
                }
                else if ( it_per_frame.m_numOfMeas > 1 //
                          && camera_id_i != it_per_cam.m_cameraId )
                {
                    Vector3d pts_j  = it_per_cam.m_measPoint;
                    int camera_id_j = it_per_cam.m_cameraId;
                    double errAngle = it_per_cam.m_errAngle;

                    ProjectionFactorMultiCam* f = new ProjectionFactorMultiCam( pts_i, pts_j, errAngle );
                    problem.AddResidualBlock( f,
                                              loss_function,
                                              paraPose[imu_i],
                                              paraPose[imu_j],
                                              paraFeature[feature_index],
                                              paraExPose[camera_id_i], //
                                              paraExPose[camera_id_j]  //
                    );
                }
                f_m_cnt++;
            }
        }
    }
    relocalize = false;

    ROS_DEBUG( "visual measurement count: %d", f_m_cnt );
    ROS_DEBUG( "prepare for ceres: %f, visual constrain: %f", t_prepare.toc( ), t_feature.toc( ) );

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations         = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if ( pWindow->margFlag( ) == MARGIN_OLD )
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );

    // cout << summary.BriefReport() << endl;
    ROS_DEBUG( "Iterations : %d", static_cast< int >( summary.iterations.size( ) ) );
    ROS_DEBUG( "solver costs: %f", t_solver.toc( ) );

    double2vector( );

#if MARG

    TicToc t_whole_marginalization;
    if ( pWindow->margFlag( ) == MARGIN_OLD )
    {
        MarginalizationInfoPtr marginalization_info( new MarginalizationInfo( ) );
        vector2double( );

        if ( last_marginalization_info )
        {
            vector< int > drop_set;
            for ( int i = 0; i < static_cast< int >( last_marginalization_parameter_blocks.size( ) ); i++ )
            {
                if ( last_marginalization_parameter_blocks[i] == paraPose[0]
                     || last_marginalization_parameter_blocks[i] == paraSpeed[0]
                     || last_marginalization_parameter_blocks[i] == paraBias[0] )
                    drop_set.push_back( i );
            }
            // construct new marginlization_factor
            MarginalizationFactor* margFactor = new MarginalizationFactor( last_marginalization_info );
            ResidualBlockInfo* residual_block_info
            = new ResidualBlockInfo( margFactor, //
                                     NULL,
                                     last_marginalization_parameter_blocks,
                                     drop_set );

            marginalization_info->addResidualBlockInfo( residual_block_info );
        }

        {
            if ( pImu->m_preIntegrations[1]->sum_dt < 10.0 )
            {
                IMUFactor* imu_factor = new IMUFactor( pImu->m_preIntegrations[1] );
                ResidualBlockInfo* residual_block_info
                = new ResidualBlockInfo( imu_factor,
                                         NULL,
                                         vector< double* >{ paraPose[0], //
                                                            paraSpeed[0],
                                                            paraBias[0],
                                                            paraPose[1],
                                                            paraSpeed[1],
                                                            paraBias[1] },
                                         vector< int >{ 0, 1, 2 } );
                marginalization_info->addResidualBlockInfo( residual_block_info );
            }
        }

        {
            int feature_index = -1;
            for ( auto& it_per_id : pWindow->m_featureManager.feature )
            {
                it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );
                if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
                    continue;

                ++feature_index;

                int imu_i = it_per_id.m_startFrame, imu_j = imu_i - 1;
                if ( imu_i != 0 )
                    continue;

                Vector3d pts_i  = it_per_id.m_fPerFrame[0].m_fPerCam[0].m_measPoint;
                int camera_id_i = it_per_id.m_fPerFrame[0].m_fPerCam[0].m_cameraId;

                for ( auto& it_per_frame : it_per_id.m_fPerFrame )
                {
                    imu_j++;
                    if ( imu_i == imu_j )
                    {
                        continue;
                    }

                    for ( auto& it_per_cam : it_per_frame.m_fPerCam )
                    {
                        if ( camera_id_i == it_per_cam.m_cameraId )
                        {
                            Vector3d pts_j  = it_per_cam.m_measPoint;
                            double errAngle = it_per_cam.m_errAngle;

                            ProjectionFactorSingleCam* f
                            = new ProjectionFactorSingleCam( pts_i, pts_j, errAngle );

                            ResidualBlockInfo* residual_block_info
                            = new ResidualBlockInfo( f,
                                                     loss_function,
                                                     vector< double* >{ paraPose[imu_i],
                                                                        paraPose[imu_j],
                                                                        paraFeature[feature_index],
                                                                        paraExPose[camera_id_i] },
                                                     vector< int >{ 0, 2 } );

                            marginalization_info->addResidualBlockInfo( residual_block_info );
                        }
                        else if ( it_per_frame.m_numOfMeas > 1 //
                                  && camera_id_i != it_per_cam.m_cameraId )
                        {
                            Vector3d pts_j  = it_per_cam.m_measPoint;
                            int camera_id_j = it_per_cam.m_cameraId;
                            double errAngle = it_per_cam.m_errAngle;

                            ProjectionFactorMultiCam* f
                            = new ProjectionFactorMultiCam( pts_i, pts_j, errAngle );

                            ResidualBlockInfo* residual_block_info
                            = new ResidualBlockInfo( f,
                                                     loss_function,
                                                     vector< double* >{ paraPose[imu_i],
                                                                        paraPose[imu_j],
                                                                        paraFeature[feature_index],
                                                                        paraExPose[camera_id_i],
                                                                        paraExPose[camera_id_j] },
                                                     vector< int >{ 0, 2 } );

                            marginalization_info->addResidualBlockInfo( residual_block_info );
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize( );
        ROS_DEBUG( "pre marginalization %f ms", t_pre_margin.toc( ) );

        TicToc t_margin;
        marginalization_info->marginalize( );
        ROS_DEBUG( "marginalization %f ms", t_margin.toc( ) );

        std::unordered_map< long, double* > addr_shift;
        for ( int i = 1; i <= WINDOW_SIZE; i++ )
        {
            addr_shift[reinterpret_cast< long >( paraPose[i] )]  = paraPose[i - 1];
            addr_shift[reinterpret_cast< long >( paraSpeed[i] )] = paraSpeed[i - 1];
            addr_shift[reinterpret_cast< long >( paraBias[i] )]  = paraBias[i - 1];
        }
        for ( int i = 0; i < NUM_OF_CAM; i++ )
        {
            addr_shift[reinterpret_cast< long >( paraExPose[i] )] = paraExPose[i];
        }

        vector< double* > parameter_blocks = marginalization_info->getParameterBlocks( addr_shift );

        last_marginalization_info.reset( );
        last_marginalization_info             = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if ( last_marginalization_info //
             && std::count( std::begin( last_marginalization_parameter_blocks ),
                            std::end( last_marginalization_parameter_blocks ),
                            paraPose[WINDOW_SIZE - 1] ) )
        {

            MarginalizationInfoPtr marginalization_info( new MarginalizationInfo( ) );
            vector2double( );
            if ( last_marginalization_info )
            {
                vector< int > drop_set;
                for ( int i = 0;
                      i < static_cast< int >( last_marginalization_parameter_blocks.size( ) );
                      i++ )
                {
                    ROS_ASSERT( last_marginalization_parameter_blocks[i] != paraSpeed[WINDOW_SIZE - 1] );
                    if ( last_marginalization_parameter_blocks[i] == paraPose[WINDOW_SIZE - 1] )
                        drop_set.push_back( i );
                }
                // construct new marginlization_factor
                MarginalizationFactor* marginalization_factor
                = new MarginalizationFactor( last_marginalization_info );
                ResidualBlockInfo* residual_block_info
                = new ResidualBlockInfo( marginalization_factor, //
                                         NULL,
                                         last_marginalization_parameter_blocks,
                                         drop_set );

                marginalization_info->addResidualBlockInfo( residual_block_info );
            }

            TicToc t_pre_margin;
            ROS_DEBUG( "begin marginalization" );
            marginalization_info->preMarginalize( );
            ROS_DEBUG( "end pre marginalization, %f ms", t_pre_margin.toc( ) );

            TicToc t_margin;
            ROS_DEBUG( "begin marginalization" );
            marginalization_info->marginalize( );
            ROS_DEBUG( "end marginalization, %f ms", t_margin.toc( ) );

            std::unordered_map< long, double* > addr_shift;
            for ( int i = 0; i <= WINDOW_SIZE; i++ )
            {
                if ( i == WINDOW_SIZE - 1 )
                    continue;
                else if ( i == WINDOW_SIZE )
                {
                    addr_shift[reinterpret_cast< long >( paraPose[i] )]  = paraPose[i - 1];
                    addr_shift[reinterpret_cast< long >( paraSpeed[i] )] = paraSpeed[i - 1];
                    addr_shift[reinterpret_cast< long >( paraBias[i] )]  = paraBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast< long >( paraPose[i] )]  = paraPose[i];
                    addr_shift[reinterpret_cast< long >( paraSpeed[i] )] = paraSpeed[i];
                    addr_shift[reinterpret_cast< long >( paraBias[i] )]  = paraBias[i];
                }
            }
            for ( int i = 0; i < NUM_OF_CAM; i++ )
            {
                addr_shift[reinterpret_cast< long >( paraExPose[i] )] = paraExPose[i];
            }

            vector< double* > parameter_blocks = marginalization_info->getParameterBlocks( addr_shift );

            last_marginalization_info.reset( );
            last_marginalization_info             = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    ROS_DEBUG( "whole marginalization costs: %f", t_whole_marginalization.toc( ) );
#endif

    ROS_DEBUG( "whole time for ceres: %f", t_whole.toc( ) );
}

void
Estimator::slideWindow( )
{
    TicTocPart t_margin;

    bool shift_depth = solver_flag == NONLINEAR ? true : false;

    if ( solver_flag == INITIAL )
    {
        vioInitialSys->slideWindow( tf_ics );
    }
    else if ( solver_flag == NONLINEAR )
    {
        pImu->slidewindow( );

        pWindow->slideWindow( shift_depth, tf_ics );
    }

    ROS_DEBUG_STREAM( "margin cost " << t_margin.tocEnd( ) );
}
