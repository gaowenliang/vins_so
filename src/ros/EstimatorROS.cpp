#include "vins_so/ros/EstimatorROS.h"

void
EstimatorROS::update( )
{
    TicToc t_predict;
    imuPropagate.latest_time = current_time;

    imuPropagate.m_P   = relocalize_r * pWindow->Pose[WINDOW_SIZE].T + relocalize_t;
    imuPropagate.m_Q   = relocalize_r * pWindow->Pose[WINDOW_SIZE].R;
    imuPropagate.m_V   = pWindow->Vel[WINDOW_SIZE];
    imuPropagate.m_Ba  = pImu->m_Bas[WINDOW_SIZE];
    imuPropagate.m_Bg  = pImu->m_Bgs[WINDOW_SIZE];
    imuPropagate.acc_0 = acc_0;
    imuPropagate.gyr_0 = gyr_0;
    imuPropagate.m_g   = g;
    imuPropagate.flag  = solver_flag;

    queue< sensor_msgs::ImuConstPtr > tmp_imu_buf = imu_buf;
    for ( sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty( ); tmp_imu_buf.pop( ) )
    {
        imuPropagate.predict( tmp_imu_buf.front( )->header.stamp.toSec( ),
                              tmp_imu_buf.front( )->linear_acceleration.x,
                              tmp_imu_buf.front( )->linear_acceleration.y,
                              tmp_imu_buf.front( )->linear_acceleration.z,
                              tmp_imu_buf.front( )->angular_velocity.x,
                              tmp_imu_buf.front( )->angular_velocity.y,
                              tmp_imu_buf.front( )->angular_velocity.z );
    }
}

void
EstimatorROS::send_imu( const sensor_msgs::ImuConstPtr& imu_msg )
{
    double t = imu_msg->header.stamp.toSec( );
    if ( current_time < 0 )
        current_time = t;
    double dt    = t - current_time;
    current_time = t;

    double ba[]{ 0.0, 0.0, 0.0 };
    double bg[]{ 0.0, 0.0, 0.0 };

    double dx = imu_msg->linear_acceleration.x - ba[0];
    double dy = imu_msg->linear_acceleration.y - ba[1];
    double dz = imu_msg->linear_acceleration.z - ba[2];

    double rx = imu_msg->angular_velocity.x - bg[0];
    double ry = imu_msg->angular_velocity.y - bg[1];
    double rz = imu_msg->angular_velocity.z - bg[2];
    // ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx,
    // ry, rz);

    processIMU( dt, Vector3d( dx, dy, dz ), Vector3d( rx, ry, rz ) );
}

void
EstimatorROS::readParameters( ros::NodeHandle& n )
{
    std::string vins_config_file;

    vins_config_file = readParam< std::string >( n, "vins_config_file" );
    VINS_FOLDER_PATH = readParam< std::string >( n, "vins_folder" );
    NUM_OF_CAM       = readParam< int >( n, "num_of_cam" );
    NUM_OF_STEREO    = readParam< int >( n, "camera_num_stereo" );

    std::cout << " NUM_OF_CAM " << NUM_OF_CAM << std::endl;
    std::cout << " NUM_OF_STEREO " << NUM_OF_STEREO << std::endl;
    if ( NUM_OF_STEREO >= NUM_OF_CAM )
    {
        ROS_ERROR( "ERROR with camera number!!!" );
        ros::shutdown( );
    }

    for ( int stereo_index = 0; stereo_index < NUM_OF_STEREO; ++stereo_index )
    {
        STEREO_CAM_IDS.push_back( std::pair< int, int >( stereo_index * 2, stereo_index * 2 + 1 ) );
        std::cout << "Stereo Pair: " << STEREO_CAM_IDS[stereo_index].first << " "
                  << STEREO_CAM_IDS[stereo_index].second << "\n";
    }

    // camera I/O parameters load
    for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
    {
        std::string prefix = boost::str( boost::format( "camera%d/" ) % camera_index );

        std::string camera_config_file
        = readParam< std::string >( n, prefix + "cam_config_file" );

        cv::FileStorage camera_fs( camera_config_file, cv::FileStorage::READ );
        if ( !camera_fs.isOpened( ) )
            std::cerr << "ERROR: Wrong path to settings 'camera_config_file', camera"
                      << camera_index << std::endl;

        std::string image_topic;
        std::string image_feature_topic;
        camera_fs["image_topic"] >> image_topic;
        camera_fs["feature_topic"] >> image_feature_topic;
        IMAGE_TOPICS.push_back( image_topic );
        FEATURE_TOPICS.push_back( image_feature_topic );
        CAM_NAMES.push_back( camera_config_file );

        IMAGE_COL = camera_fs["image_width"];
        IMAGE_ROW = camera_fs["image_height"];

        camera_fs.release( );
    }

    cv::FileStorage vins_fs( vins_config_file, cv::FileStorage::READ );
    if ( !vins_fs.isOpened( ) )
        std::cerr << "ERROR: Wrong path to settings 'vins_config_file'" << std::endl;

    WHEEL_MEC_TOPIC = "/wheelSpeeds";

    vins_fs["imu_topic"] >> IMU_TOPIC;

    MAX_DEPTH             = vins_fs["max_depth"];
    SOLVER_TIME           = vins_fs["max_solver_time"];
    NUM_ITERATIONS        = vins_fs["max_num_iterations"];
    MIN_PARALLAX          = vins_fs["keyframe_parallax"];
    MIN_VISIABLE_PARALLAX = vins_fs["min_visiable_parallax"];
    //    MIN_PARALLAX          = MIN_PARALLAX / FOCAL_LENGTH;
    MIN_PARALLAX = MIN_PARALLAX / 57.29;

    vins_fs["output_path"] >> VINS_RESULT_PATH;
    VINS_RESULT_PATH = VINS_FOLDER_PATH + VINS_RESULT_PATH;
    std::ofstream foutC( VINS_RESULT_PATH, std::ios::out );
    foutC.close( );

    ACC_N  = vins_fs["acc_n"];
    ACC_W  = vins_fs["acc_w"];
    GYR_N  = vins_fs["gyr_n"];
    GYR_W  = vins_fs["gyr_w"];
    G.z( ) = vins_fs["g_norm"];

    ESTIMATE_EXTRINSIC = vins_fs["estimate_extrinsic"];
    if ( ESTIMATE_EXTRINSIC == 2 )
    {
        ROS_WARN( "have no prior about extrinsic param, calibrate extrinsic param" );
        for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
        {
            RIC.push_back( Eigen::Matrix3d::Identity( ) );
            TIC.push_back( Eigen::Vector3d::Zero( ) );

            cv::FileStorage camera_fs( CAM_NAMES[camera_index], cv::FileStorage::READ );
            if ( !camera_fs.isOpened( ) )
                std::cerr
                << "ERROR: Wrong path to loading `ex_calib_result_path` when settings "
                   "'camera_config_file', camera"
                << camera_index << std::endl;

            std::string ex_calib_file;
            camera_fs["ex_calib_result_path"] >> ex_calib_file;
            EX_CALIB_RESULT_PATHS.push_back( VINS_FOLDER_PATH + ex_calib_file );
            camera_fs.release( );
        }
    }
    else
    {
        if ( ESTIMATE_EXTRINSIC == 1 )
            ROS_WARN( " Optimize extrinsic param around initial guess!" );
        if ( ESTIMATE_EXTRINSIC == 0 )
            ROS_WARN( " fix extrinsic param " );

        for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
        {
            cv::FileStorage camera_fs( CAM_NAMES[camera_index], cv::FileStorage::READ );
            if ( !camera_fs.isOpened( ) )
                std::cerr
                << "ERROR: Wrong path to loading `ex_calib_result_path` when settings "
                   "'camera_config_file', camera"
                << camera_index << std::endl;

            std::string ex_calib_file;
            camera_fs["ex_calib_result_path"] >> ex_calib_file;
            EX_CALIB_RESULT_PATHS.push_back( VINS_FOLDER_PATH + ex_calib_file );

            cv::Mat cv_R, cv_T;
            camera_fs["extrinsicRotation"] >> cv_R;
            camera_fs["extrinsicTranslation"] >> cv_T;

            camera_fs.release( );

            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            cv::cv2eigen( cv_R, eigen_R );
            cv::cv2eigen( cv_T, eigen_T );
            Eigen::Quaterniond Q( eigen_R );
            eigen_R = Q.normalized( );

            RIC.push_back( eigen_R );
            TIC.push_back( eigen_T );

            ROS_INFO_STREAM( "cam " << camera_index << " Ex_R : " << std::endl
                                    << RIC[camera_index] );
            ROS_INFO_STREAM( "cam " << camera_index << " Ex_T : " << std::endl
                                    << TIC[camera_index].transpose( ) );
        }
    }

    LOOP_CLOSURE = vins_fs["loop_closure"];
    if ( LOOP_CLOSURE == 1 )
    {
        vins_fs["voc_file"] >> VOC_FILE;

        vins_fs["pattern_file"] >> PATTERN_FILE;
        VOC_FILE     = VINS_FOLDER_PATH + VOC_FILE;
        PATTERN_FILE = VINS_FOLDER_PATH + PATTERN_FILE;
        MIN_LOOP_NUM = vins_fs["min_loop_num"];

        //        CAM_NAMES = config_file; // TODO
    }

    INIT_DEPTH         = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;
    MAX_KEYFRAME_NUM   = 1000;

    vins_fs.release( );
}
