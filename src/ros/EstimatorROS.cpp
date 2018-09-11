#include "vins_so/ros/EstimatorROS.h"

EstimatorROS::EstimatorROS( ros::NodeHandle& n )
: EstimateIOROS( n )
{
    // Estimator
    initEstimator( WINDOW_SIZE, NUM_OF_CAM );
    setParameter( );

    // EstimateIOROS
#ifdef MEC_WHEEL
    InitSubscribe( n, IMU_TOPIC, "/feature/feature", "/wheelSpeeds" );
#else
    InitSubscribe( n, IMU_TOPIC, "/feature/feature" );
#endif
}

void
EstimatorROS::update( )
{
    TicToc t_predict;
    {
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
    }

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
EstimatorROS::process( )
{
    while ( true )
    {

        std::unique_lock< std::mutex > lk( m_buf );

#ifdef MEC_WHEEL
        VisualInertialMecMeasurements measurements;
        con.wait( lk, [&] {
            return ( measurements = getVisualInertialMecMeasurements( ) ).size( ) != 0;
        } );
#else
        VisualInertialMeasurements measurements;
        con.wait( lk, [&] {
            return ( measurements = getVisualInertialMeasurements( ) ).size( ) != 0;
        } );
#endif

        lk.unlock( );

        for ( auto& measurement : measurements )
        {
            for ( auto& imu_msg : measurement.imu )
                send_imu( imu_msg );

#ifdef MEC_WHEEL
            vector< double > wheel_vels;
            for ( auto& vel : measurement.wheel->speeds )
                wheel_vels.push_back( vel );
            int imu_size  = measurement.imu.size( );
            auto& end_imu = measurement.imu[imu_size - 1];
            Eigen::Vector3d gyr( end_imu->angular_velocity.x,
                                 end_imu->angular_velocity.y,
                                 end_imu->angular_velocity.z );
            processWheel( wheel_vels, gyr );
            ROS_ERROR_STREAM( "gyr " << gyr.transpose( ) );
            ROS_ERROR_STREAM( "wheel_vels " << wheel_vels[0] << " " << wheel_vels[1] << " "
                                            << wheel_vels[2] << " " << wheel_vels[3] << " " );
#endif

            auto img_msg = measurement.feature;
            ROS_DEBUG( "processing vision data with stamp %f \n", img_msg->header.stamp.toSec( ) );

            TicToc t_s;
            FeatureData image; // map< int, vector< pair< int, Vector3d > > >
            for ( unsigned int i = 0; i < img_msg->points.size( ); i++ )
            {
                int feature_id = img_msg->channels[0].values[i] + 0.4; // feature id
                int camera_id  = img_msg->channels[1].values[i] + 0.4; // camera id
                double error   = img_msg->channels[2].values[i];       // error angle

                if ( camera_id > NUM_OF_CAM - 1 )
                    continue;

                image[feature_id].emplace_back( camera_id,
                                                error,
                                                Vector3d( img_msg->points[i].x, //
                                                          img_msg->points[i].y,
                                                          img_msg->points[i].z )
                                                .normalized( ) );
            }
            processImage( image, img_msg->header );

            double whole_t = t_s.toc( );
            printStatistics( *this, whole_t );

            std_msgs::Header header = img_msg->header;
            header.frame_id         = "world";
            cur_header              = header;
            m_loop_drift.lock( );
            if ( relocalize )
            {
                relocalize_t = relocalize_t;
                relocalize_r = relocalize_r;
            }

            pubOdometry( *this, header, relocalize_t, relocalize_r );
            pubKeyPoses( *this, header, relocalize_t, relocalize_r );
            pubCameraPose( *this, header, relocalize_t, relocalize_r );
            pubPointCloud( *this, header, relocalize_t, relocalize_r );
            pubTF( *this, header, relocalize_t, relocalize_r );
            m_loop_drift.unlock( );
        }
        m_buf.lock( );
        m_state.lock( );
        if ( solver_flag == Estimator::SolverFlag::NONLINEAR )
        {
            update( );
        }
        m_state.unlock( );
        m_buf.unlock( );
    }
}
