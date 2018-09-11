#include "vins_so/ros/EstimateInputROS.h"
#include "nav_msgs/Odometry.h"

EstimateInputROS::EstimateInputROS( ros::NodeHandle& n )
{
    pub_latest_odometry = n.advertise< nav_msgs::Odometry >( "imu_propagate", 1000 );
}

void
EstimateInputROS::InitSubscribe( ros::NodeHandle& n, //
                                 string imu_topic_name,
                                 string feature_topic_name )
{
    sub_imu = n.subscribe( imu_topic_name, //
                           2000,
                           &EstimateInputROS::imu_callback,
                           this,
                           ros::TransportHints( ).tcpNoDelay( ) );

    sub_image = n.subscribe( feature_topic_name,
                             2000,
                             &EstimateInputROS::feature_callback,
                             this,
                             ros::TransportHints( ).tcpNoDelay( ) );
}

void
EstimateInputROS::InitSubscribe( ros::NodeHandle& n,
                                 string imu_topic_name,
                                 string feature_topic_name,
                                 std::vector< string > image_topics )
{
    sub_imu = n.subscribe( imu_topic_name, //
                           2000,
                           &EstimateInputROS::imu_callback,
                           this,
                           ros::TransportHints( ).tcpNoDelay( ) );

    sub_image = n.subscribe( feature_topic_name,
                             2000,
                             &EstimateInputROS::feature_callback,
                             this,
                             ros::TransportHints( ).tcpNoDelay( ) );
    for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
    {
        // ros::Subscriber sub_raw_image = n.subscribe( image_topics[camera_index],
        //                                             2000,
        //                                             raw_image_callback,
        //                                             ros::TransportHints(
        //                                             ).tcpNoDelay( ) );
        // sub_images.push_back( sub_raw_image );
    }
}

void
EstimateInputROS::InitSubscribe( ros::NodeHandle& n, //
                                 string imu_topic_name,
                                 string feature_topic_name,
                                 string wheel_topic_name )
{
    sub_imu = n.subscribe( imu_topic_name, //
                           2000,
                           &EstimateInputROS::imu_callback,
                           this,
                           ros::TransportHints( ).tcpNoDelay( ) );

    sub_image = n.subscribe( feature_topic_name,
                             2000,
                             &EstimateInputROS::feature_callback,
                             this,
                             ros::TransportHints( ).tcpNoDelay( ) );

    sub_mecWheel = n.subscribe( wheel_topic_name, //
                                200,
                                &EstimateInputROS::wheel_mec_callback,
                                this,
                                ros::TransportHints( ).tcpNoDelay( ) );
}

VisualInertialMeasurements
EstimateInputROS::getVisualInertialMeasurements( )
{
    VisualInertialMeasurements measurements;

    while ( true )
    {
        if ( imu_buf.empty( ) || feature_buf.empty( ) )
            return measurements;

        if ( !( imu_buf.back( )->header.stamp > feature_buf.front( )->header.stamp ) )
        {
            ROS_WARN( "wait for imu, only should happen at the beginning" );
            sum_of_wait++;
            return measurements;
        }

        if ( !( imu_buf.front( )->header.stamp < feature_buf.front( )->header.stamp ) )
        {
            ROS_WARN( "throw img, only should happen at the beginning" );
            feature_buf.pop( );
            continue;
        }

        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front( );
        feature_buf.pop( );

        std::vector< sensor_msgs::ImuConstPtr > IMUs;
        while ( imu_buf.front( )->header.stamp <= img_msg->header.stamp )
        {
            IMUs.emplace_back( imu_buf.front( ) );
            imu_buf.pop( );
        }

        measurements.emplace_back( IMUs, img_msg );
    }
    return measurements;
}

VisualInertialMecMeasurements
EstimateInputROS::getVisualInertialMecMeasurements( )
{
    VisualInertialMecMeasurements measurements;

    while ( true )
    {
        if ( imu_buf.empty( ) || feature_buf.empty( ) )
            return measurements;

        if ( !( imu_buf.back( )->header.stamp > feature_buf.front( )->header.stamp ) )
        {
            ROS_WARN( "wait for imu, only should happen at the beginning" );
            sum_of_wait++;
            return measurements;
        }

        if ( !( imu_buf.front( )->header.stamp < feature_buf.front( )->header.stamp ) )
        {
            ROS_WARN( "throw img, only should happen at the beginning" );
            feature_buf.pop( );
            continue;
        }

        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front( );
        feature_buf.pop( );

        std::vector< sensor_msgs::ImuConstPtr > IMUs;
        while ( imu_buf.front( )->header.stamp <= img_msg->header.stamp )
        {
            IMUs.emplace_back( imu_buf.front( ) );
            imu_buf.pop( );
        }

        ROS_ERROR_STREAM( "wheel_buf size " << wheel_buf.size( ) );

        std::vector< wheel_msgs::wheelSpeedsConstPtr > wheels;
        while ( wheel_buf.front( )->header.stamp <= img_msg->header.stamp )
        {
            wheels.emplace_back( wheel_buf.front( ) );
            wheel_buf.pop( );
        }
        int wheels_size                       = wheels.size( );
        wheel_msgs::wheelSpeedsConstPtr wheel = wheels[wheels_size - 1];

        measurements.emplace_back( IMUs, img_msg, wheel );
    }
    return measurements;
}

void
EstimateInputROS::pubLatestOdometry( const Vector3d& P,
                                     const Quaterniond& Q,
                                     const Vector3d& V,
                                     const std_msgs::Header& header )
{
    Eigen::Quaterniond quadrotor_Q = Q;

    nav_msgs::Odometry odometry;
    odometry.header                  = header;
    odometry.header.frame_id         = "world";
    odometry.pose.pose.position.x    = P.x( );
    odometry.pose.pose.position.y    = P.y( );
    odometry.pose.pose.position.z    = P.z( );
    odometry.pose.pose.orientation.x = quadrotor_Q.x( );
    odometry.pose.pose.orientation.y = quadrotor_Q.y( );
    odometry.pose.pose.orientation.z = quadrotor_Q.z( );
    odometry.pose.pose.orientation.w = quadrotor_Q.w( );
    odometry.twist.twist.linear.x    = V.x( );
    odometry.twist.twist.linear.y    = V.y( );
    odometry.twist.twist.linear.z    = V.z( );

    pub_latest_odometry.publish( odometry );
}

void
EstimateInputROS::imu_callback( const sensor_msgs::ImuConstPtr& imu_msg )
{
    m_buf.lock( );
    imu_buf.push( imu_msg );
    m_buf.unlock( );
    con.notify_one( );

    {
        std::lock_guard< std::mutex > lg( m_state );

        imuPropagate.predict( imu_msg->header.stamp.toSec( ),
                              imu_msg->linear_acceleration.x,
                              imu_msg->linear_acceleration.y,
                              imu_msg->linear_acceleration.z,
                              imu_msg->angular_velocity.x,
                              imu_msg->angular_velocity.y,
                              imu_msg->angular_velocity.z );

        std_msgs::Header header = imu_msg->header;
        header.frame_id         = "world";
        if ( imuPropagate.flag == Estimator::SolverFlag::NONLINEAR )
        {
            pubLatestOdometry( imuPropagate.m_P, imuPropagate.m_Q, imuPropagate.m_V, header );
        }
    }
}

void
EstimateInputROS::wheel_mec_callback( const wheel_msgs::wheelSpeedsConstPtr& msg )
{
    m_buf.lock( );
    wheel_buf.push( msg );
    m_buf.unlock( );
    con.notify_one( );
}

void
EstimateInputROS::feature_callback( const sensor_msgs::PointCloudConstPtr& feature_msg )
{
    m_buf.lock( );
    feature_buf.push( feature_msg );
    m_buf.unlock( );
    con.notify_one( );
}

void
EstimateInputROS::raw_image_callback( const sensor_msgs::ImageConstPtr& img_msg )
{
    cv_bridge::CvImagePtr img_ptr
    = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::MONO8 );

    // image_pool[img_msg->header.stamp.toNSec()] = img_ptr->image;
    if ( LOOP_CLOSURE )
    {
        i_buf.lock( );
        image_buf.push( make_pair( img_ptr->image, img_msg->header.stamp.toSec( ) ) );
        i_buf.unlock( );
    }
}
