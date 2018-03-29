#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#define IS_LOOP 0

#include "camera_model/camera_models/CameraFactory.h"
#include "vins_so/estimator/estimator.h"
#include "vins_so/estimator/vins_parameters.h"
#include "vins_so/utility/visualization.h"
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/ros.h>
#include <stdio.h>
#include <thread>

Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue< sensor_msgs::ImuConstPtr > imu_buf;
queue< sensor_msgs::PointCloudConstPtr > feature_buf;
queue< pair< cv::Mat, double > > image_buf;

std::mutex m_posegraph_buf;
queue< int > optimize_posegraph_buf;

int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_loop_drift;
std::mutex m_update_visualization;
std::mutex m_keyframe_buf;
std::mutex m_retrive_data_buf;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

int global_frame_cnt = 0;
camera_model::CameraPtr m_camera;
vector< int > erase_index;
std_msgs::Header cur_header;
Eigen::Vector3d relocalize_t{ Eigen::Vector3d( 0, 0, 0 ) };
Eigen::Matrix3d relocalize_r{ Eigen::Matrix3d::Identity( ) };

void
predict( const sensor_msgs::ImuConstPtr& imu_msg )
{
    double t    = imu_msg->header.stamp.toSec( );
    double dt   = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{ dx, dy, dz };

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{ rx, ry, rz };

    Eigen::Vector3d un_acc_0 = tmp_Q * ( acc_0 - tmp_Ba - tmp_Q.inverse( ) * estimator.g );

    Eigen::Vector3d un_gyr = 0.5 * ( gyr_0 + angular_velocity ) - tmp_Bg;

    tmp_Q = tmp_Q * Utility::deltaQ( un_gyr * dt );

    Eigen::Vector3d un_acc_1
    = tmp_Q * ( linear_acceleration - tmp_Ba - tmp_Q.inverse( ) * estimator.g );

    Eigen::Vector3d un_acc = 0.5 * ( un_acc_0 + un_acc_1 );

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void
update( )
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P       = relocalize_r * estimator.pWindow->Pose[WINDOW_SIZE].T + relocalize_t;
    tmp_Q       = relocalize_r * estimator.pWindow->Pose[WINDOW_SIZE].R;
    tmp_V       = estimator.pWindow->Vel[WINDOW_SIZE];
    tmp_Ba      = estimator.pImu->m_Bas[WINDOW_SIZE];
    tmp_Bg      = estimator.pImu->m_Bgs[WINDOW_SIZE];
    acc_0       = estimator.acc_0;
    gyr_0       = estimator.gyr_0;

    queue< sensor_msgs::ImuConstPtr > tmp_imu_buf = imu_buf;
    for ( sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty( ); tmp_imu_buf.pop( ) )
        predict( tmp_imu_buf.front( ) );
}

std::vector< std::pair< std::vector< sensor_msgs::ImuConstPtr >, sensor_msgs::PointCloudConstPtr > >
getMeasurements( )
{
    std::vector< std::pair< std::vector< sensor_msgs::ImuConstPtr >, sensor_msgs::PointCloudConstPtr > > measurements;

    while ( true )
    {
        //        if ( imu_buf.empty( ) || feature_bufs.at( 0 ).empty( ) )
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

void
imu_callback( const sensor_msgs::ImuConstPtr& imu_msg )
{
    m_buf.lock( );
    imu_buf.push( imu_msg );
    m_buf.unlock( );
    con.notify_one( );

    {
        std::lock_guard< std::mutex > lg( m_state );
        predict( imu_msg );

        std_msgs::Header header = imu_msg->header;
        header.frame_id         = "world";
        if ( estimator.solver_flag == Estimator::SolverFlag::NONLINEAR )
        {
            //            std::cout << "V " << tmp_V.transpose( ) << "\n";
            pubLatestOdometry( tmp_P, tmp_Q, tmp_V, header );
        }
    }
}

void
raw_image_callback( const sensor_msgs::ImageConstPtr& img_msg )
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

void
feature_callback( const sensor_msgs::PointCloudConstPtr& feature_msg )
{
    m_buf.lock( );
    feature_buf.push( feature_msg );
    m_buf.unlock( );
    con.notify_one( );
}

void
send_imu( const sensor_msgs::ImuConstPtr& imu_msg )
{
    double t = imu_msg->header.stamp.toSec( );
    if ( current_time < 0 )
        current_time = t;
    double dt        = t - current_time;
    current_time     = t;

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

    estimator.processIMU( dt, Vector3d( dx, dy, dz ), Vector3d( rx, ry, rz ) );
}

// thread: visual-inertial odometry
void
process( )
{
    while ( true )
    {
        std::vector< std::pair< std::vector< sensor_msgs::ImuConstPtr >, sensor_msgs::PointCloudConstPtr > > measurements;
        std::unique_lock< std::mutex > lk( m_buf );
        con.wait( lk, [&] { return ( measurements = getMeasurements( ) ).size( ) != 0; } );
        lk.unlock( );

        for ( auto& measurement : measurements )
        {
            for ( auto& imu_msg : measurement.first )
                send_imu( imu_msg );

            auto img_msg = measurement.second;
            ROS_DEBUG( "processing vision data with stamp %f \n", img_msg->header.stamp.toSec( ) );

            TicToc t_s;
            FeatureData image; // map< int, vector< pair< int, Vector3d > > >
            for ( unsigned int i = 0; i < img_msg->points.size( ); i++ )
            {
                int v          = img_msg->channels[0].values[i] + 0.5;
                int camera_id  = img_msg->channels[1].values[i];
                int feature_id = v;

                image[feature_id].emplace_back( camera_id,
                                                Vector3d( img_msg->points[i].x, //
                                                          img_msg->points[i].y,
                                                          img_msg->points[i].z )
                                                .normalized( ) );
            }
            estimator.processImage( image, img_msg->header );

            double whole_t = t_s.toc( );
            printStatistics( estimator, whole_t );

            std_msgs::Header header = img_msg->header;
            header.frame_id         = "world";
            cur_header              = header;
            m_loop_drift.lock( );
            if ( estimator.relocalize )
            {
                relocalize_t = estimator.relocalize_t;
                relocalize_r = estimator.relocalize_r;
            }
            if ( estimator.pWindow->Pose[WINDOW_SIZE].T.z( ) > 0.3
                 && estimator.solver_flag != Estimator::INITIAL )
                IN_AIR = true;
            else
                IN_AIR = false;

            // ROS_WARN_STREAM( "IN_AIR " << IN_AIR << " , "
            //                           << estimator.pWindow->Pose[WINDOW_SIZE].T.z( ) );

            pubOdometry( estimator, header, relocalize_t, relocalize_r );
            pubKeyPoses( estimator, header, relocalize_t, relocalize_r );
            pubCameraPose( estimator, header, relocalize_t, relocalize_r );
            pubPointCloud( estimator, header, relocalize_t, relocalize_r );
            pubTF( estimator, header, relocalize_t, relocalize_r );

            m_loop_drift.unlock( );
        }
        m_buf.lock( );
        m_state.lock( );
        if ( estimator.solver_flag == Estimator::SolverFlag::NONLINEAR )
            update( );
        m_state.unlock( );
        m_buf.unlock( );
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "vins_estimator" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );
    readParameters( n );

    estimator.initEstimator( WINDOW_SIZE, NUM_OF_CAM );

    estimator.setParameter( );
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG( "EIGEN_DONT_PARALLELIZE" );
#endif
    ROS_WARN( "waiting for image and imu..." );

    registerPub( n );

    ros::Subscriber sub_imu
    = n.subscribe( IMU_TOPIC, 2000, imu_callback, ros::TransportHints( ).tcpNoDelay( ) );

    std::vector< ros::Subscriber > sub_images;
    std::vector< ros::Subscriber > sub_raw_images;

    for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
    {

        //  ros::Subscriber sub_raw_image = n.subscribe( IMAGE_TOPICS[camera_index],
        //                                               2000,
        //                                               raw_image_callback,
        //                                               ros::TransportHints( ).tcpNoDelay(
        //                                               ) );
        //
        //  sub_raw_images.push_back( sub_raw_image );
    }

    ros::Subscriber sub_image
    = n.subscribe< sensor_msgs::PointCloud >( "/feature/feature",
                                              2000,
                                              feature_callback,
                                              ros::VoidConstPtr( ),
                                              ros::TransportHints( ).tcpNoDelay( true ) );

    std::thread measurement_process{ process };

    ros::spin( );

    return 0;
}
