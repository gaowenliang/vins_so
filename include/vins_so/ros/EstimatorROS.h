#ifndef ESTIMATORROS_H
#define ESTIMATORROS_H

#include "../estimator/estimator.h"
#include "../estimator/vins_parameters.h"
#include "../utility/ros_utility.h"
#include "ros_io.h"
#include <camera_model/camera_models/CameraFactory.h>
#include <mutex>
#include <thread>

class EstimatorROS
: public Estimator
, public EstimateIOROS
{
    public:
    EstimatorROS( ros::NodeHandle& n )
    : EstimateIOROS( n )
    {
        // Estimator
        readParameters( n );
        initEstimator( WINDOW_SIZE, NUM_OF_CAM );
        setParameter( );

        // EstimateIOROS
        InitSubscribe( n, IMU_TOPIC, "/feature/feature" );
    }

    void update( );
    void send_imu( const sensor_msgs::ImuConstPtr& imu_msg );

    // thread: visual-inertial odometry
    void process( )
    {
        while ( true )
        {
            VisualInertialMeasurements measurements;
            std::unique_lock< std::mutex > lk( m_buf );
            con.wait( lk, [&] { return ( measurements = getMeasurements( ) ).size( ) != 0; } );
            lk.unlock( );

            for ( auto& measurement : measurements )
            {
                for ( auto& imu_msg : measurement.imu )
                    send_imu( imu_msg );

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

                int imu_size = measurement.imu.size( );
                calcMec( measurement.imu[imu_size - 1] );

                // TODO
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

    void calcMec( const sensor_msgs::ImuConstPtr& imu_msg )
    {
        double gyr_x = imu_msg->angular_velocity.x //
                       - paraBias[WINDOW_SIZE][3];
        double gyr_y = imu_msg->angular_velocity.y //
                       - paraBias[WINDOW_SIZE][4];
        double gyr_z = imu_msg->angular_velocity.z //
                       - paraBias[WINDOW_SIZE][5];

        double m_a = 0.307 / 2;
        double m_b = 0.420 / 2;
        Quaterniond correct_q;
        Vector3d correct_v, v_b, v_add;
        correct_q = last_Pose.R;
        correct_v = last_vel;
        v_add = Eigen::Vector3d( gyr_x, gyr_y, gyr_z ).cross( Eigen::Vector3d( 0.05, 0, 0 ) );
        v_b   = correct_q.conjugate( ) * ( correct_v + v_add );

        // std::cout << "v_add " << v_add.transpose( ) << "\n";
        double v0 = v_b.x( ) + v_b.y( ) + ( m_a + m_b ) * gyr_z;
        double v1 = v_b.x( ) - v_b.y( ) - ( m_a + m_b ) * gyr_z;
        double v2 = v_b.x( ) + v_b.y( ) - ( m_a + m_b ) * gyr_z;
        double v3 = v_b.x( ) - v_b.y( ) + ( m_a + m_b ) * gyr_z;
        std::cout << "  " << v0 << " " << v1 << " " << v2 << " " << v3 << " "
                  << "\n";
    }
    void readParameters( ros::NodeHandle& n );

    public:
    double current_time = -1;

    std::mutex m_posegraph_buf;
    queue< int > optimize_posegraph_buf;

    std::mutex m_loop_drift;
    std::mutex m_update_visualization;
    std::mutex m_keyframe_buf;
    std::mutex m_retrive_data_buf;

    int global_frame_cnt = 0;
    camera_model::CameraPtr m_camera;
    vector< int > erase_index;
    std_msgs::Header cur_header;
    Eigen::Vector3d relocalize_t{ Eigen::Vector3d( 0, 0, 0 ) };
    Eigen::Matrix3d relocalize_r{ Eigen::Matrix3d::Identity( ) };
};

#endif // ESTIMATORROS_H
