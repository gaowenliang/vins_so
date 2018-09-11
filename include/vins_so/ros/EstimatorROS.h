#ifndef ESTIMATORROS_H
#define ESTIMATORROS_H

#include "../estimator/estimator.h"
#include "../estimator/vins_parameters.h"
#include "ros_io.h"
#include <camera_model/camera_models/CameraFactory.h>
#include <mutex>
#include <thread>

class EstimatorROS
: public Estimator
, public EstimateIOROS
{
    public:
    EstimatorROS( ros::NodeHandle& n );

    void update( );
    void send_imu( const sensor_msgs::ImuConstPtr& imu_msg );

    // thread: visual-inertial odometry
    void process( );

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
        v_b   = correct_q.conjugate( ) * correct_v + v_add;

        std::cout << "v_add " << v_add.transpose( ) << "\n";
        double v0 = v_b.x( ) + v_b.y( ) + ( m_a + m_b ) * gyr_z;
        double v1 = v_b.x( ) - v_b.y( ) - ( m_a + m_b ) * gyr_z;
        double v2 = v_b.x( ) + v_b.y( ) - ( m_a + m_b ) * gyr_z;
        double v3 = v_b.x( ) - v_b.y( ) + ( m_a + m_b ) * gyr_z;
        std::cout << "  " << v0 << " " << v1 << " " << v2 << " " << v3 << " "
                  << "\n";
    }

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
