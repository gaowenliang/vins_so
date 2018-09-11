#ifndef ESTIMATEINPUTROS_H
#define ESTIMATEINPUTROS_H

#include "../estimator/InertialPredict.h"
#include "VisualInertialMeasurements.h"
#include "VisualInertialMecMeasurements.h"
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <vins_so/estimator/estimator.h>
#include <wheel_msgs/wheelSpeeds.h>

class EstimateInputROS
{
    public:
    EstimateInputROS( ros::NodeHandle& n );

    void InitSubscribe( ros::NodeHandle& n, //
                        std::string imu_topic_name,
                        std::string feature_topic_name );
    void InitSubscribe( ros::NodeHandle& n, //
                        std::string imu_topic_name,
                        std::string feature_topic_name,
                        std::vector< std::string > image_topics );
    void InitSubscribe( ros::NodeHandle& n, //
                        std::string imu_topic_name,
                        std::string feature_topic_name,
                        std::string wheel_topic_name );

    VisualInertialMeasurements getVisualInertialMeasurements( );
    VisualInertialMecMeasurements getVisualInertialMecMeasurements( );

    void pubLatestOdometry( const Eigen::Vector3d& P,
                            const Eigen::Quaterniond& Q,
                            const Eigen::Vector3d& V,
                            const std_msgs::Header& header );

    void imu_callback( const sensor_msgs::ImuConstPtr& imu_msg );
    void wheel_mec_callback( const wheel_msgs::wheelSpeedsConstPtr& msg );
    void feature_callback( const sensor_msgs::PointCloudConstPtr& feature_msg );
    void raw_image_callback( const sensor_msgs::ImageConstPtr& img_msg );

    public:
    std::queue< sensor_msgs::ImuConstPtr > imu_buf;
    std::queue< sensor_msgs::PointCloudConstPtr > feature_buf;
    std::queue< std::pair< cv::Mat, double > > image_buf;
    std::queue< wheel_msgs::wheelSpeedsConstPtr > wheel_buf;

    public:
    std::mutex m_state;
    std::mutex m_buf;
    std::mutex i_buf;
    std::condition_variable con;

    public:
    int sum_of_wait = 0;

    private:
    std::vector< ros::Subscriber > sub_images;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_image;
    ros::Subscriber sub_mecWheel;
    ros::Publisher pub_latest_odometry;

    public:
    InertialPredict imuPropagate;
};

#endif // ESTIMATEINPUTROS_H
