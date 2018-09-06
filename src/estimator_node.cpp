#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#define IS_LOOP 0

#include "vins_so/ros/EstimatorROS.h"
#include <ros/ros.h>
#include <stdio.h>

EstimatorROS* estimator_ros;

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "vins_estimator" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    estimator_ros = new EstimatorROS( n );

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG( "EIGEN_DONT_PARALLELIZE" );
#endif
    ROS_WARN( "waiting for image and imu..." );

    std::thread measurement_process{ &EstimatorROS::process, estimator_ros };

    ros::spin( );

    return 0;
}
