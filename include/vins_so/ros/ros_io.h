#ifndef ROS_IO_H
#define ROS_IO_H

#include "EstimateInputROS.h"
#include "EstimateOutputROS.h"
#include <ros/ros.h>

class EstimateIOROS
: public EstimateInputROS
, public EstimateOutputROS
{
    public:
    EstimateIOROS( ros::NodeHandle& n );

    EstimateIOROS( ros::NodeHandle& n, //
                   std::string imu_topic_name,
                   std::string feature_topic_name );
};

#endif // ROS_IO_H
