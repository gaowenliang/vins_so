#include "vins_so/ros/ros_io.h"

EstimateIOROS::EstimateIOROS( ros::NodeHandle& n )
: EstimateInputROS( n )
, EstimateOutputROS( n )
{
}

EstimateIOROS::EstimateIOROS( ros::NodeHandle& n, //
                              string imu_topic_name,
                              string feature_topic_name )
: EstimateInputROS( n )
, EstimateOutputROS( n )
{
    InitSubscribe( n, imu_topic_name, feature_topic_name );
}
