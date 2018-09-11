#include "vins_so/ros/ros_io.h"

EstimateIOROS::EstimateIOROS( ros::NodeHandle& n )
: ParamROS( n )
, EstimateInputROS( n )
, EstimateOutputROS( n )
{
}

EstimateIOROS::EstimateIOROS( ros::NodeHandle& n, //
                              string imu_topic_name,
                              string feature_topic_name )
: ParamROS( n )
, EstimateInputROS( n )
, EstimateOutputROS( n )
{
    InitSubscribe( n, imu_topic_name, feature_topic_name );
}

EstimateIOROS::EstimateIOROS( ros::NodeHandle& n, //
                              string imu_topic_name,
                              string feature_topic_name,
                              string wheel_topic_name )
: ParamROS( n )
, EstimateInputROS( n )
, EstimateOutputROS( n )
{
    InitSubscribe( n, imu_topic_name, feature_topic_name, wheel_topic_name );
}
