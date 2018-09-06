#ifndef ROS_UTILITY_H
#define ROS_UTILITY_H

#include <ros/ros.h>

template< typename T >
T
readParam( ros::NodeHandle& n, std::string name )
{
    T ans;
    if ( n.getParam( name, ans ) )
    {
        ROS_INFO_STREAM( "Loaded " << name << ": " << ans );
    }
    else
    {
        ROS_ERROR_STREAM( "Failed to load " << name );
        n.shutdown( );
    }
    return ans;
}

#endif // ROS_UTILITY_H
