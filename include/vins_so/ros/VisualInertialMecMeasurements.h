#ifndef VisualInertialMecMeasurements_H
#define VisualInertialMecMeasurements_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <wheel_msgs/wheelSpeeds.h>

struct vim_meas
{
    typedef std::vector< sensor_msgs::ImuConstPtr > imu_type;
    typedef sensor_msgs::PointCloudConstPtr feature_type;
    typedef wheel_msgs::wheelSpeedsConstPtr mecwheel_type;

    vim_meas( )
    : imu( )
    , feature( )
    , wheel( )
    {
    }

    vim_meas( const imu_type& __a, const feature_type& __b, const mecwheel_type& __c )
    : imu( __a )
    , feature( __b )
    , wheel( __c )
    {
    }

    imu_type imu;
    feature_type feature;
    mecwheel_type wheel;
};

typedef std::vector< vim_meas > VisualInertialMecMeasurements;

#endif // VisualInertialMecMeasurements_H
