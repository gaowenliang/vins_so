#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>

struct vi_meas
{
    typedef std::vector< sensor_msgs::ImuConstPtr > imu_type;
    typedef sensor_msgs::PointCloudConstPtr feature_type;

    vi_meas( )
    : imu( )
    , feature( )
    {
    }

    vi_meas( const imu_type& __a, const feature_type& __b )
    : imu( __a )
    , feature( __b )
    {
    }

    imu_type imu;
    feature_type feature;
};

typedef std::vector< vi_meas > VisualInertialMeasurements;

#endif // MEASUREMENTS_H
