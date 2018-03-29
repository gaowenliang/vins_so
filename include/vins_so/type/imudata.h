#ifndef IMUDATA_H
#define IMUDATA_H

#include <eigen3/Eigen/Eigen>

struct ImuData
{
    double t;
    Eigen::Vector3d gyr;
    Eigen::Vector3d acc;
};

struct ImuMagData
{
    double t;
    Eigen::Vector3d gyr;
    Eigen::Vector3d acc;
    Eigen::Vector3d mag;
};

struct ImuCov
{
    double acc_n, acc_w;
    double gyr_n, gyr_w;
};

#endif // IMUDATA_H
