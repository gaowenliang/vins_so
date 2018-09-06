#ifndef INERTIALPREDICT_H
#define INERTIALPREDICT_H

#include "../utility/utility.h"
#include "estimator.h"
#include <eigen3/Eigen/Eigen>

class InertialPredict
{
    public:
    InertialPredict( ) {}

    void predict( double t, //
                  double acc_x,
                  double acc_y,
                  double acc_z,
                  double gyr_x,
                  double gyr_y,
                  double gyr_z );

    public:
    Estimator::SolverFlag flag;
    double latest_time;
    Eigen::Vector3d m_P;
    Eigen::Quaterniond m_Q;
    Eigen::Vector3d m_V;
    Eigen::Vector3d m_Ba;
    Eigen::Vector3d m_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    Eigen::Vector3d m_g;
};

#endif // INERTIALPREDICT_H
