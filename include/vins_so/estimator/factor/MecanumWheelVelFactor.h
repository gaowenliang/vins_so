#ifndef MECANUMWHEELVELFACTOR_H
#define MECANUMWHEELVELFACTOR_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

class MecanumWheelVelFactor : public ceres::SizedCostFunction< 4, 7, 3 >
{
    public:
    MecanumWheelVelFactor( const Eigen::Vector4d& _wheel_vel, Eigen::Vector3d _gyr );

    virtual bool Evaluate( double const* const* parameters, double* residuals, double** jacobians ) const;

    void check( double** parameters ) {}

    public:
    Eigen::Vector4d wheel_vel;
    Eigen::Vector4d gyr_vel;
    Eigen::Vector3d gyr;
    Eigen::Matrix4Xd F;
    Eigen::Matrix4d sqrt_info;
};

#endif // MECANUMWHEELVELFACTOR_H
