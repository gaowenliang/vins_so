#pragma once

#include "vins_so/utility/utility.h"
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus( const double* x, const double* delta, double* x_plus_delta ) const;
    virtual bool ComputeJacobian( const double* x, double* jacobian ) const;
    virtual int GlobalSize( ) const { return 7; }
    virtual int LocalSize( ) const { return 6; }
};
