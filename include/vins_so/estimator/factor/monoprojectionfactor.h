#ifndef MONOPROJECTIONFACTOR_H
#define MONOPROJECTIONFACTOR_H

#include "vins_so/estimator/vins_parameters.h"
#include "vins_so/utility/tic_toc.h"
#include "vins_so/utility/utility.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ros/assert.h>

//#define UNIT_SPHERE_ERROR
//#define INV_DEPTH 1

class MonoProjectionFactor : public ceres::SizedCostFunction< 2, 7, 7, 7, 1 >
{
    public:
    MonoProjectionFactor( const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j );
    virtual bool Evaluate( double const* const* parameters, double* residuals, double** jacobians ) const;
    void check( double** parameters );

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix< double, 2, 3 > tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif // MONOPROJECTIONFACTOR_H
