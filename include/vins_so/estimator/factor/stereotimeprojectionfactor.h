#ifndef STEREOTIMEPROJECTIONFACTOR_H
#define STEREOTIMEPROJECTIONFACTOR_H

#include "vins_so/estimator/vins_parameters.h"
#include "vins_so/utility/tic_toc.h"
#include "vins_so/utility/utility.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ros/assert.h>

class StereoTimeProjectionFactor : public ceres::SizedCostFunction< 4, 7, 7, 1, 7, 7 >
{
    public:
    StereoTimeProjectionFactor( const Eigen::Vector3d& _pts_i,
                                const Eigen::Vector3d& _pts_j,
                                const Eigen::Vector3d& _pts_k );

    virtual bool Evaluate( double const* const* parameters, double* residuals, double** jacobians ) const;

    void check( double** parameters ) {}

    Eigen::Vector3d pts_i, pts_jl, pts_jr;
    Eigen::Matrix< double, 2, 3 > tangent_base_jl;
    Eigen::Matrix< double, 2, 3 > tangent_base_jr;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif // STEREOTIMEPROJECTIONFACTOR_H
