#ifndef STEREOPROJECTIONFACTOR_H
#define STEREOPROJECTIONFACTOR_H

#include "vins_so/utility/tic_toc.h"
#include "vins_so/utility/utility.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ros/assert.h>

#define UNIT_SPHERE_ERROR

class StereoProjectionFactor : public ceres::SizedCostFunction< 2, 7, 3 >
{
    public:
    StereoProjectionFactor( const Eigen::Vector3d& _pts_i,
                            const bool _is_stereo,
                            const Eigen::Vector3d _trl,
                            const Eigen::Matrix3d _rrl,
                            const Eigen::Matrix3d _ricl,
                            const Eigen::Vector3d _ticl );

    virtual bool Evaluate( double const* const* parameters, double* residuals, double** jacobians ) const;
    void check( double** parameters ) {}

    Eigen::Vector3d pts_i;
    Eigen::Matrix< double, 2, 3 > tangent_base;
    Eigen::Vector3d trl;
    Eigen::Matrix3d rrl;
    Eigen::Matrix3d ricl;
    Eigen::Vector3d ticl;
    bool is_stereo;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif // STEREOPROJECTIONFACTOR_H
