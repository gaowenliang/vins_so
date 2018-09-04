#ifndef GPSSINGLEPOINTFACTOR_H
#define GPSSINGLEPOINTFACTOR_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

class GpsSinglePointFactor : public ceres::SizedCostFunction< 3, 7, 4, 3 >
{
    public:
    GpsSinglePointFactor( const Eigen::Vector3d& _p_gps );

    virtual bool Evaluate( double const* const* parameters, double* residuals, double** jacobians ) const;

    void check( double** parameters ) {}

    public:
    Eigen::Vector3d p_gps;
};

#endif // GPSSINGLEPOINTFACTOR_H
