#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cstdlib>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;

struct SFMFeature
{
    bool state;
    int id;
    vector< pair< int, Vector3d > > observation;
    double position[3];
    double depth;
};

struct ReprojectionError3D
{
    ReprojectionError3D( double observed_u, double observed_v )
    : observed_u( observed_u )
    , observed_v( observed_v )
    {
    }

    template< typename T >
    bool operator( )( const T* const camera_R, const T* const camera_T, const T* point, T* residuals ) const
    {
        T p[3];
        ceres::QuaternionRotatePoint( camera_R, point, p );
        p[0] += camera_T[0];
        p[1] += camera_T[1];
        p[2] += camera_T[2];
        T xp         = p[0] / p[2];
        T yp         = p[1] / p[2];
        residuals[0] = xp - T( observed_u );
        residuals[1] = yp - T( observed_v );
        return true;
    }

    static ceres::CostFunction* Create( const double observed_x, const double observed_y )
    {
        return ( new ceres::AutoDiffCostFunction< ReprojectionError3D, 2, 4, 3, 3 >(
        new ReprojectionError3D( observed_x, observed_y ) ) );
    }

    double observed_u;
    double observed_v;
};

struct ReprojectionShpereError3D
{
    ReprojectionShpereError3D( double observed_x, double observed_y, double observed_z )
    : observed_x( observed_x )
    , observed_y( observed_y )
    , observed_z( observed_z )
    {
        observed_vector = Eigen::Vector3d( observed_x, observed_y, observed_z );
        observed_vector.normalize( );
        Eigen::Vector3d b1, b2;
        Eigen::Vector3d tmp( 0, 0, 1 );
        if ( observed_vector == tmp )
            tmp << 1, 0, 0;
        b1 = ( tmp - observed_vector * ( observed_vector.transpose( ) * tmp ) ).normalized( );
        b2 = observed_vector.cross( b1 );
        tangent_base.block< 1, 3 >( 0, 0 ) = b1.transpose( );
        tangent_base.block< 1, 3 >( 1, 0 ) = b2.transpose( );
    }

    template< typename T >
    bool operator( )( const T* const camera_R, const T* const camera_T, const T* point, T* residuals ) const
    {
        T p[3];
        ceres::QuaternionRotatePoint( camera_R, point, p );
        p[0] += camera_T[0];
        p[1] += camera_T[1];
        p[2] += camera_T[2];
        T norm = sqrt( p[0] * p[0] + p[1] * p[1] + p[2] * p[2] );
        T xp   = p[0] / norm;
        T yp   = p[1] / norm;
        T zp   = p[2] / norm;
        T r_x  = xp - T( observed_vector.x( ) );
        T r_y  = yp - T( observed_vector.y( ) );
        T r_z  = zp - T( observed_vector.z( ) );

        residuals[0] = T( tangent_base( 0, 0 ) ) * r_x + T( tangent_base( 0, 1 ) ) * r_y
                       + T( tangent_base( 0, 2 ) ) * r_z;
        residuals[1] = T( tangent_base( 1, 0 ) ) * r_x + T( tangent_base( 1, 1 ) ) * r_y
                       + T( tangent_base( 1, 2 ) ) * r_z;

        return true;
    }

    static ceres::CostFunction* Create( const double observed_x, const double observed_y, const double observed_z )
    {
        return ( new ceres::AutoDiffCostFunction< ReprojectionShpereError3D, 2, 4, 3, 3 >(
        new ReprojectionShpereError3D( observed_x, observed_y, observed_z ) ) );
    }

    double observed_x;
    double observed_y;
    double observed_z;
    Eigen::Vector3d observed_vector;
    Eigen::Matrix< double, 2, 3 > tangent_base;
};

class GlobalSFM
{
    public:
    GlobalSFM( );
    bool construct( int frame_num,
                    vector< Quaterniond >& q_wcs,
                    vector< Vector3d >& T_wcs,
                    int linkIndex,
                    const Matrix3d relative_R,
                    const Vector3d relative_T,
                    vector< SFMFeature >& sfm_f,
                    map< int, Vector3d >& sfm_tracked_points );

    private:
    bool solveFrameByPnP( Matrix3d& R_initial, Vector3d& P_initial, int i, vector< SFMFeature >& sfm_f );

    void triangulatePoint( Eigen::Matrix< double, 3, 4 >& Pose0,
                           Eigen::Matrix< double, 3, 4 >& Pose1,
                           Eigen::Vector3d& point0,
                           Eigen::Vector3d& point1,
                           Eigen::Vector3d& point_3d );
    void triangulateTwoFrames( int frame0,
                               Eigen::Matrix< double, 3, 4 >& Pose0,
                               int frame1,
                               Eigen::Matrix< double, 3, 4 >& Pose1,
                               vector< SFMFeature >& sfm_f );

    int feature_num;
};
