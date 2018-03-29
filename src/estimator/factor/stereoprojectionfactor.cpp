#include "vins_so/estimator/factor/stereoprojectionfactor.h"

Eigen::Matrix2d StereoProjectionFactor::sqrt_info;
double StereoProjectionFactor::sum_t;

StereoProjectionFactor::StereoProjectionFactor( const Eigen::Vector3d& _pts_i,
                                                const bool _is_stereo,
                                                const Eigen::Vector3d _trl,
                                                const Eigen::Matrix3d _rrl,
                                                const Eigen::Matrix3d _ricl,
                                                const Eigen::Vector3d _ticl )
: pts_i( _pts_i )
, ricl( _ricl )
, ticl( _ticl )
, is_stereo( _is_stereo )
{
    if ( is_stereo )
    {
        trl = _trl;
        rrl = _rrl;
    }
    else
    {
        trl = Eigen::Vector3d( 0, 0, 0 );
        rrl = Eigen::Matrix3d::Identity( );
    }
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_i.normalized( );
    Eigen::Vector3d tmp( 0, 0, 1 );
    if ( a == tmp )
        tmp << 1, 0, 0;
    b1 = ( tmp - a * ( a.transpose( ) * tmp ) ).normalized( );
    b2 = a.cross( b1 );
    tangent_base.block< 1, 3 >( 0, 0 ) = b1.transpose( );
    tangent_base.block< 1, 3 >( 1, 0 ) = b2.transpose( );
#endif
}

bool
StereoProjectionFactor::Evaluate( const double* const* parameters, double* residuals, double** jacobians ) const
{
    TicToc tic_toc;
    // pose for left camera
    Eigen::Vector3d Pi( parameters[0][0], parameters[0][1], parameters[0][2] );
    Eigen::Quaterniond Qi( parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5] );

    Eigen::Vector3d feature( parameters[1][0], parameters[1][1], parameters[1][2] );

    Eigen::Vector3d pts_imu = Qi.inverse( ) * ( feature - Pi );
    Eigen::Vector3d pts_cl  = ricl.transpose( ) * ( pts_imu - ticl );
    Eigen::Vector3d pts_cr  = rrl * pts_cl + trl;
    Eigen::Map< Eigen::Vector2d > residual( residuals );

#ifdef UNIT_SPHERE_ERROR
    residual = tangent_base * ( pts_cr.normalized( ) - pts_i.normalized( ) );
#else
    double dep = pts_cr.z( );
    residual   = ( pts_cr / dep ).head< 2 >( ) - pts_i.head< 2 >( );
#endif
    residual = sqrt_info * residual;

    if ( jacobians )
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix( );

        Eigen::Matrix< double, 2, 3 > reduce( 2, 3 );
#ifdef UNIT_SPHERE_ERROR
        double norm = pts_cr.norm( );
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = pts_cr( 0 );
        x2 = pts_cr( 1 );
        x3 = pts_cr( 2 );
        // clang-format off
    norm_jaco <<
                 1.0 / norm - x1 * x1 / pow( norm, 3 ), -x1 * x2 / pow( norm, 3 ), -x1 * x3 / pow( norm, 3 ),
        -x1 * x2 / pow( norm, 3 ), 1.0 / norm - x2 * x2 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ),
        -x1 * x3 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ), 1.0 / norm - x3 * x3 / pow( norm, 3 );
        // clang-format on
        reduce = tangent_base * norm_jaco;
#else
        reduce << 1. / dep, 0, -pts_cr( 0 ) / ( dep * dep ), 0, 1. / dep, -pts_cr( 1 ) / ( dep * dep );
#endif

        reduce = sqrt_info * reduce;

        if ( jacobians[0] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_pose_i( jacobians[0] );

            Eigen::Matrix< double, 3, 6 > jaco_i;
            jaco_i.leftCols< 3 >( ) = -rrl * ricl.transpose( ) * Ri.transpose( );
            jaco_i.rightCols< 3 >( ) = rrl * ricl.transpose( ) * Utility::skewSymm( pts_imu );

            jacobian_pose_i.leftCols< 6 >( ) = reduce * jaco_i;
            jacobian_pose_i.rightCols< 1 >( ).setZero( );
        }
        if ( jacobians[1] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 3, Eigen::RowMajor > > jacobian_feature( jacobians[1] );
            jacobian_feature = reduce * rrl * ricl.transpose( ) * Ri.transpose( );
        }
    }
    sum_t += tic_toc.toc( );

    return true;
}
