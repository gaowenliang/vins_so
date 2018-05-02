#include "vins_so/estimator/factor/ProjectionFactorSingleCam.h"

double ProjectionFactorSingleCam::sum_t;

ProjectionFactorSingleCam::ProjectionFactorSingleCam( const Eigen::Vector3d& _pts_i,
                                                      const Eigen::Vector3d& _pts_j,
                                                      const double& err )
: pts_i( _pts_i )
, pts_j( _pts_j )
{
    sqrt_info = 1.5 / err * Eigen::Matrix2d::Identity( );

#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized( );
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
ProjectionFactorSingleCam::Evaluate( const double* const* parameters, double* residuals, double** jacobians ) const
{
    sys_utils::TicToc tic_toc;
    Eigen::Vector3d Pi( parameters[0][0], parameters[0][1], parameters[0][2] );
    Eigen::Quaterniond Qi( parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5] );

    Eigen::Vector3d Pj( parameters[1][0], parameters[1][1], parameters[1][2] );
    Eigen::Quaterniond Qj( parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5] );

    double inv_dep_i = parameters[2][0];

    Eigen::Vector3d tic( parameters[3][0], parameters[3][1], parameters[3][2] );
    Eigen::Quaterniond qic( parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5] );

    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i    = qic * pts_camera_i + tic;             // imu i
    Eigen::Vector3d pts_w        = Qi * pts_imu_i + Pi;                  // world
    Eigen::Vector3d pts_imu_j    = Qj.inverse( ) * ( pts_w - Pj );       // imu j
    Eigen::Vector3d pts_camera_j = qic.inverse( ) * ( pts_imu_j - tic ); // frame j

    Eigen::Map< Eigen::Vector2d > residual( residuals );

    residual = tangent_base * ( pts_camera_j.normalized( ) - pts_j );
    residual = sqrt_info * residual;

    if ( jacobians )
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix( );
        Eigen::Matrix3d Rj = Qj.toRotationMatrix( );

        Eigen::Matrix3d ric = qic.toRotationMatrix( );

        Eigen::Matrix< double, 2, 3 > reduce_j( 2, 3 );

#ifdef UNIT_SPHERE_ERROR
        Eigen::Matrix3d norm_jaco;
        double norm, x1, x2, x3;

        norm = pts_camera_j.norm( );
        x1   = pts_camera_j( 0 );
        x2   = pts_camera_j( 1 );
        x3   = pts_camera_j( 2 );
        // clang-format off
        norm_jaco <<
        1.0 / norm - x1 * x1 / pow( norm, 3 ), -x1 * x2 / pow( norm, 3 ), -x1 * x3 / pow( norm, 3 ),
        -x1 * x2 / pow( norm, 3 ), 1.0 / norm - x2 * x2 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ),
        -x1 * x3 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ), 1.0 / norm - x3 * x3 / pow( norm, 3 );
        // clang-format on

        reduce_j = tangent_base * norm_jaco;
#endif
        reduce_j = sqrt_info * reduce_j;

        if ( jacobians[0] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_pose_i( jacobians[0] );

            // clang-format off
            Eigen::Matrix< double, 3, 6 > jaco_i2j;
            jaco_i2j.leftCols< 3 >( )  = ric.transpose( ) * Rj.transpose( );
            jaco_i2j.rightCols< 3 >( ) = ric.transpose( ) * Rj.transpose( ) * Ri * - Utility::skewSymm( pts_imu_i );
            // clang-format on

            jacobian_pose_i.block< 2, 6 >( 0, 0 ) = reduce_j * jaco_i2j;
            jacobian_pose_i.block< 2, 1 >( 0, 6 ).setZero( );
        }
        if ( jacobians[1] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_pose_j( jacobians[1] );

            // clang-format off
            Eigen::Matrix< double, 3, 6 > jaco_j;
            jaco_j.leftCols< 3 >( )  = ric.transpose( ) * -Rj.transpose( );
            jaco_j.rightCols< 3 >( ) = ric.transpose( ) * Utility::skewSymm( pts_imu_j );
            // clang-format on

            jacobian_pose_j.block< 2, 6 >( 0, 0 ) = reduce_j * jaco_j;
            jacobian_pose_j.block< 2, 1 >( 0, 6 ).setZero( );
        }
        if ( jacobians[2] )
        {
            Eigen::Map< Eigen::Vector2d > jacobian_feature( jacobians[2] );

            jacobian_feature.block< 2, 1 >( 0, 0 ) = reduce_j * ric.transpose( )
                                                     * Rj.transpose( ) * Ri * ric * pts_i
                                                     * -1.0 / ( inv_dep_i * inv_dep_i );
        }
        if ( jacobians[3] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_ex_pose( jacobians[3] );

            Eigen::Matrix3d tmp_r_l = ric.transpose( ) * Rj.transpose( ) * Ri * ric;
            // clang-format off
            Eigen::Matrix< double, 3, 6 > jaco_ex;

            jaco_ex.leftCols< 3 >( )
              = ric.transpose( ) * ( Rj.transpose( ) * Ri - Eigen::Matrix3d::Identity( ) );
            jaco_ex.rightCols< 3 >( )
              = -tmp_r_l * Utility::skewSymm( pts_camera_i )
                + Utility::skewSymm( tmp_r_l * pts_camera_i )
                + Utility::skewSymm( ric.transpose( ) * ( Rj.transpose( ) * ( Ri * tic + Pi - Pj ) - tic ) );
            // clang-format on

            jacobian_ex_pose.block< 2, 6 >( 0, 0 ) = reduce_j * jaco_ex;
            jacobian_ex_pose.block< 2, 1 >( 0, 6 ).setZero( );
        }
    }
    sum_t += tic_toc.toc( );

    return true;
}
