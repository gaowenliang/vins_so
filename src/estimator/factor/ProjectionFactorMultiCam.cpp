#include "vins_so/estimator/factor/ProjectionFactorMultiCam.h"

double ProjectionFactorMultiCam::sum_t;

ProjectionFactorMultiCam::ProjectionFactorMultiCam( const Eigen::Vector3d& _pts_i,
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
ProjectionFactorMultiCam::Evaluate( const double* const* parameters, double* residuals, double** jacobians ) const
{
    sys_utils::tic::TicToc tic_toc;
    Eigen::Vector3d Pi( parameters[0][0], parameters[0][1], parameters[0][2] );
    Eigen::Quaterniond Qi( parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5] );

    Eigen::Vector3d Pj( parameters[1][0], parameters[1][1], parameters[1][2] );
    Eigen::Quaterniond Qj( parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5] );

    double inv_dep_i = parameters[2][0];

    Eigen::Vector3d tic1( parameters[3][0], parameters[3][1], parameters[3][2] );
    Eigen::Quaterniond qic1( parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5] );

    Eigen::Vector3d tic2( parameters[4][0], parameters[4][1], parameters[4][2] );
    Eigen::Quaterniond qic2( parameters[4][6], parameters[4][3], parameters[4][4], parameters[4][5] );

    Eigen::Vector3d pts_camera_i1 = pts_i / inv_dep_i;                      // cam 1 frame i
    Eigen::Vector3d pts_imu_i     = qic1 * pts_camera_i1 + tic1;            // imu i
    Eigen::Vector3d pts_w         = Qi * pts_imu_i + Pi;                    // world
    Eigen::Vector3d pts_imu_j     = Qj.inverse( ) * ( pts_w - Pj );         // imu j
    Eigen::Vector3d pts_camera_j2 = qic2.inverse( ) * ( pts_imu_j - tic2 ); // cam 2 frame j

    Eigen::Map< Eigen::Vector2d > residual( residuals );

    residual = tangent_base * ( pts_camera_j2.normalized( ) - pts_j );
    residual = sqrt_info * residual;

    if ( jacobians )
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix( );
        Eigen::Matrix3d Rj = Qj.toRotationMatrix( );

        Eigen::Matrix3d ric1 = qic1.toRotationMatrix( );
        Eigen::Matrix3d ric2 = qic2.toRotationMatrix( );

        Eigen::Matrix< double, 2, 3 > reduce_j( 2, 3 );

#ifdef UNIT_SPHERE_ERROR
        Eigen::Matrix3d norm_jaco;
        double norm, x1, x2, x3;

        norm = pts_camera_j2.norm( );
        x1   = pts_camera_j2( 0 );
        x2   = pts_camera_j2( 1 );
        x3   = pts_camera_j2( 2 );
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
            jaco_i2j.leftCols< 3 >( )   = ric2.transpose( ) * Rj.transpose( );
            jaco_i2j.rightCols< 3 >( )  = ric2.transpose( ) * Rj.transpose( ) * Ri * - Utility::skewSymm( pts_imu_i );
            // clang-format on

            jacobian_pose_i.block< 2, 6 >( 0, 0 ) = reduce_j * jaco_i2j;
            jacobian_pose_i.block< 2, 1 >( 0, 6 ).setZero( );
        }
        if ( jacobians[1] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_pose_j( jacobians[1] );

            // clang-format off
            Eigen::Matrix< double, 3, 6 > jaco_j2;
            jaco_j2.leftCols< 3 >( )  = ric2.transpose( ) * -Rj.transpose( );
            jaco_j2.rightCols< 3 >( ) = ric2.transpose( ) * Utility::skewSymm( pts_imu_j ); // pts_imu_jr
            // clang-format on

            jacobian_pose_j.block< 2, 6 >( 0, 0 ) = reduce_j * jaco_j2;
            jacobian_pose_j.block< 2, 1 >( 0, 6 ).setZero( );
        }
        if ( jacobians[2] )
        {
            Eigen::Map< Eigen::Vector2d > jacobian_feature( jacobians[2] );

            jacobian_feature.block< 2, 1 >( 0, 0 ) = reduce_j * ric2.transpose( )
                                                     * Rj.transpose( ) * Ri * ric1 * pts_i
                                                     * -1.0 / ( inv_dep_i * inv_dep_i );
        }
        if ( jacobians[3] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_ex_pose( jacobians[3] );

            Eigen::Matrix< double, 3, 6 > jaco_ex1;

            jaco_ex1.leftCols< 3 >( )  = ric2.transpose( ) * Rj.transpose( ) * Ri;
            jaco_ex1.rightCols< 3 >( ) = ric2.transpose( ) * Rj.transpose( ) * Ri * ric1
                                         * -Utility::skewSymm( pts_camera_i1 );

            jacobian_ex_pose.block< 2, 6 >( 0, 0 ) = reduce_j * jaco_ex1;
            jacobian_ex_pose.block< 2, 1 >( 0, 6 ).setZero( );
        }
        if ( jacobians[4] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_ex_pose( jacobians[4] );

            Eigen::Matrix< double, 3, 6 > jaco_ex2;

            jaco_ex2.leftCols< 3 >( )  = -ric2.transpose( );
            jaco_ex2.rightCols< 3 >( ) = Utility::skewSymm( pts_camera_j2 );

            jacobian_ex_pose.block< 2, 6 >( 0, 0 ) = reduce_j * jaco_ex2;
            jacobian_ex_pose.block< 2, 1 >( 0, 6 ).setZero( );
        }
    }
    sum_t += tic_toc.toc( );

    return true;
}
