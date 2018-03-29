#include "vins_so/estimator/factor/stereotimeprojectionfactor.h"

Eigen::Matrix2d StereoTimeProjectionFactor::sqrt_info;
double StereoTimeProjectionFactor::sum_t;

StereoTimeProjectionFactor::StereoTimeProjectionFactor( const Eigen::Vector3d& _pts_i,
                                                        const Eigen::Vector3d& _pts_jl,
                                                        const Eigen::Vector3d& _pts_jr )
: pts_i( _pts_i )   // start frame
, pts_jl( _pts_jl ) // this frame left camera
, pts_jr( _pts_jr ) // this frame right camera
{
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_i;
    Eigen::Vector3d tmp( 0, 0, 1 );
    if ( a == tmp )
        tmp << 1, 0, 0;
    b1 = ( tmp - a * ( a.transpose( ) * tmp ) ).normalized( );
    b2 = a.cross( b1 );
    tangent_base_jl.block< 1, 3 >( 0, 0 ) = b1.transpose( );
    tangent_base_jl.block< 1, 3 >( 1, 0 ) = b2.transpose( );

    tangent_base_jr = tangent_base_jl;
// a = pts_jr;
// tmp << 0, 0, 1;
// if ( a == tmp )
//     tmp << 1, 0, 0;
// b1 = ( tmp - a * ( a.transpose( ) * tmp ) ).normalized( );
// b2 = a.cross( b1 );
// tangent_base_jr.block< 1, 3 >( 0, 0 ) = b1.transpose( );
// tangent_base_jr.block< 1, 3 >( 1, 0 ) = b2.transpose( );

#endif
}

bool
StereoTimeProjectionFactor::Evaluate( const double* const* parameters, double* residuals, double** jacobians ) const
{
    TicToc tic_toc;
    Eigen::Vector3d Pi( parameters[0][0], parameters[0][1], parameters[0][2] );
    Eigen::Quaterniond Qi( parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5] );

    Eigen::Vector3d Pj( parameters[1][0], parameters[1][1], parameters[1][2] );
    Eigen::Quaterniond Qj( parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5] );

    double inv_dep_i = parameters[2][0];

    Eigen::Vector3d tic_l( parameters[3][0], parameters[3][1], parameters[3][2] );
    Eigen::Quaterniond qic_l( parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5] );

    Eigen::Vector3d tic_r( parameters[4][0], parameters[4][1], parameters[4][2] );
    Eigen::Quaterniond qic_r( parameters[4][6], parameters[4][3], parameters[4][4], parameters[4][5] );

    Eigen::Vector3d pts_camera_i  = pts_i / inv_dep_i;                        // left cam i
    Eigen::Vector3d pts_imu_i     = qic_l * pts_camera_i + tic_l;             // imu i
    Eigen::Vector3d pts_w         = Qi * pts_imu_i + Pi;                      // world
    Eigen::Vector3d pts_imu_j     = Qj.inverse( ) * ( pts_w - Pj );           // imu j
    Eigen::Vector3d pts_camera_jl = qic_l.inverse( ) * ( pts_imu_j - tic_l ); // left cam j
    Eigen::Vector3d pts_camera_jr = qic_r.inverse( ) * ( pts_imu_j - tic_r ); // right cam j

    Eigen::Map< Eigen::Vector4d > residual( residuals );

#ifdef UNIT_SPHERE_ERROR
    residual.block< 2, 1 >( 0, 0 ) = tangent_base_jl * ( pts_camera_jl.normalized( ) - pts_jl );
    residual.block< 2, 1 >( 2, 0 ) = tangent_base_jr * ( pts_camera_jr.normalized( ) - pts_jr );
#else
    double dep_jl            = pts_camera_jl.z( );
    residual.head< 2 >( )    = ( pts_camera_jl / dep_jl ).head< 2 >( ) - pts_jl.head< 2 >( );
#endif

    residual.head( 2 ) = sqrt_info * residual.head( 2 );
    residual.tail( 2 ) = sqrt_info * residual.tail( 2 );

    if ( jacobians )
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix( );
        Eigen::Matrix3d Rj = Qj.toRotationMatrix( );

        Eigen::Matrix3d ric_l = qic_l.toRotationMatrix( );
        Eigen::Matrix3d ric_r = qic_r.toRotationMatrix( );

        Eigen::Matrix< double, 2, 3 > reduce_jl( 2, 3 );
        Eigen::Matrix< double, 2, 3 > reduce_jr( 2, 3 );

#ifdef UNIT_SPHERE_ERROR
        Eigen::Matrix3d norm_jaco;
        double norm, x1, x2, x3;

        norm = pts_camera_jl.norm( );
        x1   = pts_camera_jl( 0 );
        x2   = pts_camera_jl( 1 );
        x3   = pts_camera_jl( 2 );
        // clang-format off
        norm_jaco <<
        1.0 / norm - x1 * x1 / pow( norm, 3 ), -x1 * x2 / pow( norm, 3 ), -x1 * x3 / pow( norm, 3 ),
        -x1 * x2 / pow( norm, 3 ), 1.0 / norm - x2 * x2 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ),
        -x1 * x3 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ), 1.0 / norm - x3 * x3 / pow( norm, 3 );
        // clang-format on

        reduce_jl = tangent_base_jl * norm_jaco;

        norm = pts_camera_jr.norm( );
        x1   = pts_camera_jr( 0 );
        x2   = pts_camera_jr( 1 );
        x3   = pts_camera_jr( 2 );
        // clang-format off
        norm_jaco <<
        1.0 / norm - x1 * x1 / pow( norm, 3 ), -x1 * x2 / pow( norm, 3 ), -x1 * x3 / pow( norm, 3 ),
        -x1 * x2 / pow( norm, 3 ), 1.0 / norm - x2 * x2 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ),
        -x1 * x3 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ), 1.0 / norm - x3 * x3 / pow( norm, 3 );
        // clang-format on

        reduce_jr = tangent_base_jr * norm_jaco;
#endif

        reduce_jl = sqrt_info * reduce_jl;
        reduce_jr = sqrt_info * reduce_jr;

        if ( jacobians[0] )
        {
            Eigen::Map< Eigen::Matrix< double, 4, 7, Eigen::RowMajor > > jacobian_pose_i( jacobians[0] );

            // clang-format off
            Eigen::Matrix< double, 3, 6 > jaco_i2jl;
            jaco_i2jl.leftCols< 3 >( )   = ric_l.transpose( ) * Rj.transpose( );
            jaco_i2jl.rightCols< 3 >( )  = ric_l.transpose( ) * Rj.transpose( ) * Ri * - Utility::skewSymm( pts_imu_i );

            Eigen::Matrix< double, 3, 6 > jaco_i2jr;
            jaco_i2jr.leftCols< 3 >( )   = ric_r.transpose( ) * Rj.transpose( );
            jaco_i2jr.rightCols< 3 >( )  = ric_r.transpose( ) * Rj.transpose( ) * Ri * - Utility::skewSymm( pts_imu_i );
            // clang-format on

            jacobian_pose_i.block< 2, 6 >( 0, 0 ) = reduce_jl * jaco_i2jl;
            jacobian_pose_i.block< 2, 1 >( 0, 6 ).setZero( );

            jacobian_pose_i.block< 2, 6 >( 2, 0 ) = reduce_jr * jaco_i2jr;
            jacobian_pose_i.block< 2, 1 >( 2, 6 ).setZero( );
        }
        if ( jacobians[1] )
        {
            Eigen::Map< Eigen::Matrix< double, 4, 7, Eigen::RowMajor > > jacobian_pose_j( jacobians[1] );

            // clang-format off
            Eigen::Matrix< double, 3, 6 > jaco_jl;
            jaco_jl.leftCols< 3 >( )  = ric_l.transpose( ) * -Rj.transpose( );
            jaco_jl.rightCols< 3 >( ) = ric_l.transpose( ) * Utility::skewSymm( pts_imu_j );

            Eigen::Matrix< double, 3, 6 > jaco_jr;
            jaco_jr.leftCols< 3 >( )  = ric_r.transpose( ) * -Rj.transpose( );
            jaco_jr.rightCols< 3 >( ) = ric_r.transpose( ) * Utility::skewSymm( pts_imu_j ); // pts_imu_jr
            // clang-format on

            jacobian_pose_j.block< 2, 6 >( 0, 0 ) = reduce_jl * jaco_jl;
            jacobian_pose_j.block< 2, 1 >( 0, 6 ).setZero( );

            jacobian_pose_j.block< 2, 6 >( 2, 0 ) = reduce_jr * jaco_jr;
            jacobian_pose_j.block< 2, 1 >( 2, 6 ).setZero( );
        }
        if ( jacobians[2] )
        {
            Eigen::Map< Eigen::Vector4d > jacobian_feature( jacobians[2] );

#if INV_DEPTH
            jacobian_feature.block< 2, 1 >( 0, 0 ) = reduce_jl * ric_l.transpose( )
                                                     * Rj.transpose( ) * Ri * ric_l * pts_i
                                                     * -1.0 / ( inv_dep_i * inv_dep_i );

            jacobian_feature.block< 2, 1 >( 2, 0 ) = reduce_jr * ric_r.transpose( )
                                                     * Rj.transpose( ) * Ri * ric_l * pts_i
                                                     * -1.0 / ( inv_dep_i * inv_dep_i );
#else
            jacobian_feature = reduce * ric.transpose( ) * Rj.transpose( ) * Ri * ric * pts_i;
#endif
        }
        if ( jacobians[3] )
        {
            Eigen::Map< Eigen::Matrix< double, 4, 7, Eigen::RowMajor > > jacobian_ex_pose( jacobians[3] );

            Eigen::Matrix3d tmp_r_l = ric_l.transpose( ) * Rj.transpose( ) * Ri * ric_l;
            // clang-format off
            Eigen::Matrix< double, 3, 6 > jaco_ex_l;

            jaco_ex_l.leftCols< 3 >( )  =
                ric_l.transpose( ) * ( Rj.transpose( ) * Ri - Eigen::Matrix3d::Identity( ) );
            jaco_ex_l.rightCols< 3 >( ) =
                - tmp_r_l * Utility::skewSymm( pts_camera_i )
                + Utility::skewSymm( tmp_r_l * pts_camera_i )
                + Utility::skewSymm( ric_l.transpose( ) * ( Rj.transpose( ) * ( Ri * tic_l + Pi - Pj ) - tic_l ) );
            // clang-format on

            Eigen::Matrix< double, 3, 6 > jaco_ex_r;

            jaco_ex_r.leftCols< 3 >( )  = ric_r.transpose( ) * Rj.transpose( ) * Ri;
            jaco_ex_r.rightCols< 3 >( ) = ric_r.transpose( ) * Rj.transpose( ) * Ri * ric_l
                                          * -Utility::skewSymm( pts_camera_i );

            jacobian_ex_pose.block< 2, 6 >( 0, 0 ) = reduce_jl * jaco_ex_l;
            jacobian_ex_pose.block< 2, 1 >( 0, 6 ).setZero( );

            jacobian_ex_pose.block< 2, 6 >( 2, 0 ) = reduce_jr * jaco_ex_r;
            jacobian_ex_pose.block< 2, 1 >( 2, 6 ).setZero( );
        }
        if ( jacobians[4] )
        {
            Eigen::Map< Eigen::Matrix< double, 4, 7, Eigen::RowMajor > > jacobian_ex_pose( jacobians[4] );

            Eigen::Matrix< double, 3, 6 > jaco_ex_r;

            jaco_ex_r.leftCols< 3 >( )  = -ric_r.transpose( );
            jaco_ex_r.rightCols< 3 >( ) = Utility::skewSymm( pts_camera_jr );

            jacobian_ex_pose.block< 2, 6 >( 0, 0 ).setZero( );
            jacobian_ex_pose.block< 2, 1 >( 0, 6 ).setZero( );

            jacobian_ex_pose.block< 2, 6 >( 2, 0 ) = reduce_jr * jaco_ex_r;
            jacobian_ex_pose.block< 2, 1 >( 2, 6 ).setZero( );
        }
    }
    sum_t += tic_toc.toc( );

    return true;
}
