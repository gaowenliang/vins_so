#include "vins_so/estimator/factor/GpsSinglePointFactor.h"
#include "vins_so/utility/utility.h"

GpsSinglePointFactor::GpsSinglePointFactor( const Eigen::Vector3d& _p_gps )
: p_gps( _p_gps )
{
}

bool
GpsSinglePointFactor::Evaluate( const double* const* parameters, double* residuals, double** jacobians ) const
{
    Eigen::Vector3d P_wb( parameters[0][0], parameters[0][1], parameters[0][2] );
    Eigen::Quaterniond Q_wb( parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5] );

    Eigen::Quaterniond Q_gw( parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2] );

    Eigen::Vector3d P_b_b2( parameters[2][0], parameters[2][1], parameters[2][2] );

    Eigen::Map< Eigen::Vector3d > residual( residuals );

    residual = p_gps - Q_gw * ( P_wb + Q_wb * P_b_b2 );

    if ( jacobians )
    {
        Eigen::Matrix3d R_gw = Q_gw.toRotationMatrix( );
        Eigen::Matrix3d R_wb = Q_wb.toRotationMatrix( );

        if ( jacobians[0] )
        {
            Eigen::Map< Eigen::Matrix< double, 3, 7, Eigen::RowMajor > > jacobian_pose_i( jacobians[0] );

            Eigen::Matrix< double, 3, 6 > jaco_i;
            jaco_i.leftCols< 3 >( )  = -R_gw;
            jaco_i.rightCols< 3 >( ) = R_gw * R_wb * -Utility::skewSymm( P_b_b2 );

            jacobian_pose_i.block< 2, 6 >( 0, 0 ) = jaco_i;
            jacobian_pose_i.block< 2, 1 >( 0, 6 ).setZero( );
        }
        if ( jacobians[1] )
        {
            Eigen::Map< Eigen::Matrix< double, 3, 4, Eigen::RowMajor > > jacobian_g( jacobians[1] );

            jacobian_g.block< 3, 3 >( 0, 0 ) = -R_gw;
            jacobian_g.block< 3, 1 >( 0, 3 ).setZero( );
        }
        if ( jacobians[2] )
        {
            Eigen::Map< Eigen::Matrix< double, 3, 7, Eigen::RowMajor > > jacobian_ex( jacobians[2] );

            jacobian_ex.block< 3, 3 >( 0, 0 ) = -R_gw * R_wb;
        }
    }
}
