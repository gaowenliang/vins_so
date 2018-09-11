#include "vins_so/estimator/factor/MecanumWheelVelFactor.h"
#include "vins_so/utility/utility.h"

MecanumWheelVelFactor::MecanumWheelVelFactor( const Eigen::Vector4d& _wheel_vel, Eigen::Vector3d _gyr )
: wheel_vel( _wheel_vel )
, gyr( _gyr )
{
    sqrt_info = 1 * Eigen::Matrix4d::Identity( );

    double m_a = 0.307 / 2;
    double m_b = 0.420 / 2;

    gyr_vel <<                  //
    _gyr( 2 ) * ( m_a + m_b ),  //
    _gyr( 2 ) * -( m_a + m_b ), //
    _gyr( 2 ) * -( m_a + m_b ), //
    _gyr( 2 ) * ( m_a + m_b );

    F = Eigen::Matrix4Xd( 4, 3 );

    F << 1, 1, 0, //
    1, -1, 0,     //
    1, 1, 0,      //
    1, -1, 0;
}

bool
MecanumWheelVelFactor::Evaluate( const double* const* parameters, double* residuals, double** jacobians ) const
{
    Eigen::Vector3d P_wb( parameters[0][0], parameters[0][1], parameters[0][2] );
    Eigen::Quaterniond Q_wb( parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5] );

    Eigen::Vector3d V_w( parameters[1][0], parameters[1][1], parameters[1][2] );

    Eigen::Map< Eigen::Vector4d > residual( residuals );

    Eigen::Matrix3d R_wb = Q_wb.toRotationMatrix( );

    residual = wheel_vel - ( F * R_wb.transpose( ) * V_w + gyr_vel );
    residual = sqrt_info * residual;

    if ( jacobians )
    {
        if ( jacobians[0] )
        {
            Eigen::Map< Eigen::Matrix< double, 4, 7, Eigen::RowMajor > > jacobian_pose_i( jacobians[0] );

            Eigen::Matrix< double, 4, 3 > jaco_i;
            jaco_i = F * Utility::skewSymm( R_wb.transpose( ) * V_w );

            jacobian_pose_i.block< 4, 3 >( 0, 0 ).setZero( );
            jacobian_pose_i.block< 4, 3 >( 0, 3 ) = sqrt_info * jaco_i;
            jacobian_pose_i.block< 4, 1 >( 0, 6 ).setZero( );
        }
        if ( jacobians[1] )
        {
            Eigen::Map< Eigen::Matrix< double, 4, 3, Eigen::RowMajor > > jacobian_v( jacobians[1] );

            jacobian_v.block< 4, 3 >( 0, 0 ) = sqrt_info * -F * R_wb.transpose( );
        }
    }
    return true;
}
