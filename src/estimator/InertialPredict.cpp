#include "vins_so/estimator/InertialPredict.h"

void
InertialPredict::predict( double t, double acc_x, double acc_y, double acc_z, double gyr_x, double gyr_y, double gyr_z )
{
    double dt   = t - latest_time;
    latest_time = t;

    Eigen::Vector3d linear_acceleration{ acc_x, acc_y, acc_z };
    Eigen::Vector3d angular_velocity{ gyr_x, gyr_y, gyr_z };

    Eigen::Vector3d un_acc_0 = m_Q * ( acc_0 - m_Ba - m_Q.inverse( ) * m_g );

    Eigen::Vector3d un_gyr = 0.5 * ( gyr_0 + angular_velocity ) - m_Bg;

    m_Q = m_Q * Utility::deltaQ( un_gyr * dt );

    Eigen::Vector3d un_acc_1 = m_Q * ( linear_acceleration - m_Ba - m_Q.inverse( ) * m_g );

    Eigen::Vector3d un_acc = 0.5 * ( un_acc_0 + un_acc_1 );

    m_P = m_P + dt * m_V + 0.5 * dt * dt * un_acc;
    m_V = m_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
