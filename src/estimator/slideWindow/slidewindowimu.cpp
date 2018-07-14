#include "vins_so/estimator/slideWindow/slidewindowimu.h"

using namespace slidewindow;

void
SlideWindowIMU::setG( const Vector3d& G )
{
    m_G = G;
}

bool
SlideWindowIMU::checkObservibility( )
{
    Vector3d sum_g;
    for ( auto& imu : m_preIntegrations )
    {
        double dt      = imu->sum_dt;
        Vector3d tmp_g = imu->delta_v / dt;
        sum_g += tmp_g;
    }
    Vector3d aver_g;
    aver_g     = sum_g * 1.0 / ( ( int )m_preIntegrations.size( ) - 1 );
    double var = 0;
    for ( auto& imu : m_preIntegrations )
    {
        double dt      = imu->sum_dt;
        Vector3d tmp_g = imu->delta_v / dt;
        var += ( tmp_g - aver_g ).transpose( ) * ( tmp_g - aver_g );
    }
    var = sqrt( var / ( ( int )m_preIntegrations.size( ) - 1 ) );

    if ( var < 0.25 )
    {
        ROS_INFO( "IMU excitation not enouth!" );
        return false;
    }
    else
        return true;
}

SlideWindowIMU&
SlideWindowIMU::operator=( const SlideWindowIMU& other )
{
    if ( this != &other )
    {
        m_preIntegrations = other.m_preIntegrations;
        m_Bas             = other.m_Bas;
        m_Bgs             = other.m_Bgs;
        m_accUn           = other.m_accUn;
        m_gyrUn           = other.m_gyrUn;
        m_accLast         = other.m_accLast;
        m_gyrLast         = other.m_gyrLast;
        m_G               = other.m_G;
        first_imu         = other.first_imu;
    }
    return *this;
}

SlideWindowIMU::SlideWindowIMU( int window_size )
: SlideWindowBase( window_size )
{
    m_preIntegrations.resize( WINDOW_SIZE + 1 );

    m_Bas.resize( WINDOW_SIZE + 1 );
    m_Bgs.resize( WINDOW_SIZE + 1 );

    for ( int i = 0; i < WINDOW_SIZE + 1; i++ )
    {
        Stamps[i] = 0.0;
        m_Bas[i].setZero( );
        m_Bgs[i].setZero( );
    }

    first_imu = false;
}

void
SlideWindowIMU::clearWindow( )
{
    for ( int window_index = 0; window_index < WINDOW_SIZE + 1; window_index++ )
    {
        m_Bas[window_index].setZero( );
        m_Bgs[window_index].setZero( );

        if ( m_preIntegrations[window_index] != nullptr )
            m_preIntegrations[window_index].reset( );
    }
    first_imu = false;
}

void
SlideWindowIMU::windowToDouble( double Bias[][6] )
{
    for ( int i = 0; i <= WINDOW_SIZE; i++ )
    {
        Bias[i][0] = m_Bas[i].x( );
        Bias[i][1] = m_Bas[i].y( );
        Bias[i][2] = m_Bas[i].z( );

        Bias[i][3] = m_Bgs[i].x( );
        Bias[i][4] = m_Bgs[i].y( );
        Bias[i][5] = m_Bgs[i].z( );
    }
}

void
SlideWindowIMU::slidewindow( )
{
    if ( m_margFlag == MARGIN_OLD )
    {
        for ( int i = 0; i < WINDOW_SIZE; i++ )
        {
            std::swap( m_preIntegrations[i], m_preIntegrations[i + 1] );
        }

        m_Bas[WINDOW_SIZE] = m_Bas[WINDOW_SIZE - 1];
        m_Bgs[WINDOW_SIZE] = m_Bgs[WINDOW_SIZE - 1];

        if ( m_preIntegrations[WINDOW_SIZE] != nullptr )
        {
            m_preIntegrations[WINDOW_SIZE].reset( );
        }
        ImuPreintPtr pImuPerint( new ImuPreint( m_accLast, //
                                                m_gyrLast,
                                                m_Bas[WINDOW_SIZE],
                                                m_Bgs[WINDOW_SIZE] ) );
        m_preIntegrations[WINDOW_SIZE] = pImuPerint;
    }
    else
    {
        for ( unsigned int i = 0; i < m_preIntegrations[WINDOW_SIZE]->dt_buf.size( ); i++ )
        {
            double tmp_dt   = m_preIntegrations[WINDOW_SIZE]->dt_buf[i];
            Vector3d tmpAcc = m_preIntegrations[WINDOW_SIZE]->acc_buf[i];
            Vector3d tmpGyr = m_preIntegrations[WINDOW_SIZE]->gyr_buf[i];

            m_preIntegrations[WINDOW_SIZE - 1]->push_back( tmp_dt, tmpAcc, tmpGyr );

            m_preIntegrations[WINDOW_SIZE - 1]->dt_buf.push_back( tmp_dt );
            m_preIntegrations[WINDOW_SIZE - 1]->acc_buf.push_back( tmpAcc );
            m_preIntegrations[WINDOW_SIZE - 1]->gyr_buf.push_back( tmpGyr );
        }

        m_Bas[WINDOW_SIZE - 1] = m_Bas[WINDOW_SIZE];
        m_Bgs[WINDOW_SIZE - 1] = m_Bgs[WINDOW_SIZE];

        m_preIntegrations[WINDOW_SIZE].reset( );

        ImuPreintPtr pImuPerint( new ImuPreint( m_accLast, //
                                                m_gyrLast,
                                                m_Bas[WINDOW_SIZE],
                                                m_Bgs[WINDOW_SIZE] ) );
        m_preIntegrations[WINDOW_SIZE] = pImuPerint;

        m_preIntegrations[WINDOW_SIZE]->dt_buf.clear( );
        m_preIntegrations[WINDOW_SIZE]->acc_buf.clear( );
        m_preIntegrations[WINDOW_SIZE]->gyr_buf.clear( );
    }
}

void
SlideWindowIMU::addImuInfo( double dt, int frame_count, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr )
{
    if ( !first_imu )
    {
        first_imu = true;
        m_accLast = acc;
        m_gyrLast = gyr;
    }

    if ( m_preIntegrations[frame_count] == nullptr )
    {
        ImuPreintPtr pImuPerint( new IntegrationBase( m_accLast,
                                                      m_gyrLast,
                                                      m_Bas[frame_count], //
                                                      m_Bgs[frame_count] ) );

        m_preIntegrations[frame_count] = pImuPerint;
    }

    if ( frame_count != 0 )
    {
        m_preIntegrations[frame_count]->push_back( dt, acc, gyr );
    }
}

void
SlideWindowIMU::predictWindow( Tf& _tf,
                               Vector3d& _vel,
                               double dt, //
                               int frame_count,
                               const Vector3d& acc,
                               const Vector3d& gyr )
{

    Vector3d un_gyr = 0.5 * ( m_gyrLast + gyr ) - m_Bgs[frame_count];

    Vector3d un_acc_0 = _tf.R * ( m_accLast - m_Bas[frame_count] ) - m_G;
    Vector3d un_acc_1 = _tf.R * ( acc - m_Bas[frame_count] ) - m_G;
    Vector3d un_acc   = 0.5 * ( un_acc_0 + un_acc_1 );

    _tf.R *= Utility::deltaQ( un_gyr * dt ).toRotationMatrix( );
    _tf.T += dt * _vel + 0.5 * dt * dt * un_acc;
    _vel += dt * un_acc;

    m_accLast = acc;
    m_gyrLast = gyr;
}

void
SlideWindowIMU::setPreIntegrationsIndex( ImuPreintPtr preIntegration, int index )
{
    m_preIntegrations[index] = preIntegration;
}
