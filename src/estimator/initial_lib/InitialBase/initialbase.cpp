#include "vins_so/estimator/initial_lib/InitialBase/initialbase.h"

using namespace slidewindow;

InitVio::Initial::Initial( int window_size, int camera_size )
: m_windowSize( window_size )
, m_exParamSet( false )
, m_done( false )
{
    SlideWindowPosePtr pwindow( new SlideWindowPose( window_size, camera_size ) );
    m_window = pwindow;

    m_exParams.resize( camera_size );
    m_Bgs.resize( m_windowSize + 1 );

    for ( int window_index = 0; window_index < ( m_windowSize + 1 ); window_index++ )
    {
        m_Bgs[window_index].setZero( );
    }
}

InitVio::Initial::~Initial( )
{
    m_imageFrameAll.clear( );
    m_exParams.clear( );
    m_Bgs.clear( );

    m_tmpPreIntegration.reset( );
    m_window.reset( );
}

void
InitVio::Initial::setEx( const std::vector< Matrix3d >& Ric, const std::vector< Vector3d >& Tic )
{
    bool isSet       = true;
    int camera_index = -1;
    for ( auto& ex : m_exParams )
    {
        ++camera_index;
        ex.set( Ric[camera_index], Tic[camera_index] );
        isSet &= ex.isSet( );
    }

    m_exParamSet = isSet;
}

void
InitVio::Initial::setEx( const std::vector< exParam >& ex_ic_in )
{
    bool isSet       = true;
    int camera_index = -1;
    for ( auto& ex : m_exParams )
    {
        ++camera_index;
        ex.set( ex_ic_in[camera_index] );
        isSet &= ex.isSet( );
    }
    m_exParamSet = isSet;
}

void
InitVio::Initial::setEx( const exParam& ex_ic_in, int camera_index )
{
    m_exParams[camera_index].set( ex_ic_in );
}

bool
InitVio::Initial::checkEx( )
{
    bool isSet = true;
    for ( auto& ex : m_exParams )
    {
        isSet &= ex.isSet( );
    }
    m_exParamSet = isSet;
    return m_exParamSet;
}

void
InitVio::Initial::setCameraIndex( int cameraIndex )
{
    m_cameraIndex = cameraIndex;
}

void
InitVio::Initial::addCameraIndex( int cameraIndex )
{
    m_cameraIndex2 = cameraIndex;
}

void
InitVio::Initial::clear( )
{
    m_window->clearWindow( );

    for ( int window_index = 0; window_index < ( m_windowSize + 1 ); window_index++ )
        m_Bgs[window_index].setZero( );

    m_tmpPreIntegration.reset( );
}

void
InitVio::Initial::pushImu( const double dt, const Vector3d& acc, const Vector3d& gyr )
{
    m_tmpPreIntegration->push_back( dt, acc, gyr );
}

void
InitVio::Initial::resetImu( const Vector3d& _acc_0,
                            const Vector3d& _gyr_0,
                            const Vector3d& _linearized_ba,
                            const Vector3d& _linearized_bg )
{
    m_tmpPreIntegration = ImuPreintPtr( new ImuPreint( _acc_0,
                                                       _gyr_0, //
                                                       _linearized_ba,
                                                       _linearized_bg ) );
}
