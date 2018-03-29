#include "vins_so/estimator/initial_lib/InitialEx/InitialExRotationCamCam.h"
#include "vins_so/utility/utility.h"

using namespace initialEx;

InitialExRotationCamCam::InitialExRotationCamCam( )
: done( false )
, m_frameCount( 0 )
{
}

InitialExRotationCamCam::InitialExRotationCamCam( const Matrix3d& Rc1c0, const Vector3d& Tc1c0 )
: done( false )
, m_frameCount( 0 )
{
    setRT( Rc1c0, Tc1c0 );
}

InitialExRotationCamCam::InitialExRotationCamCam( const Tf& tf_c1c0 )
: done( false )
, m_frameCount( 0 )
{
    setRT( tf_c1c0 );
}

bool
InitialExRotationCamCam::CalibrationExRotation( vector< pair< Vector3d, Vector3d > > corres, Tf& tf_cc_result )
{
    m_ptsCorres = corres;

    double error_pre = calcEpipolarError( );

    solveRelativeRT( );

    double error_end = calcEpipolarError( );

    done = error_end < error_pre ? true : false;

    std::cout << Tfc1c0( ) << "\n";

    if ( done )
        tf_cc_result = Tfc1c0( );

    return done;
}

bool
InitialExRotationCamCam::Done( ) const
{
    return done;
}

void
InitialExRotationCamCam::setAsDone( )
{
    done = true;
}

void
InitialExRotationCamCam::setRT( const Tf& tf_c1c0 )
{
    m_tf_c0c1 = tf_c1c0;
    m_tf_c1c0 = m_tf_c0c1.inverse( );
}

void
InitialExRotationCamCam::setRT( const Matrix3d& Rc1c0, const Vector3d& Tc1c0 )
{
    m_tf_c0c1.setRT( Rc1c0, Tc1c0 );
    m_tf_c1c0 = m_tf_c0c1.inverse( );
}

Tf
InitialExRotationCamCam::Tfc1c0( ) const
{
    return m_tf_c1c0;
}

Tf
InitialExRotationCamCam::Tfc0c1( ) const
{
    return m_tf_c0c1;
}

bool
InitialExRotationCamCam::solveRelativeRT( )
{
    Eigen::Vector3d T;
    Eigen::Quaterniond q;

    T = m_tf_c1c0.getT( ).normalized( );
    q = Eigen::Quaterniond( m_tf_c1c0.getR( ) );

    // initialize the params to something close to the gt
    double ext[] = { T[0], T[1], T[2], q.x( ), q.y( ), q.z( ), q.w( ) };

    ceres::Problem problem;

    for ( unsigned int i = 0; i < m_ptsCorres.size( ); ++i )
    {
        ceres::CostFunction* f = new ceres::AutoDiffCostFunction< EpipolarError, 1, 7 >(
        new EpipolarError( m_ptsCorres[i].first, //
                           m_ptsCorres[i].second ) );

        problem.AddResidualBlock( f, NULL /* squared loss */, ext );
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.logging_type                 = ceres::SILENT;
    options.trust_region_strategy_type   = ceres::DOGLEG;
    options.max_num_iterations           = 15;

    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    //    std::cout << summary.FullReport( ) << "\n";

    q.w( ) = ext[6];
    q.x( ) = ext[3];
    q.y( ) = ext[4];
    q.z( ) = ext[5];
    T << ext[0], ext[1], ext[2];

    m_tf_c1c0.setRT( q.normalized( ).toRotationMatrix( ), T.normalized( ) );
    m_tf_c0c1 = m_tf_c1c0.inverse( );
}

double
InitialExRotationCamCam::calcEpipolarError( )
{
    double sum_error = 0;

    for ( unsigned int i = 0; i < m_ptsCorres.size( ); ++i )
    {
        double error = EpipolarError( m_ptsCorres[i].first, //
                                      m_ptsCorres[i].second )
                       .getEpipolarError( m_tf_c1c0 );

        sum_error += error;
    }

    return sqrt( sum_error * sum_error ) / m_ptsCorres.size( );
}

InitialExRotationCamCam::EpipolarError::EpipolarError( const Vector3d& cam0_point, const Vector3d& cam1_point )
: cam0_point_( cam0_point )
, cam1_point_( cam1_point )
{
}

double
InitialExRotationCamCam::EpipolarError::getEpipolarError( const Tf& tf_c1c0 )
{
    return cam1_point_.transpose( ) * math_utils::vectorToSkew( tf_c1c0.T.normalized( ) )
           * tf_c1c0.R * cam0_point_;
}
