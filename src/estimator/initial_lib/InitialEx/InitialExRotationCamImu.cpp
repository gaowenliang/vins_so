#include "vins_so/estimator/initial_lib/InitialEx/InitialExRotationCamImu.h"
#include "vins_so/utility/utility.h"

using namespace initialEx;

InitialExRotationCamImu::InitialExRotationCamImu( )
{
    m_frameCount = 0;

    m_Ric = Matrix3d::Identity( );

    m_Rimu.push_back( Matrix3d::Identity( ) );
    m_Rc.push_back( Matrix3d::Identity( ) );
    m_Rcg.push_back( Matrix3d::Identity( ) );
}

bool
InitialExRotationCamImu::CalibrationExRotation( vector< pair< Vector3d, Vector3d > > corres,
                                                Quaterniond delta_q_imu,
                                                Matrix3d& Ric_result )
{
    m_frameCount++;

    m_Rimu.push_back( delta_q_imu.toRotationMatrix( ) );

    m_Rc.push_back( solveRelativeR( corres ) );
    m_Rcg.push_back( m_Ric.inverse( ) * delta_q_imu * m_Ric );

    Eigen::MatrixXd A( m_frameCount * 4, 4 );
    A.setZero( );
    int sum_ok = 0;
    for ( int i = 1; i <= m_frameCount; i++ )
    {
        Quaterniond r1( m_Rc[i] );
        Quaterniond r2( m_Rcg[i] );

        double angular_distance = 180 / M_PI * r1.angularDistance( r2 );
        ROS_DEBUG( "%d %f", i, angular_distance );

        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
        ++sum_ok;
        Matrix4d L, R;

        double w   = Quaterniond( m_Rc[i] ).w( );
        Vector3d q = Quaterniond( m_Rc[i] ).vec( );
        L.block< 3, 3 >( 0, 0 ) = w * Matrix3d::Identity( ) + Utility::skewSymm( q );
        L.block< 3, 1 >( 0, 3 ) = q;
        L.block< 1, 3 >( 3, 0 ) = -q.transpose( );
        L( 3, 3 ) = w;

        Quaterniond R_ij( m_Rimu[i] );
        w = R_ij.w( );
        q = R_ij.vec( );
        R.block< 3, 3 >( 0, 0 ) = w * Matrix3d::Identity( ) - Utility::skewSymm( q );
        R.block< 3, 1 >( 0, 3 ) = q;
        R.block< 1, 3 >( 3, 0 ) = -q.transpose( );
        R( 3, 3 ) = w;

        A.block< 4, 4 >( ( i - 1 ) * 4, 0 ) = huber * ( L - R );
    }

    JacobiSVD< MatrixXd > svd( A, ComputeFullU | ComputeFullV );
    Matrix< double, 4, 1 > x = svd.matrixV( ).col( 3 );
    Quaterniond estimated_R( x );
    m_Ric = estimated_R.toRotationMatrix( ).inverse( );
    // cout << svd.singularValues().transpose() << endl;
    // cout << ric << endl;
    Vector3d ric_cov;
    ric_cov = svd.singularValues( ).tail< 3 >( );

    // FIXME: WINDOW_SIZE=10
    if ( m_frameCount >= 10 && ric_cov( 1 ) > 0.25 )
    {
        Ric_result = m_Ric;
        return true;
    }
    else
        return false;
}

bool
InitialExRotationCamImu::Done( ) const
{
    return done;
}

void
InitialExRotationCamImu::setAsDone( )
{
    done = true;
}

void
InitialExRotationCamImu::setRic( const Matrix3d& Ric )
{
    m_Ric = Ric;
    done  = true;
}

Matrix3d
InitialExRotationCamImu::solveRelativeR( const vector< pair< Vector3d, Vector3d > >& corres )
{
    if ( corres.size( ) >= 9 )
    {
        vector< cv::Point2f > ll, rr;
        for ( int i = 0; i < int( corres.size( ) ); i++ )
        {
            ll.push_back( cv::Point2f( corres[i].first( 0 ), corres[i].first( 1 ) ) );
            rr.push_back( cv::Point2f( corres[i].second( 0 ), corres[i].second( 1 ) ) );
        }
        cv::Mat E = cv::findFundamentalMat( ll, rr );
        cv::Mat_< double > R1, R2, t1, t2;
        decomposeE( E, R1, R2, t1, t2 );

        if ( cv::determinant( R1 ) + 1.0 < 1e-09 )
        {
            E = -E;
            decomposeE( E, R1, R2, t1, t2 );
        }

        double ratio1 = max( testTriangulation( ll, rr, R1, t1 ), //
                             testTriangulation( ll, rr, R1, t2 ) );
        double ratio2 = max( testTriangulation( ll, rr, R2, t1 ), //
                             testTriangulation( ll, rr, R2, t2 ) );
        cv::Mat_< double > ans_R_cv = ratio1 > ratio2 ? R1 : R2;

        Matrix3d ans_R_eigen;
        for ( int i = 0; i < 3; i++ )
            for ( int j = 0; j < 3; j++ )
                ans_R_eigen( j, i ) = ans_R_cv( i, j );
        return ans_R_eigen;
    }
    return Matrix3d::Identity( );
}

double
InitialExRotationCamImu::testTriangulation( const vector< cv::Point2f >& l,
                                            const vector< cv::Point2f >& r,
                                            cv::Mat_< double > R,
                                            cv::Mat_< double > t )
{
    cv::Mat pointcloud;
    cv::Matx34f P  = cv::Matx34f( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0 );
    cv::Matx34f P1 = cv::Matx34f( R( 0, 0 ),
                                  R( 0, 1 ),
                                  R( 0, 2 ),
                                  t( 0 ),
                                  R( 1, 0 ),
                                  R( 1, 1 ),
                                  R( 1, 2 ),
                                  t( 1 ),
                                  R( 2, 0 ),
                                  R( 2, 1 ),
                                  R( 2, 2 ),
                                  t( 2 ) );
    cv::triangulatePoints( P, P1, l, r, pointcloud );
    int front_count = 0;
    for ( int i = 0; i < pointcloud.cols; i++ )
    {
        double normal_factor = pointcloud.col( i ).at< float >( 3 );

        cv::Mat_< double > p_3d_l = cv::Mat( P ) * ( pointcloud.col( i ) / normal_factor );
        cv::Mat_< double > p_3d_r = cv::Mat( P1 ) * ( pointcloud.col( i ) / normal_factor );
        if ( p_3d_l( 2 ) > 0 && p_3d_r( 2 ) > 0 )
            front_count++;
    }
    ROS_DEBUG( "MotionEstimator: %f", 1.0 * front_count / pointcloud.cols );
    return 1.0 * front_count / pointcloud.cols;
}

void
InitialExRotationCamImu::decomposeE( cv::Mat E,
                                     cv::Mat_< double >& R1,
                                     cv::Mat_< double >& R2,
                                     cv::Mat_< double >& t1,
                                     cv::Mat_< double >& t2 )
{
    cv::SVD svd( E, cv::SVD::MODIFY_A );
    cv::Matx33d W( 0, -1, 0, 1, 0, 0, 0, 0, 1 );
    cv::Matx33d Wt( 0, 1, 0, -1, 0, 0, 0, 0, 1 );
    R1 = svd.u * cv::Mat( W ) * svd.vt;
    R2 = svd.u * cv::Mat( Wt ) * svd.vt;
    t1 = svd.u.col( 2 );
    t2 = -svd.u.col( 2 );
}

Matrix3d
InitialExRotationCamImu::Ric( ) const
{
    return m_Ric;
}
