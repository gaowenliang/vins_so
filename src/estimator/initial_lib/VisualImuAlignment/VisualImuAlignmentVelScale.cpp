#include "vins_so/estimator/initial_lib/VisualImuAlignment/VisualImuAlignmentVelScale.h"
#include "vins_so/estimator/initial_lib/VisualImuAlignment/Solvegyroscopebias.h"

bool
InitVio::VisualImuAlignmentVelScale::solve( map< double, InitVio::ImageImuFrame >& imageFrames,
                                            std::vector< Vector3d >& Bgs,
                                            Vector3d& g,
                                            VectorXd& x,
                                            int align_camera_index )
{

    InitVio::SolveGyroscopeBias gyro_bias;
    if ( !gyro_bias.solve( imageFrames, Bgs ) )
        return false;

    if ( LinearAlignment( imageFrames, g, x, align_camera_index ) )
        return true;
    else
        return false;
}

bool
InitVio::VisualImuAlignmentVelScale::LinearAlignment(
map< double, InitVio::ImageImuFrame >& imageFrameAll, Vector3d& g, VectorXd& x, int camera_index )
{
    int all_frame_count = imageFrameAll.size( );
    int n_state         = all_frame_count * 3 + 3 + 1;

    MatrixXd A{ n_state, n_state };
    A.setZero( );
    VectorXd b{ n_state };
    b.setZero( );

    map< double, ImageImuFrame >::iterator frame_i;
    map< double, ImageImuFrame >::iterator frame_j;
    int i = 0;
    for ( frame_i = imageFrameAll.begin( ); next( frame_i ) != imageFrameAll.end( ); frame_i++, i++ )
    {
        frame_j = next( frame_i );

        MatrixXd tmp_A( 6, 10 );
        tmp_A.setZero( );
        VectorXd tmp_b( 6 );
        tmp_b.setZero( );

        double dt = frame_j->second.preIntegration->sum_dt;
        Tf tf_i   = frame_i->second.tf_wb;
        Tf tf_j   = frame_j->second.tf_wb;

        tmp_A.block< 3, 3 >( 0, 0 ) = -dt * Matrix3d::Identity( );
        tmp_A.block< 3, 3 >( 0, 6 ) = tf_i.R.transpose( ) * dt * dt / 2 * Matrix3d::Identity( );
        // tmp_A.block< 3, 3 >( 0, 3 ) .setZero( );
        tmp_A.block< 3, 1 >( 0, 9 ) = tf_i.R.transpose( ) * ( tf_j.T - tf_i.T ) / 100.0;
        tmp_b.block< 3, 1 >( 0, 0 ) = frame_j->second.preIntegration->delta_p
                                      + tf_i.R.transpose( ) * tf_j.R * TIC[camera_index]
                                      - TIC[camera_index];
        // cout << "delta_p   " << frame_j->second.preIntegration->delta_p.transpose() <<
        // endl;
        tmp_A.block< 3, 3 >( 3, 0 ) = -Matrix3d::Identity( );
        tmp_A.block< 3, 3 >( 3, 3 ) = tf_i.R.transpose( ) * tf_j.R;
        tmp_A.block< 3, 3 >( 3, 6 ) = tf_i.R.transpose( ) * dt * Matrix3d::Identity( );
        // tmp_A.block< 3, 1 >( 3, 9 ).setZero( );
        tmp_b.block< 3, 1 >( 3, 0 ) = frame_j->second.preIntegration->delta_v;
        // cout << "delta_v   " << frame_j->second.preIntegration->delta_v.transpose() <<
        // endl;

        Matrix< double, 6, 6 > cov_inv = Matrix< double, 6, 6 >::Zero( );
        // cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        // MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity( );

        MatrixXd r_A = tmp_A.transpose( ) * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose( ) * cov_inv * tmp_b;

        A.block< 6, 6 >( i * 3, i * 3 ) += r_A.topLeftCorner< 6, 6 >( );
        b.segment< 6 >( i * 3 ) += r_b.head< 6 >( );

        A.bottomRightCorner< 4, 4 >( ) += r_A.bottomRightCorner< 4, 4 >( );
        b.tail< 4 >( ) += r_b.tail< 4 >( );

        A.block< 6, 4 >( i * 3, n_state - 4 ) += r_A.topRightCorner< 6, 4 >( );
        A.block< 4, 6 >( n_state - 4, i * 3 ) += r_A.bottomLeftCorner< 4, 6 >( );
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt( ).solve( b );

    double s = x( n_state - 1 ) / 100.0;
    ROS_DEBUG( "estimated scale: %f", s );
    g = x.segment< 3 >( n_state - 4 );
    if ( g.hasNaN( ) )
    {
        ROS_DEBUG_STREAM( " solve g NaN." );
        return false;
    }
    ROS_DEBUG_STREAM( " result g     " << g.norm( ) << " " << g.transpose( ) );
    if ( fabs( g.norm( ) - G.norm( ) ) > 1.0 || s < 0 )
    {
        return false;
    }

    RefineGravity( imageFrameAll, g, x, camera_index );
    s = ( x.tail< 1 >( ) )( 0 ) / 100.0;

    ( x.tail< 1 >( ) )( 0 ) = s;
    ROS_DEBUG_STREAM( " refine     " << g.norm( ) << " " << g.transpose( ) );
    if ( s < 0.0 )
        return false;
    else
        return true;
}

void
InitVio::VisualImuAlignmentVelScale::RefineGravity( map< double, InitVio::ImageImuFrame >& all_image_frame,
                                                    Vector3d& g,
                                                    VectorXd& x_state,
                                                    int align_camera_index )
{
    Vector3d g0 = g.normalized( ) * G.norm( );

    // VectorXd x;
    int all_frame_count = all_image_frame.size( );
    int n_state         = all_frame_count * 3 + 2 + 1;

    MatrixXd A{ n_state, n_state };
    A.setZero( );
    VectorXd b{ n_state };
    b.setZero( );

    map< double, ImageImuFrame >::iterator frame_i;
    map< double, ImageImuFrame >::iterator frame_j;
    for ( int k = 0; k < 4; k++ )
    {
        MatrixXd lxly( 3, 2 );
        lxly  = TangentBasis( g0 );
        int i = 0;
        for ( frame_i = all_image_frame.begin( ); next( frame_i ) != all_image_frame.end( );
              frame_i++, i++ )
        {
            frame_j = next( frame_i );

            MatrixXd tmp_A( 6, 9 );
            tmp_A.setZero( );
            VectorXd tmp_b( 6 );
            tmp_b.setZero( );

            double dt = frame_j->second.preIntegration->sum_dt;
            Tf tf_i   = frame_i->second.tf_wb;
            Tf tf_j   = frame_j->second.tf_wb;

            tmp_A.block< 3, 3 >( 0, 0 ) = -dt * Matrix3d::Identity( );
            tmp_A.block< 3, 2 >( 0, 6 )
            = tf_i.R.transpose( ) * dt * dt / 2 * Matrix3d::Identity( ) * lxly;
            tmp_A.block< 3, 1 >( 0, 8 ) = tf_i.R.transpose( ) * ( tf_j.T - tf_i.T ) / 100.0;
            tmp_b.block< 3, 1 >( 0, 0 )
            = frame_j->second.preIntegration->delta_p
              + tf_i.R.transpose( ) * tf_j.R * TIC[align_camera_index]
              - TIC[align_camera_index] - tf_i.R.transpose( ) * dt * dt / 2 * g0;

            tmp_A.block< 3, 3 >( 3, 0 ) = -Matrix3d::Identity( );
            tmp_A.block< 3, 3 >( 3, 3 ) = tf_i.R.transpose( ) * tf_j.R;
            tmp_A.block< 3, 2 >( 3, 6 ) = tf_i.R.transpose( ) * dt * Matrix3d::Identity( ) * lxly;
            tmp_b.block< 3, 1 >( 3, 0 ) = frame_j->second.preIntegration->delta_v
                                          - tf_i.R.transpose( ) * dt * Matrix3d::Identity( ) * g0;

            Matrix< double, 6, 6 > cov_inv = Matrix< double, 6, 6 >::Zero( );
            // cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            // MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity( );

            MatrixXd r_A = tmp_A.transpose( ) * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose( ) * cov_inv * tmp_b;

            A.block< 6, 6 >( i * 3, i * 3 ) += r_A.topLeftCorner< 6, 6 >( );
            b.segment< 6 >( i * 3 ) += r_b.head< 6 >( );

            A.bottomRightCorner< 3, 3 >( ) += r_A.bottomRightCorner< 3, 3 >( );
            b.tail< 3 >( ) += r_b.tail< 3 >( );

            A.block< 6, 3 >( i * 3, n_state - 3 ) += r_A.topRightCorner< 6, 3 >( );
            A.block< 3, 6 >( n_state - 3, i * 3 ) += r_A.bottomLeftCorner< 3, 6 >( );
        }
        A       = A * 1000.0;
        b       = b * 1000.0;
        x_state = A.ldlt( ).solve( b );

        VectorXd dg = x_state.segment< 2 >( n_state - 3 );
        g0          = ( g0 + lxly * dg ).normalized( ) * G.norm( );
        // double s = x(n_state - 1);
    }
    g = g0;
}

MatrixXd
InitVio::VisualImuAlignmentVelScale::TangentBasis( Vector3d& g0 )
{
    Vector3d b, c;
    Vector3d a = g0.normalized( );
    Vector3d tmp( 0, 0, 1 );
    if ( a == tmp )
        tmp << 1, 0, 0;
    b = ( tmp - a * ( a.transpose( ) * tmp ) ).normalized( );
    c = a.cross( b );
    MatrixXd bc( 3, 2 );
    bc.block< 3, 1 >( 0, 0 ) = b;
    bc.block< 3, 1 >( 0, 1 ) = c;
    return bc;
}
