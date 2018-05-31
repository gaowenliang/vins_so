#ifndef SOLVEGYROSCOPEBIAS_H
#define SOLVEGYROSCOPEBIAS_H

#include "../InitialBase/initialbase.h"
#include <eigen3/Eigen/Eigen>
#include <map>
#include <vector>

namespace InitVio
{

class SolveGyroscopeBias
{
    public:
    SolveGyroscopeBias( ) {}

    bool solve( map< double, InitVio::ImageImuFrame >& imageFrameAll, std::vector< Vector3d >& Bgs )
    {
        Matrix3d A;
        Vector3d b;
        Vector3d delta_bg;

        A.setZero( );
        b.setZero( );

        map< double, InitVio::ImageImuFrame >::iterator frame_i;
        map< double, InitVio::ImageImuFrame >::iterator frame_j;
        for ( frame_i = imageFrameAll.begin( ); next( frame_i ) != imageFrameAll.end( ); frame_i++ )
        {
            frame_j = next( frame_i );
            MatrixXd tmp_A( 3, 3 );
            tmp_A.setZero( );
            VectorXd tmp_b( 3 );
            tmp_b.setZero( );

            Tf tf_i = frame_i->second.tf_wb;
            Tf tf_j = frame_j->second.tf_wb;
            Eigen::Quaterniond q_ij( tf_i.R.transpose( ) * tf_j.R );
            //        ROS_WARN_STREAM( "q_ij " << q_ij.coeffs( ).transpose( ) );

            tmp_A = frame_j->second.preIntegration->jacobian.template block< 3, 3 >( O_R, O_BG );
            tmp_b = 2 * ( frame_j->second.preIntegration->delta_q.inverse( ) * q_ij ).vec( );
            A += tmp_A.transpose( ) * tmp_A;
            b += tmp_A.transpose( ) * tmp_b;
            //        ROS_WARN_STREAM( "tmp_A " << tmp_A );
        }
        delta_bg = A.ldlt( ).solve( b );
        ROS_WARN_STREAM( "gyroscope bias initial calibration " << delta_bg.transpose( ) );

        if ( delta_bg.hasNaN( ) )
        {
            ROS_WARN_STREAM( "solve gyroscope bias NaN" );
            return false;
        }

        for ( int i = 0; i <= WINDOW_SIZE; i++ )
            Bgs[i] += delta_bg;

        for ( frame_i = imageFrameAll.begin( ); next( frame_i ) != imageFrameAll.end( ); frame_i++ )
        {
            //        std::cout << "Bgs[0] " << Bgs[0].transpose( ) << "\n ";
            frame_j = next( frame_i );
            frame_j->second.preIntegration->repropagate( Vector3d::Zero( ), Bgs[0] );
        }

        return true;
    }
};
}

#endif // SOLVEGYROSCOPEBIAS_H
