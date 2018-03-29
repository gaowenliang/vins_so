#include "vins_so/estimator/slideWindow/slidewindowposevel.h"

using namespace slidewindow;

SlideWindowPoseVel::SlideWindowPoseVel( int window_size, int camera_num )
: SlideWindowPose( window_size, camera_num )
{
    Vel.resize( WINDOW_SIZE + 1 );

    for ( int i = 0; i < WINDOW_SIZE + 1; i++ )
    {
        Vel[i].setZero( );
    }
}

void
SlideWindowPoseVel::clearWindow( )
{
    for ( int i = 0; i < WINDOW_SIZE + 1; i++ )
    {
        Stamps[i] = 0.0;
        Pose[i].setZero( );
        Vel[i].setZero( );
    }
    m_featureManager.clearState( );
}

Tf
SlideWindowPoseVel::poseLast( )
{
    return Pose[WINDOW_SIZE];
}

Tf
SlideWindowPoseVel::poseIndex( int index )
{
    return Pose[index];
}

Vector3d
SlideWindowPoseVel::lastVel( )
{
    return Vel[WINDOW_SIZE];
}

void
SlideWindowPoseVel::windowToDouble( double pp_Pose[][7], double Speed[][3] )
{
    for ( int i = 0; i <= WINDOW_SIZE; i++ )
    {
        pp_Pose[i][0] = Pose[i].T.x( );
        pp_Pose[i][1] = Pose[i].T.y( );
        pp_Pose[i][2] = Pose[i].T.z( );

        Quaterniond q{ Pose[i].R };
        pp_Pose[i][3] = q.x( );
        pp_Pose[i][4] = q.y( );
        pp_Pose[i][5] = q.z( );
        pp_Pose[i][6] = q.w( );

        Speed[i][0] = Vel[i].x( );
        Speed[i][1] = Vel[i].y( );
        Speed[i][2] = Vel[i].z( );
    }
}

void
SlideWindowPoseVel::setPoseIndex( int index, Vector3d T_in, Quaterniond q_in )
{
    Pose[index].setRT( q_in.normalized( ).toRotationMatrix( ), T_in );
}

void
SlideWindowPoseVel::setPoseIndex( int index, Vector3d T_in, Matrix3d R_in )
{
    Pose[index].setRT( R_in, T_in );
}

void
SlideWindowPoseVel::setVelIndex( int index, Vector3d vel_in )
{
    Vel[index] = vel_in;
}

void
SlideWindowPoseVel::doubleToWindow( const double pp_Pose[][7], const double Speed[][3] )
{
    for ( int i = 0; i <= WINDOW_SIZE; i++ )
    {
        Pose[i].T = Eigen::Vector3d( pp_Pose[i][0], //
                                     pp_Pose[i][1],
                                     pp_Pose[i][2] );

        Pose[i].R = Eigen::Quaterniond( pp_Pose[i][6], //
                                        pp_Pose[i][3],
                                        pp_Pose[i][4],
                                        pp_Pose[i][5] )
                    .normalized( )
                    .toRotationMatrix( );

        Vel[i] = Eigen::Vector3d( Speed[i][0], Speed[i][1], Speed[i][2] );
    }
}

SlideWindowPoseVel&
SlideWindowPoseVel::operator=( const SlideWindowPoseVel& other )
{
    if ( this != &other )
    {
        WINDOW_SIZE = other.WINDOW_SIZE;
        NUM_OF_CAM  = other.NUM_OF_CAM;
        Stamps      = other.Stamps;
        Pose        = other.Pose;
        Vel         = other.Vel;
        m_margFlag  = other.m_margFlag;

        m_featureManager = other.m_featureManager;
    }
    return *this;
}

void
SlideWindowPoseVel::slideWindow( const bool shift_depth, const std::vector< Tf > tf_ic )
{
    if ( m_margFlag == MARGIN_OLD )
    {
        Tf back_Tf_wi = Pose[0];

        for ( int i = 0; i < WINDOW_SIZE; i++ )
        {
            Stamps[i] = Stamps[i + 1];
            Pose[i].swap( Pose[i + 1] );
            Vel[i].swap( Vel[i + 1] );
        }
        Stamps[WINDOW_SIZE] = Stamps[WINDOW_SIZE - 1];
        Pose[WINDOW_SIZE]   = Pose[WINDOW_SIZE - 1];
        Vel[WINDOW_SIZE]    = Vel[WINDOW_SIZE - 1];

        slideWindowOld( back_Tf_wi, tf_ic, shift_depth );
    }
    else if ( m_margFlag == MARGIN_SECOND_NEW )
    {
        Stamps[WINDOW_SIZE - 1] = Stamps[WINDOW_SIZE];
        Pose[WINDOW_SIZE - 1]   = Pose[WINDOW_SIZE];
        Vel[WINDOW_SIZE - 1]    = Vel[WINDOW_SIZE];

        slideWindowNew( );
    }
}
