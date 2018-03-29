#include "vins_so/estimator/slideWindow/slidewindowpose.h"

using namespace slidewindow;

SlideWindowPose::SlideWindowPose( int window_size, int camera_num )
: SlideWindowBase( window_size )
, NUM_OF_CAM( camera_num )
{
    Pose.resize( WINDOW_SIZE + 1 );

    for ( int i = 0; i < WINDOW_SIZE + 1; i++ )
    {
        Pose[i].setZero( );
    }
}

void
SlideWindowPose::clearWindow( )
{
    for ( int i = 0; i < WINDOW_SIZE + 1; i++ )
    {
        Stamps[i] = 0.0;
        Pose[i].setZero( );
    }
    m_featureManager.clearState( );
}

SlideWindowPose&
SlideWindowPose::operator=( const SlideWindowPose& other )
{
    if ( this != &other )
    {
        WINDOW_SIZE = other.WINDOW_SIZE;
        NUM_OF_CAM  = other.NUM_OF_CAM;
        Pose        = other.Pose;
        Stamps      = other.Stamps;
        m_margFlag  = other.m_margFlag;

        m_featureManager = other.m_featureManager;
    }
    return *this;
}

void
SlideWindowPose::slideWindow( const bool shift_depth, const std::vector< Tf > tf_ic )
{
    if ( m_margFlag == MARGIN_OLD )
    {
        Tf back_Tf_wi = Pose[0];

        for ( int i = 0; i < WINDOW_SIZE; i++ )
        {
            Pose[i].swap( Pose[i + 1] );
            Stamps[i] = Stamps[i + 1];
        }
        Stamps[WINDOW_SIZE] = Stamps[WINDOW_SIZE - 1];
        Pose[WINDOW_SIZE]   = Pose[WINDOW_SIZE - 1];

        slideWindowOld( back_Tf_wi, tf_ic, shift_depth );
    }
    else if ( m_margFlag == MARGIN_SECOND_NEW )
    {
        Stamps[WINDOW_SIZE - 1] = Stamps[WINDOW_SIZE];
        Pose[WINDOW_SIZE - 1]   = Pose[WINDOW_SIZE];

        slideWindowNew( );
    }
}

MarginalizationFlag
SlideWindowPose::addFeaturesToWindow( int _frame_count, //
                                      const map< int, vector< pair< int, Vector3d > > >& _image )
{
    ROS_DEBUG( "Adding feature points %lu", _image.size( ) );

    if ( m_featureManager.addFeatureCheckParallax( _frame_count, _image ) )
        m_margFlag = MARGIN_OLD;
    else
        m_margFlag = MARGIN_SECOND_NEW;

    return m_margFlag;
}

void
SlideWindowPose::slideWindowOld( const Tf& back_Tf_wi, const std::vector< Tf >& Tf_ic, bool shift_depth )
{
    if ( shift_depth )
    {
        std::vector< Tf > tfs_wci;
        std::vector< Tf > tfs_wcj;

        tfs_wci.resize( NUM_OF_CAM );
        tfs_wcj.resize( NUM_OF_CAM );

        for ( int cam_index = 0; cam_index < NUM_OF_CAM; ++cam_index )
        {
            tfs_wci[cam_index] = back_Tf_wi + Tf_ic[cam_index];
            tfs_wcj[cam_index] = Pose[0] + Tf_ic[cam_index];
        }

        m_featureManager.removeBackShiftDepth( tfs_wci, tfs_wcj );
    }
    else
        m_featureManager.removeBack( );
}

void
SlideWindowPose::slideWindowNew( )
{
    m_featureManager.removeFront( WINDOW_SIZE );
}
