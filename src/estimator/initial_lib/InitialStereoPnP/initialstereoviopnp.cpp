#include "vins_so/estimator/initial_lib/InitialStereoPnP/initialstereoviopnp.h"
#include "vins_so/estimator/initial_lib/VisualImuAlignment/VisualImuAlignmentVel.h"

InitVio::InitialStereoVioPnP::InitialStereoVioPnP( int window_size, int camera_size )
: Initial( window_size, camera_size )
, m_isExCalced( false )
{
}

void
InitVio::InitialStereoVioPnP::pushImage( double time, int _frame_count, const FeatureData& _points )
{
    m_window->m_featureManager.addFeatureStereo( _frame_count, _points );

    if ( m_window->m_featureManager.getFeatureCountStereo( ) < 10 )
        m_window->setMargSecondNew( );
    else
        m_window->setMargOld( );

    InitVio::ImageImuFrame image_imu_info( _points, m_tmpPreIntegration, time );
    m_imageFrameAll.insert( std::make_pair( time, image_imu_info ) );

    ROS_DEBUG( "STEREO INIT, frame is------------------%s", m_window->m_margFlag ? "reject" : "accept" );
    ROS_DEBUG( "STEREO INIT, %s", m_window->m_margFlag ? "Non-keyframe" : "Keyframe" );
    ROS_DEBUG( "STEREO INIT, number of feature: %d",
               m_window->m_featureManager.getFeatureCountStereo( ) );

    m_window->Stamps[_frame_count] = time;

    //    if ( !checkEx( ) )
    //    {
    //        calcEx( _frame_count );
    //    }

    if ( !m_isExCalced )
        calcExCamCam( );

    initPnP( _frame_count );
    stereoTriangulate( _frame_count );
}

void
InitVio::InitialStereoVioPnP::slideWindow( const std::vector< Tf > tf_ic )
{
    m_window->slideWindow( false, tf_ic );

    double t_0 = m_window->Stamps[0];
    if ( m_window->m_margFlag == slidewindow::MARGIN_OLD )
    {
        map< double, ImageImuFrame >::iterator it_0;
        it_0 = m_imageFrameAll.find( t_0 );

        it_0->second.preIntegration.reset( );
        m_imageFrameAll.erase( m_imageFrameAll.begin( ), it_0 );
    }
}

void
InitVio::InitialStereoVioPnP::copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new,
                                                slidewindow::SlideWindowIMUPtr imu_new,
                                                Vector3d& _g )
{
    if ( window_new->WINDOW_SIZE != m_window->WINDOW_SIZE )
        ROS_ERROR( "Window Size Not The Same!!!" );

    /// ---------------------------------------
    /// copy state to window, up-to scale
    /// ---------------------------------------
    for ( int i = 0; i <= m_window->WINDOW_SIZE; i++ )
    {
        Tf tf_wb = m_imageFrameAll[m_window->Stamps[i]].tf_wb;

        window_new->setStampIndex( i, m_window->Stamps[i] );
        window_new->setTfIndex( i, tf_wb );

        imu_new->m_Bgs[i] = m_Bgs[i];
        imu_new->m_Bas[i].setZero( );

        m_imageFrameAll[m_window->Stamps[i]].m_isKeyFrame = true;
    }

    /// ---------------------------------------
    ///   change state, add scale information
    /// ---------------------------------------
    for ( int i = window_new->WINDOW_SIZE; i >= 0; i-- )
    {
        window_new->Pose[i].T = window_new->Pose[i].T - window_new->Pose[0].T;
        std::cout << "P" << i << " " << window_new->Pose[i].T.transpose( ) << "\n";
    }

    int kv = -1;
    map< double, ImageImuFrame >::iterator frame_i;
    for ( frame_i = m_imageFrameAll.begin( ); frame_i != m_imageFrameAll.end( ); frame_i++ )
        if ( frame_i->second.m_isKeyFrame )
        {
            kv++;
            window_new->Vel[kv] = window_new->Pose[kv].R * m_Xtmp.segment< 3 >( kv * 3 );
        }

    /// ---------------------------------------
    ///   change state, add scale information
    /// ---------------------------------------
    double baseline = window_new->m_featureManager.baseline;
    Tf tf_rl        = window_new->m_featureManager.tf_rl;

    ROS_DEBUG_STREAM( "baseline" << baseline );
    ROS_DEBUG_STREAM( "tf_rl" << tf_rl );

    window_new->m_featureManager = m_window->m_featureManager;
    window_new->m_featureManager.setBaseline( baseline );
    window_new->m_featureManager.setTfrl( tf_rl );

    ROS_DEBUG_STREAM( "mono size " << window_new->m_featureManager.getFeatureCountMono( ) );
    ROS_DEBUG_STREAM( "stereo size " << window_new->m_featureManager.getFeatureCountStereo( ) );

    window_new->m_featureManager.clearDepth( );

    /// ---------------------------------------
    ///  triangulat on cam pose , assume no tic
    /// ---------------------------------------
    std::vector< Tf > tf_ics;
    for ( auto& ex : m_exParams )
        tf_ics.push_back( ex( ) );
    window_new->m_featureManager.triangulate( m_window->Pose, tf_ics );

    /// ---------------------------------------
    /// repropagate IMU preIntegration
    /// ---------------------------------------

    for ( int i = 0; i <= m_window->WINDOW_SIZE; i++ )
    {
        auto preIntegration = m_imageFrameAll[m_window->Stamps[i]].preIntegration;

        if ( preIntegration == nullptr )
        {
            continue;
            ROS_DEBUG_STREAM( "frame_it->second.preIntegration == nullptr " );
        }
        else
        {
            ROS_DEBUG_STREAM( " double sum_dt " << preIntegration->sum_dt );

            preIntegration->repropagate( Vector3d::Zero( ), imu_new->m_Bgs[i] );
            // imu_new->setPreIntegrationsIndex( frame_it->second.preIntegration, i );
            std::swap( imu_new->m_preIntegrations[i], preIntegration );
            // std::cout << "delta_v" << i << " "
            //           << imu_new->m_preIntegrations[i]->delta_v.transpose( ) << "\n";
        }
    }

    std::cout << "\n";

    /// --------------------------------------
    /// rotation the yaw from the first frame
    /// --------------------------------------
    Matrix3d R0       = Utility::g2R( m_g );
    double yaw        = Utility::R2ypr( R0 * window_new->Pose[0].R ).x( );
    Matrix3d rot_diff = Utility::ypr2R( Eigen::Vector3d{ -yaw, 0, 0 } ) * R0;
    m_g               = rot_diff * m_g;
    for ( int i = 0; i <= window_new->WINDOW_SIZE; i++ )
    {
        window_new->setTfIndex( i, rot_diff * window_new->Pose[i] );
        window_new->Vel[i] = rot_diff * window_new->Vel[i];
    }

    ROS_DEBUG_STREAM( "g0     " << m_g.transpose( ) );
    ROS_DEBUG_STREAM( "my R0  " << window_new->Pose[0].toYPR( ).transpose( ) );

    window_new->setMargFlag( m_window->margFlag( ) );
    imu_new->setMargFlag( m_window->margFlag( ) );

    _g = m_g;
    imu_new->setG( m_g );

    ROS_DEBUG_STREAM( "copy back initial info" );
}

bool
InitVio::InitialStereoVioPnP::initial( )
{
    if ( !checkEx( ) )
        return false;

    if ( m_imageFrameAll.size( ) > 3 && m_window->m_featureManager.getFeatureCountStereo( ) > 20 )
    {
        InitVio::VisualImuAlignmentVel viAligment;

        int align_camera_index = 0;

        bool result = viAligment.solve( m_imageFrameAll, //
                                        m_Bgs,
                                        m_g,
                                        m_Xtmp,
                                        align_camera_index );

        if ( !result )
        {
            ROS_DEBUG_STREAM( "solve g failed! g: " << m_g.transpose( ) );
            m_done = false;
            return m_done;
        }
        else
        {
            m_done = true;
            return m_done;
        }
    }
    else
    {
        m_done = false;
        return m_done;
    }
}

Tf
InitVio::InitialStereoVioPnP::Tf_icl( )
{
    return m_exParams[0]( );
}

Tf
InitVio::InitialStereoVioPnP::Tf_icr( )
{
    return m_exParams[1]( );
}

void
InitVio::InitialStereoVioPnP::calcEx( int _frame_count )
{
    PointsCorres corres = m_window->m_featureManager.getCorrespondingStereo( _frame_count );
}

void
InitVio::InitialStereoVioPnP::calcExCamCam( )
{
    m_Tf_rl  = Tf_icr( ).inverse( ) + Tf_icl( );
    m_Tf_lr  = m_Tf_rl.inverse( );
    baseline = m_Tf_rl.norm( );
    ROS_DEBUG( "calcExCamCam in init " );
    ROS_DEBUG_STREAM( "m_Tf_rl\n" << m_Tf_rl );
    ROS_DEBUG_STREAM( "baseline: " << baseline );

    m_isExCalced = true;
}

void
InitVio::InitialStereoVioPnP::initPnP( int frame )
{
    if ( frame == 0 )
        return;

    vector< Eigen::Vector3d > pts_2_vector_tmp;
    vector< Eigen::Vector3d > pts_3_vector_tmp;

    for ( auto& it_per_id : m_window->m_featureManager.feature )
    {
        if ( !it_per_id.estimated )
            continue;

        unsigned int start_frame = it_per_id.m_startFrame;
        unsigned int this_frame  = frame - start_frame;
        if ( this_frame >= it_per_id.m_fPerFrame.size( ) )
            continue;

        Vector3d point_w = it_per_id.m_position_w;
        Vector3d measure_l;
        if ( it_per_id.m_fPerFrame[this_frame].m_fPerCam[0].m_cameraId == 0 )
            measure_l = it_per_id.m_fPerFrame[this_frame].m_fPerCam[0].m_measPoint;
        else if ( it_per_id.m_fPerFrame[this_frame].m_fPerCam[1].m_cameraId == 0 )
            measure_l = it_per_id.m_fPerFrame[this_frame].m_fPerCam[1].m_measPoint;
        else
            continue;

        pts_2_vector_tmp.push_back( measure_l.normalized( ) );
        pts_3_vector_tmp.push_back( point_w );
    }

    ROS_DEBUG_STREAM( "points for PnP " << pts_3_vector_tmp.size( ) );
    if ( pts_3_vector_tmp.size( ) < 10 )
        return;
    //    std::cout << " point size " << pts_3_vector_tmp.size( ) << std::endl;

    Quaterniond q_cw;
    Vector3d P_cw;

    bool pnp_done;
    if ( frame == 1 )
    {
        cv::Pnp pnp( pts_2_vector_tmp, pts_3_vector_tmp );
        pnp_done = pnp.getRT( q_cw, P_cw );
    }
    else
    {
        Tf tf_cw_last = Tf_icl( ).inverse( ) + m_window->Pose[frame - 1];

        cv::Pnp pnp( pts_2_vector_tmp,
                     pts_3_vector_tmp, //
                     Quaterniond( tf_cw_last.R ),
                     tf_cw_last.T,
                     q_cw,
                     P_cw );
        pnp_done = pnp.getRT( q_cw, P_cw );
    }

    Tf tf_clw( q_cw, P_cw );
    std::cout << "RT_initial: " << endl << tf_clw << std::endl;

    Tf tf_wcl = tf_clw.inverse( );

    Tf tf_wi = tf_wcl + Tf_icl( ).inverse( );

    m_window->Pose[frame] = tf_wi;

    ROS_INFO( "pnp pose left %d %lf %lf %lf",
              frame,
              m_window->Pose[frame].T.x( ),
              m_window->Pose[frame].T.y( ),
              m_window->Pose[frame].T.z( ) );

    if ( ( m_window->Pose[frame].T - m_window->Pose[frame - 1].T ).norm( ) > 3 || !pnp_done ) // 10
    {
        ROS_ERROR( "pnp error" );
        m_window->Pose[frame] = m_window->Pose[frame - 1];
    }

    std::cout << "allFrameAll size: " << m_imageFrameAll.size( ) << std::endl;

    map< double, ImageImuFrame >::iterator frame_i = std::prev( m_imageFrameAll.end( ) );
    frame_i->second.tf_wb                          = m_window->Pose[frame];
}

void
InitVio::InitialStereoVioPnP::stereoTriangulate( int frame )
{
    int total_num = 0, stereo_num = 0;

    Tf Tf_wi  = m_window->Pose[frame];
    Tf Tf_wcl = Tf_wi + Tf_icl( );

    for ( auto& it_per_id : m_window->m_featureManager.feature )
    {
        if ( it_per_id.estimated )
            continue;

        total_num++;

        unsigned int start_frame = it_per_id.m_startFrame;
        unsigned int this_frame  = frame - start_frame;

        if ( it_per_id.m_fPerFrame[this_frame].m_numOfMeas < 2 )
            continue;

        auto& it_per_camera = it_per_id.m_fPerFrame[this_frame].m_fPerCam;

        Vector3d point_l, point_r;
        if ( it_per_camera[0].m_cameraId == 0 && it_per_camera[1].m_cameraId == 1 )
        {
            point_l = it_per_camera[0].m_measPoint;
            point_r = it_per_camera[1].m_measPoint;
        }
        //        else if ( it_per_camera[1].m_cameraId == 0 && it_per_camera[0].m_cameraId
        //        == 1 )
        //        {
        //            point_l = it_per_camera[1].m_measPoint;
        //            point_r = it_per_camera[0].m_measPoint;
        //        }
        else
            continue;

        stereo_num++;
        double scale = baseline / ( point_r - m_Tf_rl.R * point_l ).norm( );

        Vector3d stereo_cl = point_l * scale;
        ROS_DEBUG( "triangulate============%d %lf %lf %lf",
                   it_per_id.m_featureId,
                   stereo_cl.x( ),
                   stereo_cl.y( ),
                   stereo_cl.z( ) );

        it_per_id.m_position_w = Tf_wcl.transVector( stereo_cl );

        if ( stereo_cl.norm( ) < 0.3 )
            it_per_id.estimated = false;
        else
            it_per_id.estimated = true;
    }
    ROS_DEBUG( "%d frame, total num %d, stereo %d", frame, total_num, stereo_num );
}
