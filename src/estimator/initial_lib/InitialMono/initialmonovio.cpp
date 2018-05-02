#include "vins_so/estimator/initial_lib/InitialMono/initialmonovio.h"

using namespace initialEx;

InitVio::InitialMonoVio::InitialMonoVio( int window_size, int camera_size )
: Initial( window_size, camera_size )
{
    m_RicInitial = InitialExRotationCamImuPtr( new InitialExRotationCamImu );
}

void
InitVio::InitialMonoVio::pushImage( double time, int _frame_count, const FeatureData& _points )
{
    FeatureData cap_points = getFeatureCamIndex( m_cameraIndex, _points, 0.5 );

    m_window->m_featureManager.addFeatureCamIndex( _frame_count, cap_points, m_cameraIndex );

    if ( m_window->m_featureManager.getFeatureCount( ) < 10 )
        m_window->setMargSecondNew( );
    else
        m_window->setMargOld( );

    InitVio::ImageImuFrame image_imu_info( cap_points, m_tmpPreIntegration, time );
    m_imageFrameAll.insert( std::make_pair( time, image_imu_info ) );

    ROS_DEBUG( "MONO INIT, frame is------------------%s", m_window->m_margFlag ? "reject" : "accept" );
    ROS_DEBUG( "MONO INIT, %s", m_window->m_margFlag ? "Non-keyframe" : "Keyframe" );
    ROS_DEBUG( "MONO INIT, number of feature: %d", m_window->m_featureManager.getFeatureCount( ) );

    m_window->Stamps[_frame_count] = time;

    ROS_WARN_STREAM( "num " << m_exParams.size( ) << " checkEx( ) " << checkEx( )
                            << " m_exParamSet "
                            << m_exParamSet );

    if ( !checkEx( ) )
    {
        calcEx( _frame_count );
    }
}

bool
InitVio::InitialMonoVio::initial( )
{
    if ( !checkEx( ) )
        return false;

    m_done = initialStructure( );
    return m_done;
}

// bool
// InitVio::InitialMonoVio::getEx( std::vector< Tf >& exparam ) const
//{
//    if ( checkEx( ) )
//    {
//        exparam[m_cameraIndex] = m_exParams[0]( );
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}

FeatureData
InitVio::InitialMonoVio::getFeatureCamIndex( int cam_index, //
                                             const FeatureData& _points,
                                             double z_the )
{
    FeatureData cap_image_index;
    for ( auto& cap_feature : _points )
    {
        int feature_id = cap_feature.first;
        int camera_id  = cap_feature.second.at( 0 ).first;

        if ( camera_id == cam_index )
        {
            Vector3d means_point = cap_feature.second.at( 0 ).second;

            if ( means_point.z( ) > z_the )
                cap_image_index[feature_id].emplace_back( camera_id, means_point );
            else
                continue;
        }
        else
            continue;
    }
    return cap_image_index;
}

void
InitVio::InitialMonoVio::slideWindow( const std::vector< Tf > tf_ic )
{
    m_window->slideWindow( false, tf_ic );

    if ( m_window->m_margFlag == slidewindow::MARGIN_OLD )
    {
        double t_0 = m_window->Stamps[0];

        map< double, ImageImuFrame >::iterator it_0;
        it_0 = m_imageFrameAll.find( t_0 );
        //        delete it_0->second.preIntegration;
        m_imageFrameAll.erase( m_imageFrameAll.begin( ), it_0 );
    }
}

bool
InitVio::InitialMonoVio::initialStructure( )
{
    TicToc t_sfm;

    // check imu observibility
    {
        map< double, ImageImuFrame >::iterator frame_it;
        Vector3d sum_g;
        for ( frame_it = m_imageFrameAll.begin( ), frame_it++; frame_it != m_imageFrameAll.end( ); frame_it++ )
        {
            double dt      = frame_it->second.preIntegration->sum_dt;
            Vector3d tmp_g = frame_it->second.preIntegration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g     = sum_g * 1.0 / ( ( int )m_imageFrameAll.size( ) - 1 );
        double var = 0;
        for ( frame_it = m_imageFrameAll.begin( ), frame_it++; frame_it != m_imageFrameAll.end( ); frame_it++ )
        {
            double dt      = frame_it->second.preIntegration->sum_dt;
            Vector3d tmp_g = frame_it->second.preIntegration->delta_v / dt;
            var += ( tmp_g - aver_g ).transpose( ) * ( tmp_g - aver_g );
            // cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt( var / ( ( int )m_imageFrameAll.size( ) - 1 ) );
        // ROS_WARN("IMU variation %f!", var);
        if ( var < 0.25 )
        {
            ROS_INFO( "IMU excitation not enouth!" );
            // return false;
        }
    }

    // global sfm
    vector< Quaterniond > q_wcs;
    vector< Vector3d > T_wcs;
    map< int, Vector3d > sfm_tracked_points;
    vector< SFMFeature > sfmFeatures;

    q_wcs.resize( m_windowSize + 1 );
    T_wcs.resize( m_windowSize + 1 );

    for ( auto& it_per_id : m_window->m_featureManager.feature )
    {
        int imu_j = it_per_id.m_startFrame - 1;

        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id    = it_per_id.m_featureId;
        int camera_id     = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId;

        if ( camera_id != m_cameraIndex )
            continue;

        for ( auto& it_per_frame : it_per_id.m_fPerFrame )
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.m_fPerCam.at( 0 ).m_measPoint;
            tmp_feature.observation.push_back(
            make_pair( imu_j, Eigen::Vector3d( pts_j.x( ), pts_j.y( ), pts_j.z( ) ).normalized( ) ) );
        }

        sfmFeatures.push_back( tmp_feature );
    }

    Matrix3d relative_R;
    Vector3d relative_T;
    bool relative_pose_solved = false;
    int link_index;
    if ( !relativePose( relative_R, relative_T, link_index, relative_pose_solved ) )
    {
        ROS_INFO( "Not enough features or parallax in camera %d; Move device around", m_cameraIndex );
        return false;
    }

    GlobalSFM sfm;
    bool sfm_constructed_ok = false;

    if ( relative_pose_solved )
    {
        sfm_constructed_ok = sfm.construct( m_windowSize + 1,
                                            q_wcs,
                                            T_wcs,
                                            link_index,
                                            relative_R,
                                            relative_T, //
                                            sfmFeatures,
                                            sfm_tracked_points );
        ROS_DEBUG_STREAM( "sfm_" << m_cameraIndex << " : " << sfm_constructed_ok );
    }
    else
    {
        ROS_DEBUG( "global SFM failed!" );
        m_window->setMargOld( );
        return false;
    }

    // solve pnp for all frame
    map< double, ImageImuFrame >::iterator frame_it;
    map< int, Vector3d >::iterator it;
    bool pnp_all_ok = false;

    if ( sfm_constructed_ok )
    {
        frame_it = m_imageFrameAll.begin( );

        for ( int i = 0; frame_it != m_imageFrameAll.end( ); frame_it++ )
        {
            // provide initial guess
            if ( ( frame_it->first ) == m_window->Stamps[i] )
            {
                frame_it->second.m_isKeyFrame = true;
                frame_it->second.tf_wb.setRT( q_wcs.at( i ).toRotationMatrix( )
                                              * RIC[m_cameraIndex].transpose( ), //
                                              T_wcs.at( i ) );
                i++;

                continue;
            }

            if ( ( frame_it->first ) > m_window->Stamps[i] )
            {
                i++;
            }

            frame_it->second.m_isKeyFrame = false;

            vector< Eigen::Vector3d > pts_3_vector_tmp;
            vector< Eigen::Vector3d > pts_2_vector_tmp;
            for ( auto& id_pts : frame_it->second.points )
            {
                int feature_id = id_pts.first;
                for ( auto& i_p : id_pts.second )
                {
                    // cout << " pts image_frame " << (i_p.second.head<2>() * 460
                    // ).transpose() << endl;
                    it = sfm_tracked_points.find( feature_id );
                    if ( it->first != m_cameraIndex )
                        continue;

                    if ( it != sfm_tracked_points.end( ) )
                    {
                        Vector3d world_pts = it->second;
                        pts_3_vector_tmp.push_back( world_pts );

                        Vector3d img_pts = i_p.second.head< 3 >( );
                        pts_2_vector_tmp.push_back( img_pts );
                    }
                }
            }
            if ( pts_3_vector_tmp.size( ) < 6 )
            {
                // cout << "pts_3_vector size " << pts_3_vector.size() << endl;
                ROS_DEBUG( "Not enough points for solve pnp ! only %d ", pts_3_vector_tmp.size( ) );
                continue; //  return false;
            }

            Eigen::Quaterniond q_dst;
            Eigen::Vector3d T_dst;
            cv::Pnp( pts_2_vector_tmp, pts_3_vector_tmp, q_dst, T_dst );
            Tf tf_cw( q_dst.toRotationMatrix( ), T_dst );
            Tf tf_wc = tf_cw.inverse( );
            // Eigen::Matrix3d R_dst = q_dst.toRotationMatrix( ).transpose( );
            // Eigen::Vector3d T_pnp = R_dst * ( -T_dst );
            ROS_WARN_STREAM( "tf_wc " << tf_wc );

            // fixme
            frame_it->second.tf_wb.setRT( tf_wc.R * m_exParams[m_cameraIndex]( ).R.transpose( ),
                                          tf_wc.T );
        }
        pnp_all_ok |= true;
    }
    if ( !pnp_all_ok )
        return false;

    if ( visualInitialAlign( m_cameraIndex ) )
        return true;
    else
    {
        ROS_INFO( "misalign visual structure with IMU" );
        return false;
    }
}

void
InitVio::InitialMonoVio::copyInitInfoBack( slidewindow::SlideWindowPoseVelPtr window_new,
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
    /// change state, add scale information
    /// ---------------------------------------
    double s = ( m_Xtmp.tail< 1 >( ) )( 0 );
    Tf tf_ic = m_exParams[m_cameraIndex]( );

    for ( int i = window_new->WINDOW_SIZE; i >= 0; i-- )
    {
        window_new->Pose[i].T
        = s * window_new->Pose[i].T - window_new->Pose[i].R * tf_ic.T
          - ( s * window_new->Pose[0].T - window_new->Pose[0].R * tf_ic.T );
    }
    ROS_DEBUG( "final estimated scale: %f", s );

    int kv = -1;
    map< double, ImageImuFrame >::iterator frame_i;
    for ( frame_i = m_imageFrameAll.begin( ); frame_i != m_imageFrameAll.end( ); frame_i++ )
        if ( frame_i->second.m_isKeyFrame )
        {
            kv++;
            // std::cout << kv << ":\n" << x_tmp.segment< 3 >( kv * 3 ) << "\n";
            window_new->Vel[kv] = frame_i->second.tf_wb.R * m_Xtmp.segment< 3 >( kv * 3 );
        }

    /// ---------------------------------------
    ///   change state, add scale information
    /// ---------------------------------------
    double baseline = window_new->m_featureManager.baseline;
    Tf tf_rl        = window_new->m_featureManager.tf_rl;

    window_new->m_featureManager = m_window->m_featureManager;
    window_new->m_featureManager.setBaseline( baseline );
    window_new->m_featureManager.setTfrl( tf_rl );

    ROS_DEBUG_STREAM( "mono size " << window_new->m_featureManager.getFeatureCountMono( ) );
    ROS_DEBUG_STREAM( "stereo size " << window_new->m_featureManager.getFeatureCountStereo( ) );

    window_new->m_featureManager.clearDepthCamIndex( m_cameraIndex );

    /// ---------------------------------------
    ///  triangulat on cam pose , assume no tic
    /// ---------------------------------------
    tf_ic.T.setZero( );

    window_new->m_featureManager.triangulateCamIndex( window_new->Pose, tf_ic ); // TODO

    /// ---------------------------------------
    ///   repropagate IMU preIntegration
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
            // ROS_DEBUG_STREAM( " double sum_dt " << preIntegration->sum_dt );
            preIntegration->repropagate( Vector3d::Zero( ), imu_new->m_Bgs[i] );
            std::swap( imu_new->m_preIntegrations[i], preIntegration );
        }
    }

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
}

void
InitVio::InitialMonoVio::calcEx( int _frame_count )
{
    ROS_INFO( "calibrating extrinsic param, rotation movement is needed" );
    if ( _frame_count != 0 )
    {
        Matrix3d calib_ric;

        PointsCorres corres
        = m_window->m_featureManager.getCorrespondingCamIndex( m_cameraIndex, //
                                                               _frame_count - 1,
                                                               _frame_count );

        if ( corres.size( ) < 10 )
            return;

        double frame_time = m_window->Stamps[_frame_count];
        if ( m_RicInitial->CalibrationExRotation(
             corres, //
             m_imageFrameAll[frame_time].preIntegration->delta_q,
             calib_ric ) )
        {
            ROS_WARN( "initial extrinsic rotation calib success of Cam%d", m_cameraIndex );
            ROS_WARN_STREAM( "initial extrinsic rotation:\n" << calib_ric );
            ROS_WARN_STREAM( "initial rotation eular: " << Utility::R2ypr( calib_ric ).transpose( ) );

            Tf tf_ic;
            tf_ic.setRT( calib_ric, Eigen::Vector3d::Zero( ) );
            m_exParams[0].set( tf_ic );
        }
    }
}

bool
InitVio::InitialMonoVio::visualInitialAlign( int align_camera_index )
{
    InitVio::VisualImuAlignmentVelScale viAligment;

    // solve scale
    bool result = viAligment.solve( m_imageFrameAll, //
                                    m_Bgs,
                                    m_g,
                                    m_Xtmp,
                                    align_camera_index );

    if ( !result )
    {
        ROS_DEBUG_STREAM( "solve g failed! g:" << m_g.transpose( ) );
        return false;
    }
    else
        return true;
}

bool
InitVio::InitialMonoVio::relativePose( Matrix3d& relative_R, Vector3d& relative_T, int& l, bool& is_solved )
{
    // find previous frame which contians enough correspondance and parallex with newest
    // frame
    for ( int i = 0; i < m_windowSize; i++ )
    {
        vector< pair< Vector3d, Vector3d > > corres;
        corres = m_window->m_featureManager.getCorrespondingCamIndex( m_cameraIndex, i, m_windowSize );

        if ( corres.size( ) > 20 )
        {
            double sum_parallax = 0;
            double average_parallax;
            for ( int j = 0; j < int( corres.size( ) ); j++ )
            {
                Vector2d pts_0( corres[j].first( 0 ), corres[j].first( 1 ) );
                Vector2d pts_1( corres[j].second( 0 ), corres[j].second( 1 ) );
                double parallax = ( pts_0 - pts_1 ).norm( );
                sum_parallax    = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int( corres.size( ) );

            if ( average_parallax * 460 > 30 && m_estimator.solveRelativeRT( corres, relative_R, relative_T ) )
            {
                is_solved = true;

                l = i;
                ROS_DEBUG_STREAM( "camera " << m_cameraIndex << " points: " << corres.size( ) << "\n"
                                            << "RR\n"
                                            << relative_R
                                            << "\nTT\n"
                                            << relative_T );

                ROS_DEBUG(
                "average_parallax %f choose l %d and newest frame to "
                "triangulate the whole structure",
                average_parallax * 460,
                l );
                return true;
            }
        }
    }
    return false;
}
