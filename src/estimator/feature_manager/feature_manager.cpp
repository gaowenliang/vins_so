#include "vins_so/estimator/feature_manager/feature_manager.h"

int
FeaturePerId::endFrame( )
{
    return m_startFrame + m_fPerFrame.size( ) - 1;
}

FeatureManager::FeatureManager( )
: m_isParallaxEnough( false )
{
}

FeatureManager::~FeatureManager( ) { feature.clear( ); }

void
FeatureManager::clearState( )
{
    feature.clear( );
}

int
FeatureManager::getFeatureCount( )
{
    int cnt = 0;
    for ( auto& it : feature )
    {
        it.m_usedNum = it.m_fPerFrame.size( );

        if ( it.m_usedNum >= 2 && it.m_startFrame < WINDOW_SIZE - 2 )
        {
            cnt++;
        }
    }
    return cnt;
}

int
FeatureManager::getFeatureCountMono( )
{
    int cnt = 0;
    for ( auto& it : feature )
    {
        it.m_usedNum = it.m_fPerFrame.size( );

        if ( it.m_usedNum >= 2 && it.m_startFrame < WINDOW_SIZE - 2 && it.m_fPerFrame[0].m_numOfMeas == 1 )
        {
            cnt++;
        }
    }

    return cnt;
}

int
FeatureManager::getFeatureCountStereo( )
{
    int cnt = 0;
    for ( auto& it : feature )
    {
        it.m_usedNum = it.m_fPerFrame.size( );

        if ( it.m_usedNum >= 2 && it.m_startFrame < WINDOW_SIZE - 2 && it.m_fPerFrame[0].m_numOfMeas > 1 )
        {
            cnt++;
        }
    }

    return cnt;
}

void
FeatureManager::getFeatureCount( int& cntMono, int& cntStereo )
{
    cntMono   = 0;
    cntStereo = 0;
    for ( auto& it : feature )
    {
        it.m_usedNum = it.m_fPerFrame.size( );

        if ( it.m_usedNum >= 2 && it.m_startFrame < WINDOW_SIZE - 2 )
        {
            if ( it.m_fPerFrame[0].m_numOfMeas == 1 )
                cntMono++;
            else if ( it.m_fPerFrame[0].m_numOfMeas > 1 )
                cntStereo++;
        }
        else
            continue;
    }
}

bool
FeatureManager::addFeature( int frame_count, const FeatureData& image )
{
    ROS_DEBUG( "input feature: %d", ( int )image.size( ) );
    ROS_DEBUG( "num of feature: %d", getFeatureCount( ) );

    last_track_num = 0;
    for ( auto& id_pts : image )
    {
        vector< FeaturePerCamera > f_per_cam;
        for ( auto& i_p : id_pts.second )
        {
            f_per_cam.push_back( FeaturePerCamera( i_p.cam_id, i_p.pt ) );
        }
        //        std::cout << " stereo " << f_per_cam.size( ) << std::endl;
        FeaturePerFrame f_per_fra( f_per_cam );

        int feature_id = id_pts.first;

        auto it = find_if( feature.begin( ), feature.end( ), [feature_id]( const FeaturePerId& it ) {
            return it.m_featureId == feature_id;
        } );

        if ( it == feature.end( ) )
        {
            feature.push_back( FeaturePerId( feature_id, frame_count ) );
            feature.back( ).m_fPerFrame.push_back( f_per_fra );
        }
        else if ( it->m_featureId == feature_id )
        {
            it->m_fPerFrame.push_back( f_per_fra );
            last_track_num++;
        }
    }

    if ( frame_count < 2 || last_track_num < 20 )
        return true;
}

bool
FeatureManager::addFeatureCamIndex( int frame_count, const FeatureData& image, int camera_index )
{
    ROS_DEBUG( "input feature: %d", ( int )image.size( ) );
    ROS_DEBUG( "num of feature: %d", getFeatureCount( ) );

    last_track_num = 0;
    for ( auto& id_pts : image )
    {
        vector< FeaturePerCamera > f_per_cam;
        for ( auto& i_p : id_pts.second )
        {
            int camera_id = i_p.cam_id;
            if ( camera_id != camera_index )
                continue;

            f_per_cam.push_back( FeaturePerCamera( camera_id, i_p.pt, i_p.err ) );
        }
        //        std::cout << " stereo " << f_per_cam.size( ) << std::endl;
        FeaturePerFrame f_per_fra( f_per_cam );

        int feature_id = id_pts.first;
        auto it = find_if( feature.begin( ), feature.end( ), [feature_id]( const FeaturePerId& it ) {
            return it.m_featureId == feature_id;
        } );

        if ( it == feature.end( ) )
        {
            feature.push_back( FeaturePerId( feature_id, frame_count ) );
            feature.back( ).m_fPerFrame.push_back( f_per_fra );
        }
        else if ( it->m_featureId == feature_id )
        {
            it->m_fPerFrame.push_back( f_per_fra );
            last_track_num++;
        }
    }

    if ( frame_count < 2 || last_track_num < 20 )
        return true;
    else
        return true;
}

bool
FeatureManager::addFeatureStereo( int frame_count, const FeatureData& image )
{
    ROS_DEBUG( "input feature: %d", ( int )image.size( ) );
    ROS_DEBUG( "num of feature: %d", getFeatureCount( ) );

    last_track_num = 0;
    for ( auto& id_pts : image )
    {
        vector< FeaturePerCamera > f_per_cam;
        for ( auto& i_p : id_pts.second )
        {
            f_per_cam.push_back( FeaturePerCamera( i_p.cam_id, i_p.pt, i_p.err ) );
        }
        FeaturePerFrame f_per_fra( f_per_cam );

        if ( f_per_fra.m_numOfMeas == 1 )
            continue;

        int feature_id = id_pts.first;

        auto it = find_if( feature.begin( ), //
                           feature.end( ),
                           [feature_id]( const FeaturePerId& it ) {
                               return it.m_featureId == feature_id;
                           } );

        if ( it == feature.end( ) )
        {
            feature.push_back( FeaturePerId( feature_id, frame_count ) );
            feature.back( ).m_fPerFrame.push_back( f_per_fra );
        }
        else if ( it->m_featureId == feature_id )
        {
            it->m_fPerFrame.push_back( f_per_fra );
            last_track_num++;
        }
    }

    if ( frame_count < 2 || last_track_num < 20 )
        return true;
    return true;
}

bool
FeatureManager::addFeatureCheckParallax( int frame_count, const FeatureData& image )
{
    ROS_DEBUG( "input feature: %d", ( int )image.size( ) );
    ROS_DEBUG( "num of feature: %d", getFeatureCount( ) );

    m_isParallaxEnough  = false;
    double parallax_sum = 0;
    int parallax_num    = 0;
    last_track_num      = 0;

    int num_pts_last = getFeatureCount( );

    for ( auto& id_pts : image )
    {
        vector< FeaturePerCamera > f_per_cam;
        for ( auto& i_p : id_pts.second )
        {
            f_per_cam.push_back( FeaturePerCamera( i_p.cam_id, i_p.pt, i_p.err ) );
        }
        //        std::cout << " stereo " << f_per_cam.size( ) << std::endl;
        FeaturePerFrame f_per_fra( f_per_cam );

        int feature_id = id_pts.first;

        auto it = find_if( feature.begin( ), //
                           feature.end( ),
                           [feature_id]( const FeaturePerId& it ) {
                               return it.m_featureId == feature_id;
                           } );

        if ( it == feature.end( ) )
        {
            feature.push_back( FeaturePerId( feature_id, frame_count ) );
            feature.back( ).m_fPerFrame.push_back( f_per_fra );
        }
        else if ( it->m_featureId == feature_id )
        {
            it->m_fPerFrame.push_back( f_per_fra );
            last_track_num++;
        }
    }

    if ( frame_count < 2 || last_track_num < 30 )
    {
        ROS_WARN( "last_track_num %d", last_track_num );
        m_isParallaxEnough = true;
        return true;
    }

    for ( auto& it_per_id : feature )
    {
        if ( it_per_id.m_startFrame <= frame_count - 2
             && it_per_id.m_startFrame + int( it_per_id.m_fPerFrame.size( ) ) - 1 >= frame_count - 1 )
        {
            parallax_sum += compensatedParallaxRad( it_per_id, frame_count );
            parallax_num++;
        }
    }

    //    check wether downward camera points visiable
    //    if ( parallax_sum / parallax_num < MIN_VISIABLE_PARALLAX && !IN_AIR )
    //    {
    //        DOWN_CAMERA_VISIABLE = false;
    //        removeMonoPointCamIndex( 1 );
    //        ROS_ERROR( "DOWN_CAMERA_VISIABLE FALSE" );
    //    }
    //    else
    //        DOWN_CAMERA_VISIABLE = true;

    ROS_WARN( "current parallax: %lf deg", parallax_sum / parallax_num * 57.29 );

    if ( parallax_num == 0 )
    {
        m_isParallaxEnough = false;
        return true;
    }
    else
    {
        ROS_DEBUG( "parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num );
        ROS_DEBUG( "current parallax: %lf", parallax_sum / parallax_num );

        m_isParallaxEnough = ( parallax_sum / parallax_num >= MIN_PARALLAX );

        if ( num_pts_last < 40 )
        {
            ROS_WARN( "exis points %d", last_track_num );
            return true;
        }
        else
        {
            return m_isParallaxEnough;
        }
    }
}

void
FeatureManager::debugShow( )
{
    ROS_DEBUG( "debug show" );
    for ( auto& it : feature )
    {
        ROS_ASSERT( it.m_fPerFrame.size( ) != 0 );
        ROS_ASSERT( it.m_startFrame >= 0 );
        ROS_ASSERT( it.m_usedNum >= 0 );

        ROS_DEBUG( "%d,%d,%d ", it.m_featureId, it.m_usedNum, it.m_startFrame );
        int sum = 0;
        for ( auto& j : it.m_fPerFrame )
        {
            ROS_DEBUG( "%d,", int( j.m_isUsed ) );
            sum += j.m_isUsed;
            printf( "(%lf,%lf) ", j.m_fPerCam.at( 0 ).m_measPoint( 0 ), j.m_fPerCam.at( 0 ).m_measPoint( 1 ) );
        }
        ROS_ASSERT( it.m_usedNum == sum );
    }
}

vector< PointsCorres >
FeatureManager::getCorresponding( int frame_count_l, int frame_count_r )
{
    vector< PointsCorres > corres_cameras;

    for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
    {
        vector< pair< Vector3d, Vector3d > > corres;
        //        ROS_DEBUG_STREAM( "feature " << feature.size( ) );
        for ( auto& it : feature )
        {
            if ( it.m_startFrame <= frame_count_l && it.endFrame( ) >= frame_count_r )
            {
                Vector3d a = Vector3d::Zero( ), b = Vector3d::Zero( );
                int idx_l = frame_count_l - it.m_startFrame;
                int idx_r = frame_count_r - it.m_startFrame;
                if ( it.m_fPerFrame[idx_l].m_fPerCam.at( 0 ).m_cameraId == camera_index )
                {
                    a = it.m_fPerFrame[idx_l].m_fPerCam.at( 0 ).m_measPoint;
                    b = it.m_fPerFrame[idx_r].m_fPerCam.at( 0 ).m_measPoint;

                    corres.push_back( make_pair( a, b ) );
                }
            }
        }
        corres_cameras.push_back( corres );
    }

    return corres_cameras;
}

PointsCorres
FeatureManager::getCorrespondingCamIndex( int camera_index, int frame_count_l, int frame_count_r )
{
    PointsCorres corres;
    //        ROS_DEBUG_STREAM( "feature " << feature.size( ) );
    for ( auto& it : feature )
    {

        bool finded;
        if ( it.m_startFrame <= frame_count_l && it.endFrame( ) >= frame_count_r )
        {
            finded = true;

            Vector3d a = Vector3d::Zero( ), b = Vector3d::Zero( );

            int idx_i = frame_count_l - it.m_startFrame;
            int idx_j = frame_count_r - it.m_startFrame;

            for ( auto& f_prt_cam : it.m_fPerFrame[idx_i].m_fPerCam )
            {
                if ( f_prt_cam.m_cameraId == camera_index )
                    a = f_prt_cam.m_measPoint;
                else
                {
                    finded = false;
                    continue;
                }
            }

            for ( auto& f_prt_cam : it.m_fPerFrame[idx_j].m_fPerCam )
            {
                if ( f_prt_cam.m_cameraId == camera_index )
                    b = f_prt_cam.m_measPoint;
                else
                {
                    finded = false;
                    continue;
                }
            }

            if ( finded )
                corres.push_back( make_pair( a, b ) );
            else
                continue;
        }
    }

    return corres;
}

PointsCorres
FeatureManager::getCorrespondingStereo( int frame_count )
{
    PointsCorres corres;

    for ( auto& it : feature )
    {
        int idx = frame_count - it.m_startFrame;
        if ( it.m_fPerFrame[idx].m_numOfMeas > 1 )
        {
            Vector3d a = Vector3d::Zero( ), b = Vector3d::Zero( );

            a = it.m_fPerFrame[idx].m_fPerCam.at( 0 ).m_measPoint;
            b = it.m_fPerFrame[idx].m_fPerCam.at( 1 ).m_measPoint;

            corres.push_back( make_pair( a, b ) );
        }
        else
            continue;
    }

    return corres;
}

void
FeatureManager::setDepth( const VectorXd& x )
{
    //    cout << " setDepth " << endl;
    int feature_index = -1;
    for ( auto& it_per_id : feature )
    {
        it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );
        if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;

#if INV_DEPTH
        it_per_id.m_depth = 1.0 / x( ++feature_index );
#else
        it_per_id.estimated_depth = x( ++feature_index );
#endif

        if ( it_per_id.m_depth < 0 )
        {
            it_per_id.m_solveFlag = 2;
        }
        else
            it_per_id.m_solveFlag = 1;
    }
}

void
FeatureManager::setDepthCamIndex( const VectorXd& x, const int camera_index )
{
    //    cout << " setDepth " << endl;
    int feature_index = -1;
    for ( auto& it_per_id : feature )
    {
        it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );

        if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;

        int camera_id = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId;

        if ( camera_id != camera_index )
            continue;

#if INV_DEPTH
        it_per_id.m_depth = 1.0 / x( ++feature_index );
#else
        it_per_id.estimated_depth  = x( ++feature_index );
#endif

        if ( it_per_id.m_depth < 0 )
        {
            it_per_id.m_solveFlag = 2;
        }
        else
            it_per_id.m_solveFlag = 1;
    }
}

void
FeatureManager::removeFailures( )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;

        if ( it->m_solveFlag == 2 )
            feature.erase( it );
    }
}

void
FeatureManager::clearDepth( )
{
    for ( auto& it_per_id : feature )
    {
        it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );

        if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;

#if INV_DEPTH
        it_per_id.m_depth = -1.0;
#else
        it_per_id.estimated_depth  = -1;
#endif
    }
}

void
FeatureManager::clearDepthCamIndex( const int camera_index )
{
    for ( auto& it_per_id : feature )
    {
        it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );

        if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;

        int camera_id = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId;

        if ( camera_id != camera_index )
            continue;

#if INV_DEPTH
        it_per_id.m_depth = -1.0;
#else
        it_per_id.estimated_depth  = -1.0;
#endif
    }
}

VectorXd
FeatureManager::getDepth( int& num_of_depth, std::vector< int >& camera_ids )
{
    //    cout << " getDepthVector " << endl;
    VectorXd dep_vec( getFeatureCount( ) );
    int feature_index = -1;
    num_of_depth      = 0;
    camera_ids.clear( );
    for ( auto& it_per_id : feature )
    {
        it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );
        if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;

        int camera_id = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId;

#if INV_DEPTH
        dep_vec( ++feature_index ) = 1. / it_per_id.m_depth;
#else
        dep_vec( ++feature_index ) = it_per_id.estimated_depth;
#endif
        camera_ids.push_back( camera_id );
        ++num_of_depth;
    }
    return dep_vec;
}

Eigen::VectorXd
FeatureManager::getDepthCamIndex( int& num_of_depth, int camera_index )
{
    //    cout << " getDepthVector " << endl;
    VectorXd dep_vec( getFeatureCount( ) );
    int feature_index = -1;
    num_of_depth      = 0;
    for ( auto& it_per_id : feature )
    {
        it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );
        if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;

        int camera_id = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId;

        if ( camera_id != camera_index )
            continue;

#if INV_DEPTH
        dep_vec( ++feature_index ) = 1. / it_per_id.m_depth;
#else
        dep_vec( ++feature_index ) = it_per_id.estimated_depth;
#endif
        ++num_of_depth;
    }
    return dep_vec;
}

void
FeatureManager::triangulate( const std::vector< Tf > Tf_wi, const std::vector< Tf > Tf_ic )
{

    int cnt_pts = 0;
    for ( auto& it_per_id : feature )
    {
        //   calculated features do not need triangulate
        if ( it_per_id.m_depth > 0 )
            continue;

        // enough measurements for triangulate
        if ( !( ( it_per_id.m_fPerFrame.size( ) >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) // multi-frame time
                || it_per_id.m_fPerFrame.begin( )->m_numOfMeas > 1 ) ) // multi-camera
            continue;

        int imu_i = it_per_id.m_startFrame, imu_j = imu_i - 1;

        // first observation camera id, and camera id
        int cam_id_i_1st = it_per_id.m_fPerFrame.begin( )->m_fPerCam.begin( )->m_cameraId;
        Tf tf_wi_i       = Tf_wi[imu_i];
        Tf tf_wc_i       = tf_wi_i + Tf_ic[cam_id_i_1st];

        Eigen::MatrixXd svd_A( 3 * 1, 4 );
        int svd_idx  = 0;
        int cnt_meas = 0;
        // ROS_DEBUG( "it_per_frame %d ", it_per_id.m_fPerFrame.size( ) );
        for ( auto& it_per_frame : it_per_id.m_fPerFrame )
        {
            imu_j++;
            // ROS_DEBUG( "it_per_cam %d ", it_per_frame.m_fPerCam.size( ) );

            for ( auto& it_per_cam : it_per_frame.m_fPerCam )
            {
                ++cnt_meas;

                int cam_id = it_per_cam.m_cameraId;

                Tf tf_wc_j = Tf_wi[imu_j] + Tf_ic[cam_id];
                Tf tf_ij   = tf_wc_i.inverse( ) + tf_wc_j;

                svd_A.conservativeResize( 3 * cnt_meas, 4 );

                Eigen::Matrix< double, 3, 4 > P;
                P.leftCols< 3 >( )  = tf_ij.R.transpose( );
                P.rightCols< 1 >( ) = -tf_ij.R.transpose( ) * tf_ij.T;

                Eigen::Vector3d f = it_per_cam.m_measPoint.normalized( );

                svd_A.row( svd_idx++ ) = f[1] * P.row( 2 ) - f[2] * P.row( 1 );
                svd_A.row( svd_idx++ ) = f[2] * P.row( 0 ) - f[0] * P.row( 2 );
                svd_A.row( svd_idx++ ) = f[0] * P.row( 1 ) - f[1] * P.row( 0 );
            }

            if ( imu_i == imu_j )
                continue;
        }
        ++cnt_pts;

        Eigen::Vector4d svd_V = Eigen::JacobiSVD< Eigen::MatrixXd >( svd_A, //
                                                                     Eigen::ComputeThinV )
                                .matrixV( )
                                .rightCols< 1 >( );

        if ( svd_V.hasNaN( ) )
        {
            it_per_id.m_depth   = INIT_DEPTH;
            it_per_id.estimated = false;
            continue;
        }

        Eigen::Vector3d svd_method( svd_V[0] / svd_V[3], svd_V[1] / svd_V[3], svd_V[2] / svd_V[3] );
        //    std::cout << "svd_point " << svd_method.transpose( ) << std::endl;

        it_per_id.m_depth = svd_method.norm( );
        if ( it_per_id.m_depth < 0.2 )
        {
            it_per_id.m_depth   = INIT_DEPTH;
            it_per_id.estimated = false;
            continue;
        }
        else
        {
            it_per_id.estimated = true;
            ROS_DEBUG( "tri %d --------- %f", it_per_id.m_featureId, it_per_id.m_depth );
            continue;
        }
    }
    ROS_DEBUG_STREAM( "triangulate cnt " << cnt_pts );
}

void
FeatureManager::triangulateCamIndex( const std::vector< Tf > Tf_wi, const Tf Tf_ic )
{
    //    ROS_DEBUG_STREAM( "feature size " << feature.size( ) );
    if ( m_isParallaxEnough )
        for ( auto& it_per_id : feature )
        {
            it_per_id.m_usedNum = it_per_id.m_fPerFrame.size( );
            if ( !( it_per_id.m_usedNum >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
                continue;

            bool triangulate_solved = triangulateMono( it_per_id, Tf_wi, Tf_ic );
            if ( !triangulate_solved )
                continue;
        }
}

void
FeatureManager::removeOutlier( )
{
    ROS_BREAK( );
    int i = -1;
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;
        i += it->m_usedNum != 0;
        if ( it->m_usedNum != 0 && it->m_isOutlier == true )
        {
            feature.erase( it );
        }
    }
}

void
FeatureManager::removeBackShiftDepthCamIndex( Eigen::Matrix3d marg_R,
                                              Eigen::Vector3d marg_P,
                                              Eigen::Matrix3d new_R,
                                              Eigen::Vector3d new_P,
                                              int camera_index )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;
        if ( it->m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId != camera_index )
            continue;

        if ( it->m_startFrame != 0 )
            it->m_startFrame--;
        else
        {
            Eigen::Vector3d uv_i = it->m_fPerFrame[0].m_fPerCam.at( 0 ).m_measPoint;
            it->m_fPerFrame.erase( it->m_fPerFrame.begin( ) );

            if ( it->m_fPerFrame.size( ) < 2 )
            {
                feature.erase( it );
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i   = uv_i * it->m_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j   = new_R.transpose( ) * ( w_pts_i - new_P );
                double dep_j            = pts_j.norm( );
                if ( dep_j > 0 /* & dep_j < MAX_DEPTH */ )
                    it->m_depth = dep_j;
                else
                    it->m_depth = INIT_DEPTH;
            }
        }
    }
}

void
FeatureManager::removeBackShiftDepthCamIndex( Tf tf_wci_marg, Tf tf_wcj_marg, int camera_index )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;
        if ( it->m_fPerFrame[0].m_fPerCam[0].m_cameraId != camera_index )
            continue;

        if ( it->m_startFrame != 0 )
            it->m_startFrame--;
        else
        {
            Eigen::Vector3d uv_i = it->m_fPerFrame[0].m_fPerCam[0].m_measPoint;
            it->m_fPerFrame.erase( it->m_fPerFrame.begin( ) );

            if ( it->m_fPerFrame.size( ) < 2 )
            {
                feature.erase( it );
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i   = uv_i * it->m_depth;
                Eigen::Vector3d w_pts_i = tf_wci_marg.transVector( pts_i );
                Eigen::Vector3d pts_j   = tf_wcj_marg.inverse( ).transVector( w_pts_i );

                double dep_j = pts_j.norm( );
                if ( dep_j > 0 /* & dep_j < MAX_DEPTH */ )
                    it->m_depth = dep_j;
                else
                    it->m_depth = INIT_DEPTH;
            }
        }
    }
}

void
FeatureManager::removeBack( )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;

        if ( it->m_startFrame != 0 )
            it->m_startFrame--;
        else
        {
            it->m_fPerFrame.erase( it->m_fPerFrame.begin( ) );
            if ( it->m_fPerFrame.size( ) == 0 )
                feature.erase( it );
        }
    }
}

void
FeatureManager::removeFront( int frame_count )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;

        if ( it->m_startFrame == frame_count )
        {
            it->m_startFrame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->m_startFrame;
            it->m_fPerFrame.erase( it->m_fPerFrame.begin( ) + j );
            if ( it->m_fPerFrame.size( ) == 0 )
                feature.erase( it );
        }
    }
}

void
FeatureManager::removeMonoPointsCamIndex( int camera_index )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;

        if ( it->m_fPerFrame[0].m_numOfMeas > 1 )
            continue;

        int cam_id = it->m_fPerFrame[0].m_fPerCam.at( 0 ).m_cameraId;
        if ( cam_id == camera_index )
        {
            feature.erase( it );
        }
    }
}

void
FeatureManager::removeMonoPoints( )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;

        if ( it->m_fPerFrame[0].m_numOfMeas > 1 )
        {
            continue;
        }
        else
        {
            feature.erase( it );
        }
    }
}

void
FeatureManager::removeStereoPoints( )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;

        if ( it->m_fPerFrame[0].m_numOfMeas > 1 )
        {
            continue;
        }
        else
        {
            feature.erase( it );
        }
    }
}

void
FeatureManager::removeBackShiftDepth( std::vector< Tf > tf_wci_marg, std::vector< Tf > tf_wcj_marg )
{
    for ( auto it = feature.begin( ), it_next = feature.begin( ); it != feature.end( ); it = it_next )
    {
        it_next++;

        int cam_index = it->m_fPerFrame[0].m_fPerCam[0].m_cameraId;

        if ( it->m_startFrame != 0 )
            it->m_startFrame--;
        else
        {
            Eigen::Vector3d uv_i = it->m_fPerFrame[0].m_fPerCam[0].m_measPoint;
            it->m_fPerFrame.erase( it->m_fPerFrame.begin( ) );

            if ( it->m_fPerFrame.size( ) < 2 )
            {
                feature.erase( it );
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i   = uv_i * it->m_depth;
                Eigen::Vector3d w_pts_i = tf_wci_marg[cam_index].transVector( pts_i );
                Eigen::Vector3d pts_j = tf_wcj_marg[cam_index].inverse( ).transVector( w_pts_i );

                double dep_j = pts_j.norm( );
                if ( dep_j > 0 /* & dep_j < MAX_DEPTH */ )
                    it->m_depth = dep_j;
                else
                    it->m_depth = INIT_DEPTH;
            }
        }
    }
}

void
FeatureManager::setTfrl( const Tf& value )
{
    tf_rl = value;
    ROS_DEBUG_STREAM( "set tf_rl: "
                      << "\n"
                      << tf_rl );
}

void
FeatureManager::setBaseline( double value )
{
    baseline = value;
    ROS_DEBUG( "set baseline: %f", baseline );
}

FeatureManager&
FeatureManager::operator=( const FeatureManager& other )
{
    if ( this != &other )
    {
        feature        = other.feature;
        last_track_num = other.last_track_num;
        tf_rl          = other.tf_rl;
        baseline       = other.baseline;
    }
    return *this;
}

bool
FeatureManager::triangulateStereo( FeaturePerId& feature, const std::vector< Tf > Tf_ics, const double baseline )
{

    auto& it_per_camera = feature.m_fPerFrame.begin( )->m_fPerCam;

    Vector3d point_l, point_r;

    point_l = it_per_camera[0].m_measPoint;
    point_r = it_per_camera[1].m_measPoint;

    Eigen::MatrixXd svd_A( 3 * 2, 4 );
    int svd_idx = 0;

    Eigen::Matrix< double, 3, 4 > P;

    Eigen::Vector3d f = point_l.normalized( );

    Tf tf_;
    tf_.setZero( );

    P.leftCols< 3 >( )  = tf_.R.transpose( );
    P.rightCols< 1 >( ) = -tf_.R.transpose( ) * tf_.T;

    svd_A.row( svd_idx++ ) = f[1] * P.row( 2 ) - f[2] * P.row( 1 );
    svd_A.row( svd_idx++ ) = f[2] * P.row( 0 ) - f[0] * P.row( 2 );
    svd_A.row( svd_idx++ ) = f[0] * P.row( 1 ) - f[1] * P.row( 0 );

    Tf tf_rl = Tf_ics[it_per_camera[1].m_cameraId].inverse( ) + Tf_ics[it_per_camera[0].m_cameraId];
    Tf tf_lr = tf_rl.inverse( );

    P.leftCols< 3 >( )  = tf_lr.R.transpose( );
    P.rightCols< 1 >( ) = -tf_lr.R.transpose( ) * tf_lr.T;

    f = point_r.normalized( );

    svd_A.row( svd_idx++ ) = f[1] * P.row( 2 ) - f[2] * P.row( 1 );
    svd_A.row( svd_idx++ ) = f[2] * P.row( 0 ) - f[0] * P.row( 2 );
    svd_A.row( svd_idx++ ) = f[0] * P.row( 1 ) - f[1] * P.row( 0 );

    ROS_ASSERT( svd_idx == svd_A.rows( ) );

    Eigen::Vector4d svd_V = Eigen::JacobiSVD< Eigen::MatrixXd >( svd_A, //
                                                                 Eigen::ComputeFullV )
                            .matrixV( )
                            .rightCols< 1 >( );

    Eigen::Vector3d svd_method( svd_V[0] / svd_V[3], svd_V[1] / svd_V[3], svd_V[2] / svd_V[3] );

    ROS_DEBUG_STREAM( "stereo========= " << svd_method.transpose( ) );

    feature.m_depth = svd_method.norm( );

    if ( feature.m_depth < 0.3 )
    {
        feature.m_depth   = INIT_DEPTH;
        feature.estimated = false;
        return false;
    }
    else
    {
        feature.estimated = true;
        return true;
    }
}

bool
FeatureManager::triangulateMono( FeaturePerId& feature, const std::vector< Tf > Tf_wi, const Tf Tf_ic )
{

    int imu_i = feature.m_startFrame, imu_j = imu_i - 1;

    //        cout << "camera_index " << camera_index << endl;
    Eigen::MatrixXd svd_A( 3 * feature.m_fPerFrame.size( ), 4 );
    int svd_idx = 0;

    Tf tf_wi_i = Tf_wi[imu_i];
    Tf tf_wc_i = tf_wi_i + Tf_ic;

    for ( auto& it_per_frame : feature.m_fPerFrame )
    {
        imu_j++;

        Tf tf_wc_j = Tf_wi[imu_j] + Tf_ic;
        Tf tf_ij   = tf_wc_i.inverse( ) + tf_wc_j;

        Eigen::Matrix< double, 3, 4 > P;
        P.leftCols< 3 >( )  = tf_ij.R.transpose( );
        P.rightCols< 1 >( ) = -tf_ij.R.transpose( ) * tf_ij.T;

        Eigen::Vector3d f = it_per_frame.m_fPerCam.at( 0 ).m_measPoint.normalized( );

        svd_A.row( svd_idx++ ) = f[1] * P.row( 2 ) - f[2] * P.row( 1 );
        svd_A.row( svd_idx++ ) = f[2] * P.row( 0 ) - f[0] * P.row( 2 );
        svd_A.row( svd_idx++ ) = f[0] * P.row( 1 ) - f[1] * P.row( 0 );

        if ( imu_i == imu_j )
            continue;
    }

    ROS_ASSERT( svd_idx == svd_A.rows( ) );
    Eigen::Vector4d svd_V = Eigen::JacobiSVD< Eigen::MatrixXd >( svd_A, //
                                                                 Eigen::ComputeThinV )
                            .matrixV( )
                            .rightCols< 1 >( );

    if ( svd_V.hasNaN( ) )
    {
        feature.m_depth   = INIT_DEPTH;
        feature.estimated = false;
        return false;
    }

    Eigen::Vector3d svd_method( svd_V[0] / svd_V[3], svd_V[1] / svd_V[3], svd_V[2] / svd_V[3] );
    //    std::cout << "svd_point " << svd_method.transpose( ) << std::endl;

    feature.m_depth = svd_method.norm( );
    ROS_DEBUG( "mono %d --------- %f", feature.m_featureId, feature.m_depth );

    if ( feature.m_depth < 0.1 )
    {
        feature.m_depth   = INIT_DEPTH;
        feature.estimated = false;
        return false;
    }
    else
    {
        feature.estimated = true;
        return true;
    }
}

double
FeatureManager::compensatedParallaxRad( const FeaturePerId& it_per_id, int frame_count )
{
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame& frame_i = it_per_id.m_fPerFrame[frame_count - 2 - it_per_id.m_startFrame];
    const FeaturePerFrame& frame_j = it_per_id.m_fPerFrame[frame_count - 1 - it_per_id.m_startFrame];

    Vector3d p_i = frame_i.m_fPerCam[0].m_measPoint;
    Vector3d p_j = frame_j.m_fPerCam[0].m_measPoint;

    double acos_angle = p_i.dot( p_j );
    double angle      = acos( acos_angle );

    if ( std::isnan( angle ) )
    {
        ROS_ERROR_STREAM( "frames " << it_per_id.m_fPerFrame.size( ) << " meas "
                                    << it_per_id.m_fPerFrame[0].m_numOfMeas
                                    << " cam "
                                    << it_per_id.m_fPerFrame[0].m_fPerCam[0].m_cameraId );

        for ( auto& pt : it_per_id.m_fPerFrame )
            ROS_ERROR_STREAM( "pt " << pt.m_fPerCam[0].m_measPoint.transpose( ) );

        ROS_ERROR_STREAM( "acos " << acos_angle );
        ROS_ERROR_STREAM( "angle " << angle );

        ROS_ERROR( "compensatedParallaxRad NaN!!!" );
        angle = 0.0;
    }

    return angle;
}
