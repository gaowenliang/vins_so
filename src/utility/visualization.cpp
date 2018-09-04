#include "vins_so/utility/visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path, pub_loop_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;

std::vector< ros::Publisher > pub_camera_poses, pub_camera_pose_visuals;
ros::Publisher pub_pose_graph;
nav_msgs::Path path, loop_path;
// CameraPoseVisualization cameraposevisual( 0, 0, 1, 1 );
std::vector< CameraPoseVisualization > cameraposevisuals;
CameraPoseVisualization keyframebasevisual( 0.0, 0.0, 1.0, 1.0 );
static double sum_of_path = 0;
static Vector3d last_path( 0.0, 0.0, 0.0 );

void
registerPub( ros::NodeHandle& n )
{
    pub_latest_odometry = n.advertise< nav_msgs::Odometry >( "imu_propagate", 1000 );
    pub_path            = n.advertise< nav_msgs::Path >( "path_no_loop", 1000 );
    pub_loop_path       = n.advertise< nav_msgs::Path >( "path", 1000 );
    pub_odometry        = n.advertise< nav_msgs::Odometry >( "odometry", 1000 );
    pub_point_cloud     = n.advertise< sensor_msgs::PointCloud >( "point_cloud", 1000 );
    pub_margin_cloud    = n.advertise< sensor_msgs::PointCloud >( "history_cloud", 1000 );
    pub_key_poses       = n.advertise< visualization_msgs::Marker >( "key_poses", 1000 );

    pub_pose_graph = n.advertise< visualization_msgs::MarkerArray >( "pose_graph", 1000 );

    for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
    {
        std::stringstream ss_num;
        ss_num << camera_index;
        ros::Publisher pub_camera_pose
        = n.advertise< geometry_msgs::PoseStamped >( "camera_pose" + ss_num.str( ), 1000 );
        pub_camera_poses.push_back( pub_camera_pose );

        ros::Publisher pub_camera_pose_visual
        = n.advertise< visualization_msgs::MarkerArray >( "camera_pose_visual" + ss_num.str( ), 1000 );
        pub_camera_pose_visuals.push_back( pub_camera_pose_visual );

        CameraPoseVisualization cameraposevisual( 0.0, 0.0, 1.0, 1.0 );
        cameraposevisual.setScale( 0.2 );
        cameraposevisual.setLineWidth( 0.01 );
        cameraposevisuals.push_back( cameraposevisual );
    }

    keyframebasevisual.setScale( 0.1 );
    keyframebasevisual.setLineWidth( 0.01 );
}

void
pubLatestOdometry( const Eigen::Vector3d& P,
                   const Eigen::Quaterniond& Q,
                   const Eigen::Vector3d& V,
                   const std_msgs::Header& header )
{
    Eigen::Quaterniond quadrotor_Q = Q;

    nav_msgs::Odometry odometry;
    odometry.header                  = header;
    odometry.header.frame_id         = "world";
    odometry.pose.pose.position.x    = P.x( );
    odometry.pose.pose.position.y    = P.y( );
    odometry.pose.pose.position.z    = P.z( );
    odometry.pose.pose.orientation.x = quadrotor_Q.x( );
    odometry.pose.pose.orientation.y = quadrotor_Q.y( );
    odometry.pose.pose.orientation.z = quadrotor_Q.z( );
    odometry.pose.pose.orientation.w = quadrotor_Q.w( );
    odometry.twist.twist.linear.x    = V.x( );
    odometry.twist.twist.linear.y    = V.y( );
    odometry.twist.twist.linear.z    = V.z( );
    pub_latest_odometry.publish( odometry );
}

void
printStatistics( const Estimator& estimator, double t )
{
    if ( estimator.solver_flag != Estimator::SolverFlag::NONLINEAR )
        return;

    ROS_INFO_STREAM( "position: " << estimator.last_Pose.T.transpose( ) );
    ROS_DEBUG_STREAM( "orientation: " << estimator.last_vel.transpose( ) );

    for ( int i = 0; i < NUM_OF_CAM; i++ )
    {
        // ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM( "extirnsic tic: " << estimator.tf_ics[i].T.transpose( ) );
        ROS_DEBUG_STREAM( "          ric: " << Utility::R2ypr( estimator.tf_ics[i].R ).transpose( ) );
        if ( ESTIMATE_EXTRINSIC )
        {
            cv::FileStorage fs( EX_CALIB_RESULT_PATHS[i], cv::FileStorage::WRITE );

            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.tf_ics[i].R;
            eigen_T = estimator.tf_ics[i].T;

            cv::Mat cv_R, cv_T;
            cv::eigen2cv( eigen_R, cv_R );
            cv::eigen2cv( eigen_T, cv_T );
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release( );
        }
    }

    static double sum_of_time     = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG( "vo solver costs: %f ms", t );
    ROS_DEBUG( "average of time %f ms", sum_of_time / sum_of_calculation );

    sum_of_path += ( estimator.last_Pose.T - last_path ).norm( );
    last_path = estimator.last_Pose.T;
    ROS_DEBUG( "sum of path %f", sum_of_path );
}

void
pubOdometry( const Estimator& estimator,
             const std_msgs::Header& header,
             Eigen::Vector3d loop_correct_t,
             Eigen::Matrix3d loop_correct_r )
{
    if ( estimator.solver_flag == Estimator::SolverFlag::NONLINEAR )
    {
        nav_msgs::Odometry odometry;
        odometry.header                  = header;
        odometry.header.frame_id         = "world";
        odometry.child_frame_id          = "world";
        odometry.pose.pose.position.x    = estimator.last_Pose.T.x( );
        odometry.pose.pose.position.y    = estimator.last_Pose.T.y( );
        odometry.pose.pose.position.z    = estimator.last_Pose.T.z( );
        odometry.pose.pose.orientation.x = Quaterniond( estimator.last_Pose.R ).x( );
        odometry.pose.pose.orientation.y = Quaterniond( estimator.last_Pose.R ).y( );
        odometry.pose.pose.orientation.z = Quaterniond( estimator.last_Pose.R ).z( );
        odometry.pose.pose.orientation.w = Quaterniond( estimator.last_Pose.R ).w( );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header          = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose            = odometry.pose.pose;
        path.header                  = header;
        path.header.frame_id         = "world";
        path.poses.push_back( pose_stamped );
        pub_path.publish( path );

        Vector3d correct_t;
        Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = loop_correct_r * estimator.last_Pose.T + loop_correct_t;
        correct_q = loop_correct_r * estimator.last_Pose.R;
        correct_v = loop_correct_r * estimator.last_vel;
        odometry.pose.pose.position.x    = correct_t.x( );
        odometry.pose.pose.position.y    = correct_t.y( );
        odometry.pose.pose.position.z    = correct_t.z( );
        odometry.pose.pose.orientation.x = correct_q.x( );
        odometry.pose.pose.orientation.y = correct_q.y( );
        odometry.pose.pose.orientation.z = correct_q.z( );
        odometry.pose.pose.orientation.w = correct_q.w( );
        odometry.twist.twist.linear.x    = correct_v( 0 );
        odometry.twist.twist.linear.y    = correct_v( 1 );
        odometry.twist.twist.linear.z    = correct_v( 2 );
        pub_odometry.publish( odometry );

        pose_stamped.pose         = odometry.pose.pose;
        loop_path.header          = header;
        loop_path.header.frame_id = "world";
        loop_path.poses.push_back( pose_stamped );
        pub_loop_path.publish( loop_path );

        // write result to file
        ofstream foutC( VINS_RESULT_PATH, ios::app );
        foutC.setf( ios::fixed, ios::floatfield );
        foutC.precision( 0 );
        foutC << header.stamp.toSec( ) * 1e9 << ",";
        foutC.precision( 5 );
        foutC << correct_t.x( ) << "," << correct_t.y( ) << "," << correct_t.z( ) << ","
              << correct_q.w( ) << "," << correct_q.x( ) << "," << correct_q.y( ) << ","
              << correct_q.z( ) << "," << correct_v( 0 ) << "," << correct_v( 1 ) << ","
              << correct_v( 2 ) << "," << endl;
        foutC.close( );
    }
}

void
pubKeyPoses( const Estimator& estimator,
             const std_msgs::Header& header,
             Eigen::Vector3d loop_correct_t,
             Eigen::Matrix3d loop_correct_r )
{
    if ( estimator.key_poses.size( ) == 0 )
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header             = header;
    key_poses.header.frame_id    = "world";
    key_poses.ns                 = "key_poses";
    key_poses.type               = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action             = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime           = ros::Duration( );

    // static int key_poses_id = 0;
    key_poses.id      = 0; // key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for ( int i = 0; i <= WINDOW_SIZE; i++ )
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose  = loop_correct_r * estimator.key_poses[i] + loop_correct_t;
        pose_marker.x = correct_pose.x( );
        pose_marker.y = correct_pose.y( );
        pose_marker.z = correct_pose.z( );
        key_poses.points.push_back( pose_marker );
    }
    pub_key_poses.publish( key_poses );
}

void
pubCameraPose( const Estimator& estimator,
               const std_msgs::Header& header,
               Eigen::Vector3d loop_correct_t,
               Eigen::Matrix3d loop_correct_r )
{
    int idx2 = WINDOW_SIZE - 1;
    if ( estimator.solver_flag == Estimator::SolverFlag::NONLINEAR )
    {
        int i = idx2;

        // TODO: support multipe camera pose
        int camera_index = 0;
        for ( ; camera_index < NUM_OF_CAM; ++camera_index )
        {
            Vector3d P = estimator.pWindow->Pose[i].T
                         + estimator.pWindow->Pose[i].R * estimator.tf_ics[camera_index].T;
            Quaterniond R
            = Quaterniond( estimator.pWindow->Pose[i].R * estimator.tf_ics[camera_index].R );

            P = ( loop_correct_r * estimator.pWindow->Pose[i].T + loop_correct_t )
                + ( loop_correct_r * estimator.pWindow->Pose[i].R )
                  * estimator.tf_ics[camera_index].T;
            R = Quaterniond( ( loop_correct_r * estimator.pWindow->Pose[i].R )
                             * estimator.tf_ics[camera_index].R );

            // TODO: support multipe camera pose
            geometry_msgs::PoseStamped camera_pose;
            camera_pose.header = header;
            // camera_pose.header.frame_id = std::to_string(
            // estimator.Headers[i].stamp.toNSec( )
            // );
            stringstream ss;
            ss << camera_index;
            camera_pose.header.frame_id    = std::string( "camera" + ss.str( ) );
            camera_pose.pose.position.x    = P.x( );
            camera_pose.pose.position.y    = P.y( );
            camera_pose.pose.position.z    = P.z( );
            camera_pose.pose.orientation.w = R.w( );
            camera_pose.pose.orientation.x = R.x( );
            camera_pose.pose.orientation.y = R.y( );
            camera_pose.pose.orientation.z = R.z( );

            pub_camera_poses.at( camera_index ).publish( camera_pose );
            cameraposevisuals.at( camera_index ).reset( );
            cameraposevisuals.at( camera_index ).add_pose( P, R );
            camera_pose.header.frame_id = "world";
            cameraposevisuals.at( camera_index )
            .publish_by( pub_camera_pose_visuals.at( camera_index ), camera_pose.header );
        }
    }
}

void
pubPointCloud( const Estimator& estimator,
               const std_msgs::Header& header,
               Eigen::Vector3d loop_correct_t,
               Eigen::Matrix3d loop_correct_r )
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header      = header;
    loop_point_cloud.header = header;

    for ( auto& it_per_id : estimator.pWindow->m_featureManager.feature )
    {
        int used_num;
        used_num = it_per_id.m_fPerFrame.size( );
        if ( !( used_num >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;
        if ( it_per_id.m_startFrame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.m_solveFlag != 1 )
            continue;
        int imu_i = it_per_id.m_startFrame;
        int cam   = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId;

        Vector3d pts_i
        = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_measPoint * it_per_id.m_depth;
        Vector3d w_pts_i = loop_correct_r * estimator.pWindow->Pose[imu_i].R
                           * ( estimator.tf_ics[cam].R * pts_i + estimator.tf_ics[cam].T )
                           + loop_correct_r * estimator.pWindow->Pose[imu_i].T + loop_correct_t;

        geometry_msgs::Point32 p;
        p.x = w_pts_i( 0 );
        p.y = w_pts_i( 1 );
        p.z = w_pts_i( 2 );
        point_cloud.points.push_back( p );
    }
    pub_point_cloud.publish( point_cloud );

    // pub margined potin
    sensor_msgs::PointCloud margin_cloud, loop_margin_cloud;
    margin_cloud.header      = header;
    loop_margin_cloud.header = header;

    for ( auto& it_per_id : estimator.pWindow->m_featureManager.feature )
    {
        int used_num;
        used_num = it_per_id.m_fPerFrame.size( );
        if ( !( used_num >= 2 && it_per_id.m_startFrame < WINDOW_SIZE - 2 ) )
            continue;
        // if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag !=
        // 1)
        //        continue;

        if ( it_per_id.m_startFrame == 0 && it_per_id.m_fPerFrame.size( ) <= 2 && it_per_id.m_solveFlag == 1 )
        {
            int imu_i = it_per_id.m_startFrame;
            int cam   = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_cameraId;

            Vector3d pts_i
            = it_per_id.m_fPerFrame.at( 0 ).m_fPerCam.at( 0 ).m_measPoint * it_per_id.m_depth;
            Vector3d w_pts_i = loop_correct_r * estimator.pWindow->Pose[imu_i].R
                               * ( estimator.tf_ics[cam].R * pts_i + estimator.tf_ics[cam].T )
                               + loop_correct_r * estimator.pWindow->Pose[imu_i].T + loop_correct_t;

            geometry_msgs::Point32 p;
            p.x = w_pts_i( 0 );
            p.y = w_pts_i( 1 );
            p.z = w_pts_i( 2 );
            margin_cloud.points.push_back( p );
        }
    }
    pub_margin_cloud.publish( margin_cloud );
}

void
pubPoseGraph( CameraPoseVisualization* posegraph, const std_msgs::Header& header )
{
    posegraph->publish_by( pub_pose_graph, header );
}

void
updateLoopPath( nav_msgs::Path _loop_path )
{
    loop_path = _loop_path;
}

void
pubTF( const Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t, Eigen::Matrix3d loop_correct_r )
{
    if ( estimator.solver_flag != Estimator::SolverFlag::NONLINEAR )
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = loop_correct_r * estimator.last_Pose.T + loop_correct_t;
    correct_q = loop_correct_r * estimator.last_Pose.R;

    transform.setOrigin( tf::Vector3( correct_t( 0 ), correct_t( 1 ), correct_t( 2 ) ) );
    q.setW( correct_q.w( ) );
    q.setX( correct_q.x( ) );
    q.setY( correct_q.y( ) );
    q.setZ( correct_q.z( ) );
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, header.stamp, "world", "body" ) );

    // camera frame
    for ( int camera_index = 0; camera_index < NUM_OF_CAM; ++camera_index )
    {
        transform.setOrigin( tf::Vector3( estimator.tf_ics[camera_index].T.x( ),
                                          estimator.tf_ics[camera_index].T.y( ),
                                          estimator.tf_ics[camera_index].T.z( ) ) );
        q.setW( Quaterniond( estimator.tf_ics[camera_index].R ).w( ) );
        q.setX( Quaterniond( estimator.tf_ics[camera_index].R ).x( ) );
        q.setY( Quaterniond( estimator.tf_ics[camera_index].R ).y( ) );
        q.setZ( Quaterniond( estimator.tf_ics[camera_index].R ).z( ) );
        transform.setRotation( q );
        std::stringstream ss;
        ss << camera_index;
        br.sendTransform( tf::StampedTransform( transform, //
                                                header.stamp,
                                                "body",
                                                "camera" + ss.str( ) ) );
    }
}
