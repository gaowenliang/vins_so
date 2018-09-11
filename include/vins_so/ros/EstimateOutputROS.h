#ifndef ESTIMATEOUTPUTROS_H
#define ESTIMATEOUTPUTROS_H

#include "CameraPoseVisualization.h"
#include "vins_so/estimator/estimator.h"
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

class EstimateOutputROS
{
    public:
    EstimateOutputROS( ros::NodeHandle& n );

    void updateLoopPath( nav_msgs::Path _loop_path );

    void printStatistics( const Estimator& estimator, double t );
    void pubOdometry( const Estimator& estimator,
                      const std_msgs::Header& header,
                      Eigen::Vector3d loop_correct_t,
                      Eigen::Matrix3d loop_correct_r );
    void pubKeyPoses( const Estimator& estimator,
                      const std_msgs::Header& header,
                      Eigen::Vector3d loop_correct_t,
                      Eigen::Matrix3d loop_correct_r );
    void pubCameraPose( const Estimator& estimator,
                        const std_msgs::Header& header,
                        Eigen::Vector3d loop_correct_t,
                        Eigen::Matrix3d loop_correct_r );
    void pubPoseGraph( CameraPoseVisualization* posegraph, const std_msgs::Header& header );
    void pubPointCloud( const Estimator& estimator,
                        const std_msgs::Header& header,
                        Eigen::Vector3d loop_correct_t,
                        Eigen::Matrix3d loop_correct_r );
    void pubTF( const Estimator& estimator,
                const std_msgs::Header& header,
                Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r );

    private:
    nav_msgs::Path path, loop_path;
    std::vector< CameraPoseVisualization > cameraPoseVisuals;

    private:
    std::vector< ros::Publisher > pub_camera_poses, pub_camera_pose_visuals;
    ros::Publisher pub_odometry;
    ros::Publisher pub_path, pub_loop_path;
    ros::Publisher pub_pose;
    ros::Publisher pub_cloud, pub_map;
    ros::Publisher pub_key_poses;
    ros::Publisher pub_ref_pose, pub_cur_pose;
    ros::Publisher pub_point_cloud, pub_margin_cloud;
    ros::Publisher pub_key;
    ros::Publisher pub_pose_graph;

    private:
    static double sum_of_path;
    static Eigen::Vector3d last_path;
    static CameraPoseVisualization keyframebasevisual;
};

#endif // ESTIMATEOUTPUTROS_H
