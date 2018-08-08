#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include "vins_so/estimator/vins_parameters.h"
#include "vins_so/type/featuretype.h"
#include "vins_so/type/transform.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <list>
#include <numeric>
#include <ros/assert.h>
#include <ros/console.h>
#include <vector>

using namespace std;
using namespace Eigen;

class FeatureManager
{
    public:
    FeatureManager( );
    ~FeatureManager( );

    int getFeatureCount( );
    int getFeatureCountMono( );
    int getFeatureCountStereo( );
    void getFeatureCount( int& cntMono, int& cntStereo );

    void clearState( );

    bool addFeature( int frame_count, const FeatureData& image );
    bool addFeatureCamIndex( int frame_count, const FeatureData& image, int camera_index );
    bool addFeatureStereo( int frame_count, const FeatureData& image );
    bool addFeatureStereoIndex( int frame_count, const FeatureData& image, int camera_index, int camera_index2 );
    bool addFeatureCheckParallax( int frame_count, const FeatureData& image );
    bool addFeatureCheckParallax( int frame_count, const FeatureData& image, int max_camid );

    void debugShow( );

    vector< PointsCorres > getCorresponding( int frame_count_l, int frame_count_r );
    PointsCorres getCorrespondingCamIndex( int camera_index, int frame_count_l, int frame_count_r );
    PointsCorres getCorrespondingStereo( int frame_count );

    // void updateDepth(const VectorXd &x);
    void setDepth( const VectorXd& x );
    void setDepthCamIndex( const VectorXd& x, const int camera_index );
    void clearDepth( );
    void clearDepthCamIndex( const int camera_index );
    VectorXd getDepth( int& num_of_depth );
    VectorXd getDepthCamIndex( int& num_of_depth, int camera_index );

    void triangulate( const std::vector< Tf > Tf_wi, const std::vector< Tf > Tf_ic );
    void triangulateCamIndex( const std::vector< Tf > Tf_wi, const Tf Tf_ic );

    void removeFailures( );
    void removeBack( );
    void removeFront( int frame_count );
    void removeOutlier( );
    void removeMonoPointsCamIndex( int camera_index );
    void removeMonoPoints( );
    void removeStereoPoints( );
    void removeBackShiftDepth( std::vector< Tf > tf_wci_marg, std::vector< Tf > tf_wcj_marg );
    void removeBackShiftDepthCamIndex( Tf tf_wci_marg, Tf tf_wcj_marg, int camera_index );
    void removeBackShiftDepthCamIndex( Eigen::Matrix3d marg_R,
                                       Eigen::Vector3d marg_P,
                                       Eigen::Matrix3d new_R,
                                       Eigen::Vector3d new_P,
                                       int camera_index );

    void setTfrl( const Tf& value );
    void setBaseline( double value );

    FeatureManager& operator=( const FeatureManager& other );

    list< FeaturePerId > feature;
    int last_track_num;
    Tf tf_rl;
    double baseline;
    bool m_isParallaxEnough;

    private:
    bool triangulateStereo( FeaturePerId& feature, const std::vector< Tf > Tf_ics, const double baseline );
    bool triangulateMono( FeaturePerId& feature, const std::vector< Tf > Tf_wi, const Tf Tf_ic );

    double compensatedParallaxRad( const FeaturePerId& it_per_id, int frame_count );
};

#endif
