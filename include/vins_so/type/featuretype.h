#ifndef FEATURETYPE_H
#define FEATURETYPE_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

struct VisualMeas
{
    VisualMeas( const int _cam_id, const Eigen::Vector3d& _pt )
    : cam_id( _cam_id )
    , err( 1.0 / 460 )
    , pt( _pt )
    {
    }
    VisualMeas( const int _cam_id, const double& _err, const Eigen::Vector3d& _pt )
    : cam_id( _cam_id )
    , err( _err )
    , pt( _pt )
    {
    }

    int cam_id;         // camera id of visual measurement
    double err;         // error angle of the measurement
    Eigen::Vector3d pt; // measurement baer vector
};

typedef std::map< int, std::vector< VisualMeas > > FeatureData;
typedef std::vector< pair< Eigen::Vector3d, Eigen::Vector3d > > PointsCorres;

class FeaturePerCamera
{
    public:
    FeaturePerCamera( int _camera_id, const Vector3d& _point )
    : m_cameraId( _camera_id )
    , m_measPoint( _point.normalized( ) )
    , m_errAngle( 1.0 / 460 )
    {
    }
    FeaturePerCamera( int _camera_id, const Vector3d& _point, const double& _error )
    : m_cameraId( _camera_id )
    , m_measPoint( _point.normalized( ) )
    , m_errAngle( _error )
    {
    }

    int m_cameraId;
    Vector3d m_measPoint;
    double m_errAngle;

    double m_parallax;
    double dep_gradient;
};

class FeaturePerFrame
{
    public:
    FeaturePerFrame( vector< FeaturePerCamera >& _feature_per_camera )
    {
        m_fPerCam   = _feature_per_camera;
        m_numOfMeas = m_fPerCam.size( );
    }

    int m_numOfMeas;
    bool m_isUsed;
    vector< FeaturePerCamera > m_fPerCam;
};

class FeaturePerId
{
    public:
    int m_featureId;
    int m_startFrame;
    vector< FeaturePerFrame > m_fPerFrame;
    int m_usedNum;
    bool m_isOutlier;
    bool m_isMargin;
    double m_depth;               // the depth of feature, in first capture camera frame
    Eigen::Vector3d m_position_w; // the position of feature in world frame
    bool estimated;               //
    int m_solveFlag;              // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    int m_numOfFirstMeans;        // TODO

    Eigen::Vector3d gt_p;

    FeaturePerId( int _feature_id, int _start_frame )
    : m_featureId( _feature_id )
    , m_startFrame( _start_frame )
    , m_usedNum( 0 )
    , m_depth( -1.0 )
    , estimated( false )
    , m_solveFlag( 0 )
    {
    }

    Eigen::Vector3d
    toPosition( Eigen::Vector3d p_w, Eigen::Matrix3d r_wb, Eigen::Vector3d t_ic, Eigen::Matrix3d r_ic )
    {
        Vector3d p_c = m_depth * m_fPerFrame[0].m_fPerCam[0].m_measPoint;
        Vector3d p_i = r_ic * p_c + t_ic;
        return r_wb * p_i + p_w;
    }

    int endFrame( );
};

#endif // FEATURETYPE_H
