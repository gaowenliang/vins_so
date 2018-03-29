#ifndef INITIALEXROTATION_CamImu_H
#define INITIALEXROTATION_CamImu_H

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <vector>

using namespace Eigen;
using namespace std;

namespace initialEx
{

///
/// \brief The InitialExRotation class
///
///

/**
 * @brief The InitialExRotation class
 *      This class help you to calibrate extrinsic rotation between imu and camera when your
 * totally don't konw the extrinsic parameter
 */
class InitialExRotationCamImu
{
    public:
    InitialExRotationCamImu( );
    bool CalibrationExRotation( vector< pair< Vector3d, Vector3d > > corres,
                                Quaterniond delta_q_imu, // q_ij_imu
                                Matrix3d& Ric_result );

    /**
     * @brief Done
     * @return
     */
    bool Done( ) const;
    void setAsDone( );
    void setRic( const Matrix3d& Ric );
    Matrix3d Ric( ) const;

    private:
    Matrix3d solveRelativeR( const vector< pair< Vector3d, Vector3d > >& corres );

    double testTriangulation( const vector< cv::Point2f >& l,
                              const vector< cv::Point2f >& r,
                              cv::Mat_< double > R,
                              cv::Mat_< double > t );

    void decomposeE( cv::Mat E,
                     cv::Mat_< double >& R1,
                     cv::Mat_< double >& R2,
                     cv::Mat_< double >& t1,
                     cv::Mat_< double >& t2 );

    int m_frameCount;
    bool done;
    vector< Matrix3d > m_Rc;
    vector< Matrix3d > m_Rcg;
    vector< Matrix3d > m_Rimu;
    Matrix3d m_Ric;
};

typedef boost::shared_ptr< InitialExRotationCamImu > InitialExRotationCamImuPtr;
}

#endif // INITIALEXROTATION_CamImu_H
