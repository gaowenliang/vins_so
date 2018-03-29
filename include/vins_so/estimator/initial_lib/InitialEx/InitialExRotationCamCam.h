#ifndef INITIALEXROTATION_CamCam_H
#define INITIALEXROTATION_CamCam_H

#include <ceres/ceres.h>
#include <code_utils/math_utils/math_utils.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <vector>
#include <vins_so/type/transform.h>

using namespace Eigen;
using namespace std;

namespace initialEx
{

class InitialExRotationCamCam
{
    class EpipolarError
    {
        public:
        EpipolarError( const Eigen::Vector3d& cam0_point, const Eigen::Vector3d& cam1_point );
        double getEpipolarError( const Tf& tf_c1c0 );

        template< typename T >
        bool operator( )( const T* const ex_paramt, T* residuals ) const
        {
            Eigen::Matrix< T, 3, 1 > _t_c1_c0;
            _t_c1_c0[0] = ex_paramt[0];
            _t_c1_c0[1] = ex_paramt[1];
            _t_c1_c0[2] = ex_paramt[2];

            Eigen::Quaternion< T > _q_c1c0w( ex_paramt[6], ex_paramt[3], ex_paramt[4], ex_paramt[5] );
            _q_c1c0w.normalize( );

            Eigen::Matrix< T, 3, 3 > R_c1c0;
            R_c1c0 = _q_c1c0w.toRotationMatrix( );

            Eigen::Matrix< T, 3, 1 > v_cam0;
            v_cam0( 0 ) = T( cam0_point_( 0 ) );
            v_cam0( 1 ) = T( cam0_point_( 1 ) );
            v_cam0( 2 ) = T( cam0_point_( 2 ) );

            Eigen::Matrix< T, 3, 1 > v_cam1;
            v_cam1( 0 ) = T( cam1_point_( 0 ) );
            v_cam1( 1 ) = T( cam1_point_( 1 ) );
            v_cam1( 2 ) = T( cam1_point_( 2 ) );

            T error = v_cam1.transpose( )
                      * math_utils::vectorToSkew( _t_c1_c0.normalized( ) ) * R_c1c0 * v_cam0;
            residuals[0] = error * error;

            return true;
        }

        Eigen::Vector3d cam0_point_; // key feature in camera0
        Eigen::Vector3d cam1_point_; // key feature in camera1
    };

    public:
    InitialExRotationCamCam( );
    InitialExRotationCamCam( const Matrix3d& Rc1c0, const Vector3d& Tc1c0 );
    InitialExRotationCamCam( const Tf& tf_c1c0 );
    ~InitialExRotationCamCam( ) { m_ptsCorres.clear( ); }

    bool CalibrationExRotation( vector< pair< Vector3d, Vector3d > > corres, Tf& tf_cc_result );

    public:
    bool Done( ) const;
    void setAsDone( );
    void setRT( const Tf& tf_c1c0 );
    void setRT( const Matrix3d& Rc1c0, const Vector3d& Tc1c0 );
    Tf Tfc1c0( ) const;
    Tf Tfc0c1( ) const;

    private:
    bool solveRelativeRT( );
    double calcEpipolarError( );

    vector< pair< Vector3d, Vector3d > > m_ptsCorres;
    bool done;
    int m_frameCount;

    Tf m_tf_c0c1, m_tf_c1c0;
};
typedef boost::shared_ptr< InitialExRotationCamCam > InitialExRotationCamCamPtr;
}

#endif // INITIALEXROTATION_CamCam_H
