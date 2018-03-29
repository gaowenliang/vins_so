#ifndef SLIDEWINDOWPOSEVEL_H
#define SLIDEWINDOWPOSEVEL_H

#include "slidewindowbase.h"
#include "slidewindowpose.h"

namespace slidewindow
{

class SlideWindowPoseVel : public SlideWindowPose
{
    public:
    SlideWindowPoseVel( int window_size, int camera_num );

    void clearWindow( );

    Tf poseLast( );
    Tf poseIndex( int index );
    Eigen::Vector3d lastVel( );

    void windowToDouble( double pp_Pose[][7], double Speed[][3] );
    void setPoseIndex( int index, Vector3d T_in, Quaterniond q_in );
    void setPoseIndex( int index, Vector3d T_in, Matrix3d R_in );
    void setVelIndex( int index, Vector3d vel_in );
    void doubleToWindow( const double pp_Pose[][7], const double Speed[][3] );

    SlideWindowPoseVel& operator=( const SlideWindowPoseVel& other );

    void slideWindow( const bool shift_depth, const std::vector< Tf > tf_ic );

    public:
    std::vector< Eigen::Vector3d > Vel;
};

typedef boost::shared_ptr< SlideWindowPoseVel > SlideWindowPoseVelPtr;
}
#endif // SLIDEWINDOWPOSEVEL_H
