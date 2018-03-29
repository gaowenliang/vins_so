#ifndef VISAULIMUALIGMENT_H
#define VISAULIMUALIGMENT_H

#include "vins_so/estimator/initial_lib/InitialBase/initialbase.h"

namespace InitVio
{

class VisaulImuAligment
{
    public:
    VisaulImuAligment( ) {}

    bool VisualIMUAlignment( map< double, ImageImuFrame >& imageFrames,
                             std::vector< Vector3d >& Bgs,
                             Vector3d& g,
                             VectorXd& x,
                             int align_camera_index,
                             bool solve_scale );

    private:
    bool solveGyroscopeBias( map< double, ImageImuFrame >& imageFrameAll, std::vector< Vector3d >& Bgs );

    // for monocular VIO initialization, need scale
    bool LinearAlignmentScale( map< double, ImageImuFrame >& imageFrameAll, //
                               Vector3d& g,
                               VectorXd& x,
                               int camera_index );
    void RefineGravityScale( map< double, ImageImuFrame >& all_image_frame,
                             Vector3d& g,
                             VectorXd& x,
                             int align_camera_index );

    // for stereo VIO initialization, do not need scale
    bool LinearAlignment( map< double, ImageImuFrame >& imageFrameAll, Vector3d& g, VectorXd& x, int camera_index );
    void RefineGravity( map< double, ImageImuFrame >& all_image_frame, Vector3d& g, VectorXd& x, int align_camera_index );

    MatrixXd TangentBasis( Vector3d& g0 );
};
}

#endif // VISAULIMUALIGMENT_H
