#ifndef VisualImuAlignmentVel_H
#define VisualImuAlignmentVel_H

#include "vins_so/estimator/initial_lib/InitialBase/initialbase.h"

namespace InitVio
{

class VisualImuAlignmentVel
{
    public:
    VisualImuAlignmentVel( ) {}

    bool solve( map< double, ImageImuFrame >& imageFrames,
                std::vector< Vector3d >& Bgs,
                Vector3d& g,
                VectorXd& x,
                int align_camera_index );

    private:
    // for stereo VIO initialization, do not need scale
    bool LinearAlignment( map< double, ImageImuFrame >& imageFrameAll, Vector3d& g, VectorXd& x, int camera_index );
    void RefineGravity( map< double, ImageImuFrame >& all_image_frame, Vector3d& g, VectorXd& x, int align_camera_index );

    MatrixXd TangentBasis( Vector3d& g0 );
};
}

#endif // VisualImuAlignmentVel_H
