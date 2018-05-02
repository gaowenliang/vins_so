#ifndef SLIDEWINDOWPOSE_H
#define SLIDEWINDOWPOSE_H

#include "slidewindowbase.h"

namespace slidewindow
{

class SlideWindowPose : public SlideWindowBase
{
    public:
    SlideWindowPose( int window_size, int camera_num );
    void clearWindow( );

    SlideWindowPose& operator=( const SlideWindowPose& other );
    Tf getTfIndex( const int index ) const { return Pose[index]; }
    void setTfIndex( const int index, Tf _tf ) { Pose[index] = _tf; }

    void slideWindow( const bool shift_depth, const std::vector< Tf > tf_ic );

    MarginalizationFlag addFeaturesToWindow( int _frame_count, //
                                             const FeatureData& _image );

    void slideWindowOld( const Tf& back_Tf_wi, const std::vector< Tf >& Tf_ic, bool shift_depth );
    void slideWindowNew( );

    int NUM_OF_CAM;
    std::vector< Tf > Pose;

    public:
    FeatureManager m_featureManager;
};

typedef boost::shared_ptr< SlideWindowPose > SlideWindowPosePtr;
}
#endif // SLIDEWINDOWPOSE_H
