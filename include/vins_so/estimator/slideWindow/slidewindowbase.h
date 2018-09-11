#ifndef SLIDEWINDOWBASE_H
#define SLIDEWINDOWBASE_H

#include "vins_so/type/transform.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <std_msgs/Header.h>

namespace slidewindow
{

enum SolverFlag
{
    INITIAL,
    NON_LINEAR
};

enum MarginalizationFlag
{
    MARGIN_OLD        = 0,
    MARGIN_SECOND_NEW = 1
};

class SlideWindowBase
{
    public:
    SlideWindowBase( int window_size );

    SlideWindowBase& operator   =( const SlideWindowBase& other );
    virtual void clearWindow( ) = 0;
    void setMargOld( );
    void setMargSecondNew( );
    void setMargFlag( const MarginalizationFlag& margFlag );
    MarginalizationFlag margFlag( ) const;
    void setStampIndex( int camera_index, double _stamp );

    public:
    int WINDOW_SIZE;
    std::vector< double > Stamps;
    MarginalizationFlag m_margFlag;
};

typedef boost::shared_ptr< SlideWindowBase > SlideWindowBasePtr;
}
#endif // SLIDEWINDOWBASE_H
