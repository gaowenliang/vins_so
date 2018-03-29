#include "vins_so/estimator/slideWindow/slidewindowbase.h"

using namespace slidewindow;

SlideWindowBase::SlideWindowBase( int window_size )
: WINDOW_SIZE( window_size )
{
    Stamps.resize( WINDOW_SIZE + 1 );
}

SlideWindowBase&
SlideWindowBase::operator=( const SlideWindowBase& other )
{
    if ( this != &other )
    {
        WINDOW_SIZE = other.WINDOW_SIZE;
        Stamps      = other.Stamps;
        m_margFlag  = other.m_margFlag;
    }
    return *this;
}

void
SlideWindowBase::setMargOld( )
{
    m_margFlag = MARGIN_OLD;
}

void
SlideWindowBase::setMargSecondNew( )
{
    m_margFlag = MARGIN_SECOND_NEW;
}

void
SlideWindowBase::setStampIndex( int camera_index, double _stamp )
{
    Stamps[camera_index] = _stamp;
}

MarginalizationFlag
SlideWindowBase::margFlag( ) const
{
    return m_margFlag;
}

void
SlideWindowBase::setMargFlag( const MarginalizationFlag& margFlag )
{
    m_margFlag = margFlag;
}
