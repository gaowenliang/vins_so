#include "vins_so/estimator/initial_lib/InitialBase/initialgen.h"

boost::shared_ptr< InitVio::InitialGen > InitVio::InitialGen::m_instance;

InitVio::InitialGen::InitialGen( ) {}

boost::shared_ptr< InitVio::InitialGen >
InitVio::InitialGen::instance( )
{
    if ( m_instance.get( ) == 0 )
    {
        m_instance.reset( new InitialGen );
    }

    return m_instance;
}

InitVio::InitialPtr
InitVio::InitialGen::generateInit( InitVio::InitType type )
{
    switch ( type )
    {
        case MONO:
        {
            InitialMonoVioPtr initial( new InitialMonoVio );
            return initial;
        }
        case STEREO_NONLINEAR:
        {
            InitialStereoVioPtr initial( new InitialStereoVio );
            return initial;
        }
        case STEREO_PNP:
        {
            InitialStereoVioPnPPtr initial( new InitialStereoVioPnP );
            return initial;
        }
        default:
        {
            InitialMonoVioPtr initial( new InitialMonoVio );
            return initial;
        }
    }
    return InitialPtr( );
}
