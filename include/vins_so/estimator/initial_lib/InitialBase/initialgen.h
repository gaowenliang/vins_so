#ifndef INITIALGEN_H
#define INITIALGEN_H

#include "../InitialMono/initialmonovio.h"
#include "../InitialStereoPnP/initialstereoviopnp.h"
#include "../InitialStereoVio/initialstereovio.h"
#include "initialbase.h"

namespace InitVio
{

class InitialGen
{
    public:
    InitialGen( );
    static boost::shared_ptr< InitialGen > instance( void );

    InitialPtr generateInit( InitType type );

    private:
    static boost::shared_ptr< InitialGen > m_instance;
};
}
#endif // INITIALGEN_H
