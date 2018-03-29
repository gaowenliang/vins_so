#ifndef INITIALEX_H
#define INITIALEX_H

#include "InitialExRotationCamCam.h"
#include "InitialExRotationCamImu.h"

class InitialExParam
{
    public:
    InitialExParam( ) {}

    public:
    initialEx::InitialExRotationCamImuPtr m_RicInitial;
    initialEx::InitialExRotationCamCamPtr m_RccInitial;
};

#endif // INITIALEX_H
