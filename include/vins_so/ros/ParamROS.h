#ifndef PARAMROS_H
#define PARAMROS_H

#include "vins_so/estimator/vins_parameters.h"
#include <ros/ros.h>

class ParamROS
{
    public:
    ParamROS( ros::NodeHandle& n );

    void readParameters( ros::NodeHandle& n );
};

#endif // PARAMROS_H
