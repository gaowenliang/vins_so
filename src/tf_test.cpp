#include "estimator/slideWindow/transform.h"

#include <iostream>

int
main( int argc, char** argv )
{

    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, 0, 1, 0, 1, 0;
    Eigen::Vector3d T( 1, 2, 3 );

    Tf tf( R, T );

    std::cout << tf << std::endl;
    std::cout << "--------------------" << std::endl;

    Tf tf_2 = 2.0 * tf;
    std::cout << tf_2 << std::endl;
    std::cout << "--------------------" << std::endl;

    tf_2 *= 2.0;
    std::cout << tf_2 << std::endl;
    std::cout << "--------------------" << std::endl;

    Eigen::Matrix3d R2;
    R2 << -1, 0, 0, 0, 0, -1, 0, 1, 0;

    tf_2 *= R2;
    std::cout << tf_2 << std::endl;
    std::cout << "--------------------" << std::endl;

    Tf tf_3 = R2.transpose( ) * tf_2;
    std::cout << tf_3 << std::endl;
    std::cout << "--------------------" << std::endl;
}
