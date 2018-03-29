#include <eigen3/Eigen/Eigen>
#include <iostream>

int
main( int argc, char** argv )
{

    Eigen::Vector3d v1;
    Eigen::Vector3d v2;

    v1 = Eigen::Vector3d( -0.000886948, 0.00132282, -0.999999 ).normalized( );

    v2 = v1;

    double angle = acos( v1.dot( v2 ) );

    std::cout << "p_i " << v1.transpose( ) << "\n";
    std::cout << "p_j " << v2.transpose( ) << "\n";
    std::cout << "acos " << v1.dot( v2 ) << "\n";
    std::cout << "angle " << angle << "\n";

    Eigen::MatrixXd mat( 3, 3 );
    mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "mat " << mat << "\n";
    mat.conservativeResize( 4, 3 );
    std::cout << "mat " << mat << "\n";

    return 0;
}
