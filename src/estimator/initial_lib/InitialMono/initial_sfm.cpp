#include "vins_so/estimator/initial_lib/InitialMono/initial_sfm.h"
#include <code_utils/cv_utils/pnp/pnp.h>

GlobalSFM::GlobalSFM( ) {}

void
GlobalSFM::triangulatePoint( Eigen::Matrix< double, 3, 4 >& Pose0,
                             Eigen::Matrix< double, 3, 4 >& Pose1,
                             Eigen::Vector3d& point0,
                             Eigen::Vector3d& point1,
                             Eigen::Vector3d& point_3d )
{

    Eigen::MatrixXd svd_A( 3 * 2, 4 );
    int svd_idx = 0;

    svd_A.row( svd_idx++ ) = point0[1] * Pose0.row( 2 ) - point0[2] * Pose0.row( 1 );
    svd_A.row( svd_idx++ ) = point0[2] * Pose0.row( 0 ) - point0[0] * Pose0.row( 2 );
    svd_A.row( svd_idx++ ) = point0[0] * Pose0.row( 1 ) - point0[1] * Pose0.row( 0 );

    svd_A.row( svd_idx++ ) = point1[1] * Pose1.row( 2 ) - point1[2] * Pose1.row( 1 );
    svd_A.row( svd_idx++ ) = point1[2] * Pose1.row( 0 ) - point1[0] * Pose1.row( 2 );
    svd_A.row( svd_idx++ ) = point1[0] * Pose1.row( 1 ) - point1[1] * Pose1.row( 0 );

    Eigen::Vector4d triangulated_point = Eigen::JacobiSVD< Eigen::MatrixXd >( svd_A, //
                                                                              Eigen::ComputeThinV )
                                         .matrixV( )
                                         .rightCols< 1 >( );

    point_3d( 0 ) = triangulated_point( 0 ) / triangulated_point( 3 );
    point_3d( 1 ) = triangulated_point( 1 ) / triangulated_point( 3 );
    point_3d( 2 ) = triangulated_point( 2 ) / triangulated_point( 3 );
}

bool
GlobalSFM::solveFrameByPnP( Matrix3d& R_initial, Vector3d& P_initial, int i, vector< SFMFeature >& sfm_f )
{
    vector< Eigen::Vector3d > pts_2_vector_tmp;
    vector< Eigen::Vector3d > pts_3_vector_tmp;

    for ( int j = 0; j < feature_num; j++ )
    {
        if ( sfm_f[j].state != true )
            continue;
        for ( int k = 0; k < ( int )sfm_f[j].observation.size( ); k++ )
        {
            if ( sfm_f[j].observation[k].first == i )
            {
                Vector3d img_pts = sfm_f[j].observation[k].second;
                pts_2_vector_tmp.push_back( img_pts );
                pts_3_vector_tmp.push_back(
                Eigen::Vector3d( sfm_f[j].position[0], sfm_f[j].position[1], sfm_f[j].position[2] ) );
                break;
            }
        }
    }
    if ( int( pts_2_vector_tmp.size( ) ) < 15 )
    {
        printf( "unstable features tracking, please slowly move you device!\n" );
        if ( int( pts_2_vector_tmp.size( ) ) < 10 )
            return false;
    }

    Quaterniond q_initial2;
    Vector3d P_initial2;
    cv::Pnp init_pnp( pts_2_vector_tmp, pts_3_vector_tmp, q_initial2, P_initial2 );
    std::cout << "R_initial: " << endl << q_initial2.coeffs( ).transpose( ) << std::endl;
    std::cout << "P_initial: " << endl << P_initial2.transpose( ) << std::endl;

    std::cout << "-------------------------------------------------- " << endl;
    R_initial = q_initial2;
    P_initial = P_initial2;

    return true;
}

void
GlobalSFM::triangulateTwoFrames( int frame0,
                                 Eigen::Matrix< double, 3, 4 >& Pose0,
                                 int frame1,
                                 Eigen::Matrix< double, 3, 4 >& Pose1,
                                 vector< SFMFeature >& sfm_f )
{
    assert( frame0 != frame1 );
    for ( int j = 0; j < feature_num; j++ )
    {
        if ( sfm_f[j].state == true )
            continue;
        bool has_0 = false, has_1 = false;
        Vector3d point0;
        Vector3d point1;
        for ( int k = 0; k < ( int )sfm_f[j].observation.size( ); k++ )
        {
            if ( sfm_f[j].observation[k].first == frame0 )
            {
                point0 = sfm_f[j].observation[k].second;
                has_0  = true;
            }
            if ( sfm_f[j].observation[k].first == frame1 )
            {
                point1 = sfm_f[j].observation[k].second;
                has_1  = true;
            }
        }
        if ( has_0 && has_1 )
        {
            Vector3d point_3d;
            triangulatePoint( Pose0, Pose1, point0, point1, point_3d );
            sfm_f[j].state       = true;
            sfm_f[j].position[0] = point_3d( 0 );
            sfm_f[j].position[1] = point_3d( 1 );
            sfm_f[j].position[2] = point_3d( 2 );
            // cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " <<
            // point_3d.transpose() << endl;
        }
    }
}

// 	 q w_R_cam t w_R_cam
//  c_rotation cam_R_w
//  c_translation cam_R_w
// relative_q[i][j]  j_q_i
// relative_t[i][j]  j_t_ji  (j < i)
bool
GlobalSFM::construct( int frame_num,
                      vector< Quaterniond >& q_wcs,
                      vector< Vector3d >& T_wcs,
                      int linkIndex,
                      const Matrix3d relative_R,
                      const Vector3d relative_T,
                      vector< SFMFeature >& sfm_f,
                      map< int, Vector3d >& sfm_tracked_points )
{
    feature_num = sfm_f.size( );
    cout << "construct_R " << endl << relative_R << endl;
    cout << "construct_T " << endl << relative_T << endl;

    // cout << "set 0 and " << l << " as known " << endl;
    // have relative_r relative_t
    // intial two view
    q_wcs[linkIndex].w( ) = 1;
    q_wcs[linkIndex].x( ) = 0;
    q_wcs[linkIndex].y( ) = 0;
    q_wcs[linkIndex].z( ) = 0;
    T_wcs[linkIndex].setZero( );
    q_wcs[frame_num - 1] = q_wcs[linkIndex] * Quaterniond( relative_R );
    T_wcs[frame_num - 1] = relative_T;
    // cout << "init q_l " << q[l].w() << " " << q[l].vec().transpose() << endl;
    // cout << "init t_l " << T[l].transpose() << endl;

    // rotate to cam frame
    Matrix3d c_Rotation[frame_num];
    Vector3d c_Translation[frame_num];
    Quaterniond c_Quat[frame_num];
    double c_rotation[frame_num][4];
    double c_translation[frame_num][3];
    Eigen::Matrix< double, 3, 4 > Pose[frame_num];

    c_Quat[linkIndex]        = q_wcs[linkIndex].inverse( );
    c_Rotation[linkIndex]    = c_Quat[linkIndex].toRotationMatrix( );
    c_Translation[linkIndex] = -1 * ( c_Rotation[linkIndex] * T_wcs[linkIndex] );
    Pose[linkIndex].block< 3, 3 >( 0, 0 ) = c_Rotation[linkIndex];
    Pose[linkIndex].block< 3, 1 >( 0, 3 ) = c_Translation[linkIndex];

    c_Quat[frame_num - 1]        = q_wcs[frame_num - 1].inverse( );
    c_Rotation[frame_num - 1]    = c_Quat[frame_num - 1].toRotationMatrix( );
    c_Translation[frame_num - 1] = -1 * ( c_Rotation[frame_num - 1] * T_wcs[frame_num - 1] );
    Pose[frame_num - 1].block< 3, 3 >( 0, 0 ) = c_Rotation[frame_num - 1];
    Pose[frame_num - 1].block< 3, 1 >( 0, 3 ) = c_Translation[frame_num - 1];

    // 1: trangulate between l ----- frame_num - 1
    // 2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1;
    for ( int i = linkIndex; i < frame_num - 1; i++ )
    {
        // solve pnp
        if ( i > linkIndex )
        {
            Matrix3d R_initial = c_Rotation[i - 1];
            Vector3d P_initial = c_Translation[i - 1];
            if ( !solveFrameByPnP( R_initial, P_initial, i, sfm_f ) )
                return false;
            c_Rotation[i]    = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i]        = c_Rotation[i];
            Pose[i].block< 3, 3 >( 0, 0 ) = c_Rotation[i];
            Pose[i].block< 3, 1 >( 0, 3 ) = c_Translation[i];
        }

        // triangulate point based on the solve pnp result
        triangulateTwoFrames( i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f );
    }
    // 3: triangulate l-----l+1 l+2 ... frame_num -2
    for ( int i = linkIndex + 1; i < frame_num - 1; i++ )
        triangulateTwoFrames( linkIndex, Pose[linkIndex], i, Pose[i], sfm_f );
    // 4: solve pnp l-1; triangulate l-1 ----- l
    //             l-2              l-2 ----- l
    for ( int i = linkIndex - 1; i >= 0; i-- )
    {
        // solve pnp
        Matrix3d R_initial = c_Rotation[i + 1];
        Vector3d P_initial = c_Translation[i + 1];
        if ( !solveFrameByPnP( R_initial, P_initial, i, sfm_f ) )
            return false;
        c_Rotation[i]    = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i]        = c_Rotation[i];
        Pose[i].block< 3, 3 >( 0, 0 ) = c_Rotation[i];
        Pose[i].block< 3, 1 >( 0, 3 ) = c_Translation[i];
        // triangulate
        triangulateTwoFrames( i, Pose[i], linkIndex, Pose[linkIndex], sfm_f );
    }
    // 5: triangulate all other points
    for ( int j = 0; j < feature_num; j++ )
    {
        if ( sfm_f[j].state == true )
            continue;
        if ( ( int )sfm_f[j].observation.size( ) >= 2 )
        {
            Vector3d point0, point1;
            int frame_0 = sfm_f[j].observation[0].first;
            point0      = sfm_f[j].observation[0].second;
            int frame_1 = sfm_f[j].observation.back( ).first;
            point1      = sfm_f[j].observation.back( ).second;
            Vector3d point_3d;
            triangulatePoint( Pose[frame_0], Pose[frame_1], point0, point1, point_3d );
            sfm_f[j].state       = true;
            sfm_f[j].position[0] = point_3d( 0 );
            sfm_f[j].position[1] = point_3d( 1 );
            sfm_f[j].position[2] = point_3d( 2 );
            // cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  <<
            // j << "
            // " <<
            // point_3d.transpose() << endl;
        }
    }

    /*
        for (int i = 0; i < frame_num; i++)
        {
            q[i] = c_Rotation[i].transpose();
            cout << "solvePnP  q" << " i " << i <<"  " <<q[i].w() << "  " <<
       q[i].vec().transpose()
       << endl;
        }
        for (int i = 0; i < frame_num; i++)
        {
            Vector3d t_tmp;
            t_tmp = -1 * (q[i] * c_Translation[i]);
            cout << "solvePnP  t" << " i " << i <<"  " << t_tmp.x() <<"  "<< t_tmp.y() <<"
       "<<
       t_tmp.z() << endl;
        }
    */
    // full BA
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization( );
    // cout << " begin full BA " << endl;
    for ( int i = 0; i < frame_num; i++ )
    {
        // double array for ceres
        c_translation[i][0] = c_Translation[i].x( );
        c_translation[i][1] = c_Translation[i].y( );
        c_translation[i][2] = c_Translation[i].z( );
        c_rotation[i][0]    = c_Quat[i].w( );
        c_rotation[i][1]    = c_Quat[i].x( );
        c_rotation[i][2]    = c_Quat[i].y( );
        c_rotation[i][3]    = c_Quat[i].z( );
        problem.AddParameterBlock( c_rotation[i], 4, local_parameterization );
        problem.AddParameterBlock( c_translation[i], 3 );
        if ( i == linkIndex )
        {
            problem.SetParameterBlockConstant( c_rotation[i] );
        }
        if ( i == linkIndex || i == frame_num - 1 )
        {
            problem.SetParameterBlockConstant( c_translation[i] );
        }
    }

    for ( int i = 0; i < feature_num; i++ )
    {
        if ( sfm_f[i].state != true )
            continue;
        for ( int j = 0; j < int( sfm_f[i].observation.size( ) ); j++ )
        {
            int l = sfm_f[i].observation[j].first;
            ceres::CostFunction* cost_function
            = ReprojectionShpereError3D::Create( sfm_f[i].observation[j].second.x( ),
                                                 sfm_f[i].observation[j].second.y( ),
                                                 sfm_f[i].observation[j].second.z( ) );

            problem.AddResidualBlock(
            cost_function, NULL, c_rotation[l], c_translation[l], sfm_f[i].position );
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    // std::cout << summary.BriefReport() << "\n";
    if ( summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03 )
    {
        // cout << "vision only BA converge" << endl;
    }
    else
    {
        // cout << "vision only BA not converge " << endl;
        return false;
    }
    for ( int i = 0; i < frame_num; i++ )
    {
        q_wcs[i].w( ) = c_rotation[i][0];
        q_wcs[i].x( ) = c_rotation[i][1];
        q_wcs[i].y( ) = c_rotation[i][2];
        q_wcs[i].z( ) = c_rotation[i][3];
        q_wcs[i]      = q_wcs[i].inverse( );
        // cout << "final  q"
        //     << " i " << i << "  " << c_rotation[i][0] << "  " << c_rotation[i][1] << "  "
        //     << c_rotation[i][2] << "  " << c_rotation[i][3] << endl;
    }
    for ( int i = 0; i < frame_num; i++ )
    {
        T_wcs[i]
        = -1 * ( q_wcs[i] * Vector3d( c_translation[i][0], c_translation[i][1], c_translation[i][2] ) );
        // cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<<
        // T[i](2) <<
        // endl;
    }
    for ( int i = 0; i < ( int )sfm_f.size( ); i++ )
    {
        if ( sfm_f[i].state )
            sfm_tracked_points[sfm_f[i].id]
            = Vector3d( sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2] );
    }
    return true;
}
