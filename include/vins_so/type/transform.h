#ifndef TF_H
#define TF_H

#include <eigen3/Eigen/Eigen>

class Tf
{
    public:
    Tf( )
    {
        R.setIdentity( );
        T.setZero( );
    }
    Tf( const Eigen::Matrix3d& _R, const Eigen::Vector3d& _T )
    : R( _R )
    , T( _T )
    {
    }
    Tf( const Eigen::Quaterniond& _q, const Eigen::Vector3d& _T )
    : R( _q.toRotationMatrix( ) )
    , T( _T )
    {
    }
    inline void setZero( )
    {
        R.setIdentity( );
        T.setZero( );
    }
    inline void swap( Tf& tf_in )
    {
        R.swap( tf_in.R );
        T.swap( tf_in.T );
    }
    inline Tf& operator=( const Tf& other )
    {
        if ( this != &other )
        {
            R = other.R;
            T = other.T;
        }
        return *this;
    }
    inline Tf operator+( const Tf& tf_bc ) const
    {
        // this: tf_ab
        // R_ac = R_ab * R_bc
        // T_ac = T_ab + R_ab*T_bc
        return Tf( R * tf_bc.R, //
                   T + R * tf_bc.T );
    }
    inline void operator+=( const Tf& tf_bc )
    {
        // this: tf_ab
        // R_ac = R_ab * R_bc
        // T_ac = T_ab + R_ab*T_bc
        R = R * tf_bc.R; //
        T = T + R * tf_bc.T;
    }
    inline friend Tf operator*( const double& scale, Tf tf )
    {
        // this: tf_a
        return Tf( tf.R, scale * tf.T );
    }
    inline void operator*=( const double& scale )
    {
        // this: tf_a
        T = scale * T;
    }
    inline friend Tf operator*( const Eigen::Matrix3d& _R, Tf tf )
    {
        return Tf( _R * tf.R, _R * tf.T );
    }
    inline void operator*=( const Eigen::Matrix3d& _R )
    {
        //        this : tf_ab
        R = _R * R;
        T = _R * T;
    }
    inline Tf inverse( ) const
    {
        // R_ba = R_ab^T
        // T_ba = -R_ab^T * T_ab
        return Tf( R.transpose( ), //
                   -R.transpose( ) * T );
    }
    inline void TfToDouble( double* Pose )
    {
        Pose[0] = T.x( );
        Pose[1] = T.y( );
        Pose[2] = T.z( );

        Eigen::Quaterniond q{ R };
        Pose[3] = q.x( );
        Pose[4] = q.y( );
        Pose[5] = q.z( );
        Pose[6] = q.w( );
    }
    inline double norm( ) const { return T.norm( ); }
    inline Eigen::Matrix3d getR( ) const { return R; }
    inline Eigen::Vector3d getT( ) const { return T; }
    inline void setR( const Eigen::Matrix3d& _R ) { R = _R; }
    inline void setT( const Eigen::Vector3d& _T ) { T = _T; }
    inline void setRT( const Eigen::Matrix3d& _R, const Eigen::Vector3d& _T )
    {
        R = _R;
        T = _T;
    }

    inline Eigen::Vector3d toYPR( )
    {
        Eigen::Vector3d n = R.col( 0 );
        Eigen::Vector3d o = R.col( 1 );
        Eigen::Vector3d a = R.col( 2 );

        Eigen::Vector3d ypr( 3 );
        double y = atan2( n( 1 ), n( 0 ) );
        double p = atan2( -n( 2 ), n( 0 ) * cos( y ) + n( 1 ) * sin( y ) );
        double r
        = atan2( a( 0 ) * sin( y ) - a( 1 ) * cos( y ), -o( 0 ) * sin( y ) + o( 1 ) * cos( y ) );
        ypr( 0 ) = y;
        ypr( 1 ) = p;
        ypr( 2 ) = r;

        return ypr / M_PI * 180.0;
    }
    inline Eigen::Quaterniond toQuaternion( )
    {
        Eigen::Quaterniond q( R );
        return q;
    }
    inline Eigen::Vector3d transVector( const Eigen::Vector3d& v ) const
    {
        return T + R * v;
    }
    friend std::ostream& operator<<( std::ostream& out, const Tf& tf_tmp )
    {
        out << "   R " << std::endl << tf_tmp.R << std::endl;
        out << "   T " << std::endl << tf_tmp.T.transpose( ) << std::endl;
        return out;
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d T;
};

#endif // TF_H
