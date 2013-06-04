#include "amino.h"


static void aveq( const char * name,
                  size_t n, double *a, double *b, double tol ) {
    if( !aa_veq(n, a, b, tol) ) {
        fprintf( stderr, "FAILED: %s\n",name);
        fprintf( stderr, "a: ");
        aa_dump_vec( stderr, a, n );
        fprintf( stderr, "b: ");
        aa_dump_vec( stderr, b, n );

        assert( 0 );
    }
}


static void rotmat_axang() {
    /* Had some numerical stability issues */
    double q[4] =  {0.538444,0.141454,0.510387,0.655419};
    double R[9];
    aa_tf_quat2rotmat(q, R); // get rotation matrix

    double qr[4];
    aa_tf_rotmat2quat(R, qr); // convert to quaternion
    aveq("quat", 4, q, qr, .001);

    double vr[3], vq[3];      // get rotation vectors
    aa_tf_quat2rotvec(q, vq);
    aa_tf_rotmat2rotvec(R, vr);
    aveq("vec", 3, vq, vr, .001);

    double qvr[4], qvq[4];    // convert back to quaternions
    aa_tf_rotvec2quat( vq, qvq );
    aa_tf_rotvec2quat( vr, qvr );
    aveq("quat2", 4, qvq, qvr, .001);
}

int main() {
    rotmat_axang();
}
