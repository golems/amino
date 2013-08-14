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

static void qrel() {

    {
        double r0[4] =   {-0.255393,  0.574795,  0.777422, 0.000147};
        double r_r0[4] = { 0.257651, -0.575645, -0.776040, 0.003256};
        double x0[3] =   { 0.292773, -0.220171, -0.298606};
        double x_r0[3] = { 0.293673, -0.214814, -0.296594};
        double S0[8], S_r0[8], Se[8];
        aa_tf_qv2duqu(r0,x0,S0);
        aa_tf_qv2duqu(r_r0,x_r0,S_r0);
        //for( size_t i = 0; i < 8; i ++ ) Se[i]*=-1;

        double twist[8], dx[6], qln[4], rv[3];
        double zero[3] = {0};
        aa_tf_duqu_mulc( S0, S_r0, Se );

        aa_tf_duqu_ln( Se, twist );
        aa_tf_qln(Se, qln);
        aa_tf_quat2rotvec_near(Se, zero, rv);
        aa_tf_duqu_twist2vel( S0, twist, dx );

        fprintf(stderr,"Se: ");aa_dump_vec( stderr, Se, 8 );
        fprintf(stderr,"ln: ");aa_dump_vec( stderr, twist, 8 );
        fprintf(stderr,"qln: ");aa_dump_vec( stderr, qln, 4 );
        fprintf(stderr,"rv: ");aa_dump_vec( stderr, rv, 3 );
        fprintf(stderr,"dx: ");aa_dump_vec( stderr, dx, 6 );
        printf( "dot: %f\n",  aa_la_dot(4, r0, r_r0 )) ;
    }
    {
        double r0[4] =    {-0.277017,0.566639,0.775419,-0.030125};
        double r_r0[4]  = {0.277017,-0.566639,-0.775419,0.030125};
        double re[4], rv[4];
        aa_tf_qmulc(r0, r_r0, re);
        double zero[3] = {0};
        aa_tf_qln(re, rv);
        fprintf(stderr,"qrel: ");aa_dump_vec( stderr, re, 4 );
        fprintf(stderr,"rv: ");aa_dump_vec( stderr, rv, 4 );
    }

}

int main() {
    rotmat_axang();
    //qrel();
}
