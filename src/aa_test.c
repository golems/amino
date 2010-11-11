/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
//#define AA_ALLOC_STACK_MAX
#include "amino.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>

static void afeq( double a, double b, double tol ) {
    assert( aa_feq(a,b,tol) );
}

static void aveq( size_t n, double *a, double *b, double tol ) {
    assert( aa_veq(n, a, b, tol) );
}

static void aneq( double a, double b, double tol ) {
    assert( !aa_feq(a,b,tol) );
}

void scalar() {
    // eq
    afeq( M_PI, M_PI, 0 );
    afeq( 1, 1.001, .01 );
    aneq( 1, 2, .1 );

    // min/max loc
    assert( 1 == aa_fminloc( 3, AA_FAR( 1, 0, 10 ) ) );
    assert( 2 == aa_fminloc( 3, AA_FAR( 1, 0, -10 ) ) );
    assert( 2 == aa_fmaxloc( 3, AA_FAR( 1, 0, 10 ) ) );
    assert( 0 == aa_fmaxloc( 3, AA_FAR( 100, 0, 10 ) ) );
}

void la0() {
    // min
    afeq( 2, aa_la_min( 3, (double[3]) {10, 2, 4} ), 0 );
    // max
    afeq( 10, aa_la_max( 3, (double[3]) {10, 2, 4} ), 0 );
    // dot
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        afeq( x[0]*y[0]+x[1]*y[1]+x[2]*y[2], aa_la_dot(3, x, y), 0 );
    }
    // norm
    {
        double x[] = {1,2,3};
        afeq( sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]), aa_la_norm(3, x), 0 );
    }
    // ssd
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        afeq( pow(x[0]-y[0],2) + pow(x[1]-y[1],2) + pow(x[2]-y[2],2),
             aa_la_ssd(3, x, y), 0 );
    }
    // dist
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        afeq( sqrt(pow(x[0]-y[0],2) + pow(x[1]-y[1],2) + pow(x[2]-y[2],2)),
             aa_la_dist(3, x, y), 0 );
    }

}

void la1() {
    // scal
    {
        double x[] = {1,2,3};
        double r[] = { 1*2, 2*2, 3*2 };
        aa_la_scal(3,2,x);
        aveq( 3, r, x, 0 );
    }
    // sinc
    {
        double x[] = {1,2,3};
        double r[] = { 1+2, 2+2, 3+2 };
        aa_la_sinc(3, 2, x);
        aveq( 3, r, x, 0 );
    }
    // vinc
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        double r[] = { 1+4, 2+5, 3+6 };
        aa_la_vinc(3, x, y);
        aveq( 3, r, y, 0 );
    }
    // axpy
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        double r[] = { 2*1+4, 2*2+5, 2*3+6 };
        aa_la_axpy(3, 2, x, y);
        aveq( 3, r, y, 0 );
    }

    // axpy3
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        double z[3];
        double r[] = { 2*1+4, 2*2+5, 2*3+6 };
        aa_la_axpy3(3, 2, x, y, z);
        aveq( 3, r, z, 0 );
    }
    // sadd
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {2,3,4};
        aa_la_sadd( 3, 1, x, r );
        aveq( 3, r, y, 0 );
    }
    // ssub
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {-1,-2,-3};
        aa_la_ssub( 3, 0, x, r );
        aveq( 3, r, y, 0 );
    }
    // smul
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {2,4,6};
        aa_la_smul( 3, 2, x, r );
        aveq( 3, r, y, 0 );
    }
    // sdiv
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {2./1, 2./2, 2./3};
        aa_la_sdiv( 3, 2, x, r );
        aveq( 3, r, y, 0 );
    }

    // vadd
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 5, 7, 9 };
        aa_la_vadd( 3,  x, y, r );
        aveq( 3, r, p, 0 );
    }
    // vsub
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 1-4, 2-5, 3-6 };
        aa_la_vsub( 3,  x, y, r );
        aveq( 3, r, p, 0 );
    }
    // vmul
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 1*4, 2*5, 3*6 };
        aa_la_vmul( 3,  x, y, r );
        aveq( 3, r, p, 0 );
    }
    // vdiv
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 1./4, 2./5, 3./6 };
        aa_la_vdiv( 3,  x, y, r );
        aveq( 3, r, p, 0 );
    }

    // cross
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { -3, 6, -3 };
        aa_la_cross(  x, y, r );
        aveq( 3, r, p, 0 );
    }
    // normalize
    {
        double x[] = {1,2,3};
        double r[] = {0.26726,   0.53452,   0.80178};
        aa_la_normalize(3,x);
        aveq( 3, r, x, 0.0001 );
    }
}

void la2() {
    // mvmul
    {
        double A[] = {1,2,3,4,5,6};
        double x[] = {7, 11};
        double y[3];
        double r[3] = {51, 69, 87};
        aa_la_mvmul( 3, 2, A, x, y );
        aveq(3,y,r,0);
    }
    // SVD
    {
        double U[3*3];
        double Vt[2*2];
        double S[2];
        double Ur[] =  {0.42867, 0.56631, 0.70395, 0.80596, 0.11238, -0.58120, -0.40825, 0.81650, -0.40825};
        double Sr[] = { 9.50803, 0.77287};
        double Vtr[] = {0.38632, -0.92237, 0.92237, 0.38632};
        {
            double A[] = {1,2,3,4,5,6}; //3x2
            aa_la_svd(3,2,A,U,S,NULL);
            aveq(3*3,U,Ur, 0.001);
            aveq(2, S, Sr, 0.001);
        }
        {
            double A[] = {1,2,3,4,5,6}; //3x2
            aa_la_svd(3,2,A,NULL,S,Vt);
            aveq(2, S, Sr, 0.001);
            aveq(2*2, Vt, Vtr, 0.001);
        }

        {
            double A[] = {1,2,3,4,5,6}; //3x2
            aa_la_svd(3,2,A,U,S,Vt);
            aveq(3*3,U,Ur, 0.0001);
            aveq(2, S, Sr, 0.001);
            aveq(2*2, Vt, Vtr, 0.001);
        }
    }
    // invert
    {
        double A[] = {1,2,3,4};
        double B[] = {-2,1,1.5,-.5};
        aa_la_inv( 2, A );
        aveq( 4, A, B, 0 );
    }
    // trace
    {
        double A[9] = {1,2,3, 4,5,6, 7,8,9};
        double B[9];
        double t = aa_la_trace(3,A);
        aa_la_transpose2( 3, 3, A, B );
        double t2 = aa_la_trace(3,B);
        afeq( t, 1+5+9, 0 );
        afeq( t2, 1+5+9, 0 );
    }

    // transpose
    {
        double A[9] = {1,2,3, 4,5,6, 7,8,9};
        double At[9] =  {1,4,7,  2,5,8,  3,6,9};
        aa_la_transpose( 3, A );
        aveq( 9, A, At, 0 );

    }
    // inverse3x3
    {
        double R[9] = {0,-1,0, 1,0,0, 0,0,-1};
        double S[9];
        aa_la_inverse3x3( R, S );
        aa_la_inv(3, R );
        aveq( 9, R, S, 0.00001 );
    }
    // det3x3
    {
        double R[9] = {0,-1,0, 1,0,0, 0,0,-1};
        double S[9];
        aa_la_inverse3x3( R, S );
        double d = aa_la_det3x3( R );
        double dt = aa_la_det3x3( S );
        afeq( -1, d, .000001 );
        afeq( -1, dt, .000001 );
    }

    //dpinv
    {
        double A[] = {1,2,3,4};
        double A_star[4];
        double R[] = {  -1.92649, 0.96746,  1.44818,  -0.47711 };
        aa_la_dpinv(2,2, .005, &A[0], &A_star[0]);
        aveq( 4, A_star, R, .0001 );
    }
    {
        double A[] = {1,2,3,4,5,6};
        double A_star[6];
        double R[] =  {-1.30832, -0.32652, 0.65528, 1.06359, 0.32795, -0.40769};
        aa_la_dpinv(2,3, .005, &A[0], &A_star[0]);
        aveq( 6, A_star, R, .0001 );
    }
    // dls
    {
        double A[] = {1,2,3,4,5,6};
        double b[] = {10,20};
        double x_r[] =  {8.1885, 3.2938, -1.6009};
        double x[3];
        aa_la_dls(2, 3, .005, A, b, x);
        aveq(3, x_r, x, .0001 );
    }
    // dlsnp
    {
        double A[] = {1,2,3,4,5,6};
        double b[] = {10,20};
        double x_r[] = { 8.1938, 3.2954, -1.6029 };
        double x_p[] =  {2,4,6};
        double x[3];
        aa_la_dlsnp(2, 3, .005, A, b, x_p, x);
        aveq(3, x_r, x, .0001 );
    }

    // lls
    {
        //double A[] = {1,2,3,4};
        //double b[] = {3,7};
        //double x[2];
        //aa_la_lls(2,2,2,A,b,x);
    }

}



void axang() {
    // rotvecs
    {
        double aa[4], rv[3], aap[4];
        aa_tf_axang_make( 1,2,3, M_PI/2.0, aa );
        afeq( aa_la_norm(3, aa), 1.0, .00001 );
        aa_tf_axang2rotvec( aa, rv );
        afeq( aa_la_norm(3, rv), M_PI/2.0, .00001 );
        aa_tf_rotvec2axang( rv, aap );
        aveq( 4, aa, aap, .00001 );
    }
    // 2 quat
    {
        double q_r[4] = {1,2,3,4};
        aa_tf_qnormalize(q_r);
        double q[4];
        double a[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_axang2quat(a,q);
        aveq( 4, q, q_r, 0.001 );
    }
    {
        double aa[4], rv[3], aap[4];
        aa_tf_axang_make( 1,2,3, M_PI/2.0, aa );
        afeq( aa_la_norm(3, aa), 1.0, .00001 );
        aa_tf_axang2rotvec( aa, rv );
        afeq( aa_la_norm(3, rv), M_PI/2.0, .00001 );
        aa_tf_rotvec2axang( rv, aap );
        aveq( 4, aa, aap, .00001 );
    }
    {
        double q_r[4] = {1,2,3,4};
        aa_tf_qnormalize(q_r);
        double q[4];
        double r[3];
        double a[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_axang2rotvec(a,r);
        aa_tf_rotvec2quat(r,q);
        aveq( 4, q, q_r, 0.001 );
    }
    {
        double q[4] = {1,2,3,4};
        aa_tf_qnormalize(q);
        double r[3];
        double a[4];
        double a_r[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_quat2rotvec(q,r);
        aa_tf_rotvec2axang(r,a);
        aveq( 4, a, a_r, 0.001 );
    }
}

void angle() {
    // conversion
    afeq( aa_ang_rad2deg(3.1), 3.1*180.0/M_PI, 0 );
    afeq( aa_ang_rad2deg(M_PI), 180, 0 );
    afeq( aa_ang_deg2rad(30), 30*M_PI/180, 0 );
    afeq( aa_ang_deg2rad(180), M_PI, 0 );

    // norming
    afeq( aa_ang_norm_2pi( 3*M_PI ), M_PI, 0 );
    afeq( aa_ang_norm_2pi( -M_PI/2 ), 3*M_PI/2, .001 );
    afeq( aa_ang_norm_pi( 3*M_PI/2 ), -M_PI/2, 0 );
}

void quat() {
    // conj
    {
        double p[4] = {1,2,3,4};
        double q[4];
        double r[4] = {-1,-2,-3,4};
        aa_tf_qconj(p,q);
        aveq( 4, q, r, 0.000 );

    }
    // inv
    {
        double p[4] = {1,2,3,4};
        double q[4];
        double r[4] = { -0.0333333, -0.0666666, -0.1, 0.133333 };
        aa_tf_qinv(p,q);
        aveq( 4, q, r, 0.0001 );
    }
    // mul
    {
        double a[4] = {1,2,3,4};
        double b[4] = {9,8,7,6};
        double c[4];
        double r[4] = {32, 64, 36, -22};
        aa_tf_qmul(a,b,c);
        aveq( 4, c, r, 0.0000 );
    }
    // 2 axis-angle
    {
        double a[4] = {1,2,3,4};
        double b[4];
        double r[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_quat2axang(a,b);
        aveq( 4, b, r, 0.001 );
    }
    // 2 axis-angle
    {
        double ra_r[4] = {0,0,0,0};
        double ra[4];
        aa_tf_quat2axang( AA_FAR(0,0,0,1), ra);
        aveq( 4, ra, ra_r, 0.001 );
    }

    // from axis-angle
    {
        double q_r[4] = {0,0,0,1};
        double q[4];
        aa_tf_axang2quat( AA_FAR(0,0,0,0), q);
        aveq( 4, q, q_r, 0.00001 );
        aa_tf_axang2quat( AA_FAR(1,1,1,0), q);
        aveq( 4, q, q_r, 0.00001 );
        aa_tf_axang2quat( AA_FAR(1,2,3,0), q);
        aveq( 4, q, q_r, 0.00001 );
    }
    // rotvec identity
    {
        double rv[3], q[4];
        aa_tf_rotvec2quat(AA_TF_ROTVEC_IDENT, q);
        aveq(4,AA_TF_QUAT_IDENT,q,0.000001);
        aa_tf_quat2rotvec(AA_TF_QUAT_IDENT, rv);
        aveq(3,AA_TF_ROTVEC_IDENT,rv,0.000001);
    }
    // rotations
    {
        double Rzi[9] = {0,1,0,  -1,0,0,  0,0,1};
        double v[3] = {1,2,3};
        double vp_r0[3], vp_q[3], vp_r1[3];
        double q[4], R[9];
        aa_tf_rotmat2quat(Rzi,q);
        aa_tf_quat2rotmat( q, R );
        aa_la_mvmul(3, 3, Rzi, v, vp_r0);
        aa_tf_qrot(q,v,vp_q);
        aa_la_mvmul(3, 3, R, v, vp_r1);
        aveq(9, R, Rzi, .0001);
        aveq(3,vp_r0, vp_q, .0001);
        aveq(3,vp_r0, vp_r1, .0001);
    }

    // nearby
    {
        double q[4] = {1,0,0,1};
        double q1[4];
        aa_tf_qnormalize(q);
        double rv0[3] = {10,20,30};
        double  rvn[3];
        double p[3] = {3,2,1};
        double pr[3], pq[3];
        aa_tf_quat2rotvec_near(q, rv0, rvn );
        aa_tf_rotvec2quat( rvn, q1 );
        aa_tf_qrot(q, p, pq);
        aa_tf_qrot(q1, p, pr);
        aveq( 3, pr, pq, .00001 );
    }
}

void tm() {
    struct timespec t;

    t = aa_tm_make_norm( 100, 1e9 + 1 );
    assert( 101 == t.tv_sec && 1 == t.tv_nsec );

    t = aa_tm_make_norm( 100,  -1 );
    assert( 99 == t.tv_sec && AA_IBILLION - 1 == t.tv_nsec );

    t = aa_tm_make_norm( 100,  -AA_IBILLION - 1 );
    assert( 98 == t.tv_sec && AA_IBILLION - 1 == t.tv_nsec );
}

void mem() {
    {
        aa_region_t reg;
        aa_region_init(&reg, 10);
        void *p = aa_region_alloc(&reg,10);
        memset(p,0,10);
        aa_region_destroy(&reg);
    }
    {
        aa_region_t reg;
        aa_region_init(&reg, 2);
        void *p = aa_region_alloc(&reg,10);
        memset(p,0,10);
        aa_region_destroy(&reg);
    }
    {
        aa_region_t reg;
        aa_region_init(&reg, 256);
        double *p = (double*)aa_region_alloc(&reg,sizeof(double));
        double *q = (double*)aa_region_alloc(&reg,sizeof(double));
        *p = 1.0;
        *q = 2.0;
        afeq( *p, 1.0, 0 );
        afeq( *q, 2.0, 0 );
        *q = 3.0;
        *p = 4.0;
        afeq( *p, 4.0, 0 );
        afeq( *q, 3.0, 0 );
        aa_region_destroy(&reg);
    }
    {
        aa_region_t reg;
        aa_region_init(&reg, 1);
        double *p = (double*)aa_region_alloc(&reg,sizeof(double));
        double *q = (double*)aa_region_alloc(&reg,sizeof(double));
        *p = 1.0;
        *q = 2.0;
        afeq( *p, 1.0, 0 );
        afeq( *q, 2.0, 0 );
        *q = 3.0;
        *p = 4.0;
        afeq( *p, 4.0, 0 );
        afeq( *q, 3.0, 0 );
        aa_region_destroy(&reg);
    }
    {
        aa_region_t reg;
        aa_region_init(&reg, 256);

        char *a = (char*)aa_region_alloc(&reg, 4096);
        memset(a,10,4096);
        for(size_t i = 0; i < 4096; i ++ ) assert( 10 == a[i] );

        char *b = (char*)aa_region_alloc(&reg, 4096);
        memset(b,11,4096);
        for(size_t i = 0; i < 4096; i ++ ) assert( 10 == a[i] );
        for(size_t i = 0; i < 4096; i ++ ) assert( 11 == b[i] );

        char *c = (char*)aa_region_alloc(&reg, 4096);
        memset(c,12,4096);
        for(size_t i = 0; i < 4096; i ++ ) assert( 10 == a[i] );
        for(size_t i = 0; i < 4096; i ++ ) assert( 11 == b[i] );
        for(size_t i = 0; i < 4096; i ++ ) assert( 12 == c[i] );

        aa_region_destroy(&reg);
    }
    {
        aa_region_t reg;
        aa_region_init(&reg, 16);
        for( size_t i = 0; i < 256; i ++ ) {
            aa_region_alloc(&reg, 16);
            aa_region_alloc(&reg, 16);
            aa_region_release(&reg);
        }
        aa_region_destroy(&reg);
    }
    // zero_ar
    {
        double x[6] = {1,2,3,4,5,6};
        assert( 6*sizeof(double) == sizeof(x) );
        AA_ZERO_AR(x);
        aveq( 6, x, AA_FAR(0,0,0,0,0,0), 0 );
    }
    // set_ar
    {
        double x[3] = {0};
        AA_SET_AR(x,1.5);
        aveq(3, x, AA_FAR(1.5,1.5,1.5), 0 );
    }
}

void dbg() {
    aa_tick("usleep(10): ");
    usleep(10);
    aa_tock();

    aa_tick("relsleep(10ms): ");
    aa_tm_relsleep( aa_tm_sec2timespec(10e-3));
    aa_tock();
}

void rotmat() {
    // isrotmat
    {
        double R1[9] = {0,1,0,  -1,0,0,  0,0,1};
        double R2[9] = {1,0,0,  0,1,0,  0,0,-1};
        double R3[9] = {0,2,0,  -2,0,0,  0,0,2};
        assert( aa_tf_isrotmat(R1) );
        assert( ! aa_tf_isrotmat(R2) );
        assert( ! aa_tf_isrotmat(R3) );
    }

    // axang
    {
        double R[9] = {0,1,0,  -1,0,0,  0,0,1};
        double a[4];
        double v[3];
        double ar[4] = {0,0,1,M_PI/2};
        double vr[3] = {0,0,M_PI/2};
        aa_tf_rotmat2axang(R,a);
        aa_tf_rotmat2rotvec(R,v);
        aveq( 4, a, ar, .00001 );
        aveq( 3, v, vr, .00001 );
    }
}


void tf() {
    {
        double T[12] = {0,1,0,  -1,0,0,  0,0,1, 1,2,3};
        double p0[3] = {3, 5, 7};
        double p1[3];
        aa_tf_12(T, p0, p1);
        aveq( 3, p1, AA_FAR(-4,5,10), .001 );
    }
    // inv
    {
        double T[12] = {0,1,0,  -1,0,0,  0,0,1, 1,2,3};
        double Ti[12];
        double p0[3] = {3, 5, 7};
        double p1[3], p0p[3];
        aa_tf_12(T, p0, p1);
        aa_tf_12inv( T, Ti );
        aa_tf_12(Ti, p1, p0p);
        aveq( 3, p0, p0p, .001 );
    }
}
void kin() {
    double ta[2], tb[2];
    assert( 0 == aa_kin_planar2_ik_theta2( AA_FAR(1, 2), AA_FAR(2.2, 2), ta, tb ) );
}

int main( int argc, char **argv ) {
    (void) argc; (void) argv;
    scalar();
    la0();
    la1();
    la2();
    angle();
    quat();
    rotmat();
    axang();
    tf();
    tm();
    mem();
    dbg();
    kin();
}
