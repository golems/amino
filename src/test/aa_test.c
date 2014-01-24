/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

//#define AA_ALLOC_STACK_MAX
#include "amino.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>

void randtf(double T[12], double q[4], double R[9], double v[3]) {
    double axa[4];
    aa_vrand(3,v);
    aa_vrand(4,axa); axa[3] *= M_PI;
    aa_tf_axang2quat( axa, q );
    aa_tf_quat2rotmat( q, T );
    aa_tf_quat2rotmat( q, R );
    AA_MEM_CPY( T+9, v, 3 );
}


static void afeq( double a, double b, double tol ) {
    assert( aa_feq(a,b,tol) );
}

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

static void aneq( double a, double b, double tol ) {
    assert( !aa_feq(a,b,tol) );
}


aa_mem_region_t g_region;

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
        aveq( "la_scal", 3, r, x, 0 );
    }
    // sinc
    {
        double x[] = {1,2,3};
        double r[] = { 1+2, 2+2, 3+2 };
        aa_la_sinc(3, 2, x);
        aveq( "la_sinc", 3, r, x, 0 );
    }
    // vinc
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        double r[] = { 1+4, 2+5, 3+6 };
        aa_la_vinc(3, x, y);
        aveq( "la_vinc", 3, r, y, 0 );
    }
    // axpy
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        double r[] = { 2*1+4, 2*2+5, 2*3+6 };
        aa_la_axpy(3, 2, x, y);
        aveq( "la_axpy", 3, r, y, 0 );
    }

    // axpy3
    {
        double x[] = {1,2,3};
        double y[] = {4,5,6};
        double z[3];
        double r[] = { 2*1+4, 2*2+5, 2*3+6 };
        aa_la_axpy3(3, 2, x, y, z);
        aveq( "la_axpy3", 3, r, z, 0 );
    }
    // sadd
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {2,3,4};
        aa_la_sadd( 3, 1, x, r );
        aveq( "la_sadd", 3, r, y, 0 );
    }
    // ssub
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {-1,-2,-3};
        aa_la_ssub( 3, 0, x, r );
        aveq( "la_ssub", 3, r, y, 0 );
    }
    // smul
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {2,4,6};
        aa_la_smul( 3, 2, x, r );
        aveq( "la_smul", 3, r, y, 0 );
    }
    // sdiv
    {
        double x[] = {1,2,3};
        double r[3];
        double y[] = {2./1, 2./2, 2./3};
        aa_la_sdiv( 3, 2, x, r );
        aveq( "la_sdiv", 3, r, y, 0 );
    }

    // vadd
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 5, 7, 9 };
        aa_la_vadd( 3,  x, y, r );
        aveq( "la_vadd", 3, r, p, 0 );
    }
    // vsub
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 1-4, 2-5, 3-6 };
        aa_la_vsub( 3,  x, y, r );
        aveq( "la_vsub", 3, r, p, 0 );
    }
    // vmul
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 1*4, 2*5, 3*6 };
        aa_la_vmul( 3,  x, y, r );
        aveq( "la_vmul", 3, r, p, 0 );
    }
    // vdiv
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { 1./4, 2./5, 3./6 };
        aa_la_vdiv( 3,  x, y, r );
        aveq( "la_vdiv", 3, r, p, 0 );
    }

    // cross
    {
        double x[] = {1, 2, 3};
        double y[] = {4, 5, 6};
        double r[3];
        double p[] = { -3, 6, -3 };
        aa_la_cross(  x, y, r );
        aveq( "la_cross", 3, r, p, 0 );
    }
    // normalize
    {
        double x[] = {1,2,3};
        double r[] = {0.26726,   0.53452,   0.80178};
        aa_la_normalize(3,x);
        aveq( "la_normalize", 3, r, x, 0.0001 );
    }

    // point_plane
    {
        afeq( aa_la_point_plane( 3, (double[]){1,1,1},
                                 (double[]){1,1,1,1} ),
              2.3094,
              .001 );
        afeq( aa_la_point_plane( 3, (double[]){1,2,3},
                                 (double[]){1,1,1,0} ),
              3.4641,
              .01 );
       afeq( aa_la_point_plane( 3, (double[]){2,0,0},
                                 (double[]){1,0,0,0} ),
              2,
              .01 );
        afeq( aa_la_point_plane( 3, (double[]){1,0,0},
                                 (double[]){1,0,0,1} ),
              2,
              .01 );
        afeq( aa_la_point_plane( 3, (double[]){1,0,0},
                                 (double[]){1,0,0,-1} ),
              0,
              .01 );
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
        aveq( "la_mvmul", 3,y,r,0);
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
            aveq( "la_svd", 2, S, Sr, 0.001);
            for( size_t i = 0; i < 3*3; i ++ )
                assert( fabs( fabs(U[i]) - fabs(Ur[i]) ) < .001 );
        }
        {
            double A[] = {1,2,3,4,5,6}; //3x2
            aa_la_svd(3,2,A,NULL,S,Vt);
            aveq( "la_svd1", 2, S, Sr, 0.001);
            for( size_t i = 0; i < 2*2; i ++ )
                assert( fabs( fabs(Vt[i]) - fabs(Vtr[i]) ) < .001 );
        }

        {
            double A[] = {1,2,3,4,5,6}; //3x2
            aa_la_svd(3,2,A,U,S,Vt);
            aveq( "la_svd2", 2, S, Sr, 0.001);
            for( size_t i = 0; i < 3*3; i ++ )
                assert( fabs( fabs(U[i]) - fabs(Ur[i]) ) < .001 );
            for( size_t i = 0; i < 2*2; i ++ )
                assert( fabs( fabs(Vt[i]) - fabs(Vtr[i]) ) < .001 );
        }
    }
    // EEV
    {
        double A[4] = {1,2,3,4};
        double wr[2], wi[2];
        double rwr[2] = { -0.37228, 5.37228};
        double rwi[2] = {0};
        aa_la_d_eev(2,A,2, wr, wi, NULL, 0, NULL, 0);
        aveq( "eev val", 2, wr, rwr, 1e-4 );
        aveq( "eev val", 2, wi, rwi, 1e-4 );
    }
    // invert
    {
        double A[] = {1,2,3,4};
        double B[] = {-2,1,1.5,-.5};
        aa_la_inv( 2, A );
        aveq( "la_inv", 4, A, B, 0 );
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
        aveq( "la_transpose", 9, A, At, 0 );

    }
    {

        double A[2*4];
        double Ar[] = {1,0, 0,0, 0,1, 0,0};
        aa_la_transpose2( 4, 2,
                          AA_FAR( 1, 0, 0, 0,
                                  0, 0, 1, 0),
                          A );
        aveq( "la_transpose2", 2*4, A, Ar, 0 );
    }
    // inverse3x3
    /* { */
    /*     double R[9] = {0,-1,0, 1,0,0, 0,0,-1}; */
    /*     double S[9]; */
    /*     aa_la_inverse3x3( R, S ); */
    /*     aa_la_inv(3, R ); */
    /*     aveq( "la_inv3x3", 9, R, S, 0.00001 ); */
    /* } */
    // det3x3
    {
        double R[9] = {0,-1,0, 1,0,0, 0,0,-1};
        double S[9];
        //aa_la_inverse3x3( R, S );
        double d = aa_la_det3x3( R );
        //double dt = aa_la_det3x3( S );
        afeq( -1, d, .000001 );
        //afeq( -1, dt, .000001 );
    }

    //dpinv
    {
        double A[] = {1,2,3,4};
        double A_star[4];
        double R[] = {  -1.92649, 0.96746,  1.44818,  -0.47711 };
        aa_la_dpinv(2,2, .005, &A[0], &A_star[0]);
        aveq( "la_dpinv", 4, A_star, R, .0001 );
    }
    {
        double A[] = {1,2,3,4,5,6};
        double A_star[6];
        double R[] =  {-1.30832, -0.32652, 0.65528, 1.06359, 0.32795, -0.40769};
        aa_la_dpinv(2,3, .005, &A[0], &A_star[0]);
        aveq( "la_dpinv2", 6, A_star, R, .0001 );
    }
    // dls
    {
        double A[] = {1,2,3,4,5,6};
        double b[] = {10,20};
        double x_r[] =  {8.1885, 3.2938, -1.6009};
        double x[3];
        aa_la_dls(2, 3, .005, A, b, x);
        aveq( "la_dls", 3, x_r, x, .0001 );
    }
    // dlsnp
    {
        double A[] = {1,2,3,4,5,6};
        double b[] = {10,20};
        double x_r[] = { 8.1938, 3.2954, -1.6029 };
        double x_p[] =  {2,4,6};
        double x[3];
        aa_la_dlsnp(2, 3, .005, A, b, x_p, x);
        aveq( "la_dlsnp", 3, x_r, x, .0001 );
    }
    // linterp
    {
        double t0 = 0, t1 = 1;
        double X0[] = {0, 1, 2, 3};
        double X1[] = {0, 2, 4, 8};
        double Xi[sizeof(X0)/sizeof(double)];
        aa_la_linterp(sizeof(X0)/sizeof(double), t0, X0, t1, X1,
                      0.5, Xi );
        aveq( "la_linterp", sizeof(X0)/sizeof(double),
              Xi, AA_FAR(0, 1.5, 3, (3.0+8.0)/2 ), .00001 );
        aa_la_linterp(sizeof(X0)/sizeof(double), t0, X0, t1, X1,
                      0.75, Xi );
        aveq( "la_linterp1", sizeof(X0)/sizeof(double),
              Xi, AA_FAR(0, 1.75, 3.5, 3+(8.0-3.0)*3/4 ), .00001 );
        aa_la_linterp(sizeof(X0)/sizeof(double), t0, X0, t1, X1,
                      2, Xi );
        aveq( "la_linterp2", sizeof(X0)/sizeof(double),
              Xi, AA_FAR(0, 3, 6, 13 ), .00001 );
        aa_la_linterp(sizeof(X0)/sizeof(double), t0, X0, t1, X1,
                      -1, Xi );
        aveq( "la_linterp3", sizeof(X0)/sizeof(double),
              Xi, AA_FAR(0, 0, 0, -2 ), .00001 );
    }
}

void la3() {
    {
        double X[4];
        double Xr[] = {1.1649, 1.4987, 1.4987, 2.6484};
        aa_la_care_laub( 2, 1, 1,
                         AA_FAR(1, 2, 3, 4), AA_FAR(1,2), AA_FAR(3,4), X );

        aveq( "la_care_laub0", sizeof(X)/sizeof(double),
              X, Xr, .001 );
    }
    {
        double A[] = { 0.1576,    0.4854,    0.4218,
                       0.9706,    0.8003,    0.9157,
                       0.9572,    0.1419,    0.7922 };
        double B[] = { 1.3519,    1.4514,    1.2284,
                       1.4514,    2.0540,    1.6037,
                       1.2284,    1.6037,    1.2803 } ;
        double C[] = { 0.9575,    0.1009,    0.6952,
                       0.1009,    0.0798,    0.2632,
                       0.6952,    0.2632,    1.1703 };

        double X[9];
        double Xr[] = { 1.1776,    0.6206,   -1.0694,
                        0.6206,    1.1884,   -1.2352,
                        -1.0694,   -1.2352,    4.7984 };
        aa_la_transpose( 3, Xr );
        aa_la_care_laub( 3, 3, 3,
                         A, B, C, X );

        aveq( "la_care_laub2", sizeof(X)/sizeof(double),
              X, Xr, .01 );
    }
    // lls
    {
        double A[] = {1,2,
                      3,4};
        aa_la_transpose( 2, A );
        double x[2];
        aa_la_lls( 2, 2, 1, A, (double[]){1,2}, x );
        aveq( "la_lls", sizeof(x)/sizeof(double),
              x, (double[]){0,0.5}, 0.001 );
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
        aveq( "tf_rotvec2axang", 4, aa, aap, .00001 );
    }
    // 2 quat
    {
        double q_r[4] = {1,2,3,4};
        aa_tf_qnormalize(q_r);
        double q[4];
        double a[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_axang2quat(a,q);
        aveq( "tf_axang2quat", 4, q, q_r, 0.001 );
    }
    {
        double aa[4], rv[3], aap[4];
        aa_tf_axang_make( 1,2,3, M_PI/2.0, aa );
        afeq( aa_la_norm(3, aa), 1.0, .00001 );
        aa_tf_axang2rotvec( aa, rv );
        afeq( aa_la_norm(3, rv), M_PI/2.0, .00001 );
        aa_tf_rotvec2axang( rv, aap );
        aveq( "tf_rotvec2axang", 4, aa, aap, .00001 );
    }
    {
        double q_r[4] = {1,2,3,4};
        aa_tf_qnormalize(q_r);
        double q[4];
        double r[3];
        double a[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_axang2rotvec(a,r);
        aa_tf_rotvec2quat(r,q);
        aveq( "tf_rotvec2quat", 4, q, q_r, 0.001 );
    }
    {
        double q[4] = {1,2,3,4};
        aa_tf_qnormalize(q);
        double r[3];
        double a[4];
        double a_r[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_quat2rotvec(q,r);
        aa_tf_rotvec2axang(r,a);
        aveq( "tf_rotvec2axang", 4, a, a_r, 0.001 );
    }
}

void clapack() {
    // mcopy
    {
        double X[3*2] = { 1,2,3, 4,5,6};
        double Y32[3*2] = {0};
        double Y42[4*2] = {0};
        double Y22[2*2] = {0};
        aa_cla_dlacpy(0, 3, 2, X, 3, Y32, 3 );
        aa_cla_dlacpy(0, 3, 2, X, 3, Y42, 4 );
        aa_cla_dlacpy(0, 2, 2, X, 3, Y22, 2 );

        aveq( "cla_dlacpy", 6, Y32, X, 0.001 );
        aveq( "cla_dlacpy", 8, Y42, (double[]){1,2,3,0,4,5,6,0}, 0.001 );
        aveq( "cla_dlacpy", 4, Y22, (double[]){1,2, 4,5}, 0.001 );
    }

    // lapy2/3
    afeq( sqrt( 3*3 + 4*4 ), aa_cla_dlapy2(3, 4), .001 );
    afeq( sqrt( 3*3 + 4*4 + 5*5 ), aa_cla_dlapy3(3, 4, 5), .001 );

    //rand
    {
        int iseed[4] = {1,10,20,30};
        double X[10];
        int n = sizeof(X)/sizeof(X[0]);

        aa_cla_dlaruv( iseed, n, X );
        for( int i = 0; i < n; i++ ) assert( X[i] <= 1 && X[i] >= 0 );

        aa_cla_dlarnv(1, iseed, n, X );
        for( int i = 0; i < n; i++ ) assert( X[i] <= 1 && X[i] >= 0 );

        aa_cla_dlarnv(2, iseed, n, X );
        for( int i = 0; i < n; i++ ) assert( X[i] <= 1 && X[i] >= -1 );
    }
}

void la2_0() {
    // transpose
    {
        double X[3*2] = { 1,2,3, 4,5,6};
        double Y32[3*2] = {0};
        double Y33[3*3] = {0};
        double Y22[2*2] = {0};
        aa_la_d_transpose( 3, 2, X, 3, Y32, 2 );
        aa_la_d_transpose( 3, 2, X, 3, Y33, 3 );
        aa_la_d_transpose( 2, 2, X, 3, Y22, 2 );

        aveq( "la_d_transpose", 6, Y32, (double[]){1,4, 2,5, 3,6}, 0.001 );
        aveq( "la_d_transpose", 9, Y33, (double[]){1,4,0, 2,5,0, 3,6,0}, 0.001 );
        aveq( "la_d_transpose", 4, Y22, (double[]){1,4, 2,5}, 0.001 );
    }

}


void la2_hungarian_helper(size_t m, size_t n, double *A, size_t lda,
                          double *C, size_t ldc,
                          double expected ) {
    size_t p = AA_MAX(m,n);

    ssize_t ir[m], jr[n];

    aa_la_d_assign_hungarian( m, n, A, lda, ir, jr );
    double accum[2] = {0,0};
    for( size_t i = 0; i < p; i ++ ) {
        if( i < m && ir[i] >= 0 )
            accum[0] += AA_MATREF(C,ldc,i,(size_t)ir[i]);
        if( i < n && jr[i] >= 0)
            accum[1] += AA_MATREF(C,ldc,(size_t)jr[i], i);
    }
    afeq(accum[0],expected,0);
    afeq(accum[1],expected,0);
}

void la2_hungarian() {
    {
        double A[] = {4,4,3, 2,3,1, 8,7,6};
        la2_hungarian_helper( 3, 3, A, 3,
                              A, 3,
                              12 );

    }
    {
        // test with padding
        double A[] = {4,4,3,0, 10,18,12,0,  2,3,1,0, 8,7,6,0};
        double B[] = {4,4,3, 10,18,12,  2,3,1, 8,7,6};

        la2_hungarian_helper( 4, 4, A, 4, A, 4, 12 );
        la2_hungarian_helper( 3, 4, A, 4, A, 4, 12 );
        la2_hungarian_helper( 3, 4, B, 3, B, 3, 12 );

    }

    {
        double A[] = {1,2,3, 2,4,6, 3,6,9};
        la2_hungarian_helper( 3, 3, A, 3, A, 3, 3+3+4 );

    }
    {
        double A[] = {23,14,53,99,50,
                      27,94,3,52,100,
                      42,5,88,14,34,
                      55,77,89,1,41,
                      97,22,7,63,92};
        la2_hungarian_helper( 5, 5, A, 5, A, 5, 83 );

    }
    {
        double A[] = {62, 75, 80, 78, 90, 65,
                      75, 80, 75, 82, 85, 75,
                      80, 82, 81, 84, 85, 80,
                      0,   0,  0,  0,  0,  0,
                      93, 85, 98, 80, 80, 75,
                      95, 71, 90, 50, 85, 68,
                      97, 97, 97, 98, 99, 96};

        double Ap[6*7];
        memcpy(Ap, A, sizeof(Ap));
        aa_la_d_assign_hungarian_max2min( 6, 7, Ap, 6 );
        la2_hungarian_helper( 6, 7, Ap, 6,
                              A, 6, 543 );

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
    afeq( aa_ang_norm_2pi(2*M_PI + 3*M_PI ), M_PI, 0 );
    afeq( aa_ang_norm_2pi(-2*M_PI + -M_PI/2 ), 3*M_PI/2, .001 );

    afeq( aa_ang_norm_pi( 3*M_PI/2 ), -M_PI/2, 0 );
    afeq( aa_ang_norm_pi(2*M_PI + 3*M_PI/2 ), -M_PI/2, 0 );
    afeq( aa_ang_norm_pi(-2*M_PI + 3*M_PI/2 ), -M_PI/2, 0 );

    // delta
    afeq( aa_ang_delta( aa_ang_deg2rad(128),
                        aa_ang_deg2rad(128) ),
          aa_ang_deg2rad(0),
          .001 );
    afeq( aa_ang_delta( aa_ang_deg2rad(128),
                        aa_ang_deg2rad(127) ),
          aa_ang_deg2rad(1),
          .001 );
    afeq( aa_ang_delta( aa_ang_deg2rad(128),
                        aa_ang_deg2rad(129) ),
          aa_ang_deg2rad(-1),
          .001 );
   afeq( aa_ang_delta( aa_ang_deg2rad(90),
                       aa_ang_deg2rad(269) ),
          aa_ang_deg2rad(-179),
          .001 );
   afeq( aa_ang_delta( aa_ang_deg2rad(269),
                       aa_ang_deg2rad(90) ),
          aa_ang_deg2rad(179),
          .001 );
   afeq( aa_ang_delta( aa_ang_deg2rad(269 - 360),
                       aa_ang_deg2rad(90+720) ),
          aa_ang_deg2rad(179),
          .001 );
}

void quat() {
    // conj
    {
        double p[4] = {1,2,3,4};
        double q[4];
        double r[4] = {-1,-2,-3,4};
        aa_tf_qconj(p,q);
        aveq( "tf_qconj", 4, q, r, 0.000 );

    }
    // inv
    {
        double p[4] = {1,2,3,4};
        double q[4];
        double r[4] = { -0.0333333, -0.0666666, -0.1, 0.133333 };
        aa_tf_qinv(p,q);
        aveq( "tf_qinv", 4, q, r, 0.0001 );
    }
    // mul
    {
        double a[4] = {1,2,3,4};
        double b[4] = {9,8,7,6};
        double c[4];
        double r[4] = {32, 64, 36, -22};
        aa_tf_qmul(a,b,c);
        aveq( "tf_qmul", 4, c, r, 0.0000 );
    }
    // 2 axis-angle
    {
        double a[4] = {1,2,3,4};
        double b[4];
        double r[4] = { 0.26726, 0.53452, 0.80178, 1.5041 };
        aa_tf_qnormalize(a);
        aa_tf_quat2axang(a,b);
        aveq( "tf_quat2axang-1", 4, b, r, 0.001 );
    }
    // 2 axis-angle
    {
        double ra_r[4] = {0,0,0,0};
        double ra[4];
        aa_tf_quat2axang( AA_FAR(0,0,0,1), ra);
        aveq( "tf_quat2axang-2", 4, ra, ra_r, 0.001 );
    }

    // from axis-angle
    {
        double q_r[4] = {0,0,0,1};
        double q[4];
        aa_tf_axang2quat( AA_FAR(0,0,0,0), q);
        aveq( "tf_axang2quat", 4, q, q_r, 0.00001 );
        aa_tf_axang2quat( AA_FAR(1,1,1,0), q);
        aveq( "tf_axang2quat", 4, q, q_r, 0.00001 );
        aa_tf_axang2quat( AA_FAR(1,2,3,0), q);
        aveq( "tf_axang2quat", 4, q, q_r, 0.00001 );
    }
    // rotvec identity
    {
        double rv[3], q[4];
        aa_tf_rotvec2quat(AA_TF_ROTVEC_IDENT, q);
        aveq( "tf_rotvec2quat", 4,AA_TF_QUAT_IDENT,q,0.000001 );
        aa_tf_quat2rotvec(AA_TF_QUAT_IDENT, rv);
        aveq( "tf_rotvec2quat", 3,AA_TF_ROTVEC_IDENT,rv,0.000001 );
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
        aveq( "tf rotations", 9, R, Rzi, .0001 );
        aveq( "tf rotations", 3,vp_r0, vp_q, .0001 );
        aveq( "tf rotations", 3,vp_r0, vp_r1, .0001 );
    }
    {
        double q[4];
        aa_tf_rotmat2quat(AA_TF_IDENT,q);
        aveq( "tf_rotmat2quat", 4, q, AA_TF_QUAT_IDENT, .001 );
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
        aveq( "tf_qrot", 3, pr, pq, .00001 );
    }

    // slerp
    {
        double R0[9], R_5[9], R1[9];
        double q0[4], q_5[4], q1[4];
        double qs[4], qas[4];

        aa_tf_xangle2rotmat( 0.0, R0 );
        aa_tf_xangle2rotmat( M_PI_2, R_5 );
        aa_tf_xangle2rotmat( M_PI, R1);

        aa_tf_rotmat2quat(R0, q0);
        aa_tf_rotmat2quat(R_5, q_5);
        aa_tf_rotmat2quat(R1, q1);

        aa_tf_qslerp( 0, q0, q1, qs );
        aa_tf_qslerpalg( 0, q0, q1, qas );
        aveq( "qslerp 0", 4, q0, qs, .00001 );
        aveq( "qslerpalg 0", 4, q0, qas, .00001 );

        aa_tf_qslerp( 1, q0, q1, qs );
        aa_tf_qslerpalg( 1, q0, q1, qas );
        aveq( "qslerp 1", 4, q1, qs, .00001 );
        aveq( "qslerpalg 1", 4, q1, qas, .00001 );

        aa_tf_qslerp( .5, q0, q1, qs );
        aa_tf_qslerpalg( .5, q0, q1, qas );
        aveq( "qslerp .5", 4, q_5, qs, .00001 );
        aveq( "qslerpalg .5", 4, q_5, qas, .00001 );

        /* slerp diff */
        double dr_dtau[4];
        double w[3], dr_dtau_w[4];
        aa_tf_qslerpdiff( .5, q0, q1, dr_dtau );
        /* dtau/dt == 1.0 */
        aa_tf_qdiff2vel( qs, dr_dtau, w );
        aa_tf_qvel2diff( qs, w, dr_dtau_w );
        aveq( "qslerp diff vel", 4, dr_dtau, dr_dtau_w, .001 );

    }

}

void tm() {
    struct timespec t;

    t = aa_tm_make_norm( 100, AA_IBILLION + 1 );
    assert( 101 == t.tv_sec && 1 == t.tv_nsec );

    t = aa_tm_make_norm( 100,  -1 );
    assert( 99 == t.tv_sec && AA_IBILLION - 1 == t.tv_nsec );

    t = aa_tm_make_norm( 100,  -AA_IBILLION - 1 );
    assert( 98 == t.tv_sec && AA_IBILLION - 1 == t.tv_nsec );

    // math/cmp
    {
        struct timespec now =  aa_tm_now();
        struct timespec ms = aa_tm_sec2timespec(1e-3);

        assert( 0 ==  aa_tm_cmp( now, aa_tm_add(aa_tm_sub(now, ms), ms) ) );

        assert( 0 < aa_tm_cmp( now, aa_tm_sub( now, ms ) ) );
        assert( 0 > aa_tm_cmp( now, aa_tm_add( now, ms ) ) );


    }

    // rdtsc
#ifdef AA_FEATURE_RDTSC_NOT
    {
        uint64_t a0,a1;
        a0 = aa_rdtsc();
        a1= aa_rdtsc();
        printf("tickdiff: %" PRIu64 "\n",
                   a1 - a0);
        int k = 1;
        for( int i = 1; i < 12; i ++ ) {
            k *= 2;
            a0 = aa_rdtsc();
            usleep((__useconds_t)k);
            //aa_tm_now();
            a1 = aa_rdtsc();
            printf("us: %d, tickdiff: %"PRIu64", ns/tick: %f\n", k, a1 - a0,
                   ((double)k)*1000 / (double)(a1-a0)
                );
        }

    }
#endif// AA_FEATURE_RDTSC_NOT
}

void mem() {
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 10);
        void *p = aa_mem_region_alloc(&reg,10);
        memset(p,0,10);
        assert(! reg.node->next );
        aa_mem_region_destroy(&reg);
    }
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 2);
        void *p = aa_mem_region_alloc(&reg,10);
        memset(p,0,10);
        assert(! reg.node->next );
        assert( reg.node->end >= (uint8_t*) p + 10 );
        p = aa_mem_region_alloc(&reg,30);
        assert( reg.node->next );
        memset(p,0,30);
        aa_mem_region_destroy(&reg);
    }
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 256);
        double *p = (double*)aa_mem_region_alloc(&reg,sizeof(double));
        double *q = (double*)aa_mem_region_alloc(&reg,sizeof(double));
        *p = 1.0;
        *q = 2.0;
        afeq( *p, 1.0, 0 );
        afeq( *q, 2.0, 0 );
        *q = 3.0;
        *p = 4.0;
        afeq( *p, 4.0, 0 );
        afeq( *q, 3.0, 0 );
        aa_mem_region_destroy(&reg);
    }
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 1);
        double *p = (double*)aa_mem_region_alloc(&reg,sizeof(double));
        double *q = (double*)aa_mem_region_alloc(&reg,sizeof(double));
        *p = 1.0;
        *q = 2.0;
        afeq( *p, 1.0, 0 );
        afeq( *q, 2.0, 0 );
        *q = 3.0;
        *p = 4.0;
        afeq( *p, 4.0, 0 );
        afeq( *q, 3.0, 0 );
        aa_mem_region_destroy(&reg);
    }
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 256);

        char *a = (char*)aa_mem_region_alloc(&reg, 4096);
        memset(a,10,4096);
        for(size_t i = 0; i < 4096; i ++ ) assert( 10 == a[i] );

        char *b = (char*)aa_mem_region_alloc(&reg, 4096);
        memset(b,11,4096);
        for(size_t i = 0; i < 4096; i ++ ) assert( 10 == a[i] );
        for(size_t i = 0; i < 4096; i ++ ) assert( 11 == b[i] );

        char *c = (char*)aa_mem_region_alloc(&reg, 4096);
        memset(c,12,4096);
        for(size_t i = 0; i < 4096; i ++ ) assert( 10 == a[i] );
        for(size_t i = 0; i < 4096; i ++ ) assert( 11 == b[i] );
        for(size_t i = 0; i < 4096; i ++ ) assert( 12 == c[i] );

        aa_mem_region_destroy(&reg);
    }
    // topsize
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 16);
        size_t old_n = aa_mem_region_topsize(&reg);
        void *ptr = aa_mem_region_ptr(&reg);
        assert( ptr == aa_mem_region_tmpalloc(&reg,old_n) );
        assert( ptr == aa_mem_region_tmpalloc(&reg,old_n) );
        assert( ptr != aa_mem_region_tmpalloc(&reg,old_n+1) );

    }
    // region growth
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 16);
        for( size_t i = 0; i < 10; i ++ ) {
            assert( ! reg.node->next );
            size_t old_n = aa_mem_region_topsize(&reg);
            size_t n = old_n / 16 + 1;
            for( size_t j = 0; j < n; j ++ ) {
                aa_mem_region_alloc(&reg, 16);
            }
            assert( reg.node->next );
            assert( ! reg.node->next->next );
            assert( aa_mem_region_topsize(&reg) > old_n );
            aa_mem_region_release(&reg);
            assert( ! reg.node->next );
            assert( aa_mem_region_topsize(&reg) >= old_n + 16 );
            assert( aa_mem_region_topsize(&reg) <= 8*old_n );
        }
        for( size_t i = 0; i < 256; i ++ ) {
            for( size_t j = 0; j < i + 2; j ++ ) {
                aa_mem_region_alloc(&reg, 16);
            }
            assert( NULL == reg.node->next );
            aa_mem_region_release(&reg);
        }
        aa_mem_region_destroy(&reg);
    }
    // pop simple
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 256);
        void *p0 = aa_mem_region_alloc(&reg,4);
        void *p1 = aa_mem_region_alloc(&reg,4);
        void *p2 = aa_mem_region_alloc(&reg,4);
        void *p3 = aa_mem_region_alloc(&reg,4);
        aa_mem_region_pop(&reg, p2);
        void *p2a = aa_mem_region_alloc(&reg,4);
        void *p3a = aa_mem_region_alloc(&reg,4);
        assert( p2a == p2 );
        assert( p3a == p3 );
        aa_mem_region_pop(&reg, p0);
        assert(AA_ALIGN2((intptr_t)reg.node->d,AA_MEMREG_ALIGN) == (intptr_t)reg.head);
        void *p0a = aa_mem_region_alloc(&reg,4);
        void *p1a = aa_mem_region_alloc(&reg,4);
        assert( p0a == p0 );
        assert( p1a == p1 );
        aa_mem_region_destroy(&reg);
    }
    // pop chain
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 32);
        assert( 1 == aa_mem_region_chunk_count(&reg) );
        void *p0 = aa_mem_region_alloc(&reg,2);
        assert( AA_ALIGN2((intptr_t)reg.node->d,AA_MEMREG_ALIGN)+16 ==
                (intptr_t)reg.head ); // align16
        void *p1 = aa_mem_region_alloc(&reg,64);
        (void)p1;
        assert( AA_ALIGN2((intptr_t)reg.node->d,AA_MEMREG_ALIGN)+64 ==
                (intptr_t)reg.head );
        assert( 2 == aa_mem_region_chunk_count(&reg) );
        for( size_t i = 0; i < 5; i++ ) {
            uint8_t *v1 = (uint8_t*)aa_mem_region_alloc( &reg, 2<<20 );
            uint8_t *v2 = (uint8_t*)aa_mem_region_alloc( &reg, 2<<20 );
            // check memory reuse
            if( i > 2 ) assert(*v2 == 2); else *v2 = 2;
            if( i > 2 ) assert(*v1 == 1); else *v1 = 1;
            // check chunk counts
            if( i )  assert( 3 == aa_mem_region_chunk_count(&reg) );
            else assert( 4 == aa_mem_region_chunk_count(&reg) );
            aa_mem_region_pop( &reg, v1 );
            assert( 3 == aa_mem_region_chunk_count(&reg) );
        }
        assert( reg.node->d <= reg.head );
        aa_mem_region_pop( &reg, p0 );
        aa_mem_region_destroy(&reg);
    }
    { //tmp alloc
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 5 * (2<<20) );
        for( size_t i = 0; i < 2 << 20; i++ ) {
            aa_mem_region_tmpalloc(&reg, 2<<20);
        }
        assert( NULL == reg.node->next );
        assert( 5*(2<<20) + AA_MEMREG_ALIGN == aa_mem_region_topsize(&reg) );
        aa_mem_region_destroy(&reg);
    }
    // fuzz test, random allocations and pops
    {
#define N_ALLOC 33
#define N_LOOP 83
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, (size_t)(rand() % 128) );
        size_t m =(size_t) rand() % N_LOOP; // total iterations
        uint8_t *ptrs[N_ALLOC] = {0}; // region allocated buffers
        size_t lens[N_ALLOC] = {0};   // region buffer lengths
        uint8_t data[N_ALLOC][256]; // static buffer to hold test data
        size_t j = 0; // ofset into ptrs, lens, data arrays
        for( size_t i = 0; i < m; i ++ ) {
            size_t n = (size_t)rand()%N_ALLOC; // number of allocations this round
            // alloc some stuff
            for( ; j < n ;j ++ ) {
                assert( j < N_ALLOC );
                assert(0 == lens[j]);
                assert(NULL == ptrs[j]);
                lens[j] = (size_t)rand()%255 + 1;
                ptrs[j] = (uint8_t*)aa_mem_region_alloc(&reg, lens[j]);
                // fill with random data
                for( size_t p = 0; p < lens[j]; p++ ) {
                    data[j][p] = ptrs[j][p] = (uint8_t) (rand()%256);
                }
                assert( 0 == (uintptr_t)reg.head % 16 );
                assert(ptrs[j] >= reg.node->d);
                assert(lens[j] <= (size_t)(reg.head - reg.node->d));
            }
            // check that everything still has the proper data
            if(j) {
                for( size_t k = 0; k < j; k++ ) {
                    for( size_t p = 0; p < lens[k]; p++ ) {
                        assert(ptrs[k][p] == data[k][p]);
                    }
                }
                j = (size_t)rand()%j; // buffer to pop from region
                aa_mem_region_pop( &reg, ptrs[j] );
                for( size_t k = j; k < N_ALLOC; k++ ) {
                    lens[k] = 0; ptrs[k] = NULL;
                }
            }
        }
        aa_mem_region_destroy(&reg);
    }
    // region printf
    {
        aa_mem_region_t reg;
        aa_mem_region_init(&reg, 5);
        char *foo = aa_mem_region_printf(&reg, "foo%d",1);
        assert( 0 == strcmp(foo, "foo1") );
        assert( ! reg.node->next );
        char *bar = aa_mem_region_printf(&reg, "bar%d",2);
        assert( 0 == strcmp(bar, "bar2") );
        assert( 0 == strcmp(foo, "foo1") );
        const char *a = "0123456789abcdef";
        const char *b = "0123456789ABCDEF";
        assert(16 == strlen(a));
        assert(16 == strlen(b));
        char *fa = aa_mem_region_printf(&reg, "%s", a);
        assert(reg.node->next);
        assert(0 == strcmp(a, fa));
        char *fb = aa_mem_region_printf(&reg, "%s", b);
        assert(0 == strcmp(b, fb));
        assert(0 == strcmp(a, fa));
        aa_mem_region_destroy(&reg);
    }
    // local region
    {

        int *a = (int*)aa_mem_region_local_alloc(sizeof(int)*4);
        for( size_t i = 0; i < 4; i ++ ) a[i] = 42;
        int *b = (int*)aa_mem_region_local_alloc(sizeof(int)*4);
        for( size_t i = 0; i < 4; i ++ ) b[i] = 4;

        for( size_t i = 0; i < 4; i ++ ) {
            assert( 4 == b[i] );
            assert( 42 == a[i] );
        }
    }
    // zero_ar
    {
        double x[6] = {1,2,3,4,5,6};
        assert( 6*sizeof(double) == sizeof(x) );
        AA_MEM_ZERO(x,6);
        aveq( "zero_ar", 6, x, AA_FAR(0,0,0,0,0,0), 0 );
    }
    // set_ar
    {
        double x[3] = {0};
        AA_SET_AR(x,1.5);
        aveq( "set_ar", 3, x, AA_FAR(1.5,1.5,1.5), 0 );
    }
}

void dbg() {
    //aa_tick("usleep(1000): ");
    //usleep(1000);
    //aa_tock();

    //aa_tick("relsleep(10ms): ");
    //aa_tm_relsleep( aa_tm_sec2timespec(10e-3));
    //aa_tock();
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
        aveq( "tf_rotmat2axang", 4, a, ar, .00001 );
        aveq( "tf_rotmat2rotvec", 3, v, vr, .00001 );
    }
    // euler
    {
        double R[9] = {1,0,0,  0,1,0,  0,0,1};
        double e[3];
        double R1[9];
        aa_tf_rotmat2eulerzyx( R, e );
        aa_tf_eulerzyx2rotmat( e[0], e[1], e[2], R1 );
        aveq( "tf_rotmat2eulerzyx", 9, R, R1, .001 );

        double q[4], qr[4];
        aa_tf_eulerzyx2quat( e[0], e[1], e[2], q );
        aa_tf_rotmat2quat(R, qr);
        aveq( "tf_eulerzyx2quat", 4, q, qr, .001 );
    }
    // euler
    {
        double e[3] = { M_PI_2, 0, 0};
        double R1[9];
        double R1r[9] = {0,1,0, -1,0,0, 0,0,1};
        aa_tf_eulerzyx2rotmat( e[0], e[1], e[2], R1 );
        aveq( "tf_eulerzyx2rotmat", 9, R1r, R1, .001 );

        double q[4], qr[4];
        aa_tf_eulerzyx2quat( e[0], e[1], e[2], q );
        aa_tf_rotmat2quat(R1, qr);
        aveq( "tf_eulerzyx2quat", 4, q, qr, .001 );
    }

    //xyzangle
    {
        double R_x90[9] = { 1,0,0, 0,0,1, 0,-1,0 };
        double R_y90[9] = { 0,0,-1, 0,1,0, 1,0,0 };
        double R_z90[9] = { 0,1,0, -1,0,0, 0,0,1 };
        double R[9];
        double Rr[9];

        aa_tf_xangle2rotmat( M_PI_2, R );
        aa_tf_axang2rotmat( AA_FAR(1,0,0,M_PI_2), Rr );
        assert( aa_tf_isrotmat(R) );
        aveq( "tf_xangle2rotmat", 9, R, R_x90, .001 );
        aveq( "tf_axang2rotmat", 9, R, Rr, .001 );

        aa_tf_yangle2rotmat( M_PI_2, R );
        aa_tf_axang2rotmat( AA_FAR(0,1,0,M_PI_2), Rr );
        assert( aa_tf_isrotmat(R) );
        aveq( "tf_yangle2rotmat", 9, R, R_y90, .001);
        aveq( "tf_yangle2rotmat", 9, R, Rr, .001);

        aa_tf_zangle2rotmat( M_PI_2, R );
        assert( aa_tf_isrotmat(R) );
        aveq( "tf_zangle2rotmat", 9, R, R_z90, .001);
        aa_tf_axang2rotmat( AA_FAR(0,0,1,M_PI_2), Rr );
    }

}


void tf() {
    // tf
    {
        double T[12] = {0,1,0,  -1,0,0,  0,0,1, 1,2,3};
        double p0[3] = {3, 5, 7};
        double p1[3];
        aa_tf_12(T, p0, p1);
        aveq( "tf_12", 3, p1, AA_FAR(-4,5,10), .001 );
        aa_tf_93(T, T+9, p0, p1);
        aveq( "tf_93", 3, p1, AA_FAR(-4,5,10), .001 );
    }
    {
        double T[12] = {0,1,0,  -1,0,0,  0,0,1, 0,0,0};
        double p0[3] = {3, 5, 7};
        double p1_12[3];
        double p1_93[3];
        double p1_9[3];
        aa_tf_12(T, p0, p1_12);
        aa_tf_93(T, T+9,  p0, p1_93);
        aa_tf_9(T,  p0, p1_9);
        aveq( "tf_9", 3, p1_12, p1_93, .001 );
        aveq( "tf_9", 3, p1_12, p1_9, .001 );
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
        aveq( "tf_12_inv", 3, p0, p0p, .001 );
    }
    // chain
    {
        double T1[12] = {0,1,0,  -1,0,0,  0,0,1, 1,2,3};
        double T2[12] = {1,0,0,  0,1,0,  0,0,-1, 5,4,2};
        double T[12];
        aa_tf_12chain(T1, T2, T );
        double p0[3] = { 10, 1, 4 };
        double p1[3];
        aa_tf_12( T, p0, p1 );
        aveq( "tf_12chain", 3, p1, AA_FAR(-4,17,1), .001 );
    }
    // rel
    {
        double T1[12] = {0,1,0,  -1,0,0,  0,0,1, 1,2,3};
        double T2[12] = {1,0,0,  0,1,0,  0,0,-1, 5,4,2};
        double Trel[12];
        aa_tf_12rel( T1, T2, Trel );
        aveq( "tf_12rel", 12, Trel,
              AA_FAR( 0,-1,0,  1,0,0,  0,0,-1,  2, -4, -1 ),
              .001 );
    }
}

void tffuzz() {
    for( size_t k = 0; k < 1000; k ++ ) {
        // tfs
        double v[3], T[12], q[4], R[9];
        randtf(T,q,R,v);
        double v1[3], T1[12], q1[4], R1[9];
        randtf(T1,q1,R1,v1);

        // point
        double p0[3];
        aa_vrand(3,p0);

        // transform
        double p1a[3], p1b[3];
        aa_tf_qrot(q, p0, p1a);
        aa_tf_9rot(R, p0, p1b);
        aveq( "tf_qrot", 3, p1a, p1b, .001 );
        aa_tf_9rot(T, p0, p1b);
        aveq( "tf_9rot", 3, p1a, p1b, .001 );

        aa_tf_12(T, p0, p1a);
        aa_tf_93(R, v, p0, p1b);
        aveq( "tf_93", 3, p1a, p1b, .001 );

        // mul
        double qc[4], Rc[9];
        aa_tf_qmul( q, q1, qc );
        aa_tf_9mul( R, R1, Rc );
        aa_tf_qrot(q, p0, p1a);
        aa_tf_9rot(R, p0, p1b);
        aveq( "tf_qmul", 3, p1a, p1b, .001 );
        // inv
        aa_tf_qinv( q, qc );
        aa_la_transpose2( 3, 3, R, Rc );
        aa_tf_qrot(q, p0, p1a);
        aa_tf_9rot(R, p0, p1b);
        aveq( "tf_qrot", 3, p1a, p1b, .001 );
    }

    // 9mul
    {
        double v[5][3], T[5][12], q[5][4], R[5][9];
        double Ttmp[5][12], Rtmp[4][9], Rn[9], Tn[12];
        for( size_t i = 0; i < 5; i ++ ) {
            randtf(T[i], q[i], R[i], v[i]);
        }
        aa_tf_9mul( R[0], R[1], Rtmp[0] );
        aa_tf_9mul( Rtmp[0], R[2], Rtmp[1] );
        aa_tf_9mul( Rtmp[1], R[3], Rtmp[2] );
        aa_tf_9mul( Rtmp[2], R[4], Rtmp[3] );
        aa_tf_v9mul( Rn, R[0], R[1], R[2], R[3], R[4], NULL );
        aveq( "tf_v9muln", 9, Rtmp[3], Rn, .001 );

        aa_tf_12chain( T[0], T[1], Ttmp[0] );
        aa_tf_12chain( Ttmp[0], T[2], Ttmp[1] );
        aa_tf_12chain( Ttmp[1], T[3], Ttmp[2] );
        aa_tf_12chain( Ttmp[2], T[4], Ttmp[3] );
        aa_tf_v12chain( Tn, T[0], T[1], T[2], T[3], T[4], NULL );
        aveq( "tf_v12chain", 12, Ttmp[3], Tn, .001 );
    }
}

void kin() {
    double ta[2], tb[2];
    assert( 0 == aa_kin_planar2_ik_theta2( AA_FAR(1, 2), AA_FAR(2.2, 2), ta, tb ) );
}






void dump( void *bytes, size_t s ) {
  size_t i;
  for( i = 0; i < s; i ++ ) {
    printf("%x:", ((uint8_t*)bytes)[i] );
  }
}

int endconv() {

    uint8_t le_i32_1[] = {1,0,0,0};
    uint8_t be_i32_1[] = {0,0,0,1};

    int32_t *pbe1 =  ((int32_t*) be_i32_1);
    int32_t *ple1 =  ((int32_t*) le_i32_1);
    int32_t be1 =  *pbe1;
    int32_t le1 =  *ple1;

    uint8_t lebuf[4], bebuf[4];

    // ld
    assert( 1 == aa_endconv_ld_le_i32( le_i32_1 ) );
    assert( 1 == aa_endconv_ld_be_i32( be_i32_1 ) );

    // xe_to_h
    assert( 1 == aa_endconv_be_to_h_i32( be1 ) );
    assert( 1 == aa_endconv_le_to_h_i32( le1 ) );

    // h_to_xe
    assert( aa_endconv_h_to_le_i32(1) == le1 );
    assert( aa_endconv_h_to_be_i32(1) == be1 );

    // st
    aa_endconv_st_le_i32( lebuf, 1 );
    aa_endconv_st_be_i32( bebuf, 1 );
    assert( 0 == memcmp( lebuf, le_i32_1, 4 ) );
    assert( 0 == memcmp( bebuf, be_i32_1, 4 ) );

    return 0;
}


void validate() {
    //aa_valid_f
    assert( 0 == aa_valid_f( 1.0, 0.9, 1.1 ) );
    assert( 0 != aa_valid_f( 0.8, 0.9, 1.1 ) );
    assert( 0 != aa_valid_f( 1.2, 0.9, 1.1 ) );

    //aa_valid_v
    assert( 0 == aa_valid_v( AA_FAR(1,2,3), 3,
                             AA_FAR(0,1,2), AA_FAR(2,3,4), 3 ) );
    assert( 0 > aa_valid_v( AA_FAR(1,2,3,4), 4,
                             AA_FAR(0,1,2), AA_FAR(2,3,4), 3 ) );
    assert( 0 > aa_valid_v( AA_FAR(1,2), 2,
                             AA_FAR(0,1,2), AA_FAR(2,3,4), 3 ) );
    assert( 2 == aa_valid_v( AA_FAR(1,2,3), 3,
                             AA_FAR(0,3,2), AA_FAR(2,3,4), 3 ) );
    assert( 2 == aa_valid_v( AA_FAR(1,2,3), 3,
                             AA_FAR(0,1,2), AA_FAR(2,1,4), 3 ) );
    assert( 3 == aa_valid_v( AA_FAR(1,2,0), 3,
                             AA_FAR(0,1,2), AA_FAR(2,3,4), 3 ) );
    //aa_valid_vunit
    {
        double a[] = {1,2,3};
        double b[] = {1,2,3};
        aa_la_normalize(sizeof(a)/sizeof(double),a);
        assert(0 == aa_valid_vunit(a, sizeof(a)/sizeof(double), 1e-4));
        assert(0 != aa_valid_vunit(b, sizeof(a)/sizeof(double), 1e-4));
    }

    // timespec
    {
        struct timespec now =  aa_tm_now();
        struct timespec sec = aa_tm_sec2timespec(1);
        struct timespec ms = aa_tm_sec2timespec(1e-3);
        assert( 0 == aa_valid_timespec( now,
                                        aa_tm_sub(now, ms),
                                        aa_tm_add(now, ms) ) );
        assert( 0 != aa_valid_timespec( aa_tm_sub(now, ms),
                                        now,
                                        aa_tm_add(now, ms) ) );
        assert( 0 != aa_valid_timespec( aa_tm_add(now, ms),
                                        aa_tm_sub(now, ms),
                                        now ) );
        assert( 0 == aa_valid_timespec( now,
                                        aa_tm_sub(now, sec),
                                        aa_tm_add(now, sec) ) );

        assert( 0 != aa_valid_timespec( aa_tm_sub(now, sec),
                                        now,
                                        aa_tm_add(now, sec) ) );
        assert( 0 != aa_valid_timespec( aa_tm_add(now, sec),
                                        aa_tm_sub(now, sec),
                                        now ) );
        assert( 0 == aa_valid_timespec( now,
                                        aa_tm_sub(now, ms),
                                        aa_tm_add(now, sec) ) );
        assert( 0 == aa_valid_timespec( now,
                                        aa_tm_sub(now, sec),
                                        aa_tm_add(now, ms) ) );
    }

}

void io() {
    {
        void *buf = NULL;
        size_t max = 0;
        size_t off = 0;
        uint64_t data = 42;
        int fd[2];
        int r = pipe(fd);
        assert(0 == r);
        ssize_t s = write( fd[1], &data, sizeof(data) );
        assert(sizeof(data) == s);
        s = aa_read_realloc( fd[0], &buf, off, &max );
        assert(sizeof(data) == s);
        assert(*(uint64_t*)buf == data);
        assert( max - off >= sizeof(data) );
        free(buf);
    }
}

void sigsys() {
    // lsim_dstep
    {
        double A[] = {1,2,3,4};
        double B[] = {5,6};
        double x0[] = {1,2};
        double u[] = {3};

        double x1[2];

        aa_lsim_dstep( 2, 1,
                       A,B,
                       x0, u, x1  );
        aveq( "lsim_dstep", 2, x1, (double[]){22,28}, 0 );

        aa_lsim_estep( 2, 1, .01,
                       A,B,
                       x0, u, x1  );
        aveq( "lsim_estep", 2, x1, (double[]){1.22,2.28}, 0.001 );
    }
    // rk1
    {
        double x0[] = {1,2};
        double dx[] = {3,4};
        double dt = .1;
        double x1[2];

        aa_odestep_rk1( 2, .1, dx, x0, x1 );

        aveq( "odestep_rk1", 2, x1, (double[]){x0[0]+dt*dx[0], x0[1]+dt*dx[1]}, 0.001 );
    }

    // aa_sys_affine
    {
        double x[] = {1,2};
        double dx[2];
        aa_sys_affine_t sys = { .n = 2,
                                .A = (double[]){1,2,3,4},
                                .D = (double[]){5,6}
        };
        aa_sys_affine( &sys, 0, x, dx );
        aveq( "sys_affine", 2, dx, (double[]){12,16}, 0 );
    }
    // rk2/4/23/45
    {
        double x0[] = {1,2};
        double x1[2];
        double x4[2];
        double x5[2];
        aa_sys_affine_t sys = { .n = 2,
                                .A = (double[]){1,2,3,4},
                                .D = (double[]){5,6}
        };

        // rk2
        aa_odestep_rk2( 2, (aa_sys_fun*)aa_sys_affine, &sys, 0, .01,
                        x0, x1 );
        aveq( "odestep_rk2", 2, x1, (double[]){1.123055,2.16448}, .001 );

        // rk4
        aa_odestep_rk4( 2, (aa_sys_fun*)aa_sys_affine, &sys, 0, .01,
                        x0, x1 );
        aveq( "odestep_rk4", 2, x1, (double[]){1.123055,2.16448}, .001 );

        // rkf45
        {
            double k[5][2];
            double dx0[2];
            aa_sys_affine( &sys, 0, x0, dx0 );
            memcpy(k[0],dx0,sizeof(dx0));
            aa_odestep_rkf45( 2, (aa_sys_fun*)aa_sys_affine, &sys, 0, .01,
                              x0, k[0], x4, x5 );
            aveq( "odestep_rkf45", 2, x4, (double[]){1.123055,2.16448}, .001 );
            aveq( "odestep_rkf45", 2, x5, (double[]){1.123055,2.16448}, .001 );
        }

        // rkck45
        {
            double k[5][2];
            double dx0[2];
            aa_sys_affine( &sys, 0, x0, dx0 );
            memcpy(k[0],dx0,sizeof(dx0));
            aa_odestep_rkck45( 2, (aa_sys_fun*)aa_sys_affine, &sys, 0, .01,
                               x0, k[0], x4, x5 );
            aveq( "odestep_rkck45", 2, dx0, k[0], 0 );
            aveq( "odestep_rkck45", 2, x4, (double[]){1.123055,2.16448}, .001 );
            aveq( "odestep_rkck45", 2, x5, (double[]){1.123055,2.16448}, .001 );
        }

        // dorpri45
        {
            double k[6][2];
            double dx0[2], dx4[2];
            aa_sys_affine( &sys, 0, x0, dx0 );
            memcpy(k[0],dx0,sizeof(dx0));
            aa_odestep_dorpri45( 2, (aa_sys_fun*)aa_sys_affine, &sys, 0, .01,
                                 x0, k[0], x4, x5 );
            aa_sys_affine( &sys, 0, x4, dx4 );
            aveq( "odestep_dorpri45-1", 2, dx0, k[0], 0 );
            aveq( "odestep_dorpri45-2", 2, dx4, k[5], 0 );
            aveq( "odestep_dorpri45-3", 2, x4,
                  (double[]){1.123055,2.16448}, .001 );
            aveq( "odestep_dorpri45-4", 2, x5,
                  (double[]){1.123055,2.16448}, .001 );
        }

        // rkbs23
        {
            double k[4][2];
            double dx0[2], dx2[2], x2[2], x3[2];
            aa_sys_affine( &sys, 0, x0, dx0 );
            memcpy(k[0],dx0,sizeof(dx0));
            aa_odestep_rkbs23( 2, (aa_sys_fun*)aa_sys_affine, &sys, 0, .01,
                               x0, k[0], x2, x3 );
            aa_sys_affine( &sys, 0, x2, dx2 );
            aveq( "odestep_rkbs23", 2, dx0, k[0], 0 );
            aveq( "odestep_rkbs23", 2, dx2, k[3], 0 );
            aveq( "odestep_rkbs23",
                  2, x2, (double[]){1.123055,2.16448}, .001 );
            aveq( "odestep_rkbs23",
                  2, x3, (double[]){1.123055,2.16448}, .001 );
        }
    }

}

void vision() {
    double r[3];

    {
        double rgb[3] = {1,0,0};
        double hsv[3] = {0,1,1};
        aa_cv_rgb2hsv(rgb, r);
        aveq( "cv_rgb2hsv", 3, r, hsv, .001);
    }

    {
        double rgb[3] = {.5,1,1};
        double hsv[3] = {M_PI,.5,1};
        aa_cv_rgb2hsv(rgb, r);
        aveq( "cv_rgb2hsv", 3, r, hsv, .001);
    }

    {
        double rgb[3] = {.5, .5, 1};
        double hsv[3] = {aa_ang_norm_pi(240*M_PI/180), .5, 1};
        aa_cv_rgb2hsv(rgb, r);
        aveq( "cv_rgb2hsv", 3, r, hsv, .001);
    }

}


void stat() {
    {
        double x[] = {1,2,3,4,5,4,3,2,1};
        double s = aa_stat_std(sizeof(x)/sizeof(double), x);

        afeq(s, 1.3944, .001);
    }
    {
        double x[] = {1,2,3,4,5,6,7,8,9,10,1000000};
        double s,m,m0,s0;
        aa_stat_excluded_mean_std( sizeof(x)/sizeof(x[0]), x,
                                   &m, &s,
                                   -2, 2,
                                   10 );
        m0 = aa_stat_mean( sizeof(x)/sizeof(x[0]) - 1, x );
        s0 = aa_stat_std( sizeof(x)/sizeof(x[0]) - 1, x );
        afeq( s, s0, .001 );
        afeq( m, m0, .001 );
    }
    {
        double x[] = {1,2,3,4,5,6,7,8,9,10,1000000};
        for( size_t i = 0; i <  sizeof(x)/sizeof(x[0]); i ++ ) {
            x[i] = aa_ang_deg2rad(x[i]);
        }
        double s,m,m0,s0, ms;
        aa_stat_excluded_circ_mean_std( sizeof(x)/sizeof(x[0]), x,
                                        &m, &s,
                                        -2, 2,
                                        10 );
        m0 = aa_stat_circ_mean( sizeof(x)/sizeof(x[0]) - 1, x );
        s0 = aa_stat_circ_std( sizeof(x)/sizeof(x[0]) - 1, x );
        ms = aa_stat_mean( sizeof(x)/sizeof(x[0]) - 1, x );
        afeq( s, s0, .001 );
        afeq( m, m0, .001 );
        afeq( m, ms, .001 );
    }
    {
        double X[] = {1,2,3,
                      2.1,3,4,
                      3,4.1,5,
                      4,4.9,6.2};
        double mu[3], E[3*3];
        double mu_r[] = {2.5250,   3.5000,   4.5500};
        double E_r[] = {1.6358,  1.6167,  1.7483,
                        1.6167,  1.6067,  1.7267,
                        1.7483,  1.7267,  1.8767};
        aa_stat_vmean(3, 4, X, mu );
        aveq( "stat_vmean", 3, mu, mu_r, .001);
        aa_stat_vmean_cov(3, 4, X, mu, E );
        aveq( "stat_vmean_cov", 3, mu, mu_r, .001);
        aveq( "stat_vmean_cov", 3*3, E, E_r, .001);

        aa_la_inv(3, E);
        afeq( 24.75, aa_stat_mahalanobis( 3, (double[]){0,0,0}, mu, E ),
              .001 );
    }
}


void ang() {
    afeq( 0, aa_stat_circ_mean(3, (double[]){M_PI/2, 0, 3*M_PI/2}), .001 );
    afeq( M_PI, aa_stat_circ_mean(3, (double[]){M_PI/2, M_PI, 3*M_PI/2}), .001 );
    afeq( M_PI,
          aa_stat_circ_mean(3, (double[]){M_PI/2, M_PI, 3*M_PI/2 + 2*M_PI}),
          .001 );
}

void plane() {
    double P[] = {0,2,
                  1,0};
    double pl[3];
    double pl_r[3] = {-2, -1, 2};
    aa_la_plane_fit( 2, 2, P, pl );
    aa_la_plane_hessian( 3, pl_r);
    aveq( "la_plane_fit", 3, pl, pl_r, .001);

    double P2[] = {0,2,
                   .5,1,
                   1,0};
    aa_la_plane_fit( 2, 3, P2, pl );
    aveq( "la_plane_fit", 3, pl, pl_r, .001);
}

void misc() {
    int a = 1;
    int b = 2;
    AA_SWAP(a,b);
    assert( 1 == b );
    assert( 2 == a );

    // modulo
    afeq( aa_fremainder(90, 360), 90, 0 );
    afeq( aa_fremainder(180, 360), 180, 0 );
    afeq( aa_fremainder(270, 360), 270, 0 );
    afeq( aa_fremainder(90 + 360, 360), 90, 0 );
    afeq( aa_fremainder(180 + 360, 360), 180, 0 );
    afeq( aa_fremainder(270 + 360, 360), 270, 0 );
    afeq( aa_fremainder(90 - 360, 360), -270, 0 );
    afeq( aa_fremainder(180 - 360, 360), -180, 0 );
    afeq( aa_fremainder(270 - 360, 360), -90, 0 );

    afeq( aa_fmodulo(90, 360), 90, 0 );
    afeq( aa_fmodulo(180, 360), 180, 0 );
    afeq( aa_fmodulo(270, 360), 270, 0 );
    afeq( aa_fmodulo(90 + 360, 360), 90, 0 );
    afeq( aa_fmodulo(180 + 360, 360), 180, 0 );
    afeq( aa_fmodulo(270 + 360, 360), 270, 0 );
    afeq( aa_fmodulo(90 - 360, 360), 90, 0 );
    afeq( aa_fmodulo(180 - 360, 360), 180, 0 );
    afeq( aa_fmodulo(270 - 360, 360), 270, 0 );
}


void list() {
    int front[] = {2,   5,  2, 1, 0};
    int back[] =  {80, 50, 20, 4, 26};
    aa_mem_region_t reg;
    aa_mem_region_init(&reg, 2048);

    assert(sizeof(front) == sizeof(back));

    /* ptr */
    {
        aa_mem_rlist_t *lf = aa_mem_rlist_alloc( &reg );
        aa_mem_rlist_t *lb = aa_mem_rlist_alloc( &reg );
        for( size_t i = 0; i < sizeof(front)/sizeof(front[0]); i ++ ) {
            aa_mem_rlist_push_ptr( lf, front+i );
            aa_mem_rlist_enqueue_ptr( lf, back+i );

            aa_mem_rlist_enqueue_ptr( lb, back+i );
            aa_mem_rlist_push_ptr( lb, front+i );
        }

        for( size_t i = 0; i < sizeof(front)/sizeof(front[0]); i ++ ) {
            size_t j = sizeof(front)/sizeof(front[0]) - i - 1;
            int *f = (int*)aa_mem_rlist_pop(lf);
            int *b = (int*)aa_mem_rlist_pop(lb);
            assert( f == front + j );
            assert( b == front + j );
        }
        for( size_t i = 0; i < sizeof(front)/sizeof(front[0]); i ++ ) {
            int *f = (int*)aa_mem_rlist_pop(lf);
            int *b = (int*)aa_mem_rlist_pop(lb);
            assert( f == back + i );
            assert( b == back + i );
        }
    }

    aa_mem_region_release( &reg );

    /* copy */
    {
        aa_mem_rlist_t *lf = aa_mem_rlist_alloc( &reg );
        aa_mem_rlist_t *lb = aa_mem_rlist_alloc( &reg );
        for( size_t i = 0; i < sizeof(front)/sizeof(front[0]); i ++ ) {
            aa_mem_rlist_push_cpy( lf, front+i, sizeof(front[i]) );
            aa_mem_rlist_enqueue_cpy( lf, back+i, sizeof(back[i]) );

            aa_mem_rlist_enqueue_cpy( lb, back+i, sizeof(back[i]) );
            aa_mem_rlist_push_cpy( lb, front+i, sizeof(front[i]) );
        }

        for( size_t i = 0; i < sizeof(front)/sizeof(front[0]); i ++ ) {
            size_t j = sizeof(front)/sizeof(front[0]) - i - 1;
            int *f = (int*)aa_mem_rlist_pop(lf);
            int *b = (int*)aa_mem_rlist_pop(lb);
            assert( *f == front[j] );
            assert( *b == front[j] );
        }
        for( size_t i = 0; i < sizeof(front)/sizeof(front[0]); i ++ ) {
            int *f = (int*)aa_mem_rlist_pop(lf);
            int *b = (int*)aa_mem_rlist_pop(lb);
            assert( *f == back[i] );
            assert( *b == back[i] );
        }
    }
}


static void array(void)
{

    double x[] = {3,1,5,2,4,6};
    double m6 = aa_la_d_median( 6, x, 1);
    double m5 = aa_la_d_median( 5, x, 1);
    assert( aa_feq(m6, 3.5, 0) );
    assert( aa_feq(m5, 3, 0) );
}

int main( int argc, char **argv ) {
    printf("Init aa_test\n");
    (void) argc; (void) argv;


    // init
    srand((unsigned int)time(NULL)); // might break in 2038
    // some limits because linux (and sometimes our software) sucks
    {
        int r;
        struct rlimit lim;
        // address space
        lim.rlim_cur = (1<<30);
        lim.rlim_max = (1<<30);
        r = setrlimit( RLIMIT_AS, &lim );
        assert(0 == r );
        // cpu time
        lim.rlim_cur = 60;
        lim.rlim_max = 60;
        r = setrlimit( RLIMIT_CPU, &lim );
        assert(0 == r );
        // drop a core
        r = getrlimit( RLIMIT_CORE, &lim );
        assert(0==r);
        lim.rlim_cur = 100*1<<20;
        r = setrlimit( RLIMIT_CORE, &lim );
        assert(0==r);

    }

    aa_mem_region_init(&g_region, 1024*64);
    printf("Running aa_test\n");

    misc();

    mem();
    scalar();
    la0();
    la1();

    la2();
    la3();

    la2_0();
    la2_hungarian();

    clapack();
    angle();
    quat();
    rotmat();
    axang();
    tf();
    tffuzz();
    tm();
    dbg();
    kin();
    endconv();
    validate();
    io();
    sigsys();
    vision();
    stat();
    ang();
    plane();
    array();

    aa_mem_region_destroy(&g_region);

    list();

    printf("Ending aa_test\n");
}
