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

#include "amino.h"
#include <assert.h>
#include <stdio.h>

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
}

void la0() {
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
    // unit
    {
        double x[] = {1,2,3};
        double r[] = {0.26726,   0.53452,   0.80178};
        aa_la_unit(3,x);
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
    // invert
    {
        double A[] = {1,2,3,4};
        double B[] = {-2,1,1.5,-.5};
        aa_la_inv( 2, A );
        aveq( 4, A, B, 0 );
    }

    //dls
    {
        double A[] = {1,2,3,4};
        double A_star[4];
        double R[] = {  -1.92649, 0.96746,  1.44818,  -0.47711 };
        aa_la_dls(2,2, .005, &A[0], &A_star[0]);
        aveq( 4, A_star, R, .0001 );
    }
    {
        double A[] = {1,2,3,4,5,6};
        double A_star[6];
        double R[] =  {-1.30832, -0.32652, 0.65528, 1.06359, 0.32795, -0.40769};
        aa_la_dls(2,3, .005, &A[0], &A_star[0]);
        aveq( 6, A_star, R, .0001 );
    }

}

void angle() {
    // conversion
    afeq( aa_an_rad2deg(3.1), 3.1*180.0/M_PI, 0 );
    afeq( aa_an_rad2deg(M_PI), 180, 0 );
    afeq( aa_an_deg2rad(30), 30*M_PI/180, 0 );
    afeq( aa_an_deg2rad(180), M_PI, 0 );

    // norming
    afeq( aa_an_norm_2pi( 3*M_PI ), M_PI, 0 );
    afeq( aa_an_norm_2pi( -M_PI/2 ), 3*M_PI/2, .001 );
    afeq( aa_an_norm_pi( 3*M_PI/2 ), -M_PI/2, 0 );
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

int main( int argc, char **argv ) {
    (void) argc; (void) argv;
    scalar();
    la0();
    la1();
    la2();
    quat();
    angle();
    tm();
}
