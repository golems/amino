/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

// uncomment to check that local allocs actually get freed
// #define AA_ALLOC_STACK_MAX 0

#include "amino.h"
#include "amino/arch/arch.h"

void aa_vecm_cross( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3],
                    double c[AA_RESTRICT 3] ) {
    aa_vec_4d_t va = aa_vec_3d_ld(a);
    aa_vec_4d_t vb = aa_vec_3d_ld(b);
    aa_vec_4d_t vc = aa_vec_cross(va,vb);
    aa_vec_3d_st(c,vc);
}

void aa_vecm_qconj( const double q[4], double r[4] )  {
    aa_vec_4d_t a =  aa_vec_4d_ld(q);
    aa_vec_4d_t b =  aa_vec_qconj(a);
    aa_vec_4d_st(r,b);
    /* aa_vec_4d_t s = {-1, -1, -1, 1}; */
    /* return q*s; */
}



void aa_vecm_qmul( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4],
                   double c[AA_RESTRICT 4] ) {
    aa_vec_4d_t va = aa_vec_4d_ld(a);
    aa_vec_4d_t vb = aa_vec_4d_ld(b);
    aa_vec_4d_t vc = aa_vec_qmul(va,vb);
    aa_vec_4d_st(c,vc);
}

void aa_vecm_qrot( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                   double p[AA_RESTRICT 3] ) {

    aa_vec_4d_t vq = aa_vec_4d_ld(q);
    aa_vec_4d_t vv = aa_vec_3d_ld(v);
    aa_vec_4d_t vp = aa_vec_qrot(vq,vv);
    aa_vec_3d_st(p,vp);
}

void aa_vecm_tfmul( const double T0[AA_RESTRICT 12], const double T1[AA_RESTRICT 12],
                    double U[AA_RESTRICT 12] ) {

    aa_vec_4d_t T0c0, T0c1, T0c2, T0c3;
    aa_vec_4d_t Uc0, Uc1, Uc2, Uc3;

    AA_VEC_TFMAT_LD( T0c0, T0c1, T0c2, T0c3, T0 );
    AA_VEC_TFMUL( T0c0, T0c1, T0c2, T0c3, T1, Uc0, Uc1, Uc2, Uc3 );
    aa_vec_tfmat_st( U, Uc0, Uc1, Uc2, Uc3 );

    /* U[0] =  T0[0]*T1[0] + T0[3]*T1[1] + T0[6]*T1[2]; */
    /* U[1] =  T0[1]*T1[0] + T0[4]*T1[1] + T0[7]*T1[2]; */
    /* U[2] =  T0[2]*T1[0] + T0[5]*T1[1] + T0[8]*T1[2]; */

    /* U[3] =  T0[0]*T1[3] + T0[3]*T1[4] + T0[6]*T1[5]; */
    /* U[4] =  T0[1]*T1[3] + T0[4]*T1[4] + T0[7]*T1[5]; */
    /* U[5] =  T0[2]*T1[3] + T0[5]*T1[4] + T0[8]*T1[5]; */

    /* U[6] =  T0[0]*T1[6] + T0[3]*T1[7] + T0[6]*T1[8]; */
    /* U[7] =  T0[1]*T1[6] + T0[4]*T1[7] + T0[7]*T1[8]; */
    /* U[8] =  T0[2]*T1[6] + T0[5]*T1[7] + T0[8]*T1[8]; */

    /* U[9]  = T0[0]*T1[9] + T0[3]*T1[10] + T0[6]*T1[11] + T0[9]; */
    /* U[10] = T0[1]*T1[9] + T0[4]*T1[10] + T0[7]*T1[11] + T0[10]; */
    /* U[11] = T0[2]*T1[9] + T0[5]*T1[10] + T0[8]*T1[11] + T0[11]; */

}


void aa_vecm_tfmul_2d( const double T0[AA_RESTRICT 12], const double T1[AA_RESTRICT 12],
                       double U[AA_RESTRICT 12] ) {
    aa_vec_2d_t c0, c1, c2, c3;
    aa_vec_2d_t r0, r1, r2, r3;
    c0 = aa_vec_2d_ld( T0 );
    c1 = aa_vec_2d_ld( T0 + 3 );
    c2 = aa_vec_2d_ld( T0 + 6 );
    c3 = aa_vec_2d_ld( T0 + 9 );

    r0 = c0*T1[0+0*3] + c1*T1[1+0*3] + c2*T1[2+0*3];
    r1 = c0*T1[0+1*3] + c1*T1[1+1*3] + c2*T1[2+1*3];
    r2 = c0*T1[0+2*3] + c1*T1[1+2*3] + c2*T1[2+2*3];
    r3 = c0*T1[0+3*3] + c1*T1[1+3*3] + c2*T1[2+3*3] + c3;

    aa_vec_2d_st( U,   r0 );
    aa_vec_2d_st( U+3, r1 );
    aa_vec_2d_st( U+6, r2 );
    aa_vec_2d_st( U+9, r3 );

    U[2+0*3] = T0[2+0*3]*T1[0+0*3] + T0[2+2]*T1[1+0*3] + T0[2+3]*T1[2+0*3];
    U[2+1*3] = T0[2+0*3]*T1[0+1*3] + T0[2+2]*T1[1+1*3] + T0[2+3]*T1[2+1*3];
    U[2+2*3] = T0[2+0*3]*T1[0+2*3] + T0[2+2]*T1[1+2*3] + T0[2+3]*T1[2+2*3];
    U[2+3*3] = T0[2+0*3]*T1[0+3*3] + T0[2+2]*T1[1+3*3] + T0[2+3]*T1[2+3*3] + T0[2+3*3];

}

/** Vectorized dual quaternion multiply */
void aa_vecm_duqu_mul( const double d0[AA_RESTRICT 8], const double d1[AA_RESTRICT 8],
                       double d2[AA_RESTRICT 8] ) {

    aa_vec_4d_t d0r, d0d, d1r, d1d, d2r, d2d;
    d0r = aa_vec_4d_ld( d0 );
    d0d = aa_vec_4d_ld( d0+4 );
    d1r = aa_vec_4d_ld( d1 );
    d1d = aa_vec_4d_ld( d1+4 );
    AA_VEC_DUQU_MUL( d0r, d0d, d1r, d1d, d2r, d2d );
    aa_vec_4d_st( d2, d2r );
    aa_vec_4d_st( d2+4, d2d );
}
