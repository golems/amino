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

#ifndef AA_AMINO_ARCH_GCC_H
#define AA_AMINO_ARCH_GCC_H

typedef double aa_vec_2d_t __attribute__ ((vector_size (16), aligned(16)));
typedef double aa_vec_4d_t __attribute__ ((vector_size (32), aligned(32)));
typedef int64_t aa_vec_4d_size_t __attribute__ ((vector_size (32)));
typedef int64_t aa_vec_2d_size_t __attribute__ ((vector_size (16)));



static inline aa_vec_4d_t
aa_vec_4d_shuffle( aa_vec_4d_t a,
                   int64_t i0, int64_t i1, int64_t i2, int64_t i3  ) {
    aa_vec_4d_size_t m = {i0,i1,i2,i3};
    return __builtin_shuffle(a,m);
}

static inline aa_vec_2d_t
aa_vec_2d_shuffle( aa_vec_2d_t a,
                   int64_t i0, int64_t i1  ) {
    aa_vec_2d_size_t m = {i0,i1};
    return __builtin_shuffle(a,m);
}


static inline aa_vec_2d_t
aa_vec_2d_swap( aa_vec_2d_t a ) {
    return aa_vec_2d_shuffle(a, 1, 0 );
}

static inline aa_vec_4d_t
aa_vec_4d_shuffle2( aa_vec_4d_t a, aa_vec_4d_t b,
                    int64_t i0, int64_t i1, int64_t i2, int64_t i3  ) {
    aa_vec_4d_size_t m = {i0,i1,i2,i3};
    return __builtin_shuffle(a,b,m);
}


/** Load a vec4 from memory */
static inline aa_vec_4d_t
aa_vec_4d_ld( const double src[4] ) {
    return *(aa_vec_4d_t*)src;
}

/** Store a vec4 to memory */
static inline void
aa_vec_4d_st( double dst[4], const aa_vec_4d_t src ) {
    *(aa_vec_4d_t*)dst = src;
}


/** Load a vec2 from memory */
static inline aa_vec_2d_t
aa_vec_2d_ld( const double src[2] ) {
    return *(aa_vec_2d_t*)src;
}

/** Store a vec2 to memory */
static inline void
aa_vec_2d_st( double dst[2], const aa_vec_2d_t src ) {
    *(aa_vec_2d_t*)dst = src;
}

#ifdef __AVX__

#include "amino/arch/avx.h"

#else // generic load/store


/** Load a vec3 from memory */
static inline aa_vec_4d_t
aa_vec_3d_ld( const double src[3] ) {
    aa_vec_4d_t dst = {src[0], src[1], src[2] };
    return dst;
}

/** Store a vec3 to memory */
static inline void
aa_vec_3d_st( double dst[3], const aa_vec_4d_t src ) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

#endif



/** 4D dot product */
double
aa_vec_4d_dot( const aa_vec_4d_t a, const aa_vec_4d_t b ) {
    aa_vec_4d_t sq = a*b;
    aa_vec_2d_t y = {sq[2], sq[3]};
    aa_vec_2d_t x = {sq[0], sq[1]};
    aa_vec_2d_t z = x+y;
    return z[0] + z[1];
}

/** Load a tf matrix from memory */
#define AA_VEC_TFMAT_LD( col0, col1, col2, col3, T ){   \
        col0[0] = T[0];                                 \
        col0[1] = T[1];                                 \
        col0[2] = T[2];                                 \
        col1[0] = T[3];                                 \
        col1[1] = T[4];                                 \
        col1[2] = T[5];                                 \
        col2[0] = T[6];                                 \
        col2[1] = T[7];                                 \
        col2[2] = T[8];                                 \
        col3[0] = T[9];                                 \
        col3[1] = T[10];                                \
        col3[2] = T[11];                                \
    }

/** Store a tf matrix to memory */
static inline void
aa_vec_tfmat_st( double T[AA_RESTRICT 12],
                 aa_vec_4d_t col0, aa_vec_4d_t col1, aa_vec_4d_t col2, aa_vec_4d_t col3 ) {
    T[0]  = col0[0];
    T[1]  = col0[1];
    T[2]  = col0[2];
    T[3]  = col1[0];
    T[4]  = col1[1];
    T[5]  = col1[2];
    T[6]  = col2[0];
    T[7]  = col2[1];
    T[8]  = col2[2];
    T[9]  = col3[0];
    T[10] = col3[1];
    T[11] = col3[2];
}

/** Vectorized cross product */
static inline aa_vec_4d_t
aa_vec_cross( const aa_vec_4d_t a, const aa_vec_4d_t b ) {
    return ( aa_vec_4d_shuffle( a, 1,2,0,3 ) * aa_vec_4d_shuffle( b, 2,0,1,3 ) -
             aa_vec_4d_shuffle( a, 2,0,1,3 ) * aa_vec_4d_shuffle( b, 1,2,0,3 ) );
}

void aa_vecm_cross( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3],
                    double c[AA_RESTRICT 3] );

/** Vectorized quaternion conjugate */
static inline aa_vec_4d_t
aa_vec_qconj( const aa_vec_4d_t q )  {
    aa_vec_4d_t c = -q;
    c[3] *= -1;
    return c;
}


/** Vectorized quaternion multiplication */
static inline aa_vec_4d_t
aa_vec_qmul( const aa_vec_4d_t a, const aa_vec_4d_t b ) {
    aa_vec_4d_t vc;
    vc = ( aa_vec_4d_shuffle( a, 0,2,3,1 ) * aa_vec_4d_shuffle( b, 3,0,2,1) +
           aa_vec_4d_shuffle( a, 1,3,2,0 ) * aa_vec_4d_shuffle( b, 2,1,3,0) +
           aa_vec_4d_shuffle( a, 3,1,0,2 ) * aa_vec_4d_shuffle( b, 0,3,1,2) -
           aa_vec_4d_shuffle( a, 2,0,1,3 ) * aa_vec_4d_shuffle( b, 1,2,0,3) );

    vc[3] = -vc[3];

    return vc;
}

#define AA_VEC_QMUL_2DB( ax, ay, az, aw, bxy, bzw, rxy, rzw ) { \
    aa_vec_2d_t aa_vec_tmp;                                     \
    aa_vec_tmp = ax*bzw - az*bxy;                               \
    aa_vec_tmp[0] = -aa_vec_tmp[0];                             \
    rx_xy = ay*bzw + aw*bxy + aa_vec_2d_swap(aa_vec_tmp);       \
    aa_vec_tmp = ax*bxy + az*bzw;                               \
    aa_vec_tmp[0] = -aa_vec_tmp[0];                             \
    rx_wz = aw*bzw - ay*bxy + aa_vec_2d_swap(aa_vec_tmp);       \
    }

static inline aa_vec_4d_t
aa_vec_vqmul( const aa_vec_4d_t v, const aa_vec_4d_t q ) {
    aa_vec_4d_t t = aa_vec_4d_shuffle(v, 2,0,1,1);
    t[3] = -v[2];

    aa_vec_4d_t y;
    y =  aa_vec_4d_shuffle(v, 1,2,0,0) * aa_vec_4d_shuffle(q, 2,0,1,0);
    y += aa_vec_4d_shuffle(v, 0,1,2,1) * aa_vec_4d_shuffle(q, 3,3,3,2);
    y -= t * aa_vec_4d_shuffle(q, 1,2,0,2);
    y[3] = -y[3];
    return y;
}

/** Vectorized quaternion multiplication */
void aa_vecm_qmul( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4],
                   double c[AA_RESTRICT 4] );


/** Vectorized quaternion rotation */
static inline aa_vec_4d_t
aa_vec_qrot( const aa_vec_4d_t q, const aa_vec_4d_t v ) {
    aa_vec_4d_t a = ( aa_vec_4d_shuffle( v, 2,0,1,3 ) * q[3]  +
                      aa_vec_4d_shuffle( v, 1,2,0,3 ) * q     -
                      v * aa_vec_4d_shuffle( q, 1,2,0,3 ) );

    aa_vec_4d_t b = aa_vec_4d_shuffle( a, 2,0,1,3 );

    return 2 * ( aa_vec_4d_shuffle(q, 1,2,0,3)*a - aa_vec_4d_shuffle(q, 2,0,1,3)*b ) + v;
}


/** Vectorized quaternion rotate */
void aa_vecm_qrot( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                   double p[AA_RESTRICT 3] );

/** Vectorized tf matrix multiply */
#define AA_VEC_TFMUL( T0c0, T0c1, T0c2, T0c3, T1, Uc0, Uc1, Uc2, Uc3 ) { \
        Uc0 = T0c0*T1[0] + T0c1*T1[1]  + T0c2*T1[2];                    \
        Uc1 = T0c0*T1[3] + T0c1*T1[4]  + T0c2*T1[5];                    \
        Uc2 = T0c0*T1[6] + T0c1*T1[7]  + T0c2*T1[8];                    \
        Uc3 = T0c0*T1[9] + T0c1*T1[10] + T0c2*T1[11] + T0c3;            \
    }

/** Vectorized transformation matrix multiply */
void aa_vecm_tfmul( const double T0[AA_RESTRICT 12], const double T1[AA_RESTRICT 12],
                    double U[AA_RESTRICT 12] );



/** Vectorized dual quaternion multiply */
#define AA_VEC_DUQU_MUL( d0r, d0d, d1r, d1d, d2r, d2d ) {               \
        d2r = aa_vec_qmul( d0r, d1r );                                  \
        d2d = aa_vec_qmul( d0r, d1d ) + aa_vec_qmul( d0d, d1r );        \
    }

/** Vectorized dual quaternion multiply */
void aa_vecm_duqu_mul( const double d0[AA_RESTRICT 8], const double d1[AA_RESTRICT 8],
                       double d2[AA_RESTRICT 8] );

/** Extract dual quaternion translation */
static inline aa_vec_4d_t
aa_vec_duqu_trans( aa_vec_4d_t r, aa_vec_4d_t d ) {
    return 2 * aa_vec_qmul( d, aa_vec_qconj(r));
}

/** Compute dual quaternion dual part */
static inline aa_vec_4d_t
aa_vec_qv2duqu_dual( aa_vec_4d_t r, aa_vec_4d_t d ) {
    return aa_vec_vqmul( aa_vec_qconj(r), d ) / 2;
}

#endif //AA_AMINO_ARCH_GCC_H
