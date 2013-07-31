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
#ifndef AA_VEC_H
#define AA_VEC_H


typedef double aa_vec_d4_t __attribute__ ((vector_size (32)));
typedef int64_t aa_vec_d4_size_t __attribute__ ((vector_size (32)));


static inline aa_vec_d4_t
aa_vec_d4_shuffle( aa_vec_d4_t a,
                   int64_t i0, int64_t i1, int64_t i2, int64_t i3  ) {
    aa_vec_d4_size_t m = {i0,i1,i2,i3};
    return __builtin_shuffle(a,m);
}


/** Load a vec4 from memory */
static inline aa_vec_d4_t
aa_vec_d4_ld( const double src[4] ) {
    aa_vec_d4_t dst = {src[0], src[1], src[2], src[3]};
    return dst;
}

/** Store a vec4 to memory */
static inline void
aa_vec_d4_st( double dst[4], const aa_vec_d4_t src ) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
}

/** Load a vec3 from memory */
static inline aa_vec_d4_t
aa_vec_d3_ld( const double src[3] ) {
    aa_vec_d4_t dst = {src[0], src[1], src[2] };
    return dst;
}

/** Store a vec3 to memory */
static inline void
aa_vec_d3_st( double dst[3], const aa_vec_d4_t src ) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
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
                 aa_vec_d4_t col0, aa_vec_d4_t col1, aa_vec_d4_t col2, aa_vec_d4_t col3 ) {
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
static inline aa_vec_d4_t
aa_vec_cross( const aa_vec_d4_t a, const aa_vec_d4_t b ) {
    return ( aa_vec_d4_shuffle( a, 1,2,0,3 ) * aa_vec_d4_shuffle( b, 2,0,1,3 ) -
             aa_vec_d4_shuffle( a, 2,0,1,3 ) * aa_vec_d4_shuffle( b, 1,2,0,3 ) );
}

void aa_vecm_cross( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3],
                    double c[AA_RESTRICT 3] );

/** Vectorized quaternion multiplication */
static inline aa_vec_d4_t
aa_vec_qmul( const aa_vec_d4_t a, const aa_vec_d4_t b ) {
    aa_vec_d4_t vc;
    vc = ( aa_vec_d4_shuffle( a, 0,2,3,1 ) * aa_vec_d4_shuffle( b, 3,0,2,1) +
           aa_vec_d4_shuffle( a, 1,3,2,0 ) * aa_vec_d4_shuffle( b, 2,1,3,0) +
           aa_vec_d4_shuffle( a, 3,1,0,2 ) * aa_vec_d4_shuffle( b, 0,3,1,2) -
           aa_vec_d4_shuffle( a, 2,0,1,3 ) * aa_vec_d4_shuffle( b, 1,2,0,3) );

    vc[3] = -vc[3];

    return vc;
}

/** Vectorized quaternion multiplication */
void aa_vecm_qmul( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4],
                   double c[AA_RESTRICT 4] );


/** Vectorized quaternion rotation */
static inline aa_vec_d4_t
aa_vec_qrot( const aa_vec_d4_t q, const aa_vec_d4_t v ) {
    aa_vec_d4_t a = ( aa_vec_d4_shuffle( v, 2,0,1,3 ) * q[3]  +
                      aa_vec_d4_shuffle( v, 1,2,0,3 ) * q     -
                      v * aa_vec_d4_shuffle( q, 1,2,0,3 ) );

    aa_vec_d4_t b = aa_vec_d4_shuffle( a, 2,0,1,3 );

    return 2 * ( aa_vec_d4_shuffle(q, 1,2,0,3)*a - aa_vec_d4_shuffle(q, 2,0,1,3)*b ) + v;
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
        d2d =  aa_vec_qmul( d0r, d1d ) + aa_vec_qmul( d0d, d1r );       \
    }

/** Vectorized dual quaternion multiply */
void aa_vecm_duqu_mul( const double d0[AA_RESTRICT 8], const double d1[AA_RESTRICT 8],
                       double d2[AA_RESTRICT 8] );

#endif //AA_VEC_H
