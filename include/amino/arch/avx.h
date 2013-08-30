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

#ifndef AA_AMINO_ARCH_AVX_H
#define AA_AMINO_ARCH_AVX_H

/* /\** Load a vec4 from memory *\/ */
/* static inline aa_vec_4d_t */
/* aa_vec_4d_ld( const double src[4] ) { */
/*     //const double *a = __builtin_assume_aligned(src, 32); */
/*     return *(aa_vec_4d_t*)src; */
/*     //aa_vec_4d_t dst = a; */
/*     /\* dst[0] = a[0]; *\/ */
/*     /\* dst[1] = a[1]; *\/ */
/*     /\* dst[2] = a[2]; *\/ */
/*     /\* dst[3] = a[3]; *\/ */
/*     //return dst; */

/*     //return __builtin_ia32_loadupd256(src); */
/* } */

/* /\** Store a vec4 to memory *\/ */
/* static inline void */
/* aa_vec_4d_st( double dst[4], const aa_vec_4d_t src ) { */
/*     __builtin_ia32_storeupd256( dst, src ); */
/* } */


/** Load a vec3 from memory */
static inline aa_vec_4d_t
aa_vec_3d_ld( const double src[3] ) {
    aa_vec_2d_t d2 = __builtin_ia32_loadupd(src);
    aa_vec_4d_t d4;
    d4[0] = d2[0];
    d4[1] = d2[1];
    d4[3] = src[3];
    return d4;
}

/** Store a vec3 to memory */
static inline void
aa_vec_3d_st( double dst[3], const aa_vec_4d_t src ) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

#endif //AA_AMINO_ARCH_AVX_H
