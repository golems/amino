/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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
#ifndef AA_CLAPACK_H
#define AA_CLAPACK_H

/// produce token for a lapack function
#define AA_LAPACK_NAME( name, prefix ) prefix ## name ## _


/// produce token for a lapack function
#define AA_CBLAS_NAME( name, prefix ) cblas_ ## prefix ## name

/// produce token for a lapack function
#define AA_CLAPACK_NAME( name, prefix ) aa_clapack_ ## prefix ## name


/// create lapack definitions for some type
#define AA_LAPACK_DEC( TYPE, prefix, PREFIX )                           \
    AA_API int AA_CLAPACK_NAME(getrf, prefix)                           \
    ( int m, int n, TYPE *A, int lda, int *ipiv );                      \
    AA_API int AA_CLAPACK_NAME(getri, prefix)                           \
    ( int n, TYPE *A, int lda, int *ipiv, TYPE *work, int lwork );      \
    AA_API int AA_CLAPACK_NAME(gelsd_smlsiz, prefix) (void);            \
    AA_API int AA_CLAPACK_NAME(gelsd_nlvl, prefix) ( int m, int n );    \
    AA_API int AA_CLAPACK_NAME(gelsd_miniwork, prefix) (int m, int n);  \
    AA_API int AA_CLAPACK_NAME(gelsd, prefix)                           \
    ( int m, int n, int nrhs,                                           \
      TYPE *A, int lda,                                                 \
      TYPE *B, int ldb,                                                 \
      TYPE *S, TYPE *rcond, int *rank,                                  \
      TYPE *work, int lwork, int *iwork ) ;                             \
    AA_API void AA_CLAPACK_NAME(lacpy, prefix)                          \
    ( char uplo, int m, int n,                                          \
      TYPE *A, int lda,                                                 \
      TYPE *B, int ldb );                                               \
    static inline TYPE AA_CLAPACK_NAME(lapy2, prefix)                   \
    (TYPE x, TYPE y) {                                                  \
        return AA_LAPACK_NAME(lapy2, prefix)(&x, &y);                   \
    }                                                                   \
    static inline TYPE AA_CLAPACK_NAME(lapy3, prefix)                   \
    (TYPE x, TYPE y, TYPE z) {                                          \
        return AA_LAPACK_NAME(lapy3, prefix)(&x, &y, &z);               \
    }

/// lapacks defs for double float
AA_LAPACK_DEC( double, d, D );
/// lapacks defs for single float
AA_LAPACK_DEC( float, s, S );


AA_API int aa_clapack_ilaenv( int ispec, const char *name, const char *opts,
                              int n1, int n2, int n3, int n4 );

#endif // AA_CLAPACK_H
