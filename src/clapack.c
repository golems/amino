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

#include <cblas.h>

#include <amino.h>

/// create lapack definitions for some type
#define AA_LAPACK_DEF( TYPE, prefix, PREFIX )                           \
    int AA_CLAPACK_NAME(getrf, prefix)                                  \
    ( int m, int n, TYPE *A, int lda, int *ipiv ) {                     \
        int info;                                                       \
        AA_LAPACK_NAME( getrf, prefix)                                  \
            (&m, &n, A, &lda, ipiv, &info );                            \
        return info;                                                    \
    }                                                                   \
    int AA_CLAPACK_NAME(getri, prefix)                                  \
    ( int n, TYPE *A, int lda, int *ipiv, TYPE *work, int lwork ) {     \
            int info;                                                   \
            AA_LAPACK_NAME( getri, prefix )                             \
                ( &n, A, &lda, ipiv, work, &lwork, &info );             \
            return info;                                                \
    }                                                                   \
    int AA_CLAPACK_NAME(gelsd_smlsiz, prefix) () {                      \
        return aa_clapack_ilaenv(9, #PREFIX "GELSD", "", 0, 0, 0, 0 );  \
        }                                                               \
    int AA_CLAPACK_NAME(gelsd_nlvl, prefix) ( int m, int n ) {          \
            int minmn = AA_MIN(m,n);                                    \
            int smlsiz = AA_CLAPACK_NAME(gelsd_smlsiz,prefix)();        \
                return (int)AA_MAX(0, 1 + log2( minmn / (1 + smlsiz))); \
    }                                                                   \
    int AA_CLAPACK_NAME(gelsd_miniwork, prefix) ( int m, int n ) {      \
            int minmn = AA_MIN(m,n);                                    \
            int nlvl = AA_CLAPACK_NAME(gelsd_nlvl,prefix)(m,n);         \
            return AA_MAX(1,                                            \
                          3 * minmn * nlvl + 11 * minmn);               \
    }                                                                   \
    int AA_CLAPACK_NAME(gelsd, prefix)                                  \
    ( int m, int n, int nrhs,                                           \
      TYPE *A, int lda,                                                 \
      TYPE *B, int ldb,                                                 \
      TYPE *S, TYPE *rcond, int *rank,                                  \
      TYPE *work, int lwork, int *iwork ) {                             \
        int info;                                                       \
        AA_LAPACK_NAME( gelsd, prefix )                                 \
            ( &m, &n, &nrhs, A, &lda, B, &ldb,                          \
              S, rcond, rank, work, &lwork, iwork, &info );             \
        return info;                                                    \
    }                                                                   \
    void AA_CLAPACK_NAME(lacpy, prefix) ( char uplo, int m, int n,      \
                                          TYPE *A, int lda,             \
                                          TYPE *B, int ldb ) {          \
        AA_LAPACK_NAME(lacpy, prefix) (&uplo, &m, &n,                   \
                                       A, &lda, B, &ldb );              \
    }                                                                   \

/// lapacks defs for double float
AA_LAPACK_DEF( double, d, D );
/// lapacks defs for single float
AA_LAPACK_DEF( float, s, S );

int aa_clapack_ilaenv( int ispec, const char *name, const char *opts,
               int n1, int n2, int n3, int n4 ) {
    int nl = (int)strlen(name);
    int ol = (int)strlen(opts);
    return ilaenv_(&ispec, name, opts, &n1, &n2, &n3, &n4, nl, ol );
}
