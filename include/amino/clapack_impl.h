/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011-2012, Georgia Tech Research Corporation
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

#include "amino/def.h"

/** Row Factor.
 *
 * \sa dgetrf_
 */
static inline int AA_CLA_NAME(getrf)
( int m, int n, AA_TYPE *A, int lda, int *ipiv ) {
    int info;
    AA_LAPACK_NAME( getrf)
        (&m, &n, A, &lda, ipiv, &info );
    return info;
}

/** Invert matrix.
 *
 * \sa dgetrf_
 */
static inline int AA_CLA_NAME(getri)
( int n, AA_TYPE *A, int lda, int *ipiv, AA_TYPE *work, int lwork ) {
    int info;
    AA_LAPACK_NAME( getri )
        ( &n, A, &lda, ipiv, work, &lwork, &info );
    return info;
}

/// part of worksize computation for xgelsd
static inline int AA_CLA_NAME(gelsd_smlsiz) () {
    return aa_cla_ilaenv( 9, AA_LAPACK_PREFIX_STR
                          "GELSD", "", 0, 0, 0, 0 );
}

/// part of worksize computation for xgelsd
static inline int AA_CLA_NAME(gelsd_nlvl)
( int m, int n ) {
    int minmn = AA_MIN(m,n);
    int smlsiz = AA_CLA_NAME(gelsd_smlsiz)();
    return (int)AA_MAX(0, 1 + log2( minmn / (1 + smlsiz)));
}

/// Minimum iwork for xgelsd
static inline int AA_CLA_NAME(gelsd_miniwork)
( int m, int n ) {
    int minmn = AA_MIN(m,n);
    int nlvl = AA_CLA_NAME(gelsd_nlvl)(m,n);
    return AA_MAX(1,
                  3 * minmn * nlvl + 11 * minmn);
}

/** Linear least squares.
 *
 * \sa dgelsd_
 */
static inline int AA_CLA_NAME(gelsd)
( int m, int n, int nrhs,
  AA_TYPE *A, int lda,
  AA_TYPE *B, int ldb,
  AA_TYPE *S, AA_TYPE *rcond, int *rank,
  AA_TYPE *work, int lwork, int *iwork ) {
    int info;
    AA_LAPACK_NAME( gelsd )
        ( &m, &n, &nrhs, A, &lda, B, &ldb,
          S, rcond, rank, work, &lwork, iwork, &info );
    return info;
}

/** Matrix copy.
 *
 * \sa dlacpy_
 */
static inline void AA_CLA_NAME(lacpy)
( char uplo, int m, int n,
  const AA_TYPE *A, int lda,
  AA_TYPE *B, int ldb ) {
    AA_LAPACK_NAME(lacpy) (&uplo, &m, &n,
                           A, &lda, B, &ldb );
}

/** Set values in a matrix.
 *
 * \sa dlaset_
 */
static inline void AA_CLA_NAME(laset)
( char UPLO, int M, int N,
  AA_TYPE ALPHA,
  AA_TYPE BETA,
  AA_TYPE *A, int LDA )
{
    AA_LAPACK_NAME(laset) (&UPLO, &M, &N,
                           &ALPHA, &BETA,
                           A, &LDA);
}

/** Norm-2
 *
 * \sa dlapy2_
 */
static inline AA_TYPE AA_CLA_NAME(lapy2)
( AA_TYPE x, AA_TYPE y )
{
    return AA_LAPACK_NAME(lapy2)(&x, &y);
}

/** Norm-2
 *
 * \sa dlapy3_
 */
static inline AA_TYPE AA_CLA_NAME(lapy3)
( AA_TYPE x, AA_TYPE y, AA_TYPE z )
{
    return AA_LAPACK_NAME(lapy3)(&x, &y, &z);
}

/** Uniform random vector
 *
 * \sa dlaruv_
 */
static inline void AA_CLA_NAME(laruv)
( int iseed[4], int n, AA_TYPE *X )
{
    AA_LAPACK_NAME(laruv) (iseed, &n, X);
}

/** Normal random vector
 *
 * \sa dlarnv_
 */
static inline void AA_CLA_NAME(larnv)
( int idist, int iseed[4], int n, AA_TYPE *X )
{
    AA_LAPACK_NAME(larnv) (&idist, iseed, &n, X);
}

/** Scale matrix.
 *
 * \sa dlascl_
 */
static inline int AA_CLA_NAME(lascl)
( char TYPE, int KL, int KU,
  AA_TYPE CFROM, AA_TYPE CTO,
  int M, int N, AA_TYPE *A, int LDA ) {
    int info;
    AA_LAPACK_NAME( lascl )
        ( &TYPE, &KL, &KU, &CFROM, &CTO,
          &M, &N, A, &LDA, &info );
    return info;
}

#if AA_TYPE == double
/** Convert double to single float
 *
 * \sa dlag2s_
 */
static inline int AA_CLA_NAME(lag2s)
( int M, int N,
  double *A, int LDA,
  float *SA, int LDSA ) {
    int info;
    dlag2s_( &M, &N, A, &LDA, SA, &LDSA, &info);
    return info;
}

#endif // AA_TYPE == double

#if AA_TYPE == float
/** Convert single to double float
 *
 * \sa dlag2d_
 */
static inline int AA_CLA_NAME(lag2d)
( int M, int N,
  float * SA, int LDSA,
  double *A, int LDA ) {
    int info;
    slag2d_( &M, &N, SA, &LDSA, A, &LDA, &info );
    return info;
}

#endif // AA_TYPE == float

#include "amino/undef.h"
