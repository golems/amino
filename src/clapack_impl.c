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

int AA_CLAPACK_NAME(getrf)
( int m, int n, AA_LA_TYPE *A, int lda, int *ipiv ) {
    int info;
    AA_LAPACK_NAME( getrf)
        (&m, &n, A, &lda, ipiv, &info );
    return info;
}
int AA_CLAPACK_NAME(getri)
( int n, AA_LA_TYPE *A, int lda, int *ipiv, AA_LA_TYPE *work, int lwork ) {
    int info;
    AA_LAPACK_NAME( getri )
        ( &n, A, &lda, ipiv, work, &lwork, &info );
    return info;
}
int AA_CLAPACK_NAME(gelsd_smlsiz) () {
    return aa_clapack_ilaenv(9, AA_LAPACK_PREFIX_STR "GELSD", "", 0, 0, 0, 0 );
}
int AA_CLAPACK_NAME(gelsd_nlvl) ( int m, int n ) {
    int minmn = AA_MIN(m,n);
    int smlsiz = AA_CLAPACK_NAME(gelsd_smlsiz)();
    return (int)AA_MAX(0, 1 + log2( minmn / (1 + smlsiz)));
}
int AA_CLAPACK_NAME(gelsd_miniwork) ( int m, int n ) {
    int minmn = AA_MIN(m,n);
    int nlvl = AA_CLAPACK_NAME(gelsd_nlvl)(m,n);
    return AA_MAX(1,
                  3 * minmn * nlvl + 11 * minmn);
}
int AA_CLAPACK_NAME(gelsd)
( int m, int n, int nrhs,
  AA_LA_TYPE *A, int lda,
  AA_LA_TYPE *B, int ldb,
  AA_LA_TYPE *S, AA_LA_TYPE *rcond, int *rank,
  AA_LA_TYPE *work, int lwork, int *iwork ) {
    int info;
    AA_LAPACK_NAME( gelsd )
        ( &m, &n, &nrhs, A, &lda, B, &ldb,
          S, rcond, rank, work, &lwork, iwork, &info );
    return info;
}
void AA_CLAPACK_NAME(lacpy) ( char uplo, int m, int n,
                              AA_LA_TYPE *A, int lda,
                              AA_LA_TYPE *B, int ldb ) {
    AA_LAPACK_NAME(lacpy) (&uplo, &m, &n,
                           A, &lda, B, &ldb );
}

#undef AA_LA_TYPE
#undef AA_CLAPACK_NAME
#undef AA_LAPACK_NAME
#undef AA_CBLAS_NAME
#undef AA_LAPACK_PREFIX_STR
