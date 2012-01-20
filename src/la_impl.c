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


void AA_LA_NAME(transpose) ( size_t m, size_t n,
                             const AA_LA_TYPE *A, size_t lda,
                             AA_LA_TYPE *B, size_t ldb ) {
    for( size_t i=0, ia=0; i < n; i++, ia+=lda ) {
        for( size_t j=0, ib=0; j < m; j++, ib+=ldb ) {
            B[ib+i] = A[ia+j];
        }
    }
}

void AA_LA_NAME(cmean)
( size_t m, size_t n,
  const AA_LA_TYPE *A, size_t lda,
  AA_LA_TYPE *x)
{
    memset( x, 0, sizeof(x[0])*m );
    for( size_t i=0, j=0; i < n; i++, j+=lda ) {
        AA_CBLAS_NAME(axpy)( (int)m, 1.0, A+j, (int)1,
                             x, 1 );
    }
    AA_CBLAS_NAME(scal)((int)m, ((AA_LA_TYPE)1.0)/(AA_LA_TYPE)n,
                        x, 1);
}

void AA_LA_NAME(ccov)
( size_t m, size_t n,
  const AA_LA_TYPE *A, size_t lda,
  const AA_LA_TYPE *x,
  AA_LA_TYPE *E, size_t lde ) {
    for( size_t j = 0; j < m*lde; j+=lde ) {
        memset( E+j, 0, sizeof(E[0])*m );
    }
    for( size_t j = 0; j < n*lda; j+=lda )
    {
        /* t := - mu + A_i */
        AA_LA_TYPE t[m];
        memcpy( t, A+j, sizeof(t[0])*m );
        AA_CBLAS_NAME(axpy) ((int)m, -1.0, x, (int)1,
                             t, 1 );
        /* E += t * t' */
        AA_CBLAS_NAME(syr)( CblasColMajor, CblasUpper,
                            (int)m, 1.0, t, 1,
                            E, (int)lde );
    }
    AA_CBLAS_NAME(scal) ( (int)(m*m),
                          (AA_LA_TYPE)1.0/(AA_LA_TYPE)(n-1),
                          E, 1 );
    /* fill lower half */
    for( size_t i = 0; i < m; i ++ ) {
        for( size_t j = i+1; j < m; j ++ ) {
            AA_MATREF(E, m, j, i) = AA_MATREF(E, m, i, j);
        }
    }
}

#undef AA_LA_NAME
#undef AA_LA_TYPE
#undef AA_CBLAS_NAME
