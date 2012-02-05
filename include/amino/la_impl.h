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




/** Transpose A into B.
 *
 * \param m rows of A, cols of B
 * \param n cols of A, rows of B
 * \param[in] A source matrix, m*n
 * \param[in] B destination matrix, n*m
 * \param lda leading dimension of A
 * \param ldb leading dimension of B
 */
AA_API void AA_LA_NAME(_transpose)
( size_t m, size_t n,
  const AA_LA_TYPE *A, size_t lda,
  AA_LA_TYPE *B, size_t ldb );

/** Mean of columns of A.
 * \param m rows of A
 * \param n cols of A
 * \param lda leading dimension of A
 * \param[in] A source matrix, m*n
 * \param[out] x destination vector for mean, length m
 */
AA_API void AA_LA_NAME(_cmean)
( size_t m, size_t n,
  const AA_LA_TYPE *A, size_t lda,
  AA_LA_TYPE *x );

/** Covariance of columns of A.
 * \param m rows of A
 * \param n cols of A
 * \param lda leading dimension of A
 * \param[in] A source matrix, m*n
 * \param[in] x mean of columns of A, length m
 * \param[out] E covariance of columns of A, m*m
 * \param lde leading dimension of E
 */
AA_API void AA_LA_NAME(_ccov)
( size_t m, size_t n,
  const AA_LA_TYPE *A, size_t lda,
  const AA_LA_TYPE *x,
  AA_LA_TYPE *E, size_t lde );



/** Hungarian algorithm to solve min assignment problem
 * \param n rows and cols of A
 * \param A cost matrix, column major, destroyed on exit
 * \param lda leading dimension of A
 * \param row_assign array of column assigned to row at index i
 * \param iwork array of size 3*n*n+n
 */
AA_API void AA_LA_NAME(_opt_hungarian)
( size_t n, AA_LA_TYPE *A, size_t lda,
  ssize_t *row_assign,
  ssize_t *iwork);

/** Hungarian algorithm for rectangular distance matrix by padding with zeros.
 *
 * \param m rows of A
 * \param n cols of A
 * \param A distance matrix for minimization problem
 * \param lda leading dimension of A
 * \param[out] row_assign array of column assigned to row at index i,
 *             length m. Unmatched elements have the value of -1.
 * \param work work array of length max(m,n)**2
 * \param iwork integer work array of length (3*max(m,n)**2 + 2*max(m,n))
 */
AA_API void AA_LA_NAME(_opt_hungarian_pad)
( size_t m, size_t n, const AA_LA_TYPE *A, size_t lda,
  ssize_t *row_assign,
  AA_LA_TYPE *work, ssize_t *iwork);



/** Converts max assignment to min assignment for Hungarian algorithm.
 * \param n rows and cols of A
 * \param A cost matrix for max problem,
 *        converted to matrix for min proble on exit
 * \param lda leading dimension of A
 */
AA_API void AA_LA_NAME(_opt_hungarian_max2min)
( size_t n, AA_LA_TYPE *A, size_t lda );

/** Minimum location of vector x */
static inline size_t AA_LA_NAME(_minloc)
( size_t n, AA_LA_TYPE *x, size_t incx ) {
    size_t imin = 0;
    AA_LA_TYPE xmin = *x;
    for( size_t i = 1; i < n; i ++ ) {
        if( x[i*incx] < xmin ) {
            imin = i;
            xmin = x[i*incx];
        }
    }
    return imin;
}

/** Minimum location of vector x */
static inline AA_LA_TYPE AA_LA_NAME(_mat_max)
( size_t m, size_t n, AA_LA_TYPE *A, size_t lda,
  size_t *pi, size_t *pj ) {
    size_t im = 0;
    size_t jm = 0;
    AA_LA_TYPE xm = *A;
    for( size_t i = 0; i < m; i++) {
        for( size_t j = 0; j < n; j++) {
            if( AA_MATREF(A, lda, i, j ) > xm ) {
                im = i;
                jm = j;
                xm =  AA_MATREF(A, lda, i, j );
            }
        }
    }
    if( pi ) *pi = im;
    if( pj ) *pj = jm;
    return xm;
}


/** Maximum location of vector x */
static inline size_t AA_LA_NAME(_maxloc)
( size_t n, AA_LA_TYPE *x, size_t incx ) {
    size_t imax = 0;
    AA_LA_TYPE xmax = *x;
    for( size_t i = 1; i < n; i ++ ) {
        if( x[i*incx] > xmax ) {
            imax = i;
            xmax = x[i*incx];
        }
    }
    return imax;
}


#undef AA_LA_NAME
#undef AA_LA_TYPE
