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

/** Transpose A into B.
 *
 * \param m rows of A, cols of B
 * \param n cols of A, rows of B
 * \param[in] A source matrix, m*n
 * \param[in] B destination matrix, n*m
 * \param lda leading dimension of A
 * \param ldb leading dimension of B
 */
AA_API void AA_NAME(la,transpose)
( size_t m, size_t n,
  const AA_TYPE *A, size_t lda,
  AA_TYPE *B, size_t ldb );

/** Sum-square differences of x and y.
 * \param n length of vectors
 * \param x first vector
 * \param y second vector
 * \param incx stepsize of x
 * \param incy stepsize of y
 *
 * \sa aa_la_d_ssd
*/
AA_FDEC(AA_TYPE, la, ssd,
        size_t n,
        const AA_TYPE *x, size_t incx,
        const AA_TYPE *y, size_t incy )

/** Linear interpolation of vectors
 *
 * \param n length of vectors
 * \param u interpolation parameter, at 0, vu=v1, and at 1, vu=v2
 * \param v1 initial vector
 * \param v2 final vector
 * \param inc1 stepsize of v1
 * \param inc2 stepsize of v2
 * \param incu stepsize of vu
*/
AA_API void AA_NAME(la,lerp)
( size_t n, AA_TYPE u,
  const AA_TYPE *v1, size_t inc1,
  const AA_TYPE *v2, size_t inc2,
  AA_TYPE *vu, size_t incu );

/** Compute cubic spline parameters */
AA_API void AA_NAME(la,3spline_param)
( size_t n, AA_TYPE tf,
  const AA_TYPE *x1, size_t incx1,
  const AA_TYPE *dx1, size_t incdx1,
  const AA_TYPE *x2, size_t inc2,
  const AA_TYPE *dx2, size_t incdx2,
  AA_TYPE *a2, AA_TYPE *a3 );

/** Compute cubic spline value */
AA_API void AA_NAME(la,3spline)
( size_t n, AA_TYPE tf,
  const AA_TYPE *x1, size_t incx1,
  const AA_TYPE *dx1, size_t incdx1,
  const AA_TYPE *a2, const AA_TYPE *a3,
  AA_TYPE *x, size_t incx,
  AA_TYPE *dx, size_t incdx,
  AA_TYPE *ddx, size_t incddx );

/** Compute quintic spline parameters */
AA_API void AA_NAME(la,5spline_param)
( size_t n, AA_TYPE tf,
  const AA_TYPE *x1, size_t incx1,
  const AA_TYPE *dx1, size_t incdx1,
  const AA_TYPE *ddx1, size_t incddx1,
  const AA_TYPE *x2, size_t inc2,
  const AA_TYPE *dx2, size_t incdx2,
  const AA_TYPE *ddx2, size_t incddx2,
  AA_TYPE *a3, AA_TYPE *a4, AA_TYPE *a5 );

/** Compute quintic spline value */
AA_API void AA_NAME(la,5spline)
( size_t n, AA_TYPE tf,
  const AA_TYPE *x1, size_t incx1,
  const AA_TYPE *dx1, size_t incdx1,
  const AA_TYPE *ddx1, size_t incddx1,
  const AA_TYPE *a3, const AA_TYPE *a4, const AA_TYPE *a5,
  AA_TYPE *x, size_t incx,
  AA_TYPE *dx, size_t incdx,
  AA_TYPE *ddx, size_t incddx );

/** Standard deviation of vector.
 * \param n length of d
 * \param x vector
 * \param incx increment of x
 * \param mu mean
 */
AA_FDEC(AA_TYPE, la, vecstd,
        size_t n,
        const AA_TYPE *x, size_t incx,
        AA_TYPE mu);

/** Mean of columns of A.
 * \param m rows of A
 * \param n cols of A
 * \param lda leading dimension of A
 * \param[in] A source matrix, m*n
 * \param[out] x destination vector for mean, length m
 *
 * \sa aa_la_d_colmean
 */
AA_FDEC(AA_TYPE, la, colmean,
        size_t m, size_t n,
        const AA_TYPE *A, size_t lda,
        AA_TYPE *x);

/* AA_API void AA_NAME(la,_cmean) */
/* ( size_t m, size_t n, */
/*   const AA_TYPE *A, size_t lda, */
/*   AA_TYPE *x ); */

/** Covariance of columns of A.
 * \param m rows of A
 * \param n cols of A
 * \param lda leading dimension of A
 * \param[in] A source matrix, m*n
 * \param[in] x mean of columns of A, length m
 * \param[out] E covariance of columns of A, m*m
 * \param lde leading dimension of E
 *
 * \sa aa_la_d_colcov
 */

AA_FDEC(AA_TYPE, la, colcov,
        size_t m, size_t n,
        const AA_TYPE *A, size_t lda,
        const AA_TYPE *x,
        AA_TYPE *E, size_t lde);

/* AA_API void AA_NAME(la,_ccov) */
/* ( size_t m, size_t n, */
/*   const AA_TYPE *A, size_t lda, */
/*   const AA_TYPE *x, */
/*   AA_TYPE *E, size_t lde ); */



/* Hungarian algorithm to solve min assignment problem
 * \param n rows and cols of A
 * \param A cost matrix, column major, destroyed on exit
 * \param[out] row_assign array of column assigned to row at index i,
 *             length m. Unmatched elements have the value of -1.
 * \param col_assign mapping from col to matched row
 * \param lda leading dimension of A
 * \param row_assign array of column assigned to row at index i
 * \param iwork array of size 3*n*n +2*n
 */
/* AA_API void AA_NAME(la,opt_hungarian) */
/* ( size_t n, AA_TYPE *A, size_t lda, */
/*   ssize_t *row_assign, */
/*   ssize_t *col_assign, */
/*   ssize_t *iwork); */


/* Compute minimum iwork size for hungarian algorithm
 *
 * \sa aa_la_d_opt_hungarian
 */
/* static inline size_t AA_NAME(la,opt_hungarian_iworksize) */
/* ( size_t n ) { */
/*     return 3*n*n + 2*n; */
/* } */

/* Hungarian algorithm for rectangular distance matrix by padding with zeros.
 *
 * \param m rows of A
 * \param n cols of A
 * \param A distance matrix for minimization problem
 * \param lda leading dimension of A
 * \param[out] row_assign array of column assigned to row at index i,
 *             length m. Unmatched elements have the value of -1.
 * \param[out] col_assign mapping from col to matched row
 * \param work work array of length max(m,n)**2
 * \param iwork integer work array of length (3*max(m,n)**2 + 4*max(m,n))
 */
/* AA_API void AA_NAME(la,opt_hungarian_pad) */
/* ( size_t m, size_t n, const AA_TYPE *A, size_t lda, */
/*   ssize_t *row_assign, */
/*   ssize_t *col_assign, */
/*   AA_TYPE *work, ssize_t *iwork); */

/* Compute minimum iwork size for padded hungarian algorithm
 *
 * \sa aa_la_d_opt_hungarian_pad
 */
/* static inline size_t AA_NAME(la,opt_hungarian_pad_iworksize) */
/* (size_t m,size_t n) { */
/*     size_t p = AA_MAX(m,n); */
/*     return AA_NAME(la,opt_hungarian_iworksize)(p) + 2*p; */
/* } */


/* Compute minimum work size for padded hungarian algorithm
 *
 * \sa aa_la_d_opt_hungarian_pad
 */
/* static inline size_t AA_NAME(la,opt_hungarian_pad_worksize) */
/* (size_t m,size_t n) { */
/*     size_t p = AA_MAX(m,n); */
/*     return p*p; */
/* } */


/* Converts max assignment to min assignment for Hungarian algorithm.
 * \param m rows of A
 * \param n cols of A
 * \param A cost matrix for max problem,
 *        converted to matrix for min proble on exit
 * \param lda leading dimension of A
 */
/* AA_API void AA_NAME(la,opt_hungarian_max2min) */
/* ( size_t m, size_t n, AA_TYPE *A, size_t lda ); */


/** Hungarian algorithm to solve min assignment problem
 * \param m rows of A
 * \param n cols of A
 * \param[in] A cost matrix, column major, destroyed on exit
 * \param lda leading dimension of A
 * \param[out] row_assign array of column assigned to row at index i,
 *             length m. Unmatched elements have the value of -1.
 * \param[out] col_assign mapping from col to matched row
 */
AA_FDEC( void, la, assign_hungarian,
         size_t m, size_t n,
         const AA_TYPE *A, size_t lda,
         ssize_t *row_assign, ssize_t *col_assign );

/** Converts max assignment to min assignment for Hungarian algorithm.
 * \param m rows of A
 * \param n cols of A
 * \param A cost matrix for max problem,
 *        converted to matrix for min proble on exit
 * \param lda leading dimension of A
 */

AA_FDEC( void, la, assign_hungarian_max2min,
         size_t m, size_t n,
         const AA_TYPE *A, size_t lda );


/** Minimum location of vector x */
static inline size_t AA_NAME(la,minloc)
( size_t n, AA_TYPE *x, size_t incx ) {
    size_t imin = 0;
    AA_TYPE xmin = *x;
    for( size_t i = 1; i < n; i ++ ) {
        if( x[i*incx] < xmin ) {
            imin = i;
            xmin = x[i*incx];
        }
    }
    return imin;
}

/** Minimum location of vector x */
static inline AA_TYPE AA_NAME(la,mat_max)
( size_t m, size_t n, AA_TYPE *A, size_t lda,
  size_t *pi, size_t *pj ) {
    size_t im = 0;
    size_t jm = 0;
    AA_TYPE xm = *A;
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
static inline size_t AA_NAME(la,maxloc)
( size_t n, AA_TYPE *x, size_t incx ) {
    size_t imax = 0;
    AA_TYPE xmax = *x;
    for( size_t i = 1; i < n; i ++ ) {
        if( x[i*incx] > xmax ) {
            imax = i;
            xmax = x[i*incx];
        }
    }
    return imax;
}

/** Angle between vectors.
 *
 * \param n length of vectors
 * \param x first vector
 * \param incx stepsize of x
 * \param y second vector
 * \param incy stepsize of y
 *
 * \sa aa_la_d_angle
 */
AA_FDEC(AA_TYPE, la, angle,
        size_t n, const AA_TYPE *x, size_t incx,
        const AA_TYPE *y, size_t incy)



/** Linear Least Squares.
 * \f[ b = Ax \f]
 * Solves for x.
 * \param m rows in A
 * \param n cols in A
 * \param p cols in b and x
 * \param A matrix
 * \param lda leading dimension of A
 * \param b offset matrix
 * \param ldb leading dimension of b
 * \param x solution matrix
 * \param ldx leading dimension of x
 */
AA_API void AA_NAME(la,lls)
( size_t m, size_t n, size_t p,
  const AA_TYPE *A, size_t lda,
  const AA_TYPE *b, size_t ldb,
  AA_TYPE *x, size_t ldx );


/** Singular Value Decomposition of A.
 *
 * \f[ A =  U \Sigma V^T \f]
 *
 * \param m rows
 * \param n columns
 * \param A \f$A \in \Re^m \times \Re^n\f$, column major
 * \param lda leading dimension of A
 * \param U \f$U \in \Re^m \times \Re^m\f$, column major.
 *    If null or ldu==0, U is returned in A.  U and Vt cannot both be null.
 * \param ldu leading dimension of U
 * \param S \f$S \in \Re^m min(m,n)\f$, singular values
 * \param Vt \f$V^T \in \Re^n \times \Re^n\f$, singular vectors. If
 *   null or ldvt==0, Vt is returned in A.  Vt and U cannot both be null.
 * \param ldvt leading dimension of Vt
 */
AA_API int AA_NAME(la,svd)
( size_t m, size_t n, const AA_TYPE *A, size_t lda,
  AA_TYPE *U, size_t ldu,
  AA_TYPE *S,
  AA_TYPE *Vt, size_t ldvt );


/** Cmpute median.
 *
 * @param n number of elements in x
 * @param x data array
 * @param incx increment amount for x
 */
AA_API AA_TYPE AA_NAME(la,median)
( size_t n, const AA_TYPE *x, size_t incx );

/** Destructive median computation.
 *
 * @param n number of elements in x
 * @param x data array
 * @param incx increment amount for x
 *
 * @post entries of x are undefined
 */
AA_API AA_TYPE AA_NAME(la,nmedian)
( size_t n, AA_TYPE *x );


/** Compute median and pop array from local memory region
 */
static inline AA_TYPE AA_NAME(la,nmedian_pop)
( size_t n, AA_TYPE *x )
{
    AA_TYPE u = AA_NAME(la,nmedian)(n, x);
    aa_mem_region_local_pop(x);
    return u;
}

/** Median Absolute Deviation
 *
 * @param n number of elements
 * @param u median
 * @param x data array
 * @param incx increment of x
 */
AA_TYPE AA_NAME(la,mad)
( size_t n, const AA_TYPE u, const AA_TYPE *x, size_t incx );

/** Median Absolute Deviation with Euclidean distance
 *
 * @param m rows of A, length of u
 * @param n cols of A
 * @param u median (increment of 1)
 * @param A data matrix
 * @param lda leading dimension of A
 *
 */
AA_TYPE AA_NAME(la,mad2)
( size_t m, size_t n, const AA_TYPE *u, const AA_TYPE *A, size_t lda );

/** Compute eigen values and vectors */
AA_API int AA_NAME(la,eev)
( size_t n, const AA_TYPE *A, size_t lda,
  AA_TYPE *wr,
  AA_TYPE *wi,
  AA_TYPE *Vl, size_t ldvl,
  AA_TYPE *Vr,  size_t ldvr );

/** Fit a least-squares hyperplane to columns of A.
 *
 * Normalizes data first.
 *
 * \param m rows of A, size of space
 * \param n cols of A, number of points
 * \param A data points
 * \param lda leading dimension of A
 * \param x output hyperplance in hessian normal form
 *
 * \sa aa_la_d_colfit
 */
AA_FDEC( void, la, colfit,
         size_t m, size_t n,
         const AA_TYPE *A, size_t lda, AA_TYPE *x );


/* Ordering / Sort comparison function */
int AA_NAME(la,compar)( const void *a, const void *b );

#include "amino/undef.h"
