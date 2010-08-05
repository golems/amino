/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/** \file lapack.h
 * \brief C prototypes to various fortran lapack routines.
 *
 * Since there is no official c binding to lapack as there as with
 * BLAS, the only reasonable way to interface with lapack from C is to
 * call the fortran methods directly.
 *
 * Authors:
 *   Neil T. Dantam
 */

#ifndef LAPACK_H_
#define LAPACK_H_

/** Inverse of matrix using LU factorization by *getrf.

    You must call *getrf before you call *getri.

    \param n Order of the matrix A
    \param A on entry the L and U factors from *getrf,
      on exit the inverse of the original A
    \param lda number of rows in A
    \param ipiv pivot indices from sgetrf
    \param work workspace array
    \param lwork length of work, optimally > n*nb where nb is the
      optimal blocksize return by ilaenv_
    \param info output.  info==0 for success, info<zero for illegal
      argument, info > 0 for singular matrix
 */
AA_CDECL void sgetri_( const int *n, float *A, const int *lda,
              const int *ipiv, float *work, const int *lwork, int *info );
/** Inverse of matrix using LU factorization by dgetrf.
    \sa sgetri_
*/
AA_CDECL void dgetri_( const int *n, double *A, const int *lda,
              const int *ipiv, double *work, const int *lwork, int *info );


/** Compute an LU factorization.
    \param m number of rows of matrix A
    \param n number of columns of matrix A
    \param A matrix in column-major order, on exit the L and U factors
    \param lda leading dimesion of A, probably just rows in A
    \param ipiv of length min(m,n), on exit the pivot indices
    \param info on success: info==0
 */
AA_CDECL void sgetrf_( const int *m, const int *n, float *A, const int *lda,
              int *ipiv, int *info );

/** Compute an LU factorization.
 */
AA_CDECL void dgetrf_( const int *m, const int *n, double *A, const int *lda,
              int *ipiv, int *info );


/** Compute SVD.
*
*
*  \param jobu    (input) CHARACTER*1
*          Specifies options for computing all or part of the matrix U:
*          - = 'A':  all M columns of U are returned in array U:
*          - = 'S':  the first min(m,n) columns of U (the left singular
*                  vectors) are returned in the array U;
*          - = 'O':  the first min(m,n) columns of U (the left singular
*                  vectors) are overwritten on the array A;
*          - = 'N':  no columns of U (no left singular vectors) are
*                  computed.
*
*  \param jobvt   (input) CHARACTER*1
*          Specifies options for computing all or part of the matrix
*          V**T:
*          - = 'A':  all N rows of V**T are returned in the array VT;
*          - = 'S':  the first min(m,n) rows of V**T (the right singular
*                  vectors) are returned in the array VT;
*          - = 'O':  the first min(m,n) rows of V**T (the right singular
*                  vectors) are overwritten on the array A;
*          - = 'N':  no rows of V**T (no right singular vectors) are
*                  computed.
*          JOBVT and JOBU cannot both be 'O'.
*
*  \param m  (input) INTEGER
*          The number of rows of the input matrix A.  M >= 0.
*
*  \param n       (input) INTEGER
*          The number of columns of the input matrix A.  N >= 0.
*
*  \param A       (input/output) DOUBLE PRECISION array, dimension (LDA,N)
*          On entry, the M-by-N matrix A.
*          On exit,
*          - if JOBU = 'O',  A is overwritten with the first min(m,n)
*                          columns of U (the left singular vectors,
*                          stored columnwise);
*          - if JOBVT = 'O', A is overwritten with the first min(m,n)
*                          rows of V**T (the right singular vectors,
*                          stored rowwise);
*          - if JOBU .ne. 'O' and JOBVT .ne. 'O', the contents of A
*                          are destroyed.
*
*  \param lda     (input) INTEGER
*          The leading dimension of the array A.  LDA >= max(1,M).
*
*  \param S       (output) DOUBLE PRECISION array, dimension (min(M,N))
*          The singular values of A, sorted so that S(i) >= S(i+1).
*
*  \param U       (output) DOUBLE PRECISION array, dimension (LDU,UCOL)
*          (LDU,M) if JOBU = 'A' or (LDU,min(M,N)) if JOBU = 'S'.
*          - If JOBU = 'A', U contains the M-by-M orthogonal matrix U;
*          - if JOBU = 'S', U contains the first min(m,n) columns of U
*          (the left singular vectors, stored columnwise);
*          - if JOBU = 'N' or 'O', U is not referenced.
*
*  \param ldu     (input) INTEGER
*          The leading dimension of the array U.  LDU >= 1; if
*          JOBU = 'S' or 'A', LDU >= M.
*  \param Vt      (output) DOUBLE PRECISION array, dimension (LDVT,N)
*         - If JOBVT = 'A', VT contains the N-by-N orthogonal matrix
*          V**T;
*         - if JOBVT = 'S', VT contains the first min(m,n) rows of
*          V**T (the right singular vectors, stored rowwise);
*         - if JOBVT = 'N' or 'O', VT is not referenced.
*
*  \param ldvt    (input) INTEGER
*          The leading dimension of the array VT.  LDVT >= 1; if
*          JOBVT = 'A', LDVT >= N; if JOBVT = 'S', LDVT >= min(M,N).
*
*  \param work    (workspace/output) DOUBLE PRECISION array, dimension (MAX(1,LWORK))
*          On exit, if INFO = 0, WORK(1) returns the optimal LWORK;
*          if INFO > 0, WORK(2:MIN(M,N)) contains the unconverged
*          superdiagonal elements of an upper bidiagonal matrix B
*          whose diagonal is in S (not necessarily sorted). B
*          satisfies A = U * B * VT, so it has the same singular values
*          as A, and singular vectors related by U and VT.
*
*  \param lwork (input) INTEGER The dimension of the array WORK.
*          LWORK >= MAX(1,3*MIN(M,N)+MAX(M,N),5*MIN(M,N)).  For good
*          performance, LWORK should generally be larger.
*          If LWORK = -1, then a workspace query is assumed; the
*          routine only calculates the optimal size of the WORK array,
*          returns this value as the first entry of the WORK array,
*          and no error message related to LWORK is issued by XERBLA.
*
*  \param info    (output) INTEGER
*          - = 0:  successful exit.
*          - < 0:  if INFO = -i, the i-th argument had an illegal value.
*          - > 0:  if DBDSQR did not converge, INFO specifies how many
*                superdiagonals of an intermediate bidiagonal form B
*                did not converge to zero. See the description of WORK
*                above for details.
*/
AA_CDECL void dgesvd_( const char jobu[1], const char jobvt[1],
                      const int *m, const int *n,
                      double *A, const int *lda,
                      double *S, double *U,
                      const int *ldu, double *Vt, int *ldvt,
                      double *work, const int *lwork, int *info );
#endif
