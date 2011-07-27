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
AA_API void sgetri_( const int *n, float *A, const int *lda,
              const int *ipiv, float *work, const int *lwork, int *info );
/** Inverse of matrix using LU factorization by dgetrf.
    \sa sgetri_
*/
AA_API void dgetri_( const int *n, double *A, const int *lda,
              const int *ipiv, double *work, const int *lwork, int *info );


/** Compute an LU factorization.
    \param m number of rows of matrix A
    \param n number of columns of matrix A
    \param A matrix in column-major order, on exit the L and U factors
    \param lda leading dimesion of A, probably just rows in A
    \param ipiv of length min(m,n), on exit the pivot indices
    \param info on success: info==0
 */
AA_API void sgetrf_( const int *m, const int *n, float *A, const int *lda,
              int *ipiv, int *info );

/** Compute an LU factorization.
 */
AA_API void dgetrf_( const int *m, const int *n, double *A, const int *lda,
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
AA_API void dgesvd_( const char jobu[1], const char jobvt[1],
                      const int *m, const int *n,
                      double *A, const int *lda,
                      double *S, double *U,
                      const int *ldu, double *Vt, int *ldvt,
                      double *work, const int *lwork, int *info );

/** ILAENV is called from the LAPACK routines to choose problem-dependent
 *  parameters for the local environment.
 *
 *  \param ispec
 *          Specifies the parameter to be returned as the value of
 *          ILAENV.
 *          - = 1: the optimal blocksize; if this value is 1, an unblocked
 *               algorithm will give the best performance.
 *          - = 2: the minimum block size for which the block routine
 *               should be used; if the usable block size is less than
 *               this value, an unblocked routine should be used.
 *          - = 3: the crossover point (in a block routine, for N less
 *               than this value, an unblocked routine should be used)
 *          - = 4: the number of shifts, used in the nonsymmetric
 *               eigenvalue routines (DEPRECATED)
 *          - = 5: the minimum column dimension for blocking to be used;
 *               rectangular blocks must have dimension at least k by m,
 *               where k is given by ILAENV(2,...) and m by ILAENV(5,...)
 *          - = 6: the crossover point for the SVD (when reducing an m by n
 *               matrix to bidiagonal form, if max(m,n)/min(m,n) exceeds
 *               this value, a QR factorization is used first to reduce
 *               the matrix to a triangular form.)
 *          - = 7: the number of processors
 *          - = 8: the crossover point for the multishift QR method
 *               for nonsymmetric eigenvalue problems (DEPRECATED)
 *          - = 9: maximum size of the subproblems at the bottom of the
 *               computation tree in the divide-and-conquer algorithm
 *               (used by xGELSD and xGESDD)
 *          - =10: ieee NaN arithmetic can be trusted not to trap
 *          - =11: infinity arithmetic can be trusted not to trap
 *          - 12 <= ISPEC <= 16:
 *               xHSEQR or one of its subroutines,
 *               see IPARMQ for detailed explanation
 *
 *  \param name
 *          The name of the calling subroutine, in either upper case or
 *          lower case.
 *
 *  \param opts
 *          The character options to the subroutine NAME, concatenated
 *          into a single character string.  For example, UPLO = 'U',
 *          TRANS = 'T', and DIAG = 'N' for a triangular routine would
 *          be specified as OPTS = 'UTN'.
 *
 *  \param n1      (input) INTEGER
 *  \param n2      (input) INTEGER
 *  \param n3      (input) INTEGER
 *  \param n4      (input) INTEGER
 *          Problem dimensions for the subroutine NAME; these may not all
 *          be required.
 * \param name_length fortran string brain damage, length of name
 * \param opts_length fortran string brain damage, length of opts
 */
AA_API int ilaenv_( const int *ispec, const char *name, const char *opts,
                      const int *n1, const int * n2, const int *n3, const int *n4,
                      int name_length, int opts_length );


/** DGELSD computes the minimum-norm solution to a real linear least
 *  squares problem.
 *
 *  Minimizes \f$| b - A*x |\f$ using the singular value decomposition
 *  (SVD) of A. A is an M-by-N matrix which may be rank-deficient.
 *
 *  Several right hand side vectors b and solution vectors x can be
 *  handled in a single call; they are stored as the columns of the
 *  M-by-NRHS right hand side matrix B and the N-by-NRHS solution
 *  matrix X.
 *
 *  The problem is solved in three steps:
 *  - (1) Reduce the coefficient matrix A to bidiagonal form with
 *      Householder transformations, reducing the original problem
 *      into a "bidiagonal least squares problem" (BLS)
 *  - (2) Solve the BLS using a divide and conquer approach.
 *  - (3) Apply back all the Householder tranformations to solve
 *      the original least squares problem.
 *
 *  The effective rank of A is determined by treating as zero those
 *  singular values which are less than RCOND times the largest singular
 *  value.
 *
 *  The divide and conquer algorithm makes very mild assumptions about
 *  floating point arithmetic. It will work on machines with a guard
 *  digit in add/subtract, or on those binary machines without guard
 *  digits which subtract like the Cray X-MP, Cray Y-MP, Cray C-90, or
 *  Cray-2. It could conceivably fail on hexadecimal or decimal machines
 *  without guard digits, but we know of none.
 *
 *
 *  \param M       (input) INTEGER
 *          The number of rows of A. M >= 0.
 *
 *  \param N       (input) INTEGER
 *          The number of columns of A. N >= 0.
 *
 *  \param NRHS    (input) INTEGER
 *          The number of right hand sides, i.e., the number of columns
 *          of the matrices B and X. NRHS >= 0.
 *
 *  \param A       (input) DOUBLE PRECISION array, dimension (LDA,N)
 *          On entry, the M-by-N matrix A.
 *          On exit, A has been destroyed.
 *
 *  \param LDA     (input) INTEGER
 *          The leading dimension of the array A.  LDA >= max(1,M).
 *
 *  \param B       (input/output) DOUBLE PRECISION array, dimension (LDB,NRHS)
 *          On entry, the M-by-NRHS right hand side matrix B.
 *          On exit, B is overwritten by the N-by-NRHS solution
 *          matrix X.  If m >= n and RANK = n, the residual
 *          sum-of-squares for the solution in the i-th column is given
 *          by the sum of squares of elements n+1:m in that column.
 *
 *  \param LDB     (input) INTEGER
 *          The leading dimension of the array B. LDB >= max(1,max(M,N)).
 *
 *  \param S       (output) DOUBLE PRECISION array, dimension (min(M,N))
 *          The singular values of A in decreasing order.
 *          The condition number of A in the 2-norm = S(1)/S(min(m,n)).
 *
 *  \param RCOND   (input) DOUBLE PRECISION
 *          RCOND is used to determine the effective rank of A.
 *          Singular values S(i) <= RCOND*S(1) are treated as zero.
 *          If RCOND < 0, machine precision is used instead.
 *
 *  \param RANK    (output) INTEGER
 *          The effective rank of A, i.e., the number of singular values
 *          which are greater than RCOND*S(1).
 *
 *  \param WORK    (workspace/output) DOUBLE PRECISION array, dimension (MAX(1,LWORK))
 *          On exit, if INFO = 0, WORK(1) returns the optimal LWORK.
 *
 *  \param LWORK   (input) INTEGER
 *          The dimension of the array WORK. LWORK must be at least 1.
 *          The exact minimum amount of workspace needed depends on M,
 *          N and NRHS. As long as LWORK is at least
 *              12*N + 2*N*SMLSIZ + 8*N*NLVL + N*NRHS + (SMLSIZ+1)**2,
 *          if M is greater than or equal to N or
 *              12*M + 2*M*SMLSIZ + 8*M*NLVL + M*NRHS + (SMLSIZ+1)**2,
 *          if M is less than N, the code will execute correctly.
 *          SMLSIZ is returned by ILAENV and is equal to the maximum
 *          size of the subproblems at the bottom of the computation
 *          tree (usually about 25), and
 *             NLVL = MAX( 0, INT( LOG_2( MIN( M,N )/(SMLSIZ+1) ) ) + 1 )
 *          For good performance, LWORK should generally be larger.
 *          If LWORK = -1, then a workspace query is assumed; the routine
 *          only calculates the optimal size of the WORK array, returns
 *          this value as the first entry of the WORK array, and no error
 *          message related to LWORK is issued by XERBLA.
 *
 *  \param IWORK   (workspace) INTEGER array, dimension (MAX(1,LIWORK))
 *          LIWORK >= max(1, 3 * MINMN * NLVL + 11 * MINMN),
 *          where MINMN = MIN( M,N ).
 *          On exit, if INFO = 0, IWORK(1) returns the minimum LIWORK.
 *
 *  \param INFO    (output) INTEGER
 *          = 0:  successful exit
 *          < 0:  if INFO = -i, the i-th argument had an illegal value.
 *          > 0:  the algorithm for computing the SVD failed to converge;
 *                if INFO = i, i off-diagonal elements of an intermediate
 *                bidiagonal form did not converge to zero.
 */
 AA_API void dgelsd_( const int *M, const int *N, const int *NRHS,
                        double *A, const int *LDA, double *B, const int *LDB,
                        double *S, const double *RCOND, int *RANK,
                        double *WORK, int *LWORK, int *IWORK, int *INFO );

/**  DGEBAL balances a general real matrix A.  This involves, first,
*    permuting A by a similarity transformation to isolate eigenvalues
*    in the first 1 to ILO-1 and last IHI+1 to N elements on the
*    diagonal; and second, applying a diagonal similarity
*    transformation to rows and columns ILO to IHI to make the rows
*    and columns as close in norm as possible.  Both steps are
*    optional.
*
*  Balancing may reduce the 1-norm of the matrix, and improve the
*  accuracy of the computed eigenvalues and/or eigenvectors.
*
*
*  \param JOB     (input) CHARACTER*1
*          Specifies the operations to be performed on A:
*          = 'N':  none:  simply set ILO = 1, IHI = N, SCALE(I) = 1.0
*                  for i = 1,...,N;
*          = 'P':  permute only;
*          = 'S':  scale only;
*          = 'B':  both permute and scale.
*
*  \param N       (input) INTEGER
*          The order of the matrix A.  N >= 0.
*
*  \param A       (input/output) DOUBLE PRECISION array, dimension (LDA,N)
*          On entry, the input matrix A.
*          On exit,  A is overwritten by the balanced matrix.
*          If JOB = 'N', A is not referenced.
*          See Further Details.
*
*  \param LDA     (input) INTEGER
*          The leading dimension of the array A.  LDA >= max(1,N).
*
*  \param ILO     (output) INTEGER
*  \param IHI     (output) INTEGER
*          ILO and IHI are set to integers such that on exit
*          A(i,j) = 0 if i > j and j = 1,...,ILO-1 or I = IHI+1,...,N.
*          If JOB = 'N' or 'S', ILO = 1 and IHI = N.
*
*  \param SCALE   (output) DOUBLE PRECISION array, dimension (N)
*          Details of the permutations and scaling factors applied to
*          A.  If P(j) is the index of the row and column interchanged
*          with row and column j and D(j) is the scaling factor
*          applied to row and column j, then
*          SCALE(j) = P(j)    for j = 1,...,ILO-1
*                   = D(j)    for j = ILO,...,IHI
*                   = P(j)    for j = IHI+1,...,N.
*          The order in which the interchanges are made is N to IHI+1,
*          then 1 to ILO-1.
*
*  \param INFO    (output) INTEGER
*          = 0:  successful exit.
*          < 0:  if INFO = -i, the i-th argument had an illegal value.
*/
void dgebal_( const char JOB[1], int *N, double *A, const int *LDA,
              int *ILO, int *IHI, double *SCALE, int *INFO );


/**  DGEES computes for an N-by-N real nonsymmetric matrix A, the
*    eigenvalues, the real Schur form T, and, optionally, the matrix
*    of Schur vectors Z.  This gives the Schur factorization A =
*    Z*T*(Z**T).
*
*  Optionally, it also orders the eigenvalues on the diagonal of the
*  real Schur form so that selected eigenvalues are at the top left.
*  The leading columns of Z then form an orthonormal basis for the
*  invariant subspace corresponding to the selected eigenvalues.
*
*  A matrix is in real Schur form if it is upper quasi-triangular with
*  1-by-1 and 2-by-2 blocks. 2-by-2 blocks will be standardized in the
*  form
*          [  a  b  ]
*          [  c  a  ]
*
*  where b*c < 0. The eigenvalues of such a block are a +- sqrt(bc).
*
*
*  \param JOBVS   (input) CHARACTER*1
*          = 'N': Schur vectors are not computed;
*          = 'V': Schur vectors are computed.
*
*  \param SORT    (input) CHARACTER*1
*          Specifies whether or not to order the eigenvalues on the
*          diagonal of the Schur form.
*          = 'N': Eigenvalues are not ordered;
*          = 'S': Eigenvalues are ordered (see SELECT).
*
*  \param SELECT  (external procedure) LOGICAL FUNCTION of two DOUBLE PRECISION arguments
*          SELECT must be declared EXTERNAL in the calling subroutine.
*          If SORT = 'S', SELECT is used to select eigenvalues to sort
*          to the top left of the Schur form.
*          If SORT = 'N', SELECT is not referenced.
*          An eigenvalue WR(j)+sqrt(-1)*WI(j) is selected if
*          SELECT(WR(j),WI(j)) is true; i.e., if either one of a complex
*          conjugate pair of eigenvalues is selected, then both complex
*          eigenvalues are selected.
*          Note that a selected complex eigenvalue may no longer
*          satisfy SELECT(WR(j),WI(j)) = .TRUE. after ordering, since
*          ordering may change the value of complex eigenvalues
*          (especially if the eigenvalue is ill-conditioned); in this
*          case INFO is set to N+2 (see INFO below).
*
*  \param N       (input) INTEGER
*          The order of the matrix A. N >= 0.
*
*  \param A       (input/output) DOUBLE PRECISION array, dimension (LDA,N)
*          On entry, the N-by-N matrix A.
*          On exit, A has been overwritten by its real Schur form T.
*
*  \param LDA     (input) INTEGER
*          The leading dimension of the array A.  LDA >= max(1,N).
*
*  \param SDIM    (output) INTEGER
*          If SORT = 'N', SDIM = 0.
*          If SORT = 'S', SDIM = number of eigenvalues (after sorting)
*                         for which SELECT is true. (Complex conjugate
*                         pairs for which SELECT is true for either
*                         eigenvalue count as 2.)
*
*  \param WR      (output) DOUBLE PRECISION array, dimension (N)
*  \param WI      (output) DOUBLE PRECISION array, dimension (N)
*          WR and WI contain the real and imaginary parts,
*          respectively, of the computed eigenvalues in the same order
*          that they appear on the diagonal of the output Schur form T.
*          Complex conjugate pairs of eigenvalues will appear
*          consecutively with the eigenvalue having the positive
*          imaginary part first.
*
*  \param VS      (output) DOUBLE PRECISION array, dimension (LDVS,N)
*          If JOBVS = 'V', VS contains the orthogonal matrix Z of Schur
*          vectors.
*          If JOBVS = 'N', VS is not referenced.
*
*  \param LDVS    (input) INTEGER
*          The leading dimension of the array VS.  LDVS >= 1; if
*          JOBVS = 'V', LDVS >= N.
*
*  \param WORK    (workspace/output) DOUBLE PRECISION array, dimension (MAX(1,LWORK))
*          On exit, if INFO = 0, WORK(1) contains the optimal LWORK.
*
*  \param LWORK   (input) INTEGER
*          The dimension of the array WORK.  LWORK >= max(1,3*N).
*          For good performance, LWORK must generally be larger.
*
*          If LWORK = -1, then a workspace query is assumed; the routine
*          only calculates the optimal size of the WORK array, returns
*          this value as the first entry of the WORK array, and no error
*          message related to LWORK is issued by XERBLA.
*
*  \param BWORK   (workspace) LOGICAL array, dimension (N)
*          Not referenced if SORT = 'N'.
*
*  \param INFO    (output) INTEGER
*          = 0: successful exit
*          < 0: if INFO = -i, the i-th argument had an illegal value.
*          > 0: if INFO = i, and i is
*             <= N: the QR algorithm failed to compute all the
*                   eigenvalues; elements 1:ILO-1 and i+1:N of WR and WI
*                   contain those eigenvalues which have converged; if
*                   JOBVS = 'V', VS contains the matrix which reduces A
*                   to its partially converged Schur form.
*             = N+1: the eigenvalues could not be reordered because some
*                   eigenvalues were too close to separate (the problem
*                   is very ill-conditioned);
*             = N+2: after reordering, roundoff changed values of some
*                   complex eigenvalues so that leading eigenvalues in
*                   the Schur form no longer satisfy SELECT=.TRUE.  This
*                   could also be caused by underflow due to scaling.
*
*/

void dgees_( const char JOBVS[1], const char SORT[1],
             int (*SELECT)(const double*,const double*),
             const int *N, double *A, const int *LDA, int *SDIM, double *WR, double *WI,
             double *VS, const int *LDVS, double *WORK, int *LWORK, int *BWORK, int *INFO );
#endif
