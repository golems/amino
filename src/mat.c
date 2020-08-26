/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@miens.edu>
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

// uncomment to check that local allocs actually get freed
// #define AA_ALLOC_STACK_MAX 0


#include <stdlib.h>
#include <cblas.h>

#include "amino.h"
#include "amino/mat.h"
#include "amino/mat_internal.h"

#define VEC_LEN(X) ((int)(X->len))
#define MAT_ROWS(X) ((int)(X->rows))
#define MAT_COLS(X) ((int)(X->cols))

/******************/
/* Error Handling */
/******************/

static void
s_err_default( const char *message )
{
    fprintf(stderr, "AMINO ERROR: %s\n",message);
    abort();
    exit(EXIT_FAILURE);
}

static aa_la_err_fun  *s_err_fun = s_err_default;

void
aa_la_set_err( aa_la_err_fun *fun )
{
    s_err_fun = fun;
}

AA_API void
aa_la_err( const char *message ) {
    s_err_fun(message);
}

AA_API void
aa_la_fail_size( size_t a, size_t b )
{
        const size_t size = 256;
        char buf[size];
        snprintf(buf,size, "Mismatched sizes: %lu != %lu\n", a, b);
        aa_la_err(buf);
}

/****************/
/* Construction */
/****************/

AA_API void
aa_dvec_view( struct aa_dvec *vec, size_t len, double *data, size_t inc )
{
    AA_DVEC_VIEW(vec, len, data, inc);
}


AA_API void
aa_dvec_slice( const struct aa_dvec *src,
               size_t start,
               size_t stop,
               size_t step,
               struct aa_dvec *dst )
{
    if( stop > src->len || stop < start ) {
        aa_la_err("Slice out-of-bounds\n");
    }
    *dst = AA_DVEC_INIT( (stop - start) / step,
                         src->data + src->inc*start,
                         src->inc*step );
}


AA_API void
aa_dmat_row_vec( const struct aa_dmat *src, size_t row, struct aa_dvec *dst )
{
    if( row >= src->rows ) {
        aa_la_err("Row vector out-of-bounds\n");
    }
    aa_dvec_view(dst,src->cols, src->data + row, src->ld);
}

AA_API void
aa_dmat_col_vec( const struct aa_dmat *src, size_t col, struct aa_dvec *dst )
{
    if( col >= src->cols ) {
        aa_la_err("Row vector out-of-bounds\n");
    }
    aa_dvec_view(dst, src->rows, src->data + col*src->ld, 1);
}

AA_API void
aa_dmat_diag_vec( const struct aa_dmat *src, struct aa_dvec *dst )
{
    aa_dvec_view(dst, AA_MIN(src->rows, src->cols), src->data, src->ld + 1);
}

AA_API void
aa_dmat_view( struct aa_dmat *mat, size_t rows, size_t cols,
              double *data, size_t ld )
{
    AA_DMAT_VIEW(mat, rows, cols, data, ld);
}

AA_API void
aa_dmat_view_block(  struct aa_dmat *dst,
                     const struct aa_dmat *src,
                     size_t row_start, size_t col_start,
                     size_t rows, size_t cols )
{
    aa_dmat_block( src,
                   row_start, col_start,
                   row_start + rows, col_start + cols,
                   dst );
}

AA_API void
aa_dmat_block( const struct aa_dmat *src,
               size_t row_start, size_t col_start,
               size_t row_end, size_t col_end,
               struct aa_dmat *dst )
{
    size_t m = src->rows, n = src->cols;
    if( row_start >= row_end ||
        row_start >= m ||
        row_end > m ||
        col_start >= col_end ||
        col_start >= n ||
        col_end > n )
    {
        aa_la_err("Block out-of-bounds\n");
    }

    aa_dmat_view( dst,
                  row_end - row_start, col_end - col_start,
                  src->data + col_start*src->ld + row_start, src->ld );
}


AA_API struct aa_dvec *
aa_dvec_malloc( size_t len ) {
    struct aa_dvec *r;
    const size_t s_desc = sizeof(*r);
    const size_t s_elem = sizeof(r->data[0]);
    const size_t pad =  s_elem - (s_desc % s_elem);
    const size_t size = s_desc + pad + len * s_elem;

    const size_t off = s_desc + pad;

    char *ptr = (char*)malloc( size );

    r = (struct aa_dvec*)ptr;
    aa_dvec_view(r, len, (double*)(ptr+off), 1);
    return r;
}

AA_API struct aa_dvec *
aa_dvec_alloc( struct aa_mem_region *reg, size_t len ) {
    struct aa_dvec *r;
    const size_t s_desc = sizeof(*r);
    const size_t s_elem = sizeof(r->data[0]);
    const size_t pad =  s_elem - (s_desc % s_elem);
    char *ptr = (char*)aa_mem_region_alloc(reg, s_desc + pad + len * s_elem );

    r = (struct aa_dvec*)ptr;
    aa_dvec_view(r, len, (double*)(ptr+s_desc+pad), 1);
    return r;
}

AA_API struct aa_dvec *
aa_dvec_dup( struct aa_mem_region *reg, const struct aa_dvec *src)
{
    struct aa_dvec *dst = aa_dvec_alloc(reg,src->len);
    aa_dvec_copy(src,dst);
    return dst;
}

AA_API struct aa_dvec *
aa_dvec_mdup( const struct aa_dvec *src)
{
    struct aa_dvec *dst = aa_dvec_malloc(src->len);
    aa_dvec_copy(src,dst);
    return dst;
}

AA_API struct aa_dmat *
aa_dmat_dup( struct aa_mem_region *reg, const struct aa_dmat *src)
{
    struct aa_dmat *dst = aa_dmat_alloc(reg,src->rows,src->cols);
    aa_dmat_copy(src,dst);
    return dst;
}


AA_API struct aa_dmat *
aa_dmat_mdup( const struct aa_dmat *src)
{
    struct aa_dmat *dst = aa_dmat_malloc(src->rows,src->cols);
    aa_dmat_copy(src,dst);
    return dst;
}

AA_API struct aa_dmat *
aa_dmat_malloc( size_t rows, size_t cols )
{
    struct aa_dmat *r;
    const size_t s_desc = sizeof(*r);
    const size_t s_elem = sizeof(r->data[0]);
    const size_t pad =  s_elem - (s_desc % s_elem);
    char *ptr = (char*)malloc( s_desc + pad + rows*cols * s_elem );

    r = (struct aa_dmat*)ptr;
    aa_dmat_view(r, rows, cols, (double*)(ptr+s_desc+pad), rows);

    return r;
}

AA_API struct aa_dmat *
aa_dmat_alloc( struct aa_mem_region *reg, size_t rows, size_t cols )
{
    struct aa_dmat *r;
    const size_t s_desc = sizeof(*r);
    const size_t s_elem = sizeof(r->data[0]);
    const size_t pad =  s_elem - (s_desc % s_elem);
    size_t size = s_desc + pad + rows*cols * s_elem ;

    char *ptr = (char*)aa_mem_region_alloc(reg, size);

    r = (struct aa_dmat*)ptr;
    aa_dmat_view( r, rows, cols, (double*)(ptr+s_desc+pad), rows );

    return r;
}

AA_API void
aa_dmat_set( struct aa_dmat *A, double alpha, double beta )
{
    int mi   = (int)(A->rows);
    int ni   = (int)(A->cols);
    int ldai = (int)(A->ld);

    dlaset_( "G", &mi, &ni,
             &alpha, &beta,
             A->data, &ldai );

}

AA_API void
aa_dvec_set( struct aa_dvec *vec, double alpha )
{
    double *end = vec->data + vec->len*vec->inc;
    for( double *x = vec->data; x < end; x += vec->inc ) {
        *x = alpha;
    }
}

void
aa_dvec_zero( struct aa_dvec *vec )
{
    aa_dvec_set(vec,0);
}


AA_API void
aa_dmat_zero( struct aa_dmat *mat )
{
    aa_dmat_set(mat, 0, 0);
}


/* Level 1 BLAS */
AA_API void
aa_dvec_swap( struct aa_dvec *x, struct aa_dvec *y )
{
    aa_la_check_size(x->len, y->len);
    cblas_dswap( VEC_LEN(x), AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API void
aa_dvec_scal( double a, struct aa_dvec *x )
{
    cblas_dscal( VEC_LEN(x), a, AA_VEC_ARGS(x) );
}

static void s_inc( size_t n, double alpha, double *x, size_t inc ) {
    for( double *end = x + n*inc; x < end; x += inc ) {
        *x += alpha;
    }
}

AA_API void
aa_dvec_inc( double alpha, struct aa_dvec *x )
{
    s_inc( x->len, alpha, x->data, x->inc );
}

AA_API void
aa_dvec_copy( const struct aa_dvec *x, struct aa_dvec *y )
{
    aa_la_check_size(x->len, y->len);
    cblas_dcopy( VEC_LEN(x), AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API void
aa_dvec_axpy( double a, const struct aa_dvec *x, struct aa_dvec *y )
{
    aa_la_check_size(x->len, y->len);
    cblas_daxpy( VEC_LEN(x), a, AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API double
aa_dvec_dot( const struct aa_dvec *x, struct aa_dvec *y )
{
    aa_la_check_size(x->len, y->len);
    return cblas_ddot( VEC_LEN(x), AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API double
aa_dvec_nrm2( const struct aa_dvec *x )
{
    return cblas_dnrm2( VEC_LEN(x), AA_VEC_ARGS(x) );
}

/* Level 2 BLAS */
AA_API void
aa_dmat_gemv( AA_CBLAS_TRANSPOSE trans,
              double alpha, const struct aa_dmat *A,
              const struct aa_dvec *x,
              double beta, struct aa_dvec *y )
{
    if( CblasTrans == trans ) {
        aa_la_check_size( A->rows, x->len );
        aa_la_check_size( A->cols, y->len );
    } else {
        aa_la_check_size( A->rows, y->len );
        aa_la_check_size( A->cols, x->len );
    }

    cblas_dgemv( CblasColMajor, trans,
                 MAT_ROWS(A), MAT_COLS(A),
                 alpha, AA_MAT_ARGS(A),
                 AA_VEC_ARGS(x),
                 beta, AA_VEC_ARGS(y) );

}

/* Level 3 BLAS */
AA_API void
aa_dmat_gemm( AA_CBLAS_TRANSPOSE transA, AA_CBLAS_TRANSPOSE transB,
              double alpha, const struct aa_dmat *A,
              const struct aa_dmat *B,
              double beta, struct aa_dmat *C )
{
    aa_la_check_size( A->rows, C->rows );
    aa_la_check_size( A->cols, B->rows );
    aa_la_check_size( B->cols, C->cols );

    assert( A->rows <= A->ld );
    assert( B->rows <= B->ld );
    assert( C->rows <= C->ld );

    cblas_dgemm( CblasColMajor,
                 transA, transB,
                 MAT_ROWS(A), MAT_COLS(B), MAT_COLS(A),
                 alpha, AA_MAT_ARGS(A),
                 AA_MAT_ARGS(B),
                 beta, AA_MAT_ARGS(C) );
}

/* LAPACK */

AA_API void
aa_dmat_lacpy( const char uplo[1],
               const struct aa_dmat *A,
               struct aa_dmat *B )
{

    aa_la_check_size(A->rows,B->rows);
    aa_la_check_size(A->cols,B->cols);
    int mi   = (int)(A->rows);
    int ni   = (int)(A->cols);
    int ldai = (int)(A->ld);
    int ldbi = (int)(B->ld);

    dlacpy_(uplo, &mi, &ni,
            A->data, &ldai,
            B->data, &ldbi);
}

AA_API void
aa_dmat_copy(  const struct aa_dmat *A, struct aa_dmat *B)
{
    aa_dmat_lacpy("G",A,B);
}


/* Matrix Functions */

static double s_ssd( size_t n,
                     double a,
                     double *x, size_t incx,
                     double *y, size_t incy )
{
    for( double *e = x + n*incx; x < e; x += incx, y+=incy ) {
        double d = *x - *y;
        a += d*d;
    }
    return a;
}


AA_API double
aa_dvec_ssd( const struct aa_dvec *x, const struct aa_dvec *y)
{
    aa_la_check_size( x->len, y->len );
    return s_ssd( x->len, 0,
                  x->data, x->inc,
                  y->data, y->inc );
}

AA_API double
aa_dmat_ssd( const struct aa_dmat *A, const struct aa_dmat *B)
{
    size_t m = A->rows, n = A->cols;
    aa_la_check_size( m, B->rows );
    aa_la_check_size( n, B->cols );
    double a = 0;
    for( double *Ac=A->data, *Bc=B->data, *ec=A->data + n*A->ld;
         Ac < ec;
         Ac+=A->ld, Bc+=B->ld )
    {
        a = s_ssd( m, a, Ac, 1, Bc, 1 );
    }
    return a;
}

AA_API double
aa_dmat_nrm2( const struct aa_dmat *A )
{
    int m=(int)A->rows, n=(int)A->cols, ld=(int)A->ld;
    return dlange_("F", &m, &n, A->data, &ld, NULL );
}

AA_API void
aa_dmat_scal( struct aa_dmat *x, double alpha )
{
    int m = (int)x->rows, n=(int)x->cols, ld=(int)x->ld;
    double cfrom=1;
    int info;
    dlascl_("G", NULL, NULL,
            &cfrom, &alpha,
            &m, &n, x->data, &ld,
            &info);
}

AA_API void
aa_dmat_inc( struct aa_dmat *A, double alpha )
{
    size_t m=A->rows, n=A->cols, ld=A->ld;
    double *x = A->data;

    if( m == n ) {
        s_inc(m*n, alpha, x, 1);
    } else {
        for( double *e = x + n*ld; x < e; x+=ld ) {
            s_inc(m, alpha, x, 1);
        }
    }

}

AA_API void
aa_dmat_axpy( double alpha, const struct aa_dmat *X, struct aa_dmat *Y)
{
    size_t m=X->rows, n=X->cols, ldX=X->ld, ldY=Y->ld;
    double *x=X->data, *y=Y->data;

    aa_la_check_size(m, Y->rows);
    aa_la_check_size(n, Y->cols);


    if( m == ldX && m == ldY ) {
        cblas_daxpy( (int)(m*n), alpha, x,1, y,1 );
    } else {
        int mi = (int)m;
        for( double *e = x + n*ldX; x < e; x+=ldX, y+=ldY ) {
            cblas_daxpy( mi, alpha, x,1, y,1 );
        }
    }
}

void
aa_dmat_trans( const struct aa_dmat *A, struct aa_dmat *B)
{
    const size_t m   = A->rows;
    const size_t n   = A->cols;
    const size_t lda = A->ld;
    const size_t ldb = B->ld;

    aa_la_check_size(m, B->cols);
    aa_la_check_size(n, B->rows);

    for ( double *Acol = A->data, *Brow=B->data, *Be=B->data + n;
          Brow < Be;
          Brow++, Acol+=lda )
    {
        cblas_dcopy( (int)m, Acol, 1, Brow, (int)ldb );
    }
}

AA_API int
aa_dmat_inv( struct aa_dmat *A )
{
    aa_la_check_size(A->rows,A->cols);
    int info;

    int *ipiv = (int*)
        aa_mem_region_local_alloc(sizeof(int)*A->rows);

    // LU-factor
    info = aa_cla_dgetrf( MAT_ROWS(A), MAT_COLS(A), AA_MAT_ARGS(A), ipiv );

    int lwork = -1;
    while(1) {
        double *work = (double*)
            aa_mem_region_local_tmpalloc( sizeof(double)*
                                      (size_t)(lwork < 0 ? 1 : lwork) );
        aa_cla_dgetri( MAT_ROWS(A), AA_MAT_ARGS(A), ipiv, work, lwork );
        if( lwork > 0 ) break;
        assert( -1 == lwork );
        lwork = (int)work[0];
    }

    aa_mem_region_local_pop(ipiv);

    return info;
}


int s_svd_helper ( const struct aa_dmat *A,
                   struct aa_mem_region *reg,
                   size_t *pkmin, size_t *pkmax,
                   double **pU, double **pVt, double **pS )
{

    size_t m = A->rows;
    size_t n = A->cols;

    // find  min/max dimensions
    if( m < n ) {
        *pkmin = m;
        *pkmax = n;
    } else {
        *pkmin = n;
        *pkmax = m;
    }

    // This method uses the SVD
    double *W  = (double*)aa_mem_region_alloc( reg, sizeof(double) * (m*m + n*n + *pkmin) );
    *pU  = W;        // size m*m
    *pVt = *pU + m*m; // size n*n
    *pS  = *pVt + n*n; // size min(m,n)

    // A = U S V^T
    aa_la_d_svd( m,n,
                 A->data, A->ld,
                 *pU, m, *pS, *pVt, n );


    return 0;
}


int
aa_dmat_pinv( const struct aa_dmat *A, double tol, struct aa_dmat *As )
{

    aa_la_check_size(A->rows, As->cols);
    aa_la_check_size(A->cols, As->rows);

    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    size_t kmin, kmax;
    double *U, *Vt, *S;
    s_svd_helper( A, reg,
                  &kmin, &kmax, &U, &Vt, &S );

    size_t m = A->rows;
    const int mi = (int)(A->rows);
    const int ni = (int)(A->cols);

    if( tol < 0 ) {
        tol = (double)kmax * S[0] * DBL_EPSILON;
    }

    // \sum 1/s_i * v_i * u_i^T
    aa_dmat_zero(As);
    double *Asd = As->data;
    for( size_t i = 0; i < kmin && S[i] > tol; i ++ ) {
        cblas_dger( CblasColMajor, ni, mi, 1/S[i],
                    Vt + i, ni,
                    U + m*i, 1,
                    Asd, ni
            );
    }

    aa_mem_region_pop( reg, ptrtop );

    return 0;

    /* struct aa_mem_region *reg = aa_mem_region_local_get(); */
    /* struct aa_dmat *Ap = aa_dmat_alloc(reg,m,n); */

    /* // B = AA^T */
    /* double *B; */
    /* int ldb; */
    /* if( m <= n ) { */
    /*     /\* Use A_star as workspace when it's big enough *\/ */
    /*     B = As->data; */
    /*     ldb = (int)(As->ld); */
    /* } else { */
    /*     B = AA_MEM_REGION_NEW_N( reg, double, m*m ); */
    /*     ldb = (int)m; */
    /* } */

    /* aa_la_dlacpy( "A", A, Ap ); */

    /* // B is symmetric.  Only compute the upper half. */
    /* cblas_dsyrk( CblasColMajor, CblasUpper, CblasNoTrans, */
    /*              MAT_ROWS(Ap), MAT_COLS(Ap), */
    /*              1, AA_MAT_ARGS(Ap), */
    /*              0, B, ldb ); */

    /* // B += kI */
    /* /\* for( size_t i = 0; i < m*(size_t)ldb; i += ((size_t)ldb+1) ) *\/ */
    /* /\*     B[i] += k; *\/ */

    /* /\* Solve via Cholesky Decomp *\/ */
    /* /\* B^T (A^*)^T = A    and     B = B^T (Hermitian) *\/ */

    /* aa_cla_dposv( 'U', (int)m, (int)n, */
    /*               B, ldb, */
    /*               AA_MAT_ARGS(Ap) ); */

    /* aa_dmat_trans( Ap, As ); */

    /* aa_mem_region_pop( reg, Ap ); */

}

int
aa_dmat_dpinv( const struct aa_dmat *A, double k, struct aa_dmat *As)
{
    aa_la_check_size(A->rows, As->cols);
    aa_la_check_size(A->cols, As->rows);

    // TODO: Try DGESV for non-positive-definite matrices

    size_t m = A->rows;
    size_t n = A->cols;

    struct aa_mem_region *reg = aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);
    int r = -1;

    /* As = inv(A'*A - k*I) * A' A'*inv(A*A' - k*I) */
    if( m <= n ) {
        /*
         * (A^*) = A^T*inv(A*A^T - k*I)
         * (A^*) = A^T*inv(B)
         * (A^*)*B = A^T
         * B^T (A^*)^T = A    and     B = B^T (Hermitian)
         *
         */

        /* Use A_star as workspace for B */
        double *B = As->data;
        int ldb = (int)(As->ld);

        /* Ap = A */
        struct aa_dmat *Ap = aa_dmat_alloc(reg,m,n);
        aa_dmat_lacpy( "A", A, Ap );

        // B is symmetric.  Only compute the upper half.
        // B = A * A'
        cblas_dsyrk( CblasColMajor, CblasUpper, CblasNoTrans,
                     MAT_ROWS(Ap), MAT_COLS(Ap),
                     1, AA_MAT_ARGS(Ap),
                     0, B, ldb );

        /* B += kI */
        for( double *x = B, *e = B+(int)m*ldb; x < e; x+=ldb+1 )
            *x += k;

        /* B^T (A^*)^T = A    and     B = B^T (Hermitian) */

        /* Solve via Cholesky (positive definite) */
        r = aa_cla_dposv( 'U', (int)m, (int)n,
                           B, ldb,
                           AA_MAT_ARGS(Ap) );

        /* Solve via LU */
        /* int *ipiv = AA_MEM_REGION_NEW_N(reg,int,m); */
        /* r = aa_la_d_sysv( "U", m, n, */
        /*                   B, (size_t)ldb, */
        /*                   ipiv, */
        /*                   Ap->data, Ap->ld ); */


        aa_dmat_trans( Ap, As );

    } else {

        /*
         * (A^*) = inv(A^T*A - k*I) * A^T
         * (A^*) = inv(B) * A^T
         * B*(A^*) = A^T
         */

        /* Use A_star as workspace for B */
        double *B = AA_MEM_REGION_NEW_N(reg,double,n*n);
        int ldb = (int)(n);

        /* As = A^T */
        aa_dmat_trans( A, As );

        // B is symmetric.  Only compute the upper half.
        // B = A' * A = As * As'
        cblas_dsyrk( CblasColMajor, CblasUpper, CblasNoTrans,
                     MAT_ROWS(As), MAT_COLS(As),
                     1, AA_MAT_ARGS(As),
                     0, B, ldb );

        /* B += kI */
        //for( size_t i = 0; i < n*(size_t)ldb; i += ((size_t)ldb+1) )
        for( double *x = B, *e = B+n*n; x < e; x+=ldb+1 )
            *x += k;

        /* B * (A^*)^T = A    and     B = B^T (Hermitian) */

        /* Solve via Cholesky (positive definite) */
        r = aa_cla_dposv( 'U', (int)n, (int)m,
                           B, ldb,
                           AA_MAT_ARGS(As) );

        /* Solve via LU */
        /* int *ipiv = AA_MEM_REGION_NEW_N(reg,int,n); */
        /* r = aa_la_d_sysv( "U", n, m, */
        /*                   B, (size_t)ldb, */
        /*                   ipiv, */
        /*                   As->data, As->ld ); */

    }


    aa_mem_region_pop( reg, ptrtop );

    return r;
}

int
aa_dmat_dzdpinv(  const struct aa_dmat *A, double s_min, struct aa_dmat *As)
{

    aa_la_check_size(A->rows, As->cols);
    aa_la_check_size(A->cols, As->rows);

    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    size_t kmin, kmax;
    double *U, *Vt, *S;
    s_svd_helper( A, reg,
                  &kmin, &kmax, &U, &Vt, &S );

    size_t m = A->rows;
    const int mi = (int)(A->rows);
    const int ni = (int)(A->cols);

    // \sum s_i/(s_i**2+k) * v_i * u_i^T
    aa_dmat_zero(As);
    double *Asd = As->data;
    size_t i = 0;

    // Undamped parts
    for( ; i < kmin && S[i] >= s_min; i ++ ) {
        cblas_dger( CblasColMajor, ni, mi, 1/S[i],
                    Vt + i, ni,
                    U + m*i, 1,
                    Asd, ni
            );
    }

    // Damped parts
    double s2 = s_min*s_min;
    for( ; i < kmin; i ++ ) {
        cblas_dger( CblasColMajor, ni, mi, S[i]/s2,
                    Vt + i, ni,
                    U + m*i, 1,
                    Asd, ni
            );
    }

    aa_mem_region_pop( reg, ptrtop );

    return 0;

}
