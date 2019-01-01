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

static aa_lb_err_fun  *s_err_fun = s_err_default;

void
aa_lb_set_err( aa_lb_err_fun *fun )
{
    s_err_fun = fun;
}

AA_API void
aa_lb_err( const char *message ) {
    s_err_fun(message);
}

AA_API void
aa_lb_fail_size( size_t a, size_t b )
{
        const size_t size = 256;
        char buf[size];
        snprintf(buf,size, "Mismatched sizes: %lu != %lu\n", a, b);
        aa_lb_err(buf);
}

/****************/
/* Construction */
/****************/

AA_API void
aa_dvec_view( struct aa_dvec *vec, size_t len, double *data, size_t inc )
{
    *vec = AA_DVEC_INIT(len,data,inc);
}

AA_API void
aa_dmat_view( struct aa_dmat *mat, size_t rows, size_t cols, double *data, size_t ld )
{
    *mat = AA_DMAT_INIT(rows,cols,data,ld);
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
    aa_dvec_view( r, len, (double*)(ptr+off), 1 );
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
    aa_dvec_view( r, len, (double*)(ptr+s_desc+pad), 1 );
    return r;
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
    aa_dmat_view( r, rows, cols, (double*)(ptr+s_desc+pad), rows );

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


void
aa_dvec_zero( struct aa_dvec *vec )
{
    double *end = vec->data + vec->len*vec->inc;
    for( double *x = vec->data; x < end; x += vec->inc ) {
        *x = 0.0;
    }
}


/* Level 1 BLAS */
AA_API void
aa_lb_dswap( struct aa_dvec *x, struct aa_dvec *y )
{
    aa_lb_check_size(x->len, y->len);
    cblas_dswap( VEC_LEN(x), AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API void
aa_lb_dscal( double a, struct aa_dvec *x )
{
    cblas_dscal( VEC_LEN(x), a, AA_VEC_ARGS(x) );
}

AA_API void
aa_lb_dcopy( const struct aa_dvec *x, struct aa_dvec *y )
{
    aa_lb_check_size(x->len, y->len);
    cblas_dcopy( VEC_LEN(x), AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API void
aa_lb_daxpy( double a, const struct aa_dvec *x, struct aa_dvec *y )
{
    aa_lb_check_size(x->len, y->len);
    cblas_daxpy( VEC_LEN(x), a, AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API double
aa_lb_ddot( const struct aa_dvec *x, struct aa_dvec *y )
{
    aa_lb_check_size(x->len, y->len);
    return cblas_ddot( VEC_LEN(x), AA_VEC_ARGS(x), AA_VEC_ARGS(y) );
}

AA_API double
aa_lb_dnrm2( const struct aa_dvec *x )
{
    return cblas_dnrm2( VEC_LEN(x), AA_VEC_ARGS(x) );
}

/* Level 2 BLAS */
AA_API void
aa_lb_dgemv( CBLAS_TRANSPOSE trans,
             double alpha, const struct aa_dmat *A,
             const struct aa_dvec *x,
             double beta, struct aa_dvec *y )
{
    aa_lb_check_size( A->rows, y->len );
    aa_lb_check_size( A->cols, x->len );

    cblas_dgemv( CblasColMajor, trans,
                 MAT_ROWS(A), MAT_COLS(A),
                 alpha, AA_MAT_ARGS(A),
                 AA_VEC_ARGS(x),
                 beta, AA_VEC_ARGS(y) );

}

/* Level 3 BLAS */
AA_API void
aa_lb_dgemm( CBLAS_TRANSPOSE transA, CBLAS_TRANSPOSE transB,
             double alpha, const struct aa_dmat *A,
             const struct aa_dmat *B,
             double beta, struct aa_dmat *C )
{
    aa_lb_check_size( A->rows, C->rows );
    aa_lb_check_size( A->cols, B->rows );
    aa_lb_check_size( B->cols, C->cols );

    cblas_dgemm( CblasColMajor,
                 transA, transB,
                 MAT_ROWS(A), MAT_COLS(A), MAT_COLS(B),
                 alpha, AA_MAT_ARGS(A),
                 AA_MAT_ARGS(B),
                 beta, AA_MAT_ARGS(C) );
}

/* LAPACK */

AA_API void
aa_lb_dlacpy( const char uplo[1],
              const struct aa_dmat *A,
              struct aa_dmat *B )
{

    aa_lb_check_size(A->rows,B->rows);
    aa_lb_check_size(A->cols,B->cols);
    int mi   = (int)(A->rows);
    int ni   = (int)(A->cols);
    int ldai = (int)(A->ld);
    int ldbi = (int)(B->ld);

    dlacpy_(uplo, &mi, &ni,
            A->data, &ldai,
            B->data, &ldbi);
}


/* Matrix Functions */

void
aa_dmat_trans( const struct aa_dmat *A, struct aa_dmat *B)
{
    const size_t m   = A->rows;
    const size_t n   = A->cols;
    const size_t lda = A->ld;
    const size_t ldb = B->ld;

    aa_lb_check_size(m, B->cols);
    aa_lb_check_size(n, B->rows);

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
    aa_lb_check_size(A->rows,A->cols);
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


/* int */
/* aa_dmat_pinv( const struct aa_dmat *A, struct aa_dmat *As ) */
/* { */
/*     aa_lb_check_size(A->rows, As->cols); */
/*     aa_lb_check_size(A->cols, As->rows); */

/*     size_t m = A->rows; */
/*     size_t n = A->cols; */

/*     struct aa_mem_region *reg = aa_mem_region_local_get(); */
/*     struct aa_dmat *Ap = aa_dmat_alloc( reg,n,m); */

/*     // B = AA^T */
/*     double *B; */
/*     int ldb; */
/*     if( m <= n ) { */
/*         /\* Use A_star as workspace when it's big enough *\/ */
/*         B = As->data; */
/*         ldb = (int)(As->ld); */
/*     } else { */
/*         B = AA_MEM_REGION_NEW_N( reg, double, m*m ); */
/*         ldb = (int)m; */
/*     } */

/*     aa_lb_dlacpy( "A", A, Ap ); */

/*     // B is symmetric.  Only compute the upper half. */
/*     cblas_dsyrk( CblasColMajor, CblasUpper, CblasNoTrans, */
/*                  MAT_ROWS(Ap), MAT_COLS(Ap), */
/*                  1, AA_MAT_ARGS(Ap), */
/*                  0, B, ldb ); */

/*     // B += kI */
/*     /\* for( size_t i = 0; i < m*(size_t)ldb; i += ((size_t)ldb+1) ) *\/ */
/*     /\*     B[i] += k; *\/ */

/*     /\* Solve via Cholesky Decomp *\/ */
/*     /\* B^T (A^*)^T = A    and     B = B^T (Hermitian) *\/ */

/*     aa_cla_dposv( 'U', (int)m, (int)n, */
/*                   B, ldb, */
/*                   AA_MAT_ARGS(Ap) ); */

/*     aa_la_d_transpose( m, n, Ap, m, As, n ); */

/*     aa_mem_region_pop( reg, Ap ); */
/* } */

/* int */
/* aa_dmat_dpinv( double k, const struct aa_dmat *A, struct aa_dmat *Ap); */

/* int */
/* aa_dmat_dzdpinv( double s2min, const struct aa_dmat *A, struct aa_dmat *Ap); */
