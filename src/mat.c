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


AA_API void
aa_dvec_view( struct aa_dvec *vec, size_t len, double *data, size_t inc )
{
    vec->len = len;
    vec->data = data;
    vec->inc = inc;
}

AA_API void
aa_dmat_view( struct aa_dmat *mat, size_t rows, size_t cols, double *data, size_t ld )
{
    mat->rows = rows;
    mat->cols = cols;
    mat->data = data;
    mat->ld = ld;
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
