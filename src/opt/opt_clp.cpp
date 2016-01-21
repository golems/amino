/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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


#include "amino.h"
#include "amino/opt/lp.h"
#include "amino/opt/opt.h"
#include "opt_internal.h"

#include <coin/ClpSimplex.hpp>

typedef ClpSimplex SolverType;


int s_solve( struct aa_opt_cx *cx, size_t n, double *x )
{
    SolverType * M = static_cast<SolverType*>(cx->data);

    /* Solve */
    int r = M->initialSolve();

    /* Result */
    AA_MEM_CPY( x, M->getColSolution(), n );

    return r;
}
int s_destroy( struct aa_opt_cx *cx )
{
    if( cx ) {
        SolverType * M = static_cast<SolverType*>(cx->data);
        if( M ) {
            delete M;
        }
        free(cx);
    }
    return 0;
}

static struct aa_opt_vtab s_vtab = {
    .solve = s_solve,
    .destroy = s_destroy
};


AA_API struct aa_opt_cx* aa_opt_clp_gmcreate (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper
    )
{


    ClpSimplex *pM = new ClpSimplex();
    ClpSimplex &M = *pM;

    int rows[m];
    int mi = (int)m;
    int ni = (int)n;
    M.resize(mi,0);

    for( int i = 0; i < mi; i ++ ) rows[i] = i;

    /* A, c, l, u */
    for( size_t j = 0; j < n; j ++ ) {
        M.addColumn( mi, rows,
                     AA_MATCOL(A,ldA,j),
                     x_lower[j], x_upper[j], c[j] );
    }

    /* b */
    for( int i = 0; i < mi; i ++ ) {
        M.setRowBounds(i,b_lower[i],b_upper[i]);
    }


    M.setOptimizationDirection( -1 );


    return cx_finish( &s_vtab, pM );
}


AA_API struct aa_opt_cx* aa_opt_clp_crscreate (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper )
{
    ClpSimplex *pM = new ClpSimplex();
    ClpSimplex &M = *pM;
    int rows[m];
    int mi = (int)m;
    int ni = (int)n;
    M.resize(0,ni);

    /* A, b */
    for( int i = 0; i < mi; i ++ ) {
        int start = A_row_ptr[i];
        int end = A_row_ptr[i + 1];
        M.addRow( end - start,
                  A_cols+start,
                  A_values+start,
                  b_lower[i],
                  b_upper[i] );
    }

    /* c, xl, xu */
    M.chgColumnLower(x_lower);
    M.chgColumnUpper(x_upper);
    M.chgObjCoefficients(c);

    /* Solve */
    M.setOptimizationDirection( -1 );


    return cx_finish( &s_vtab, pM );
}

int aa_opt_lp_clp (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{

    struct aa_opt_cx *cx = aa_opt_clp_gmcreate( m, n,
                                                A, ldA,
                                                b_lower, b_upper,
                                                c,
                                                x_lower, x_upper );
    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;
}

AA_API int aa_opt_lp_crs_clp (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{
    struct aa_opt_cx *cx = aa_opt_clp_crscreate( m, n,
                                                 A_values, A_cols, A_row_ptr,
                                                 b_lower, b_upper,
                                                 c,
                                                 x_lower, x_upper );
    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;
}
