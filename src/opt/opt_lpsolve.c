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

#include <lpsolve/lp_lib.h>

int aa_opt_lp_lpsolve (
    size_t m, size_t n,
    enum aa_opt_rel_type *types,
    const double *A, size_t ldA,
    const double *b,
    const double *c,
    const double *l, const double *u,
    double *x )
{
    assert( sizeof(REAL) == sizeof(*A) );

    int mi = (int)m;
    int ni = (int)n;
    int r = -1;
    lprec *lp = make_lp(mi, ni);

    if( NULL == lp ) goto ERROR;
    set_add_rowmode(lp, FALSE);
    set_maxim(lp);

    /* TODO: maybe rearrange c/A and give to lp_solve together? */

    /* c */
    for( size_t j = 0; j < n; j ++ ) {
        set_obj( lp, 1+(int)j, c[j] );
    }

    /* A */
    for( size_t j = 0; j < n; j ++ ) {
        for( size_t i = 0; i < m; i ++ ) {
            int row = 1+(int)i, col = 1 + (int)j;
            double v = AA_MATREF(A, ldA, i, j);
            printf("%f\n",v);
            set_mat( lp, row, col, AA_MATREF(A, ldA, i, j) );
        }
    }

    /* types */
    for( int i = 0; i < mi; i ++ ) {
        switch( types[i] ) {
        case AA_OPT_REL_EQ:
            if( ! set_constr_type( lp, 1+i, EQ ) ) goto ERROR;
            break;
        case AA_OPT_REL_LEQ:
            if( ! set_constr_type( lp, 1+i, LE ) ) goto ERROR;
            break;
        case AA_OPT_REL_GEQ:
            if( ! set_constr_type( lp, 1+i, GE ) ) goto ERROR;
            break;
        default: {
            goto ERROR;
        }
        }
    }

    /* b */
    for( int i = 0; i < mi; i ++ ) {
        set_rh(lp,i+1,b[i]);
    }

    /* l/u */
    for( size_t j = 0; j < n; j ++ ) {
        set_bounds( lp, 1+(int)j, l[j], u[j] );
    }

    /* solve */
    r = solve(lp);
    assert(ni == get_Ncolumns(lp));

    /* result */
    get_variables(lp,x);

ERROR:
    if( lp ) delete_lp(lp);
    return r;

}
