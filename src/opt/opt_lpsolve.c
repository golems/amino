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
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
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

    /* A, b */
    int ilp = 1;
    for( size_t i = 0; i < m; i ++ ) {
        /* set row */
        for( size_t j = 0; j < n; j ++ ) {
            int col = 1 + (int)j;
            double v = AA_MATREF(A, ldA, i, j);
            set_mat( lp, ilp, col, v );
        }

        /* set row bound */
        double rh;
        int con_type;
        {
            double l=b_lower[i], u=b_upper[i];
            if( aa_feq(l,u,0) ) {
                // equality
                rh = u;
                con_type = EQ;
            } else if (-DBL_MAX >= l ||
                       -1 == isinf(l) ) {
                // less than
                rh = u;
                con_type = LE;
            } else if (DBL_MAX <= u ||
                       1 == isinf(u)) {
                rh = l;
                con_type = GE;
            } else {
                // leq
                set_rh(lp,ilp,u);
                if( ! set_constr_type( lp, ilp, LE ) ) goto ERROR;
                ilp++;
                // geq
                for( size_t j = 0; j < n; j ++ ) {
                    int col = 1 + (int)j;
                    double v = AA_MATREF(A, ldA, i, j);
                    set_mat( lp, ilp, col, v );
                }
                rh = l;
                con_type = GE;
            }
        }
        set_rh(lp,ilp,rh);
        if( ! set_constr_type( lp, ilp, con_type ) ) goto ERROR;
        ilp++;
    }

    /* l/u */
    for( size_t j = 0; j < n; j ++ ) {
        set_bounds( lp, 1+(int)j, x_lower[j], x_upper[j] );
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
