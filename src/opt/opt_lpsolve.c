/* -*- mode: C; c-basic-offset: 4; -*- */
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

#include "opt_internal.h"

#include <lpsolve/lp_lib.h>

static int aa_opt_lp_lpsolve_finish (
    lprec *lp,
    size_t m, size_t n,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{
    int ni = (int)n;
    int mi = (int)m;
    (void)mi;


    /* printf("c:"); aa_dump_vec(stdout, c, n ); */
    /* printf("xl:"); aa_dump_vec(stdout, x_lower, n ); */
    /* printf("xu:"); aa_dump_vec(stdout, x_upper, n ); */

    /* c */
    for( size_t j = 0; j < n; j ++ ) {
        set_obj( lp, 1+(int)j, c[j] );
    }

    /* l/u */
    for( size_t j = 0; j < n; j ++ ) {
        set_bounds( lp, 1+(int)j, x_lower[j], x_upper[j] );
    }

    /* solve */
    int r = solve(lp);
    assert(ni == get_Ncolumns(lp));

    /* result */
    get_variables(lp,x);
    return r;
}


int aa_opt_lp_lpsolve (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{
    struct aa_opt_cx *cx = aa_opt_lpsolve_gmcreate( m, n,
                                                    A, ldA,
                                                    b_lower, b_upper,
                                                    c,
                                                    x_lower, x_upper );
    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;


}

AA_API int aa_opt_lp_crs_lpsolve (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{
    struct aa_opt_cx *cx = aa_opt_lpsolve_crscreate( m, n,
                                                     A_values, A_cols, A_row_ptr,
                                                     b_lower, b_upper,
                                                     c,
                                                     x_lower, x_upper );
    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;
}






int s_solve( struct aa_opt_cx *cx, size_t n, double *x )
{

    lprec *lp = (lprec*)cx->data;
    int r = solve(lp);
    assert((int)n == get_Ncolumns(lp));

    /* result */
    get_variables(lp,x);

    return r;
}
int s_destroy( struct aa_opt_cx *cx )
{
    if( cx ) {
        lprec *lp = (lprec*)cx->data;
        if( lp ) {
            delete_lp( lp );
        }
        free(cx);
    }
    return 0;
}

static struct aa_opt_vtab vtab = {
    .solve = s_solve,
    .destroy = s_destroy
};


AA_API struct aa_opt_cx* aa_opt_lpsolve_gmcreate (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper
    )
{
    assert( sizeof(REAL) == sizeof(*A) );

    int mi = (int)m;
    int ni = (int)n;
    lprec *lp = make_lp(mi, ni);

    if( NULL == lp ) goto ERROR;
    set_add_rowmode(lp, FALSE);
    set_maxim(lp);

    /* TODO: maybe rearrange c/A and give to lp_solve together? */

    /*  printf("A:\n"); aa_dump_mat(stdout, A, ldA, n ); */
    /* printf("bl:"); aa_dump_vec(stdout, b_lower, m ); */
    /* printf("bu:"); aa_dump_vec(stdout, b_upper, m ); */


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
            } else {
                int lb = aa_opt_is_lbound(l);
                int ub = aa_opt_is_ubound(u);
                if ( aa_opt_is_leq(lb,ub) ) {
                    // less than
                    rh = u;
                    con_type = LE;
                } else if ( aa_opt_is_geq(lb,ub) ) {
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
        }
        set_rh(lp,ilp,rh);
        if( ! set_constr_type( lp, ilp, con_type ) ) goto ERROR;
        ilp++;
    }


    /* c */
    for( size_t j = 0; j < n; j ++ ) {
        set_obj( lp, 1+(int)j, c[j] );
    }

    /* l/u */
    for( size_t j = 0; j < n; j ++ ) {
        set_bounds( lp, 1+(int)j, x_lower[j], x_upper[j] );
    }

ERROR:
    if( lp ) {
        struct aa_opt_cx *cx = AA_NEW(struct aa_opt_cx);
        cx->vtab = &vtab;
        cx->data = lp;
        return cx;
    } else {
        return NULL;
    }


}

AA_API struct aa_opt_cx *
aa_opt_lpsolve_crscreate (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper )
{
    assert( sizeof(REAL) == sizeof(*A_values) );

    int mi = (int)m;
    int ni = (int)n;
    int r = -1;
    lprec *lp = make_lp(0, ni);

    if( NULL == lp ) goto ERROR;
    set_maxim(lp);

    /* A, b_l, b_u */
    set_add_rowmode(lp, TRUE);
    for( int i = 0; i < mi; i ++ ) {
        double l=b_lower[i], u=b_upper[i];
        int start = A_row_ptr[i];
        int end = A_row_ptr[i + 1];
        int count = end - start;
        double *vals = (double*)A_values + start;
        int *raw_inds = (int*)A_cols + start;
        int inds[count];
        for( int j = 0; j < count; j ++ ) {
            inds[j] = raw_inds[j] + 1;
        }
        double rh;
        int con_type;
        if( aa_feq(l,u,0) ) {
            // equality
            con_type = EQ;
            rh = u;
        } else {
            int lb = aa_opt_is_lbound(l);
            int ub = aa_opt_is_ubound(u);
            if ( aa_opt_is_leq(lb,ub) )  {
                // less than
                con_type = LE;
                rh = u;
            } else if ( aa_opt_is_geq(lb,ub) )  {
                rh = l;
                con_type = GE;
            } else {
                // leq
                add_constraintex(lp, count, vals, inds, LE, u);
                con_type = GE;
                rh = l;
            }
        }
        add_constraintex(lp, count, vals, inds, con_type, rh);
    }
    set_add_rowmode(lp, FALSE);

   /* c */
    for( size_t j = 0; j < n; j ++ ) {
        set_obj( lp, 1+(int)j, c[j] );
    }

    /* l/u */
    for( size_t j = 0; j < n; j ++ ) {
        set_bounds( lp, 1+(int)j, x_lower[j], x_upper[j] );
    }

ERROR:
    if( lp ) {
        struct aa_opt_cx *cx = AA_NEW(struct aa_opt_cx);
        cx->vtab = &vtab;
        cx->data = lp;
        return cx;
    } else {
        return NULL;
    }
}
