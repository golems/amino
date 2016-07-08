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

static int
s_set_direction( struct aa_opt_cx *cx, enum aa_opt_direction dir )
{
    lprec *lp = (lprec*)cx->data;
    switch(dir) {
    case AA_OPT_MAXIMIZE:
        set_maxim(lp);
        return 0;
    case AA_OPT_MINIMIZE:
        set_minim(lp);
        return 0;
    }
    return -1;
}


static int s_set_quad_obj_crs( struct aa_opt_cx *cx, size_t n,
                               const double *Q_values, int *Q_cols, int *Q_row_ptr )
{
    (void)cx;
    (void)n;
    (void)Q_values;
    (void)Q_cols;
    (void)Q_row_ptr;
    return -1;
}

static int s_set_type( struct aa_opt_cx *cx, size_t i, enum aa_opt_type type ) {
    lprec *lp = (lprec*)cx->data;
    int ii = (int) i + 1;

    switch( type ) {
    case AA_OPT_CONTINUOUS:
        set_int(lp, ii, 0);
        set_binary(lp, ii, 0);
        break;
    case AA_OPT_BINARY:
        set_binary(lp, ii, 1);
        break;
    case AA_OPT_INTEGER:
        set_int(lp, ii, 1);
        break;
    default:
        return -1;
    }

    return 0;
}

static struct aa_opt_vtab vtab = {
    .solve = s_solve,
    .destroy = s_destroy,
    .set_direction = s_set_direction,
    .set_quad_obj_crs = s_set_quad_obj_crs,
    .set_type = s_set_type
};

static struct aa_opt_cx*
s_finish (
    size_t m, size_t n,
    lprec *lp,
    const double *c,
    const double *x_lower, const double *x_upper )
{
    (void)m;
    /* c */
    for( size_t j = 0; j < n; j ++ ) {
        set_obj( lp, 1+(int)j, c[j] );
    }

    /* l/u */
    for( size_t j = 0; j < n; j ++ ) {
        set_bounds( lp, 1+(int)j, x_lower[j], x_upper[j] );
    }

    return cx_finish( &vtab, lp );
}

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

    return s_finish(m,n,lp,
                    c, x_lower, x_upper);

ERROR:
    if( lp ) {
        delete_lp(lp);
    }
    return NULL;

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
    lprec *lp = make_lp(0, ni);

    if( NULL == lp ) return NULL;
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

    return s_finish(m,n,lp,
                    c, x_lower, x_upper);
}
