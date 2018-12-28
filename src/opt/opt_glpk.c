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
#include "amino/opt/opt.h"
#include "opt_internal.h"

#include <glpk.h>

static int s_solve( struct aa_opt_cx *cx, size_t n, double *x )
{
    glp_prob *lp = (glp_prob*)(cx->data);

    /* /\* Solve *\/ */
    /* int r = glp_interior( lp, NULL ); */

    /* /\* Result *\/ */
    /* for( int j = 0; j < (int)n; j ++ ) { */
    /*     x[j] = glp_ipt_col_prim( lp, j+1 ); */
    /* } */


    /* Solve */
    int r;
    int nb = glp_get_num_bin(lp);
    int ni = glp_get_num_int(lp);
    if( nb + ni > 0 ) {
        glp_simplex( lp, NULL );
        r = glp_intopt( lp, NULL );
        /* Result */
        for( int j = 0; j < (int)n; j ++ ) {
            x[j] = glp_mip_col_val( lp, j+1 );
        }
    } else {
        r = glp_simplex( lp, NULL );
        /* Result */
        for( int j = 0; j < (int)n; j ++ ) {
            x[j] = glp_get_col_prim( lp, j+1 );
        }
    }


    return r;
}

static int s_destroy( struct aa_opt_cx *cx )

{
    if( cx ) {
        glp_prob *lp = (glp_prob*)(cx->data);
        if( lp ) {
            glp_delete_prob(lp);
        }
        free(cx);
    }
    return 0;
}


static int
s_set_direction( struct aa_opt_cx *cx, enum aa_opt_direction dir )
{
    glp_prob *lp = (glp_prob*)(cx->data);
    switch(dir) {
    case AA_OPT_MAXIMIZE:
        glp_set_obj_dir(lp, GLP_MAX);
        return 0;
    case AA_OPT_MINIMIZE:
        glp_set_obj_dir(lp, GLP_MIN);
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
    glp_prob *lp = (glp_prob*)(cx->data);

    int type_glpk;
    switch( type ) {
    case AA_OPT_CONTINUOUS:
        type_glpk = GLP_CV;
        break;
    case AA_OPT_BINARY:
        type_glpk = GLP_BV;
        break;
    case AA_OPT_INTEGER:
        type_glpk = GLP_IV;
        break;
    default:
        return -1;
    }

    glp_set_col_kind( lp, (int)i+1, type_glpk );

    return 0;
}


static int bounds_type (double l, double u) {
    if( aa_opt_is_eq(l,u) ) {
        return GLP_FX;
    } else {
        int lb = aa_opt_is_lbound(l);
        int ub = aa_opt_is_ubound(u);
        if( aa_opt_is_leq(lb,ub) ) {
            assert( u < DBL_MAX );
            return GLP_UP;
        } else if( aa_opt_is_geq(lb,ub) ) {
            assert( l > -DBL_MAX );
            return GLP_LO;
        } else if( aa_opt_is_bound(lb,ub) ) {
            assert( u < DBL_MAX );
            assert( l > -DBL_MAX );
            return GLP_DB;
        } else if( aa_opt_is_free(lb,ub) ) {
            return GLP_FR;
        }
    }
    assert(0);
    return -1;
}




static int
s_set_obj( struct aa_opt_cx *cx, size_t n, const double * c)
{
    glp_prob *lp = (glp_prob*)(cx->data);
    int ni = (int) n;
    for( int j = 0; j < ni; j ++ ) {
        glp_set_obj_coef(lp, j+1, c[j]);
    }
    return 0;
}

static int
s_set_bnd( struct aa_opt_cx *cx, size_t n,
           const double * x_lower, const double *x_upper)
{
    glp_prob *lp = (glp_prob*)(cx->data);
    int ni = (int) n;
    for( int j = 0; j < ni; j ++ ) {
        double l=x_lower[j], u=x_upper[j];
        int type = bounds_type(l,u);;
        glp_set_col_bnds(lp, j+1, type, l, u);
    }
    return 0;
}

static int
s_set_cstr_gm( struct aa_opt_cx *cx,
               size_t m, size_t n,
               const double *A, size_t ldA )
{
    glp_prob *lp = (glp_prob*)(cx->data);
    int rows[m], mi = (int)m, ni = (int)n;
    for( int i = 0; i < mi; i ++ ) rows[i] = i+1;
    for( int j = 0; j < ni; j ++ ) {
        glp_set_mat_col( lp, j+1, mi,
                         rows-1, AA_MATCOL(A,ldA,(size_t)j) - 1 );
    }
    return 0;
}

static int
s_set_cstr_bnd( struct aa_opt_cx *cx, size_t m,
                const double * b_lower, const double *b_upper )
{
    glp_prob *lp = (glp_prob*)(cx->data);
    int mi = (int) m;
    for( int i = 0; i < mi; i ++ ) {
        double l=b_lower[i], u=b_upper[i];
        int type = bounds_type(l,u);;
        glp_set_row_bnds(lp, i+1, type, l, u);
    }
    return 0;
}


static struct aa_opt_vtab s_vtab = {
    .solve = s_solve,
    .destroy = s_destroy,
    .set_direction = s_set_direction,
    .set_quad_obj_crs = s_set_quad_obj_crs,
    .set_type = s_set_type,
    .set_obj = s_set_obj,
    .set_bnd = s_set_bnd,
    .set_cstr_bnd = s_set_cstr_bnd,
    .set_cstr_gm = s_set_cstr_gm
};

AA_API struct aa_opt_cx *
s_init (
    size_t m, size_t n,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper )
{
    glp_prob *lp = glp_create_prob();
    struct aa_opt_cx *cx = cx_finish( &s_vtab, lp );

    int ni = (int)n;
    int mi = (int)m;

    glp_set_obj_dir(lp, GLP_MAX);

    glp_add_rows(lp,mi);
    glp_add_cols(lp,ni);

    /* b_lower / b_upper */
    s_set_cstr_bnd(cx, m, b_lower, b_upper);

    /* x_lower / x_upper */
    s_set_bnd(cx, n, x_lower, x_upper);

    /* c */
    s_set_obj(cx, n, c );

    return cx;
}

AA_API struct aa_opt_cx *
aa_opt_glpk_gmcreate (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper )
{
    struct aa_opt_cx * cx =
        s_init(m,n,
               b_lower, b_upper,
               c,
               x_lower, x_upper );

    s_set_cstr_gm(cx, m, n, A, ldA);

    return cx;
}


AA_API struct aa_opt_cx *
aa_opt_glpk_crscreate (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper )
{
    int mi = (int)m;
    struct aa_opt_cx * cx =
        s_init(m,n,
               b_lower, b_upper,
               c,
               x_lower, x_upper );
    glp_prob *lp = (glp_prob*)(cx->data);

    /* A */
    for( int i = 0; i < mi; i ++ ) {
        int start = A_row_ptr[i];
        int end = A_row_ptr[i + 1];
        int count = end - start;
        const double *vals = A_values + start;
        const int *raw_inds = A_cols + start;
        int inds[count];
        for( int j = 0; j < count; j ++ ) {
            inds[j] = raw_inds[j] + 1;
        }
        glp_set_mat_row(lp, i+1, count, inds-1, vals-1);
    }


    return cx_finish( &s_vtab, lp );
}



AA_API int aa_opt_lp_crs_glpk (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{
    struct aa_opt_cx *cx = aa_opt_glpk_crscreate( m, n,
                                                  A_values, A_cols, A_row_ptr,
                                                  b_lower, b_upper,
                                                  c,
                                                  x_lower, x_upper );

    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;
}


AA_API int
aa_opt_lp_glpk (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{
    struct aa_opt_cx *cx = aa_opt_glpk_gmcreate( m, n,
                                                 A, ldA,
                                                 b_lower, b_upper,
                                                 c,
                                                 x_lower, x_upper );

    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;
}
