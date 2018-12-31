/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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
 *   * Neither the name of the Rice University nor the names of its
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
#include "amino/rx/rxtype.h"
#include "amino/rx/rxerr.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_kin_internal.h"
#include "amino/rx/scene_sub.h"

#include "amino/rx/scene_wk.h"

#include "amino/opt/opt.h"

#include "amino/rx/scene_wk_internal.h"

struct aa_rx_wk_lc3_cx {
    struct aa_opt_cx *opt_cx;
    const struct aa_rx_sg_sub *ssg;
    double s2min;
    double k_dls;
};



static void
lc3_constraints (
    const struct aa_rx_wk_lc3_cx *cx,
    double dt,
    size_t n_x, size_t n_q,
    size_t n_tf, const double *TF_abs, size_t ld_tf,
    const double *dx_r,
    const double *q_a, const double *dq_a, const double *dq_r,
    double **pA,
    double **pb_min,
    double **pb_max,
    double **px_min,
    double **px_max,
    double **pc
    )
{
    assert(TF_abs);
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    const struct aa_rx_sg_sub *ssg = cx->ssg;
    {
        size_t rows,cols;
        aa_rx_sg_sub_jacobian_size( ssg, &rows, &cols );
        assert(n_x == rows);
        assert(n_q == cols);
    }

    /* Optimization variables
     *
     * Maximize: x .dot. c
     * Subject to:
     *   x_lower <=   x <= x_upper
     *   b_lower <= A*x <= b_upper
     */
    double *A     = AA_MEM_REGION_NEW_N( reg, double, n_q * (n_x+1) );
    double *b_min = AA_MEM_REGION_NEW_N( reg, double, n_q );
    double *b_max = AA_MEM_REGION_NEW_N( reg, double, n_q );
    double *x_min = AA_MEM_REGION_NEW_N( reg, double, n_x + 1 );
    double *x_max = AA_MEM_REGION_NEW_N( reg, double, n_x + 1 );
    double *c     = AA_MEM_REGION_NEW_N( reg, double, n_x + 1 );
    *pA     = A;
    *pb_min = b_min;
    *pb_max = b_max;
    *px_min = x_min;
    *px_max = x_max;
    *pc     = c;


    // Rest of allocated arrays are local temps
    void *ptrtop = aa_mem_region_ptr(reg);

    double *J     = AA_MEM_REGION_NEW_N(reg, double, n_x*n_q);
    double *N     = AA_MEM_REGION_NEW_N(reg, double, n_q*n_q);
    double *dq_rn = AA_MEM_REGION_NEW_N(reg, double, n_q );

    assert(TF_abs);
    aa_rx_sg_sub_jacobian( ssg, n_tf, TF_abs, ld_tf, J, n_x );


    {
        double *J_star = A; // Reuse A matrix

        // Compute a damped pseudo inverse
        if( cx->s2min > 0 ) {
            aa_la_dzdpinv( n_x, n_q, cx->s2min, J, J_star );
        } else  {
            aa_la_dpinv( n_x, n_q, cx->k_dls, J, J_star );
        }

        // Compute nullspace projection matrix
        aa_la_np( n_x, n_q, J, J_star, N );
    }

    // Fill A
    /*  A = [J_star*dt, N*(dq_a - dq_r) ] */
    cblas_dscal( (int)(n_q*n_x), dt, A, 1 ); // A = J_star * dt
    for( size_t i = 0; i < n_q; i++) {
        dq_rn[i] = dq_a[i] - dq_r[i];
    }
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)n_q, (int)n_q,
                 1, N, (int)n_q,
                 dq_rn, 1,
                 0.0, A+n_q*n_x, 1 );

    // Fill C
    // actual workspace velocity
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)n_x, (int)n_q,
                 1.0, J, (int)n_x,
                 dq_a, 1,
                 0.0, c, 1 );
    // c now holds dx_a
    for( size_t i = 0; i < n_x; i ++ ) {
        c[i] = (dx_r[i] - c[i])/dt;
    }
    c[n_x] = 1; // TODO: add weighting parameter to options

    // Fill x bounds
    for( size_t i = 0; i < n_x; i ++ ) {
        if( c[i] > 0 ) {
            x_min[i] = 0;
            x_max[i] = c[i];
        } else if ( c[i] < 0 ) {
            x_min[i] = c[i];
            x_max[i] = 0;
        } else {
            x_min[i] = 0;
            x_max[i] = 0;
        }
    }
    x_min[n_x] = 0;
    x_max[n_x] = 1; // TODO: better max

    // Fill b bounds
    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(ssg);
    for( size_t i = 0; i < n_q; i ++ ) {
        double   q_max = INFINITY,   q_min = -INFINITY;
        double  dq_max = INFINITY,  dq_min = -INFINITY;
        double ddq_max = INFINITY, ddq_min = -INFINITY;
        aa_rx_config_id j = aa_rx_sg_sub_config(ssg, i);
        b_min[i] = -INFINITY;
        b_max[i] =  INFINITY;
        aa_rx_sg_get_limit_pos(sg, j, &q_min, &q_max);
        aa_rx_sg_get_limit_vel(sg, j, &dq_min, &dq_max);
        aa_rx_sg_get_limit_acc(sg, j, &ddq_min, &ddq_max);

        // velocity constraints
        if( isfinite(dq_min) ) b_min[i] = dq_min - dq_a[i];
        if( isfinite(dq_max) ) b_max[i] = dq_max - dq_a[i];

        if( isfinite(ddq_max) ) {
            // acceleration constraint
            b_max[i] = fmin( b_max[i], ddq_max*dt );
            // position constraint
            if( isfinite(q_min) )
                b_min[i] = fmax( b_min[i],
                                 (q_min-q_a[i])/dt - ddq_max*(dt/2) - dq_a[i] );
        }

        if( isfinite(ddq_min) ) {
            // acceleration constraint
            b_min[i] = fmax( b_min[i], ddq_min*dt );
            // position constraint
            if( isfinite(q_max) )
                b_max[i] = fmin( b_max[i],
                                 (q_max - q_a[i])/dt - ddq_min*(dt/2) - dq_a[i] );
        }

    }

    aa_mem_region_pop(reg,ptrtop);
}


AA_API  struct aa_rx_wk_lc3_cx *
aa_rx_wk_lc3_create ( const const struct aa_rx_sg_sub *ssg,
                      const struct aa_rx_wk_opts * opts )
{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);


    struct aa_rx_wk_lc3_cx *cx = AA_NEW(struct aa_rx_wk_lc3_cx);
    cx->s2min = opts->s2min;
    cx->k_dls = opts->k_dls;
    cx->ssg = ssg;

    size_t n_x, n_q;
    aa_rx_sg_sub_jacobian_size( ssg, &n_x, &n_q );
    double dt = .01;
    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(ssg);
    size_t n_qall = aa_rx_sg_config_count(sg);
    size_t n_tf = aa_rx_sg_frame_count(sg);


    double *dx_r  = AA_MEM_REGION_ZNEW_N(reg,double,n_x);
    double *dq_a  = AA_MEM_REGION_ZNEW_N(reg,double,n_q);
    double *dq_r  = AA_MEM_REGION_ZNEW_N(reg,double,n_q);

    double *q_a   = AA_MEM_REGION_NEW_N(reg,double,n_q);
    double *q_all = AA_MEM_REGION_NEW_N(reg,double,n_qall);
    aa_rx_sg_center_configs(sg, n_qall, q_all);
    aa_rx_sg_sub_center_configs(ssg, n_q, q_a);

    struct aa_dvec qv;
    aa_dvec_view( &qv, n_qall, q_all, 1 );
    struct aa_dmat *TF_abs = aa_rx_sg_tf_abs(sg, reg, &qv);

    double *A;
    double *b_min, *b_max;
    double *x_min, *x_max;
    double *c;

    lc3_constraints (
        cx, dt,
        n_x, n_q,
        n_tf, TF_abs->data, TF_abs->ld,
        dx_r,
        q_a, dq_a, dq_r,
        &A,
        &b_min, &b_max,
        &x_min, &x_max,
        &c );


    cx->opt_cx =
        aa_opt_gmcreate( opts->lp_solver,
                         n_q, n_x+1,
                         A, n_q,
                         b_min, b_max,
                         c,
                         x_min, x_max );

    aa_opt_set_direction( cx->opt_cx, AA_OPT_MAXIMIZE );

    aa_mem_region_pop(reg,ptrtop);

    return cx;
}


AA_API int
aa_rx_wk_dx2dq_lc3( const struct aa_rx_wk_lc3_cx *cx,
                    double dt,
                    size_t n_tf, const double *TF_abs, size_t ld_tf,
                    size_t n_x, const double *dx_r,
                    size_t n_q,
                    const double *q_a, const double *dq_a,
                    const double *dq_r, double *dq )
{

    if( dt <= 0 ) return -1;


    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);


    double *A;
    double *b_min, *b_max;
    double *x_min, *x_max;
    double *c;

    lc3_constraints (
        cx, dt,
        n_x, n_q,
        n_tf, TF_abs, ld_tf,
        dx_r,
        q_a, dq_a, dq_r,
        &A,
        &b_min, &b_max,
        &x_min, &x_max,
        &c );

    double *opt_x = AA_MEM_REGION_NEW_N(reg,double, 1+n_x);
    struct aa_opt_cx *opt_cx = cx->opt_cx;
    aa_opt_set_obj( opt_cx, n_x+1, c );
    aa_opt_set_bnd( opt_cx, n_x+1, x_min, x_max );
    aa_opt_set_cstr_gm( opt_cx, n_q, n_x+1, A, n_q, b_min, b_max );

    int  r = aa_opt_solve( opt_cx, n_x+1, opt_x );

    if( 0 == r ) {
        // extract velocity
        AA_MEM_CPY( dq, dq_a, n_q );

        // WS reference
        cblas_dgemv( CblasColMajor, CblasNoTrans,
                     (int)n_q, (int)n_x,
                     1.0, A, (int)n_q,
                     opt_x, 1,
                     1.0, dq, 1 );

        // Nullspace Projection
        cblas_daxpy( (int)n_x,
                     opt_x[n_x], A+(n_x*n_q), 1,
                     dq, 1 );

    }

    aa_mem_region_pop(reg,ptrtop);

    return r;
}
