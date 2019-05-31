/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018-2019, Colorado School of Mines
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
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_ik_internal.h"
#include "amino/rx/scene_fk.h"

#include "amino/math.h"

#include "amino/rx/scene_wk.h"

#include "amino/opt/opt.h"

#include "amino/mat_internal.h"

#include "amino/rx/scene_wk_internal.h"

struct aa_rx_wk_lc3_cx {
    struct aa_opt_cx *opt_cx;
    const struct aa_rx_sg_sub *ssg;
    struct aa_rx_wk_opts wk_opts;
};


static void
lc3_constraints (
    const struct aa_rx_wk_lc3_cx *cx,
    double dt,
    size_t n_x, size_t n_q,
    const struct aa_rx_fk *fk,
    /* const double *dx_r, */
    /* const double *q_a, const double *dq_a, const double *dq_r, */
    const struct aa_dvec *dx_r,
    const struct aa_dvec *q_a,
    const struct aa_dvec *dq_a,
    const struct aa_dvec *dq_r,
    double **pA,
    double **pb_min,
    double **pb_max,
    double **px_min,
    double **px_max,
    double **pc
    )
{
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

    struct aa_dmat *J = aa_rx_sg_sub_jac_vel_get( ssg, reg, fk );
    //struct aa_dmat *J = aa_rx_sg_sub_get_jac_vel_get( ssg, reg, fk );
    struct aa_dmat vJstar = AA_DMAT_INIT( n_q, n_x, A, n_q );
    struct aa_dmat *N  = aa_dmat_alloc(reg, n_q, n_q);
    aa_rx_wk_get_js( ssg, &cx->wk_opts, fk, J, &vJstar );
    aa_rx_wk_get_n( ssg, &cx->wk_opts, J, &vJstar, 1, N );

    struct aa_dvec *dq_rn  = aa_dvec_alloc(reg, n_q );
    struct aa_dvec vAn    = AA_DVEC_INIT( n_q, A+n_q*n_x, 1 );
    struct aa_dvec vcx    = AA_DVEC_INIT( n_x, c, 1 );

    // Fill A
    /*  A = [J_star*dt, N*(dq_a - dq_r) ] */
    cblas_dscal( (int)(n_q*n_x), dt, A, 1 ); // A = J_star * dt
    // dq_rn = dq_a - dq_r
    aa_dvec_copy(dq_a, dq_rn);
    aa_dvec_axpy( -1, dq_r, dq_rn );
    aa_dmat_gemv( CblasNoTrans,
                  1, N, dq_rn,
                  0, &vAn );

    // Fill C
    // actual workspace velocity
    aa_dmat_gemv( CblasNoTrans,
                  1.0, J, dq_a,
                  0, &vcx );
    // c now holds dx_a
    // c = (c-dx_r)/(-dt)
    aa_dvec_axpy(-1, dx_r, &vcx);
    aa_dvec_scal(-1/dt, &vcx);

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

        double v_dq_a = dq_a->data[i*dq_a->inc];
        double v_q_a = q_a->data[i*q_a->inc];

        // velocity constraints
        if( isfinite(dq_min) ) b_min[i] = dq_min - v_dq_a;
        if( isfinite(dq_max) ) b_max[i] = dq_max - v_dq_a;

        if( isfinite(ddq_max) ) {
            // acceleration constraint
            b_max[i] = fmin( b_max[i], ddq_max*dt );
            // position constraint
            if( isfinite(q_min) )
                b_min[i] = fmax( b_min[i],
                                 (q_min-v_q_a)/dt - ddq_max*(dt/2) - v_dq_a );
        }

        if( isfinite(ddq_min) ) {
            // acceleration constraint
            b_min[i] = fmax( b_min[i], ddq_min*dt );
            // position constraint
            if( isfinite(q_max) )
                b_max[i] = fmin( b_max[i],
                                 (q_max - v_q_a)/dt - ddq_min*(dt/2) - v_dq_a );
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
    cx->ssg = ssg;
    memcpy(&cx->wk_opts, opts, sizeof(*opts));

    size_t n_x, n_q;
    aa_rx_sg_sub_jacobian_size( ssg, &n_x, &n_q );
    double dt = .01;
    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(ssg);
    size_t n_qall = aa_rx_sg_config_count(sg);


    struct aa_dvec *dx_r = aa_dvec_alloc(reg,n_x);
    struct aa_dvec *dq_a = aa_dvec_alloc(reg,n_q);
    struct aa_dvec *dq_r = aa_dvec_alloc(reg,n_q);
    struct aa_dvec *q_a  = aa_dvec_alloc(reg,n_q);

    double *q_all = AA_MEM_REGION_NEW_N(reg,double,n_qall);
    aa_rx_sg_center_configs(sg, n_qall, q_all);
    aa_rx_sg_sub_center_configs(ssg, n_q, q_a->data);

    aa_dvec_zero(dq_a);
    aa_dvec_zero(dq_r);
    aa_dvec_zero(dx_r);

    struct aa_dvec qv;
    aa_dvec_view( &qv, n_qall, q_all, 1 );

    struct aa_rx_fk *fk = aa_rx_fk_alloc(sg,reg);
    aa_rx_fk_all(fk,&qv);


    double *A;
    double *b_min, *b_max;
    double *x_min, *x_max;
    double *c;

    lc3_constraints (
        cx, dt,
        n_x, n_q,
        fk,
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
                    const struct aa_rx_fk *fk,
                    const struct aa_dvec *dx_r,
                    const struct aa_dvec *q_a,  const struct aa_dvec *dq_a,
                    const struct aa_dvec *dq_r, struct aa_dvec *dq )
{

    if( dt <= 0 ) return -1;
    const struct aa_rx_sg_sub *ssg = cx->ssg;


    size_t n_x, n_q;
    aa_rx_sg_sub_jacobian_size( ssg, &n_x, &n_q );
    aa_la_check_size( dx_r->len, n_x);
    aa_la_check_size( q_a->len,  n_q);
    aa_la_check_size( dq_a->len, n_q);
    aa_la_check_size( dq_r->len, n_q);
    aa_la_check_size( dq->len,   n_q);


    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);


    double *A;
    double *b_min, *b_max;
    double *x_min, *x_max;
    double *c;

    lc3_constraints (
        cx, dt,
        n_x, n_q,
        fk,
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
        aa_dvec_copy(dq_a, dq);

        // WS reference
        cblas_dgemv( CblasColMajor, CblasNoTrans,
                     (int)n_q, (int)n_x,
                     1.0, A, (int)n_q,
                     opt_x, 1,
                     1.0, AA_VEC_ARGS(dq) );

        // Nullspace Projection
        cblas_daxpy( (int)n_x,
                     opt_x[n_x], A+(n_x*n_q), 1,
                     AA_VEC_ARGS(dq) );

    }

    aa_mem_region_pop(reg,ptrtop);

    return r;
}
