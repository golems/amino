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
#include "amino/rx/rx_ct.h"

#include "amino/opt/opt.h"

#include "rx_ct_internal.h"

AA_API int
aa_rx_ct_wk_dx2dq_lc3( const const struct aa_rx_sg_sub *ssg,
                       const struct aa_rx_ct_wk_opts * opts,
                       double dt,
                       size_t n_tf, const double *TF_abs, size_t ld_tf,
                       size_t n_x, const double *dx_r,
                       size_t n_q,
                       const double *q_a, const double *dq_a,
                       const double *dq_r, double *dq )
{

    size_t rows,cols;
    aa_rx_sg_sub_jacobian_size( ssg, &rows, &cols );
    assert(n_x == rows);
    assert(n_q == cols);

    double *J = AA_MEM_REGION_LOCAL_NEW_N(double, rows*cols);
    double *J_star = AA_MEM_REGION_LOCAL_NEW_N(double, rows*cols);
    double *N = AA_MEM_REGION_LOCAL_NEW_N(double, cols*cols);
    double *dx_a = AA_MEM_REGION_LOCAL_NEW_N(double, n_x);

    aa_rx_sg_sub_jacobian( ssg, n_tf, TF_abs, ld_tf, J, rows );

    // Compute a damped pseudo inverse
    if( opts->s2min > 0 ) {
        aa_la_dzdpinv( n_x, n_q, opts->s2min, J, J_star );
    } else  {
        aa_la_dpinv( n_x, n_q, opts->k_dls, J, J_star );
    }

    // Compute nullspace projection matrix
    aa_la_np( n_x, n_q, J, J_star, N );

    // actual workspace velocity
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)n_x, (int)n_q,
                 1.0, J, (int)n_x,
                 dq_a, 1,
                 0.0, dx_a, 1 );

    // Optimization variables

    /*
     * Maximize: x .dot. c
     * Subject to:
     *   x_lower <=   x <= x_upper
     *   b_lower <= A*x <= b_upper
     */

    /* // A = [J_star*dt, N*dq_rn ] */
    double *A     = AA_MEM_REGION_LOCAL_NEW_N( double, n_q * (n_x+1) );
    double *b_min = AA_MEM_REGION_LOCAL_NEW_N( double, n_q );
    double *b_max = AA_MEM_REGION_LOCAL_NEW_N( double, n_q );
    double *x_min = AA_MEM_REGION_LOCAL_NEW_N( double, n_x + 1 );
    double *x_max = AA_MEM_REGION_LOCAL_NEW_N( double, n_x + 1 );
    double *c     = AA_MEM_REGION_LOCAL_NEW_N( double, n_x + 1 );
    double *opt_x = AA_MEM_REGION_LOCAL_NEW_N( double, n_x + 1 );
    double *dq_rn = AA_MEM_REGION_LOCAL_NEW_N( double, n_q );


    // Fill A
    AA_MEM_CPY(A, J_star, n_q*n_x );
    cblas_dscal( (int)(n_q*n_x), dt, A, 1 );
    for( size_t i = 0; i < n_q; i++) {
        dq_rn[i] = dq_a[i] - dq_r[i];
    }
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)n_q, (int)n_q,
                 1.0, N, (int)n_q,
                 dq_rn, 1,
                 0.0, A+n_q*n_x, 1 );

    // Fill C
    for( size_t i = 0; i < n_x; i ++ ) {
        c[i] = (dx_r[i] - dx_a[i])/dt;
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
        b_max[i] = -INFINITY;
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
                                 (q_min-q_a[i])/dt - ddq_max*dt/2 - dq_a[i] );
        }

        if( isfinite(ddq_min) ) {
            // acceleration constraint
            b_min[i] = fmax( b_min[i], ddq_min*dt );
            // position constraint
            if( isfinite(q_max) )
                b_max[i] = fmin( b_max[i],
                                 (q_max - q_a[i])/dt - ddq_min*dt/2 - dq_a[i] );
        }
    }

    struct aa_opt_cx *opt_cx = NULL;
    opt_cx =
        aa_opt_clp_gmcreate( n_q, n_x+1,
                             A, n_q,
                             b_min, b_max,
                             c,
                             x_min, x_max );

    aa_opt_set_direction( opt_cx, AA_OPT_MAXIMIZE );

    int r = 0;
    r = aa_opt_solve( opt_cx, n_x+1, opt_x );

    if( 0 == r ) {
        /* printf("x_min: "); aa_dump_vec(stdout, x_min, n_x + 1 ); */
        /* printf("x_max: "); aa_dump_vec(stdout, x_max, n_x + 1 ); */
        /* printf("opt_x: "); aa_dump_vec(stdout, opt_x, n_x + 1 ); */
        /* printf("opt_x: "); aa_dump_vec(stdout, opt_x, n_x + 1 ); */
        /* printf("opt_k: %f\n",opt_x[n_x]); */


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

    aa_opt_destroy(opt_cx);


    aa_mem_region_local_pop(J);


    //printf("--\n");
    return r;

}
