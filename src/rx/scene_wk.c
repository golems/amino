/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2017, Rice University
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
#include "amino/rx/scene_wk.h"

#include "amino/rx/scene_fk.h"

#include "amino/rx/scene_wk_internal.h"
#include "amino/mat_internal.h"

AA_API struct aa_rx_wk_opts *
aa_rx_wk_opts_create(void)
{
    struct aa_rx_wk_opts * r = AA_NEW(struct aa_rx_wk_opts);

    r->s2min = 1e-3;
    r->k_dls = 1e-3;
    r->gain_np = 1;
    r->gain_trans = 1;
    r->gain_angle = 1;
    r->lp_solver = AA_OPT_LP_SOLVER_DEFAULT;

    return r;
}

AA_API void
aa_rx_wk_opts_destroy( struct aa_rx_wk_opts * opts)
{
    free(opts);
}

AA_API int
aa_rx_wk_dx2dq( const struct aa_rx_sg_sub *ssg,
                const struct aa_rx_wk_opts * opts,
                const struct aa_rx_fk *fk,
                const struct aa_dvec *dx,
                struct aa_dvec *dq )
{
    size_t rows,cols;
    aa_rx_sg_sub_jacobian_size( ssg, &rows, &cols );
    aa_la_check_size(dx->len,   rows);
    aa_la_check_size(dq->len,   cols);

    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    struct aa_dmat *J = aa_dmat_alloc(reg,rows,cols);
    struct aa_dmat *J_star = aa_dmat_alloc(reg,cols,rows);
    aa_rx_wk_get_js( ssg, opts, fk, J, J_star );

    // workspace solution: dq = J^* dx
    aa_dmat_gemv( CblasNoTrans,
                  1.0, J_star, dx,
                  0.0, dq );

    aa_mem_region_pop(reg,ptrtop);

    return 0;
}

AA_API int
aa_rx_wk_dx2dq_np( const struct aa_rx_sg_sub *ssg,
                   const struct aa_rx_wk_opts * opts,
                   const struct aa_rx_fk *fk,
                   const struct aa_dvec *dx, const struct aa_dvec *dq_r,
                   struct aa_dvec *dq )
{
    size_t rows, cols;
    aa_rx_sg_sub_jacobian_size( ssg, &rows, &cols );
    aa_la_check_size(dx->len,   rows);
    aa_la_check_size(dq_r->len, cols);
    aa_la_check_size(dq->len,   cols);

    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    struct aa_dmat *J = aa_dmat_alloc(reg,rows,cols);
    struct aa_dmat *J_star = aa_dmat_alloc(reg,cols,rows);
    struct aa_dmat *N = aa_dmat_alloc(reg,cols,cols);
    aa_rx_wk_get_js( ssg, opts, fk, J, J_star );
    aa_rx_wk_get_n( ssg, opts, J, J_star, 1, N );

    // workspace solution: dq = J^* dx
    aa_dmat_gemv(CblasNoTrans,
                 1, J_star, dx,
                 0, dq );

    // Nullspace projection: dq = dq - N*dq_r
    aa_dmat_gemv(CblasNoTrans,
                 -1, N, dq_r,
                 1, dq );

    aa_mem_region_pop(reg,ptrtop);
    return 0;
}

AA_API void
aa_rx_wk_dqcenter( const struct aa_rx_sg_sub *ssg,
                   const struct aa_rx_wk_opts * opts,
                   const struct aa_dvec *q,
                   struct aa_dvec *dq_r )
{
    size_t n_q = aa_rx_sg_sub_config_count(ssg);
    aa_la_check_size(q->len, n_q);
    aa_la_check_size(dq_r->len, n_q);

    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(ssg);

    double *xq = q->data;
    double *xdq_r = dq_r->data;
    for( size_t i = 0; i < n_q; i ++, xq+=q->inc, xdq_r+=dq_r->inc ) {
        double min=0 ,max=0;
        aa_rx_config_id config_id = aa_rx_sg_sub_config(ssg,i);
        if( aa_rx_sg_get_limit_pos(sg, config_id, &min, &max) ) {
            *xdq_r =  0;
        } else {
            double c = (max + min)/2;
            double d = (c - *xq) / (max - min);
            *xdq_r =  opts->gain_np * d;// * fabs(d);
        }
    }
}

AA_API void
aa_rx_wk_dx_pos( const struct aa_rx_wk_opts * opts,
                 const double *E_act, const double *E_ref,
                 struct aa_dvec *dx )
{
    aa_la_check_size(dx->len, 6);

    double Ee[7];
    aa_tf_qutr_mulc( E_act, E_ref, Ee );
    aa_tf_qminimize(Ee + AA_TF_QUTR_Q );

    double w[6];
    aa_tf_qutr_lnv( Ee, w );

    double vel[6];
    aa_tf_qutr_twist2vel( E_act, w, vel );

    const size_t inc = dx->inc;
    double *dv = dx->data + AA_TF_DX_V*inc;
    double *dw = dx->data + AA_TF_DX_W*inc;

    for(size_t i = 0; i < 3; i++, dv+=inc, dw+=inc ) {
        *dw -= opts->gain_angle * vel[AA_TF_DX_W + i];
        *dv -= opts->gain_trans * vel[AA_TF_DX_V + i];
    }
}
