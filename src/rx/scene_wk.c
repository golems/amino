/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2017, Rice University
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
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_wk.h"

#include "amino/rx/scene_wk_internal.h"

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

static int
s_jstar( const const struct aa_rx_sg_sub *ssg,
         const struct aa_rx_wk_opts * opts,
         struct aa_dmat *TF_abs,
         size_t *prows, size_t *pcols,
         struct aa_dmat **pJ, struct aa_dmat **pJstar
    )
{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    size_t rows,cols;
    aa_rx_sg_sub_jacobian_size( ssg, &rows, &cols );
    *prows = rows;
    *pcols = cols;

    *pJ = aa_rx_sg_sub_jacobian_alloc( ssg, reg, TF_abs );
    *pJstar = aa_dmat_alloc(reg,rows,cols);


    // Compute a damped pseudo inverse
    // TODO: Try DGECON to avoid damping when possible without taking the SVD
    if( opts->s2min > 0 ) {
        aa_la_dzdpinv( rows, cols, opts->s2min, (*pJ)->data, (*pJstar)->data );
    } else  {
        aa_la_dpinv( rows, cols, opts->k_dls, (*pJ)->data, (*pJstar)->data );
    }

    return 0;
}

AA_API int
aa_rx_wk_dx2dq( const const struct aa_rx_sg_sub *ssg,
                const struct aa_rx_wk_opts * opts,
                size_t n_tf, const double*TF_abs, size_t ld_tf,
                size_t n_x, const double *dx,
                size_t n_q, double *dq )
{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);
    size_t rows,cols;

    struct aa_dmat tfd;
    aa_dmat_view( &tfd, 7, n_tf, (double*)TF_abs, ld_tf );

    struct aa_dmat *J, *J_star;
    s_jstar( ssg, opts,
             &tfd,
             &rows, &cols,
             &J, &J_star );


    assert(n_x == rows);
    assert(n_q == cols);

    struct aa_dvec dxd, dqd;
    aa_dvec_view( &dxd, n_x, (double*)dx, 1 );
    aa_dvec_view( &dqd, n_q, (double*)dq, 1 );

    aa_lb_dgemv( CblasNoTrans,
                 1.0, J_star, &dxd,
                 0.0, &dqd );

    aa_mem_region_pop(reg,ptrtop);

    return 0;
}


AA_API int
aa_rx_wk_dx2dq_np( const const struct aa_rx_sg_sub *ssg,
                   const struct aa_rx_wk_opts * opts,
                   size_t n_tf, const double *TF_abs, size_t ld_tf,
                   size_t n_x, const double *dx,
                   size_t n_q, const double *dq_r, double *dq )
{

    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);
    size_t rows,cols;

    struct aa_dmat tfd;
    aa_dmat_view( &tfd, 7, n_tf, (double*)TF_abs, ld_tf );

    struct aa_dmat *J, *J_star;
    s_jstar( ssg, opts,
             &tfd,
             &rows, &cols,
             &J, &J_star );

    assert(n_x == rows);
    assert(n_q == cols);


    aa_la_xlsnp( n_x, n_q, J->data, J_star->data, dx, dq_r, dq );


    aa_mem_region_pop(reg,ptrtop);
    return 0;
}

AA_API void
aa_rx_wk_dqcenter( const const struct aa_rx_sg_sub *ssg,
                   const struct aa_rx_wk_opts * opts,
                   size_t n_q, const double *q, double *dq_r )
{
    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(ssg);
    for( size_t i = 0; i < n_q; i ++ ) {
        double min=0 ,max=0;
        aa_rx_config_id config_id = aa_rx_sg_sub_config(ssg,i);
        if( aa_rx_sg_get_limit_pos(sg, config_id, &min, &max) ) {
            dq_r[i] =  0;
        } else {
            double c = (max + min)/2;
            double d = (c - q[i]) / (max - min);
            dq_r[i] =  opts->gain_np * d;// * fabs(d);
        }
    }
}

AA_API void
aa_rx_wk_dx_pos( const struct aa_rx_wk_opts * opts,
                 const double *E_act, const double *E_ref,
                 double *dx )
{
    double Ee[7];
    aa_tf_qutr_mulc( E_act, E_ref, Ee );
    aa_tf_qminimize(Ee + AA_TF_QUTR_Q );

    double w[6];
    aa_tf_qutr_lnv( Ee, w );

    double vel[6];
    aa_tf_qutr_twist2vel( E_act, w, vel );

    for(size_t i = 0; i < 3; i++ ) {
        dx[AA_TF_DX_W + i] -= opts->gain_angle * vel[AA_TF_DX_W + i];
        dx[AA_TF_DX_V + i] -= opts->gain_trans * vel[AA_TF_DX_V + i];
    }
}
