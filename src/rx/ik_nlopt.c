/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
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

#include <nlopt.h>

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/rxerr.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_kin_internal.h"


typedef int (*rfx_kin_duqu_fun) ( const void *cx, const double *q, double S[8],  double *J);



struct kin_solve_cx {
    size_t n;
    const struct aa_rx_ksol_opts *opts;
    const struct aa_rx_sg_sub *ssg;
    const double *E1;
    //const double *dq_dt;

    size_t iteration;

    struct aa_mem_region *reg;

    const double *q0_all;
    size_t n_all;
    double *TF_rel0;
    double *TF_abs0;
};

static void s_tf (const struct kin_solve_cx *cx, const double *q_s,
                  struct aa_dmat **pTF_abs,
                  double *E)
{
    const struct aa_rx_sg_sub *ssg = cx->ssg;
    const struct aa_rx_sg *sg = ssg->scenegraph;

    size_t n_q = aa_rx_sg_config_count(sg);
    size_t n_sq = aa_rx_sg_sub_config_count(ssg);

    /* Get the configuration */
    double q_all[n_q];
    AA_MEM_CPY(q_all, cx->q0_all, n_q);
    aa_rx_sg_sub_config_set( ssg,
                             n_sq, q_s,
                             n_q, q_all);

    /* Compute the transforms */
    {
        struct aa_dvec qv;
        aa_dvec_view( &qv, n_q, q_all, 1 );
        // TODO: faster TF update
        *pTF_abs = aa_rx_sg_get_tf_abs(sg, cx->reg, &qv );
    }


    /* aa_rx_sg_tf_update( sg, */
    /*                     n_q, cx->q0_all, q_all, */
    /*                     n_f, */
    /*                     cx->TF_rel0, 7, cx->TF_abs0, 7, */
    /*                     TF_rel, ld_rel, TF_abs, ld_abs ); */

    /* Fill the desired transform */
    aa_rx_frame_id id_ee = ( AA_RX_FRAME_NONE == cx->opts->frame )
        /* default to last frame in chain */
        ? aa_rx_sg_sub_frame_ee(ssg)
        /* use specified frame */
        : cx->opts->frame;

    double *E_ee = (*pTF_abs)->data + (int)(*pTF_abs)->ld*id_ee;
    AA_MEM_CPY(E, E_ee, 7);
}

static double s_serr( const double E_act[7], const double E_ref[7] )
{

    double E_rel[7];
    aa_tf_qutr_cmul( E_act, E_ref, E_rel );
    aa_tf_qminimize(E_rel+AA_TF_QUTR_Q);

    double w[6];
    aa_tf_qutr_lnv(E_rel,w);
    struct aa_dvec wv = AA_DVEC_INIT(6,w,1);
    return aa_lb_dnrm2(&wv);
}



static double
s_obj(unsigned n, const double *q, double *dq, void *vcx)
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);


    size_t n_qs, n_x;
    aa_rx_sg_sub_jacobian_size(cx->ssg,&n_x,&n_qs);

    //printf("q:\t");
    //aa_dump_vec(stdout, q, n_qs);

    double  E_act[7];
    struct aa_dmat *TF_abs;
    s_tf( cx, q, &TF_abs, E_act );

    if( dq ) {
        struct aa_dvec *w_e = aa_dvec_alloc(cx->reg,n_x);
        aa_dvec_zero(w_e);
        aa_rx_wk_dx_pos( &cx->opts->wk_opts, E_act, cx->E1, w_e );

        struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1);
        aa_dvec_view(&v_dq, n_qs, dq, 1);

        if( cx->opts->q_ref ) {
            double dqnull[n_qs];
            for( size_t i = 0; i < n_qs; i ++ )  {
                dqnull[i] = - cx->opts->dq_dt[i] * ( q[i] - cx->opts->q_ref[i] );
            }
            struct aa_dvec v_dqnull;
            aa_dvec_view(&v_dqnull, n_qs, dqnull, 1);
            aa_rx_wk_dx2dq_np( cx->ssg, &cx->opts->wk_opts,
                               TF_abs, w_e, &v_dqnull, &v_dq );
        } else {
            aa_rx_wk_dx2dq( cx->ssg, &cx->opts->wk_opts,
                            TF_abs,
                            w_e, &v_dq );
        }

        aa_lb_dscal(-1,&v_dq);

        //printf("dq:\t");
        //aa_dump_vec(stdout, dq, n_qs);
    }

    double x = s_serr( E_act, cx->E1 );
    //printf("err:\t%f\t%f\n", theta_err, x_err);

    aa_mem_region_pop(cx->reg, ptrtop);
    return x;

}



AA_API int
aa_rx_ik_nlopt(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ksol_opts *opts,
               const struct aa_dmat *TF,
               struct aa_dvec *q )
{

    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    size_t n_x, n_qs;
    aa_rx_sg_sub_jacobian_size(ssg,&n_x,&n_qs);
    assert( n_qs == q->len );

    if( 1 != TF->cols
        || 6 != n_x ) {
        /* Only chains for now */
        return AA_RX_INVALID_PARAMETER;
    }

    // seed
    const double *q_start_all = opts->q_all_seed;
    size_t n_q_all = opts->n_all_seed;

    if( 0 == n_q_all || NULL == q_start_all ) {
        return AA_RX_INVALID_PARAMETER;
    }

    assert( aa_rx_sg_sub_all_config_count(ssg) == n_q_all );

    const struct aa_rx_sg *sg = ssg->scenegraph;
    size_t n_f = aa_rx_sg_frame_count(sg);


    struct aa_dvec *q0_sub = aa_dvec_alloc(reg, n_qs);
    struct aa_dvec *q_sub = aa_dvec_alloc(reg, n_qs);
    //double q0_sub[n_qs];

    aa_rx_sg_config_get( ssg->scenegraph, n_q_all, n_qs, aa_rx_sg_sub_configs(ssg),
                         q_start_all, q0_sub->data );
    aa_lb_dcopy(q0_sub, q_sub);

    /* printf("jacobian ik\n"); */
    /* printf("q ref:  %s\n", opts->q_ref ? "yes" : "no"); */
    /* if( opts->q_ref ) { */
    /*     aa_dump_vec( stdout, opts->q_ref, ssg->config_count ); */
    //}

    struct kin_solve_cx cx;
    cx.n = n_qs;
    cx.opts = opts;
    cx.E1 = TF->data;
    cx.ssg = ssg;
    cx.iteration = 0;


    cx.q0_all = q_start_all;
    cx.n_all = n_q_all;

    cx.reg = reg;
    cx.TF_rel0 = AA_MEM_REGION_NEW_N(reg, double, 7*n_f);
    cx.TF_abs0 = AA_MEM_REGION_NEW_N(reg, double, 7*n_f);

    aa_rx_sg_tf( ssg->scenegraph,
                 cx.n_all, cx.q0_all,
                 n_f,
                 cx.TF_rel0, 7,
                 cx.TF_abs0, 7 );


    //nlopt_algorithm alg = NLOPT_LN_BOBYQA;
    nlopt_algorithm alg = NLOPT_LD_SLSQP;
    nlopt_opt opt = nlopt_create(alg, (unsigned)n_qs); /* algorithm and dimensionality */
    //nlopt_set_lower_bounds(opt, lb);
    nlopt_set_min_objective(opt, s_obj, &cx);
    nlopt_set_xtol_rel(opt, 1e-4);
    { // bounds
        double lb[n_qs], ub[n_qs];
        for( size_t i = 0; i < n_qs; i ++ ) {
            aa_rx_config_id id = aa_rx_sg_sub_config(ssg,i);
            if( aa_rx_sg_get_limit_pos(sg, id, lb+i, ub+i) )
            {
                lb[i] = -DBL_MAX;
                ub[i] = DBL_MAX;
            }

        }
    }

    int r = -1;
    double minf;
    if (nlopt_optimize(opt, q_sub->data, &minf) < 0) {
        printf("nlopt failed!\n");
    } else {
        r = 0;
        aa_lb_dcopy( q_sub, q );
        printf("found minimum\n");
    }

    nlopt_destroy(opt);

    //aa_mem_region_pop(cx.reg, cx.TF_rel0);

    aa_mem_region_pop(reg,ptrtop);

    if( r ) {
        return AA_RX_NO_SOLUTION | AA_RX_NO_IK;
    } else {
        return 0;
    }
}
