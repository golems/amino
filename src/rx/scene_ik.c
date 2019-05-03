/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/rxerr.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_kin_internal.h"



static int
s_ik_jpinv( struct kin_solve_cx *cx,
            struct aa_dvec *q );


static int
s_ik_nlopt( struct kin_solve_cx *cx,
            double (*obj)(unsigned n, const double *q, double *dq, void *vcx),
            struct aa_dvec *q );

static double
s_nlobj_jpinv(unsigned n, const double *q, double *dq, void *vcx);

static double
s_nlobj_dq_fd(unsigned n, const double *q, double *dq, void *vcx);

static double
s_nlobj_dq_an(unsigned n, const double *q, double *dq, void *vcx);

AA_API struct aa_rx_ik_cx *
aa_rx_ik_cx_create(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ksol_opts *opts )
{
    struct aa_rx_ik_cx *cx = AA_NEW0(struct aa_rx_ik_cx);
    cx->ssg = ssg;
    cx->opts = opts;

    cx->q_start = aa_dvec_malloc( aa_rx_sg_sub_all_config_count(ssg) );
    aa_dvec_zero(cx->q_start);

    cx->q_seed = aa_dvec_malloc( aa_rx_sg_sub_config_count(ssg) );
    aa_rx_ik_set_seed_center(cx);

    return cx;
}

AA_API void
aa_rx_ik_cx_destroy( struct aa_rx_ik_cx *cx )
{
    free(cx->q_start);
    free(cx->q_seed);
    free(cx);
}

AA_API struct aa_dvec *
aa_rx_ik_get_start( const struct aa_rx_ik_cx *context )
{
    return context->q_start;
}

AA_API struct aa_dvec *
aa_rx_ik_get_seed( const struct aa_rx_ik_cx *context )
{
    return context->q_seed;
}

AA_API void
aa_rx_ik_set_start( const struct aa_rx_ik_cx *context, const struct aa_dvec *q_start )
{
    aa_lb_dcopy( q_start, context->q_start );
}

AA_API void
aa_rx_ik_set_seed( const struct aa_rx_ik_cx *context, const struct aa_dvec *q_seed )
{
    aa_lb_dcopy( q_seed, context->q_seed );
}

AA_API void
aa_rx_ik_set_seed_center( const struct aa_rx_ik_cx *context )
{
    size_t n_s = context->q_seed->len;
    double q_s[n_s];
    aa_rx_sg_sub_center_configs( context->ssg, n_s, q_s );
    struct aa_dvec v = AA_DVEC_INIT(n_s,q_s,1);
    aa_rx_ik_set_seed( context, &v );
}


static struct kin_solve_cx *
s_kin_solve_cx_alloc( const struct aa_rx_ik_cx *ik_cx,
                      struct aa_mem_region *reg )
{
    struct kin_solve_cx *cx = AA_MEM_REGION_NEW(reg, struct kin_solve_cx);
    cx->reg = reg;

    cx->ssg = ik_cx->ssg;
    cx->opts = ik_cx->opts;

    cx->iteration = 0;

    const struct aa_rx_sg *sg = cx->ssg->scenegraph;
    size_t n_f = aa_rx_sg_frame_count(sg);
    cx->TF_rel0 = AA_MEM_REGION_NEW_N(cx->reg, double, AA_RX_TF_LEN*n_f);
    cx->TF_abs0 = AA_MEM_REGION_NEW_N(cx->reg, double, AA_RX_TF_LEN*n_f);

    cx->q_all = aa_dvec_dup(reg, ik_cx->q_start );
    cx->q_sub = aa_dvec_dup(reg, ik_cx->q_seed );
    aa_rx_sg_sub_config_scatter( cx->ssg, cx->q_sub, cx->q_all );

    return cx;
}


AA_API int
aa_rx_ik_solve( const struct aa_rx_ik_cx *context,
                const struct aa_dmat *TF,
                struct aa_dvec *q )
{
    if( aa_rx_sg_sub_config_count(context->ssg) != q->len
        || AA_RX_TF_LEN != TF->rows
        )
    {
        return AA_RX_INVALID_PARAMETER;
    }


    /* Init Context */
    struct kin_solve_cx *kcx = s_kin_solve_cx_alloc( context,
                                                     aa_mem_region_local_get() );

    if( 1 != TF->cols )
    {   /* only chains */
        return AA_RX_INVALID_PARAMETER;
    }
    kcx->E1 = TF->data; // TODO: be more general

    aa_rx_sg_tf( kcx->ssg->scenegraph,
                 kcx->q_all->len, kcx->q_all->data,
                 aa_rx_sg_frame_count(kcx->ssg->scenegraph),
                 kcx->TF_rel0, AA_RX_TF_LEN,
                 kcx->TF_abs0, AA_RX_TF_LEN );

    /* Dispatch and solve */
    int r = AA_RX_NO_IK;
    switch(context->opts->ik_algo) {
    case AA_RX_IK_JPINV:
        assert(q->inc == 1);
        r = s_ik_jpinv( kcx, q );
        goto END;
    case AA_RX_IK_LMA:
        break;
#ifdef HAVE_NLOPT
    case AA_RX_IK_SQP_JPINV:
        r = s_ik_nlopt( kcx, s_nlobj_jpinv, q );
        goto END;
    case AA_RX_IK_SQP_DQ_FD:
        r = s_ik_nlopt( kcx, s_nlobj_dq_fd, q );
        goto END;
    case AA_RX_IK_SQP_DQ_AN:
        r = s_ik_nlopt( kcx, s_nlobj_dq_an, q );
        goto END;
#else /*HAVE_NLOPT*/
        /* Can't implement these without NLOPT */
    case AA_RX_IK_SQP_JPINV:
    case AA_RX_IK_SQP_DQ_FD:
    case AA_RX_IK_SQP_DQ_AN:
        fprintf(stderr, "Error: Need NLOPT for SQP IK algorithms");
        break;
#endif /*HAVE_NLOPT*/
    }


    fprintf(stderr, "Error: unimplemented IK algorithm");
    r = AA_RX_NO_SOLUTION | AA_RX_NO_IK | AA_RX_INVALID_PARAMETER;

END:
    aa_mem_region_pop(kcx->reg, kcx);
    return r;
}






/* static void s_tf0 (const struct kin_solve_cx *cx, const double *q_s, */
/*                    struct aa_dmat **pTF_abs, */
/*                    double *E) */

static void s_tf (const struct kin_solve_cx *cx, const double *q_s,
                  struct aa_dmat **pTF_abs,
                  double *E)
{
    const struct aa_rx_sg_sub *ssg = cx->ssg;
    const struct aa_rx_sg *sg = ssg->scenegraph;

    //size_t n_q = aa_rx_sg_config_count(sg);
    size_t n_sq = aa_rx_sg_sub_config_count(ssg);


    struct aa_dvec vqs = AA_DVEC_INIT(n_sq, (double*)q_s, 1);
    aa_rx_sg_sub_config_scatter( ssg, &vqs, cx->q_all );

    /* Compute the transforms */
    // TODO: faster TF update
    *pTF_abs = aa_rx_sg_get_tf_abs( sg, cx->reg, cx->q_all );


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
    AA_MEM_CPY(E, E_ee, AA_RX_TF_LEN);
}


/* AA_API int */
/* aa_rx_ik_check( const struct aa_rx_ik_cx *cx, */
/*                 const struct aa_dmat *TF, */
/*                 struct aa_dvec *q ) */
/* { */
/*     void *ptrtop = aa_mem_region_ptr(cx->reg); */

/*     double theta, x, E_sol[7]; */
/*     s_tf(cx, cx->q_sub->data, &TF_abs, E_sol); */
/*     s_err2(E_sol, cx->E1, &theta, &x); */

/*     int r = ( theta < cx->opts->tol_angle && x < cx->opts->tol_trans ) ? */
/*         0 : AA_RX_NO_SOLUTION | AA_RX_NO_IK; */

/*     aa_mem_region_pop(cx->reg, ptrtop); */

/*     return r; */
/* } */


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

static void s_err2( const double E_act[7], const double E_ref[7],
                    double *theta, double *x ) {

    double E_rel[7];
    aa_tf_qutr_cmul( E_act, E_ref, E_rel );

    double *q_rel = E_rel+AA_TF_QUTR_Q;
    double *xe = E_rel+AA_TF_QUTR_T;
    aa_tf_qminimize(q_rel);

    // quaternion angle
    *theta = aa_tf_qangle(q_rel);

    // translation
    *x = aa_tf_vnorm(xe);
}


static void s_ksol_jpinv( const struct kin_solve_cx *cx,
                          const double *q,
                          const struct aa_dmat *TF_abs,
                          const double E_act[7],
                          struct aa_dvec *dq )
{
    size_t n_x, n_qs;
    aa_rx_sg_sub_jacobian_size(cx->ssg,&n_x,&n_qs);
    struct aa_dvec *w_e = aa_dvec_alloc(cx->reg,n_x);
    aa_dvec_zero(w_e);
    aa_rx_wk_dx_pos( &cx->opts->wk_opts, E_act, cx->E1, w_e );

    if( cx->opts->q_ref ) {
        struct aa_dvec *v_dqnull = aa_dvec_alloc(cx->reg,n_qs);
        double *dqnull = v_dqnull->data;
        for( size_t i = 0; i < n_qs; i ++ )  {
            dqnull[i] = - cx->opts->dq_dt[i] * ( q[i] - cx->opts->q_ref[i] );
        }
        aa_rx_wk_dx2dq_np( cx->ssg, &cx->opts->wk_opts,
                           TF_abs, w_e, v_dqnull, dq );
    } else {
        aa_rx_wk_dx2dq( cx->ssg, &cx->opts->wk_opts,
                        TF_abs,
                        w_e, dq );
    }
}




#include "ik_jacobian.c"

#ifdef HAVE_NLOPT
#include "ik_nlopt.c"
#endif /*HAVE_NLOPT*/