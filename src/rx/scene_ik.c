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
#include "amino/rx/scene_fk.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_ik_internal.h"



static int
s_ik_jpinv( struct kin_solve_cx *cx,
            struct aa_dvec *q );


static int
s_ik_nlopt( struct kin_solve_cx *cx,
            struct aa_dvec *q );

AA_API struct aa_rx_ik_cx *
aa_rx_ik_cx_create(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ik_parm *opts )
{
    struct aa_rx_ik_cx *cx = AA_NEW0(struct aa_rx_ik_cx);
    cx->ssg = ssg;
    cx->opts = opts;


    cx->frame = ( AA_RX_FRAME_NONE == opts->frame )
        /* default to last frame in chain */
        ? aa_rx_sg_sub_frame_ee(ssg)
        /* use specified frame */
        : opts->frame;

    cx->q_start = aa_dvec_malloc( aa_rx_sg_sub_all_config_count(ssg) );
    aa_dvec_zero(cx->q_start);

    cx->q_seed = aa_dvec_malloc( aa_rx_sg_sub_config_count(ssg) );

    cx->TF = aa_dmat_malloc( AA_RX_TF_LEN, aa_rx_sg_sub_all_frame_count(ssg) );
    cx->fk = aa_rx_fk_malloc(aa_rx_sg_sub_sg(ssg));

    aa_rx_ik_set_seed_center(cx);
    aa_rx_sg_fill_tf_abs( aa_rx_sg_sub_sg(cx->ssg), cx->q_start, cx->TF );
    aa_rx_fk_all( cx->fk, cx->q_start );


    return cx;
}

AA_API void
aa_rx_ik_cx_destroy( struct aa_rx_ik_cx *cx )
{
    aa_rx_fk_destroy(cx->fk);
    free(cx->q_start);
    free(cx->q_seed);
    free(cx->TF);
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
aa_rx_ik_set_start( struct aa_rx_ik_cx *context, const struct aa_dvec *q_start )
{
    aa_dvec_copy( q_start, context->q_start );
    aa_rx_sg_fill_tf_abs( aa_rx_sg_sub_sg(context->ssg), q_start, context->TF );
    aa_rx_fk_all( context->fk, q_start );
}

AA_API void
aa_rx_ik_set_seed( struct aa_rx_ik_cx *context, const struct aa_dvec *q_seed )
{
    aa_dvec_copy( q_seed, context->q_seed );
}


AA_API void
aa_rx_ik_set_seed_center( struct aa_rx_ik_cx *context )
{
    size_t n_s = context->q_seed->len;
    double q_s[n_s];
    aa_rx_sg_sub_center_configs( context->ssg, n_s, q_s );
    struct aa_dvec v = AA_DVEC_INIT(n_s,q_s,1);
    aa_rx_ik_set_seed( context, &v );
}

AA_API void
aa_rx_ik_set_seed_rand( struct aa_rx_ik_cx *context )
{
    aa_rx_sg_sub_rand_config( context->ssg, context->q_seed );
}

AA_API void
aa_rx_ik_set_frame_name( struct aa_rx_ik_cx *context, const char *name )
{
    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(context->ssg);
    aa_rx_frame_id id = aa_rx_sg_frame_id(sg, name);
    aa_rx_ik_set_frame_id(context, id);
}

AA_API void
aa_rx_ik_set_frame_id( struct aa_rx_ik_cx *context, aa_rx_frame_id id )
{
    context->frame = id;
}

AA_API void
aa_rx_ik_set_restart_time( struct aa_rx_ik_cx *context, double t )
{
    context->restart_time = t;
}


AA_API double
aa_rx_ik_get_restart_time( struct aa_rx_ik_cx *context )
{
    return context->restart_time;
}

static struct kin_solve_cx *
s_kin_solve_cx_alloc( const struct aa_rx_ik_cx *ik_cx,
                      struct aa_mem_region *reg )
{
    struct kin_solve_cx *cx = AA_MEM_REGION_NEW(reg, struct kin_solve_cx);
    cx->reg = reg;

    cx->ik_cx = ik_cx;

    cx->ssg = ik_cx->ssg;
    cx->opts = ik_cx->opts;

    cx->frame = ik_cx->frame;

    cx->iteration = 0;


    cx->fk = aa_rx_fk_alloc(aa_rx_sg_sub_sg(cx->ssg), reg);
    aa_rx_fk_cpy(cx->fk, ik_cx->fk);

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
    kcx->TF_ref = TF;

    /* aa_rx_sg_tf( kcx->ssg->scenegraph, */
    /*              kcx->q_all->len, kcx->q_all->data, */
    /*              aa_rx_sg_frame_count(kcx->ssg->scenegraph), */
    /*              kcx->TF_rel0, AA_RX_TF_LEN, */
    /*              kcx->TF_abs0, AA_RX_TF_LEN ); */


    /* Dispatch and solve */
    struct timespec t0;
    if(context->restart_time > 0) {
        t0 = aa_tm_now();
    }
    int r = AA_RX_NO_IK;
    int restart = 0;
    do  {
        if(context->opts->debug) {
            fprintf(stderr, "AMINO IK SEED: ");
            aa_dump_vec(stderr, q->data, q->len);
        }
        restart = 0;
        switch(context->opts->ik_algo) {
        case AA_RX_IK_JPINV:
            assert(q->inc == 1);
            r = s_ik_jpinv( kcx, q );
            break;
        case AA_RX_IK_LMA:
            goto ERR;
        case AA_RX_IK_SQP:
#ifdef HAVE_NLOPT
            r = s_ik_nlopt( kcx, q );
            break;
#else /*HAVE_NLOPT*/
            /* Can't implement these without NLOPT */
            fprintf(stderr, "Error: Need NLOPT for SQP IK algorithms");
            goto ERR;
#endif /*HAVE_NLOPT*/
        }

        if(r && (context->restart_time > 0) ) {
            // try restart
            double dt = aa_tm_timespec2sec( aa_tm_sub(aa_tm_now(),t0) );
            if( dt < context->restart_time ) {
                // random reseed
                //printf("restart\n");
                aa_rx_sg_sub_rand_config( context->ssg, kcx->q_sub );
                restart = 1;
            }
        }
    } while (restart);

    aa_mem_region_pop(kcx->reg, kcx);
    return r;

ERR:
    aa_mem_region_pop(kcx->reg, kcx);
    fprintf(stderr, "Error: unimplemented IK algorithm");
    return AA_RX_NO_SOLUTION | AA_RX_NO_IK | AA_RX_INVALID_PARAMETER;
}


/* static struct aa_dmat * */
/* s_tf0 ( struct aa_mem_region *reg, */
/*         const struct aa_rx_sg_sub *ssg, aa_rx_frame_id frame, */
/*         const struct aa_dvec *q_all, double *E ) */
/* { */
/*     const struct aa_rx_sg *sg = ssg->scenegraph; */
/*     struct aa_dmat *TF = aa_rx_sg_get_tf_abs( sg, reg, q_all ); */

/*     double *E_ee = &AA_DMAT_REF(TF,0,(size_t)frame); */
/*     AA_MEM_CPY(E, E_ee, AA_RX_TF_LEN); */

/*     return TF; */
/* } */

/* static void s_tf (const struct kin_solve_cx *cx, */
/*                   const struct aa_dvec *qs, */
/*                   struct aa_dmat **pTF_abs, */
/*                   double *E) */
/* { */
/*     const struct aa_rx_sg_sub *ssg = cx->ssg; */
/*     aa_rx_sg_sub_config_scatter( ssg, qs, cx->q_all ); */
/*     *pTF_abs = s_tf0( cx->reg, ssg, cx->frame, cx->q_all, E ); */

/*     /\* aa_rx_sg_tf_update( sg, *\/ */
/*     /\*                     n_q, cx->q0_all, q_all, *\/ */
/*     /\*                     n_f, *\/ */
/*     /\*                     cx->TF_rel0, 7, cx->TF_abs0, 7, *\/ */
/*     /\*                     TF_rel, ld_rel, TF_abs, ld_abs ); *\/ */

/* } */

static double s_serr( const double E_act[7], const double E_ref[7] )
{

    double E_rel[7];
    aa_tf_qutr_cmul( E_act, E_ref, E_rel );
    aa_tf_qminimize(E_rel+AA_TF_QUTR_Q);

    double w[6];
    aa_tf_qutr_lnv(E_rel,w);
    struct aa_dvec wv = AA_DVEC_INIT(6,w,1);
    return aa_dvec_nrm2(&wv);
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



static int
s_check( const struct aa_rx_ik_cx *cx,
         const struct aa_dmat *TF_ref,
         const double *E )
{
    double theta, x;
    s_err2(E, TF_ref->data, &theta, &x);

    if( cx->opts->debug ) {
        fprintf(stderr,
                "AMINO IK ERR MAX: %f, %f\n"
                "AMINO IK ERR ACT: %f, %f\n",
                cx->opts->tol_angle,
                cx->opts->tol_trans,
                theta, x
            );
    }

    int r = ( theta < cx->opts->tol_angle && x < cx->opts->tol_trans ) ?
        0 : AA_RX_NO_SOLUTION | AA_RX_NO_IK;


    return r;
}

AA_API int
aa_rx_ik_check( const struct aa_rx_ik_cx *cx,
                const struct aa_dmat *TF,
                struct aa_dvec *q_sub )
{
    struct aa_mem_region *reg = aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    struct aa_dvec *q_all = aa_dvec_dup(reg,cx->q_start);
    aa_rx_sg_sub_config_scatter( cx->ssg, q_sub, q_all );

    struct aa_rx_fk *fk = aa_rx_fk_alloc(aa_rx_sg_sub_sg(cx->ssg), reg);
    aa_rx_fk_sub(fk, cx->ssg, q_sub);
    double *E_act = aa_rx_fk_ref(fk, cx->frame);

    int r = s_check(cx,TF, E_act);
    aa_mem_region_pop(reg, ptrtop);
    return r;
}


static void s_ksol_jpinv( const struct kin_solve_cx *cx,
                          const double *q,
                          struct aa_dvec *dq )
{
    size_t n_x, n_qs;
    aa_rx_sg_sub_jacobian_size(cx->ssg,&n_x,&n_qs);
    struct aa_dvec *w_e = aa_dvec_alloc(cx->reg,n_x);
    aa_dvec_zero(w_e);

    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);
    aa_rx_wk_dx_pos( &cx->opts->wk_opts, E_act, cx->TF_ref->data, w_e );


    if( cx->opts->q_ref ) {
        struct aa_dvec *v_dqnull = aa_dvec_alloc(cx->reg,n_qs);
        double *dqnull = v_dqnull->data;
        for( size_t i = 0; i < n_qs; i ++ )  {
            dqnull[i] = - cx->opts->dq_dt[i] * ( q[i] - cx->opts->q_ref[i] );
        }
        aa_rx_wk_dx2dq_np( cx->ssg, &cx->opts->wk_opts,
                           cx->fk, w_e, v_dqnull, dq );
    } else {
        aa_rx_wk_dx2dq( cx->ssg, &cx->opts->wk_opts,
                        cx->fk,
                        w_e, dq );
    }
}




#include "ik_jacobian.c"



AA_API void
aa_rx_ik_parm_set_obj( struct aa_rx_ik_parm *ko,
                       aa_rx_ik_opt_fun *fun )
{
    ko->obj_fun = fun;
}


AA_API void
aa_rx_ik_parm_set_eqct( struct aa_rx_ik_parm *opts,
                        aa_rx_ik_opt_fun *fun,
                        double tol)
{
    opts->eqct_fun = fun;
    opts->eqct_tol = tol;
}

#ifdef HAVE_NLOPT
#include "ik_nlopt.c"
#endif /*HAVE_NLOPT*/
