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
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_kin_internal.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/rx_ct.h"

#include "rx_ct_internal.h"

AA_API struct aa_rx_ct_wk_opts *
aa_rx_ct_wk_opts_create(void)
{
    struct aa_rx_ct_wk_opts * r = AA_NEW(struct aa_rx_ct_wk_opts);

    r->s2min = 1e-3;
    r->k_dls = 1e-3;
    r->k_np = 1;
    r->k_pos = 1;

    return r;
}

AA_API void
aa_rx_ct_wk_opts_destroy( struct aa_rx_ct_wk_opts * opts)
{
    free(opts);
}

AA_API int
aa_rx_ct_wk_dx2dq( const const struct aa_rx_sg_sub *ssg,
                   const struct aa_rx_ct_wk_opts * opts,
                   size_t n_tf, const double *TF_abs, size_t ld_tf,
                   size_t n_x, const double *dx,
                   size_t n_q, double *dq )
{

    size_t rows,cols;
    aa_rx_sg_sub_jacobian_size( ssg, &rows, &cols );
    assert(n_x == rows);
    assert(n_q == cols);

    double *J = AA_MEM_REGION_LOCAL_NEW_N(double, rows*cols);
    double *J_star = AA_MEM_REGION_LOCAL_NEW_N(double, rows*cols);

    aa_rx_sg_sub_jacobian( ssg, n_tf, TF_abs, ld_tf, J, rows );

    // Compute a damped pseudo inverse
    if( opts->s2min > 0 ) {
        aa_la_dzdpinv( n_x, n_q, opts->s2min, J, J_star );
    } else  {
        aa_la_dpinv( n_x, n_q, opts->k_dls, J, J_star );
    }

    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)n_q, (int)n_x,
                 1.0, J_star, (int)n_q,
                 dx, 1,
                 0.0, dq, 1 );

    aa_mem_region_local_pop(J);

    return 0;
}


AA_API int
aa_rx_ct_wk_dx2dq_np( const const struct aa_rx_sg_sub *ssg,
                      const struct aa_rx_ct_wk_opts * opts,
                      size_t n_tf, const double *TF_abs, size_t ld_tf,
                      size_t n_x, const double *dx,
                      size_t n_q, const double *dq_r, double *dq )
{

    size_t rows,cols;
    aa_rx_sg_sub_jacobian_size( ssg, &rows, &cols );
    assert(n_x == rows);
    assert(n_q == cols);

    double *J = AA_MEM_REGION_LOCAL_NEW_N(double, rows*cols);
    double *J_star = AA_MEM_REGION_LOCAL_NEW_N(double, rows*cols);

    aa_rx_sg_sub_jacobian( ssg, n_tf, TF_abs, ld_tf, J, rows );

    // Compute a damped pseudo inverse
    if( opts->s2min > 0 ) {
        aa_la_dzdpinv( n_x, n_q, opts->s2min, J, J_star );
    } else  {
        aa_la_dpinv( n_x, n_q, opts->k_dls, J, J_star );
    }

    aa_la_xlsnp( n_x, n_q, J, J_star, dx, dq_r, dq );

    aa_mem_region_local_pop(J);

    return 0;
}

AA_API void
aa_rx_ct_wk_dqcenter( const const struct aa_rx_sg_sub *ssg,
                      const struct aa_rx_ct_wk_opts * opts,
                      size_t n_q, const double *q, double *dq_r )
{
    aa_rx_sg_sub_center_configs(ssg, n_q, dq_r);
    for( size_t i = 0; i < n_q; i ++ ) {
        double x = (q[i] - dq_r[i]);
        //dq_r[i] = - opts->k_np * x * fabs(x);
        dq_r[i] = - opts->k_np * x;
    }
}


AA_API void
aa_rx_ct_wk_dx_pos( const struct aa_rx_ct_wk_opts * opts,
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

    for(size_t i = 0; i < 6; i++ ) {
        dx[i] -= opts->k_pos * vel[i];
    }
}



AA_API struct aa_ct_pt_list *
aa_rx_ct_pt_list( struct aa_mem_region *region,
                     size_t n_q, size_t n_path, double *path )
{
    struct aa_ct_pt_list *list = aa_ct_pt_list_create(region);

    /* TODO: memory ownership? */
    for( size_t i = 0; i < n_q*n_path; i+=n_q ) {
        struct aa_ct_state *s = AA_MEM_REGION_NEW(region, struct aa_ct_state);
        AA_MEM_ZERO(s,1);
        s->n_q = n_q;
        s->q = &(path[i]);
        aa_ct_pt_list_add( list, s );
    }

    return list;
}

static void set_inf( double *min, double *max )
{
    *min = -INFINITY;
    *max = INFINITY;
}



AA_API struct aa_ct_limit *
aa_rx_ct_limits( struct aa_mem_region *region, const struct aa_rx_sg *sg )
{
    struct aa_ct_limit *limit = AA_MEM_REGION_NEW(region, struct aa_ct_limit);
    size_t n_q = aa_rx_sg_config_count(sg);

    struct aa_ct_state *smin = limit->min = aa_ct_state_alloc(region, n_q, 0);
    struct aa_ct_state *smax = limit->max = aa_ct_state_alloc(region, n_q, 0);

    for( aa_rx_config_id i = 0; i < (aa_rx_config_id)n_q; i++ )
    {
        if( aa_rx_sg_get_limit_pos(sg, i, &(smin->q[i]), &(smax->q[i])) )
            set_inf( &smin->q[i], &smax->q[i] );

        if( aa_rx_sg_get_limit_vel(sg, i, &(smin->dq[i]), &(smax->dq[i])) )
            set_inf( &smin->dq[i], &smax->dq[i] );

        if( aa_rx_sg_get_limit_acc(sg, i, &(smin->ddq[i]), &(smax->ddq[i])) )
            set_inf( &smin->ddq[i], &smax->ddq[i] );

        if( aa_rx_sg_get_limit_eff(sg, i, &(smin->eff[i]), &(smax->eff[i])) )
            set_inf( &smin->eff[i], &smax->eff[i] );
    }

    return limit;
}




struct path_cx {
    const struct aa_rx_ksol_opts *opts;
    const struct aa_rx_sg_sub *ssg;
    const struct aa_rx_sg *sg;
    size_t n_q_all;
    size_t n_q_sub;
    size_t n_f_all;
    aa_rx_frame_id frame;

    struct aa_ct_limit *limits;

    /* initial state */
    const double *q_start_all;
    const double *TF_rel0;
    const double *TF_abs0;

    /* work area */
    struct aa_ct_state *state;
    double *J;
    double *TF_rel;
    double *TF_abs;
    double *q_all;

    struct aa_ct_seg_list *segs;
    struct aa_mem_region *region;


};


static void path_sys( const void *vcx,
                      double t, const double *AA_RESTRICT q,
                      double *AA_RESTRICT dq )
{
    struct path_cx *cx = (struct path_cx*)vcx;
    int r = aa_ct_seg_list_eval(cx->segs, cx->state, t);


    aa_rx_sg_sub_config_set( cx->ssg,
                             cx->n_q_sub, q,
                             cx->n_q_all, cx->q_all );

    aa_rx_sg_tf_update( cx->sg,
                        cx->n_q_all, cx->q_start_all, cx->q_all,
                        cx->n_f_all,
                        cx->TF_rel0, 7, cx->TF_abs0, 7,
                        cx->TF_rel, 7, cx->TF_abs, 7 );


    double *E_act = cx->TF_abs + 7*cx->frame;

    aa_rx_sg_sub_jacobian( cx->ssg, cx->n_f_all, cx->TF_abs, 7,
                           cx->J, 6 );


    aa_rx_ik_jac_x2dq ( cx->opts,
                        cx->n_q_sub, q, E_act,
                        cx->state->X,
                        cx->state->dX,
                        cx->J, dq );
    //aa_dump_vec( stdout, dq, cx->n_q_sub);
}

static int path_check( void *vcx, double t, double *AA_RESTRICT q, double *AA_RESTRICT dq )
{
    struct path_cx *cx = (struct path_cx*)vcx;
    (void) q;
    (void) dq;
    return t >= aa_ct_seg_list_duration(cx->segs);
}

AA_API int
aa_rx_ct_tjx_path( struct aa_mem_region *region,
                   const struct aa_rx_ksol_opts *opts,
                   const struct aa_rx_sg_sub *ssg,
                   struct aa_ct_seg_list *segs,
                   size_t n_q_all, const double *q_start_all,
                   size_t *n_points, double **path )

{
    /* Only chains for now */
    if( 0 == n_q_all || NULL == q_start_all ) {
        return AA_RX_INVALID_PARAMETER;
    }

    assert( aa_rx_sg_sub_all_config_count(ssg) == n_q_all );

    struct aa_mem_region *local_region = aa_mem_region_local_get();

    struct path_cx *cx = AA_MEM_REGION_NEW( local_region, struct path_cx );
    {
        cx->opts = opts;
        cx->ssg = ssg;
        cx->sg = aa_rx_sg_sub_sg(ssg);

        cx->frame = (AA_RX_FRAME_NONE ==  opts->frame)
            ? aa_rx_sg_sub_frame_ee(ssg)
            : opts->frame;

        cx->n_q_all = n_q_all;
        cx->n_q_sub = aa_rx_sg_sub_config_count(ssg);
        cx->n_f_all = aa_rx_sg_sub_all_frame_count(ssg);

        cx->q_start_all = q_start_all;

        cx->TF_rel = aa_rx_sg_alloc_tf( cx->sg, local_region );
        cx->TF_abs = aa_rx_sg_alloc_tf( cx->sg, local_region );

        cx->limits = aa_rx_ct_sg_limits(region, cx->sg);

        double *TF_rel0 = aa_rx_sg_alloc_tf( cx->sg, local_region );
        double *TF_abs0 = aa_rx_sg_alloc_tf( cx->sg, local_region );
        cx->TF_rel0 = TF_rel0;
        cx->TF_abs0 = TF_abs0;

        cx->q_all = AA_MEM_REGION_DUP(local_region, double, q_start_all, n_q_all);
        cx->J = aa_rx_sg_sub_alloc_jacobian(ssg, local_region);
        cx->state = aa_ct_state_alloc( local_region, n_q_all, cx->n_f_all );

        cx->segs = segs;
        cx->region = region;

        AA_MEM_CPY( cx->q_all, cx->q_start_all, cx->n_q_all );

        aa_rx_sg_tf( cx->sg,
                     cx->n_q_all, cx->q_start_all,
                     cx->n_f_all,
                     TF_rel0, 7,
                     TF_abs0, 7 );
    }

    struct aa_ode_sol_opts sol_opts;
    {
        sol_opts.adapt_tol_dec = opts->tol_dq / 16;
        sol_opts.adapt_tol_inc = opts->tol_dq / 2;
        sol_opts.adapt_factor_dec = 0.1;
        sol_opts.adapt_factor_inc = 2.0;
    }
    double *q_start_sub = aa_rx_sg_sub_alloc_config( ssg, local_region );


    aa_rx_sg_sub_config_get( cx->ssg,
                             cx->n_q_all, cx->q_all,
                             cx->n_q_sub, q_start_sub );


    // Adapative integration does not work properly for this
    int r = aa_ode_path( AA_ODE_RK4, &sol_opts,
                         cx->n_q_sub,
                         path_sys, cx,
                         path_check, cx,
                         0, opts->dt, q_start_sub,
                         region, n_points, path );


    // clamp
    for( size_t k = 0; k < *n_points; k ++ ) {
        for( size_t i = 0; i < cx->n_q_sub; i ++ ) {
            aa_rx_config_id config_id = aa_rx_sg_sub_config(ssg, i);
            (*path)[i + k*cx->n_q_sub] =
                aa_fclamp( (*path)[i + k*cx->n_q_sub],
                           cx->limits->min->q[config_id],
                           cx->limits->max->q[config_id]);
        }
    }
    aa_mem_region_pop(local_region, cx);


    return r;
    /* aa_mem_region_pop(cx.reg, cx.TF_rel0); */

    /* if( r ) { */
    /*     return AA_RX_NO_SOLUTION | AA_RX_NO_IK; */
    /* } else { */
    /*     return 0; */
    /* } */

}

AA_API struct aa_ct_limit *
aa_rx_ct_sg_limits( struct aa_mem_region *region, const struct aa_rx_sg *sg )
{
    struct aa_ct_limit *lim = AA_MEM_REGION_NEW(region, struct aa_ct_limit);
    size_t n_q = aa_rx_sg_config_count(sg);

    lim->min = aa_ct_state_alloc(region, n_q, 0);
    lim->max = aa_ct_state_alloc(region, n_q, 0);

    for( size_t i = 0; i < n_q; i ++ ) {
        aa_rx_sg_get_limit_pos(sg, (aa_rx_config_id)i,
                               lim->min->q+i, lim->max->q+i);
        aa_rx_sg_get_limit_vel(sg, (aa_rx_config_id)i,
                               lim->min->dq+i, lim->max->dq+i);
        aa_rx_sg_get_limit_acc(sg, (aa_rx_config_id)i,
                               lim->min->ddq+i, lim->max->ddq+i);
        aa_rx_sg_get_limit_eff(sg, (aa_rx_config_id)i,
                               lim->min->eff+i, lim->max->eff+i);
    }

    return lim;
}

/* AA_API int */
/* aa_rx_ct_tjx_path( struct aa_mem_region *region, */
/*                    const struct aa_rx_ksol_opts *opts, */
/*                    const struct aa_rx_sg_sub *ssg, aa_rx_frame_id frame, */
/*                    struct aa_ct_seg_list *segs, */
/*                    size_t n_q, const double *q0, */
/*                    size_t *n_path, double **path ) */
/* { */
/*     const struct aa_rx_sg *sg = ssg->scenegraph; */
/*     double dt0 = .1; */
/*     double dt1 = .01; */
/*     double t1 = aa_ct_seg_list_duration(segs); */
/*     size_t h = (size_t)(t1 / dt0); */
/*     size_t n_f = aa_rx_sg_frame_count(sg); */
/*     size_t n_q_sub = aa_rx_sg_sub_config_count(ssg); */

/*     *n_path = n_q*h; */
/*     *path = AA_MEM_REGION_NEW_N( region, double, *n_path ); */
/*     double *qq = *path; */
/*     AA_MEM_CPY(qq, q0, n_q); */

/*     double t = 0; */
/*     double *TF_rel = AA_MEM_REGION_NEW_N(region, double, 7*n_f); */
/*     double *TF_abs = AA_MEM_REGION_NEW_N(region, double, 7*n_f); */

/*     for( size_t k = 0; k < h; k ++ ) { */
/*         double tt = 0; */
/*         AA_MEM_CPY(qq+n_q, qq, n_q); */
/*         while( tt < dt0 ) { */
/*             double qsub[n_q_sub], dq[n_q_sub], E_ref[7]; */

/*             // Get state and ref */
/*             aa_rx_sg_tf(sg, n_q, qq, n_f, */
/*                         TF_rel, 7, */
/*                         TF_abs, 7); */
/*             double J[6*n_q_sub]; */
/*             aa_rx_sg_sub_jacobian( ssg, n_f, TF_abs, 7, J, 6 ); */
/*             aa_rx_sg_sub_config_get( ssg, */
/*                                      n_q, qq, */
/*                                      n_q_sub, qsub ); */

/*             // TODO: eval trajectory */

/*             // compute control */
/*             aa_rx_ik_jac_x2dq ( opts, n_q_sub, */
/*                                 qsub,  TF_abs + frame, */
/*                                 E_ref, J, dq ); */
/*             // integrate */
/*             for( size_t i = 0; i < n_q_sub; i ++ ) { */
/*                 qsub[i] += (dt1*dq[i]); */
/*             } */


/*             // update */
/*             aa_rx_sg_sub_config_get( ssg, */
/*                                      n_q_sub, qsub, */
/*                                      n_q, qq ); */
/*             t += dt1; */
/*             tt += dt1; */
/*         } */
/*         qq += n_q; */
/*     } */

/*     return 0; */
/* } */
