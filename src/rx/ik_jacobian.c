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
    const double *S1;
    const double *dq_dt;

    size_t iteration;

    struct aa_mem_region *reg;

    const double *q0_all;
    size_t n_all;
    double *TF_rel0;
    double *TF_abs0;
};


static int ksol_duqu ( const struct kin_solve_cx *cx, const double *q_s, double S[8],  double *J)
{
    const struct aa_rx_sg_sub *ssg = cx->ssg;
    const struct aa_rx_sg *sg = ssg->scenegraph;

    size_t n_f = aa_rx_sg_frame_count(sg);
    size_t n_q = aa_rx_sg_config_count(sg);
    size_t n_sq = aa_rx_sg_sub_config_count(ssg);

    /* Get the configuration */
    double q_all[n_q];
    AA_MEM_CPY(q_all, cx->q0_all, n_q);
    aa_rx_sg_sub_config_set( ssg,
                             n_sq, q_s,
                             n_q, q_all);

    /* Compute the transforms */
    double *TF_rel = AA_MEM_REGION_NEW_N(cx->reg, double, 7*n_f);
    double *TF_abs = AA_MEM_REGION_NEW_N(cx->reg, double, 7*n_f);

    aa_rx_sg_tf_update( sg,
                        n_q, cx->q0_all, q_all,
                        n_f,
                        cx->TF_rel0, 7, cx->TF_abs0, 7,
                        TF_rel, 7, TF_abs, 7 );

    /* Fill the desired transform */
    if( S ) {
        aa_rx_frame_id id_ee = ( AA_RX_FRAME_NONE == cx->opts->frame )
            /* default to last frame in chain */
            ? aa_rx_sg_sub_frame(ssg, aa_rx_sg_sub_frame_count(ssg) - 1 )
            /* use specified frame */
            : cx->opts->frame;

        double *E_ee = TF_abs + 7*id_ee;
        aa_tf_qutr2duqu( E_ee, S );
    }

    /* Fill the Jacobian */
    if( J ) {
        aa_rx_sg_sub_jacobian( ssg, n_f, TF_abs, 7,
                               J, 6 );
    }

    aa_mem_region_pop(cx->reg, TF_rel);
    return 0;
}


static void rfx_kin_duqu_werr( const double S[8], const double S_ref[8], double werr[6] ) {
    double twist[8], de[8];
    aa_tf_duqu_mulc( S, S_ref, de );  // de = d*conj(d_r)
    aa_tf_duqu_minimize(de);
    aa_tf_duqu_ln( de, twist );     // twist = log( de )
    aa_tf_duqu_twist2vel( S, twist, werr );
}


static void rfx_kin_duqu_serr( const double S[8], const double S_ref[8],
                       double *theta, double *x ) {
    double S_rel[8];
    aa_tf_duqu_cmul( S, S_ref, S_rel ); // relative dual quaternion
    aa_tf_duqu_minimize(S_rel);
    //printf("srel: "); aa_dump_vec(stdout, S_rel, 8 );

    // quaternion angle
    *theta = atan2( sqrt(S_rel[0]*S_rel[0] + S_rel[1]*S_rel[1] + S_rel[2]*S_rel[2]),
                    S_rel[3] );

    // translation
    double xe[3];
    aa_tf_duqu_trans( S_rel, xe );
    *x = sqrt( xe[0]*xe[0] + xe[1]*xe[1] + xe[2]*xe[2] );
}


static void kin_solve_sys( const void *vcx,
                           double t, const double *AA_RESTRICT q,
                           double *AA_RESTRICT dq ) {
    (void) t; // time invariant
    const struct kin_solve_cx *cx = (const struct kin_solve_cx*)vcx;
    //printf("ksolve\n");

    // compute kinematics
    double S[8];
    double J[6*cx->n];
    double J_star[6*cx->n];
    ksol_duqu( cx, q, S, J );

    // position error
    double w_e[6];
    rfx_kin_duqu_werr( S, cx->S1, w_e );
    for( size_t i = 0; i < 3; i ++ ) {
        w_e[AA_TF_DX_V + i] *= -cx->opts->gain_trans;
        w_e[AA_TF_DX_W + i] *= -cx->opts->gain_angle;
    }

    double theta_err, x_err;
    rfx_kin_duqu_serr( S, cx->S1, &theta_err, &x_err );

    // TODO: Try DGECON to avoid damping when possible without taking the SVD

    // damped least squares
    if( theta_err < cx->opts->tol_angle_svd &&
        x_err < cx->opts->tol_trans_svd )
    {
        aa_la_dzdpinv( 6, cx->n, cx->opts->s2min, J, J_star );
    } else {
        aa_la_dpinv( 6, cx->n, cx->opts->k_dls, J, J_star );
    }

    if( cx->opts->q_ref ) {
        //printf("nullspace projection\n");
        // nullspace projection
        double dqnull[cx->n];
        for( size_t i = 0; i < cx->n; i ++ )  {
            dqnull[i] = - cx->dq_dt[i] * ( q[i] - cx->opts->q_ref[i] );
        }
        //aa_dump_vec( stdout, dqnull, cx->n );
        aa_la_xlsnp( 6, cx->n, J, J_star, w_e, dqnull, dq );
    } else {
        //printf("no projection\n");
        aa_la_mvmul(cx->n,6,J_star,w_e,dq);
    }
}

/* struct kin_solve_cx { */
/*     const struct aa_rx_sg_sub *ssg; */
/* } */

static int kin_solve_check( void *vcx, double t, double *AA_RESTRICT x, double *AA_RESTRICT y )
{
    //printf("check\n");
    (void)t;
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    const struct aa_rx_sg_sub *ssg  = cx->ssg;
    /* Clamp */
    for( size_t i = 0; i < cx->n; i ++ ) {
        double min,max;
        aa_rx_config_id id = aa_rx_sg_sub_config(ssg,i);
        if( 0 == aa_rx_sg_get_limit_pos(ssg->scenegraph, id, &min, &max ) ) {
            x[i] = aa_fclamp(x[i], min, max );
        }
    }

    /* Check term */
    double dq_norm = aa_la_dot( cx->n, y, y );

    double S[8];
    ksol_duqu( cx, x, S, NULL );
    double theta_err, x_err;
    rfx_kin_duqu_serr( S, cx->S1, &theta_err, &x_err );
    //printf("%f, %f, %f\n", t, theta_err, x_err );
    //printf("x: ");
    //aa_dump_vec(stdout, x, cx->n);
    //printf("y: ");
    //aa_dump_vec(stdout, y, cx->n);

    cx->iteration++;
    if( (theta_err < cx->opts->tol_angle) &&
        (x_err < cx->opts->tol_trans) &&
        (dq_norm < cx->opts->tol_dq) )
    {
        return 1;
    } else if( cx->iteration > cx->opts->max_iterations ) {
        return -1;
    } else {
        return 0;
    }

}


static int
aa_rx_sg_sub_ksol_dls( const struct aa_rx_sg_sub *ssg,
                       const struct aa_rx_ksol_opts *opts,
                       size_t n_tf, const double *TF, size_t ld_TF,
                       size_t n_q_all, const double *q_start_all,
                       size_t n_q, double *q_subset )
{

    /* Only chains for now */
    assert(n_tf == 1 );
    assert(n_q == aa_rx_sg_sub_config_count(ssg) );
    (void)ld_TF;

    if( 0 == n_q_all || NULL == q_start_all ) {
        q_start_all = opts->q_all_seed;
        n_q_all = opts->n_all_seed;
    }
    if( 0 == n_q_all || NULL == q_start_all ) {
        return AA_RX_INVALID_PARAMETER;
    }

    assert( aa_rx_sg_sub_all_config_count(ssg) == n_q_all );

    const struct aa_rx_sg *sg = ssg->scenegraph;
    size_t n_f = aa_rx_sg_frame_count(sg);

    double S[8];
    aa_tf_qutr2duqu( TF, S );

    assert( aa_rx_sg_sub_config_count(ssg) == n_q);

    double q0_sub[n_q];

    aa_rx_sg_config_get( ssg->scenegraph, n_q_all, n_q, aa_rx_sg_sub_configs(ssg),
                         q_start_all, q0_sub );

    /* printf("jacobian ik\n"); */
    /* printf("q ref:  %s\n", opts->q_ref ? "yes" : "no"); */
    /* if( opts->q_ref ) { */
    /*     aa_dump_vec( stdout, opts->q_ref, ssg->config_count ); */
    //}


    struct kin_solve_cx cx;
    cx.n = n_q;
    cx.opts = opts;
    cx.S1 = S;
    cx.ssg = ssg;
    cx.dq_dt = opts->dq_dt;
    cx.iteration = 0;

    struct aa_ode_sol_opts sol_opts;
    sol_opts.adapt_tol_dec = opts->tol_dq / 16;
    sol_opts.adapt_tol_inc = opts->tol_dq / 2;
    sol_opts.adapt_factor_dec = 0.1;
    sol_opts.adapt_factor_inc = 2.0;

    cx.q0_all = q_start_all;
    cx.n_all = n_q_all;

    cx.reg = aa_mem_region_local_get();
    cx.TF_rel0 = AA_MEM_REGION_NEW_N(cx.reg, double, 7*n_f);
    cx.TF_abs0 = AA_MEM_REGION_NEW_N(cx.reg, double, 7*n_f);

    aa_rx_sg_tf( ssg->scenegraph,
                 cx.n_all, cx.q0_all,
                 n_f,
                 cx.TF_rel0, 7,
                 cx.TF_abs0, 7 );

    int r = aa_ode_sol( AA_ODE_RK23_BS, &sol_opts, n_q,
                        kin_solve_sys, &cx,
                        kin_solve_check, &cx,
                        0, opts->dt, q0_sub, q_subset );

    aa_mem_region_pop(cx.reg, cx.TF_rel0);

    if( r ) {
        return AA_RX_NO_SOLUTION | AA_RX_NO_IK;
    } else {
        return 0;
    }

    /* return kin_solve( ssg, */
    /*            n_q, q0_sub, S, */
    /*            ksol_duqu, (void*)ssg, */
    /*            q_subset, opts ); */

}

struct aa_rx_ik_jac_cx
{
    const struct aa_rx_sg_sub *ssg;
    const struct aa_rx_ksol_opts *opts;
};

AA_API struct aa_rx_ik_jac_cx *
aa_rx_ik_jac_cx_create(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ksol_opts *opts )
{
    struct aa_rx_ik_jac_cx *cx = AA_NEW0(struct aa_rx_ik_jac_cx);
    cx->ssg = ssg;
    cx->opts = opts;
    return cx;
}

AA_API void
aa_rx_ik_jac_cx_destroy( struct aa_rx_ik_jac_cx *cx )
{
    free(cx);
}


AA_API int aa_rx_ik_jac_solve( const struct aa_rx_ik_jac_cx *context,
                               size_t n_tf, const double *TF, size_t ld_TF,
                               size_t n_q, double *q )
{
    return aa_rx_sg_sub_ksol_dls( context->ssg, context->opts,
                                  n_tf, TF, ld_TF,
                                  0, NULL,
                                  n_q, q );
}

AA_API int aa_rx_ik_jac_fun( void *context_,
                             size_t n_tf, const double *TF, size_t ld_TF,
                             size_t n_q, double *q )
{
    struct aa_rx_ik_jac_cx *cx = (struct aa_rx_ik_jac_cx*)context_;
    return aa_rx_ik_jac_solve( cx,
                               n_tf, TF, ld_TF,
                               n_q, q );
}



/* Levenberg Marquardt
 *
 *
 *   dq = pinv(J) * log( conj(S)*S_ref ) + (I-pinv(J)*J) dq_dt * (q - q_ref)
 *   q(k+1) = q(k) + dt(k) * dq(k)
 *
 *   dt is adjusted using an adaptive Runge-Kutta
 *   dq_dt is adjusted when dq is too small
 */
/* static int kin_solve( const struct aa_rx_sg_sub *ssg, */
/*                       size_t n, const double *q0, const double S1[8], */
/*                       rfx_kin_duqu_fun kin_fun, void *kin_cx, */
/*                       double *q1, */
/*                       const struct aa_rx_ksol_opts *opts ) { */

    /* struct kin_solve_cx cx; */
    /* cx.n = n; */
    /* cx.opts = opts; */
    /* cx.S1 = S1; */
    /* cx.kin_fun = kin_fun; */
    /* cx.kin_cx = kin_cx; */
    /* cx.dq_dt = opts->dq_dt; */

    /* struct aa_ode_sol_opts sol_opts; */
    /* sol_opts.adapt_tol_dec = opts->tol_dq / 16; */
    /* sol_opts.adapt_tol_inc = opts->tol_dq / 2; */
    /* sol_opts.adapt_factor_dec = 0.1; */
    /* sol_opts.adapt_factor_inc = 2.0; */

    /* aa_ode_sol( AA_ODE_RK1, &sol_opts, n, */
    /*             kin_solve_sys, &cx, */
    /*             kin_solve_check, &cx, */
    /*             0, opts->dt, q0, q1 ); */

    /* return 0; */



    /*             aa_odestep_dorpri45( n, kin_solve_sys, &cx, */
    /*                                  0, dt, */
    /*                                  q1, k, q4, q5 ); */


    /* AA_MEM_CPY(q1, q0, n); */

    /* size_t iters = 0; */

    /* double k[n*6]; // adaptive runge-kutta internal derivatives */
    /* kin_solve_sys( &cx, 0, q1, k ); // initial dx for adaptive runge-kutta */
    /* double dt = opts->dt; // adaptive timestep */
    /* double dq_dt[n]; */
    /* if( opts->dq_dt ) { */
    /*     AA_MEM_CPY( dq_dt, opts->dq_dt, n ); */
    /*     cx.dq_dt = dq_dt; */
    /* } else { */
    /*     cx.dq_dt = NULL; */
    /* } */

    /* double dq_norm = opts->tol_dq; */
    /* double theta_err = opts->tol_angle; */
    /* double x_err = opts->tol_trans; */

    /* while( (fabs(theta_err) >= opts->tol_angle || */
    /*         fabs(x_err) >= opts->tol_trans || */
    /*         dq_norm >= opts->tol_dq ) */
    /*     && iters < opts->max_iterations ) */
    /* { */
    /*     //printf("iter: %d\n", iters); */
    /*     iters++; */


    /*     dq_norm = 0; */

    /*     // integrate */
    /*     /\* double dq[n]; *\/ */
    /*     /\* kin_solve_sys( &cx, 0, q1, dq ); *\/ */
    /*     /\* for( size_t i = 0; i < n; i ++ ) { *\/ */
    /*     /\*     dq_norm += dq[i] * dq[i]; *\/ */
    /*     /\*     q1[i] += dq[i]; *\/ */
    /*     /\* } *\/ */

    /*     /\* double q[n]; *\/ */
    /*     /\* aa_odestep_rk4( n, kin_solve_sys, &cx, *\/ */
    /*     /\*                 0, opts->dt0, *\/ */
    /*     /\*                 q1, q ); *\/ */
    /*     /\* for( size_t i = 0; i < n; i ++ ) { *\/ */
    /*     /\*     dq_norm += (q1[i]-q[i]) * (q1[i]-q[i]); *\/ */
    /*     /\*     q1[i] = q[i]; *\/ */
    /*     /\* } *\/ */


    /*     // Adaptive step */
    /*     { */
    /*         double q5[n]; */
    /*         double qerr; */
    /*         do { */
    /*             double q4[n]; */
    /*             aa_odestep_dorpri45( n, kin_solve_sys, &cx, */
    /*                                  0, dt, */
    /*                                  q1, k, q4, q5 ); */
    /*             qerr = aa_la_ssd( n, q4, q5 ); */
    /*             // adapt the step size */
    /*             if( qerr > opts->tol_dq / 2) { */
    /*                 dt /= 10; */
    /*                 //printf("reducing dt: %f\n", dt); */
    /*             } else if ( qerr < opts->tol_dq / 16 ) { */
    /*                 dt *= 2; */
    /*                 //printf("increasing dt: %f\n", dt); */
    /*             } */
    /*         } while( qerr > opts->tol_dq ); */

    /*         dq_norm = aa_la_ssd( n, q1, q5 ); */
    /*         AA_MEM_CPY(k, k+5*n, n); // write dx to k0 */

    /*         // copy and clamp clamp */
    /*         for( size_t i = 0; i < n; i ++ ) { */
    /*             double min,max; */
    /*             aa_rx_config_id id = aa_rx_sg_sub_config(ssg,i); */
    /*             if( 0 == aa_rx_sg_get_limit_pos(ssg->scenegraph, id, &min, &max ) ) { */
    /*                 q1[i] = aa_fclamp(q5[i], min, max ); */
    /*             } else { */
    /*                 q1[i] = q5[i]; */
    /*             } */
    /*         } */
    /*     } */

    /*     //printf("q: "); aa_dump_vec(stdout, q1, n ); */
    /*     //dq_norm = aa_la_norm( n, k ); */

    /*     // check error */
    /*     double S[8]; */
    /*     kin_fun( kin_cx, q1, S, NULL ); */
    /*     rfx_kin_duqu_serr( S, S1, &theta_err, &x_err ); */
    /*     //printf("err: theta: %f, x: %f, dqn: %f\n", theta_err, x_err, dq_norm); */
    /*     //printf("dq_norm: %f\n", dq_norm ); */

    /*     // adapt the nullspace gain */
    /*     /\* if( cx.dq_dt && *\/ */
    /*     /\*     dq_norm < opts->tol_dq && *\/ */
    /*     /\*     (theta_err > opts->tol_angle || *\/ */
    /*     /\*      x_err > opts->tol_trans ) ) *\/ */
    /*     /\* { *\/ */
    /*     /\*     for( size_t i = 0; i < n; i ++  ) dq_dt[i] /= 2; *\/ */
    /*     /\*     //printf("halving nullspace gain: %f\n", cx.dq_dt); *\/ */
    /*     /\* } *\/ */
    /* }; */

    //double S[8], E[7];
    //kin_fun( kin_cx, q1, S, NULL );
    //aa_tf_duqu2qutr(S,E);
    //aa_dump_vec( stdout, E, 7 );

    //printf(" iter: %d\n", iters );
    //printf(" norm: %f\n", aa_la_dot( n, q1, q1 ) );

    /* Did we get there? */
    /* if( fabs(theta_err) >= opts->tol_angle || */
    /*     fabs(x_err) >= opts->tol_trans ) */
    /* { */
    /*     return -1; */
    /* } else { */
    /*     return 0; */
    /* } */

/*     return 0; */
/* } */
