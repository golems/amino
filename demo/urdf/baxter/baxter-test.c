/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
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
 *   AND ON ANY HEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include "baxter-demo.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_sdl.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_wk.h"
#include "amino/diffeq.h"



struct display_cx {
    struct aa_rx_win *win;
    const struct aa_rx_sg *scenegraph;
    const struct aa_rx_sg_sub *ssg;
    struct aa_rx_wk_opts *wk_opts;
    const struct aa_rx_wk_lc3_cx *lc3;
    double E0[7];
    double *q;
    struct aa_dvec *dq_subset;
};


struct vec_field_fk_cx {
    const struct aa_rx_sg_sub *ssg;
    struct aa_dvec *q_all;
};

void de_vec_field_fk( void *vcx, const struct aa_dvec *q_sub,
                      struct aa_dvec *S_ee )
{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);



    struct vec_field_fk_cx *cx = (struct vec_field_fk_cx*) vcx;
    const struct aa_rx_sg *scenegraph = aa_rx_sg_sub_sg(cx->ssg);
    struct aa_dvec *q_all = aa_dvec_dup(reg, cx->q_all);

    aa_rx_sg_config_set( scenegraph, q_all->len, q_sub->len,
                         aa_rx_sg_sub_configs(cx->ssg),
                         q_sub->data, q_all->data );

    struct aa_dmat *TF_abs = aa_rx_sg_get_tf_abs(scenegraph, reg, q_all );

    double *E_act =  TF_abs->data + TF_abs->ld*(size_t)aa_rx_sg_sub_frame_ee(cx->ssg);
    double S[8]; aa_tf_qutr2duqu(E_act,S);
    struct aa_dvec Sv = AA_DVEC_INIT(8,S,1);
    aa_lb_dcopy(&Sv, S_ee);

    aa_mem_region_pop(reg,ptrtop);
}

int check( const struct aa_rx_sg_sub *ssg,
           struct aa_dvec *q_all,
           struct aa_dvec *q_sub,
           struct aa_dmat *TF_abs,
           struct aa_dvec *dx_r,
           struct aa_dvec *dq_u )

{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    /* Check computed command velocity */
    struct aa_dmat *J = aa_rx_sg_sub_get_jacobian( ssg, reg, TF_abs );
    size_t m = J->rows, n = J->cols;
    struct aa_dvec *dx_check = aa_dvec_alloc(reg, m);
    aa_lb_dgemv( CblasNoTrans, 1, J, dq_u, 0, dx_check );
    printf("ssd(dx): %f\n", aa_dvec_ssd( dx_r, dx_check ) );

    /* Check duqu derivative Jacobian */
    double *E_act =  TF_abs->data + TF_abs->ld*(size_t)aa_rx_sg_sub_frame_ee(ssg);
    double S[8]; aa_tf_qutr2duqu(E_act,S);
    struct aa_dmat *JS = aa_dmat_alloc(reg,8,n);
    aa_tf_duqu_jac_vel2diff(S, J, JS);

    struct aa_dvec *dSx = aa_dvec_alloc(reg,8);
    struct aa_dvec *dSJ = aa_dvec_alloc(reg,8);
    aa_tf_duqu_vel2diff(S, dx_check->data, dSx->data);
    aa_lb_dgemv( CblasNoTrans, 1, JS, dq_u, 0, dSJ );
    printf("ssd(dS): %f\n", aa_dvec_ssd( dSx, dSJ ) );

    /* Check finite difference Jacobian */
    struct vec_field_fk_cx fkcx;
    fkcx.ssg = ssg;
    fkcx.q_all = q_all;
    struct aa_dmat *JSfd = aa_dmat_alloc(reg,8,n);
    aa_de_jac_fd(de_vec_field_fk, &fkcx, q_sub, 1e-6, JSfd);
    printf("ssd(JS / FD): %f\n", aa_dmat_ssd( JS, JSfd ) );

    aa_mem_region_pop(reg,ptrtop);
    return 0;
}


int display( struct aa_rx_win *win, void *cx_, struct aa_sdl_display_params *params )
{
    (void)win;
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    struct display_cx *cx = (struct display_cx *)cx_;
    const struct aa_rx_sg *scenegraph = cx->scenegraph;

    const struct timespec *now = aa_sdl_display_params_get_time_now(params);
    const struct timespec *first = aa_sdl_display_params_get_time_initial(params);
    const struct timespec *last = aa_sdl_display_params_get_time_last(params);

    double t = aa_tm_timespec2sec( aa_tm_sub(*now, *first) );
    double dt = aa_tm_timespec2sec( aa_tm_sub(*now, *last) );


    size_t m = aa_rx_sg_config_count(scenegraph);
    size_t n = aa_rx_sg_frame_count(scenegraph);
    size_t n_c = aa_rx_sg_sub_config_count(cx->ssg);

    struct aa_dvec *q_subset = aa_dvec_alloc(reg,n_c);
    aa_rx_sg_config_get( scenegraph, m, n_c,
                         aa_rx_sg_sub_configs(cx->ssg), cx->q, q_subset->data );


    struct aa_dvec qv;
    aa_dvec_view( &qv, m, cx->q, 1 );
    struct aa_dmat *TF_abs = aa_rx_sg_get_tf_abs(scenegraph, reg, &qv );

    aa_rx_win_display_sg_tf( cx->win, params, scenegraph,
                             n, TF_abs->data, TF_abs->ld );

    /* Reference Velocity and Position */
    /* double dx_r[6]; */
    /* AA_MEM_ZERO(dx_r, 6); */
    struct aa_dvec *dx_r = aa_dvec_alloc(reg,6);
    aa_dvec_zero(dx_r);
    {
        double *E_act =  TF_abs->data + TF_abs->ld*(size_t)aa_rx_sg_sub_frame_ee(cx->ssg);

        double z_pos = sin(t*2*M_PI) / (4*M_PI);
        double z_vel = cos( t*2*M_PI ) / 2; // derivative of position


        double wx_pos = sin(t*2*M_PI) / (2*M_PI);
        double wx_vel = cos( t*2*M_PI );

        // feed-forward velocity
        dx_r->data[AA_TF_DX_V+2] += z_vel;
        dx_r->data[AA_TF_DX_W+0] += wx_vel;

        // proportional control on position
        double E_rel[7]; // Relative pose
        aa_tf_xangle2quat(wx_pos, E_rel+AA_TF_QUTR_Q);
        E_rel[AA_TF_QUTR_T + 0] = 0;
        E_rel[AA_TF_QUTR_T + 1] = 0;
        E_rel[AA_TF_QUTR_T + 2] = z_pos;

        double E_ref[7]; // Reference pose
        aa_tf_qutr_mul(cx->E0, E_rel, E_ref);

        // compute the proportional control
        aa_rx_wk_dx_pos( cx->wk_opts, E_act, E_ref, dx_r );
    }

    // joint-centering velocity for the nullspace projection
    struct aa_dvec *dqr_subset = aa_dvec_alloc(reg,n_c);
    aa_rx_wk_dqcenter( cx->ssg, cx->wk_opts,
                       q_subset, dqr_subset );

    // Cartesian to joint velocities, with nullspace projection
    struct aa_dvec *dq_subset = aa_dvec_alloc(reg,n_c);


    static int firsttime = 1;
    if (firsttime) {
        aa_dvec_zero(dq_subset);
        firsttime = 0;
    } else  {
        //aa_tick("solve: ");

        /* int r = aa_rx_wk_dx2dq( cx->ssg, cx->wk_opts, */
        /*                         TF_abs, */
        /*                         6, dx_r, */
        /*                         n_c, dq_subset ); */

        /* int r = aa_rx_wk_dx2dq_np( cx->ssg, cx->wk_opts, */
        /*                               TF_abs, */
        /*                               6, dx_r, */
        /*                               n_c, dqr_subset, dq_subset ); */

        int r = aa_rx_wk_dx2dq_lc3( cx->lc3, dt,
                                    TF_abs,
                                    dx_r,
                                    q_subset, cx->dq_subset,
                                    dqr_subset, dq_subset );

        assert(0 == r);
        //aa_tock();
    }

    check( cx->ssg, &qv, q_subset, TF_abs, dx_r, dq_subset );

    // integrate
    aa_lb_dcopy(dq_subset, cx->dq_subset);
    aa_lb_daxpy(dt, dq_subset, q_subset);

    aa_rx_sg_config_set( scenegraph, m, n_c,
                         aa_rx_sg_sub_configs(cx->ssg),
                         q_subset->data, cx->q );

    aa_sdl_display_params_set_update(params);

    aa_mem_region_pop(reg,ptrtop);

    return 0;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    // Initialize scene graph
    struct aa_rx_sg *scenegraph = baxter_demo_load_baxter(NULL);
    aa_rx_sg_init(scenegraph);

    // setup window
    struct aa_rx_win * win = baxter_demo_setup_window(scenegraph);
    struct aa_gl_globals *globals = aa_rx_win_gl_globals(win);
    aa_gl_globals_set_show_visual(globals, 1);
    aa_gl_globals_set_show_collision(globals, 0);

    // setup control context
    size_t n_q = aa_rx_sg_config_count(scenegraph);
    struct display_cx cx = {0};
    cx.win = win;
    cx.scenegraph = scenegraph;
    cx.q = AA_NEW0_AR(double, n_q );

    aa_rx_frame_id tip_id = aa_rx_sg_frame_id(scenegraph, "right_w2");
    cx.ssg = aa_rx_sg_chain_create( scenegraph, AA_RX_FRAME_ROOT, tip_id);
    cx.wk_opts = aa_rx_wk_opts_create();
    cx.dq_subset = aa_dvec_malloc(aa_rx_sg_sub_config_count(cx.ssg));
    cx.lc3 = aa_rx_wk_lc3_create(cx.ssg,cx.wk_opts);


    // set start and goal states
    const char *names[] = {"right_s0",
                           "right_s1",
                           "right_e0",
                           "right_e1",
                           "right_w0",
                           "right_w1",
                           "right_w2"};
    aa_rx_config_id ids[7];
    aa_rx_sg_config_indices( scenegraph, 7,
                             names, ids );
    double q1[7] = {-.25 * M_PI, // s0
                    0 * M_PI, // s1
                    M_PI, // e0
                    .5*M_PI, // e1
                    0, // w0
                    0*M_PI, // w1
                    0 // w2
    };
    aa_rx_sg_config_set( scenegraph, n_q, 7,
                         ids, q1, cx.q );

    // initial end-effector config
    {
        size_t n = aa_rx_sg_frame_count(scenegraph);
        double TF_abs[7*n];
        double TF_rel[7*n];
        aa_rx_sg_tf( scenegraph, n_q, cx.q,
                     n,
                     TF_rel, 7,
                     TF_abs, 7 );
        AA_MEM_CPY( cx.E0, TF_abs + 7*tip_id, 7 );
    }


    aa_rx_win_set_display( win, display, &cx, NULL );
    aa_rx_win_run();

    // cleanup
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
