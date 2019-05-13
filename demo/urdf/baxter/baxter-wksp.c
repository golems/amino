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
#include "amino/rx/scene_fk.h"
#include "amino/rx/scene_sdl.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_wk.h"



struct display_cx {
    struct aa_rx_win *win;
    const struct aa_rx_sg *scenegraph;
    const struct aa_rx_sg_sub *ssg;
    struct aa_rx_wk_opts *wk_opts;
    const struct aa_rx_wk_lc3_cx *lc3;
    double E0[7];
    struct aa_dvec *q_all;
    struct aa_dvec *dq_subset;
    struct aa_rx_fk *fk;
};

int display( struct aa_rx_win *win, void *cx_, struct aa_sdl_display_params *params )
{
    (void)win;
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    struct display_cx *cx = (struct display_cx *)cx_;

    const struct timespec *now = aa_sdl_display_params_get_time_now(params);
    const struct timespec *first = aa_sdl_display_params_get_time_initial(params);
    const struct timespec *last = aa_sdl_display_params_get_time_last(params);

    double t = aa_tm_timespec2sec( aa_tm_sub(*now, *first) );
    double dt = aa_tm_timespec2sec( aa_tm_sub(*now, *last) );


    size_t n_c = aa_rx_sg_sub_config_count(cx->ssg);
    struct aa_dvec *q_subset = aa_dvec_alloc(reg,n_c);
    aa_rx_sg_sub_config_gather(cx->ssg, cx->q_all, q_subset );

    struct aa_rx_fk *fk = cx->fk;
    aa_rx_fk_all(fk,cx->q_all);

    aa_rx_win_display_fk( cx->win, params, fk );

    /* Reference Velocity and Position */
    struct aa_dvec *dx_r = aa_dvec_alloc(reg,6);
    aa_dvec_zero(dx_r);
    {
        double E_act[7];
        aa_rx_fk_get_abs_qutr( fk, aa_rx_sg_sub_frame_ee(cx->ssg), E_act );

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
        /*                         fk, dx_r, dq_subset ); */

        /* int r = aa_rx_wk_dx2dq_np( cx->ssg, cx->wk_opts, */
        /*                            fk, dx_r, */
        /*                            dqr_subset, dq_subset ); */

        int r = aa_rx_wk_dx2dq_lc3( cx->lc3, dt,
                                    fk,
                                    dx_r,
                                    q_subset, cx->dq_subset,
                                    dqr_subset, dq_subset );

        assert(0 == r);
        //aa_tock();
    }


    // integrate
    aa_lb_dcopy(dq_subset, cx->dq_subset);
    aa_lb_daxpy(dt,dq_subset,q_subset);
    aa_rx_sg_sub_config_scatter(cx->ssg, q_subset, cx->q_all);

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
    cx.q_all = aa_dvec_malloc(n_q);
    aa_dvec_zero(cx.q_all);

    aa_rx_frame_id tip_id = aa_rx_sg_frame_id(scenegraph, "right_w2");
    cx.ssg = aa_rx_sg_chain_create( scenegraph, AA_RX_FRAME_ROOT, tip_id);
    cx.wk_opts = aa_rx_wk_opts_create();
    cx.dq_subset = aa_dvec_malloc(aa_rx_sg_sub_config_count(cx.ssg));
    cx.lc3 = aa_rx_wk_lc3_create(cx.ssg,cx.wk_opts);
    cx.fk = aa_rx_fk_malloc(scenegraph);


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
    // initial end-effector config
    {
        double q1[7] = {-.25 * M_PI, // s0
                        0 * M_PI, // s1
                        M_PI, // e0
                        .5*M_PI, // e1
                        0, // w0
                        0*M_PI, // w1
                        0 // w2
        };
        struct aa_dvec vq1 = AA_DVEC_INIT(7,q1,1);
        aa_rx_sg_sub_config_scatter(cx.ssg, &vq1, cx.q_all);

        aa_rx_fk_all(cx.fk, cx.q_all);
        aa_rx_fk_get_abs_qutr( cx.fk, tip_id, cx.E0 );
    }


    aa_rx_win_set_display( win, display, &cx, NULL );
    aa_rx_win_run();

    // cleanup
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
