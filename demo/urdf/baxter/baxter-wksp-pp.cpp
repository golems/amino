/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
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

#include "amino/mat.hpp"
#include "amino/tf.hpp"

using namespace amino;

struct display_cx {
    struct aa_rx_win *win;
    const struct aa_rx_sg *scenegraph;
    const struct aa_rx_sg_sub *ssg;
    struct aa_rx_wk_opts *wk_opts;
    const struct aa_rx_wk_lc3_cx *lc3;
    QuatTran E0;
    DVec &q;
    DVec &dq_subset;
};

int display( struct aa_rx_win *win, void *cx_, struct aa_sdl_display_params *params )
{
    (void)win;

    RegionScope rs;
    struct aa_mem_region *reg =  rs.reg();

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

    DVec &q_subset = *DVec::alloc(reg,n_c);
    aa_rx_sg_config_get( scenegraph, m, n_c,
                         aa_rx_sg_sub_configs(cx->ssg), cx->q.data(), q_subset.data() );


    struct aa_dmat *TF_abs = aa_rx_sg_get_tf_abs(scenegraph, reg, &cx->q );

    aa_rx_win_display_sg_tf( cx->win, params, scenegraph,
                             n, TF_abs->data, TF_abs->ld );

    /* Reference Velocity and Position */
    DVec &dx_r = *DVec::alloc(reg,6);
    dx_r = 0.0;
    {
        aa_rx_config_id id = aa_rx_sg_sub_frame_ee(cx->ssg) ;
        double *E_act = DMat::col_vec(TF_abs,(size_t)id).data();

        double z_pos = sin(t*2*M_PI) / (4*M_PI);
        double z_vel = cos( t*2*M_PI ) / 2; // derivative of position

        double wx_pos = sin(t*2*M_PI) / (2*M_PI);
        double wx_vel = cos( t*2*M_PI );

        // feed-forward velocity
        dx_r[AA_TF_DX_V+2] += z_vel;
        dx_r[AA_TF_DX_W+0] += wx_vel;

        // proportional control on position
        QuatTran E_rel( XAngle(wx_pos), Vec3(0,0,z_pos) );

        QuatTran E_ref = cx->E0 * E_rel; // Reference pose

        // compute the proportional control
        aa_rx_wk_dx_pos( cx->wk_opts, E_act, E_ref.data, &dx_r );
    }

    // joint-centering velocity for the nullspace projection
    DVec &dqr_subset = *DVec::alloc(reg,n_c);
    aa_rx_wk_dqcenter( cx->ssg, cx->wk_opts,
                       &q_subset, &dqr_subset );

    // Cartesian to joint velocities, with nullspace projection
    DVec &dq_subset = *DVec::alloc(reg,n_c);


    static int firsttime = 1;
    if (firsttime) {
        dq_subset = 0.0;
        firsttime = 0;
    } else  {
        //aa_tick("solve: ");

        /* int r = aa_rx_wk_dx2dq( cx->ssg, cx->wk_opts, */
        /*                         TF_abs, */
        /*                         6, &dx_r, */
        /*                         n_c, &dq_subset ); */

        /* int r = aa_rx_wk_dx2dq_np( cx->ssg, cx->wk_opts, */
        /*                               TF_abs, */
        /*                               6, &dx_r, */
        /*                               n_c, &dqr_subset, &dq_subset ); */

        int r = aa_rx_wk_dx2dq_lc3( cx->lc3, dt,
                                    TF_abs,
                                    &dx_r,
                                    &q_subset, &cx->dq_subset,
                                    &dqr_subset, &dq_subset );

        assert(0 == r);
        //aa_tock();
    }

    // copy velocity
    cx->dq_subset = dq_subset;

    // integrate
    q_subset += dt*dq_subset;
    aa_rx_sg_config_set( scenegraph, m, n_c,
                         aa_rx_sg_sub_configs(cx->ssg),
                         q_subset.data(), cx->q.data() );

    aa_sdl_display_params_set_update(params);


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
    aa_rx_frame_id tip_id = aa_rx_sg_frame_id(scenegraph, "right_w2");
    struct display_cx cx = {
        .win = win,
        .scenegraph  =  scenegraph,
        .ssg = aa_rx_sg_chain_create( scenegraph, AA_RX_FRAME_ROOT, tip_id),
        .wk_opts = aa_rx_wk_opts_create(),
        .lc3 = aa_rx_wk_lc3_create(cx.ssg,cx.wk_opts),
        .E0 = QuatTran(),
        .q = *DVec::alloc_local(n_q),
        .dq_subset = *DVec::alloc_local(aa_rx_sg_sub_config_count(cx.ssg))
    };



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
                         ids, q1, cx.q.data() );

    // initial end-effector config
    {
        RegionScope rs;
        struct aa_dmat *TF_abs = aa_rx_sg_get_tf_abs( scenegraph, rs.reg(), &cx.q );
        cx.E0 = QuatTran::from_qv( DMat::col_vec(TF_abs,tip_id).data() );
    }


    aa_rx_win_set_display( win, display, &cx, NULL );
    aa_rx_win_run();

    // cleanup
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
