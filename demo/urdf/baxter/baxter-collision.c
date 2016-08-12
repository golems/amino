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
#include "amino/rx/scene_collision.h"


struct display_cx {
    struct aa_rx_win *win;
    const struct aa_rx_sg *scenegraph;
    struct aa_rx_cl *cl;
    double q;
    aa_rx_config_id i_q;
    //struct timespec last;
};

int display( struct aa_rx_win *win, void *cx_, struct aa_sdl_display_params *params )
{
    (void)win;
    struct display_cx *cx = (struct display_cx *)cx_;
    const struct timespec *now = aa_sdl_display_params_get_time_now(params);
    const struct timespec *last = aa_sdl_display_params_get_time_last(params);

    const struct aa_rx_sg *scenegraph = cx->scenegraph;

    size_t n = aa_rx_sg_frame_count(scenegraph);
    size_t m = aa_rx_sg_config_count(scenegraph);
    double q[m];
    AA_MEM_ZERO(q,m);

    int first = aa_sdl_display_params_is_first(params);
    if( ! first ) {
        double dt = aa_tm_timespec2sec( aa_tm_sub(*now, *last) );
        cx->q += dt * 45 * (M_PI/180);
    }
    q[ cx->i_q ] = cx->q;

    double TF_rel[7*n];
    double TF_abs[7*n];
    aa_rx_sg_tf(scenegraph, m, q,
                n,
                TF_rel, 7,
                TF_abs, 7 );

    aa_rx_win_display_sg_tf( cx->win, params, scenegraph,
                             n, TF_abs, 7 );

    int col = aa_rx_cl_check( cx->cl, n, TF_abs, 7, NULL );
    printf("in collision: %s\n",
           col ? "yes" : "no" );


    return 0;
}


int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    // Initialize scene graph
    struct aa_rx_sg *scenegraph = baxter_demo_load_baxter(NULL);
    aa_rx_sg_init(scenegraph);
    aa_rx_sg_cl_init(scenegraph);


    // setup window
    struct aa_rx_win * win = baxter_demo_setup_window(scenegraph);
    struct aa_gl_globals *globals = aa_rx_win_gl_globals(win);
    aa_gl_globals_set_show_visual(globals, 0);
    aa_gl_globals_set_show_collision(globals, 1);

    struct display_cx cx = {0};
    cx.win = win;
    cx.scenegraph = scenegraph;
    cx.i_q = aa_rx_sg_config_id(scenegraph, "left_s0");
    cx.cl = aa_rx_cl_create( scenegraph );

    // Identify allowed collisions
    {
        size_t n = aa_rx_sg_frame_count(scenegraph);
        size_t m = aa_rx_sg_config_count(scenegraph);
        double q[m];
        AA_MEM_ZERO(q,m);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(scenegraph, m, q,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );

        struct aa_rx_cl_set *allowed = aa_rx_cl_set_create( scenegraph );
        aa_rx_cl_check( cx.cl, n, TF_abs, 7, allowed );
        aa_rx_cl_allow_set( cx.cl, allowed );
        aa_rx_cl_set_destroy( allowed );
    }

    aa_rx_win_set_display( win, display, &cx, NULL );
    aa_rx_win_display_loop(win);


    // Cleanup
    aa_rx_cl_destroy( cx.cl );
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
