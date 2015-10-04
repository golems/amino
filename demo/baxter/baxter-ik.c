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
#define GL_GLEXT_PROTOTYPES

#include <error.h>
#include <stdio.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/amino_gl.h"
#include "amino/rx/amino_sdl.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_kin.h"

#include "baxter-demo.h"


struct display_cx {
    const struct aa_gl_globals *globals;
    const struct aa_rx_sg *scenegraph;
    double *q;
};

int display( void *cx_, int updated, const struct timespec *now )
{
    if( !updated ) return 0;
    struct display_cx *cx = (struct display_cx*) cx_;
    const struct aa_gl_globals *globals = cx->globals;
    const struct aa_rx_sg *scenegraph = cx->scenegraph;

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    baxter_demo_check_error("glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    baxter_demo_check_error("glClear");

    aa_rx_frame_id n = aa_rx_sg_frame_count(scenegraph);
    aa_rx_frame_id m = aa_rx_sg_config_count(scenegraph);
    //double q[m];
    //AA_MEM_ZERO(q,m);
    double TF_rel[7*n];
    double TF_abs[7*n];
    aa_rx_sg_tf(scenegraph, m, cx->q,
                n,
                TF_rel, 7,
                TF_abs, 7 );
    aa_rx_sg_render( scenegraph, globals,
                     (size_t)n, TF_abs, 7 );
    return updated;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;
    SDL_Window* window = NULL;
    SDL_GLContext gContext = NULL;
    struct aa_gl_globals *globals;

    // Initialize scene graph
    struct aa_rx_sg *scenegraph = generate_scenegraph(NULL);
    aa_rx_sg_index(scenegraph);

    // setup window
    baxter_demo_setup_window( scenegraph,
                              &window, &gContext, &globals );
    aa_gl_globals_set_show_visual(globals, 1);
    aa_gl_globals_set_show_collision(globals, 0);


    // Display Context
    size_t n_q = aa_rx_sg_config_count(scenegraph);
    struct display_cx dcx = { .globals=globals,
                              .scenegraph=scenegraph,
                              .q = AA_NEW0_AR(double,n_q) };


    aa_rx_frame_id tip_id = aa_rx_sg_frame_id(scenegraph, "right_w2");
    struct aa_rx_sg_sub *ssg = aa_rx_sg_chain_create( scenegraph, AA_RX_FRAME_ROOT, tip_id);
    size_t n_qs = aa_rx_sg_sub_config_count(ssg);
    double E[7] = {0, 0, 0, 1,
                   0.6, -0.0, 0.3};

    assert( 7 == n_qs );
    double qstart_s[7] = {
        .05 * M_PI, // s0
        -.25 * M_PI, // s1
        0, // e0
        .25*M_PI, // e1
        0, // w0
        .25*M_PI, // w1
        0 // w2
    };
    double qstart_all[n_q];
    AA_MEM_ZERO(qstart_all, n_q);
    aa_rx_sg_config_set( scenegraph, n_q, n_qs, aa_rx_sg_sub_configs(ssg),
                         qstart_s, qstart_all );


    double qs[n_qs];
    double q_ref[n_qs];
    double q_gain[n_qs];
    struct aa_rx_ksol_opts *ko = aa_rx_ksol_opts_create();

    for( size_t i = 0; i < n_qs; i ++ ) {
        double min=0 ,max=0;
        aa_rx_config_id config_id = aa_rx_sg_sub_config(ssg, i);
        aa_rx_sg_get_limit_pos( scenegraph, config_id, &min, &max );
        q_ref[i] = (max + min) / 2;
        //q_gain[i] = 0;
        q_gain[i] = 10;
    }

    aa_rx_ksol_opts_set_tol_dq( ko, .01 );
    aa_rx_ksol_opts_take_config( ko, n_qs, q_ref, AA_MEM_BORROW );
    aa_rx_ksol_opts_take_gain_config( ko, n_qs, q_gain, AA_MEM_BORROW );


    aa_rx_sg_chain_ksol_dls( ssg, ko,
                             E, n_q, qstart_all,
                             n_qs, qs );

    aa_rx_sg_config_set( scenegraph, n_q, n_qs, aa_rx_sg_sub_configs(ssg),
                         qs, dcx.q );



    // Do Display
    aa_sdl_display_loop( window, globals,
                         display,
                         &dcx );

    SDL_GL_DeleteContext(gContext);
    SDL_DestroyWindow( window );

    SDL_Quit();
    return 0;
}
