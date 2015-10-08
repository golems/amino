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

// Fuck you, C++
#define protected public
#define protected public

#include <error.h>
#include <stdio.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_sdl.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"

#include "baxter-demo.h"


//amino::sgStateValidityChecker *g_checker;
double *g_path = NULL;
size_t g_n_path = 0;


static void motion_plan( const struct aa_rx_sg *scenegraph)
{

    // Initialize Spaces
    const char *names[] = {"right_s0",
                           "right_s1",
                           "right_e0",
                           "right_e1",
                           "right_w0",
                           "right_w1",
                           "right_w2"};


    struct aa_rx_mp *mp = aa_rx_mp_create( scenegraph, 7, names, NULL );

    size_t n_q = aa_rx_sg_config_count(scenegraph);
    double q0[n_q];
    AA_MEM_ZERO(q0, n_q);
    aa_rx_mp_set_start( mp, n_q, q0);

    // set start and goal states
    double q1[7] = {.05 * M_PI, // s0
                    -.25 * M_PI, // s1
                    0, // e0
                    .25*M_PI, // e1
                    0, // w0
                    .25*M_PI, // w1
                    0 // w2
    };
    aa_rx_mp_set_goal( mp, 7, q1 );

    // plan
    int r = aa_rx_mp_plan( mp, 5.0,
                           &g_n_path,
                           &g_path );

    aa_rx_mp_destroy(mp);

    if( r < 0 ) {
        fprintf(stderr, "Planning failed!\n");
        exit(EXIT_FAILURE);
    }
}


struct display_cx {
    const struct aa_gl_globals *globals;
    const struct aa_rx_sg *scenegraph;
    struct aa_rx_cl *cl;
    double q;
    aa_rx_config_id i_q;
    int init;
    struct timespec last;
    struct timespec t0;
};

int display( void *cx_, int updated, const struct timespec *now )
{
    struct display_cx *cx = (struct display_cx *)cx_;
    const struct aa_gl_globals *globals = cx->globals;
    const struct aa_rx_sg *scenegraph = cx->scenegraph;


    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    baxter_demo_check_error("glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    baxter_demo_check_error("glClear");

    if( 0 == cx->init ) {
        cx->init = 1;
        memcpy( &cx->t0, now, sizeof(*now) );
        return updated;
    }

    aa_rx_frame_id m = aa_rx_sg_config_count(scenegraph);
    aa_rx_frame_id n = aa_rx_sg_frame_count(scenegraph);

    double t = aa_tm_timespec2sec( aa_tm_sub( *now, cx->t0 ) );
    double dt = .8;
    //double t_max = (g_n_path-1) * dt;


    int i0 = t / dt;
    int i1 = i0 + 1;
    double *q0, *q1;
    if( g_n_path == i1 ) {
        i1 = i0;
    } else if( g_n_path < i1 ) {
        t = 0;
        i1 = 0;
        i0 = 0;
        memcpy( &cx->t0, now, sizeof(*now) );
    }

    assert( i0 < g_n_path );
    assert( i1 < g_n_path );
    q0 = AA_MATCOL(g_path,m,i0);
    q1 = AA_MATCOL(g_path,m,i1);


    double q[m];
    if( q0 == q1 ) {
        AA_MEM_CPY( q, q0, m );
    } else {
        aa_la_linterp( m,
                       i0*dt, q0,
                       i1*dt, q1,
                       t, q );
    }
    //double qs[ g_space->config_count_set() ];
    //g_space->state_get( q, qs );


    double *TF_rel = AA_MEM_REGION_LOCAL_NEW_N( double, 7*n );
    double *TF_abs = AA_MEM_REGION_LOCAL_NEW_N( double, 7*n );
    aa_rx_sg_tf(scenegraph, m, q,
                n,
                TF_rel, 7,
                TF_abs, 7 );
    aa_rx_sg_render( scenegraph, globals,
                     (size_t)n, TF_abs, 7 );

    memcpy( &cx->last, now, sizeof(*now) );


    aa_mem_region_local_pop( TF_rel );
    return 1;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    // Initialize scene graph
    aa_rx_cl_init();
    struct aa_rx_sg *scenegraph = generate_scenegraph(NULL);
    aa_rx_sg_init(scenegraph);
    aa_rx_sg_cl_init(scenegraph);

    // Do Planning
    motion_plan(scenegraph);

    // setup window
    struct aa_rx_sdl_win * win = baxter_demo_setup_window(scenegraph);
    struct aa_gl_globals *globals = aa_rx_sdl_win_gl_globals(win);
    aa_gl_globals_set_show_visual(globals, 1);
    aa_gl_globals_set_show_collision(globals, 0);


    struct display_cx cx = {0};
    cx.globals = globals;
    cx.scenegraph = scenegraph;
    cx.i_q = aa_rx_sg_config_id(scenegraph, "left_s0");
    cx.cl = aa_rx_cl_create( scenegraph );

    {
        aa_rx_frame_id n = aa_rx_sg_frame_count(scenegraph);
        aa_rx_frame_id m = aa_rx_sg_config_count(scenegraph);
        double q[m];
        AA_MEM_ZERO(q,m);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(scenegraph, m, q,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );

        struct aa_rx_cl_set *allowed = aa_rx_cl_set_create( scenegraph );
        int col = aa_rx_cl_check( cx.cl, n, TF_abs, 7, allowed );
        aa_rx_cl_allow_set( cx.cl, allowed );
        aa_rx_cl_set_destroy( allowed );
    }

    aa_sdl_win_display_loop( win, display, &cx );

    // Cleanup
    if( g_path ) free( g_path );
    aa_rx_cl_destroy( cx.cl );
    aa_rx_sg_destroy(scenegraph);
    aa_rx_sdl_win_destroy(win);
    aa_mem_region_local_destroy();
    SDL_Quit();

    return 0;
}
