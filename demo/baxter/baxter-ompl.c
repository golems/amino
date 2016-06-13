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
#include "amino/rx/rxerr.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"


const char *allowed_collision[][2] = {{"right_w0_fixed" , "right_wrist-collision"},
                                      {NULL,NULL}};

static void check_mp_error( int r ) {
    if( r ) {
        char *e = aa_rx_errstr( aa_mem_region_local_get(), r );
        fprintf(stderr, "Inverse Kinematics failed: `%s' (0x%x)\n", e, r);
        aa_mem_region_local_pop(e);
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    // Initialize scene graph
    aa_rx_cl_init(); /* initialize the collision library */
    struct aa_rx_sg *scenegraph = baxter_demo_load_baxter(NULL); /* load compiled URDF for robot scenegraph */
    aa_rx_sg_init(scenegraph); /* initialize scene graph internal structures */
    aa_rx_sg_cl_init(scenegraph); /* initialize scene graph collision objects */

    // Create sub-scene-graph to plan over
    struct aa_rx_sg_sub *ssg = aa_rx_sg_chain_create( scenegraph,
                                                      AA_RX_FRAME_ROOT,
                                                      aa_rx_sg_frame_id(scenegraph, "right_w2") );

    struct aa_rx_mp *mp = aa_rx_mp_create( ssg ); /* Create motion planning context */
    double *g_path = NULL; /* storage for plan path */
    size_t g_n_path = 0; /* storage for plan length */

    /* Start state state to all zero */
    size_t n_q = aa_rx_sg_config_count(scenegraph); /* extract scene graph configuration space size */
    double q0[n_q];
    AA_MEM_ZERO(q0, n_q);
    aa_rx_mp_set_start( mp, n_q, q0);

    /* Set allowed collisions */
    for (size_t i = 0; NULL != allowed_collision[i][0]; i ++ ) {
        aa_rx_frame_id id0 = aa_rx_sg_frame_id(scenegraph, allowed_collision[i][0]);
        aa_rx_frame_id id1 = aa_rx_sg_frame_id(scenegraph, allowed_collision[i][1]);
        aa_rx_mp_allow_collision( mp, id0, id1, 1 );
    }

    /* set joint space goal state */
    assert( 7 == aa_rx_sg_sub_config_count(ssg) );
    double q1[7] = {.05 * M_PI, // s0
                    -.25 * M_PI, // s1
                    0, // e0
                    .25*M_PI, // e1
                    0, // w0
                    .25*M_PI, // w1
                    0 // w2
    };
    aa_rx_mp_set_goal( mp, 7, q1 );

    /* Enable path simplification */
    aa_rx_mp_set_simplify(mp,1);

    /* Execute Planner */
    int r = aa_rx_mp_plan( mp, aa_rx_mp_make_rrtconnect(mp), 5, &g_n_path, &g_path );
    if(r)  check_mp_error(r);

    /* Setup Window */
    struct aa_rx_win * win = baxter_demo_setup_window(scenegraph);
    struct aa_gl_globals *globals = aa_rx_win_gl_globals(win);
    aa_gl_globals_set_show_visual(globals, 1);
    aa_gl_globals_set_show_collision(globals, 0);

    aa_rx_win_set_display_plan(win, scenegraph, g_n_path, g_path );

    /* Run display loop */
    aa_rx_win_display_loop(win);

    /* Cleanup */
    aa_rx_mp_destroy(mp);
    aa_rx_sg_sub_destroy(ssg);
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    aa_mem_region_local_destroy();
    SDL_Quit();

    return 0;
}
