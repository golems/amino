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

#include <stdio.h>
#include <math.h>
#include <getopt.h>
#include "amino/amino_gl.h"
#include <SDL.h>


static int SCREEN_WIDTH = 800;
static int SCREEN_HEIGHT = 600;

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_win.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_sdl.h"

#include "amino/rx/rxerr.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"

#include "jaco-model.c.h"

/* const char *allowed_collision[][2] = {{"right_w0_fixed" , "right_wrist-collision"}, */
/*                                       {NULL,NULL}}; */

static void check_mp_error( int r ) {
    if( r ) {
        char *e = aa_rx_errstr( aa_mem_region_local_get(), r );
        fprintf(stderr, "motion planning failed: `%s' (0x%x)\n", e, r);
        aa_mem_region_local_pop(e);
        exit(EXIT_FAILURE);
    }
}


const char *config_names[] = { "jaco_joint_1",
                               "jaco_joint_2",
                               "jaco_joint_3",
                               "jaco_joint_4",
                               "jaco_joint_5",
                               "jaco_joint_6",
                               /* "jaco_joint_finger_1", */
                               /* "jaco_joint_finger_2", */
                               /* "jaco_joint_finger_3", */
                               /* "jaco_joint_finger_tip_1", */
                               /* "jaco_joint_finger_tip_2", */
                               /* "jaco_joint_finger_tip_3" */
};

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    // Initialize scene graph
    struct aa_rx_sg *scenegraph = aa_rx_dl_sg__jaco(NULL);

    // add an obstacle
    {
        double q[4] = {0.0, 0.0, 0.0, 1.0};
        static const double v[3] = {-0.5, 0.2, 0.40};
        aa_tf_yangle2quat(M_PI_2,q);

        aa_rx_sg_add_frame_fixed(scenegraph, "", "obstacle", q, v);

        struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
        aa_rx_geom_opt_set_color3(opt, 1.0, 0.0, 0.0);
        aa_rx_geom_opt_set_specular3(opt, 0.3, 0.3, 0.3);
        aa_rx_geom_opt_set_visual(opt, 1);
        aa_rx_geom_opt_set_collision(opt, 1);
        struct aa_rx_geom *geom = aa_rx_geom_cylinder(opt, 1, .05);
        aa_rx_geom_attach(scenegraph, "obstacle", geom);
        aa_rx_geom_opt_destroy(opt);
    }

    aa_rx_sg_init(scenegraph); /* initialize scene graph internal structures */
    aa_rx_sg_cl_init(scenegraph); /* initialize scene graph collision objects */

    /* Center configurations */
    size_t m = aa_rx_sg_config_count(scenegraph);

    double q[m];
    for(size_t i = 0; i < m; i ++ ) {
        double min=0,max=0;
        aa_rx_sg_get_limit_pos(scenegraph,(aa_rx_config_id)i,&min,&max);
        q[i] = (max + min)/2;
    }

    double q1[m];
    AA_MEM_CPY(q1,q,m);
    for(size_t i = 0; i < 6; i ++ ) {
        q1[i] += M_PI / 2;
    }

    // Create sub-scene-graph to plan over
    struct aa_rx_sg_sub *ssg =
        aa_rx_sg_chain_create( scenegraph,
                               AA_RX_FRAME_ROOT,
                               aa_rx_sg_frame_id(scenegraph, "jaco_joint_6") );

    struct aa_rx_mp *mp = aa_rx_mp_create( ssg ); /* Create motion planning context */
    double *g_path = NULL; /* storage for plan path */
    size_t g_n_path = 0; /* storage for plan length */

    /* Start and goal states */
    aa_rx_mp_set_start(mp, m, q);

    double q_goal[6];
    for( size_t i = 0; i < 6; i++ ) {
        aa_rx_config_id j_a = aa_rx_sg_sub_config(ssg,i);
        q_goal[i] = q1[j_a];
    }
    {
        int r =  aa_rx_mp_set_goal( mp, 6, q_goal );
        if (r) {
            fprintf(stderr, "Could not set motion planning goal\n");
            check_mp_error(r);
        }
    }

    /* Execute Planner */
    aa_rx_mp_set_rrt(mp,NULL);
    aa_rx_mp_set_simplify(mp,1);
    int r = aa_rx_mp_plan( mp, 10, &g_n_path, &g_path );
    if(r)  check_mp_error(r);

    /* Setup Window */
    struct aa_rx_win * win =
        aa_rx_win_default_create ( "Amino: Jaco OMPL Demo", SCREEN_WIDTH, SCREEN_HEIGHT );
    /* struct aa_gl_globals *globals = aa_rx_win_gl_globals(win); */
    /* aa_gl_globals_set_show_visual(globals, 1); */
    /* aa_gl_globals_set_show_collision(globals, 0); */

    aa_rx_win_set_display_plan(win, scenegraph, g_n_path, g_path );

    /* Run display loop */
    aa_rx_win_run();

    /* Cleanup */
    aa_rx_mp_destroy(mp);
    aa_rx_sg_sub_destroy(ssg);
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    aa_mem_region_local_destroy();
    SDL_Quit();

    return 0;
}
