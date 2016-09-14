/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015-2016, Rice University
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

#include "config.h"

#define GL_GLEXT_PROTOTYPES

#include <stdio.h>
#include <math.h>
#include <getopt.h>

#include <SDL.h>
#include <SDL_opengl.h>
#include <pthread.h>
#include <unistd.h>



#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_win.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_sdl.h"

#include <dlfcn.h>

static int SCREEN_WIDTH = 800;
static int SCREEN_HEIGHT = 600;

struct aa_rx_sg * NAME(struct aa_rx_sg *sg);

static int visual = 1;
static int collision = 0;

static struct aa_rx_sg *
create_sg(void)
{
    // Initialize scene graph
    struct aa_rx_sg * scenegraph = aa_rx_sg_create();

    // Add a Box
    {
        static const double q[4] = {0.0, 0.0, 0.0, 1.0};
        static const double v[3] = {0.0, 0.0, 0.05};
        aa_rx_sg_add_frame_fixed(scenegraph, "", "boxframe", q, v);

        struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
        aa_rx_geom_opt_set_color3(opt, 1.0, 0.0, 0.0);
        aa_rx_geom_opt_set_specular3(opt, 0.3, 0.3, 0.3);
        aa_rx_geom_opt_set_visual(opt, 1);
        aa_rx_geom_opt_set_collision(opt, 1);
        static const double dimension[3] = {0.1, 0.1, 0.1};
        struct aa_rx_geom *geom = aa_rx_geom_box(opt, dimension);
        aa_rx_geom_attach(scenegraph, "boxframe", geom);
        aa_rx_geom_opt_destroy(opt);
    }

    // Add a Mesh
    {
        double v[3] = {0,0,-1e-3};
        aa_rx_sg_add_frame_fixed(scenegraph, "", "mesh", aa_tf_quat_ident, v);
        struct aa_rx_mesh * mesh = aa_rx_mesh_create();
#define F .5
        static const float vertices[4*3] = { -F,-F,0,
                                             -F,F,0,
                                             F,-F,0,
                                             F,F,0 };
        static const float normals[4*3] = { 0,0,1,
                                            0,0,1,
                                            0,0,1,
                                            0,0,1 };
        static const unsigned indices [3*2] = {0, 1, 2,
                                               3,1,2};
        aa_rx_mesh_set_vertices(mesh, 4, vertices, 0);
        aa_rx_mesh_set_normals(mesh, 4, normals, 0);
        aa_rx_mesh_set_indices(mesh, 2, indices, 0);

        struct aa_rx_geom * geom;
        struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
        double d = .5;
        aa_rx_geom_opt_set_color3(opt, d*.91, d*.96, d*.88);

        aa_rx_geom_opt_set_alpha(opt, 1.0);
        aa_rx_geom_opt_set_visual(opt, 1);
        aa_rx_geom_opt_set_collision(opt, 0);
        aa_rx_geom_opt_set_no_shadow(opt, 0);
        aa_rx_mesh_set_texture(mesh, opt);
        (geom) = (aa_rx_geom_mesh(opt, mesh));
        aa_rx_geom_attach(scenegraph, "mesh", geom);
        aa_rx_geom_opt_destroy(opt);
    }


    assert(scenegraph);
    aa_rx_sg_init(scenegraph); /* initialize scene graph internal structures */

    return scenegraph;
}

void *thread_fun( void *arg )
{
    struct aa_rx_win * win = (struct aa_rx_win * )arg;

    struct aa_rx_sg *sg = NULL, *sg_old = NULL;

    while(aa_rx_win_is_running(win)) {
        sg_old = sg;
        sg = create_sg();


        /* Center configurations */
        size_t m = aa_rx_sg_config_count(sg);
        double q[m];
        for(size_t i = 0; i < m; i ++ ) {
            double min=0,max=0;
            aa_rx_sg_get_limit_pos(sg,(aa_rx_config_id)i,&min,&max);
            q[i] = (max + min)/2;
        }

        aa_rx_win_set_sg(win, sg); /* Set the scenegraph for the window */
        aa_rx_win_set_config(win, m, q);

        if( sg_old ) aa_rx_sg_destroy(sg_old);

        struct aa_gl_globals *globals = aa_rx_win_gl_globals(win);

        aa_rx_win_lock(win);
        aa_gl_globals_set_show_visual(globals, visual);
        aa_gl_globals_set_show_collision(globals, collision);
        aa_rx_win_unlock(win);

        sleep(1);
    }

    return NULL;
}

int main(int argc, char *argv[])
{

    /* Parse Options */
    {
        int c;
        opterr = 0;

        while( (c = getopt( argc, argv, "?c")) != -1 ) {
            switch(c) {
            case 'c':
                visual = 0;
                collision = 1;
                break;
            case '?':
                puts("Usage: COMMAND [OPTIONS] \n"
                     "Viewer for Amino scene graphs"
                     "\n"
                     "Options:\n"
                     "  -c              view collision geometry\n"
                     "\n"
                     "\n"
                     "Report bugs to " PACKAGE_BUGREPORT "\n" );
                exit(EXIT_SUCCESS);
                break;
            }

        }
    }


    /* setup window */
    struct aa_rx_win * win =
        aa_rx_win_default_create ( "Amino: AARX-View", SCREEN_WIDTH, SCREEN_HEIGHT );

    pthread_t thread;
    if( pthread_create( &thread, NULL, thread_fun, win ) ) {
        fprintf(stderr, "could not create thread\n");
        abort();
        exit(EXIT_FAILURE);
    }

    /* start display */
    aa_rx_win_run();

    /* Wait for other thread */
    if( pthread_join(thread, NULL) ) {
        fprintf(stderr, "could not join thread\n");
        abort();
        exit(EXIT_FAILURE);

    }


    /* Cleanup */
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
