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


#define GL_GLEXT_PROTOTYPES

#include <error.h>
#include <stdio.h>
#include <math.h>
#include <getopt.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>



#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_win.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_sdl.h"

#include "scene/scene.c.h"

#include <dlfcn.h>

static int SCREEN_WIDTH = 800;
static int SCREEN_HEIGHT = 600;

int main(int argc, char *argv[])
{
    int visual = 1;
    int collision = 0;

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
                     "\n" );
                exit(EXIT_SUCCESS);
                break;
            }

        }
    }

    /* Initialize scene graph */
    struct aa_rx_sg *scenegraph = aa_rx_dl_sg__scenegraph( NULL);

    assert(scenegraph);
    aa_rx_sg_init(scenegraph); /* initialize scene graph internal structures */

    /* Center configurations */
    size_t m = aa_rx_sg_config_count(scenegraph);
    double q[m];
    for(size_t i = 0; i < m; i ++ ) {
        double min=0,max=0;
        aa_rx_sg_get_limit_pos(scenegraph,(aa_rx_config_id)i,&min,&max);
        q[i] = (max + min)/2;
    }

    /* setup window */
    struct aa_rx_win * win =
        aa_rx_win_default_create ( "Amino: AARX-View", SCREEN_WIDTH, SCREEN_HEIGHT );
    aa_rx_win_set_sg(win, scenegraph); /* Set the scenegraph for the window */
    aa_rx_win_set_config(win, m, q);

    struct aa_gl_globals *globals = aa_rx_win_gl_globals(win);
    aa_gl_globals_set_show_visual(globals, visual);
    aa_gl_globals_set_show_collision(globals, collision);

    /* start display */
    aa_rx_win_start(win);

    /* Cleanup */
    aa_rx_win_join(win);
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
