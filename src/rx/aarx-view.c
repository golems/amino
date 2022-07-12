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



#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_win.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_plugin.h"

#include <dlfcn.h>

static int SCREEN_WIDTH = 800;
static int SCREEN_HEIGHT = 600;

int main(int argc, char *argv[])
{
    const char *name="scenegraph";
    const char *plugin=NULL;
    const char *end_effector=NULL;
    int visual = 1;
    int collision = 0;

    int print_config = 0;
    int print_frame = 0;

    /* Parse Options */
    {
        int c;
        opterr = 0;

        while( (c = getopt( argc, argv, "n:e:?cQF")) != -1 ) {
            switch(c) {
            case 'n':
                name = optarg;
                break;
            case 'c':
                visual = 0;
                collision = 1;
                break;
            case 'e':
                end_effector = optarg;
                break;
            case 'Q':
                print_config = 1;
                break;
            case 'F':
                print_frame = 1;
                break;
            case '?':
                puts("Usage: aarx-view [OPTIONS] PLUGIN_NAME\n"
                     "Viewer for Amino scene graphs"
                     "\n"
                     "Options:\n"
                     "  -n NAME         scene graph name (default: scenegraph)\n"
                     "  -e NAME         name of an end-effector to control\n"
                     "  -Q              Print configuration names and exit\n"
                     "  -F              Print frame names and exit\n"
                     "  -c              view collision geometry\n"
                     "\n"
                     "\n"
                     "Report bugs to " PACKAGE_BUGREPORT "\n" );
                exit(EXIT_SUCCESS);
                break;
            default:
                plugin=optarg;
            }

        }

        while( optind < argc ) {
            plugin = argv[optind++];
        }
    }

    if( NULL == plugin ) {
        fprintf(stderr,
                "ERROR: scene graph plugin not specified.  "
                "See `aarx-view -?` for options\n");
        exit(EXIT_FAILURE);
    }

    /* Initialize scene graph */
    struct aa_rx_sg *scenegraph = aa_rx_dl_sg(plugin, name, NULL);
    assert(scenegraph);
    aa_rx_sg_init(scenegraph); /* initialize scene graph internal structures */

    /* print things */
    if( print_config || print_frame ) {

        if( print_config ) {
            for (aa_rx_config_id i = 0; i < (int)aa_rx_sg_config_count(scenegraph); i++) {
                printf("config[%ld]: %s\n", i, aa_rx_sg_config_name(scenegraph, i));
            }
        }
        if( print_frame ) {
            for (aa_rx_frame_id i = 0; i < (int)aa_rx_sg_frame_count(scenegraph); i++) {
                printf("frame[%ld]: %s\n", i, aa_rx_sg_frame_name(scenegraph, i));
            }
        }
        exit(EXIT_SUCCESS);
    }

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

    /* Set the scenegraph for the window */
    struct aa_rx_sg_sub *sub = NULL;
    if( end_effector ) {

        aa_rx_frame_id id = aa_rx_sg_frame_id(scenegraph, end_effector);
        if( AA_RX_FRAME_NONE == id ) {
            fprintf(stderr, "Could not find frame `%s'\n", end_effector);
            exit(EXIT_FAILURE);
        }
        sub = aa_rx_sg_chain_create( scenegraph, AA_RX_FRAME_ROOT, id);
        aa_rx_win_set_sg_sub(win, sub);
    } else {
        aa_rx_win_set_sg(win, scenegraph);
    }
    aa_rx_win_set_config(win, m, q);

    struct aa_gl_globals *globals = aa_rx_win_gl_globals(win);
    aa_gl_globals_set_show_visual(globals, visual);
    aa_gl_globals_set_show_collision(globals, collision);

    /* start display */
    aa_rx_win_run();

    /* Cleanup */
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
